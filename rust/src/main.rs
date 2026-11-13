
use drivers::bus::DeviceBus;
use drivers::lsm6dsox::{self, Lsm6dsoxDriver};
use drivers::ms5611::{self, Ms5611Driver};
use drivers::types::Raw3;
use std::cell::RefCell;
use hpr_rust::acceleration::AccelerationState;
use hpr_rust::event_logic::{Flight, Phase};
use hpr_rust::flight_clock::FlightClock;
use hpr_rust::settings::{SensorInputs, Settings};
mod orient;
enum Driver<'a, I, P> {
    Imu(&'a RefCell<Lsm6dsoxDriver<I>>),
    Barometer(Ms5611Driver<P>),
}

enum SensorSample {
    Imu(lsm6dsox::Sample),
    Barometer(i32),
}
enum SensorKind {
    Acceleration,
    Gyroscope,
    Barometer,
}

trait Sensor {
    fn is_due(&self, now_us: u32) -> bool;
    fn get(&mut self);
    fn orient(&mut self);
    fn advance_schedule(&mut self, now_us: u32);
    fn poll(&mut self, now_us: u32) {
        if self.is_due(now_us) {
            self.get();
            self.orient();
            self.advance_schedule(now_us);
        }
    }

}
struct AG_Sensor<I, P> {
    driver: Driver<I, P>,
    sample: Option<SensorSample>,
    last_sample_us: u32,
    interval: u32,
    kind: SensorKind,
    bias: Raw3,
    axes: [usize; 3],
    signs: [i32; 3],
    oriented: Option<[i32;3]>,
}

impl<'a, I: DeviceBus, P: DeviceBus> AG_Sensor<I, P> {
    fn new(driver: Driver<'a, I, P>, kind: SensorKind, bias: Raw3, axes: [usize;3], signs:[i32; 3], now_us: u32) -> Self {
        let interval = match &driver {
            Driver::Imu(driver) => driver.borrow().sample_period_us(),
            Driver::Barometer(driver) => driver.sample_period_us(),
        };
        Self {
            driver,
            kind,
            bias,
            axes,
            signs,
            interval,
            last_sample_us: now_us,
            sample: None,
            oriented: None
        }

    }
}


impl<I: DeviceBus, P: DeviceBus> Sensor for AG_Sensor<I, P> {
    fn is_due(&self, now_us: u32) -> bool {
        if now_us.wrapping_sub(self.last_sample_us) >= self.interval {
            return true;
        }
        return false
    }
    fn get(&mut self) {
        self.sample = None;

        match &mut self.driver {
            Driver::Imu(driver) => {
                match driver.borrow_mut().read() {
                    Ok(sample) => {
                        self.sample = Some(SensorSample::Imu(sample));
                    }
                    Err(_error) => {
                        //Nothing here yet, tbd later.
                    }
                }
            }
            Driver::Barometer(driver) => {
                match driver.poll() {
                    Ok(Some(pressure)) => {
                        self.sample = Some(SensorSample::Barometer(pressure));
                    }
                    Ok(None) => {
                        //Nothing here yet, tbd later
                    }
                    Err(_error) => {
                        //Nothing here yet, tbd later
                    }
                }
            }
        }       
    }


    fn orient(&mut self) {
        self.oriented = None;
        //extracts data
        let raw = match (&self.sample, &self.kind) {
            (Some(SensorSample::Imu(sample)), SensorKind::Acceleration) => sample.accel(),
            (Some(SensorSample::Imu(sample)), SensorKind::Gyroscope) => sample.gyro(),
            _ => return,
        };

        self.oriented = Some(orient::orient(raw, self.bias, self.axes, self.signs));
    }
    fn advance_schedule(&mut self, now_us: u32) {
        self.last_sample_us = now_us;
    } 
}


fn setup_sensors<I: DeviceBus, P: DeviceBus>(imu: &RefCell<Lsm6dsoxDriver<I>>, barometer_driver: Ms5611Driver<P>, accel_calibration: (Raw3, [usize; 3], [i32; 3]), gyro_calibration: (Raw3, [usize; 3], [i32; 3]), now_us: u32) -> [AG_Sensor<I, P>; 3] {
    let accelerometer = AG_Sensor::new(Driver::Imu(imu), SensorKind::Acceleration, accel_calibration.0, accel_calibration.1, accel_calibration.2, now_us);
    let gyroscope = AG_Sensor::new(Driver::Imu(imu), SensorKind::Gyroscope, gyro_calibration.0, gyro_calibration.1, gyro_calibration.2, now_us);
    let barometer = AG_Sensor::new(Driver::Barometer(barometer_driver), SensorKind::Barometer, Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1], now_us);

    [accelerometer, gyroscope, barometer]
}

fn run_polling(sensors: &mut [impl Sensor], mut clock: impl FnMut() -> u32) {
    for sensor in sensors {
        sensor.poll(clock());
    }
}

fn main() {
    let mut sensors: [AG_Sensor<_, _>; 3] = todo!("Initialize sensors with setup_sensors");
    let mut clock = || -> u32 { todo!("Read the microsecond clock") };
    let mut flight = Flight::new(Settings::default());
    let mut flight_clock = FlightClock::new();
    let mut acceleration = AccelerationState::new();
    let mut inputs = SensorInputs::default();

    run_polling(&mut sensors, &mut clock);
    let now_us = clock();

    let Some(oriented) = sensors[0].oriented else {
        return;
    };
    let Ok(accel_z) = i16::try_from(oriented[2]) else {
        return;
    };
    let accel_scale = match &sensors[0].driver {
        Driver::Imu(driver) => driver.borrow().accel_scale(),
        Driver::Barometer(_) => return,
    };

    inputs.accel_z = accel_z;
    flight.update_current_info(&inputs);

    if flight.flight_phase == Phase::Pad {
        flight.liftoff_check();
        if flight.flight_phase == Phase::BoostUnconfirmed {
            flight_clock.start(now_us);
            acceleration = AccelerationState::new();
        }
    }

    if matches!(flight.flight_phase, Phase::Pad | Phase::Landed | Phase::TimedOut) {
        return;
    }

    if let Some(dt_us) = flight_clock.advance(now_us) {
        let accel_now = (oriented[2] - i32::from(flight.settings.counts_per_g)) as f32
            * accel_scale
            * 9.80665;

        if matches!(flight.flight_phase, Phase::Drogue | Phase::Main) {
            acceleration.accel_now = accel_now;
        } else {
            acceleration.update(accel_now, dt_us);
        }

        inputs.flight_time_us = flight_clock.flight_time_us;
        inputs.accel_now = acceleration.accel_now;
        inputs.accel_vel = acceleration.accel_vel;
        inputs.accel_alt = acceleration.accel_alt;
        flight.update_current_info(&inputs);
    }
}
