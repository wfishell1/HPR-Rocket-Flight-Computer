
use drivers::bus::DeviceBus;
use drivers::lsm6dsox::{self, Lsm6dsoxDriver};
use drivers::ms5611::{self, Ms5611Driver};
use drivers::types::Raw3;
use std::cell::RefCell;
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
    //setup_sensors
    //run_polling
}
