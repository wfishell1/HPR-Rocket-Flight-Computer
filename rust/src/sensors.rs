use crate::orient;
use drivers::bus::DeviceBus;
use drivers::h3lis331dl::H3lis331dlDriver;
use drivers::lis3mdl::Lis3mdlDriver;
use drivers::lsm6dsox::Lsm6dsoxDriver;
use drivers::ms5611::Ms5611Driver;
use drivers::types::Raw3;
use std::cell::RefCell;

pub(crate) enum Driver<'a, I, P, H, M> {
    Imu(&'a RefCell<Lsm6dsoxDriver<I>>),
    Barometer(Ms5611Driver<P>),
    HighG(H3lis331dlDriver<H>),
    Magnetometer(Lis3mdlDriver<M>),
}

pub enum SensorSample {
    Vector(Raw3),
    Barometer(i32),
}

pub(crate) enum SensorKind {
    Acceleration,
    Gyroscope,
    Barometer,
    HighG,
    Magnetometer,
}

pub trait Sensor {
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

pub struct AG_Sensor<'a, I, P, H, M> {
    driver: Driver<'a, I, P, H, M>,
    pub sample: Option<SensorSample>,
    pub last_sample_us: u32,
    interval: u32,
    pub(crate) kind: SensorKind,
    pub(crate) bias: Raw3,
    pub(crate) axes: [usize; 3],
    pub(crate) signs: [i32; 3],
    pub oriented: Option<[i32; 3]>,
}

impl<'a, I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>
    AG_Sensor<'a, I, P, H, M>
{
    pub(crate) fn new(driver: Driver<'a, I, P, H, M>, kind: SensorKind, bias: Raw3, axes: [usize; 3], signs: [i32; 3], now_us: u32) -> Self {
        let interval = match &driver {
            Driver::Imu(driver) => driver.borrow().sample_period_us(),
            Driver::Barometer(driver) => driver.sample_period_us(),
            Driver::HighG(driver) => driver.sample_period_us(),
            Driver::Magnetometer(driver) => driver.sample_period_us(),
        };
        Self {
            driver, kind, bias, axes, signs, interval,
            last_sample_us: now_us,
            sample: None,
            oriented: None,
        }
    }
}

impl<'a, I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>
    Sensor for AG_Sensor<'a, I, P, H, M>
{
    fn is_due(&self, now_us: u32) -> bool {
        now_us.wrapping_sub(self.last_sample_us) >= self.interval
    }

    fn get(&mut self) {
        self.sample = match &mut self.driver {
            Driver::Imu(driver) => driver.borrow_mut().read().ok().and_then(|sample| {
                match self.kind {
                    SensorKind::Acceleration => Some(SensorSample::Vector(sample.accel())),
                    SensorKind::Gyroscope => Some(SensorSample::Vector(sample.gyro())),
                    _ => None,
                }
            }),
            Driver::Barometer(driver) => driver.poll().ok().flatten().map(SensorSample::Barometer),
            Driver::HighG(driver) => driver.read().ok().map(SensorSample::Vector),
            Driver::Magnetometer(driver) => driver.read().ok().map(SensorSample::Vector),
        };
    }

    fn orient(&mut self) {
        self.oriented = None;
        if let Some(SensorSample::Vector(raw)) = &self.sample {
            self.oriented = Some(orient::orient(*raw, self.bias, self.axes, self.signs));
        }
    }

    fn advance_schedule(&mut self, now_us: u32) {
        self.last_sample_us = now_us;
    }
}

pub fn setup_sensors<'a, I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    imu: &'a RefCell<Lsm6dsoxDriver<I>>,
    barometer_driver: Ms5611Driver<P>,
    high_g_driver: H3lis331dlDriver<H>,
    magnetometer_driver: Lis3mdlDriver<M>,
    accel_calibration: (Raw3, [usize; 3], [i32; 3]),
    gyro_calibration: (Raw3, [usize; 3], [i32; 3]),
    high_g_calibration: (Raw3, [usize; 3], [i32; 3]),
    magnetometer_calibration: (Raw3, [usize; 3], [i32; 3]),
    now_us: u32,
) -> [AG_Sensor<'a, I, P, H, M>; 5] {
    let accelerometer = AG_Sensor::new(Driver::Imu(imu), SensorKind::Acceleration, accel_calibration.0, accel_calibration.1, accel_calibration.2, now_us);
    let gyroscope = AG_Sensor::new(Driver::Imu(imu), SensorKind::Gyroscope, gyro_calibration.0, gyro_calibration.1, gyro_calibration.2, now_us);
    let barometer = AG_Sensor::new(Driver::Barometer(barometer_driver), SensorKind::Barometer, Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1], now_us);
    let high_g = AG_Sensor::new(Driver::HighG(high_g_driver), SensorKind::HighG, high_g_calibration.0, high_g_calibration.1, high_g_calibration.2, now_us);
    let magnetometer = AG_Sensor::new(Driver::Magnetometer(magnetometer_driver), SensorKind::Magnetometer, magnetometer_calibration.0, magnetometer_calibration.1, magnetometer_calibration.2, now_us);
    [accelerometer, gyroscope, barometer, high_g, magnetometer]
}

pub fn run_polling(sensors: &mut [impl Sensor], mut clock: impl FnMut() -> u32) {
    for sensor in sensors {
        sensor.poll(clock());
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use drivers::bus::I2cBus;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    pub(crate) type MockSensor<'a> = AG_Sensor<'a, I2cBus<Mock>, I2cBus<Mock>, I2cBus<Mock>, I2cBus<Mock>>;

    pub(crate) fn imu_expectations() -> Vec<Transaction> {
        let mut tx = vec![Transaction::write_read(0x6A, vec![0x0F], vec![0x6C])];
        for (reg, value) in [(0x12, 0x44), (0x11, 0x8C), (0x10, 0x84)] {
            tx.push(Transaction::write(0x6A, vec![reg, value]));
            tx.push(Transaction::write_read(0x6A, vec![reg], vec![value]));
        }
        tx
    }

    #[test]
    fn polling_obeys_period_wrap_and_retains_oriented_values() {
        let mut tx = imu_expectations();
        tx.push(Transaction::write_read(0x6A, vec![0x22], vec![0, 0, 0, 0, 0, 0, 1, 0, 2, 0, 3, 0]));
        tx.push(Transaction::write_read(0x6A, vec![0x22], vec![0; 12])
            .with_error(embedded_hal::i2c::ErrorKind::Other));
        let mut mock = Mock::new(&tx);
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let start = u32::MAX - 300;
        let mut sensors: [MockSensor<'_>; 1] = [AG_Sensor::new(
            Driver::Imu(&imu), SensorKind::Acceleration, Raw3::new(1, 1, 1),
            [2, 1, 0], [-1, 1, 1], start,
        )];
        run_polling(&mut sensors, || start.wrapping_add(601));
        assert!(sensors[0].sample.is_none());
        run_polling(&mut sensors, || start.wrapping_add(602));
        assert!(matches!(sensors[0].sample.take(), Some(SensorSample::Vector(raw)) if raw == Raw3::new(1, 2, 3)));
        assert_eq!(sensors[0].oriented, Some([-2, 1, 0]));
        run_polling(&mut sensors, || start.wrapping_add(1203));
        assert!(sensors[0].sample.is_none());
        assert_eq!(sensors[0].oriented, Some([-2, 1, 0]));
        run_polling(&mut sensors, || start.wrapping_add(1204));
        assert!(sensors[0].sample.is_none());
        assert!(sensors[0].oriented.is_none());
        assert_eq!(sensors[0].last_sample_us, start.wrapping_add(1204));
        mock.done();
    }
}
