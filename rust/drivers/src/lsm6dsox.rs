use embedded_hal::i2c::I2c;
use crate::types::Raw3;

// chip addresses
const WHO_AM_I_ADDR: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const OUTX_L_G: u8 = 0x22;

// who am i expected value
const WHO_AM_I_VAL: u8 = 0x6C;

// chip constants
const ACCEL_SCALE: f32 = 0.000488;      // g per count
const ACCEL_SATURATION: i16 = 32112;    // 98% of full scale
const GYRO_SCALE: f32 = 0.07;           // °/s per count
const GYRO_SATURATION: i16 = 32112;     // 98% of full scale
const SAMPLE_PERIOD_US: u32 = 602;      // 1660 Hz


pub struct Lsm6dsoxDriver<I> {
    i2c: I,
    addr: u8,
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    RegisterMismatch { expected: u8, found: u8 },
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sample {
    gyro: Raw3,
    accel: Raw3,
}

impl Sample {
    pub fn new(gyro: Raw3, accel: Raw3) -> Self {
        Self {
            gyro,
            accel,
        }
    }
    pub fn gyro(&self) -> Raw3 { self.gyro }
    pub fn accel(&self) -> Raw3 { self.accel }
}

impl<I: I2c> Lsm6dsoxDriver<I> {
    pub fn new(i2c: I, addr: u8) -> Result<Lsm6dsoxDriver<I>, Error<I::Error>> {
        
        let mut driver = Self::new_not_setup(i2c, addr);

        // ensure correct chip with who am i
        driver.confirm_register(WHO_AM_I_ADDR, WHO_AM_I_VAL)?;

        // set auto-increment on, block data update on with 0x44 and read it back
        driver.i2c.write(driver.addr, &[CTRL3_C, 0x44])?;
        driver.confirm_register(CTRL3_C, 0x44)?;

        // gyroscope set 2000dps Range, 1.66kHz ODR = 0b10001100 and read it back
        driver.i2c.write(driver.addr, &[CTRL2_G, 0b10001100])?;
        driver.confirm_register(CTRL2_G, 0b10001100)?;

        // accelerometer set 16G Range, 1.66kHz ODR, 400Hz BW = 0b10000100 and read it back
        driver.i2c.write(driver.addr, &[CTRL1_XL, 0b10000100])?;
        driver.confirm_register(CTRL1_XL, 0b10000100)?;

        Ok(driver)
    }

    fn new_not_setup(i2c: I, addr: u8) -> Self { Self { i2c, addr } }

    fn confirm_register(&mut self, reg: u8, val: u8) -> Result<(), Error<I::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.addr, &[reg], &mut buf)?;
        
        if buf[0] != val {
            return Err(Error::RegisterMismatch { expected: val, found: buf[0] });
        }

        Ok(())
    }

    pub fn read(&mut self) -> Result<Sample, Error<I::Error>> {
        let mut buf = [0u8; 12];
        self.i2c.write_read(self.addr, &[OUTX_L_G], &mut buf)?;
        Ok(Self::parse(&buf))
    }

    fn parse(buf: &[u8; 12]) -> Sample {
        let word = |i: usize| i16::from_le_bytes([buf[i], buf[i+1]]);

        Sample::new(Raw3::new(word(0), word(2), word(4)), Raw3::new(word(6), word(8), word(10)))
    }

    pub fn accel_scale(&self) -> f32 { ACCEL_SCALE }

    pub fn accel_saturation(&self) -> i16 { ACCEL_SATURATION }

    pub fn gyro_scale(&self) -> f32 { GYRO_SCALE }

    pub fn gyro_saturation(&self) -> i16 { GYRO_SATURATION }

    pub fn sample_period_us(&self) -> u32 { SAMPLE_PERIOD_US }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    const ADDR: u8 = 0x6A;

    fn init_expectations() -> Vec<Transaction> {
        vec![
            Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![WHO_AM_I_VAL]),
            Transaction::write(ADDR, vec![CTRL3_C, 0x44]),
            Transaction::write_read(ADDR, vec![CTRL3_C], vec![0x44]),
            Transaction::write(ADDR, vec![CTRL2_G, 0b10001100]),
            Transaction::write_read(ADDR, vec![CTRL2_G], vec![0b10001100]),
            Transaction::write(ADDR, vec![CTRL1_XL, 0b10000100]),
            Transaction::write_read(ADDR, vec![CTRL1_XL], vec![0b10000100]),
        ]
    }

    #[test]
    fn new_succeeds_with_correct_chip() {
        let mut mock = Mock::new(&init_expectations());

        let result = Lsm6dsoxDriver::new(mock.clone(), ADDR);

        assert!(result.is_ok());
        mock.done();
    }

    #[test]
    fn new_fails_on_wrong_chip() {
        let expectations = [
            Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![0x69]),
        ];
        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(mock.clone(), ADDR);

        assert!(matches!(
            result,
            Err(Error::RegisterMismatch { expected: 0x6C, found: 0x69 })
        ));
        mock.done();
    }

    #[test]
    fn new_fails_when_setting_does_not_stick() {
        let expectations = [
            Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![WHO_AM_I_VAL]),
            Transaction::write(ADDR, vec![CTRL3_C, 0x44]),
            Transaction::write_read(ADDR, vec![CTRL3_C], vec![0x00]), // chip ignored the write
        ];
        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(mock.clone(), ADDR);

        assert!(matches!(
            result,
            Err(Error::RegisterMismatch { expected: 0x44, found: 0x00 })
        ));
        mock.done();
    }

    #[test]
    fn new_reports_bus_error() {
        let expectations = [
            Transaction::write_read(ADDR, vec![WHO_AM_I_ADDR], vec![0x00])
                .with_error(ErrorKind::Other),
        ];
        let mut mock = Mock::new(&expectations);

        let result = Lsm6dsoxDriver::new(mock.clone(), ADDR);

        assert!(matches!(result, Err(Error::Bus(ErrorKind::Other))));
        mock.done();
    }

    #[test]
    fn parse_puts_bytes_in_right_places() {
        //          gyro x   gyro y   gyro z   accel x  accel y  accel z
        let buf = [1, 0,    2, 0,    3, 0,    4, 0,    5, 0,    6, 0];

        let sample = Lsm6dsoxDriver::<Mock>::parse(&buf);

        assert_eq!(sample.gyro(), Raw3::new(1, 2, 3));
        assert_eq!(sample.accel(), Raw3::new(4, 5, 6));
    }

    #[test]
    fn parse_handles_full_range() {
        let buf = [0xFF, 0x7F,  0x00, 0x80,  0x01, 0x08,  0, 0,  0, 0,  0, 0];

        let sample = Lsm6dsoxDriver::<Mock>::parse(&buf);

        assert_eq!(sample.gyro(), Raw3::new(32767, -32768, 2049));
    }

    #[test]
    fn read_returns_parsed_sample() {
        let mut expectations = init_expectations();
        expectations.push(Transaction::write_read(
            ADDR,
            vec![OUTX_L_G],
            vec![1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6, 0],
        ));
        let mut mock = Mock::new(&expectations);

        let mut driver = Lsm6dsoxDriver::new(mock.clone(), ADDR).unwrap();
        let sample = driver.read().unwrap();

        assert_eq!(sample.gyro(), Raw3::new(1, 2, 3));
        assert_eq!(sample.accel(), Raw3::new(4, 5, 6));
        mock.done();
    }
}