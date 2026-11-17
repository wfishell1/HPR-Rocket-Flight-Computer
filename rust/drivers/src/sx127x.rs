use crate::bus::SpiMasks;
use crate::register::{self, RegisterController};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

// more complex driver, reg keeps track of addresses
mod reg {
    pub const FIFO: u8 = 0x00;
    pub const OP_MODE: u8 = 0x01;
    pub const FRF_MSB: u8 = 0x06;
    pub const FRF_MID: u8 = 0x07;
    pub const FRF_LSB: u8 = 0x08;
    pub const PA_CONFIG: u8 = 0x09;
    pub const FIFO_PTR: u8 = 0x0D;
    pub const FIFO_TX_BASE: u8 = 0x0E;
    pub const FIFO_RX_BASE: u8 = 0x0F;
    pub const IRQ_MASK: u8 = 0x11;
    pub const IRQ_FLAGS: u8 = 0x12;
    pub const MODEM_CONFIG1: u8 = 0x1D;
    pub const MODEM_CONFIG2: u8 = 0x1E;
    pub const PREAMBLE_MSB: u8 = 0x20;
    pub const PREAMBLE_LSB: u8 = 0x21;
    pub const PAYLOAD_LEN: u8 = 0x22;
    pub const MODEM_CONFIG3: u8 = 0x26;
    pub const DIO_MAPPING1: u8 = 0x40;
    pub const VERSION: u8 = 0x42;
    pub const PA_DAC: u8 = 0x4D;
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Mode { Sleep = 0x00, Standby = 0x01, Tx = 0x03 }

const LORA: u8 = 0x80;        // or'd into every mode write
const VERSION_VAL: u8 = 0x12;
const TX_DONE: u8 = 0x08;
const CLEAR_FLAGS: u8 = 0xFF; // writing 1s clears irq flags
const FIFO_START: u8 = 0x00;  // start of chip's buffer

const PA_DAC_ON: u8 = 0x07;   // high power mode, adds 3 dbm
const PA_DAC_OFF: u8 = 0x04;
const PA_BOOST: u8 = 0x80;    // select pa_boost output pin

// lora bit can only be set asleep, so sleep -> lora sleep -> configure -> standby
const CONFIG: &[(u8, u8)] = &[
    (reg::OP_MODE, Mode::Sleep as u8),
    (reg::OP_MODE, LORA | Mode::Sleep as u8),
    (reg::MODEM_CONFIG1, 0b01110010), // 125kHz bw, 4/5 cr, explicit header
    (reg::MODEM_CONFIG2, 0b01110100), // sf7, crc on
    (reg::MODEM_CONFIG3, 0x04),       // auto gain
    (reg::PREAMBLE_MSB, 0x00),        // preamble length 8
    (reg::PREAMBLE_LSB, 0x08),
    (reg::FIFO_TX_BASE, FIFO_START),
    (reg::FIFO_RX_BASE, FIFO_START),
    (reg::OP_MODE, LORA | Mode::Standby as u8),
    (reg::DIO_MAPPING1, 0x40),        // dio0 = tx done
    (reg::IRQ_MASK, 0x00),            // no irqs masked
];

pub const MASKS: SpiMasks = SpiMasks { read: 0x00, write: 0x80, inc: 0x00 };

// only new touches the reset pin, dont make everything carry extra error type
#[derive(Debug)]
pub enum InitError<E, PE> {
    Register(register::Error<E>),
    Reset(PE),
}

impl<E, PE> From<register::Error<E>> for InitError<E, PE> {
    fn from(e: register::Error<E>) -> Self { InitError::Register(e) }
}

impl<E, PE> From<E> for InitError<E, PE> {
    fn from(e: E) -> Self { InitError::Register(register::Error::Bus(e)) }
}

#[derive(Debug)]
pub enum Error<E> {
    Register(register::Error<E>),
    PacketTooLong { len: usize }, // payload length register is 1 byte
    Busy,                         // still sending last packet
}

impl<E> From<register::Error<E>> for Error<E> {
    fn from(e: register::Error<E>) -> Self { Error::Register(e) }
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self { Error::Register(register::Error::Bus(e)) }
}

pub struct Sx127xDriver<R> {
    reg_ctrl: R,
    transmitting: bool,
}

impl<R: RegisterController> Sx127xDriver<R> {
    pub fn new<P: OutputPin>(mut reg_ctrl: R, reset: &mut P, delay: &mut impl DelayNs) -> Result<Self, InitError<R::Error, P::Error>> {
        // pulse reset low for 100us
        reset.set_high().map_err(InitError::Reset)?;
        delay.delay_ms(10);
        reset.set_low().map_err(InitError::Reset)?;
        delay.delay_us(100);
        reset.set_high().map_err(InitError::Reset)?;
        delay.delay_ms(10);

        reg_ctrl.check_id(reg::VERSION, VERSION_VAL)?;

        reg_ctrl.configure(CONFIG)?;

        // not in config, flags read back 0 after clearing so verify would fail
        reg_ctrl.write_reg(reg::IRQ_FLAGS, CLEAR_FLAGS)?;

        Ok(Self { reg_ctrl, transmitting: false })
    }

    pub fn set_frequency(&mut self, freq_hz: u32) -> Result<(), Error<R::Error>> {
        if self.transmitting {
            return Err(Error::Busy);
        }

        self.set_mode(Mode::Standby)?;

        // frf = freq * 2^19 / 32MHz crystal
        let frf = (freq_hz as u64 * (1 << 19) / 32_000_000) as u32;
        let [_, msb, mid, lsb] = frf.to_be_bytes();

        self.reg_ctrl.write_verified(reg::FRF_MSB, msb)?;
        self.reg_ctrl.write_verified(reg::FRF_MID, mid)?;
        self.reg_ctrl.write_verified(reg::FRF_LSB, lsb)?;

        Ok(())
    }

    // range is 2-20 dbm, above 17 needs the high power dac which adds 3 on its own, need to subtract 3 in this case
    pub fn set_power(&mut self, power_dbm: i8) -> Result<(), Error<R::Error>> {
        let mut power_dbm = power_dbm.clamp(2, 20);

        if power_dbm > 17 {
            self.reg_ctrl.write_verified(reg::PA_DAC, PA_DAC_ON)?;
            power_dbm -= 3;
        } else {
            self.reg_ctrl.write_verified(reg::PA_DAC, PA_DAC_OFF)?;
        }

        self.reg_ctrl.write_verified(reg::PA_CONFIG, PA_BOOST | (power_dbm - 2) as u8)?;

        Ok(())
    }

    pub fn send(&mut self, data: &[u8]) -> Result<(), Error<R::Error>> {
        if self.transmitting {
            return Err(Error::Busy);
        }
        if data.len() > u8::MAX as usize {
            return Err(Error::PacketTooLong { len: data.len() });
        }

        self.set_mode(Mode::Standby)?;
        self.reg_ctrl.write_reg(reg::IRQ_FLAGS, CLEAR_FLAGS)?;
        self.reg_ctrl.write_reg(reg::FIFO_PTR, FIFO_START)?;
        self.reg_ctrl.write_bytes(reg::FIFO, data)?;
        self.reg_ctrl.write_reg(reg::PAYLOAD_LEN, data.len() as u8)?;
        self.set_mode(Mode::Tx)?;

        self.transmitting = true;

        Ok(())
    }

    // true once last packet finished sending (or nothing was sending)
    pub fn is_done(&mut self) -> Result<bool, Error<R::Error>> {
        if !self.transmitting {
            return Ok(true);
        }

        if self.reg_ctrl.read_byte(reg::IRQ_FLAGS)? & TX_DONE == 0 {
            return Ok(false);
        }

        // chip goes back to standby on its own after tx
        self.reg_ctrl.write_reg(reg::IRQ_FLAGS, CLEAR_FLAGS)?;
        self.transmitting = false;

        Ok(true)
    }

    pub fn sleep(&mut self) -> Result<(), Error<R::Error>> {
        self.set_mode(Mode::Sleep)?;
        self.transmitting = false; // sleeping cancels any send in progress
        Ok(())
    }

    // helper so lora bit is always set
    fn set_mode(&mut self, mode: Mode) -> Result<(), Error<R::Error>> {
        self.reg_ctrl.write_reg(reg::OP_MODE, LORA | mode as u8)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::SpiBus;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::digital::{Mock as PinMock, State, Transaction as PinTransaction};
    use embedded_hal_mock::eh1::spi::{Mock, Transaction};

    fn spi_read(reg: u8, data: Vec<u8>) -> Vec<Transaction<u8>> {
        vec![
            Transaction::transaction_start(),
            Transaction::write_vec(vec![reg]),
            Transaction::read_vec(data),
            Transaction::transaction_end(),
        ]
    }

    fn spi_write(reg: u8, val: u8) -> Vec<Transaction<u8>> {
        vec![
            Transaction::transaction_start(),
            Transaction::write_vec(vec![reg | 0x80, val]),
            Transaction::transaction_end(),
        ]
    }

    fn spi_write_verified(reg: u8, val: u8) -> Vec<Transaction<u8>> {
        let mut tx = spi_write(reg, val);
        tx.extend(spi_read(reg, vec![val]));
        tx
    }

    fn reset_pin() -> PinMock {
        PinMock::new(&[
            PinTransaction::set(State::High),
            PinTransaction::set(State::Low),
            PinTransaction::set(State::High),
        ])
    }

    /// everything new says to the chip, in order
    fn init_expectations() -> Vec<Transaction<u8>> {
        let mut tx = spi_read(reg::VERSION, vec![VERSION_VAL]);
        for &(reg, val) in CONFIG {
            tx.extend(spi_write_verified(reg, val));
        }
        tx.extend(spi_write(reg::IRQ_FLAGS, CLEAR_FLAGS));
        tx
    }

    /// everything send says to the chip for a given packet
    fn send_expectations(data: &[u8]) -> Vec<Transaction<u8>> {
        let mut tx = spi_write(reg::OP_MODE, LORA | Mode::Standby as u8);
        tx.extend(spi_write(reg::IRQ_FLAGS, CLEAR_FLAGS));
        tx.extend(spi_write(reg::FIFO_PTR, FIFO_START));
        tx.extend([
            Transaction::transaction_start(),
            Transaction::write_vec(vec![reg::FIFO | 0x80]),
            Transaction::write_vec(data.to_vec()),
            Transaction::transaction_end(),
        ]);
        tx.extend(spi_write(reg::PAYLOAD_LEN, data.len() as u8));
        tx.extend(spi_write(reg::OP_MODE, LORA | Mode::Tx as u8));
        tx
    }

    /// driver that's already through new, with every later expectation queued up
    fn driver(after_init: Vec<Transaction<u8>>) -> (Sx127xDriver<SpiBus<Mock<u8>>>, Mock<u8>) {
        let mut expectations = init_expectations();
        expectations.extend(after_init);
        let mock = Mock::new(&expectations);
        let mut reset = reset_pin();
        let driver = Sx127xDriver::new(SpiBus::new(mock.clone(), MASKS), &mut reset, &mut NoopDelay::new()).unwrap();
        reset.done();
        (driver, mock)
    }

    #[test]
    fn new_succeeds() {
        let (_driver, mut mock) = driver(vec![]);
        mock.done();
    }

    #[test]
    fn new_fails_on_wrong_chip() {
        let mut mock = Mock::new(&spi_read(reg::VERSION, vec![0x00]));
        let mut reset = reset_pin();

        let result = Sx127xDriver::new(SpiBus::new(mock.clone(), MASKS), &mut reset, &mut NoopDelay::new());

        assert!(matches!(
            result,
            Err(InitError::Register(register::Error::WrongId { expected: VERSION_VAL, found: 0x00 }))
        ));
        mock.done();
        reset.done();
    }

    #[test]
    fn set_frequency_915mhz() {
        // 915e6 * 2^19 / 32e6 = 0xE4C000
        let mut tx = spi_write(reg::OP_MODE, LORA | Mode::Standby as u8);
        tx.extend(spi_write_verified(reg::FRF_MSB, 0xE4));
        tx.extend(spi_write_verified(reg::FRF_MID, 0xC0));
        tx.extend(spi_write_verified(reg::FRF_LSB, 0x00));
        let (mut driver, mut mock) = driver(tx);

        driver.set_frequency(915_000_000).unwrap();
        mock.done();
    }

    #[test]
    fn set_power_high_uses_dac() {
        // 20 dbm: dac on, then 20 - 3 - 2 = 15
        let mut tx = spi_write_verified(reg::PA_DAC, PA_DAC_ON);
        tx.extend(spi_write_verified(reg::PA_CONFIG, PA_BOOST | 15));
        let (mut driver, mut mock) = driver(tx);

        driver.set_power(20).unwrap();
        mock.done();
    }

    #[test]
    fn send_then_wait_for_done() {
        let packet = [1, 2, 3];
        let mut tx = send_expectations(&packet);
        tx.extend(spi_read(reg::IRQ_FLAGS, vec![0x00]));    // not done yet
        tx.extend(spi_read(reg::IRQ_FLAGS, vec![TX_DONE])); // done
        tx.extend(spi_write(reg::IRQ_FLAGS, CLEAR_FLAGS));
        let (mut driver, mut mock) = driver(tx);

        driver.send(&packet).unwrap();
        assert!(matches!(driver.send(&packet), Err(Error::Busy))); // no bus traffic
        assert!(!driver.is_done().unwrap());
        assert!(driver.is_done().unwrap());
        assert!(driver.is_done().unwrap()); // not transmitting, no bus traffic
        mock.done();
    }

    #[test]
    fn send_rejects_long_packet() {
        let (mut driver, mut mock) = driver(vec![]);

        assert!(matches!(driver.send(&[0; 256]), Err(Error::PacketTooLong { len: 256 })));
        mock.done();
    }
}