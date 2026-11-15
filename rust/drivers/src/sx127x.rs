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