use super::{process, FakeOutputPin};
use hpr_rust::sensors::setup_sensors;
use drivers::h3lis331dl::H3lis331dlDriver;
use drivers::lis3mdl::Lis3mdlDriver;
use drivers::bus::I2cBus;
use drivers::lsm6dsox::Lsm6dsoxDriver;
use drivers::ms5611::Ms5611Driver;
use drivers::types::Raw3;
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
use hpr_rust::acceleration::AccelerationState;
use hpr_rust::event_logic::Flight;
use hpr_rust::flight_clock::FlightClock;
use hpr_rust::settings::{SensorInputs, Settings};
use std::cell::RefCell;

fn initialize_hardware() {}

pub(super) fn mock_aux_sensors(high_g_reads: usize, mag_reads: usize) -> (
    H3lis331dlDriver<I2cBus<Mock>>, Lis3mdlDriver<I2cBus<Mock>>, Mock, Mock,
) {
    let mut high_g = vec![Transaction::write_read(0x18, vec![0x0F], vec![0x32])];
    for (reg, value) in [(0x23, 0x00), (0x20, 0x3F)] {
        high_g.push(Transaction::write(0x18, vec![reg, value]));
        high_g.push(Transaction::write_read(0x18, vec![reg], vec![value]));
    }
    high_g.extend((0..high_g_reads).map(|_| Transaction::write_read(0x18, vec![0xA8], vec![0; 6])));
    let high_g_mock = Mock::new(&high_g);
    let high_g_driver = H3lis331dlDriver::new(I2cBus::new(high_g_mock.clone(), 0x18).with_inc(0x80)).unwrap();
    let mut mag = vec![Transaction::write_read(0x1C, vec![0x0F], vec![0x3D])];
    for (reg, value) in [(0x20, 0x78), (0x21, 0x00), (0x23, 0x0C), (0x22, 0x00)] {
        mag.push(Transaction::write(0x1C, vec![reg, value]));
        mag.push(Transaction::write_read(0x1C, vec![reg], vec![value]));
    }
    mag.extend((0..mag_reads).map(|_| Transaction::write_read(0x1C, vec![0xA8], vec![0; 6])));
    let mag_mock = Mock::new(&mag);
    let mag_driver = Lis3mdlDriver::new(I2cBus::new(mag_mock.clone(), 0x1C).with_inc(0x80)).unwrap();
    (high_g_driver, mag_driver, high_g_mock, mag_mock)
}

pub(super) fn imu_init_expectations() -> Vec<Transaction> {
    let address = 0x6A;
    vec![
        Transaction::write_read(address, vec![0x0F], vec![0x6C]),
        Transaction::write(address, vec![0x12, 0x44]),
        Transaction::write_read(address, vec![0x12], vec![0x44]),
        Transaction::write(address, vec![0x11, 0b10001100]),
        Transaction::write_read(address, vec![0x11], vec![0b10001100]),
        Transaction::write(address, vec![0x10, 0b10000100]),
        Transaction::write_read(address, vec![0x10], vec![0b10000100]),
    ]
}

pub(super) fn barometer_init_expectations() -> Vec<Transaction> {
    let barometer_address = 0x77;
    let calibration: [u16; 6] = [40127, 36924, 23317, 23282, 33464, 28312];
    let mut barometer_expectations = vec![Transaction::write(barometer_address, vec![0x1E])];
    for (index, value) in calibration.iter().enumerate() {
        barometer_expectations.push(Transaction::write_read(
            barometer_address,
            vec![0xA2 + 2 * index as u8],
            value.to_be_bytes().to_vec(),
        ));
    }
    barometer_expectations.extend([
        Transaction::write(barometer_address, vec![0x58]),
        Transaction::write_read(
            barometer_address,
            vec![0x00],
            8569150_u32.to_be_bytes()[1..].to_vec(),
        ),
        Transaction::write(barometer_address, vec![0x48]),
    ]);
    barometer_expectations
}

pub(super) fn initialize() {
    //This is all setup code, currently done with fake inputs. Will have to switch to real inputs if we do this on actual hardware
    let address = 0x6A;
    let mut expectations = imu_init_expectations();
    for _ in 0..2 {
        expectations.push(Transaction::write_read(
            address,
            vec![0x22],
            vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x08],
        ));
    }
    let mut mock = Mock::new(&expectations);
    let bus = I2cBus::new(mock.clone(), address);
    let imu = RefCell::new(Lsm6dsoxDriver::new(bus).unwrap());

    let barometer_address = 0x77;
    let mut barometer_expectations = barometer_init_expectations();
    barometer_expectations.extend([
        Transaction::write_read(
            barometer_address,
            vec![0x00],
            9085466_u32.to_be_bytes()[1..].to_vec(),
        ),
        Transaction::write(barometer_address, vec![0x48]),
    ]);
    let mut barometer_mock = Mock::new(&barometer_expectations);
    let barometer_driver = Ms5611Driver::new(
        I2cBus::new(barometer_mock.clone(), barometer_address),
        &mut NoopDelay::new(),
    ).unwrap();

    let mut pyro1 = FakeOutputPin::default();
    let mut pyro2 = FakeOutputPin::default();
    let mut pyro3 = FakeOutputPin::default();
    let mut pyro4 = FakeOutputPin::default();
    let (high_g, mag, mut high_g_mock, mut mag_mock) = mock_aux_sensors(1, 0);
    let mut sensors = setup_sensors(
        &imu,
        barometer_driver,
        high_g,
        mag,
        (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
        (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
        (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
        (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
        0,
    );
    let clock = || 10_000_u32;
    let mut flight = Flight::new(Settings::default());
    let mut flight_clock = FlightClock::new();
    let mut acceleration = AccelerationState::new();
    let mut barometer = hpr_rust::baro_samp::BarometerState::new();
    let sea_level_pressure_pa: f32 = 101_325.0;
    let mut inputs = SensorInputs::default();
    let mut rotation = hpr_rust::rotations::rotation_info::new(flight.settings);

    process(
        &mut sensors, clock, &mut pyro1, &mut pyro2, &mut pyro3, &mut pyro4,
        &mut flight, &mut flight_clock, &mut acceleration, &mut barometer, sea_level_pressure_pa,
        &mut inputs, &mut rotation,
    ).unwrap();
    mock.done();
    barometer_mock.done();
    high_g_mock.done();
    mag_mock.done();
}
