
use drivers::bus::DeviceBus;
use embedded_hal::digital::{ErrorType, OutputPin};
use drivers::lsm6dsox;
use hpr_rust::acceleration::AccelerationState;
use hpr_rust::event_logic::{Flight, Phase};
use hpr_rust::flight_clock::FlightClock;
use hpr_rust::settings::SensorInputs;
mod initialization;
use hpr_rust::sensors::{run_polling, AG_Sensor, SensorSample};

#[derive(Default)]
struct FakeOutputPin {
    high: bool,
}

impl ErrorType for FakeOutputPin {
    type Error = std::convert::Infallible;
}

impl OutputPin for FakeOutputPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.high = false;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.high = true;
        Ok(())
    }
}

fn main() {
    initialization::initialize();
}

fn process<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus, O: OutputPin>(
    sensors: &mut [AG_Sensor<'_, I, P, H, M>; 5],
    mut clock: impl FnMut() -> u32,
    pyro1: &mut O,
    pyro2: &mut O,
    pyro3: &mut O,
    pyro4: &mut O,
    flight: &mut Flight,
    flight_clock: &mut FlightClock,
    acceleration: &mut AccelerationState,
    barometer: &mut hpr_rust::baro_samp::BarometerState,
    sea_level_pressure_pa: f32,
    inputs: &mut SensorInputs,
    rotation: &mut hpr_rust::rotations::rotation_info,
) -> Result<(), O::Error> {
    //From here, the actual logic of the loop starts

    run_polling(sensors, &mut clock);
    let now_us = clock();

    let mut new_baro_sample = false;
    if let Some(SensorSample::Barometer(pressure_pa)) = sensors[2].sample.take() {
        if pressure_pa > 0 {
            //we convert raw pressure reading into altitude here
            let raw_altitude_m = 44_330.0 * (1.0 - (pressure_pa as f32 / sea_level_pressure_pa).powf(0.1903));
            barometer.process_sample(
                raw_altitude_m, sensors[2].last_sample_us, flight.flight_phase,
                matches!(flight.flight_phase, Phase::Apogee | Phase::Drogue | Phase::Main | Phase::Landed),
            );
            new_baro_sample = true;
        }
    }

    inputs.baro_alt = barometer.alt;
    inputs.baro_base_alt = barometer.base_alt;
    inputs.baro_vel = barometer.vel;
    inputs.baro_touchdown = barometer.touchdown_count;

    let Some(oriented) = sensors[0].oriented else {
        return Ok(());
    };
    let Ok(accel_z) = i16::try_from(oriented[2]) else {
        return Ok(());
    };

    inputs.accel_z = accel_z;
    flight.update_current_info(inputs);

    if flight.flight_phase == Phase::Pad {
        flight.liftoff_check();
        if flight.flight_phase == Phase::BoostUnconfirmed {
            flight_clock.start(now_us);
            *acceleration = AccelerationState::new();
        }
    }

    if matches!(flight.flight_phase, Phase::Pad | Phase::Landed | Phase::TimedOut) {
        return Ok(());
    }

    if let Some(dt_us) = flight_clock.advance(now_us) {
        //turns raw acceloremeter reading into something which can be used for flight logic
        let accel_now = (oriented[2] - i32::from(flight.settings.counts_per_g)) as f32 * lsm6dsox::ACCEL_SCALE * 9.80665;

        if matches!(flight.flight_phase, Phase::Drogue | Phase::Main) {
            acceleration.accel_now = accel_now;
        } else {
            acceleration.update(accel_now, dt_us, barometer, new_baro_sample);
        }

        if let Some(gyro) = sensors[1].oriented {
            rotation.accumulate(gyro, dt_us);
        }
        rotation.update(flight_clock.flight_time_us, f64::from(lsm6dsox::GYRO_SCALE));

        inputs.flight_time_us = flight_clock.flight_time_us;
        inputs.accel_now = acceleration.accel_now;
        inputs.accel_vel = acceleration.accel_vel;
        inputs.accel_alt = acceleration.accel_alt;
        inputs.fusion_vel = acceleration.fusion_vel;
        let output = flight.check_events(inputs);
        let results = [
            if output.pyros[0] { pyro1.set_high() } else { pyro1.set_low() },
            if output.pyros[1] { pyro2.set_high() } else { pyro2.set_low() },
            if output.pyros[2] { pyro3.set_high() } else { pyro3.set_low() },
            if output.pyros[3] { pyro4.set_high() } else { pyro4.set_low() },
        ];
        for result in results {
            result?;
        }
        //Skipping the current switching to highG for now
        //Also skipping canard updates
        //In addition, telemetry is left undone for now
        //Also skipping battery updates
        //Deferring beeper, eeprom, radio, and serial stuff for now as well
    }
    Ok(())
}






#[cfg(test)]
mod tests {
    use super::*;
    use super::initialization::{barometer_init_expectations, imu_init_expectations, mock_aux_sensors};
    use drivers::lsm6dsox::Lsm6dsoxDriver;
    use drivers::ms5611::Ms5611Driver;
    use drivers::types::Raw3;
    use std::cell::RefCell;
    use hpr_rust::sensors::setup_sensors;
    use drivers::bus::I2cBus;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use hpr_rust::settings::Settings;

    #[test]
    fn process_complete_single_stage_flight() {
        // Ideal vertical flight, 100 Hz, constant temperature, no sensor noise.
        // Recovery is scripted independently of the flight computer's decisions.
        let settings = Settings {
            main_deploy_alt_m: 100.0,
            apogee_delay_us: 200_000, // Keep Apogee observable between ticks.
            ..Settings::default()
        };
        let mut expectations = imu_init_expectations();
        let mut barometer_expectations = barometer_init_expectations();
        let mut altitude = 0.0_f64;
        let mut velocity = 0.0_f64;
        let mut ground_time = None;
        let mut peak_altitude = 0.0_f64;
        let dt = 0.01;
        let gain = f64::from(lsm6dsox::ACCEL_SCALE) * 9.80665;
        // Invert the datasheet pressure equation for the initialization helper's
        // factory coefficients and D2=8569150 (20.07 C, no cold compensation).
        let off = (36924_i64 << 16) + ((23282_i64 * 2366) >> 7);
        let sens = (40127_i64 << 15) + ((23317_i64 * 2366) >> 8);
        for tick in 1..=6000_u32 {
            let t = (tick as f64 * dt - 1.0).max(0.0);
            let net_accel = if tick < 100 {
                0.0
            } else if t < 3.0 {
                4096.0 * gain
            } else if t < 10.0 {
                -2048.0 * gain
            } else if altitude > 0.0 {
                let target = if altitude > 80.0 { -8.0 } else { -4.0 };
                ((target - velocity) / dt).clamp(-16.0, 16.0)
            } else {
                0.0
            };
            // Samples describe the start of each interval; integrate the same
            // acceleration to generate the next interval's pressure altitude.
            let accel_z = (2048.0 + net_accel / gain).round() as i16;
            let mut bytes = vec![0; 12]; // Zero gyro: ideal nonrotating rocket.
            bytes[10..12].copy_from_slice(&accel_z.to_le_bytes());
            for _ in 0..2 {
                expectations.push(Transaction::write_read(0x6A, vec![0x22], bytes.clone()));
            }
            let pressure = 101325.0 * (1.0 - altitude / 44330.0).powf(1.0 / 0.1903);
            let raw_pressure = (((pressure * 32768.0 + off as f64) * 2097152.0)
                / sens as f64).round() as u32;
            assert!(raw_pressure < 1 << 24);
            // Every eleventh poll reads temperature, following ten pressures.
            let raw = if tick % 11 == 0 { 8569150 } else { raw_pressure };
            barometer_expectations.extend([
                Transaction::write_read(0x77, vec![0x00], raw.to_be_bytes()[1..].to_vec()),
                Transaction::write(0x77, vec![if tick % 11 == 10 { 0x58 } else { 0x48 }]),
            ]);
            altitude += velocity * dt + 0.5 * net_accel * dt * dt;
            velocity += net_accel * dt;
            peak_altitude = peak_altitude.max(altitude);
            if t > 10.0 && altitude <= 0.0 {
                ground_time.get_or_insert((tick + 1) * 10_000);
                altitude = 0.0;
                velocity = 0.0;
            }
        }
        let mut mock = Mock::new(&expectations);
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let mut barometer_mock = Mock::new(&barometer_expectations);
        let barometer_driver = Ms5611Driver::new(
            I2cBus::new(barometer_mock.clone(), 0x77), &mut NoopDelay::new(),
        ).unwrap();
        // High-G is due every tick; 25 ms magnetometer period rounds up to 30 ms.
        let (high_g, mag, mut high_g_mock, mut mag_mock) = mock_aux_sensors(6000, 2000);
        let mut sensors = setup_sensors(
            &imu, barometer_driver, high_g, mag,
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]), 0,
        );
        let mut pyro1 = FakeOutputPin::default();
        let mut pyro2 = FakeOutputPin::default();
        let mut pyro3 = FakeOutputPin::default();
        let mut pyro4 = FakeOutputPin::default();
        let mut flight = Flight::new(settings);
        let mut flight_clock = FlightClock::new();
        let mut acceleration = AccelerationState::new();
        let mut barometer = hpr_rust::baro_samp::BarometerState::new();
        let mut inputs = SensorInputs::default();
        let mut rotation = hpr_rust::rotations::rotation_info::new(settings);
        let mut timeline = vec![(0, Phase::Pad)];
        let mut pyro_edges = Vec::new();
        let mut previous_pyros = [false; 4];
        let mut landed_at = None;
        for tick in 1..=6000_u32 {
            let now_us = tick * 10_000;
            process(
                &mut sensors, || now_us, &mut pyro1, &mut pyro2, &mut pyro3, &mut pyro4,
                &mut flight, &mut flight_clock, &mut acceleration, &mut barometer, 101_325.0,
                &mut inputs, &mut rotation,
            ).unwrap();
            if timeline.last().unwrap().1 != flight.flight_phase {
                timeline.push((now_us, flight.flight_phase));
            }
            let pin_states = [pyro1.high, pyro2.high, pyro3.high, pyro4.high];
            for channel in 0..4 {
                let active = pin_states[channel];
                assert_eq!(active, flight.pyro_n[channel].0);
                if active != previous_pyros[channel] {
                    pyro_edges.push((inputs.flight_time_us, channel, active));
                    previous_pyros[channel] = active;
                }
            }
            if flight.flight_phase == Phase::Landed {
                landed_at.get_or_insert(now_us);
            } else if now_us >= 1_000_000 {
                assert_eq!(flight_clock.flight_time_us, now_us - 1_000_000);
            }
        }
        mock.done();
        barometer_mock.done();
        high_g_mock.done();
        mag_mock.done();
        println!("phase timeline (wall us): {timeline:?}");
        println!("pyro edges (flight us, zero-based channel, active): {pyro_edges:?}");
        println!("trajectory peak={peak_altitude:.2} m, ground={ground_time:?} us, landed={landed_at:?} us");
        assert_eq!(timeline.iter().map(|entry| entry.1).collect::<Vec<_>>(), vec![
            Phase::Pad, Phase::BoostUnconfirmed, Phase::Boost, Phase::BurnoutCheck,
            Phase::Coast, Phase::Apogee, Phase::Drogue, Phase::Main, Phase::Landed,
        ]);
        let ground_time = ground_time.expect("trajectory must reach the ground");
        let landed_at = landed_at.expect("flight computer must detect touchdown");
        assert!(landed_at >= ground_time && landed_at < ground_time + 1_000_000);
        assert_eq!(flight.flight_end_time_us, Some(landed_at - 1_000_000));
        assert_eq!(flight_clock.flight_time_us, landed_at - 1_000_000);
        assert_eq!(pyro_edges.len(), 4, "one pulse per recovery channel");
        for (edges, channel) in pyro_edges.chunks_exact(2).zip([2, 3]) {
            assert_eq!((edges[0].1, edges[0].2), (channel, true));
            assert_eq!((edges[1].1, edges[1].2), (channel, false));
            assert_eq!(edges[1].0 - edges[0].0, settings.fire_time_us + 10_000);
        }
        assert!(flight.sync_apogee && flight.sync_mains);
        assert!(barometer.max_alt > 250.0 && barometer.max_alt < 280.0);
        assert!(acceleration.max_velocity > 50.0);
        assert!(acceleration.accel_alt > 200.0);
        assert!(flight.pyro_n.iter().all(|pyro| !pyro.0));
        assert!(!pyro1.high && !pyro2.high && !pyro3.high && !pyro4.high);
    }

    #[test]
    fn process_retains_state_from_pad_through_liftoff_and_flight_update() {
        let mut expectations = imu_init_expectations();
        for accel_z in [2048_i16, 6144, 6144] {
            let mut bytes = vec![0; 12];
            bytes[10..12].copy_from_slice(&accel_z.to_le_bytes());
            // Acceleration and gyroscope polling each read the shared IMU.
            for _ in 0..2 {
                expectations.push(Transaction::write_read(0x6A, vec![0x22], bytes.clone()));
            }
        }
        let mut mock = Mock::new(&expectations);
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());

        let mut barometer_expectations = barometer_init_expectations();
        // Pressure conversions are due at 10 and 20 ms, but not at 25 ms
        // (9090 us sample period). Initialization already read temperature.
        for _ in 0..2 {
            barometer_expectations.extend([
                Transaction::write_read(0x77, vec![0x00], 9085466_u32.to_be_bytes()[1..].to_vec()),
                Transaction::write(0x77, vec![0x48]),
            ]);
        }
        let mut barometer_mock = Mock::new(&barometer_expectations);
        let barometer_driver = Ms5611Driver::new(
            I2cBus::new(barometer_mock.clone(), 0x77), &mut NoopDelay::new(),
        ).unwrap();
        let (high_g, mag, mut high_g_mock, mut mag_mock) = mock_aux_sensors(3, 1);
        let mut sensors = setup_sensors(
            &imu, barometer_driver, high_g, mag,
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]), 0,
        );
        let mut pyro1 = FakeOutputPin::default();
        let mut pyro2 = FakeOutputPin::default();
        let mut pyro3 = FakeOutputPin::default();
        let mut pyro4 = FakeOutputPin::default();
        let mut flight = Flight::new(Settings::default());
        let mut flight_clock = FlightClock::new();
        let mut acceleration = AccelerationState::new();
        let mut barometer = hpr_rust::baro_samp::BarometerState::new();
        let mut inputs = SensorInputs::default();
        let mut rotation = hpr_rust::rotations::rotation_info::new(flight.settings);
        let raw_altitude = 44_330.0 * (1.0 - (100_009.0_f32 / 101_325.0).powf(0.1903));

        // Only 5 ms of flight: deliberately excludes the known broken quaternion
        // calculation, which runs after more than 10 ms of flight time.
        for now_us in [10_000, 20_000, 25_000] {
            process(
                &mut sensors, || now_us, &mut pyro1, &mut pyro2, &mut pyro3, &mut pyro4,
                &mut flight, &mut flight_clock, &mut acceleration, &mut barometer, 101_325.0,
                &mut inputs, &mut rotation,
            ).unwrap();
            assert_eq!(sensors[0].last_sample_us, now_us);
            assert_eq!(sensors[1].last_sample_us, now_us);
            assert_eq!(sensors[1].oriented, Some([0, 0, 0]));
            assert_eq!(sensors[2].last_sample_us, now_us.min(20_000));
            assert!(sensors[2].sample.is_none());
            assert_eq!(inputs.baro_alt, barometer.alt);
            assert_eq!(inputs.baro_base_alt, barometer.base_alt);
            assert_eq!(inputs.baro_vel, barometer.vel);
            assert_eq!(flight.pyro_n, [(false, 0); 4]);
            assert!(!pyro1.high && !pyro2.high && !pyro3.high && !pyro4.high);

            if now_us == 10_000 {
                assert_eq!(flight.flight_phase, Phase::Pad);
                assert_eq!(inputs.accel_z, 2048);
                assert_eq!(flight_clock.flight_time_us, 0);
                assert_eq!(inputs.flight_time_us, 0);
                assert_eq!(acceleration.accel_now, 0.0);
                assert_eq!(acceleration.accel_vel, 0.0);
                assert_eq!(acceleration.accel_alt, 0.0);
                assert!((barometer.base_alt - raw_altitude / 30.0).abs() < 1e-5);
                // Nonzero prelaunch integration makes the liftoff reset observable.
                acceleration.accel_vel = 12.0;
                acceleration.accel_alt = 34.0;
                acceleration.fusion_vel = 56.0;
                acceleration.update_high_g(123);
            } else {
                assert_eq!(flight.flight_phase, Phase::BoostUnconfirmed);
                assert_eq!(inputs.accel_z, 6144);
                assert!(inputs.accel_z > flight.settings.g_trigger_counts);
                assert_eq!(sensors[0].oriented, Some([0, 0, 6144]));
                assert!((barometer.base_alt - raw_altitude * 2.0 / 30.0).abs() < 1e-5);
                assert_eq!(flight_clock.flight_time_us, now_us - 20_000);
                assert_eq!(inputs.flight_time_us, flight_clock.flight_time_us);
                assert_eq!(acceleration.high_g_smooth, 0);
                assert!((acceleration.accel_now - 19.602003).abs() < 1e-5);
                if now_us == 20_000 {
                    assert_eq!(acceleration.accel_vel, 0.0);
                    assert_eq!(acceleration.accel_alt, 0.0);
                    assert_eq!(acceleration.fusion_vel, 0.0);
                } else {
                    assert!(flight_clock.flight_time_us > 0);
                    assert!((acceleration.accel_vel - 0.098010015).abs() < 1e-7);
                    assert!((acceleration.accel_alt - 0.0004900501).abs() < 1e-9);
                    assert_eq!(acceleration.fusion_vel, acceleration.accel_vel);
                }
                assert_eq!(inputs.accel_now, acceleration.accel_now);
                assert_eq!(inputs.accel_vel, acceleration.accel_vel);
                assert_eq!(inputs.accel_alt, acceleration.accel_alt);
                assert_eq!(inputs.fusion_vel, acceleration.fusion_vel);
            }
        }
        mock.done();
        barometer_mock.done();
        high_g_mock.done();
        mag_mock.done();
    }
}
