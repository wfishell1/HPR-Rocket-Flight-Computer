use crate::sensors::{run_polling, AG_Sensor, SensorKind, SensorSample};
use drivers::bus::DeviceBus;
use drivers::types::Raw3;
use embedded_hal::delay::DelayNs;

pub fn accelcalibrate<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    sensors: &mut [AG_Sensor<'_, I, P, H, M>],
    mut clock: impl FnMut() -> u32,
    orient_x: char,
    orient_y: char,
    orient_z: char,
    accel_counts_per_g: i32,
    high_g_counts_per_g: i32,
) -> [Raw3; 3] {
    let mut sums = [[0_i64; 3]; 3];
    let mut counts = [0_i64; 3];
    for sensor in sensors.iter_mut() {
        sensor.sample.take();
    }
    let start = clock();
    loop {
        let now = clock();
        if now.wrapping_sub(start) >= 3_000_000 {
            break;
        }
        run_polling(sensors, || now);
        for sensor in sensors.iter_mut() {
            let Some(SensorSample::Vector(raw)) = sensor.sample.take() else { continue };
            let channel = match sensor.kind {
                SensorKind::Acceleration => 0,
                SensorKind::Gyroscope => 1,
                SensorKind::HighG => 2,
                _ => continue,
            };
            counts[channel] += 1;
            for (sum, value) in sums[channel].iter_mut().zip([raw.x(), raw.y(), raw.z()]) {
                *sum += i64::from(value);
            }
        }
    }
    let mut averages = [Raw3::default(); 3];
    for i in 0..3 {
        averages[i] = Raw3::new(
            (sums[i][0] / counts[i]) as i16,
            (sums[i][1] / counts[i]) as i16,
            (sums[i][2] / counts[i]) as i16,
        );
    }
    set_orientation(sensors, averages[0], orient_x, orient_y, orient_z);
    apply_calibration_biases(sensors, averages, accel_counts_per_g, high_g_counts_per_g);
    return averages;
}

pub fn magcalibrate<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    magnetometer: &mut AG_Sensor<'_, I, P, H, M>,
    mut clock: impl FnMut() -> u32,
    delay: &mut impl DelayNs,
) -> Raw3 {
    let mut minimum = [0_i32; 3];
    let mut maximum = [0_i32; 3];
    let mut count = 0;

    magnetometer.bias = Raw3::default();
    magnetometer.sample.take();

    while count < 300 {
        delay.delay_ms(100);
        run_polling(std::slice::from_mut(magnetometer), &mut clock);

        let Some(SensorSample::Vector(raw)) = magnetometer.sample.take() else {
            continue;
        };

        let values = [
            i32::from(raw.x()),
            i32::from(raw.y()),
            i32::from(raw.z()),
        ];

        if count == 0 {
            minimum = values;
            maximum = values;
        }
        
        //corresponds with lines 191-197 in the original
        for axis in 0..3 {
            if values[axis] < minimum[axis] {
                minimum[axis] = values[axis];
            }
            if values[axis] > maximum[axis] {
                maximum[axis] = values[axis];
            }
        }

        count += 1;
    }

    let bias = Raw3::new(
        (minimum[0] + (maximum[0] - minimum[0]) / 2) as i16,
        (minimum[1] + (maximum[1] - minimum[1]) / 2) as i16,
        (minimum[2] + (maximum[2] - minimum[2]) / 2) as i16,
    );

    magnetometer.bias = bias;
    return bias;
}

pub fn set_orientation<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    sensors: &mut [AG_Sensor<'_, I, P, H, M>],
    accel_average: Raw3,
    orient_x: char,
    orient_y: char,
    orient_z: char,
) {
    let orient_case = detect_orientation_case(accel_average, orient_x, orient_y, orient_z);
    match orient_case {
        1 => {}
        2 => {
            flip_axis(sensors, 2);
            flip_axis(sensors, 1);
        }
        3 => {
            flip_axis(sensors, 2);
            exchange_axes(sensors, 1, 2);
        }
        4 => {
            flip_axis(sensors, 1);
            exchange_axes(sensors, 1, 2);
        }
        5 => {
            flip_axis(sensors, 2);
            exchange_axes(sensors, 0, 2);
        }
        6 => {
            flip_axis(sensors, 0);
            exchange_axes(sensors, 0, 2);
        }
        _ => {
            println!("Error in determining flight computer orientation");
        }
    }
}

pub fn apply_calibration_biases<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    sensors: &mut [AG_Sensor<'_, I, P, H, M>],
    averages: [Raw3; 3],
    accel_counts_per_g: i32,
    high_g_counts_per_g: i32,
) {
    for sensor in sensors.iter_mut() {
        let (average, counts_per_g) = match sensor.kind {
            SensorKind::Acceleration => (averages[0], accel_counts_per_g),
            SensorKind::Gyroscope => {
                sensor.bias = averages[1];
                continue;
            }
            SensorKind::HighG => (averages[2], high_g_counts_per_g),
            _ => continue,
        };
        let mut bias = [
            i32::from(average.x()),
            i32::from(average.y()),
            i32::from(average.z()),
        ];
        let vertical_axis = sensor.axes[2];
        let vertical_sign = sensor.signs[2];
        bias[vertical_axis] -= vertical_sign * counts_per_g;
        sensor.bias = Raw3::new(bias[0] as i16, bias[1] as i16, bias[2] as i16);
    }
}

pub fn flip_axis<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    sensors: &mut [AG_Sensor<'_, I, P, H, M>],
    axis: usize,
) {
    for sensor in sensors.iter_mut() {
        match sensor.kind {
            SensorKind::Barometer => continue,
            _ => sensor.signs[axis] *= -1,
        }
    }
}

pub fn exchange_axes<I: DeviceBus, P: DeviceBus, H: DeviceBus, M: DeviceBus>(
    sensors: &mut [AG_Sensor<'_, I, P, H, M>],
    axis_a: usize,
    axis_b: usize,
) {
    for sensor in sensors.iter_mut() {
        match sensor.kind {
            SensorKind::Barometer => continue,
            _ => {
                sensor.axes.swap(axis_a, axis_b);
                sensor.signs.swap(axis_a, axis_b);
            }
        }
    }
}
//corresponds to lines 275-305 in the original. Primarily for acceleration, we take the largest axis, and use it for orientation
pub fn detect_orientation_case(
    accel_average: Raw3,
    orient_x: char,
    orient_y: char,
    orient_z: char,
) -> u8 {
    let x = i32::from(accel_average.x());
    let y = i32::from(accel_average.y());
    let z = i32::from(accel_average.z());
    let mut orient_case = 0;

    for (value, other1, other2, raw_axis) in [
        (z, y, x, 'Z'),
        (y, z, x, 'Y'),
        (x, z, y, 'X'),
    ] {
        if value.abs() > other1.abs() && value.abs() > other2.abs() {
            if orient_z == raw_axis {
                orient_case = if value > 0 { 1 } else { 2 };
            }
            if orient_y == raw_axis {
                orient_case = if value > 0 { 3 } else { 4 };
            }
            if orient_x == raw_axis {
                orient_case = if value > 0 { 5 } else { 6 };
            }
        }
    }
    return orient_case;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensors::{Driver, tests::{imu_expectations, MockSensor}};
    use drivers::{bus::I2cBus, h3lis331dl::H3lis331dlDriver, lsm6dsox::Lsm6dsoxDriver};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
    use std::cell::RefCell;

    #[test]
    fn magnetometer_calibration_stores_midpoint_of_raw_extremes() {
        use drivers::lis3mdl::Lis3mdlDriver;
        use embedded_hal_mock::eh1::delay::{CheckedDelay, Transaction as DelayTransaction};

        let mut tx = vec![Transaction::write_read(0x1C, vec![0x0F], vec![0x3D])];
        for (reg, value) in [(0x20, 0x78), (0x21, 0), (0x23, 0x0C), (0x22, 0)] {
            tx.push(Transaction::write(0x1C, vec![reg, value]));
            tx.push(Transaction::write_read(0x1C, vec![reg], vec![value]));
        }
        for i in 0..300 {
            let values: [i16; 3] = if i % 2 == 0 { [100, -300, -100] } else { [300, -100, 500] };
            let bytes = values.into_iter().flat_map(i16::to_le_bytes).collect();
            tx.push(Transaction::write_read(0x1C, vec![0xA8], bytes));
        }
        let mut mock = Mock::new(&tx);
        let driver = Lis3mdlDriver::new(I2cBus::new(mock.clone(), 0x1C).with_inc(0x80)).unwrap();
        let mut sensor: MockSensor<'_> = AG_Sensor::new(
            Driver::Magnetometer(driver), SensorKind::Magnetometer,
            Raw3::new(9, 8, 7), [2, 0, 1], [-1, 1, -1], 0,
        );
        sensor.sample = Some(SensorSample::Vector(Raw3::new(30000, 30000, 30000)));
        let delays = vec![DelayTransaction::delay_ms(100); 300];
        let mut delay = CheckedDelay::new(&delays);
        let mut now = 0;
        let bias = magcalibrate(&mut sensor, || { now += 100_000; now }, &mut delay);

        assert_eq!(bias, Raw3::new(200, -200, 200));
        assert_eq!(sensor.bias, bias);
        assert_eq!(sensor.axes, [2, 0, 1]);
        assert_eq!(sensor.signs, [-1, 1, -1]);
        assert_eq!(now, 30_000_000);
        assert!(sensor.sample.is_none());
        mock.done();
        delay.done();
    }

    #[test]
    fn gravity_correction_preserves_one_g_for_each_axis_and_sign() {
        use crate::sensors::Sensor;

        let mut mock = Mock::new(&imu_expectations());
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        for axes in [[1, 2, 0], [2, 0, 1], [0, 1, 2]] {
            for sign in [-1, 1] {
                for (kind, counts_per_g, channel) in [
                    (SensorKind::Acceleration, 2048, 0),
                    (SensorKind::HighG, 20, 2),
                ] {
                    let mut raw = [22_i16, -8, 12];
                    raw[axes[2]] += (sign * counts_per_g) as i16;
                    let average = Raw3::new(raw[0], raw[1], raw[2]);
                    let mut averages = [Raw3::default(); 3];
                    averages[channel] = average;
                    let mut sensors: [MockSensor<'_>; 1] = [AG_Sensor::new(
                        Driver::Imu(&imu), kind, Raw3::default(), axes, [1, 1, sign], 0,
                    )];
                    apply_calibration_biases(&mut sensors, averages, 2048, 20);
                    assert_eq!(sensors[0].bias, Raw3::new(22, -8, 12));
                    sensors[0].sample = Some(SensorSample::Vector(average));
                    sensors[0].orient();
                    assert_eq!(sensors[0].oriented, Some([0, 0, counts_per_g]));
                }
            }
        }
        mock.done();
    }

    #[test]
    fn gyro_bias_has_no_gravity_correction() {
        use crate::sensors::Sensor;

        let mut mock = Mock::new(&imu_expectations());
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let average = Raw3::new(3, -5, 2);
        let mut sensors: [MockSensor<'_>; 1] = [AG_Sensor::new(
            Driver::Imu(&imu), SensorKind::Gyroscope, Raw3::default(), [2, 0, 1], [-1, 1, -1], 0,
        )];
        apply_calibration_biases(&mut sensors, [Raw3::default(), average, Raw3::default()], 2048, 20);
        assert_eq!(sensors[0].bias, average);
        sensors[0].sample = Some(SensorSample::Vector(average));
        sensors[0].orient();
        assert_eq!(sensors[0].oriented, Some([0, 0, 0]));
        mock.done();
    }

    #[test]
    fn sets_orientation_for_all_six_cases() {
        let mut mock = Mock::new(&imu_expectations());
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        for (average, axes, signs) in [
            (Raw3::new(0, 0, 2048), [0, 1, 2], [1, 1, 1]),
            (Raw3::new(0, 0, -2048), [0, 1, 2], [1, -1, -1]),
            (Raw3::new(0, 2048, 0), [0, 2, 1], [1, -1, 1]),
            (Raw3::new(0, -2048, 0), [0, 2, 1], [1, 1, -1]),
            (Raw3::new(2048, 0, 0), [2, 1, 0], [-1, 1, 1]),
            (Raw3::new(-2048, 0, 0), [2, 1, 0], [1, 1, -1]),
            (Raw3::new(0, 0, 0), [0, 1, 2], [1, 1, 1]),
        ] {
            let mut sensors: [MockSensor<'_>; 1] = [AG_Sensor::new(
                Driver::Imu(&imu), SensorKind::Acceleration,
                Raw3::default(), [0, 1, 2], [1; 3], 0,
            )];
            set_orientation(&mut sensors, average, 'X', 'Y', 'Z');
            assert_eq!(sensors[0].axes, axes);
            assert_eq!(sensors[0].signs, signs);
        }
        mock.done();
    }

    #[test]
    fn exchanged_axes_keep_their_signs_and_can_be_restored() {
        use crate::sensors::Sensor;

        let mut mock = Mock::new(&imu_expectations());
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let mut sensors: [MockSensor<'_>; 2] = [
            SensorKind::Acceleration, SensorKind::Gyroscope,
        ].map(|kind| AG_Sensor::new(
            Driver::Imu(&imu), kind, Raw3::default(), [2, 0, 1], [1, -1, 1], 0,
        ));
        for sensor in &mut sensors {
            sensor.sample = Some(SensorSample::Vector(Raw3::new(10, 20, 30)));
        }

        exchange_axes(&mut sensors, 1, 2);
        for sensor in &mut sensors {
            sensor.orient();
            assert_eq!(sensor.axes, [2, 1, 0]);
            assert_eq!(sensor.signs, [1, 1, -1]);
            assert_eq!(sensor.oriented, Some([30, 20, -10]));
        }

        exchange_axes(&mut sensors, 1, 2);
        for sensor in &mut sensors {
            sensor.orient();
            assert_eq!(sensor.axes, [2, 0, 1]);
            assert_eq!(sensor.signs, [1, -1, 1]);
            assert_eq!(sensor.oriented, Some([30, -10, 20]));
        }
        mock.done();
    }

    #[test]
    fn flipped_sign_is_used_by_orientation_and_can_be_restored() {
        use crate::sensors::Sensor;

        let mut mock = Mock::new(&imu_expectations());
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let mut sensors: [MockSensor<'_>; 2] = [
            SensorKind::Acceleration, SensorKind::Gyroscope,
        ].map(|kind| AG_Sensor::new(
            Driver::Imu(&imu), kind, Raw3::default(), [2, 0, 1], [1, -1, 1], 0,
        ));
        for sensor in &mut sensors {
            sensor.sample = Some(SensorSample::Vector(Raw3::new(10, 20, 30)));
        }

        flip_axis(&mut sensors, 2);
        for sensor in &mut sensors {
            sensor.orient();
            assert_eq!(sensor.signs, [1, -1, -1]);
            assert_eq!(sensor.oriented, Some([30, -10, -20]));
        }

        flip_axis(&mut sensors, 2);
        for sensor in &mut sensors {
            sensor.orient();
            assert_eq!(sensor.signs, [1, -1, 1]);
            assert_eq!(sensor.oriented, Some([30, -10, 20]));
        }
        mock.done();
    }

    #[test]
    fn detects_cases_for_each_raw_axis_and_mapping() {
        for mapping in [
            ['X', 'Y', 'Z'], ['X', 'Z', 'Y'],
            ['Y', 'X', 'Z'], ['Y', 'Z', 'X'],
            ['Z', 'X', 'Y'], ['Z', 'Y', 'X'],
        ] {
            for (axis, name) in ['X', 'Y', 'Z'].into_iter().enumerate() {
                for sign in [1_i16, -1] {
                    let mut values = [0; 3];
                    values[axis] = sign * 2048;
                    let mapped_axis = mapping.iter().position(|&value| value == name).unwrap();
                    let expected = [5, 3, 1][mapped_axis] + if sign < 0 { 1 } else { 0 };
                    assert_eq!(detect_orientation_case(
                        Raw3::new(values[0], values[1], values[2]),
                        mapping[0], mapping[1], mapping[2],
                    ), expected);
                }
            }
        }
    }

    #[test]
    fn tied_axes_leave_orientation_case_zero() {
        for average in [
            Raw3::new(0, 0, 0), Raw3::new(2048, -2048, 0),
            Raw3::new(2048, 0, -2048), Raw3::new(0, 2048, -2048),
        ] {
            assert_eq!(detect_orientation_case(average, 'X', 'Y', 'Z'), 0);
        }
    }

    #[test]
    fn calibrates_from_fresh_raw_averages_across_wrap_then_polls_corrected_vectors() {
        let mut tx = imu_expectations();
        for value in [2_i16, 6, 4] {
            let bytes: Vec<_> = [value, -value, value + 10, value + 20, -value, 2048]
                .into_iter().flat_map(i16::to_le_bytes).collect();
            for _ in 0..2 {
                tx.push(Transaction::write_read(0x6A, vec![0x22], bytes.clone()));
            }
        }
        let mut mock = Mock::new(&tx);
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let mut high_tx = vec![Transaction::write_read(0x18, vec![0x0F], vec![0x32])];
        for (reg, value) in [(0x23, 0), (0x20, 0x3F)] {
            high_tx.push(Transaction::write(0x18, vec![reg, value]));
            high_tx.push(Transaction::write_read(0x18, vec![reg], vec![value]));
        }
        for value in [2_i16, 6, 4] {
            let bytes = [value << 4, -value << 4, 20 << 4].into_iter()
                .flat_map(i16::to_le_bytes).collect();
            high_tx.push(Transaction::write_read(0x18, vec![0xA8], bytes));
        }
        let mut high_mock = Mock::new(&high_tx);
        let high_g = H3lis331dlDriver::new(I2cBus::new(high_mock.clone(), 0x18).with_inc(0x80)).unwrap();
        let start = u32::MAX - 1500;
        let mut sensors: [MockSensor<'_>; 3] = [
            (Driver::Imu(&imu), SensorKind::Acceleration),
            (Driver::Imu(&imu), SensorKind::Gyroscope),
            (Driver::HighG(high_g), SensorKind::HighG),
        ].map(|(driver, kind)| AG_Sensor::new(
            driver, kind, Raw3::new(100, 100, 100), [2, 0, 1], [1; 3], start,
        ));
        for sensor in &mut sensors {
            sensor.sample = Some(SensorSample::Vector(Raw3::new(999, 999, 999)));
        }
        // Unequal repeats would skew means if retained samples were counted again.
        let mut times = [0, 0, 1000, 1000, 1500, 2000, 2000, 2000, 2500, 3_000_000].into_iter();
        let means = accelcalibrate(
            &mut sensors, || start.wrapping_add(times.next().unwrap()),
            'Z', 'X', 'Y', 2048, 20,
        );
        assert_eq!(means, [Raw3::new(24, -4, 2048), Raw3::new(4, -4, 14), Raw3::new(4, -4, 20)]);
        assert!(sensors.iter().all(|sensor| sensor.sample.is_none()));
        assert_eq!(sensors[0].oriented, Some([1948, -74, -106]));
        assert!(times.next().is_none());
        for sensor in &sensors {
            assert_eq!(sensor.axes, [1, 0, 2]);
            assert_eq!(sensor.signs, [-1, 1, 1]);
        }
        assert_eq!(sensors[0].bias, Raw3::new(24, -4, 0));
        assert_eq!(sensors[1].bias, means[1]);
        assert_eq!(sensors[2].bias, Raw3::new(4, -4, 0));
        run_polling(&mut sensors, || start.wrapping_add(3_000_000));
        for (sensor, raw) in sensors.iter().zip(means) {
            assert!(matches!(sensor.sample, Some(SensorSample::Vector(value)) if value == raw));
        }
        assert_eq!(sensors[0].oriented, Some([0, 0, 2048]));
        assert_eq!(sensors[1].oriented, Some([0, 0, 0]));
        assert_eq!(sensors[2].oriented, Some([0, 0, 20]));
        mock.done();
        high_mock.done();
    }

    #[test]
    fn failed_reads_and_preexisting_samples_are_not_averaged() {
        let mut tx = imu_expectations();
        tx.push(Transaction::write_read(0x6A, vec![0x22], vec![0; 12])
            .with_error(embedded_hal::i2c::ErrorKind::Other));
        let mut mock = Mock::new(&tx);
        let imu = RefCell::new(Lsm6dsoxDriver::new(I2cBus::new(mock.clone(), 0x6A)).unwrap());
        let mut sensors: [MockSensor<'_>; 1] = [AG_Sensor::new(
            Driver::Imu(&imu), SensorKind::Acceleration, Raw3::default(), [0, 1, 2], [1; 3], 0,
        )];
        sensors[0].sample = Some(SensorSample::Vector(Raw3::new(1, 2, 3)));
        let mut times = [0, 0, 602, 602, 3_000_000].into_iter();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            accelcalibrate(&mut sensors, || times.next().unwrap(), 'X', 'Y', 'Z', 2048, 20)
        }));
        assert!(result.is_err(), "zero samples must reach the unguarded division");
        mock.done();
    }
}
