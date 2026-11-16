//! Rust replacements for the lookup-table trig in `Main Code/Speed_Trig.ino`.
//!
//! The C++ uses 901-entry tables (0.1° steps) plus a binary search, because older boards
//! had slow trig. The Teensy 4.1 has a hardware double-precision FPU, so these use `libm`
//! (works under `no_std`) and convert to the same output format as the originals:
//! **tenths of a degree, as an integer, rounded toward zero**.
//!
//! The table search can land one step away from the exact answer, so compare against the
//! C++ harness within ±1 tenth of a degree.

/// Clamp to [-1, 1]. Float noise in a rotation matrix can give 1.0000001, and `asin`
/// would return NaN. (The C++ only checks this in testMode.)
fn clamp_unit(v: f64) -> f64 {
    if v > 1.0 {
        1.0
    } else if v < -1.0 {
        -1.0
    } else {
        v
    }
}

/// Radians → tenths of a degree, rounded toward zero (matches the C++ table search,
/// which always lands on the step below).
fn rad_to_tenths(rad: f64) -> i64 {
    (rad.to_degrees() * 10.0) as i64
}

/// `speedArcSin()` (Speed_Trig.ino:453): angle whose sine is `v`.
/// Returns tenths of a degree, -900..=900. Inputs outside [-1, 1] are clamped.
pub fn speed_arc_sin(v: f64) -> i64 {
    rad_to_tenths(libm::asin(clamp_unit(v)))
}

/// `speedAtan2()` (Speed_Trig.ino:536): angle of the point (x, y), any quarter of the circle.
/// Returns tenths of a degree, -1800..=1800. `(0, 0)` returns 0, like the C++.
pub fn speed_atan2(y: f64, x: f64) -> i64 {
    rad_to_tenths(libm::atan2(y, x))
}

/// `speedTan()` (Speed_Trig.ino:393): tangent of an angle given in tenths of a degree.
/// Fixes two C++ bugs: the C++ reads past the table for |rotn| > 1800, and always returns a
/// positive value (tan(-x) and tan(91°..179°) come out positive). Rotation.ino only uses the
/// result squared, so the off-vertical maths gives the same answer either way.
/// At exactly ±90° the result is very large (~1.6e16), not infinity.
pub fn speed_tan(rotn: i64) -> f64 {
    libm::tan((rotn as f64 / 10.0).to_radians())
}

/// `speedArcTan()` (Speed_Trig.ino:402): angle whose tangent is `v`.
/// Returns tenths of a degree, -899..=900 (the C++ returns 900 for v > 573 = tan(89.9°)).
pub fn speed_arc_tan(v: f64) -> i64 {
    rad_to_tenths(libm::atan(v))
}

/// `speedSin()` (Speed_Trig.ino:431): sine of an angle given in tenths of a degree.
/// Any angle works; the C++ folds it into 0..3600 first.
pub fn speed_sin(rotn: i64) -> f64 {
    libm::sin((rotn as f64 / 10.0).to_radians())
}

/// `speedCos()` (Speed_Trig.ino:485): cosine of an angle given in tenths of a degree.
pub fn speed_cos(rotn: i64) -> f64 {
    libm::cos((rotn as f64 / 10.0).to_radians())
}

/// `speedArcCos()` (Speed_Trig.ino:504): angle whose cosine is `v`.
/// Returns tenths of a degree, 0..=1800. Inputs outside [-1, 1] are clamped.
/// Fixes a C++ bug: the binary search treats the decreasing `Cos` table as increasing,
/// so it returns 900 for 1.0, 0 for 0.0, and nonsense in between. Only `magRotn()`
/// (never called) uses it, so no C++ output depends on the broken values.
pub fn speed_arc_cos(v: f64) -> i64 {
    rad_to_tenths(libm::acos(clamp_unit(v)))
}

/// `cosSmallAngle()` (Speed_Trig.ino:552): cosine of a small angle in radians.
/// The C++ uses a Taylor series to avoid `cosf` on slow boards. Its small-angle cutoff
/// compares against `Cos[7]` (~0.99993) instead of `Sin[7]` (0.7° in radians), so the
/// rough `1 - x²/2` is used up to ~57° (off by up to ~4%). The FPU makes `cos` cheap,
/// so this just calls it. Only `getDCM2DRotn()` uses it.
pub fn cos_small_angle(rad: f64) -> f64 {
    libm::cos(rad)
}

/// `sinSmallAngle()` (Speed_Trig.ino:591): sine of a small angle in radians.
/// Same idea as `cos_small_angle`; the C++ cutoffs here are correct, so results match.
pub fn sin_small_angle(rad: f64) -> f64 {
    libm::sin(rad)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(got: i64, want: i64) {
        assert!((got - want).abs() <= 1, "got {}, want {} (±1)", got, want);
    }

    #[test]
    fn arc_sin_known_values() {
        close(speed_arc_sin(0.0), 0);
        close(speed_arc_sin(0.5), 300);
        close(speed_arc_sin(-0.5), -300);
        close(speed_arc_sin(1.0), 900);
        close(speed_arc_sin(-1.0), -900);
    }

    #[test]
    fn arc_sin_clamps_float_noise() {
        assert_eq!(speed_arc_sin(1.000_000_1), 900);
        assert_eq!(speed_arc_sin(-1.000_000_1), -900);
    }

    #[test]
    fn tan_known_values() {
        assert!((speed_tan(0) - 0.0).abs() < 1e-12);
        assert!((speed_tan(450) - 1.0).abs() < 1e-12);
        assert!((speed_tan(-450) + 1.0).abs() < 1e-12); // signed, unlike the C++
        assert!((speed_tan(1350) + 1.0).abs() < 1e-12); // 135° is negative, unlike the C++
        assert!((speed_tan(2250) - 1.0).abs() < 1e-12); // > 180° doesn't overrun
        assert!(speed_tan(900) > 1e15);
    }

    #[test]
    fn arc_tan_known_values() {
        close(speed_arc_tan(0.0), 0);
        close(speed_arc_tan(1.0), 450);
        close(speed_arc_tan(-1.0), -450);
        close(speed_arc_tan(1000.0), 899);
        close(speed_arc_tan(1e20), 900);
    }

    #[test]
    fn tan_arc_tan_round_trip() {
        for a in -899..=899 {
            close(speed_arc_tan(speed_tan(a)), a);
        }
    }

    #[test]
    fn atan2_all_quarters() {
        close(speed_atan2(1.0, 1.0), 450);     // up-right
        close(speed_atan2(1.0, -1.0), 1350);   // up-left
        close(speed_atan2(-1.0, -1.0), -1350); // down-left
        close(speed_atan2(-1.0, 1.0), -450);   // down-right
        close(speed_atan2(1.0, 0.0), 900);
        close(speed_atan2(0.0, -1.0), 1800);
        assert_eq!(speed_atan2(0.0, 0.0), 0);
    }

    #[test]
    fn sin_cos_all_quarters() {
        let eq = |a: f64, b: f64| assert!((a - b).abs() < 1e-12, "got {}, want {}", a, b);
        eq(speed_sin(0), 0.0);
        eq(speed_sin(300), 0.5);
        eq(speed_sin(900), 1.0);
        eq(speed_sin(1500), 0.5);
        eq(speed_sin(2700), -1.0);
        eq(speed_sin(-300), -0.5);
        eq(speed_sin(3900), 0.5); // past one full turn
        eq(speed_cos(0), 1.0);
        eq(speed_cos(600), 0.5);
        eq(speed_cos(1200), -0.5);
        eq(speed_cos(1800), -1.0);
        eq(speed_cos(-600), 0.5);
    }

    #[test]
    fn arc_cos_known_values() {
        close(speed_arc_cos(1.0), 0);
        close(speed_arc_cos(0.5), 600);
        close(speed_arc_cos(0.0), 900);
        close(speed_arc_cos(-0.5), 1200);
        close(speed_arc_cos(-1.0), 1800);
        assert_eq!(speed_arc_cos(1.000_000_1), 0);
    }

    #[test]
    fn arc_sin_arc_cos_round_trip() {
        for a in -900..=900 {
            close(speed_arc_sin(speed_sin(a)), a);
        }
        for a in 0..=1800 {
            close(speed_arc_cos(speed_cos(a)), a);
        }
    }

    #[test]
    fn small_angle_matches_series_below_45_degrees() {
        // The C++ Taylor series (correct branch), as a reference for |x| < pi/4.
        let cos_series = |x: f64| {
            let x2 = x * x;
            1.0 - x2 / 2.0 + x2 * x2 / 24.0 - x2 * x2 * x2 / 720.0
                + x2 * x2 * x2 * x2 / 40320.0
                - x2 * x2 * x2 * x2 * x2 / 3628800.0
        };
        let sin_series = |x: f64| {
            let x2 = x * x;
            x - x * x2 / 6.0 + x * x2 * x2 / 120.0 - x * x2 * x2 * x2 / 5040.0
                + x * x2 * x2 * x2 * x2 / 362880.0
        };
        for i in -78..=78 {
            let x = i as f64 / 100.0;
            assert!((cos_small_angle(x) - cos_series(x)).abs() < 1e-8);
            assert!((sin_small_angle(x) - sin_series(x)).abs() < 1e-8);
        }
    }
}
