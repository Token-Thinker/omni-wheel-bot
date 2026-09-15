//! Numeric helpers shared by the joystick and settings UI.
//!
//! The 20 Hz send throttle itself lives inline in
//! [`crate::components::joystick`], which owns the pointer state it needs. This
//! module previously also carried a standalone `Throttle` struct implementing
//! the same cadence; it was never constructed, and keeping a second copy of the
//! dedup/keepalive/trailing-flush logic meant a bug had to be fixed twice, so
//! it was removed in favour of the live implementation.

pub fn clamp01(v: f64) -> f64 {
    if !v.is_finite() {
        return 0.0;
    }
    v.clamp(0.0, 1.0)
}

pub fn wrap360(v: f64) -> f64 {
    if !v.is_finite() {
        return 0.0;
    }
    let mut r = v % 360.0;
    if r < 0.0 {
        r += 360.0;
    }
    r
}

pub fn round_to(
    v: f64,
    places: u32,
) -> f64 {
    let factor = 10_f64.powi(places as i32);
    (v * factor).round() / factor
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp01_rejects_non_finite_and_bounds() {
        assert_eq!(clamp01(f64::NAN), 0.0);
        assert_eq!(clamp01(f64::INFINITY), 0.0);
        assert_eq!(clamp01(-0.5), 0.0);
        assert_eq!(clamp01(1.5), 1.0);
        assert_eq!(clamp01(0.25), 0.25);
    }

    #[test]
    fn wrap360_normalises_into_range() {
        assert_eq!(wrap360(f64::NAN), 0.0);
        assert_eq!(wrap360(0.0), 0.0);
        assert_eq!(wrap360(360.0), 0.0);
        assert_eq!(wrap360(-90.0), 270.0);
        assert_eq!(wrap360(450.0), 90.0);
    }

    #[test]
    fn round_to_respects_places() {
        assert_eq!(round_to(1.2345, 2), 1.23);
        assert_eq!(round_to(1.2355, 0), 1.0);
        assert_eq!(round_to(359.6, 0), 360.0);
    }
}
