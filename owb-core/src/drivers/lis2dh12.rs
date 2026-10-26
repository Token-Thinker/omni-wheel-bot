//! Minimal LIS2DH12 register driver used for board presence checks, basic
//! bring-up, and gravity-derived tilt.
//!
//! # This part has no gyroscope
//!
//! The LIS2DH12 is a 3-axis **accelerometer only**. There is no rate sensor, so
//! **yaw is not observable from this device** — a rotation about the gravity
//! vector leaves every axis unchanged. Heading on this platform must come from
//! wheel odometry instead (see [`crate::motion::encoder`]).
//!
//! Roll and pitch *are* observable, because tilting changes how gravity
//! projects onto the axes. [`tilt_from_accel_g`] does that conversion.

use crate::ext::hal;

pub const DEFAULT_ADDR_LOW: u8 = 0x18;
pub const DEFAULT_ADDR_HIGH: u8 = 0x19;
pub const REG_WHO_AM_I: u8 = 0x0F;
pub const WHO_AM_I_RESPONSE: u8 = 0x33;
pub const REG_CTRL1: u8 = 0x20;
pub const REG_CTRL4: u8 = 0x23;
pub const REG_OUT_X_L: u8 = 0x28;
const AUTO_INCREMENT: u8 = 0x80;

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
}

/// Chassis attitude derived from a single gravity measurement.
///
/// Yaw is deliberately absent: see the [module docs](self) — this sensor cannot
/// observe rotation about the gravity vector.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Tilt {
    /// Rotation about the body +X (forward) axis, degrees. Positive = right
    /// side down.
    pub roll_deg: f32,
    /// Rotation about the body +Y (left) axis, degrees. Positive = nose up.
    pub pitch_deg: f32,
    /// How far the measured acceleration magnitude deviates from 1 g.
    ///
    /// This is the trust signal for the angles above. A tilt estimate from a
    /// single accelerometer is only valid while the sensor is **quasi-static**,
    /// because it cannot distinguish gravity from the robot's own acceleration.
    /// Near 0 means the reading was dominated by gravity and the angles are
    /// meaningful; a large value means the robot was accelerating, vibrating,
    /// or being knocked, and the angles should be ignored or heavily filtered.
    ///
    /// See [`Tilt::is_trustworthy`] for the usual gate.
    pub gravity_error_g: f32,
}

impl Tilt {
    /// Default tolerance for [`Tilt::is_trustworthy`], in g.
    ///
    /// 0.08 g admits normal drive accelerations on a wheeled platform while
    /// rejecting impacts and hard starts.
    pub const DEFAULT_GRAVITY_TOLERANCE_G: f32 = 0.08;

    /// True when the acceleration magnitude was close enough to 1 g that the
    /// angles reflect orientation rather than motion.
    pub fn is_trustworthy(&self) -> bool {
        self.gravity_error_g.abs() <= Self::DEFAULT_GRAVITY_TOLERANCE_G
    }
}

/// Convert a gravity reading in g into roll and pitch.
///
/// Inputs are body-frame accelerations in g, as returned by
/// [`Lis2dh12::read_accel_g`]: `+x` forward, `+y` left, `+z` up. At rest on a
/// level surface the expected reading is `(0, 0, 1)`.
///
/// Returns `None` when the sample carries no usable direction information —
/// i.e. the magnitude is ~0 (sensor asleep or disconnected) or any component is
/// non-finite. Callers must treat `None` as "attitude unknown" rather than
/// substituting zeros, since level and unknown are not the same thing on a
/// robot that might be tipping.
///
/// Check [`Tilt::is_trustworthy`] on the result before acting on the angles;
/// this function reports the deviation but does not reject on it, so a caller
/// running a complementary filter can still use a noisy sample with a low
/// weight.
///
/// # Examples
///
/// ```
/// use owb_core::drivers::lis2dh12::tilt_from_accel_g;
///
/// // Level: gravity entirely on +z.
/// let t = tilt_from_accel_g(0.0, 0.0, 1.0).unwrap();
/// assert!(t.roll_deg.abs() < 0.01 && t.pitch_deg.abs() < 0.01);
/// assert!(t.is_trustworthy());
///
/// // Nose up 45 degrees puts -x under gravity.
/// use core::f32::consts::FRAC_1_SQRT_2 as G45;
/// let t = tilt_from_accel_g(-G45, 0.0, G45).unwrap();
/// assert!((t.pitch_deg - 45.0).abs() < 0.1);
///
/// // A 2 g impact is reported but flagged as untrustworthy.
/// let t = tilt_from_accel_g(0.0, 0.0, 2.0).unwrap();
/// assert!(!t.is_trustworthy());
///
/// // Free fall carries no direction.
/// assert!(tilt_from_accel_g(0.0, 0.0, 0.0).is_none());
/// ```
pub fn tilt_from_accel_g(
    ax: f32,
    ay: f32,
    az: f32,
) -> Option<Tilt> {
    if !ax.is_finite() || !ay.is_finite() || !az.is_finite() {
        return None;
    }
    let magnitude = libm::sqrtf(ax * ax + ay * ay + az * az);
    // Below this the sample is indistinguishable from noise, and atan2 would
    // happily return a confident-looking angle from nothing.
    if magnitude < 0.05 {
        return None;
    }
    let horizontal = libm::sqrtf(ay * ay + az * az);
    Some(Tilt {
        roll_deg: libm::atan2f(ay, az).to_degrees(),
        // atan2 against the y/z magnitude (not az alone) keeps pitch correct
        // past 90 degrees of roll, where az changes sign.
        pitch_deg: libm::atan2f(-ax, horizontal).to_degrees(),
        gravity_error_g: magnitude - 1.0,
    })
}

pub struct Lis2dh12<I2C> {
    bus: I2C,
    address: u8,
}

impl<I2C, E> Lis2dh12<I2C>
where
    I2C: hal::i2c::I2c<Error = E>,
{
    pub const fn new(
        bus: I2C,
        address: u8,
    ) -> Self {
        Self { bus, address }
    }

    pub fn who_am_i(&mut self) -> Result<u8, Error<E>> { self.read_reg(REG_WHO_AM_I) }

    pub fn enable_default(&mut self) -> Result<(), Error<E>> { self.write_reg(REG_CTRL1, 0x57) }

    pub fn configure_runtime(&mut self) -> Result<(), Error<E>> {
        // 100 Hz, all axes enabled, high-resolution, BDU on, +/-2 g range.
        self.write_reg(REG_CTRL1, 0x57)?;
        self.write_reg(REG_CTRL4, 0x88)
    }

    pub fn read_accel_g(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let mut bytes = [0u8; 6];
        self.bus
            .write_read(self.address, &[REG_OUT_X_L | AUTO_INCREMENT], &mut bytes)
            .map_err(Error::Bus)?;

        let x = decode_axis(bytes[0], bytes[1]);
        let y = decode_axis(bytes[2], bytes[3]);
        let z = decode_axis(bytes[4], bytes[5]);

        // In high-resolution +/-2g mode the LIS2DH12 reports 1 mg/LSB.
        Ok((x as f32 * 0.001, y as f32 * 0.001, z as f32 * 0.001))
    }

    /// Read the accelerometer and convert it straight to roll/pitch.
    ///
    /// Convenience wrapper over [`Lis2dh12::read_accel_g`] and
    /// [`tilt_from_accel_g`]. `Ok(None)` means the bus read succeeded but the
    /// sample carried no usable direction (see [`tilt_from_accel_g`]); that is
    /// distinct from `Err`, which means the I2C transaction itself failed.
    pub fn read_tilt(&mut self) -> Result<Option<Tilt>, Error<E>> {
        let (ax, ay, az) = self.read_accel_g()?;
        Ok(tilt_from_accel_g(ax, ay, az))
    }

    pub fn release(self) -> I2C { self.bus }

    fn write_reg(
        &mut self,
        reg: u8,
        value: u8,
    ) -> Result<(), Error<E>> {
        self.bus
            .write(self.address, &[reg, value])
            .map_err(Error::Bus)
    }

    fn read_reg(
        &mut self,
        reg: u8,
    ) -> Result<u8, Error<E>> {
        let mut value = [0u8; 1];
        self.bus
            .write_read(self.address, &[reg], &mut value)
            .map_err(Error::Bus)?;
        Ok(value[0])
    }
}

fn decode_axis(
    low: u8,
    high: u8,
) -> i16 {
    i16::from_le_bytes([low, high]) >> 4
}

#[cfg(test)]
mod tests {
    use core::f32::consts::FRAC_1_SQRT_2 as G45;

    use super::*;

    fn approx(
        a: f32,
        b: f32,
        tol: f32,
    ) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn level_reads_zero_tilt() {
        let t = tilt_from_accel_g(0.0, 0.0, 1.0).expect("level sample");
        assert!(approx(t.roll_deg, 0.0, 0.01));
        assert!(approx(t.pitch_deg, 0.0, 0.01));
        assert!(approx(t.gravity_error_g, 0.0, 0.001));
        assert!(t.is_trustworthy());
    }

    #[test]
    fn roll_and_pitch_signs_follow_the_documented_convention() {
        // Right side down puts gravity on +y -> positive roll.
        let right_down = tilt_from_accel_g(0.0, G45, G45).expect("sample");
        assert!(approx(right_down.roll_deg, 45.0, 0.1), "{right_down:?}");
        assert!(approx(right_down.pitch_deg, 0.0, 0.1), "{right_down:?}");

        // Nose up tips gravity onto -x -> positive pitch.
        let nose_up = tilt_from_accel_g(-G45, 0.0, G45).expect("sample");
        assert!(approx(nose_up.pitch_deg, 45.0, 0.1), "{nose_up:?}");
        assert!(approx(nose_up.roll_deg, 0.0, 0.1), "{nose_up:?}");

        // Nose down is the mirror image.
        let nose_down = tilt_from_accel_g(G45, 0.0, G45).expect("sample");
        assert!(approx(nose_down.pitch_deg, -45.0, 0.1), "{nose_down:?}");
    }

    #[test]
    fn pitch_survives_past_ninety_degrees_of_roll() {
        // Rolled fully onto its side: az flips sign. Taking atan2 against az
        // alone would fold pitch; against the y/z magnitude it stays put.
        let t = tilt_from_accel_g(0.0, 1.0, 0.0).expect("sample");
        assert!(
            approx(t.pitch_deg, 0.0, 0.1),
            "pitch should stay level: {t:?}"
        );
        assert!(approx(t.roll_deg, 90.0, 0.1), "{t:?}");

        let upside_down = tilt_from_accel_g(0.0, 0.0, -1.0).expect("sample");
        assert!(
            approx(upside_down.roll_deg.abs(), 180.0, 0.1),
            "{upside_down:?}"
        );
        assert!(approx(upside_down.pitch_deg, 0.0, 0.1), "{upside_down:?}");
    }

    #[test]
    fn acceleration_is_flagged_untrustworthy() {
        // Hard brake: gravity plus a forward deceleration.
        let t = tilt_from_accel_g(0.5, 0.0, 1.0).expect("sample");
        assert!(t.gravity_error_g > 0.1, "{t:?}");
        assert!(
            !t.is_trustworthy(),
            "an accelerating sample must not be trusted as attitude: {t:?}"
        );
        // Gentle drive accel stays within tolerance so a filter can use it.
        let gentle = tilt_from_accel_g(0.03, 0.0, 1.0).expect("sample");
        assert!(gentle.is_trustworthy(), "{gentle:?}");
    }

    #[test]
    fn degenerate_samples_return_none() {
        assert!(tilt_from_accel_g(0.0, 0.0, 0.0).is_none(), "free fall");
        assert!(tilt_from_accel_g(f32::NAN, 0.0, 1.0).is_none());
        assert!(tilt_from_accel_g(0.0, f32::INFINITY, 1.0).is_none());
    }

    /// The reading measured on the real v1-alpha bench unit while level.
    #[test]
    fn matches_measured_bench_reading() {
        let t = tilt_from_accel_g(0.032, 0.005, 0.995).expect("sample");
        assert!(t.is_trustworthy(), "{t:?}");
        // Small mounting offset, not a real tilt.
        assert!(t.roll_deg.abs() < 1.0, "{t:?}");
        assert!(t.pitch_deg.abs() < 3.0, "{t:?}");
    }
}
