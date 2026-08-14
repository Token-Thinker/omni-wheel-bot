//! Kinematics utilities for 3-wheeled omni-directional robots.
//!
//! The `EmbodiedKinematics` struct computes wheel velocity mappings based on
//! desired translational and rotational motion and inverts wheel measurements
//! back to body velocities. It operates on normalised commands (±1) so its
//! outputs can be fed directly to PWM drivers while still offering helpers to
//! recover physical speeds when needed.
//!
//! # Example
//! ```rust
//! use owb_core::motion::kinematics::EmbodiedKinematics;
//! let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
//! let wheel_speeds = kin.compute_wheel_velocities(0.8, 90.0, 0.0, 0.2);
//! ```
use core::f32::consts::PI;

use libm;

/// Represents the kinematics of a three-wheeled omni-wheel robot.
pub struct EmbodiedKinematics {
    /// Radius of each wheel (m)
    wheel_radius: f32,
    /// Robot center-to-wheel distance (m)
    robot_radius: f32,
    /// Precomputed wheel mounting angles (rad)
    wheel_angles: [f32; 3],
    /// Maximum wheel angular speed (rad/s) used for normalisation
    max_wheel_rad_s: f32,
}

impl EmbodiedKinematics {
    /// Instantiate with a given wheel/robot radii and wheel speed limit.
    pub fn new(
        wheel_radius: f32,
        robot_radius: f32,
        max_wheel_rad_s: f32,
    ) -> Self {
        let wheel_angles = [PI / 3.0, PI, 5.0 * PI / 3.0];
        Self {
            wheel_radius,
            robot_radius,
            wheel_angles,
            max_wheel_rad_s,
        }
    }

    /// Maximum straight-line chassis speed (m/s) for a unit command.
    pub fn max_linear_speed(&self) -> f32 { self.max_wheel_rad_s * self.wheel_radius }

    /// Maximum body yaw rate (rad/s) for a unit command.
    pub fn max_yaw_rad_s(&self) -> f32 {
        if self.robot_radius <= 0.0 {
            0.0
        }
        else {
            self.max_linear_speed() / self.robot_radius
        }
    }

    /// Maximum body yaw rate (deg/s) for a unit command.
    pub fn max_yaw_deg_s(&self) -> f32 { self.max_yaw_rad_s() * (180.0 / PI) }

    /// Convert a yaw rate in deg/s into a clamped normalised command.
    pub fn normalize_yaw_rate_deg(
        &self,
        yaw_rate_deg_s: f32,
    ) -> f32 {
        let max = self.max_yaw_deg_s();
        if max <= 0.0 {
            0.0
        }
        else {
            (yaw_rate_deg_s / max).clamp(-1.0, 1.0)
        }
    }

    /// Convert normalised wheel speeds into physical rad/s.
    pub fn wheel_norm_to_rad_s(
        &self,
        wheel_norm: [f32; 3],
    ) -> [f32; 3] {
        wheel_norm.map(|v| v * self.max_wheel_rad_s)
    }

    /// Transform a global motion command into body-frame velocities.
    ///
    /// `speed` is the translational magnitude (normalised), while `angle` and
    /// `orientation` are in degrees (0° = +X, increasing CCW). Returns `(vx, vy)`
    /// as normalised body-frame components.
    pub fn convert_to_body_frame(
        speed: f32,
        angle: f32,
        orientation: f32,
    ) -> (f32, f32) {
        let a = angle * (PI / 180.0);
        let o = orientation * (PI / 180.0);
        let vx = speed * libm::cosf(a - o);
        let vy = speed * libm::sinf(a - o);
        (-vy, vx)
    }

    /// Build Jacobian J such that ω_wheels = J * [vx, vy, ω_body]
    pub fn construct_jacobian(&self) -> [[f32; 3]; 3] {
        let r = self.wheel_radius;
        let l = self.robot_radius;
        let mut j = [[0.0; 3]; 3];
        for (i, &t) in self.wheel_angles.iter().enumerate() {
            j[i][0] = libm::cosf(t) / r;
            j[i][1] = libm::sinf(t) / r;
            j[i][2] = l / r;
        }
        j
    }

    /// Recover body velocities from measured wheel speeds.
    ///
    /// # Returns
    ///
    /// `(vx, vy, ω)` where `vx`/`vy` are normalised body-frame velocities and `ω`
    /// is the normalised yaw rate.
    pub fn compute_body_velocity(
        &self,
        wheel_velocity: [f32; 3],
    ) -> (f32, f32, f32) {
        let wheel_rad_s = self.wheel_norm_to_rad_s(wheel_velocity);
        let j = self.construct_jacobian();
        let inv = match invert_3x3(j) {
            Some(inv) => inv,
            // Singular/degenerate geometry: report zero body velocity rather
            // than emitting NaN odometry.
            None => return (0.0, 0.0, 0.0),
        };
        let vx =
            inv[0][0] * wheel_rad_s[0] + inv[0][1] * wheel_rad_s[1] + inv[0][2] * wheel_rad_s[2];
        let vy =
            inv[1][0] * wheel_rad_s[0] + inv[1][1] * wheel_rad_s[1] + inv[1][2] * wheel_rad_s[2];
        let w =
            inv[2][0] * wheel_rad_s[0] + inv[2][1] * wheel_rad_s[1] + inv[2][2] * wheel_rad_s[2];
        let max_lin = self.max_linear_speed();
        let max_yaw = self.max_yaw_rad_s();
        let vx_norm = if max_lin > 0.0 { vx / max_lin } else { 0.0 };
        let vy_norm = if max_lin > 0.0 { vy / max_lin } else { 0.0 };
        let w_norm = if max_yaw > 0.0 { w / max_yaw } else { 0.0 };
        (vx_norm, vy_norm, w_norm)
    }

    /// Compute wheel angular velocities to achieve the desired motion.
    ///
    /// `speed` is a normalised forward translational command, `angle` and
    /// `orientation` are in degrees, and `omega` is a normalised rotation rate.
    ///
    /// Returns normalised wheel speeds clamped to ±1.
    pub fn compute_wheel_velocities(
        &self,
        speed: f32,
        angle: f32,
        orientation: f32,
        omega: f32,
    ) -> [f32; 3] {
        let (vx_norm, vy_norm) = Self::convert_to_body_frame(speed, angle, orientation);
        let vx = vx_norm * self.max_linear_speed();
        let vy = vy_norm * self.max_linear_speed();
        let omega = omega.clamp(-1.0, 1.0) * self.max_yaw_rad_s();
        let j = self.construct_jacobian();
        let mut out = [0.0; 3];
        for i in 0..3 {
            let rad_s = j[i][0] * vx + j[i][1] * vy + j[i][2] * omega;
            let norm = if self.max_wheel_rad_s > 0.0 {
                rad_s / self.max_wheel_rad_s
            }
            else {
                0.0
            };
            // A non-finite norm (NaN/inf from degenerate inputs) must collapse
            // to zero: `NaN.clamp(..)` returns NaN, which would otherwise slip
            // through to PWM and drive a wheel uncontrollably.
            out[i] = if !norm.is_finite() || norm.abs() < 1e-6 {
                0.0
            }
            else {
                norm.clamp(-1.0, 1.0)
            };
        }
        out
    }
}

/// Invert a 3×3 matrix using cofactor expansion.
///
/// Returns `None` when the matrix is singular or numerically degenerate
/// (determinant at/near zero, or any non-finite entry). Callers must treat
/// `None` as "no motion" rather than propagating `inf`/`NaN` into wheel
/// commands — a `NaN` PWM value would bypass the downstream clamp and is a
/// physical-safety hazard on an actuated robot.
fn invert_3x3(m: [[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    // Guard against singular (det≈0) and non-finite Jacobians before dividing.
    if !det.is_finite() || libm::fabsf(det) < 1e-9 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convert_to_body_frame_forward() {
        // Forward unit command at 0° gives vx ≈ 0, vy ≈ 1
        let (vx, vy) = EmbodiedKinematics::convert_to_body_frame(1.0, 0.0, 0.0);
        assert!((vx - 0.0).abs() < 1e-6);
        assert!((vy - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_round_trip_body_wheel_velocity() {
        let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
        let speed = 0.5;
        let angle = 30.0;
        let orientation = 15.0;
        let omega = 0.2;
        // compute wheel velocities and back to body velocity
        let wheel_v = kin.compute_wheel_velocities(speed, angle, orientation, omega);
        let (vx, vy, w) = kin.compute_body_velocity(wheel_v);
        // vx, vy should be close to body-frame conversion, w close to omega
        let (bx, by) = EmbodiedKinematics::convert_to_body_frame(speed, angle, orientation);
        assert!((vx - bx).abs() < 1e-3);
        assert!((vy - by).abs() < 1e-3);
        assert!((w - omega).abs() < 1e-3);
    }

    #[test]
    fn test_convert_to_body_frame() {
        // Forward at 0°, no orientation offset => body vx=0, vy=1
        let (vx, vy) = EmbodiedKinematics::convert_to_body_frame(1.0, 0.0, 0.0);
        assert!((vx - 0.0).abs() < 1e-6);
        assert!((vy - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_invert_3x3_identity() {
        let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let inv = invert_3x3(id).expect("identity is invertible");
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (inv[i][j] - id[i][j]).abs() < 1e-6,
                    "inv != id at {}:{}",
                    i,
                    j
                );
            }
        }
    }

    #[test]
    fn test_invert_3x3_singular_returns_none() {
        // A matrix with a zero row is singular: must return None, not panic.
        let singular = [[1.0, 2.0, 3.0], [0.0, 0.0, 0.0], [4.0, 5.0, 6.0]];
        assert!(invert_3x3(singular).is_none());
    }

    #[test]
    fn test_degenerate_geometry_yields_zero_wheels() {
        // A zero wheel radius makes the Jacobian non-finite; the body-velocity
        // recovery must collapse to zero rather than emit NaN.
        let kin = EmbodiedKinematics::new(0.0, 0.2, 7.0);
        let (vx, vy, w) = kin.compute_body_velocity([0.5, -0.5, 0.25]);
        assert_eq!((vx, vy, w), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_compute_wheel_velocities_are_finite() {
        // Non-finite-prone inputs must never produce a NaN wheel command.
        let kin = EmbodiedKinematics::new(0.1, 0.2, 7.0);
        for wheels in [
            kin.compute_wheel_velocities(f32::NAN, 45.0, 0.0, 0.3),
            kin.compute_wheel_velocities(0.8, f32::INFINITY, 0.0, 0.0),
        ] {
            for w in wheels {
                assert!(w.is_finite(), "wheel command must be finite, got {w}");
            }
        }
    }

    #[test]
    fn test_compute_wheel_velocities_zero() {
        let kin = EmbodiedKinematics::new(0.1, 0.2, 7.0);
        let wheels = kin.compute_wheel_velocities(0.0, 0.0, 0.0, 0.0);
        assert_eq!(wheels, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_round_trip_body_velocity() {
        let kin = EmbodiedKinematics::new(0.1, 0.2, 7.0);
        // Some arbitrary motion command (normalised inputs)
        let speed = 0.8;
        let angle = 45.0;
        let orientation = 10.0;
        let omega = 0.5;
        // Compute wheel speeds, then invert back
        let wheel_speeds = kin.compute_wheel_velocities(speed, angle, orientation, omega);
        let (vx, vy, w) = kin.compute_body_velocity(wheel_speeds);
        // vx, vy, w should approximate body-frame motion
        let (exp_vx, exp_vy) = EmbodiedKinematics::convert_to_body_frame(speed, angle, orientation);
        assert!((vx - exp_vx).abs() < 1e-3);
        assert!((vy - exp_vy).abs() < 1e-3);
        assert!((w - omega).abs() < 1e-3);
    }

    #[test]
    fn test_unit_translation_matches_geometry() {
        let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
        let wheels = kin.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);
        let expected = libm::sinf(core::f32::consts::PI / 3.0);
        assert!((wheels[0].abs() - expected).abs() < 1e-5);
        assert!((wheels[2].abs() - expected).abs() < 1e-5);
        assert!((wheels[1]).abs() < 1e-6);
        let rad = kin.wheel_norm_to_rad_s(wheels);
        assert!(rad.iter().all(|&v| v.abs() <= 7.0 + 1e-5));
    }

    #[test]
    fn test_unit_rotation_saturates_norm() {
        let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
        let wheels = kin.compute_wheel_velocities(0.0, 0.0, 0.0, 1.0);
        assert!(wheels.iter().all(|&v| (v.abs() - 1.0).abs() < 1e-5));
    }
}
