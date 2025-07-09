//! Kinematics utilities for 3-wheeled omni-directional robots.
//!
//! The `EmbodiedKinematics` struct computes wheel velocity mappings based on
//! desired translational and rotational motion and inverts wheel measurements back
//! to body velocities.
//!
//! # Example
//! ```rust
//! use owb_core::utils::math::kinematics::EmbodiedKinematics;
//! let kin = EmbodiedKinematics::new(0.148, 0.195);
//! let wheel_speeds = kin.compute_wheel_velocities(1.0, 90.0, 0.0, 0.0);
//! ```
//!
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
}

impl EmbodiedKinematics {
    /// Instantiate with a given wheel and robot radii.
    pub fn new(
        wheel_radius: f32,
        robot_radius: f32,
    ) -> Self {
        let wheel_angles = [PI / 3.0, PI, 5.0 * PI / 3.0];
        Self {
            wheel_radius,
            robot_radius,
            wheel_angles,
        }
    }

    /// Transform a global motion command into body-frame velocities.
    ///
    /// `speed` is the translational magnitude, `angle` and `orientation` are in degrees
    /// (0° = +X, increasing CCW). Returns `(vx, vy)` in the robot's body frame.
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
    /// `(vx, vy, ω)` where `vx`/`vy` are linear body-frame velocities and `ω` is angular velocity.
    pub fn compute_body_velocity(
        &self,
        wheel_velocity: [f32; 3],
    ) -> (f32, f32, f32) {
        let j = self.construct_jacobian();
        let inv = invert_3x3(j);
        let vx = inv[0][0] * wheel_velocity[0]
            + inv[0][1] * wheel_velocity[1]
            + inv[0][2] * wheel_velocity[2];
        let vy = inv[1][0] * wheel_velocity[0]
            + inv[1][1] * wheel_velocity[1]
            + inv[1][2] * wheel_velocity[2];
        let w = inv[2][0] * wheel_velocity[0]
            + inv[2][1] * wheel_velocity[1]
            + inv[2][2] * wheel_velocity[2];
        (vx, vy, w)
    }

    /// Compute wheel angular velocities to achieve the desired motion.
    ///
    /// `speed` is forward translational speed, `angle` and `orientation` are in degrees,
    /// and `omega` is rotational speed (deg/sec). Returns an array of wheel speeds.
    pub fn compute_wheel_velocities(
        &self,
        speed: f32,
        angle: f32,
        orientation: f32,
        omega: f32,
    ) -> [f32; 3] {
        let (vx, vy) = Self::convert_to_body_frame(speed, angle, orientation);
        let v = [vx, vy, omega];
        let j = self.construct_jacobian();
        let mut out = [0.0; 3];
        fn clamp_small(
            v: f32,
            eps: f32,
        ) -> f32 {
            if v.abs() < eps {
                0.0
            } else {
                v
            }
        }
        for i in 0..3 {
            out[i] = clamp_small(j[i][0] * v[0] + j[i][1] * v[1] + j[i][2] * v[2], 1e-6);
        }
        out
    }
}

/// Invert a 3×3 matrix using cofactor expansion.
///
/// # Panics
///
/// Panics if the matrix is singular (determinant is zero).
fn invert_3x3(m: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    let inv_det = 1.0 / det;
    [
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
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let inv = invert_3x3(id);
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
    fn test_compute_wheel_velocities_zero() {
        let kin = EmbodiedKinematics::new(0.1, 0.2);
        let wheels = kin.compute_wheel_velocities(0.0, 0.0, 0.0, 0.0);
        assert_eq!(wheels, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_round_trip_body_velocity() {
        let kin = EmbodiedKinematics::new(0.1, 0.2);
        // Some arbitrary motion command
        let speed = 1.23;
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
}
