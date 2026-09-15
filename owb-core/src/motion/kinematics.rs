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
//! // wheel radius 19 mm (38 mm Nexus omni), centre-to-wheel 195 mm,
//! // max wheel speed 50 rad/s (measured).
//! let kin = EmbodiedKinematics::new(0.019, 0.195, 50.0);
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
    /// Precomputed wheel **drive-direction** angles (rad).
    ///
    /// These are the directions each wheel pushes, not its mounting position
    /// on the chassis circle. That is why [`Self::construct_jacobian`] uses
    /// `(cos, sin)` — the radial-looking form is correct for drive directions.
    /// (Mounting-position angles would require the tangential `(-sin, cos)`.)
    /// Inherited from the original OmniBot animation model.
    wheel_angles: [f32; 3],
    /// Maximum wheel angular speed (rad/s) used for normalisation
    max_wheel_rad_s: f32,
}

/// Default wheel drive-direction angles in degrees: 60°, 180°, 300°.
///
/// Evenly spaced 120° apart, matching the original OmniBot animation model and
/// every board profile to date.
pub const DEFAULT_WHEEL_ANGLES_DEG: [f32; 3] = [60.0, 180.0, 300.0];

impl EmbodiedKinematics {
    /// Instantiate with the [`DEFAULT_WHEEL_ANGLES_DEG`] layout.
    ///
    /// Note that `wheel_radius` and `robot_radius` **cancel out** of the
    /// normalised control path (see
    /// [`Self::compute_wheel_velocities`]) — they matter only for physical
    /// quantities such as [`Self::max_yaw_deg_s`],
    /// [`Self::max_linear_speed`], and encoder-to-distance odometry. The wheel
    /// *angles* are what actually determine steering, so use
    /// [`Self::new_with_angles`] if your layout differs.
    pub fn new(
        wheel_radius: f32,
        robot_radius: f32,
        max_wheel_rad_s: f32,
    ) -> Self {
        Self::new_with_angles(
            wheel_radius,
            robot_radius,
            max_wheel_rad_s,
            DEFAULT_WHEEL_ANGLES_DEG,
        )
    }

    /// Instantiate with an explicit wheel layout.
    ///
    /// `wheel_angles_deg` are **drive-direction** angles (the direction each
    /// wheel pushes), not mounting positions on the chassis circle — see the
    /// [`Self::wheel_angles`] field docs.
    ///
    /// This is the only geometry input that changes how the robot steers, which
    /// is why it is worth carrying per-board rather than hard-coding. A layout
    /// rotated to suit the chassis or the camera's forward direction is a
    /// config change, not a code change.
    ///
    /// # Examples
    ///
    /// ```
    /// use owb_core::motion::kinematics::{EmbodiedKinematics, DEFAULT_WHEEL_ANGLES_DEG};
    ///
    /// // Equivalent to `new`.
    /// let a = EmbodiedKinematics::new(0.019, 0.195, 50.0);
    /// let b = EmbodiedKinematics::new_with_angles(
    ///     0.019, 0.195, 50.0, DEFAULT_WHEEL_ANGLES_DEG,
    /// );
    /// assert_eq!(
    ///     a.compute_wheel_velocities(0.5, 30.0, 0.0, 0.2),
    ///     b.compute_wheel_velocities(0.5, 30.0, 0.0, 0.2),
    /// );
    /// ```
    pub fn new_with_angles(
        wheel_radius: f32,
        robot_radius: f32,
        max_wheel_rad_s: f32,
        wheel_angles_deg: [f32; 3],
    ) -> Self {
        Self {
            wheel_radius,
            robot_radius,
            wheel_angles: wheel_angles_deg.map(|d| d * (PI / 180.0)),
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
    /// `speed` is the translational magnitude (normalised); `angle` and
    /// `orientation` are in degrees, increasing CCW, with `angle = 0` meaning
    /// "straight ahead".
    ///
    /// # Return convention
    ///
    /// Returns `(strafe, forward)` — **not** `(vx, vy)`:
    ///
    /// ```text
    /// .0 = -speed * sin(angle - orientation)   // strafe, positive = right
    /// .1 =  speed * cos(angle - orientation)   // forward
    /// ```
    ///
    /// So `angle = 0` yields `(0, +speed)` and `angle = 90` yields
    /// `(-speed, 0)`. The pair is the natural `(cos, sin)` heading vector
    /// rotated +90°, which is why the forward component lands in `.1`.
    ///
    /// This ordering is **load-bearing, not incidental**:
    ///
    /// * [`Self::compute_wheel_velocities`] feeds `.0` into the Jacobian's
    ///   `cos(θ)` column and `.1` into its `sin(θ)` column, so with the default
    ///   [`DEFAULT_WHEEL_ANGLES_DEG`] layout a pure-forward command drives
    ///   wheels 1 and 3 in opposition and leaves wheel 2 idle
    ///   (`[+0.866, 0, -0.866]`).
    /// * The v1-alpha two-wheel drive path destructures this as
    ///   `let (_, forward) = convert_to_body_frame(..)`. Reordering the tuple to
    ///   a conventional `(vx, vy)` would silently hand that path the *strafe*
    ///   component, and a 0° forward command would drive nothing at all.
    ///
    /// The residual +90° offset between "command 0°" and the Jacobian's X axis
    /// is redundant now that wheel angles are configurable — the same result is
    /// expressible by rotating [`Self::new_with_angles`] inputs by 90°. It is
    /// kept because the board profiles' `wheel_phase_invert` flags were tuned on
    /// hardware against *this* convention. See
    /// `body_frame_puts_forward_in_second_element` for the pinning test.
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
        let mut out = [0.0f32; 3];
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
            out[i] = if norm.is_finite() { norm } else { 0.0 };
        }

        // Scale the wheel vector as a whole instead of clamping each wheel
        // independently. Truncating only the saturated wheel changes the ratios
        // between wheels, and on an omni platform those ratios *are* the
        // commanded direction — so per-wheel clamping made the robot veer off
        // the commanded heading under a combined translate+yaw, rather than
        // simply travelling slower along it.
        let peak = out.iter().fold(0.0f32, |acc, v| acc.max(v.abs()));
        if peak > 1.0 {
            let scale = 1.0 / peak;
            for v in out.iter_mut() {
                *v *= scale;
            }
        }

        for v in out.iter_mut() {
            if v.abs() < 1e-6 {
                *v = 0.0;
            }
            // Backstop against residual float error from the scaling above.
            *v = v.clamp(-1.0, 1.0);
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

    /// The normalised control path must be invariant to wheel/robot radius.
    ///
    /// `compute_wheel_velocities` scales the command by
    /// `max_linear_speed = max_wheel_rad_s * r` while the Jacobian divides by
    /// `r`, and the yaw term cancels through `l` the same way — so only the
    /// wheel *angles* survive. This is load-bearing: it means the radii are
    /// needed solely for genuinely physical quantities (deg/s yaw mapping,
    /// reported m/s, encoder-to-distance odometry), and a geometry correction
    /// cannot silently change how the robot steers.
    #[test]
    fn normalised_output_is_invariant_to_radii() {
        let cases = [(1.0, 0.0, 0.0), (0.6, 35.0, 0.3), (0.4, 200.0, -0.5)];
        for (speed, angle, omega) in cases {
            let reference = EmbodiedKinematics::new(0.019, 0.195, 50.0)
                .compute_wheel_velocities(speed, angle, 0.0, omega);
            for (r, l, w) in [(0.148, 0.195, 25.0), (1.0, 2.0, 7.0), (0.03, 0.08, 120.0)] {
                let other = EmbodiedKinematics::new(r, l, w)
                    .compute_wheel_velocities(speed, angle, 0.0, omega);
                for i in 0..3 {
                    assert!(
                        (reference[i] - other[i]).abs() < 1e-5,
                        "wheel {i} changed with geometry r={r} l={l} w={w}: \
                         {} vs {}",
                        reference[i],
                        other[i]
                    );
                }
            }
        }
    }

    /// The physical derived figures must match the measured robot.
    ///
    /// Guards the geometry correction: 38 mm Nexus omni wheel (r = 0.019 m)
    /// with the bench-measured `max_wheel_rad_s = 50` gives ~0.95 m/s, which
    /// the host-sim independently arrived at as 0.9. The previous
    /// (0.148, 25.0) pair implied 3.7 m/s and 1087 deg/s — about 3 rev/s.
    #[test]
    fn physical_limits_match_measured_hardware() {
        let kin = EmbodiedKinematics::new(0.019, 0.195, 50.0);
        let lin = kin.max_linear_speed();
        assert!(
            (lin - 0.95).abs() < 0.01,
            "max_linear_speed should be ~0.95 m/s, got {lin}"
        );
        let yaw = kin.max_yaw_deg_s();
        assert!(
            (200.0..350.0).contains(&yaw),
            "max_yaw_deg_s should be a plausible ~279 deg/s, got {yaw}"
        );
    }

    /// Regression guard: saturation must preserve the commanded direction.
    ///
    /// Clamping each wheel independently truncated only the saturated wheel,
    /// changing the ratios between wheels — which on an omni base rotates the
    /// actual direction of travel away from what was commanded.
    #[test]
    fn saturation_preserves_wheel_ratios() {
        let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
        // Full translation plus full yaw: guaranteed to push a wheel past 1.0.
        let unscaled = {
            // Reproduce the pre-scaling vector straight from the Jacobian.
            let (vx_n, vy_n) = EmbodiedKinematics::convert_to_body_frame(1.0, 35.0, 0.0);
            let vx = vx_n * kin.max_linear_speed();
            let vy = vy_n * kin.max_linear_speed();
            let omega = kin.max_yaw_rad_s();
            let j = kin.construct_jacobian();
            let mut v = [0.0f32; 3];
            for i in 0..3 {
                v[i] = (j[i][0] * vx + j[i][1] * vy + j[i][2] * omega) / 7.0;
            }
            v
        };
        let peak = unscaled.iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(
            peak > 1.0,
            "test setup must actually saturate (peak={peak})"
        );

        let wheels = kin.compute_wheel_velocities(1.0, 35.0, 0.0, 1.0);
        assert!(
            wheels.iter().all(|&v| v.abs() <= 1.0 + 1e-5),
            "output must stay within +/-1: {wheels:?}"
        );
        // At least one wheel should now sit at the limit.
        let out_peak = wheels.iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(
            (out_peak - 1.0).abs() < 1e-4,
            "peak should be 1.0: {out_peak}"
        );

        // The direction is encoded in the ratios; uniform scaling preserves them.
        for i in 0..3 {
            let expected = unscaled[i] / peak;
            assert!(
                (wheels[i] - expected).abs() < 1e-4,
                "wheel {i} ratio distorted: got {} expected {expected}",
                wheels[i]
            );
        }
    }

    /// Pins the `(strafe, forward)` return order of `convert_to_body_frame`.
    ///
    /// `board/real.rs::two_wheel_front_targets` destructures this as
    /// `let (_, forward) = ..`, so swapping the tuple to a conventional
    /// `(vx, vy)` would feed the two-wheel drive path the strafe component and
    /// a 0-degree forward command would produce no motion. If this test fails,
    /// that call site (and `wheel_phase_invert` on every profile) must be
    /// revisited in the same change -- do not just update the expectation.
    #[test]
    fn body_frame_puts_forward_in_second_element() {
        let (strafe, forward) = EmbodiedKinematics::convert_to_body_frame(1.0, 0.0, 0.0);
        assert!(
            forward > 0.99,
            "0 deg must be full forward in .1, got {forward}"
        );
        assert!(
            strafe.abs() < 1e-6,
            "0 deg must have no strafe, got {strafe}"
        );

        let (strafe, forward) = EmbodiedKinematics::convert_to_body_frame(1.0, 90.0, 0.0);
        assert!(
            strafe < -0.99,
            "90 deg must be full negative strafe in .0, got {strafe}"
        );
        assert!(
            forward.abs() < 1e-6,
            "90 deg must have no forward, got {forward}"
        );
    }

    /// Orientation is subtracted from the command angle, so holding a heading
    /// while the body is rotated keeps the world-frame direction fixed.
    #[test]
    fn body_frame_compensates_orientation() {
        let (_, forward) = EmbodiedKinematics::convert_to_body_frame(1.0, 90.0, 90.0);
        assert!(
            forward > 0.99,
            "command 90 deg with body at 90 deg is forward in body frame, got {forward}"
        );
    }

    /// The default layout's pure-forward wheel pattern. Wheel 2 idles because
    /// its drive direction (180 deg) is perpendicular to the Jacobian axis that
    /// forward lands on. Guards against a silent 90-degree convention shift.
    #[test]
    fn forward_command_matches_default_layout_pattern() {
        let kin = EmbodiedKinematics::new(0.019, 0.195, 50.0);
        let wheels = kin.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);
        assert!(
            (wheels[0] - 0.866).abs() < 0.01,
            "wheel 1 expected +0.866, got {}",
            wheels[0]
        );
        assert!(
            wheels[1].abs() < 1e-6,
            "wheel 2 expected idle, got {}",
            wheels[1]
        );
        assert!(
            (wheels[2] + 0.866).abs() < 0.01,
            "wheel 3 expected -0.866, got {}",
            wheels[2]
        );
    }

    /// A 90-degree strafe redistributes effort onto wheel 2 -- the complement
    /// of the forward pattern above.
    #[test]
    fn strafe_command_matches_default_layout_pattern() {
        let kin = EmbodiedKinematics::new(0.019, 0.195, 50.0);
        let wheels = kin.compute_wheel_velocities(1.0, 90.0, 0.0, 0.0);
        assert!((wheels[0] + 0.5).abs() < 0.01, "got {}", wheels[0]);
        assert!((wheels[1] - 1.0).abs() < 0.01, "got {}", wheels[1]);
        assert!((wheels[2] + 0.5).abs() < 0.01, "got {}", wheels[2]);
    }
}
