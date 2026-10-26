//! Shared omni-wheel actuation planning for board runtimes.
//!
//! This keeps the "command -> wheel intent -> GPIO/PWM intent" mapping in core
//! so embedded targets can reuse the same motion planning even when the final
//! electrical interface differs.

use crate::motion::kinematics::EmbodiedKinematics;

/// Shared scalar PID gains for the three-wheel drive loop.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

/// Drivetrain geometry and control-loop tuning for a board profile.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveTuning {
    /// Wheel radius (m). Cancels out of the normalised control path — it only
    /// affects physical quantities (reported m/s, encoder-to-distance
    /// odometry, and the deg/s yaw mapping via `robot_radius_m`).
    pub wheel_radius_m: f32,
    /// Centre-to-wheel distance (m). Also cancels out of the normalised path;
    /// it sets absolute yaw authority.
    pub robot_radius_m: f32,
    pub max_wheel_rad_s: f32,
    /// Wheel **drive-direction** angles in degrees.
    ///
    /// Unlike the two radii above, this genuinely determines how the robot
    /// steers, so it belongs per-board. Defaults to
    /// [`crate::motion::kinematics::DEFAULT_WHEEL_ANGLES_DEG`] (60/180/300).
    pub wheel_angles_deg: [f32; 3],
    pub control_dt_ms: u16,
    pub motor_deadband_norm: f32,
    pub counts_per_wheel_revolution: u16,
    pub pid: PidGains,
}

/// Per-wheel half-bridge intent for simple direction + drive runtimes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HalfBridgeOutput {
    pub phase_high: bool,
    pub drive_high: bool,
}

impl HalfBridgeOutput {
    pub const fn stop() -> Self {
        Self {
            phase_high: false,
            drive_high: false,
        }
    }
}

/// High-level drive plan for three motor outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriWheelDrivePlan {
    pub enabled: bool,
    pub wheels: [HalfBridgeOutput; 3],
}

impl TriWheelDrivePlan {
    pub const fn stopped(enabled: bool) -> Self {
        Self {
            enabled,
            wheels: [
                HalfBridgeOutput::stop(),
                HalfBridgeOutput::stop(),
                HalfBridgeOutput::stop(),
            ],
        }
    }
}

/// Convert a signed normalised wheel speed into a simple half-bridge command.
pub fn half_bridge_from_speed(
    speed_norm: f32,
    deadband: f32,
) -> HalfBridgeOutput {
    let speed = if speed_norm.is_finite() {
        speed_norm.clamp(-1.0, 1.0)
    }
    else {
        0.0
    };

    if speed.abs() <= deadband.max(0.0) {
        HalfBridgeOutput::stop()
    }
    else {
        HalfBridgeOutput {
            phase_high: speed.is_sign_positive(),
            drive_high: true,
        }
    }
}

/// Build a simple direction/drive plan from a full omni-wheel motion request.
pub fn drive_plan_from_motion(
    kinematics: &EmbodiedKinematics,
    enabled: bool,
    speed_norm: f32,
    direction_deg: f32,
    orientation_deg: f32,
    yaw_norm: f32,
    deadband: f32,
) -> TriWheelDrivePlan {
    if !enabled {
        return TriWheelDrivePlan::stopped(false);
    }

    let wheel_speeds =
        kinematics.compute_wheel_velocities(speed_norm, direction_deg, orientation_deg, yaw_norm);

    TriWheelDrivePlan {
        enabled: true,
        wheels: wheel_speeds.map(|speed| half_bridge_from_speed(speed, deadband)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn half_bridge_stops_inside_deadband() {
        assert_eq!(half_bridge_from_speed(0.03, 0.05), HalfBridgeOutput::stop());
    }

    #[test]
    fn half_bridge_sets_forward_direction() {
        assert_eq!(
            half_bridge_from_speed(0.4, 0.05),
            HalfBridgeOutput {
                phase_high: true,
                drive_high: true,
            }
        );
    }

    #[test]
    fn half_bridge_sets_reverse_direction() {
        assert_eq!(
            half_bridge_from_speed(-0.4, 0.05),
            HalfBridgeOutput {
                phase_high: false,
                drive_high: true,
            }
        );
    }

    #[test]
    fn disabled_drive_plan_stops_everything() {
        let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
        let plan = drive_plan_from_motion(&kin, false, 1.0, 0.0, 0.0, 0.0, 0.05);
        assert_eq!(plan, TriWheelDrivePlan::stopped(false));
    }

    #[test]
    fn pure_yaw_drive_plan_drives_all_wheels() {
        let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
        let plan = drive_plan_from_motion(&kin, true, 0.0, 0.0, 0.0, 0.4, 0.05);
        assert!(plan.enabled);
        assert!(plan.wheels.iter().all(|wheel| wheel.drive_high));
    }
}
