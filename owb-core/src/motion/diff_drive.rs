//! Differential-drive helpers for the two-wheel fallback/runtime shape.

use crate::msg::DriveCommand;

/// Normalized two-wheel command.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DiffDriveCommand {
    pub left_norm: f32,
    pub right_norm: f32,
}

/// Simple two-wheel drive plan.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DiffDrivePlan {
    pub enabled: bool,
    pub wheels: DiffDriveCommand,
}

impl DiffDrivePlan {
    pub const fn stopped(enabled: bool) -> Self {
        Self {
            enabled,
            wheels: DiffDriveCommand {
                left_norm: 0.0,
                right_norm: 0.0,
            },
        }
    }

    pub fn from_drive_command(
        enabled: bool,
        command: DriveCommand,
        deadband: f32,
    ) -> Self {
        if !enabled {
            return Self::stopped(false);
        }

        let command = command.clamped();
        let left = apply_deadband(command.linear_norm - command.angular_norm, deadband);
        let right = apply_deadband(command.linear_norm + command.angular_norm, deadband);

        Self {
            enabled: true,
            wheels: DiffDriveCommand {
                left_norm: left,
                right_norm: right,
            },
        }
    }
}

fn apply_deadband(
    value: f32,
    deadband: f32,
) -> f32 {
    let value = if value.is_finite() {
        value.clamp(-1.0, 1.0)
    }
    else {
        0.0
    };

    if value.abs() <= deadband.max(0.0) {
        0.0
    }
    else {
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diff_drive_mixes_forward_and_yaw() {
        let plan = DiffDrivePlan::from_drive_command(
            true,
            DriveCommand {
                linear_norm: 0.5,
                angular_norm: 0.25,
            },
            0.05,
        );

        assert_eq!(plan.wheels.left_norm, 0.25);
        assert_eq!(plan.wheels.right_norm, 0.75);
    }

    #[test]
    fn diff_drive_stops_when_disabled() {
        let plan = DiffDrivePlan::from_drive_command(
            false,
            DriveCommand {
                linear_norm: 1.0,
                angular_norm: 1.0,
            },
            0.0,
        );

        assert_eq!(plan, DiffDrivePlan::stopped(false));
    }
}
