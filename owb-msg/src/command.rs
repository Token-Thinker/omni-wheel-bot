//! Normal robot commands.
//!
//! These are the commands an operator app should prefer. Bench/HIL controls
//! stay in `dev` so they do not become part of the normal driving API.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub use crate::legacy::{AutonomyCommand, CommandReply, LEDCommand, PowerCommand, SystemCommand};

/// User-facing velocity command in normalized robot space.
///
/// `linear_norm` is forward/back drive intent in `[-1.0, 1.0]`.
/// `angular_norm` is yaw intent in `[-1.0, 1.0]`.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DriveCommand {
    pub linear_norm: f32,
    pub angular_norm: f32,
}

impl DriveCommand {
    pub const fn stopped() -> Self {
        Self {
            linear_norm: 0.0,
            angular_norm: 0.0,
        }
    }

    pub fn clamped(self) -> Self {
        Self {
            linear_norm: clamp_norm(self.linear_norm),
            angular_norm: clamp_norm(self.angular_norm),
        }
    }
}

/// Canonical app-level robot command.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "cmd", rename_all = "snake_case"))]
pub enum RobotCommand {
    Drive(DriveCommand),
    Stop,
    Enable,
    Disable,
    ReadTelemetry,
}

impl RobotCommand {
    pub fn as_drive(self) -> Option<DriveCommand> {
        match self {
            Self::Drive(cmd) => Some(cmd.clamped()),
            Self::Stop => Some(DriveCommand::stopped()),
            Self::Enable | Self::Disable | Self::ReadTelemetry => None,
        }
    }
}

fn clamp_norm(value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(-1.0, 1.0)
    }
    else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drive_command_clamps_and_drops_nan() {
        let cmd = DriveCommand {
            linear_norm: 2.0,
            angular_norm: f32::NAN,
        }
        .clamped();

        assert_eq!(cmd.linear_norm, 1.0);
        assert_eq!(cmd.angular_norm, 0.0);
    }
}
