//! Development and HIL-only command contracts.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub use crate::legacy::I2CCommand as LegacyI2CCommand;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProbeCommand {
    pub wheel: u8,
    pub forward: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProbeSpeedCommand {
    pub wheel: u8,
    pub speed_norm: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RawBridgeCommand {
    pub wheel: u8,
    pub phase_gpio_high: bool,
    pub drive_gpio_high: bool,
}

/// Commands kept for bench bring-up, HIL, and hardware isolation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "dev_cmd", rename_all = "snake_case"))]
pub enum DevCommand {
    Probe(ProbeCommand),
    ProbeSpeed(ProbeSpeedCommand),
    PidProbeSpeed(ProbeSpeedCommand),
    RawBridge(RawBridgeCommand),
    IndicatorLed { index: u8, on: bool },
    WheelTrim { w1: f32, w2: f32, w3: f32 },
    StictionKick { ms: u16, boost: f32 },
}

impl From<DevCommand> for LegacyI2CCommand {
    fn from(cmd: DevCommand) -> Self {
        match cmd {
            DevCommand::Probe(cmd) => Self::Probe {
                wheel: cmd.wheel,
                forward: cmd.forward,
            },
            DevCommand::ProbeSpeed(cmd) => Self::ProbeSpeed {
                wheel: cmd.wheel,
                speed_norm: cmd.speed_norm,
            },
            DevCommand::PidProbeSpeed(cmd) => Self::PidProbeSpeed {
                wheel: cmd.wheel,
                speed_norm: cmd.speed_norm,
            },
            DevCommand::RawBridge(cmd) => Self::RawBridge {
                wheel: cmd.wheel,
                phase_gpio_high: cmd.phase_gpio_high,
                drive_gpio_high: cmd.drive_gpio_high,
            },
            DevCommand::IndicatorLed { index, on } => Self::IndicatorLed { index, on },
            DevCommand::WheelTrim { w1, w2, w3 } => Self::WheelTrim { w1, w2, w3 },
            DevCommand::StictionKick { ms, boost } => Self::StictionKick { ms, boost },
        }
    }
}
