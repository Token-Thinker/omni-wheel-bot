//! Message contracts used by command sources, runtime nodes, and board adapters.
//!
//! This module is the canonical place for robot-facing command and telemetry
//! names. The existing `controllers::sysctrl` types remain available for
//! compatibility while this surface becomes the easier API for app authors.

pub mod command;
pub mod dev;
pub mod legacy;
pub mod telemetry;

pub use command::{DriveCommand, RobotCommand};
pub use dev::{DevCommand, ProbeCommand, ProbeSpeedCommand, RawBridgeCommand};
pub use legacy::{
    AutonomyCommand,
    CommandReply,
    EncoderData,
    EnvData,
    I2CCommand,
    ImuData,
    LEDCommand,
    PowerCommand,
    PowerData,
    SystemCommand,
};
pub use telemetry::{DiagnosticTelemetry, TelemetrySnapshot};
