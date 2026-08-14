//! Telemetry contracts shared by transports, host simulation, and board runtimes.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

pub use crate::legacy::{EncoderData, EnvData, ImuData, PowerData};

/// Runtime status payload that can be logged or sent over a transport.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DiagnosticTelemetry {
    pub enabled: bool,
    pub faulted: bool,
    pub command_count: u32,
}

/// Compact snapshot for the normal app/control surface.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TelemetrySnapshot {
    pub encoders: EncoderData,
    pub power: PowerData,
    pub diagnostics: DiagnosticTelemetry,
}
