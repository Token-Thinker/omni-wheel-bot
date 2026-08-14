//! Runtime node boundaries.
//!
//! Home for the command bus, board profiles and capabilities, and the
//! diagnostics/health payloads.

#[cfg(feature = "net")]
pub mod command_bus;

#[cfg(feature = "net")]
pub use command_bus::{
    command_bus_metrics_snapshot,
    note_command_queue_full_drop,
    run_command_bus,
    spawn_command_sink,
    CommandBusMetricsSnapshot,
};
