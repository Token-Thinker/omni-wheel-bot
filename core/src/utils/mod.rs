//! Core Modules
//!
//! This file defines core modules essential for the system's functionality.
//!
//! # Modules
//! - `connection`: Manages network connections and communication protocols.
//! - `controllers`: Handles hardware control logic, including motor and sensor
//! - `frontend`: Handles user interface and user input
//! - `packages`: Handles communication packages and their management.

/// Module for managing network connections and communication protocols.
pub mod connection;
/// Module for handling hardware controllers, such as motors and sensors.
pub mod controllers;
/// Module for handling user interface and user input.
pub(crate) mod frontend;
/// Module for handling communication packages and their management.
pub mod packages;

pub use controllers::SystemController;
pub use packages::smart_leds;

#[macro_export]
// Static memory allocation macro
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
