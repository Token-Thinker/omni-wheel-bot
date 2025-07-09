//! Utility re-exports and helper macros for the Omni-Wheel Bot.
//!
//! This module re-exports core components, timing, kinematics, and connection
//! controllers, and provides helper macros and embedded web assets:
//!
//! - `connection`: WebSocket server and message handling
//! - `controllers`: I2C and LED controllers for robotics hardware
//! - `math`: kinematics calculations for omni-wheel motion
//! - `frontend`: compressed HTML/CSS/JS assets for the web UI
//!
//! The `mk_static!` macro simplifies static initialization in no-std contexts.

pub mod connection;
pub mod controllers;
pub(crate) mod frontend;
pub mod math;

pub use connection::server::run as wss;
pub use controllers::SystemController;
pub use embassy_time::*;
pub use math::kinematics::EmbodiedKinematics as ek;

#[macro_export]
/// Initialize a no-std static cell and write the given value into it.
///
/// This macro creates a `static_cell::StaticCell` for type `$t` and initializes
/// it with `$val`, returning a mutable reference to the stored value.
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit().write($val)
    }};
}
