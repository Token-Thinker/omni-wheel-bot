//! Core robotics building blocks for Omni-Wheel Bot firmware.
//!
//! The crate keeps the reusable robot pieces in a ROS-shaped layout without
//! requiring ROS at runtime:
//!
//! - `ctrl`: command execution and hardware controller traits
//! - `drivers`: onboard sensor and peripheral drivers
//! - `motion`: kinematics, PID, encoders, and drive planning
//! - `net`: HTTP server, WebSocket, DHCP, auth, and protocol parsing
//! - `node`: command bus, routing, telemetry, and runtime orchestration
//!
//! The `mk_static!` macro simplifies static initialization in no-std contexts.

#![cfg_attr(not(feature = "std"), no_std)]
#![recursion_limit = "256"]

pub mod ext {
    pub use core::cell::RefCell;

    #[cfg(feature = "net")]
    pub use embassy_net as net;
    #[cfg(feature = "net")]
    pub use embassy_time as time;
    pub use embedded_hal as hal;
    pub use static_cell;

    // Convenience re-exports for frequent usage
    #[allow(non_snake_case)]
    pub mod prelude {
        pub use core::cell::RefCell;

        pub use static_cell::StaticCell;

        pub use crate::ext::hal;
        #[cfg(feature = "net")]
        pub use crate::ext::net::{
            self,
            Config as NetConfig,
            Ipv4Address,
            Ipv4Cidr,
            Stack,
            StackResources,
            StaticConfigV4,
        };
        #[cfg(feature = "net")]
        pub use crate::ext::time::{Duration, Timer};
    }
}

pub mod ctrl;
pub mod drivers;
pub mod motion;
pub mod msg;
pub mod node;
pub mod platform;
#[cfg(feature = "net")]
pub mod runtime;
// `net` hosts both the full networking stack (under `net`) and the wire
// protocol (under `serde`), so it is available whenever either is enabled. The
// individual submodules are feature-gated inside `net::mod`.
#[cfg(any(feature = "net", feature = "serde"))]
pub mod net;
// Make protocol available without the full `net` stack for host-side tools.
#[cfg(feature = "serde")]
pub mod protocol {
    pub use crate::net::protocol::*;
}
#[cfg(feature = "sim")]
pub mod sim;

/// Back-compatibility alias for the pre-0.2.0 `controllers` module, which was
/// renamed to [`ctrl`]. Lets existing `owb_core::controllers::sysctrl::…`
/// paths keep resolving. New code should use [`ctrl`] directly.
#[deprecated(
    since = "0.2.0",
    note = "renamed to `ctrl`; use `owb_core::ctrl` instead"
)]
pub use ctrl as controllers;
pub use motion::kinematics::EmbodiedKinematics as ek;

#[macro_export]
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        STATIC_CELL.uninit().write($val)
    }};
}
