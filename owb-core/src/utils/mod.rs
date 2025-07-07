pub mod connection;
pub mod controllers;
pub(crate) mod frontend;
mod math;

pub use controllers::SystemController;
pub use embassy_time::*;
pub use connection::server::run as wss;
pub use math::kinematics::EmbodiedKinematics as ek;




#[macro_export]
// Static memory allocation macro
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
