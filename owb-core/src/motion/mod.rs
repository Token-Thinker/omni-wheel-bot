//! Motion-domain helpers.
//!
//! The ROS-shaped home for drive math and control loops: omni-wheel and
//! differential kinematics, the PID stack, encoder decoding, and drive
//! planning.

pub mod diff_drive;
pub mod encoder;
pub mod kinematics;
pub mod omni;
pub mod pid;

pub use diff_drive::{DiffDriveCommand, DiffDrivePlan};
pub use encoder::{QuadratureDecoder, VelocityEstimator};
pub use kinematics::EmbodiedKinematics;
pub use omni::{
    drive_plan_from_motion,
    half_bridge_from_speed,
    DriveTuning,
    HalfBridgeOutput,
    PidGains,
    TriWheelDrivePlan,
};
pub use pid::{PidController, TriWheelPidController, WheelPidTerms};
