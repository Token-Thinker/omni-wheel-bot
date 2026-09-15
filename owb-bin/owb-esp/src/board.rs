#![cfg(feature = "esp32s3")]

#[path = "board/bridges.rs"]
mod bridges;
#[path = "board/capabilities.rs"]
mod capabilities;
#[path = "board/diagnostics.rs"]
mod diagnostics;
#[path = "board/encoders.rs"]
mod encoders;
#[path = "board/executor.rs"]
mod executor;
#[path = "board/leds.rs"]
mod leds;
#[path = "board/probe.rs"]
mod probe;
#[path = "board/real.rs"]
mod real;
#[path = "board/sensors.rs"]
mod sensors;
// Only compiled when the real GPIO executors are off — otherwise the whole
// shadow runtime is unreachable and every item in it reports as dead code.
#[cfg(not(feature = "real-executors"))]
#[path = "board/shadow.rs"]
mod shadow;
#[cfg(feature = "real-executors")]
use bridges::{GpioBridge, MotorBridge, PhasePwmBridge, PwmBridge};
pub use capabilities::BoardCapabilities;
#[cfg(feature = "real-executors")]
use encoders::PcntEncoderCapture;
#[cfg(feature = "real-executors")]
use leds::{AddressableLedDriver, GpioStatusLed, RgbColor};
pub use probe::boot_probe;
#[cfg(feature = "real-executors")]
pub use real::RealBoardRuntime;
#[cfg(feature = "real-executors")]
use sensors::{
    OnboardHdc2010Sensor,
    OnboardLis2dh12Imu,
    OnboardMax17048Gauge,
    OnboardTca9555Indicators,
};
#[cfg(not(feature = "real-executors"))]
pub use shadow::ShadowBoardRuntime;
