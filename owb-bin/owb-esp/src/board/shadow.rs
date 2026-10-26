use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use log::{debug, info};
use owb_core::{
    ctrl::{
        sysctrl::{EncoderData, EnvData, I2CCommand, PowerData},
        CommandExecutor,
        CommandReply,
        ImuData,
        SystemCommand,
    },
    platform::BoardRuntimeKind,
};

static SHADOW_ENABLED: AtomicBool = AtomicBool::new(true);
static SHADOW_LED: AtomicU8 = AtomicU8::new(0);
static LAST_DIR_DEG: AtomicU32 = AtomicU32::new(0f32.to_bits());
static LAST_SPEED_NORM: AtomicU32 = AtomicU32::new(0f32.to_bits());
static LAST_YAW_DEG_S: AtomicU32 = AtomicU32::new(0f32.to_bits());

pub struct ShadowBoardRuntime {
    runtime: BoardRuntimeKind,
}

impl ShadowBoardRuntime {
    pub const fn new(runtime: BoardRuntimeKind) -> Self { Self { runtime } }

    fn log_shadow_motion(
        &self,
        direction_deg: f32,
        speed_norm: f32,
        yaw_rate_deg_s: f32,
    ) {
        LAST_DIR_DEG.store(direction_deg.to_bits(), Ordering::Relaxed);
        LAST_SPEED_NORM.store(speed_norm.to_bits(), Ordering::Relaxed);
        LAST_YAW_DEG_S.store(yaw_rate_deg_s.to_bits(), Ordering::Relaxed);
        debug!(
            "shadow runtime {:?}: dir={:.1} speed={:.2} yaw={:.1}",
            self.runtime, direction_deg, speed_norm, yaw_rate_deg_s
        );
    }
}

impl CommandExecutor for ShadowBoardRuntime {
    type Error = ();

    fn execute_command(
        &mut self,
        cmd: SystemCommand,
    ) -> Result<CommandReply, Self::Error> {
        match cmd {
            SystemCommand::I(ic) => match ic {
                I2CCommand::T {
                    direction_deg,
                    speed_norm,
                } => {
                    let yaw_rate = f32::from_bits(LAST_YAW_DEG_S.load(Ordering::Relaxed));
                    self.log_shadow_motion(direction_deg, speed_norm, yaw_rate);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Y {
                    yaw_rate_deg_s,
                    orientation_deg,
                } => {
                    let direction = f32::from_bits(LAST_DIR_DEG.load(Ordering::Relaxed));
                    info!(
                        "shadow runtime {:?}: yaw={:.1} orientation={:?}",
                        self.runtime, yaw_rate_deg_s, orientation_deg
                    );
                    let speed = f32::from_bits(LAST_SPEED_NORM.load(Ordering::Relaxed));
                    self.log_shadow_motion(direction, speed, yaw_rate_deg_s);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::O {
                    direction_deg,
                    speed_norm,
                    yaw_rate_deg_s,
                    orientation_deg,
                } => {
                    info!(
                        "shadow runtime {:?}: omni orientation={:?}",
                        self.runtime, orientation_deg
                    );
                    self.log_shadow_motion(direction_deg, speed_norm, yaw_rate_deg_s);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Probe { wheel, forward } => {
                    SHADOW_ENABLED.store(true, Ordering::Relaxed);
                    info!(
                        "shadow runtime {:?}: probe wheel={} forward={}",
                        self.runtime, wheel, forward
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::ProbeSpeed { wheel, speed_norm } => {
                    SHADOW_ENABLED.store(true, Ordering::Relaxed);
                    info!(
                        "shadow runtime {:?}: probe_speed wheel={} speed_norm={:.2}",
                        self.runtime, wheel, speed_norm
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::PidProbeSpeed { wheel, speed_norm } => {
                    SHADOW_ENABLED.store(true, Ordering::Relaxed);
                    info!(
                        "shadow runtime {:?}: pid_probe_speed wheel={} speed_norm={:.2}",
                        self.runtime, wheel, speed_norm
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::RawBridge {
                    wheel,
                    phase_gpio_high,
                    drive_gpio_high,
                } => {
                    SHADOW_ENABLED.store(true, Ordering::Relaxed);
                    info!(
                        "shadow runtime {:?}: raw_bridge wheel={} phase_gpio_high={} drive_gpio_high={}",
                        self.runtime, wheel, phase_gpio_high, drive_gpio_high
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::IndicatorLed { index, on } => {
                    info!(
                        "shadow runtime {:?}: indicator_led index={} on={}",
                        self.runtime, index, on
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::WheelTrim { w1, w2, w3 } => {
                    info!(
                        "shadow runtime {:?}: wheel_trim w1={:.3} w2={:.3} w3={:.3} (no-op)",
                        self.runtime, w1, w2, w3
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::HeadingHold { enable, kp } => {
                    info!(
                        "shadow runtime {:?}: heading_hold enable={} kp={:.4} (no-op)",
                        self.runtime, enable, kp
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::StictionKick { ms, boost } => {
                    info!(
                        "shadow runtime {:?}: stiction_kick ms={} boost={:.3} (no-op)",
                        self.runtime, ms, boost
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::DriveLoop {
                    closed_loop,
                    pwm_stagger,
                    trace,
                    max_duty,
                } => {
                    info!(
                        "shadow runtime {:?}: drive_loop closed_loop={:?} pwm_stagger={:?} trace={:?} max_duty={:?} (no-op)",
                        self.runtime, closed_loop, pwm_stagger, trace, max_duty
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Enable => {
                    SHADOW_ENABLED.store(true, Ordering::Relaxed);
                    info!("shadow runtime {:?}: enable", self.runtime);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Disable => {
                    SHADOW_ENABLED.store(false, Ordering::Relaxed);
                    self.log_shadow_motion(
                        f32::from_bits(LAST_DIR_DEG.load(Ordering::Relaxed)),
                        0.0,
                        0.0,
                    );
                    info!("shadow runtime {:?}: disable", self.runtime);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::ReadIMU => Ok(CommandReply::ImuData(ImuData::default())),
                I2CCommand::ReadEnv => {
                    info!("shadow runtime {:?}: env default", self.runtime);
                    Ok(CommandReply::EnvData(EnvData::default()))
                }
                I2CCommand::ReadPower => {
                    info!("shadow runtime {:?}: power default", self.runtime);
                    Ok(CommandReply::PowerData(PowerData::default()))
                }
                I2CCommand::ReadEncoders => {
                    info!("shadow runtime {:?}: encoders default", self.runtime);
                    Ok(CommandReply::EncoderData(EncoderData::default()))
                }
                I2CCommand::Snap => {
                    #[cfg(not(feature = "no-camera"))]
                    crate::camera::snap();
                    #[cfg(feature = "no-camera")]
                    esp_println::println!("[SNAP] error: no-camera build");
                    Ok(CommandReply::Ok)
                }
            },
            SystemCommand::L(lc) => {
                let brightness = match lc {
                    owb_core::ctrl::sysctrl::LEDCommand::On => 255,
                    owb_core::ctrl::sysctrl::LEDCommand::Off => 0,
                    owb_core::ctrl::sysctrl::LEDCommand::SC { r, g, b } => r.max(g).max(b),
                    owb_core::ctrl::sysctrl::LEDCommand::Brightness { v } => v,
                    owb_core::ctrl::sysctrl::LEDCommand::Breath { r, g, b } => r.max(g).max(b),
                    owb_core::ctrl::sysctrl::LEDCommand::Wheel => 255,
                };
                SHADOW_LED.store(brightness, Ordering::Relaxed);
                info!("shadow runtime {:?}: led={}", self.runtime, brightness);
                Ok(CommandReply::Ok)
            }
            SystemCommand::P(pc) => {
                match pc {
                    owb_core::ctrl::sysctrl::PowerCommand::MotorsToggle => {
                        let next = !SHADOW_ENABLED.load(Ordering::Relaxed);
                        SHADOW_ENABLED.store(next, Ordering::Relaxed);
                        info!("shadow runtime {:?}: motors_toggle -> {next}", self.runtime);
                    }
                    owb_core::ctrl::sysctrl::PowerCommand::DozeToggle => {
                        info!("shadow runtime {:?}: doze_toggle", self.runtime);
                    }
                    owb_core::ctrl::sysctrl::PowerCommand::CameraToggle => {
                        let enabled = crate::camera::toggle_device_enabled();
                        info!(
                            "shadow runtime {:?}: camera_toggle -> {enabled}",
                            self.runtime
                        );
                    }
                    owb_core::ctrl::sysctrl::PowerCommand::Estop => {
                        SHADOW_ENABLED.store(false, Ordering::Relaxed);
                        self.log_shadow_motion(
                            f32::from_bits(LAST_DIR_DEG.load(Ordering::Relaxed)),
                            0.0,
                            0.0,
                        );
                        info!("shadow runtime {:?}: estop", self.runtime);
                    }
                }
                Ok(CommandReply::Ok)
            }
            SystemCommand::A(ac) => {
                info!("shadow runtime {:?}: autonomy={ac:?}", self.runtime);
                Ok(CommandReply::Ok)
            }
        }
    }
}
