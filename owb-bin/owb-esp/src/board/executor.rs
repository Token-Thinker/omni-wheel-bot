#[cfg(feature = "real-executors")]
use log::info;
#[cfg(feature = "real-executors")]
use owb_core::ctrl::{
    sysctrl::{I2CCommand, LEDCommand, PowerCommand},
    CommandExecutor,
    CommandReply,
    SystemCommand,
};

#[cfg(feature = "real-executors")]
use super::{PcntEncoderCapture, RealBoardRuntime, RgbColor};

#[cfg(feature = "real-executors")]
impl CommandExecutor for RealBoardRuntime {
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
                    self.apply_translation(direction_deg, speed_norm);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Y {
                    yaw_rate_deg_s,
                    orientation_deg,
                } => {
                    self.apply_yaw(yaw_rate_deg_s, orientation_deg);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::O {
                    direction_deg,
                    speed_norm,
                    yaw_rate_deg_s,
                    orientation_deg,
                } => {
                    self.apply_motion(
                        direction_deg,
                        speed_norm,
                        yaw_rate_deg_s,
                        orientation_deg.unwrap_or(self.last_orientation_deg),
                    );
                    Ok(CommandReply::Ok)
                }
                I2CCommand::ReadIMU => {
                    let data = self.imu.as_mut().map(|imu| imu.read()).unwrap_or_default();
                    info!(
                        "real runtime {:?}: imu ax={:.3}g ay={:.3}g az={:.3}g gx={:.1} gy={:.1} gz={:.1}",
                        self.runtime,
                        data.accel.0, data.accel.1, data.accel.2,
                        data.gyro.0, data.gyro.1, data.gyro.2
                    );
                    Ok(CommandReply::ImuData(data))
                }
                I2CCommand::Snap => {
                    #[cfg(not(feature = "no-camera"))]
                    crate::camera::snap();
                    #[cfg(feature = "no-camera")]
                    esp_println::println!("[SNAP] error: no-camera build");
                    Ok(CommandReply::Ok)
                }
                I2CCommand::ReadEnv => {
                    let data = self.env.as_ref().map(|env| env.read()).unwrap_or_default();
                    info!(
                        "real runtime {:?}: env temp={:.1}C humidity={:.1}%",
                        self.runtime, data.temp_c, data.humidity_percent
                    );
                    Ok(CommandReply::EnvData(data))
                }
                I2CCommand::ReadPower => {
                    let data = self
                        .power
                        .as_ref()
                        .map(|power| power.read())
                        .unwrap_or_default();
                    info!(
                        "real runtime {:?}: power battery={}mV soc={:.1}%",
                        self.runtime, data.battery_mv, data.soc_percent
                    );
                    Ok(CommandReply::PowerData(data))
                }
                I2CCommand::ReadEncoders => {
                    let data = self.read_encoder_snapshot();
                    let raw_counts = self
                        .encoders
                        .as_ref()
                        .map(PcntEncoderCapture::raw_counts)
                        .unwrap_or([0; 3]);
                    let raw_levels = self
                        .encoders
                        .as_ref()
                        .map(PcntEncoderCapture::raw_levels)
                        .unwrap_or([false; 6]);
                    info!(
                        "real runtime {:?}: encoders counts=[{},{},{}] delta=[{},{},{}] raw_counts=[{},{},{}] raw_levels=[({},{}) ({},{}) ({},{})]",
                        self.runtime,
                        data.counts[0],
                        data.counts[1],
                        data.counts[2],
                        data.delta_counts[0],
                        data.delta_counts[1],
                        data.delta_counts[2],
                        raw_counts[0],
                        raw_counts[1],
                        raw_counts[2],
                        raw_levels[0],
                        raw_levels[1],
                        raw_levels[2],
                        raw_levels[3],
                        raw_levels[4],
                        raw_levels[5]
                    );
                    Ok(CommandReply::EncoderData(data))
                }
                I2CCommand::Probe { wheel, forward } => {
                    self.apply_probe(wheel, forward);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::ProbeSpeed { wheel, speed_norm } => {
                    self.apply_probe_speed(wheel, speed_norm);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::PidProbeSpeed { wheel, speed_norm } => {
                    self.apply_pid_probe_speed(wheel, speed_norm);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::RawBridge {
                    wheel,
                    phase_gpio_high,
                    drive_gpio_high,
                } => {
                    self.apply_raw_bridge(wheel, phase_gpio_high, drive_gpio_high);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::IndicatorLed { index, on } => {
                    self.set_indicator_led(index, on);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::WheelTrim { w1, w2, w3 } => {
                    self.set_wheel_trim(w1, w2, w3);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::HeadingHold { enable, kp } => {
                    self.set_heading_hold(enable, kp);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::StictionKick { ms, boost } => {
                    // Clamp to executor-safe ranges before reaching the runtime
                    // so a bad command can't park PWM at full boost forever.
                    let ms = ms.min(1000);
                    let boost = boost.clamp(0.0, 0.5);
                    self.set_stiction_kick(ms, boost);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::DriveLoop {
                    closed_loop,
                    pwm_stagger,
                    trace,
                    max_duty,
                } => {
                    self.set_drive_loop(closed_loop, pwm_stagger, trace, max_duty);
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Enable => {
                    self.set_enabled(true);
                    if let Some(imu) = self.imu.as_mut() {
                        imu.set_enabled(true);
                    }
                    Ok(CommandReply::Ok)
                }
                I2CCommand::Disable => {
                    self.set_enabled(false);
                    if let Some(imu) = self.imu.as_mut() {
                        imu.set_enabled(false);
                    }
                    Ok(CommandReply::Ok)
                }
            },
            SystemCommand::L(lc) => {
                use super::leds::LedEffect;
                // Solid commands clear any running effect first; the animated
                // ones install an effect that the background tick advances.
                match lc {
                    LEDCommand::On => {
                        self.set_led_effect(LedEffect::None);
                        self.set_led_color(RgbColor {
                            r: 255,
                            g: 255,
                            b: 255,
                        });
                    }
                    LEDCommand::Off => {
                        self.set_led_effect(LedEffect::None);
                        self.set_led_color(RgbColor { r: 0, g: 0, b: 0 });
                    }
                    LEDCommand::SC { r, g, b } => {
                        self.set_led_effect(LedEffect::None);
                        self.set_led_color(RgbColor { r, g, b });
                    }
                    LEDCommand::Brightness { v } => {
                        self.set_led_effect(LedEffect::None);
                        self.set_led_color(RgbColor { r: v, g: v, b: v });
                    }
                    LEDCommand::Breath { r, g, b } => {
                        self.set_led_effect(LedEffect::Breath { r, g, b });
                    }
                    LEDCommand::Wheel => {
                        self.set_led_effect(LedEffect::Wheel);
                    }
                }
                Ok(CommandReply::Ok)
            }
            SystemCommand::P(pc) => {
                match pc {
                    PowerCommand::MotorsToggle => self.set_enabled(!self.enabled),
                    PowerCommand::DozeToggle => {
                        if let Some(imu) = self.imu.as_mut() {
                            let next = !imu.is_enabled();
                            imu.set_enabled(next);
                            info!("real runtime {:?}: doze_toggle imu={next}", self.runtime);
                        }
                        info!("real runtime {:?}: doze_toggle", self.runtime);
                    }
                    PowerCommand::CameraToggle => {
                        let enabled = crate::camera::toggle_device_enabled();
                        info!(
                            "real runtime {:?}: camera_toggle -> {enabled}",
                            self.runtime
                        );
                    }
                    PowerCommand::Estop => {
                        self.set_enabled(false);
                        self.apply_motion(0.0, 0.0, 0.0, self.last_orientation_deg);
                    }
                }
                Ok(CommandReply::Ok)
            }
            SystemCommand::A(ac) => {
                info!("real runtime {:?}: autonomy={ac:?}", self.runtime);
                Ok(CommandReply::Ok)
            }
        }
    }

    fn background_poll_interval_ms(&self) -> Option<u64> {
        let effect_active = !matches!(self.led_effect, super::leds::LedEffect::None);
        if self.enabled {
            let motion = u64::from(self.drive.control_dt_ms.max(1));
            // Cap at 30 ms while an effect runs so the animation stays smooth
            // without out-pacing the motion-control update.
            Some(if effect_active {
                motion.min(30)
            }
            else {
                motion
            })
        }
        else if effect_active {
            // Keep ticking to animate the LED even with motors disabled.
            Some(30)
        }
        else {
            None
        }
    }

    fn background_tick(&mut self) -> Result<(), Self::Error> {
        // Advance the LED effect first so it animates even with motors off.
        if let Some(color) = self.led_effect.color_at(self.led_phase) {
            self.render_addressable(color);
            // ~4 phase steps/tick @ 30 ms ≈ 2 s per breath/wheel cycle.
            self.led_phase = self.led_phase.wrapping_add(4);
        }

        if !self.enabled {
            return Ok(());
        }

        let plan = self.plan_from_state();
        self.apply_planned(plan);
        Ok(())
    }
}
