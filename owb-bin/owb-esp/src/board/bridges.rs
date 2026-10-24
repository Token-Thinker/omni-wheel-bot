#[cfg(feature = "real-executors")]
use esp_hal::{
    gpio::{AnyPin, DriveMode, Level, Output, OutputConfig},
    ledc::{
        channel::{self as ledc_channel, Channel as LedcChannel, ChannelIFace},
        timer::{self as ledc_timer, Timer as LedcTimer, TimerIFace},
        LSGlobalClkSource,
        Ledc,
        LowSpeed,
    },
    time::Rate,
};
#[cfg(feature = "real-executors")]
use log::{info, warn};
#[cfg(feature = "real-executors")]
use owb_core::{ext::prelude::StaticCell, motion::omni::HalfBridgeOutput};

#[cfg(feature = "real-executors")]
use crate::profile::{MotorBridgeMode, MotorPins};

#[cfg(feature = "real-executors")]
// Keep motor PWM above the audible band for the alpha demo. Short-burst
// testing at 4 kHz did not improve overall W1/W2 consistency, so the alpha
// joystick limits motor authority in software instead of changing PWM rate.
const MOTOR_PWM_FREQ_KHZ: u32 = 20;

/// When set, W2's drive pulse is right-aligned in the PWM period instead of
/// sharing W1's left-aligned edge. Both channels run off LEDC Timer0, so by
/// default both motors switch on at the same instant every period and the
/// motor rail sees the summed inrush; staggering them halves the peak at duty
/// <= 50% and minimises overlap above it.
#[cfg(feature = "real-executors")]
static PWM_STAGGER: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[cfg(feature = "real-executors")]
pub(super) fn set_pwm_stagger(enabled: bool) {
    PWM_STAGGER.store(enabled, core::sync::atomic::Ordering::Relaxed);
}

#[cfg(feature = "real-executors")]
pub(super) fn pwm_stagger() -> bool { PWM_STAGGER.load(core::sync::atomic::Ordering::Relaxed) }

/// Read back W2's (LEDC channel 1) HPOINT so a bench log can confirm the
/// stagger write landed; there is no other way to see it without a scope.
#[cfg(feature = "real-executors")]
pub(super) fn staggered_channel_hpoint() -> u32 {
    let reg = (LEDC_BASE + LEDC_CH_STRIDE + LEDC_CH_HPOINT_OFFSET) as *const u32;
    // SAFETY: plain read of a LEDC status/config register; no side effects.
    unsafe { core::ptr::read_volatile(reg) & 0x3fff }
}

/// LEDC register block on the ESP32-S3 (TRM ch. 36). esp-hal keeps the PAC
/// crate-private and always programs HPOINT = 0, so the stagger writes the
/// per-channel HPOINT register directly.
#[cfg(feature = "real-executors")]
const LEDC_BASE: usize = 0x6001_9000;
#[cfg(feature = "real-executors")]
const LEDC_CH_STRIDE: usize = 0x14;
#[cfg(feature = "real-executors")]
const LEDC_CH_HPOINT_OFFSET: usize = 0x04;
/// Counter steps per period at `Duty8Bit`.
#[cfg(feature = "real-executors")]
const LEDC_PERIOD_STEPS: u32 = 256;

/// HPOINT that right-aligns a pulse of `duty_pct`, or 0 (left-aligned) when
/// staggering is off or the pulse is fully off/on.
///
/// The output goes high at HPOINT and low at HPOINT + duty. A low point of
/// 256 never matches the 8-bit counter and would latch the output high, so the
/// pulse ends one step early (at 255) -- 0.4% of a period.
#[cfg(feature = "real-executors")]
fn right_aligned_hpoint(duty_pct: u8) -> u32 {
    // Same rounding esp-hal's `set_duty` uses, so the pulse width is unchanged.
    let duty = LEDC_PERIOD_STEPS * u32::from(duty_pct.min(100)) / 100;
    if duty == 0 || duty >= LEDC_PERIOD_STEPS - 1 {
        0
    }
    else {
        LEDC_PERIOD_STEPS - 1 - duty
    }
}

#[cfg(feature = "real-executors")]
type LedcLowSpeedTimer = LedcTimer<'static, LowSpeed>;
#[cfg(feature = "real-executors")]
type LedcLowSpeedChannel = LedcChannel<'static, LowSpeed>;

#[cfg(feature = "real-executors")]
pub(super) struct GpioBridge {
    mode: MotorBridgeMode,
    phase: Output<'static>,
    drive: Output<'static>,
}

#[cfg(feature = "real-executors")]
pub(super) struct PwmBridge {
    mode: MotorBridgeMode,
    phase: &'static mut LedcLowSpeedChannel,
    drive: &'static mut LedcLowSpeedChannel,
}

#[cfg(feature = "real-executors")]
pub(super) struct PhasePwmBridge {
    mode: MotorBridgeMode,
    phase: Output<'static>,
    drive: &'static mut LedcLowSpeedChannel,
    /// LEDC channel number when this bridge takes part in `PWM_STAGGER`.
    stagger_channel: Option<u8>,
}

#[cfg(feature = "real-executors")]
pub(super) enum MotorBridge {
    Disabled,
    Gpio(GpioBridge),
    Pwm(PwmBridge),
    PhasePwm(PhasePwmBridge),
}

#[cfg(feature = "real-executors")]
impl GpioBridge {
    fn output_config(
        mode: MotorBridgeMode,
        pin: u8,
    ) -> OutputConfig {
        let config = OutputConfig::default();

        // v1 W3 routes GPIO47/48 through 2N7002 pull-up level shifters. Drive
        // low actively and release high so the shifter can translate cleanly.
        if mode == MotorBridgeMode::Drv8845PhaseEnable && matches!(pin, 47 | 48) {
            config.with_drive_mode(DriveMode::OpenDrain)
        }
        else {
            config
        }
    }

    pub(super) fn levels_for(
        mode: MotorBridgeMode,
        output: HalfBridgeOutput,
    ) -> (bool, bool) {
        match mode {
            MotorBridgeMode::PhaseEnable => (output.phase_high, output.drive_high),
            MotorBridgeMode::Drv8845PhaseEnable => (output.phase_high, !output.drive_high),
            MotorBridgeMode::In1In2 => {
                if output.drive_high {
                    if output.phase_high {
                        (true, false)
                    }
                    else {
                        (false, true)
                    }
                }
                else {
                    (false, false)
                }
            }
        }
    }

    pub(super) fn new(
        mode: MotorBridgeMode,
        phase_pin: u8,
        drive_pin: u8,
    ) -> Self {
        let mut phase = Output::new(
            unsafe { AnyPin::steal(phase_pin) },
            Level::Low,
            Self::output_config(mode, phase_pin),
        );
        let mut drive = Output::new(
            unsafe { AnyPin::steal(drive_pin) },
            Level::Low,
            Self::output_config(mode, drive_pin),
        );
        phase.set_low();
        drive.set_low();
        Self { mode, phase, drive }
    }

    fn apply(
        &mut self,
        output: HalfBridgeOutput,
    ) {
        let (phase_high, drive_high) = Self::levels_for(self.mode, output);
        self.apply_raw_levels(phase_high, drive_high);
    }

    fn apply_raw_levels(
        &mut self,
        phase_high: bool,
        drive_high: bool,
    ) {
        if phase_high {
            self.phase.set_high();
        }
        else {
            self.phase.set_low();
        }

        if drive_high {
            self.drive.set_high();
        }
        else {
            self.drive.set_low();
        }
    }
}

#[cfg(feature = "real-executors")]
impl PwmBridge {
    pub(super) fn new_wheel0(
        mode: MotorBridgeMode,
        phase_pin: u8,
        drive_pin: u8,
    ) -> Option<Self> {
        static TIMER: StaticCell<LedcLowSpeedTimer> = StaticCell::new();
        static PHASE: StaticCell<LedcLowSpeedChannel> = StaticCell::new();
        static DRIVE: StaticCell<LedcLowSpeedChannel> = StaticCell::new();

        let mut ledc = Ledc::new(unsafe { esp_hal::peripherals::LEDC::steal() });
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let timer = TIMER.init(ledc.timer::<LowSpeed>(ledc_timer::Number::Timer0));
        if let Err(err) = timer.configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(MOTOR_PWM_FREQ_KHZ),
        }) {
            warn!("board pwm: timer configure failed: {err:?}");
            return None;
        }

        let phase = PHASE.init(ledc.channel(ledc_channel::Number::Channel0, unsafe {
            AnyPin::steal(phase_pin)
        }));
        if let Err(err) = phase.configure(ledc_channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        }) {
            warn!("board pwm: phase channel configure failed: {err:?}");
            return None;
        }

        let drive = DRIVE.init(ledc.channel(ledc_channel::Number::Channel1, unsafe {
            AnyPin::steal(drive_pin)
        }));
        if let Err(err) = drive.configure(ledc_channel::config::Config {
            timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        }) {
            warn!("board pwm: drive channel configure failed: {err:?}");
            return None;
        }

        Some(Self { mode, phase, drive })
    }

    fn set_pct(
        &mut self,
        phase_pct: u8,
        drive_pct: u8,
    ) {
        let _ = self.phase.set_duty(phase_pct.min(100));
        let _ = self.drive.set_duty(drive_pct.min(100));
    }

    fn apply(
        &mut self,
        output: HalfBridgeOutput,
    ) {
        match self.mode {
            MotorBridgeMode::PhaseEnable => {
                self.set_pct(
                    if output.phase_high { 100 } else { 0 },
                    if output.drive_high { 100 } else { 0 },
                );
            }
            MotorBridgeMode::Drv8845PhaseEnable => {
                self.set_pct(
                    if output.phase_high { 100 } else { 0 },
                    if output.drive_high { 0 } else { 100 },
                );
            }
            MotorBridgeMode::In1In2 => {
                let (phase_high, drive_high) = GpioBridge::levels_for(self.mode, output);
                self.set_pct(
                    if phase_high { 100 } else { 0 },
                    if drive_high { 100 } else { 0 },
                );
            }
        }
    }

    fn apply_raw_levels(
        &mut self,
        phase_high: bool,
        drive_high: bool,
    ) {
        self.set_pct(
            if phase_high { 100 } else { 0 },
            if drive_high { 100 } else { 0 },
        );
    }

    fn apply_signed_norm(
        &mut self,
        speed_norm: f32,
    ) {
        let speed = speed_norm.clamp(-1.0, 1.0);
        let duty_pct = ((speed.abs() * 100.0) + 0.5).clamp(0.0, 100.0) as u8;

        match self.mode {
            MotorBridgeMode::PhaseEnable => {
                self.set_pct(
                    if speed.is_sign_positive() && duty_pct > 0 {
                        100
                    }
                    else {
                        0
                    },
                    duty_pct,
                );
            }
            MotorBridgeMode::Drv8845PhaseEnable => {
                self.set_pct(
                    if speed.is_sign_positive() && duty_pct > 0 {
                        100
                    }
                    else {
                        0
                    },
                    100u8.saturating_sub(duty_pct),
                );
            }
            MotorBridgeMode::In1In2 => {
                if duty_pct == 0 {
                    self.set_pct(0, 0);
                }
                else if speed.is_sign_positive() {
                    self.set_pct(duty_pct, 0);
                }
                else {
                    self.set_pct(0, duty_pct);
                }
            }
        }
    }
}

#[cfg(feature = "real-executors")]
impl PhasePwmBridge {
    pub(super) fn new_three(
        pins: MotorPins,
        active_wheels: [bool; 3],
    ) -> Option<[MotorBridge; 3]> {
        static TIMER: StaticCell<LedcLowSpeedTimer> = StaticCell::new();
        static DRIVE0: StaticCell<LedcLowSpeedChannel> = StaticCell::new();
        static DRIVE1: StaticCell<LedcLowSpeedChannel> = StaticCell::new();
        static DRIVE2: StaticCell<LedcLowSpeedChannel> = StaticCell::new();

        let mut ledc = Ledc::new(unsafe { esp_hal::peripherals::LEDC::steal() });
        ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

        let timer = TIMER.init(ledc.timer::<LowSpeed>(ledc_timer::Number::Timer0));
        if let Err(err) = timer.configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(MOTOR_PWM_FREQ_KHZ),
        }) {
            warn!("board pwm: timer configure failed: {err:?}");
            return None;
        }

        let wheel0 = if active_wheels[0] {
            let mut phase = Self::new_phase_output(pins.mode, pins.bridges[0].phase);
            phase.set_low();
            let drive = DRIVE0.init(ledc.channel(ledc_channel::Number::Channel0, unsafe {
                AnyPin::steal(pins.bridges[0].drive)
            }));
            if let Err(err) = drive.configure(ledc_channel::config::Config {
                timer,
                duty_pct: Self::inactive_drive_pct(pins.mode),
                drive_mode: Self::drive_mode(pins.mode, pins.bridges[0].drive),
            }) {
                warn!("board pwm: drive0 channel configure failed: {err:?}");
                return None;
            }
            MotorBridge::PhasePwm(Self {
                mode: pins.mode,
                phase,
                drive,
                stagger_channel: None,
            })
        }
        else {
            MotorBridge::Disabled
        };

        let wheel1 = if active_wheels[1] {
            let mut phase = Self::new_phase_output(pins.mode, pins.bridges[1].phase);
            phase.set_low();
            let drive = DRIVE1.init(ledc.channel(ledc_channel::Number::Channel1, unsafe {
                AnyPin::steal(pins.bridges[1].drive)
            }));
            if let Err(err) = drive.configure(ledc_channel::config::Config {
                timer,
                duty_pct: Self::inactive_drive_pct(pins.mode),
                drive_mode: Self::drive_mode(pins.mode, pins.bridges[1].drive),
            }) {
                warn!("board pwm: drive1 channel configure failed: {err:?}");
                return None;
            }
            MotorBridge::PhasePwm(Self {
                mode: pins.mode,
                phase,
                drive,
                stagger_channel: Some(ledc_channel::Number::Channel1 as u8),
            })
        }
        else {
            MotorBridge::Disabled
        };

        let wheel2 = if active_wheels[2] {
            let mut phase = Self::new_phase_output(pins.mode, pins.bridges[2].phase);
            phase.set_low();
            let drive = DRIVE2.init(ledc.channel(ledc_channel::Number::Channel2, unsafe {
                AnyPin::steal(pins.bridges[2].drive)
            }));
            if let Err(err) = drive.configure(ledc_channel::config::Config {
                timer,
                duty_pct: Self::inactive_drive_pct(pins.mode),
                drive_mode: Self::drive_mode(pins.mode, pins.bridges[2].drive),
            }) {
                warn!("board pwm: drive2 channel configure failed: {err:?}");
                return None;
            }
            MotorBridge::PhasePwm(Self {
                mode: pins.mode,
                phase,
                drive,
                stagger_channel: None,
            })
        }
        else {
            MotorBridge::Disabled
        };

        info!(
            "board pwm: phase/enable PWM active wheels={:?} drive GPIOs [{},{},{}] at {}kHz",
            active_wheels,
            pins.bridges[0].drive,
            pins.bridges[1].drive,
            pins.bridges[2].drive,
            MOTOR_PWM_FREQ_KHZ
        );

        Some([wheel0, wheel1, wheel2])
    }

    fn new_phase_output(
        mode: MotorBridgeMode,
        phase_pin: u8,
    ) -> Output<'static> {
        Output::new(
            unsafe { AnyPin::steal(phase_pin) },
            Level::Low,
            GpioBridge::output_config(mode, phase_pin),
        )
    }

    fn drive_mode(
        mode: MotorBridgeMode,
        drive_pin: u8,
    ) -> DriveMode {
        if mode == MotorBridgeMode::Drv8845PhaseEnable && matches!(drive_pin, 47 | 48) {
            DriveMode::OpenDrain
        }
        else {
            DriveMode::PushPull
        }
    }

    fn inactive_drive_pct(mode: MotorBridgeMode) -> u8 {
        if mode == MotorBridgeMode::Drv8845PhaseEnable {
            100
        }
        else {
            0
        }
    }

    fn set_phase(
        &mut self,
        high: bool,
    ) {
        if high {
            self.phase.set_high();
        }
        else {
            self.phase.set_low();
        }
    }

    fn set_drive_pct(
        &mut self,
        duty_pct: u8,
    ) {
        let duty_pct = duty_pct.min(100);
        if let Some(channel) = self.stagger_channel {
            let hpoint = if pwm_stagger() {
                right_aligned_hpoint(duty_pct)
            }
            else {
                0
            };
            let reg = (LEDC_BASE + usize::from(channel) * LEDC_CH_STRIDE + LEDC_CH_HPOINT_OFFSET)
                as *mut u32;
            // SAFETY: CHn_HPOINT is a plain R/W register owned by this
            // channel, which this bridge holds exclusively. It is latched by
            // the PARA_UP that `set_duty` issues right after, so the new
            // HPOINT and duty take effect in the same PWM period.
            unsafe { core::ptr::write_volatile(reg, hpoint & 0x3fff) };
        }
        let _ = self.drive.set_duty(duty_pct);
    }

    fn apply(
        &mut self,
        output: HalfBridgeOutput,
    ) {
        self.set_phase(output.phase_high);
        match self.mode {
            MotorBridgeMode::PhaseEnable => {
                self.set_drive_pct(if output.drive_high { 100 } else { 0 });
            }
            MotorBridgeMode::Drv8845PhaseEnable => {
                self.set_drive_pct(if output.drive_high { 0 } else { 100 });
            }
            MotorBridgeMode::In1In2 => {
                let (_, drive_high) = GpioBridge::levels_for(self.mode, output);
                self.set_drive_pct(if drive_high { 100 } else { 0 });
            }
        }
    }

    fn apply_raw_levels(
        &mut self,
        phase_high: bool,
        drive_high: bool,
    ) {
        self.set_phase(phase_high);
        self.set_drive_pct(if drive_high { 100 } else { 0 });
    }

    fn apply_signed_norm(
        &mut self,
        speed_norm: f32,
    ) {
        let speed = speed_norm.clamp(-1.0, 1.0);
        let duty_pct = ((speed.abs() * 100.0) + 0.5).clamp(0.0, 100.0) as u8;

        self.set_phase(speed.is_sign_positive() && duty_pct > 0);
        match self.mode {
            MotorBridgeMode::PhaseEnable => self.set_drive_pct(duty_pct),
            MotorBridgeMode::Drv8845PhaseEnable => {
                self.set_drive_pct(100u8.saturating_sub(duty_pct));
            }
            MotorBridgeMode::In1In2 => self.set_drive_pct(duty_pct),
        }
    }
}

#[cfg(feature = "real-executors")]
impl MotorBridge {
    pub(super) fn apply(
        &mut self,
        output: HalfBridgeOutput,
    ) {
        match self {
            Self::Disabled => {}
            Self::Gpio(bridge) => bridge.apply(output),
            Self::Pwm(bridge) => bridge.apply(output),
            Self::PhasePwm(bridge) => bridge.apply(output),
        }
    }

    pub(super) fn apply_raw_levels(
        &mut self,
        phase_high: bool,
        drive_high: bool,
    ) {
        match self {
            Self::Disabled => {}
            Self::Gpio(bridge) => bridge.apply_raw_levels(phase_high, drive_high),
            Self::Pwm(bridge) => bridge.apply_raw_levels(phase_high, drive_high),
            Self::PhasePwm(bridge) => bridge.apply_raw_levels(phase_high, drive_high),
        }
    }

    pub(super) fn apply_signed_norm(
        &mut self,
        speed_norm: f32,
    ) {
        match self {
            Self::Disabled => {}
            Self::Gpio(bridge) => bridge.apply(HalfBridgeOutput {
                phase_high: speed_norm.is_sign_positive(),
                drive_high: speed_norm.abs() > 0.0,
            }),
            Self::Pwm(bridge) => bridge.apply_signed_norm(speed_norm),
            Self::PhasePwm(bridge) => bridge.apply_signed_norm(speed_norm),
        }
    }
}
