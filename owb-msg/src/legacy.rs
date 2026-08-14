//! Legacy wire command and telemetry payloads.
//!
//! These names preserve the current transport JSON and controller API while
//! giving message contracts a canonical home under `msg`.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImuData {
    pub accel: (f32, f32, f32),
    pub gyro: (f32, f32, f32),
    pub temp: f32,
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EnvData {
    pub temp_c: f32,
    pub humidity_percent: f32,
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PowerData {
    pub battery_mv: u16,
    pub soc_percent: f32,
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EncoderData {
    pub counts: [i32; 3],
    pub delta_counts: [i32; 3],
}

/// LED command variants for switching on/off or setting a color.
/// Serialized as JSON with tag `"lc"`.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "lc", rename_all = "snake_case"))]
pub enum LEDCommand {
    On,
    Off,
    SC {
        r: u8,
        g: u8,
        b: u8,
    },
    #[cfg_attr(feature = "serde", serde(rename = "brightness"))]
    Brightness {
        v: u8,
    },
    /// Brightness-pulse "breath" effect on the given base color.
    Breath {
        r: u8,
        g: u8,
        b: u8,
    },
    /// Continuous hue cycle through the color wheel.
    Wheel,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "ic", rename_all = "snake_case"))]
pub enum I2CCommand {
    T {
        #[cfg_attr(feature = "serde", serde(alias = "d", rename = "direction_deg"))]
        direction_deg: f32,
        #[cfg_attr(feature = "serde", serde(alias = "s", rename = "speed_norm"))]
        speed_norm: f32,
    },
    Y {
        #[cfg_attr(feature = "serde", serde(alias = "s", rename = "yaw_rate_deg_s"))]
        yaw_rate_deg_s: f32,
        #[cfg_attr(feature = "serde", serde(alias = "o", rename = "orientation_deg"))]
        orientation_deg: Option<f32>,
    },
    O {
        #[cfg_attr(feature = "serde", serde(alias = "d", rename = "direction_deg"))]
        direction_deg: f32,
        #[cfg_attr(feature = "serde", serde(alias = "s", rename = "speed_norm"))]
        speed_norm: f32,
        #[cfg_attr(feature = "serde", serde(alias = "rs", rename = "yaw_rate_deg_s"))]
        yaw_rate_deg_s: f32,
        #[cfg_attr(feature = "serde", serde(alias = "o", rename = "orientation_deg"))]
        orientation_deg: Option<f32>,
    },
    Probe {
        wheel: u8,
        forward: bool,
    },
    ProbeSpeed {
        wheel: u8,
        #[cfg_attr(feature = "serde", serde(alias = "s", rename = "speed_norm"))]
        speed_norm: f32,
    },
    PidProbeSpeed {
        wheel: u8,
        #[cfg_attr(feature = "serde", serde(alias = "s", rename = "speed_norm"))]
        speed_norm: f32,
    },
    RawBridge {
        wheel: u8,
        phase_gpio_high: bool,
        drive_gpio_high: bool,
    },
    IndicatorLed {
        index: u8,
        on: bool,
    },
    /// Read the IMU with the human-friendly wire name used by the UI.
    #[cfg_attr(feature = "serde", serde(rename = "read_imu", alias = "read_i_m_u"))]
    ReadIMU,
    ReadEnv,
    ReadPower,
    ReadEncoders,
    /// Capture one JPEG frame and emit it as base64 over USB serial.
    Snap,
    Enable,
    Disable,
    /// Per-wheel speed-norm trim multipliers applied in the runtime drive path.
    /// 1.0 = no change. Values are clamped to a safe range in the executor.
    WheelTrim {
        #[cfg_attr(feature = "serde", serde(alias = "w1", rename = "w1"))]
        w1: f32,
        #[cfg_attr(feature = "serde", serde(alias = "w2", rename = "w2"))]
        w2: f32,
        #[cfg_attr(feature = "serde", serde(alias = "w3", rename = "w3"))]
        w3: f32,
    },
    /// Enable closed-loop straightness correction in v1-alpha two-wheel mode.
    HeadingHold {
        enable: bool,
        #[cfg_attr(feature = "serde", serde(default, alias = "kp"))]
        kp: f32,
    },
    /// Override the runtime stiction-kick pulse parameters.
    StictionKick {
        #[cfg_attr(feature = "serde", serde(default))]
        ms: u16,
        #[cfg_attr(feature = "serde", serde(default))]
        boost: f32,
    },
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "pc", rename_all = "snake_case"))]
pub enum PowerCommand {
    MotorsToggle,
    DozeToggle,
    CameraToggle,
    Estop,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "ac", rename_all = "snake_case"))]
pub enum AutonomyCommand {
    AutoMode,
    Start,
    Stop,
}

#[derive(Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(tag = "ct", rename_all = "snake_case"))]
pub enum SystemCommand {
    I(I2CCommand),
    L(LEDCommand),
    P(PowerCommand),
    A(AutonomyCommand),
}

/// Shared result of executing a high-level system command.
#[derive(Debug, Clone, Copy)]
pub enum CommandReply {
    Ok,
    ImuData(ImuData),
    EnvData(EnvData),
    PowerData(PowerData),
    EncoderData(EncoderData),
    /// The `CommandExecutor` returned an error for this command (e.g. PWM
    /// write fault, I2C NACK).  Additive, backward-compatible variant: no
    /// serde derive on `CommandReply`, so the wire shape is owned by the
    /// manual `command_reply_value` match in `net/gateway.rs`.  Old clients
    /// that do not recognise `"status":"error"` will log an unknown reply
    /// type — they will not crash or misinterpret an Ok.
    Err,
}
