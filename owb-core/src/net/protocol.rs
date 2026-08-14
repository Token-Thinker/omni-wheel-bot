//! Shared protocol types for host and embedded WebSocket handlers.
//!
//! Contains the wire format used by the web UI and a mapper that converts
//! incoming requests into internal `SystemCommand`s.

extern crate alloc;
use serde::Deserialize;

use crate::ctrl::SystemCommand;

pub const API_SCHEMA_JSON: &str = r#"{
  "version": 1,
  "transport": {
    "http": {
        "static": ["/", "/setup", "/style.css", "/script.js"],
      "health": "/healthz",
      "schema": "/api/schema",
      "camera_page_alias": "/camera",
      "camera_stream": "/camera/stream"
    },
    "websocket": {
      "path": "/ws",
      "query": ["session"],
      "heartbeat": {
        "request": {"type": "ping"},
        "response": {"type": "pong"}
      }
    }
  },
  "system_command": {
    "translate": {"ct": "i", "ic": "t", "direction_deg": "f32", "speed_norm": "f32"},
    "yaw": {"ct": "i", "ic": "y", "yaw_rate_deg_s": "f32", "orientation_deg": "f32|null"},
    "omni": {"ct": "i", "ic": "o", "direction_deg": "f32", "speed_norm": "f32", "yaw_rate_deg_s": "f32", "orientation_deg": "f32|null"},
    "probe": {"ct": "i", "ic": "probe", "wheel": "u8", "forward": "bool"},
    "probe_speed": {"ct": "i", "ic": "probe_speed", "wheel": "u8", "speed_norm": "f32"},
    "pid_probe_speed": {"ct": "i", "ic": "pid_probe_speed", "wheel": "u8", "speed_norm": "f32"},
    "raw_bridge": {"ct": "i", "ic": "raw_bridge", "wheel": "u8", "phase_gpio_high": "bool", "drive_gpio_high": "bool"},
    "indicator_led": {"ct": "i", "ic": "indicator_led", "index": "u8", "on": "bool"},
    "enable": {"ct": "i", "ic": "enable"},
    "disable": {"ct": "i", "ic": "disable"},
    "read_imu": {"ct": "i", "ic": "read_imu"},
    "snap": {"ct": "i", "ic": "snap"},
    "read_env": {"ct": "i", "ic": "read_env"},
    "read_power": {"ct": "i", "ic": "read_power"},
    "read_encoders": {"ct": "i", "ic": "read_encoders"},
    "led_brightness": {"ct": "l", "lc": "brightness", "v": "u8"},
    "motors_toggle": {"ct": "p", "pc": "motors_toggle"},
    "doze_toggle": {"ct": "p", "pc": "doze_toggle"},
    "camera_toggle": {"ct": "p", "pc": "camera_toggle"},
    "estop": {"ct": "p", "pc": "estop"},
    "auto_mode": {"ct": "a", "ac": "auto_mode"}
  },
  "request_v1": {
    "i2c_translate": {"type": "i2c", "cmd": "t"},
    "i2c_yaw": {"type": "i2c", "cmd": "y"},
    "i2c_omni": {"type": "i2c", "cmd": "o"},
    "led_brightness": {"type": "led", "cmd": "brightness", "value": "u8"}
  },
  "replies": {
    "ack_ok": {"type": "ack", "status": "ok"},
    "ack_error": {"type": "ack", "status": "error", "message": "string"},
    "imu_ok": {"type": "imu", "status": "ok", "data": "imu_payload"},
    "env_ok": {"type": "env", "status": "ok", "data": {"temp_c": "f32", "humidity_percent": "f32"}},
    "power_ok": {"type": "power", "status": "ok", "data": {"battery_mv": "u16", "soc_percent": "f32"}},
    "encoders_ok": {"type": "encoders", "status": "ok", "data": {"counts": ["i32", "i32", "i32"], "delta_counts": ["i32", "i32", "i32"]}},
    "telemetry": {"type": "telemetry", "frame": "telemetry_frame"}
  }
}"#;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandParseError {
    InvalidFormat,
    UnsupportedMessage,
}

#[derive(Deserialize)]
#[serde(tag = "type")]
pub enum RequestV1 {
    #[serde(rename = "i2c")]
    I2c {
        cmd: alloc::string::String,
        #[serde(default)]
        direction_deg: Option<f32>,
        #[serde(default)]
        speed_norm: Option<f32>,
        #[serde(default)]
        yaw_rate_deg_s: Option<f32>,
        #[serde(default)]
        orientation_deg: Option<f32>,
        #[serde(alias = "d")]
        _d: Option<f32>,
        #[serde(alias = "s")]
        _s: Option<f32>,
        #[serde(alias = "rs")]
        _rs: Option<f32>,
        #[serde(alias = "o")]
        _o: Option<f32>,
    },
    #[serde(rename = "led")]
    Led {
        cmd: alloc::string::String,
        value: Option<u8>,
    },
}

pub fn map_request_to_command(r: RequestV1) -> Option<SystemCommand> {
    match r {
        RequestV1::I2c {
            cmd,
            direction_deg,
            speed_norm,
            yaw_rate_deg_s,
            orientation_deg,
            _d,
            _s,
            _rs,
            _o,
        } => {
            let d = direction_deg.or(_d).unwrap_or(0.0);
            let s = speed_norm.or(_s).unwrap_or(0.0);
            let rs = yaw_rate_deg_s.or(_rs).unwrap_or(0.0);
            let o = orientation_deg.or(_o);
            match cmd.as_str() {
                "t" => Some(SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::T {
                    direction_deg: d,
                    speed_norm: s,
                })),
                "y" => Some(SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::Y {
                    yaw_rate_deg_s: rs,
                    orientation_deg: o,
                })),
                "o" => Some(SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::O {
                    direction_deg: d,
                    speed_norm: s,
                    yaw_rate_deg_s: rs,
                    orientation_deg: o,
                })),
                _ => None,
            }
        }
        RequestV1::Led { cmd, value } => {
            use crate::ctrl::sysctrl::LEDCommand as LC;
            let c = match cmd.as_str() {
                "on" => LC::On,
                "off" => LC::Off,
                "brightness" => LC::Brightness {
                    v: value.unwrap_or(0),
                },
                _ => return None,
            };
            Some(SystemCommand::L(c))
        }
    }
}

/// Parse a text websocket payload into a `SystemCommand`.
pub fn parse_command_from_str(s: &str) -> Result<SystemCommand, CommandParseError> {
    if let Ok(cmd) = serde_json::from_str::<SystemCommand>(s) {
        return Ok(cmd);
    }

    if let Ok(req) = serde_json::from_str::<RequestV1>(s) {
        return map_request_to_command(req).ok_or(CommandParseError::UnsupportedMessage);
    }

    match serde_json::from_str::<serde_json::Value>(s) {
        Ok(_) => Err(CommandParseError::UnsupportedMessage),
        Err(_) => Err(CommandParseError::InvalidFormat),
    }
}

/// Parse a binary websocket payload into a `SystemCommand`.
pub fn parse_command_from_slice(bytes: &[u8]) -> Result<SystemCommand, CommandParseError> {
    if let Ok(cmd) = serde_json::from_slice::<SystemCommand>(bytes) {
        return Ok(cmd);
    }

    if let Ok(req) = serde_json::from_slice::<RequestV1>(bytes) {
        return map_request_to_command(req).ok_or(CommandParseError::UnsupportedMessage);
    }

    match serde_json::from_slice::<serde_json::Value>(bytes) {
        Ok(_) => Err(CommandParseError::UnsupportedMessage),
        Err(_) => Err(CommandParseError::InvalidFormat),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_translate_command() {
        let cmd = RequestV1::I2c {
            cmd: "t".into(),
            direction_deg: Some(90.0),
            speed_norm: Some(0.5),
            yaw_rate_deg_s: None,
            orientation_deg: None,
            _d: None,
            _s: None,
            _rs: None,
            _o: None,
        };
        let mapped = map_request_to_command(cmd).expect("should map translate command");
        match mapped {
            SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::T {
                direction_deg,
                speed_norm,
            }) => {
                assert_eq!(direction_deg, 90.0);
                assert_eq!(speed_norm, 0.5);
            }
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn map_yaw_uses_explicit_rate() {
        let cmd = RequestV1::I2c {
            cmd: "y".into(),
            direction_deg: None,
            speed_norm: Some(0.5),
            yaw_rate_deg_s: Some(12.5),
            orientation_deg: Some(45.0),
            _d: None,
            _s: None,
            _rs: None,
            _o: None,
        };
        let mapped = map_request_to_command(cmd).expect("should map yaw command");
        match mapped {
            SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::Y {
                yaw_rate_deg_s,
                orientation_deg,
            }) => {
                assert_eq!(yaw_rate_deg_s, 12.5);
                assert_eq!(orientation_deg, Some(45.0));
            }
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn map_old_short_fields_still_work() {
        let cmd = RequestV1::I2c {
            cmd: "o".into(),
            direction_deg: None,
            speed_norm: None,
            yaw_rate_deg_s: None,
            orientation_deg: None,
            _d: Some(90.0),
            _s: Some(0.75),
            _rs: Some(15.0),
            _o: Some(5.0),
        };
        let mapped = map_request_to_command(cmd).expect("should map legacy fields");
        match mapped {
            SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::O {
                direction_deg,
                speed_norm,
                yaw_rate_deg_s,
                orientation_deg,
            }) => {
                assert_eq!(direction_deg, 90.0);
                assert_eq!(speed_norm, 0.75);
                assert_eq!(yaw_rate_deg_s, 15.0);
                assert_eq!(orientation_deg, Some(5.0));
            }
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn parse_command_reports_unsupported_json() {
        let err = parse_command_from_str(r#"{"type":"i2c","cmd":"unknown"}"#)
            .expect_err("unsupported command should error");
        assert_eq!(err, CommandParseError::UnsupportedMessage);
    }

    #[test]
    fn parse_read_imu_command_uses_ui_wire_name() {
        let parsed =
            parse_command_from_str(r#"{"ct":"i","ic":"read_imu"}"#).expect("read_imu should parse");
        match parsed {
            SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::ReadIMU) => {}
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn parse_pid_probe_speed_command() {
        let parsed = parse_command_from_str(
            r#"{"ct":"i","ic":"pid_probe_speed","wheel":1,"speed_norm":0.25}"#,
        )
        .expect("pid probe should parse");
        match parsed {
            SystemCommand::I(crate::ctrl::sysctrl::I2CCommand::PidProbeSpeed {
                wheel,
                speed_norm,
            }) => {
                assert_eq!(wheel, 1);
                assert_eq!(speed_norm, 0.25);
            }
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn parse_command_reports_invalid_json() {
        let err = parse_command_from_str("{").expect_err("invalid json should error");
        assert_eq!(err, CommandParseError::InvalidFormat);
    }

    #[test]
    fn schema_json_is_valid() {
        let parsed = serde_json::from_str::<serde_json::Value>(API_SCHEMA_JSON)
            .expect("schema should be valid json");
        assert!(parsed.get("system_command").is_some());
        assert!(parsed.get("transport").is_some());
    }

    #[test]
    fn parse_led_wheel_command() {
        let parsed =
            parse_command_from_str(r#"{"ct":"l","lc":"wheel"}"#).expect("wheel should parse");
        match parsed {
            SystemCommand::L(crate::msg::legacy::LEDCommand::Wheel) => {}
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn parse_led_breath_command() {
        let parsed = parse_command_from_str(r#"{"ct":"l","lc":"breath","r":0,"g":80,"b":255}"#)
            .expect("breath should parse");
        match parsed {
            SystemCommand::L(crate::msg::legacy::LEDCommand::Breath { r, g, b }) => {
                assert_eq!((r, g, b), (0, 80, 255));
            }
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn parse_led_solid_color_command() {
        let parsed = parse_command_from_str(r#"{"ct":"l","lc":"s_c","r":255,"g":0,"b":128}"#)
            .expect("solid color should parse");
        match parsed {
            SystemCommand::L(crate::msg::legacy::LEDCommand::SC { r, g, b }) => {
                assert_eq!((r, g, b), (255, 0, 128));
            }
            other => panic!("unexpected command {other:?}"),
        }
    }

    #[test]
    fn parse_led_off_command() {
        let parsed = parse_command_from_str(r#"{"ct":"l","lc":"off"}"#).expect("off should parse");
        match parsed {
            SystemCommand::L(crate::msg::legacy::LEDCommand::Off) => {}
            other => panic!("unexpected command {other:?}"),
        }
    }
}
