use owb_core::protocol::{parse_command_from_slice, parse_command_from_str, API_SCHEMA_JSON};

#[test]
fn api_schema_contains_required_routes() {
    let schema = serde_json::from_str::<serde_json::Value>(API_SCHEMA_JSON)
        .expect("schema must be valid json");
    let http = &schema["transport"]["http"];
    assert_eq!(http["static"][0], "/");
    assert_eq!(http["static"][1], "/setup");
    assert_eq!(http["health"], "/healthz");
    assert_eq!(http["schema"], "/api/schema");
    assert_eq!(http["camera_page_alias"], "/camera");
    assert_eq!(http["camera_stream"], "/camera/stream");
    assert_eq!(schema["transport"]["websocket"]["path"], "/ws");
    assert_eq!(
        schema["system_command"]["motors_toggle"]["pc"],
        "motors_toggle"
    );
    assert_eq!(schema["system_command"]["auto_mode"]["ac"], "auto_mode");
}

#[test]
fn parsers_accept_primary_mvp_commands() {
    let enable =
        parse_command_from_str(r#"{"ct":"i","ic":"enable"}"#).expect("enable should parse");
    match enable {
        owb_core::ctrl::SystemCommand::I(owb_core::ctrl::sysctrl::I2CCommand::Enable) => {}
        other => panic!("unexpected command {other:?}"),
    }

    let led = parse_command_from_slice(br#"{"type":"led","cmd":"brightness","value":96}"#)
        .expect("legacy led request should parse");
    match led {
        owb_core::ctrl::SystemCommand::L(owb_core::ctrl::sysctrl::LEDCommand::Brightness { v }) => {
            assert_eq!(v, 96);
        }
        other => panic!("unexpected command {other:?}"),
    }

    let estop = parse_command_from_str(r#"{"ct":"p","pc":"estop"}"#).expect("estop should parse");
    match estop {
        owb_core::ctrl::SystemCommand::P(owb_core::ctrl::sysctrl::PowerCommand::Estop) => {}
        other => panic!("unexpected command {other:?}"),
    }

    let camera_toggle = parse_command_from_str(r#"{"ct":"p","pc":"camera_toggle"}"#)
        .expect("camera_toggle should parse");
    match camera_toggle {
        owb_core::ctrl::SystemCommand::P(owb_core::ctrl::sysctrl::PowerCommand::CameraToggle) => {}
        other => panic!("unexpected command {other:?}"),
    }

    let autonomy =
        parse_command_from_str(r#"{"ct":"a","ac":"auto_mode"}"#).expect("auto-mode should parse");
    match autonomy {
        owb_core::ctrl::SystemCommand::A(owb_core::ctrl::sysctrl::AutonomyCommand::AutoMode) => {}
        other => panic!("unexpected command {other:?}"),
    }

    let read_env =
        parse_command_from_str(r#"{"ct":"i","ic":"read_env"}"#).expect("read_env should parse");
    match read_env {
        owb_core::ctrl::SystemCommand::I(owb_core::ctrl::sysctrl::I2CCommand::ReadEnv) => {}
        other => panic!("unexpected command {other:?}"),
    }

    let read_power =
        parse_command_from_str(r#"{"ct":"i","ic":"read_power"}"#).expect("read_power should parse");
    match read_power {
        owb_core::ctrl::SystemCommand::I(owb_core::ctrl::sysctrl::I2CCommand::ReadPower) => {}
        other => panic!("unexpected command {other:?}"),
    }

    let read_encoders = parse_command_from_str(r#"{"ct":"i","ic":"read_encoders"}"#)
        .expect("read_encoders should parse");
    match read_encoders {
        owb_core::ctrl::SystemCommand::I(owb_core::ctrl::sysctrl::I2CCommand::ReadEncoders) => {}
        other => panic!("unexpected command {other:?}"),
    }
}
