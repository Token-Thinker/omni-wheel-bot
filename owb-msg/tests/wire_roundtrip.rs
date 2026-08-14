use owb_msg::legacy::{I2CCommand, LEDCommand, SystemCommand};

#[test]
fn i2c_translate_tag() {
    let cmd = SystemCommand::I(I2CCommand::T {
        direction_deg: 90.0,
        speed_norm: 0.5,
    });
    let json = serde_json::to_string(&cmd).unwrap();
    assert!(json.contains(r#""ct":"i""#), "ct tag missing: {json}");
    assert!(json.contains(r#""ic":"t""#), "ic tag missing: {json}");
}

#[test]
fn led_sc_tag() {
    // The s_c tag is load-bearing: rename_all="snake_case" on LEDCommand converts SC → s_c
    let cmd = SystemCommand::L(LEDCommand::SC {
        r: 255,
        g: 128,
        b: 0,
    });
    let json = serde_json::to_string(&cmd).unwrap();
    assert!(json.contains(r#""lc":"s_c""#), "s_c tag missing: {json}");
}

#[test]
fn i2c_enable_roundtrip() {
    let cmd = SystemCommand::I(I2CCommand::Enable);
    let json = serde_json::to_string(&cmd).unwrap();
    let back: SystemCommand = serde_json::from_str(&json).unwrap();
    assert!(matches!(back, SystemCommand::I(I2CCommand::Enable)));
}

#[test]
fn power_command_tag() {
    use owb_msg::legacy::{PowerCommand, SystemCommand};
    let cmd = SystemCommand::P(PowerCommand::Estop);
    let json = serde_json::to_string(&cmd).unwrap();
    assert!(json.contains(r#""ct":"p""#), "ct=p missing: {json}");
    assert!(json.contains(r#""pc":"estop""#), "pc tag missing: {json}");
}
