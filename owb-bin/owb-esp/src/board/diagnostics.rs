use log::{info, warn};
use owb_core::platform::BoardRuntimeKind;

use super::BoardCapabilities;
use crate::profile::{
    AddressableLed,
    BoardProfile,
    CameraPins,
    EncoderChannelPins,
    MotorPins,
    StatusLed,
};

/// Upper bound on tracked pin assignments. Worst case today is I2C (2) +
/// motor (7) + encoder (6) + LEDs (2) + camera (14) = 31, so this leaves
/// headroom for V2's wider pin map rather than sitting one pin from the cap.
const MAX_PIN_USES: usize = 48;

type PinUses = [Option<PinUse>; MAX_PIN_USES];

#[derive(Clone, Copy)]
struct PinUse {
    pin: u8,
    label: &'static str,
}

fn push_pin_use(
    uses: &mut PinUses,
    len: &mut usize,
    pin: u8,
    label: &'static str,
) {
    if *len >= uses.len() {
        // Dropping a pin here would silently weaken the conflict check, so say
        // so: the table needs raising, not a quiet truncation.
        warn!(
            "board wiring: pin-use table full ({} entries), cannot check {} on GPIO{}; raise MAX_PIN_USES",
            uses.len(),
            label,
            pin
        );
        return;
    }
    uses[*len] = Some(PinUse { pin, label });
    *len += 1;
}

fn collect_pin_uses(profile: &BoardProfile) -> (PinUses, usize) {
    let mut uses: PinUses = [None; MAX_PIN_USES];
    let mut len = 0;

    push_pin_use(&mut uses, &mut len, profile.i2c_bus.sda, "i2c.sda");
    push_pin_use(&mut uses, &mut len, profile.i2c_bus.scl, "i2c.scl");

    if let Some(motor) = profile.motor {
        collect_motor_pins(&mut uses, &mut len, motor);
    }

    if let Some(encoders) = profile.encoder {
        collect_encoder_pins(&mut uses, &mut len, encoders);
    }

    if let Some(StatusLed { pin, .. }) = profile.status_led {
        push_pin_use(&mut uses, &mut len, pin, "status_led");
    }

    if let Some(AddressableLed { pin, .. }) = profile.addressable_led {
        push_pin_use(&mut uses, &mut len, pin, "addressable_led");
    }

    // Only when the camera is actually brought up. `profile.camera` is not an
    // Option, so an inactive profile still carries a pin map -- and on the v1
    // drive/PID profiles those pins are deliberately reused for motors and
    // encoders. Including them unconditionally would report conflicts that do
    // not exist on that boot.
    if profile.has_camera {
        collect_camera_pins(&mut uses, &mut len, profile.camera);
    }

    (uses, len)
}

fn collect_motor_pins(
    uses: &mut PinUses,
    len: &mut usize,
    motor: MotorPins,
) {
    push_pin_use(uses, len, motor.sleep, "motor.sleep");
    push_pin_use(uses, len, motor.bridges[0].phase, "motor1.phase");
    push_pin_use(uses, len, motor.bridges[0].drive, "motor1.drive");
    push_pin_use(uses, len, motor.bridges[1].phase, "motor2.phase");
    push_pin_use(uses, len, motor.bridges[1].drive, "motor2.drive");
    push_pin_use(uses, len, motor.bridges[2].phase, "motor3.phase");
    push_pin_use(uses, len, motor.bridges[2].drive, "motor3.drive");
}

fn collect_encoder_pins(
    uses: &mut PinUses,
    len: &mut usize,
    encoders: [EncoderChannelPins; 3],
) {
    push_pin_use(uses, len, encoders[0].a, "encoder1.a");
    push_pin_use(uses, len, encoders[0].b, "encoder1.b");
    push_pin_use(uses, len, encoders[1].a, "encoder2.a");
    push_pin_use(uses, len, encoders[1].b, "encoder2.b");
    push_pin_use(uses, len, encoders[2].a, "encoder3.a");
    push_pin_use(uses, len, encoders[2].b, "encoder3.b");
}

fn collect_camera_pins(
    uses: &mut PinUses,
    len: &mut usize,
    camera: CameraPins,
) {
    push_pin_use(uses, len, camera.xclk, "camera.xclk");
    push_pin_use(uses, len, camera.pclk, "camera.pclk");
    push_pin_use(uses, len, camera.vsync, "camera.vsync");
    push_pin_use(uses, len, camera.href, "camera.href");
    for (index, pin) in [
        camera.d0, camera.d1, camera.d2, camera.d3, camera.d4, camera.d5, camera.d6, camera.d7,
    ]
    .into_iter()
    .enumerate()
    {
        // Labels are &'static, so index into a fixed table rather than format.
        const DATA_LABELS: [&str; 8] = [
            "camera.d0",
            "camera.d1",
            "camera.d2",
            "camera.d3",
            "camera.d4",
            "camera.d5",
            "camera.d6",
            "camera.d7",
        ];
        push_pin_use(uses, len, pin, DATA_LABELS[index]);
    }
    if let Some(pin) = camera.pwdn {
        push_pin_use(uses, len, pin, "camera.pwdn");
    }
    if let Some(pin) = camera.reset {
        push_pin_use(uses, len, pin, "camera.reset");
    }
}

pub(super) fn log_profile_wiring_diagnostics(
    profile: &BoardProfile,
    caps: &BoardCapabilities,
) {
    if profile.runtime == BoardRuntimeKind::DevKitReference && !caps.has_motion_driver {
        warn!(
            "board wiring: devkit runtime has no motor map active; set OWB_DEVKIT_MTR_* in config/devkit_wiring.local.env before build if you want real motor control"
        );
    }

    if profile.runtime == BoardRuntimeKind::DevKitReference
        && profile.encoder.is_some()
        && !caps.has_motion_driver
    {
        warn!(
            "board wiring: encoder pins are present, but motion is disabled; this boot can probe encoders only"
        );
    }

    let (uses, len) = collect_pin_uses(profile);
    let mut found_conflict = false;
    let mut i = 0;
    while i < len {
        let Some(lhs) = uses[i]
        else {
            i += 1;
            continue;
        };
        let mut j = i + 1;
        while j < len {
            let Some(rhs) = uses[j]
            else {
                j += 1;
                continue;
            };
            if lhs.pin == rhs.pin {
                found_conflict = true;
                warn!(
                    "board wiring: GPIO{} conflict between {} and {}",
                    lhs.pin, lhs.label, rhs.label
                );
            }
            j += 1;
        }
        i += 1;
    }

    if !found_conflict {
        info!("board wiring: no GPIO conflicts detected in active profile");
    }
}
