#![allow(dead_code)]

use owb_core::{
    ext::prelude::StaticCell,
    motion::omni::{DriveTuning, PidGains},
    platform::{BoardFamily, BoardRuntimeKind, RuntimeExecutionPlan},
};

/// Primary I2C bus pins for a board profile.
///
/// The same bus is used for camera SCCB on current ESP32-S3 targets and, on
/// integrated boards, will also carry shared board peripherals.
#[derive(Clone, Copy)]
pub struct I2cPins {
    pub sda: u8,
    pub scl: u8,
}

/// Camera-related GPIO mapping for a board profile.
#[derive(Clone, Copy)]
pub struct CameraPins {
    pub xclk: u8,
    pub pclk: u8,
    pub vsync: u8,
    pub href: u8,
    pub d0: u8,
    pub d1: u8,
    pub d2: u8,
    pub d3: u8,
    pub d4: u8,
    pub d5: u8,
    pub d6: u8,
    pub d7: u8,
    pub pwdn: Option<u8>,
    pub reset: Option<u8>,
}

/// Optional status LED mapping for bring-up and test builds.
#[derive(Clone, Copy)]
pub struct StatusLed {
    pub pin: u8,
    pub active_high: bool,
}

/// One-wire addressable RGB LED chain (WS2812/SK6812 class).
#[derive(Clone, Copy)]
pub struct AddressableLed {
    pub pin: u8,
    pub pixels: u8,
    pub inverted: bool,
}

/// GPIO interpretation for each motor bridge pair.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorBridgeMode {
    /// `phase` selects direction while `drive` enables the bridge.
    PhaseEnable,
    /// DRV8845 DC mode: tied I0x/I1x enable is active-low, PHASE selects direction.
    Drv8845PhaseEnable,
    /// `phase` and `drive` are the two logic inputs of a conventional H-bridge.
    In1In2,
}

/// One logical motor bridge pair.
#[derive(Clone, Copy)]
pub struct MotorBridgePins {
    pub phase: u8,
    pub drive: u8,
}

/// GPIO mapping for the three-motor DRV8845 interface.
#[derive(Clone, Copy)]
pub struct MotorPins {
    pub sleep: u8,
    pub mode: MotorBridgeMode,
    pub bridges: [MotorBridgePins; 3],
}

/// One quadrature encoder channel pair.
#[derive(Clone, Copy)]
pub struct EncoderChannelPins {
    pub a: u8,
    pub b: u8,
}

/// Normal drive planner selected for a board profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveControlMode {
    TriWheelOmni,
    TwoWheelFront,
}

/// High-level board profile used to select pin mappings and runtime behavior.
#[derive(Clone, Copy)]
pub struct BoardProfile {
    pub name: &'static str,
    pub family: BoardFamily,
    pub runtime: BoardRuntimeKind,
    pub i2c_bus: I2cPins,
    pub camera: CameraPins,
    pub motor: Option<MotorPins>,
    pub encoder: Option<[EncoderChannelPins; 3]>,
    pub drive: DriveTuning,
    pub drive_mode: DriveControlMode,
    pub active_wheels: [bool; 3],
    pub status_led: Option<StatusLed>,
    pub addressable_led: Option<AddressableLed>,
    pub width: u16,
    pub height: u16,
    pub jpeg_quality_qs: u8,
    pub frame_interval_ms: u16,
    pub has_camera: bool,
    pub camera_only: bool,
    pub execution_plan: RuntimeExecutionPlan,
    /// Baked-in per-wheel speed-norm trim multipliers applied at boot. The
    /// runtime `WheelTrim` command overrides at session scope. `[1.0; 3]` is
    /// the neutral default; tune per profile after HIL straight-line tests.
    pub default_wheel_trim: [f32; 3],
    /// Minimum effective forward speed-norm magnitude pushed to the motors
    /// when the operator commands any forward motion above the deadband.
    /// Designed to bypass the static-friction floor measured on this chassis.
    /// 0.0 disables the floor.
    pub min_forward_speed_norm: f32,
    /// Ceiling on the duty the drive path sends to any wheel, applied after
    /// trim. Commands above it clamp rather than scale, so the stiction floor
    /// keeps its meaning. 1.0 = no cap.
    pub max_drive_duty: f32,
    /// Duration in milliseconds of an additive PWM "kick" applied when the
    /// operator transitions from idle to commanded forward motion (or flips
    /// the direction of travel). Designed to break the chassis out of static
    /// friction on cold starts. `0` disables.
    pub stiction_kick_ms: u16,
    /// Additive boost on top of the commanded forward speed during the
    /// stiction-kick window. The result is clamped so that the total effective
    /// |speed_norm| stays within `[-1.0, 1.0]`. `0.0` disables the kick.
    pub stiction_kick_boost_norm: f32,
    /// Per-wheel phase-polarity invert flag. `true` flips the sign of the
    /// commanded speed before it hits the PWM bridge, which inverts the
    /// motor's rotation direction. Designed for chassis-reconfiguration
    /// cases like v1-alpha, where the 2-wheel front-drive layout reuses a
    /// motor that was oriented for the original 3-wheel omni positioning.
    /// All-false is the natural default; v1-alpha sets `[false, true, false]`.
    pub wheel_phase_invert: [bool; 3],
}

const POLOLU_5161_COUNTS_PER_WHEEL_REV: u16 = 12 * 50;
const EXEC_PLAN_S3_DUAL: RuntimeExecutionPlan = RuntimeExecutionPlan::dual_core(1, 0, 0);

const DEFAULT_DRIVE_TUNING: DriveTuning = DriveTuning {
    // 38 mm Nexus omni wheel => 19 mm radius. NOTE the units: metres, not mm.
    //
    // Why this is safe to change: `wheel_radius_m` and `robot_radius_m` CANCEL
    // OUT of the normalised control path. `compute_wheel_velocities` scales the
    // command by `max_linear_speed = max_wheel_rad_s * r` and the Jacobian
    // divides by `r`, so the normalised wheel output depends only on the wheel
    // *angles*. Verified numerically with r = 0.019, 0.148 and 1.0 — identical
    // output. So these values only affect genuinely physical quantities:
    //   * `max_yaw_deg_s`, hence the deg/s -> normalised yaw mapping
    //   * `max_linear_speed`, i.e. any m/s figure we report
    //   * encoder counts -> distance (odometry)
    wheel_radius_m: 0.019,
    robot_radius_m: 0.195,
    // Measured on the v1-alpha bench unit, not nominal: `probe_speed 1.0` gave
    // 4769 counts/s = 7.95 rev/s = 49.9 rad/s. The previous 25.0 was half the
    // real value, which mattered because `TriWheelPidController` normalises
    // feedback as `measured_rad_s / max_wheel_rad_s` — so `measured_norm`
    // clamped to 1.0 permanently and the PID fought its own setpoint.
    //
    // With r = 0.019 and 50.0 the derived figures finally match reality:
    // max_linear_speed = 0.95 m/s (the host-sim independently uses 0.9), and
    // max_yaw = 279 deg/s. The old pair implied 3.7 m/s and 1087 deg/s
    // (3 rev/s), neither of which this robot can do.
    max_wheel_rad_s: 50.0,
    // The one geometry input that actually changes steering (the two radii
    // above provably cancel). Carried per-board so a rotated layout is a
    // config change, not a code change.
    wheel_angles_deg: owb_core::motion::kinematics::DEFAULT_WHEEL_ANGLES_DEG,
    control_dt_ms: 50,
    motor_deadband_norm: 0.15,
    counts_per_wheel_revolution: POLOLU_5161_COUNTS_PER_WHEEL_REV,
    pid: PidGains {
        kp: 0.9,
        ki: 0.2,
        kd: 0.02,
    },
};

/// Seeed Studio XIAO ESP32S3 Sense camera profile.
pub const XIAO_SEEDSTUDIO_SENSE_CAM: BoardProfile = BoardProfile {
    name: "xiao-seedstudio-sense-cam",
    family: BoardFamily::Esp32S3,
    runtime: BoardRuntimeKind::CameraOnly,
    i2c_bus: I2cPins { sda: 40, scl: 39 },
    camera: CameraPins {
        xclk: 10,
        pclk: 13,
        vsync: 38,
        href: 47,
        d0: 15,
        d1: 17,
        d2: 18,
        d3: 16,
        d4: 14,
        d5: 12,
        d6: 11,
        d7: 48,
        pwdn: None,
        reset: None,
    },
    motor: None,
    encoder: None,
    drive: DEFAULT_DRIVE_TUNING,
    drive_mode: DriveControlMode::TriWheelOmni,
    active_wheels: [true, true, true],
    status_led: Some(StatusLed {
        pin: 21,
        active_high: true,
    }),
    addressable_led: None,
    width: 320,
    height: 240,
    jpeg_quality_qs: 0x20,
    frame_interval_ms: 66,
    has_camera: true,
    camera_only: true,
    execution_plan: EXEC_PLAN_S3_DUAL,
    default_wheel_trim: [1.0, 1.0, 1.0],
    min_forward_speed_norm: 0.0,
    max_drive_duty: 1.0,
    stiction_kick_ms: 0,
    stiction_kick_boost_norm: 0.0,
    wheel_phase_invert: [false, false, false],
};

/// ESP32-S3-DEVKITC-1-N32R16 test board profile.
///
/// This board is used as a reference ESP32-S3 platform. It does not carry the
/// integrated camera harness used by the XIAO or custom OWB boards, but we
/// keep a profile entry so the deploy flow matches the actual test hardware.
pub const ESP32_S3_DEVKITC_1_N32R16: BoardProfile = BoardProfile {
    name: "esp32-s3-devkitc-1-n32r16",
    family: BoardFamily::Esp32S3,
    runtime: BoardRuntimeKind::DevKitReference,
    i2c_bus: I2cPins { sda: 8, scl: 9 },
    camera: CameraPins {
        xclk: 0,
        pclk: 0,
        vsync: 0,
        href: 0,
        d0: 0,
        d1: 0,
        d2: 0,
        d3: 0,
        d4: 0,
        d5: 0,
        d6: 0,
        d7: 0,
        pwdn: None,
        reset: None,
    },
    motor: None,
    // DevKit encoder probe mode borrows the current v2 schematic assumption so
    // PCNT can be exercised on breadboard hardware without pulling in the
    // camera path. The final HIL map still needs confirmation.
    encoder: Some([
        EncoderChannelPins { a: 38, b: 39 },
        EncoderChannelPins { a: 40, b: 41 },
        EncoderChannelPins { a: 42, b: 43 },
    ]),
    drive: DEFAULT_DRIVE_TUNING,
    drive_mode: DriveControlMode::TriWheelOmni,
    active_wheels: [true, true, true],
    // The onboard addressable LED sits on GPIO38 on newer DevKitC-1 boards,
    // which conflicts with the provisional encoder probe harness above, so it
    // is disabled in real breadboard mode.
    status_led: None,
    addressable_led: None,
    width: 320,
    height: 240,
    jpeg_quality_qs: 0x20,
    frame_interval_ms: 66,
    has_camera: false,
    camera_only: false,
    execution_plan: EXEC_PLAN_S3_DUAL,
    default_wheel_trim: [1.0, 1.0, 1.0],
    min_forward_speed_norm: 0.0,
    max_drive_duty: 1.0,
    stiction_kick_ms: 0,
    stiction_kick_boost_norm: 0.0,
    wheel_phase_invert: [false, false, false],
};

/// First ESP32-S3 custom board layout, closest to the devkit-era camera bring-up.
pub const OWB_ESP32S3_V1: BoardProfile = BoardProfile {
    name: "owb-esp32s3-v1",
    family: BoardFamily::Esp32S3,
    runtime: BoardRuntimeKind::OwbS3V1,
    i2c_bus: I2cPins { sda: 4, scl: 5 },
    camera: CameraPins {
        xclk: 15,
        pclk: 13,
        vsync: 6,
        href: 7,
        d0: 11,
        d1: 9,
        d2: 8,
        d3: 10,
        d4: 12,
        d5: 18,
        d6: 17,
        d7: 16,
        pwdn: None,
        reset: None,
    },
    motor: Some(MotorPins {
        sleep: 45,
        mode: MotorBridgeMode::Drv8845PhaseEnable,
        bridges: [
            // v1 mapping follows the archived ESP32-S3 v1 schematic reference:
            // MCU GPIO labels, not module pin numbers.
            MotorBridgePins {
                phase: 3,
                drive: 46,
            },
            MotorBridgePins {
                phase: 14,
                drive: 21,
            },
            MotorBridgePins {
                phase: 47,
                drive: 48,
            },
        ],
    }),
    // Encoder GPIOs from the v1 schematic; sign/polarity still needs HIL.
    encoder: Some([
        EncoderChannelPins { a: 38, b: 39 },
        EncoderChannelPins { a: 40, b: 41 },
        EncoderChannelPins { a: 2, b: 1 },
    ]),
    drive: DEFAULT_DRIVE_TUNING,
    drive_mode: DriveControlMode::TriWheelOmni,
    active_wheels: [true, true, true],
    // CAM_LED is a level-shifted SK6812 chain, not a simple GPIO LED.
    status_led: None,
    addressable_led: Some(AddressableLed {
        pin: 42,
        pixels: 2,
        inverted: false,
    }),
    width: 400,
    height: 296,
    jpeg_quality_qs: 0x1c,
    frame_interval_ms: 50,
    // Disabled during v1 drive/PID bring-up so camera SCCB does not perturb
    // the motor-control and network crash isolation path.
    has_camera: false,
    camera_only: false,
    execution_plan: EXEC_PLAN_S3_DUAL,
    default_wheel_trim: [1.0, 1.0, 1.0],
    min_forward_speed_norm: 0.0,
    max_drive_duty: 1.0,
    stiction_kick_ms: 0,
    stiction_kick_boost_norm: 0.0,
    wheel_phase_invert: [false, false, false],
};

/// Second ESP32-S3 custom board using the WROOM-2 N32R16V module and newer
/// antenna-oriented routing.
/// V1 alpha deployment: W1/W2 are the product drive wheels; W3 is hard-disabled.
pub const OWB_ESP32S3_V1_ALPHA: BoardProfile = BoardProfile {
    name: "owb-esp32s3-v1-alpha",
    drive_mode: DriveControlMode::TwoWheelFront,
    active_wheels: [true, true, false],
    has_camera: true,
    camera_only: false,
    // Empirical defaults from the v1-alpha controls validation report.
    // Calibrated 2026-05-24 HIL; W1-reverse retest 2026-05-25. W1 trimmed
    // to 0.93 produces visibly straight forward tracking at speed_norm=0.6
    // against the chassis tested. W2 phase-inverted because the 2-wheel
    // front-drive reconfiguration reorients the motor relative to the
    // original 3-wheel omni mounting. Pin mapping is inherited via
    // `..OWB_ESP32S3_V1` below: W1 phase=GPIO3 (drive=GPIO46), W2
    // phase=GPIO14 (drive=GPIO21). GPIO3 is an ESP32-S3 strapping pin
    // (see docs/reports/v1_hardware_lessons.md §1) but functions cleanly
    // in both directions on this chassis; next board should still route
    // motor PHASE off any strapping pin as a defensive measure.
    default_wheel_trim: [0.93, 1.0, 1.0],
    // Measured breakaway is 0.40 duty on both wheels (`hil-check
    // --characterize`, 2026-10-23); 0.05 of margin. The old 0.55 dated from
    // when W1 was jammed by a screw.
    min_forward_speed_norm: 0.45,
    // Two wheels together run ~4000 c/s at 0.8 but drop to ~1600 at >= 0.9,
    // and a single wheel at 1.0 runs ~35% slower than at 0.95 -- most likely
    // the 1S LiPo supply sagging under load. Nothing above 0.8 is faster with
    // both wheels driving, so cap there.
    max_drive_duty: 0.8,
    // Startup PWM boost so cold-start commands above the deadband but near the
    // stiction floor still break the wheels free reliably. Pulse expires after
    // ~120 ms and the runtime reverts to the operator's commanded speed.
    stiction_kick_ms: 120,
    stiction_kick_boost_norm: 0.25,
    wheel_phase_invert: [false, true, false],
    // 160x120 YUV422 fits in the 40KB DMA buffer (38,400 bytes).
    width: 160,
    height: 120,
    jpeg_quality_qs: 0x3c,
    ..OWB_ESP32S3_V1
};

/// **PLACEHOLDER — every pin in this profile is wrong and must be redone.**
///
/// The v2 board is schematic-only and its structure is not settled, so this map
/// was never derived from a real layout. Treat the numbers as filler that exists
/// only to keep the profile table compiling; do not wire anything to them and do
/// not use them as a starting point for the real map.
///
/// Known-bad today (the boot conflict checker reports all of these):
/// `GPIO4` sda+motor3.phase, `GPIO38` motor1.drive+enc1.a,
/// `GPIO39` motor2.phase+enc1.b, `GPIO40` motor3.drive+enc2.a,
/// `GPIO42` enc3.a+addressable_led. The camera pins happen not to collide,
/// which is luck rather than design.
///
/// When the v2 structure is finalised, this whole block gets replaced --
/// including re-deriving `drive_mode`, `active_wheels`, and
/// `wheel_phase_invert`, none of which have seen hardware.
pub const OWB_ESP32S3_V2: BoardProfile = BoardProfile {
    name: "owb-esp32s3-v2",
    family: BoardFamily::Esp32S3,
    runtime: BoardRuntimeKind::OwbS3V2,
    i2c_bus: I2cPins { sda: 4, scl: 5 },
    camera: CameraPins {
        xclk: 15,
        pclk: 16,
        vsync: 6,
        href: 7,
        d0: 12,
        d1: 11,
        d2: 10,
        d3: 9,
        d4: 3,
        d5: 8,
        d6: 18,
        d7: 17,
        pwdn: None,
        reset: None,
    },
    motor: Some(MotorPins {
        sleep: 47,
        mode: MotorBridgeMode::PhaseEnable,
        bridges: [
            // v2 follows the rev7-style routing where the first bridge remains
            // on the low GPIO bank and the later bridges move to the upper bank.
            MotorBridgePins {
                phase: 2,
                drive: 38,
            },
            MotorBridgePins {
                phase: 39,
                drive: 21,
            },
            MotorBridgePins {
                phase: 4,
                drive: 40,
            },
        ],
    }),
    // Assumed from the current schematic; sign/polarity still needs HIL.
    encoder: Some([
        EncoderChannelPins { a: 38, b: 39 },
        EncoderChannelPins { a: 40, b: 41 },
        EncoderChannelPins { a: 42, b: 43 },
    ]),
    drive: DEFAULT_DRIVE_TUNING,
    drive_mode: DriveControlMode::TriWheelOmni,
    active_wheels: [true, true, true],
    // CAM_LED is a level-shifted SK6812 chain, not a simple GPIO LED.
    status_led: None,
    addressable_led: Some(AddressableLed {
        pin: 42,
        pixels: 2,
        inverted: false,
    }),
    width: 320,
    height: 240,
    jpeg_quality_qs: 0x3c,
    frame_interval_ms: 50,
    has_camera: true,
    camera_only: false,
    execution_plan: EXEC_PLAN_S3_DUAL,
    default_wheel_trim: [1.0, 1.0, 1.0],
    min_forward_speed_norm: 0.0,
    max_drive_duty: 1.0,
    stiction_kick_ms: 0,
    stiction_kick_boost_norm: 0.0,
    wheel_phase_invert: [false, false, false],
};

pub const PROFILES: &[BoardProfile] = &[
    XIAO_SEEDSTUDIO_SENSE_CAM,
    ESP32_S3_DEVKITC_1_N32R16,
    OWB_ESP32S3_V1,
    OWB_ESP32S3_V1_ALPHA,
    OWB_ESP32S3_V2,
];

fn apply_devkit_overrides(
    mut profile: BoardProfile,
    motor: Option<MotorPins>,
    encoder: Option<[EncoderChannelPins; 3]>,
    led_pin: Option<u8>,
) -> BoardProfile {
    let motor_override_active = motor.is_some();
    let encoder_override_active = encoder.is_some();

    if let Some(motor) = motor {
        profile.motor = Some(motor);
    }

    if let Some(encoder) = encoder {
        profile.encoder = Some(encoder);
    }

    if motor_override_active && !encoder_override_active {
        // For DevKit breadboard bring-up, a motor-only override is the safest
        // default. The placeholder encoder probe map reuses upper GPIOs that
        // commonly overlap with the DRV harness, so disable encoder capture
        // until the real quadrature wiring is stated explicitly.
        profile.encoder = None;
    }

    if let Some(pin) = led_pin {
        if !motor_override_active && !encoder_override_active {
            // LED-only DevKit smoke tests should not inherit the provisional
            // encoder probe map because it reuses GPIO38 on newer boards.
            profile.encoder = None;
        }
        profile.addressable_led = Some(AddressableLed {
            pin,
            pixels: 1,
            inverted: false,
        });
    }

    profile
}

fn devkit_profile_with_overrides() -> &'static BoardProfile {
    macro_rules! env_pin {
        ($name:literal) => {
            option_env!($name).and_then(|value| value.parse::<u8>().ok())
        };
    }

    let motor = if let (
        Some(sleep),
        Some(phase1),
        Some(drive1),
        Some(phase2),
        Some(drive2),
        Some(phase3),
        Some(drive3),
    ) = (
        env_pin!("OWB_DEVKIT_MTR_SLEEP"),
        env_pin!("OWB_DEVKIT_MTR1_PHASE"),
        env_pin!("OWB_DEVKIT_MTR1_DRIVE"),
        env_pin!("OWB_DEVKIT_MTR2_PHASE"),
        env_pin!("OWB_DEVKIT_MTR2_DRIVE"),
        env_pin!("OWB_DEVKIT_MTR3_PHASE"),
        env_pin!("OWB_DEVKIT_MTR3_DRIVE"),
    ) {
        Some(MotorPins {
            sleep,
            // The current DevKit breadboard harness uses DRV8874 carriers in
            // PH/EN mode, so keep the local override aligned with the custom
            // S3 board semantics.
            mode: MotorBridgeMode::PhaseEnable,
            bridges: [
                MotorBridgePins {
                    phase: phase1,
                    drive: drive1,
                },
                MotorBridgePins {
                    phase: phase2,
                    drive: drive2,
                },
                MotorBridgePins {
                    phase: phase3,
                    drive: drive3,
                },
            ],
        })
    }
    else {
        None
    };

    let encoder = if let (Some(a1), Some(b1), Some(a2), Some(b2), Some(a3), Some(b3)) = (
        env_pin!("OWB_DEVKIT_ENC1_A"),
        env_pin!("OWB_DEVKIT_ENC1_B"),
        env_pin!("OWB_DEVKIT_ENC2_A"),
        env_pin!("OWB_DEVKIT_ENC2_B"),
        env_pin!("OWB_DEVKIT_ENC3_A"),
        env_pin!("OWB_DEVKIT_ENC3_B"),
    ) {
        Some([
            EncoderChannelPins { a: a1, b: b1 },
            EncoderChannelPins { a: a2, b: b2 },
            EncoderChannelPins { a: a3, b: b3 },
        ])
    }
    else {
        None
    };

    let profile = apply_devkit_overrides(
        ESP32_S3_DEVKITC_1_N32R16,
        motor,
        encoder,
        env_pin!("OWB_DEVKIT_LED_PIN"),
    );

    static DEVKIT_PROFILE: StaticCell<BoardProfile> = StaticCell::new();
    DEVKIT_PROFILE.uninit().write(profile)
}

/// Select a profile using `OWB_IO_PROFILE`, defaulting to the current v2
/// custom board.
pub fn select_profile() -> &'static BoardProfile {
    let wanted = option_env!("OWB_IO_PROFILE").unwrap_or(OWB_ESP32S3_V2.name);

    if wanted == ESP32_S3_DEVKITC_1_N32R16.name {
        return devkit_profile_with_overrides();
    }

    PROFILES
        .iter()
        .find(|profile| profile.name == wanted)
        .unwrap_or(&OWB_ESP32S3_V2)
}

#[cfg(test)]
mod tests {
    use owb_core::platform::CpuTopology;

    use super::*;

    #[test]
    fn custom_s3_profiles_use_pololu_encoder_counts() {
        assert_eq!(OWB_ESP32S3_V1.drive.counts_per_wheel_revolution, 600);
        assert_eq!(OWB_ESP32S3_V1_ALPHA.drive.counts_per_wheel_revolution, 600);
        assert_eq!(OWB_ESP32S3_V2.drive.counts_per_wheel_revolution, 600);
    }

    #[test]
    fn custom_s3_profiles_define_assumed_encoder_maps() {
        assert!(OWB_ESP32S3_V1.encoder.is_some());
        assert!(OWB_ESP32S3_V1_ALPHA.encoder.is_some());
        assert!(OWB_ESP32S3_V2.encoder.is_some());
    }

    /// The stiction floor must still break the wheels free (breakaway 0.40)
    /// and sit under the duty cap, or every low command would clamp to it.
    #[test]
    fn v1_alpha_floor_sits_between_breakaway_and_duty_cap() {
        let p = OWB_ESP32S3_V1_ALPHA;
        assert!(p.min_forward_speed_norm > 0.40);
        assert!(p.min_forward_speed_norm < p.max_drive_duty);
        assert!(p.max_drive_duty <= 0.8, "two wheels collapse above 0.8 duty");
    }

    #[test]
    fn v1_alpha_uses_two_front_wheel_drive_mode() {
        assert_eq!(
            OWB_ESP32S3_V1_ALPHA.drive_mode,
            DriveControlMode::TwoWheelFront
        );
        assert_eq!(OWB_ESP32S3_V1_ALPHA.active_wheels, [true, true, false]);
        assert_eq!(
            OWB_ESP32S3_V1_ALPHA.motor.expect("motor map").bridges[2].phase,
            47
        );
        assert!(OWB_ESP32S3_V1_ALPHA.has_camera);
    }

    #[test]
    fn drive_tuning_keeps_control_terms_positive() {
        for profile in PROFILES {
            assert!(profile.drive.control_dt_ms > 0);
            assert!(profile.drive.max_wheel_rad_s > 0.0);
            assert!(profile.drive.pid.kp >= 0.0);
            assert!(profile.drive.pid.ki >= 0.0);
            assert!(profile.drive.pid.kd >= 0.0);
        }
    }

    #[test]
    fn esp32s3_profiles_define_dual_core_execution_policy() {
        for profile in PROFILES {
            assert_eq!(profile.execution_plan.topology, CpuTopology::DualCore);
            assert_eq!(profile.execution_plan.control_cpu, 1);
            assert_eq!(profile.execution_plan.network_cpu, 0);
        }
    }

    #[test]
    fn devkit_motor_override_uses_phase_enable_mode() {
        let profile = apply_devkit_overrides(
            ESP32_S3_DEVKITC_1_N32R16,
            Some(MotorPins {
                sleep: 45,
                mode: MotorBridgeMode::PhaseEnable,
                bridges: [
                    MotorBridgePins {
                        phase: 2,
                        drive: 46,
                    },
                    MotorBridgePins {
                        phase: 13,
                        drive: 21,
                    },
                    MotorBridgePins {
                        phase: 47,
                        drive: 48,
                    },
                ],
            }),
            None,
            Some(38),
        );

        let motor = profile.motor.expect("motor override should be present");
        assert_eq!(motor.mode, MotorBridgeMode::PhaseEnable);
        assert!(profile.encoder.is_none());
        assert_eq!(profile.addressable_led.expect("led override").pin, 38);
    }
}
