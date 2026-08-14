#[cfg(feature = "real-executors")]
use esp_hal::gpio::{AnyPin, Input, InputConfig};
use esp_hal::{i2c::master::I2c, Blocking};
use log::{info, warn};
use owb_core::{
    drivers::{
        hdc2010::{Hdc2010, DEFAULT_ADDR_HIGH as HDC_ADDR_HIGH, DEFAULT_ADDR_LOW as HDC_ADDR_LOW},
        lis2dh12::{
            Lis2dh12,
            DEFAULT_ADDR_HIGH as LIS_ADDR_HIGH,
            DEFAULT_ADDR_LOW as LIS_ADDR_LOW,
            WHO_AM_I_RESPONSE,
        },
        max17048::{Max17048, DEFAULT_ADDR as MAX17048_ADDR},
        tca9555::{Tca9555, DEFAULT_ADDR as TCA9555_DEFAULT_ADDR},
        tof::{bringup_sequence, DEFAULT_TOF_CHANNELS},
    },
    platform::BoardRuntimeKind,
};

use super::{diagnostics::log_profile_wiring_diagnostics, BoardCapabilities};
use crate::profile::BoardProfile;

const TCA9555_ADDR_CANDIDATES: [u8; 8] = [
    TCA9555_DEFAULT_ADDR,
    TCA9555_DEFAULT_ADDR + 1,
    TCA9555_DEFAULT_ADDR + 2,
    TCA9555_DEFAULT_ADDR + 3,
    TCA9555_DEFAULT_ADDR + 4,
    TCA9555_DEFAULT_ADDR + 5,
    TCA9555_DEFAULT_ADDR + 6,
    TCA9555_DEFAULT_ADDR + 7,
];

pub fn boot_probe(
    i2c: &mut I2c<'_, Blocking>,
    profile: &BoardProfile,
) -> BoardCapabilities {
    let mut caps = BoardCapabilities::for_profile(profile);
    log_profile_wiring_diagnostics(profile, &caps);
    match profile.runtime {
        BoardRuntimeKind::OwbS3V1 | BoardRuntimeKind::OwbS3V2 => {
            if let Some(motor) = profile.motor {
                #[cfg(feature = "real-executors")]
                log_motor_sleep_boot_level(motor.sleep);
                info!(
                    "board probe: drv8845 sleep={} bridges=[({},{}) ({},{}) ({},{})]",
                    motor.sleep,
                    motor.bridges[0].phase,
                    motor.bridges[0].drive,
                    motor.bridges[1].phase,
                    motor.bridges[1].drive,
                    motor.bridges[2].phase,
                    motor.bridges[2].drive
                );
            }
            let tof_steps = bringup_sequence(&DEFAULT_TOF_CHANNELS);
            info!(
                "board probe: tof addrs=[0x{:02X},0x{:02X},0x{:02X}] lpn_steps=[({:#04X},{:#04X}) ({:#04X},{:#04X}) ({:#04X},{:#04X})]",
                DEFAULT_TOF_CHANNELS[0].target_addr,
                DEFAULT_TOF_CHANNELS[1].target_addr,
                DEFAULT_TOF_CHANNELS[2].target_addr,
                tof_steps[0].port0,
                tof_steps[0].port1,
                tof_steps[1].port0,
                tof_steps[1].port1,
                tof_steps[2].port0,
                tof_steps[2].port1
            );
            info!(
                "board probe: drive wheel_r={:.3}m robot_r={:.3}m max_wheel={:.2}rad/s dt={}ms deadband={:.2} counts/rev={} pid=({:.2},{:.2},{:.2})",
                profile.drive.wheel_radius_m,
                profile.drive.robot_radius_m,
                profile.drive.max_wheel_rad_s,
                profile.drive.control_dt_ms,
                profile.drive.motor_deadband_norm,
                profile.drive.counts_per_wheel_revolution,
                profile.drive.pid.kp,
                profile.drive.pid.ki,
                profile.drive.pid.kd
            );
            if let Some(encoders) = profile.encoder {
                info!(
                    "board probe: encoder map=[({},{}) ({},{}) ({},{})]",
                    encoders[0].a,
                    encoders[0].b,
                    encoders[1].a,
                    encoders[1].b,
                    encoders[2].a,
                    encoders[2].b
                );
            }
            else {
                info!("board probe: encoder map pending HIL confirmation");
            }
            if let Some(led) = profile.addressable_led {
                info!(
                    "board probe: addressable_led pin={} pixels={} inverted={}",
                    led.pin, led.pixels, led.inverted
                );
            }
            caps.tca9555_addr = probe_tca9555(i2c);
            caps.has_tca9555 = caps.tca9555_addr.is_some();
            caps.has_lis2dh12 = probe_lis2dh12(i2c);
            caps.has_hdc2010 = probe_hdc2010(i2c);
            caps.has_max17048 = probe_max17048(i2c);
        }
        BoardRuntimeKind::DevKitReference => {
            if let Some(motor) = profile.motor {
                info!(
                    "board probe: devkit motor map sleep={} bridges=[({},{}) ({},{}) ({},{})]",
                    motor.sleep,
                    motor.bridges[0].phase,
                    motor.bridges[0].drive,
                    motor.bridges[1].phase,
                    motor.bridges[1].drive,
                    motor.bridges[2].phase,
                    motor.bridges[2].drive
                );
            }
            if let Some(encoders) = profile.encoder {
                info!(
                    "board probe: encoder map=[({},{}) ({},{}) ({},{})]",
                    encoders[0].a,
                    encoders[0].b,
                    encoders[1].a,
                    encoders[1].b,
                    encoders[2].a,
                    encoders[2].b
                );
            }
            if let Some(led) = profile.addressable_led {
                info!(
                    "board probe: addressable_led pin={} pixels={} inverted={}",
                    led.pin, led.pixels, led.inverted
                );
            }
            info!("board probe: devkit reference runtime (no shared peripherals expected)");
        }
        _ => {}
    }

    info!(
        "board probe summary: camera={} motion={} status_led={} addressable_led={} tca9555={} lis2dh12={} hdc2010={} max17048={}",
        caps.has_camera,
        caps.has_motion_driver,
        caps.has_status_led,
        caps.has_addressable_led,
        caps.has_tca9555,
        caps.has_lis2dh12,
        caps.has_hdc2010,
        caps.has_max17048
    );

    caps
}

#[cfg(feature = "real-executors")]
fn log_motor_sleep_boot_level(pin: u8) {
    let input = Input::new(unsafe { AnyPin::steal(pin) }, InputConfig::default());
    let level = if input.is_high() { "high" } else { "low" };
    info!("board probe: motor sleep GPIO{pin} sampled_level={level} before output init");
}

fn probe_tca9555(i2c: &mut I2c<'_, Blocking>) -> Option<u8> {
    for addr in TCA9555_ADDR_CANDIDATES {
        if let Ok([p0, p1]) = Tca9555::new(&mut *i2c, addr).read_inputs() {
            info!("board probe: TCA9555 @0x{addr:02X} inputs=[0x{p0:02X},0x{p1:02X}]");
            return Some(addr);
        }
    }

    warn!("board probe: TCA9555 missing in 0x20..0x27");
    None
}

fn probe_lis2dh12(i2c: &mut I2c<'_, Blocking>) -> bool {
    for addr in [LIS_ADDR_LOW, LIS_ADDR_HIGH] {
        let mut imu = Lis2dh12::new(&mut *i2c, addr);
        match imu.who_am_i() {
            Ok(who) => {
                info!(
                    "board probe: LIS2DH12 @0x{addr:02X} whoami=0x{who:02X} expected=0x{:02X}",
                    WHO_AM_I_RESPONSE
                );
                return true;
            }
            Err(_) => {}
        }
    }
    warn!("board probe: LIS2DH12 missing on 0x18/0x19");
    false
}

fn probe_hdc2010(i2c: &mut I2c<'_, Blocking>) -> bool {
    for addr in [HDC_ADDR_LOW, HDC_ADDR_HIGH] {
        let mut sensor = Hdc2010::new(&mut *i2c, addr);
        if let (Ok(temp), Ok(humidity)) =
            (sensor.read_temperature_c(), sensor.read_humidity_percent())
        {
            info!("board probe: HDC2010 @0x{addr:02X} temp={temp:.1}C humidity={humidity:.1}%");
            return true;
        }
    }
    warn!("board probe: HDC2010 missing on 0x40/0x41");
    false
}

fn probe_max17048(i2c: &mut I2c<'_, Blocking>) -> bool {
    let mut gauge = Max17048::new(&mut *i2c, MAX17048_ADDR);
    match (
        gauge.read_version(),
        gauge.read_cell_voltage_mv(),
        gauge.read_soc_percent(),
    ) {
        (Ok(version), Ok(mv), Ok(soc)) => info!(
            "board probe: MAX17048 @0x36 version=0x{version:04X} vcell={}mV soc={:.1}%",
            mv, soc
        ),
        _ => {
            warn!("board probe: MAX17048 @0x36 missing");
            return false;
        }
    }
    true
}
