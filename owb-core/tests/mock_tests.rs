use owb_core::{
    ctrl::{sysctrl::I2CCommand, Imu, ImuData, Lights, MotorPwm, SystemController},
    motion::kinematics::EmbodiedKinematics,
};

struct MockPwm {
    last: [f32; 3],
    enabled: bool,
}
impl MotorPwm for MockPwm {
    type Error = ();
    fn enable(&mut self) -> Result<(), Self::Error> {
        self.enabled = true;
        Ok(())
    }
    fn disable(&mut self) -> Result<(), Self::Error> {
        self.enabled = false;
        Ok(())
    }
    fn set_wheel(
        &mut self,
        index: usize,
        speed: f32,
    ) -> Result<(), Self::Error> {
        self.last[index] = speed;
        Ok(())
    }
}

struct MockImu;
impl Imu for MockImu {
    type Error = ();
    fn read(&mut self) -> Result<ImuData, Self::Error> { Ok(ImuData::default()) }
}

struct DummyLights;
impl Lights for DummyLights {
    type Error = ();
    fn set_brightness(
        &mut self,
        _value: u8,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn test_controller_handles_zero_motion() {
    let mut ctrl = SystemController::new(
        MockPwm {
            last: [1.0; 3],
            enabled: false,
        },
        MockImu,
        DummyLights,
        0.148,
        0.195,
        7.0,
    );
    let _ = ctrl.handle_i2c(I2CCommand::T {
        direction_deg: 0.0,
        speed_norm: 0.0,
    });
    assert_eq!(ctrl.pwm.last, [0.0, 0.0, 0.0]);
}

#[test]
fn wheel_velocities_nonzero() {
    let kin = EmbodiedKinematics::new(0.148, 0.195, 7.0);
    let wheels = kin.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);
    assert!(wheels.iter().any(|&v| v != 0.0));
}

#[test]
fn example_embodied_kinematics() {
    let ek = EmbodiedKinematics::new(0.148, 0.195, 7.0);
    let wheels = ek.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);
    assert!(wheels.iter().any(|&v| v != 0.0));
}

#[test]
fn yaw_command_spins_in_place() {
    let mut ctrl = SystemController::new(
        MockPwm {
            last: [0.0; 3],
            enabled: true,
        },
        MockImu,
        DummyLights,
        0.148,
        0.195,
        7.0,
    );
    let _ = ctrl.handle_i2c(I2CCommand::Y {
        yaw_rate_deg_s: 45.0,
        orientation_deg: Some(0.0),
    });
    assert!(ctrl.pwm.last.iter().all(|&v| v.abs() > 0.0));
    let sign = ctrl.pwm.last[0].signum();
    for &v in &ctrl.pwm.last {
        assert_eq!(v.signum(), sign);
        assert!(v.abs() <= 1.0 + 1e-6);
    }
    let rad = ctrl.embodied.wheel_norm_to_rad_s(ctrl.pwm.last);
    assert!(rad.iter().all(|&v| v.abs() <= 7.0 + 1e-6));
}

#[test]
fn disable_command_zeroes_wheels() {
    let mut ctrl = SystemController::new(
        MockPwm {
            last: [0.5, -0.3, 0.1],
            enabled: true,
        },
        MockImu,
        DummyLights,
        0.148,
        0.195,
        7.0,
    );
    let _ = ctrl.handle_i2c(I2CCommand::Disable);
    assert_eq!(ctrl.pwm.last, [0.0, 0.0, 0.0]);
    assert!(!ctrl.pwm.enabled);
}

#[test]
fn omni_command_uses_reported_orientation() {
    let mut ctrl = SystemController::new(
        MockPwm {
            last: [0.0; 3],
            enabled: true,
        },
        MockImu,
        DummyLights,
        0.148,
        0.195,
        7.0,
    );
    let speed = 0.6;
    let direction = 15.0;
    let orientation = 22.0;
    let yaw_rate = 45.0;
    let _ = ctrl.handle_i2c(I2CCommand::O {
        direction_deg: direction,
        speed_norm: speed,
        yaw_rate_deg_s: yaw_rate,
        orientation_deg: Some(orientation),
    });
    let mut orientation_norm = orientation % 360.0;
    if !orientation_norm.is_finite() {
        orientation_norm = 0.0;
    }
    else if orientation_norm < 0.0 {
        orientation_norm += 360.0;
    }
    let expected = ctrl.embodied.compute_wheel_velocities(
        speed.clamp(-1.0, 1.0),
        direction,
        orientation_norm,
        ctrl.embodied.normalize_yaw_rate_deg(yaw_rate),
    );
    for (actual, exp) in ctrl.pwm.last.iter().zip(expected.iter()) {
        assert!((actual - exp).abs() < 1e-6);
    }
}
