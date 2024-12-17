use core::cell::RefCell;
use utils::controllers::{I2CDevices, WheelKinematics};

mod mock_devices;
use mock_devices::*;

#[test]
fn wheel_velocities() {
    let wkin = WheelKinematics::new(0.148, 0.195);
    let w = wkin.compute_wheel_velocities(1.0, 0.0, 0.0, 0.0);

    #[cfg(feature = "std")]
    println!("Wheel velocities: {:?}", w);

    assert!(w[0] > 3.0);
}

/*#[test]
fn i2cdevices_init() {
    // Define only the initialization-related transactions
    let expectations = [
        write_read(IMU_ADDRESS, vec![0x75], vec![0x67]),
        write_read(IMU_ADDRESS, vec![0x21], vec![0x00]),
        write(IMU_ADDRESS, vec![0x21, 0x00]),
        write_read(IMU_ADDRESS, vec![0x20], vec![0x00]),
        write(IMU_ADDRESS, vec![0x20, 0x00]),
        write_read(IMU_ADDRESS, vec![0x1F], vec![0x0F]),
        write(IMU_ADDRESS, vec![0x1F, 0x0F]),
        write(PWM_ADDRESS, vec![0x00, 0x01]),
        write(PWM_ADDRESS, vec![0x00, 0x11]),
        write(PWM_ADDRESS, vec![0xFE, 0x03]),
        write(PWM_ADDRESS, vec![0x00, 0x01]),
        write(PWM_ADDRESS, vec![0x00, 0x21]),
        write(
            PWM_ADDRESS,
            vec![
                0x06, // Starting register: LED0_ON_L
                0x00, 0x00, 0xFF, 0x0F, // LED0 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED1 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED2 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED3 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED4 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED5 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED6 ON and OFF
                0x00, 0x00, 0xFF, 0x0F, // LED7 ON and OFF
                0x00, 0x00, 0x00, 0x00, // LED8 OFF
                0x00, 0x00, 0x00, 0x00, // LED9 OFF
                0x00, 0x00, 0x00, 0x00, // LED10 OFF
                0x00, 0x00, 0x00, 0x00, // LED11 OFF
                0x00, 0x00, 0x00, 0x00, // LED12 OFF
                0x00, 0x00, 0x00, 0x00, // LED13 OFF
                0x00, 0x00, 0x00, 0x00, // LED14 OFF
                0x00, 0x00, 0x00, 0x00, // LED15 OFF
            ],
        ),
        // Motor 0 (C6, C7)
        write(PWM_ADDRESS, vec![0x1E, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x22, 0x00, 0x00, 0xFF, 0x0F]),
        // Motor 1 (C2, C3)
        write(PWM_ADDRESS, vec![0x0E, 0x00, 0x00, 0x00, 0x00]),
        write(PWM_ADDRESS, vec![0x12, 0x00, 0x00, 0x00, 0x00]),
        // Motor 2 (C4, C5)
        write(PWM_ADDRESS, vec![0x16, 0x00, 0x00, 0xFF, 0x0F]),
        write(PWM_ADDRESS, vec![0x1A, 0x00, 0x00, 0xFF, 0x0F]),
    ];

    // Initialize the mock with expectations
    let mock = I2cMock::new(&expectations);
    let static_i2c = I2C_CELL.init(RefCell::new(mock));

    // Test `I2CDevices::new()`
    let mut devices = I2CDevices::new(static_i2c, 0.148, 0.195).unwrap();

    devices
        .set_motor_velocities(0.0, 1.0, false, None)
        .expect("Failed to set motor velocities");

    // Verify expectations
    static_i2c.borrow_mut().done();
}
*/