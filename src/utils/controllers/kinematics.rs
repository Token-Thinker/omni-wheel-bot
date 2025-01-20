use core::f32::consts::PI;

use libm;

/// Represents the kinematics of an omni-wheel robot.
///
/// This structure models the geometric configuration and kinematic properties
/// of a three-wheeled omni-wheel robot, including wheel positioning, robot
/// size, and methods for computing velocities and constructing a Jacobian
/// matrix.
pub struct WheelKinematics {
    /// Radius of each wheel in meters.
    wheel_radius: f32,
    /// Distance from the robot's center to each wheel in meters.
    robot_radius: f32,
    /// Angular positions of the wheels in radians, measured counterclockwise
    /// from the x-axis.
    wheel_angles: [f32; 3],
}

impl WheelKinematics {
    /// Creates a new `WheelKinematics` instance.
    ///
    /// # Parameters
    /// - `wheel_radius`: The radius of the robot's omni-wheels in meters.
    /// - `robot_radius`: The distance from the robot's center to each wheel in
    ///   meters.
    ///
    /// # Returns
    /// A new `WheelKinematics` instance with predefined wheel angles.
    pub fn new(
        wheel_radius: f32,
        robot_radius: f32,
    ) -> Self {
        let wheel_angles = [PI / 3.0, PI, 5.0 * PI / 3.0];
        Self {
            wheel_radius,
            robot_radius,
            wheel_angles,
        }
    }

    /// Converts a global velocity vector into the robot's body frame.
    ///
    /// # Parameters
    /// - `speed`: Linear speed in meters per second.
    /// - `angle`: Direction of movement in degrees, relative to the global
    ///   frame.
    /// - `orientation`: Robot's current orientation in degrees, relative to the
    ///   global frame.
    ///
    /// # Returns
    /// A tuple `(vx, vy)` representing the velocity components in the robot's
    /// body frame.
    pub fn convert_to_body_frame(
        speed: f32,
        angle: f32,
        orientation: f32,
    ) -> (f32, f32) {
        let angle_rad = angle * (PI / 180.0);
        let orientation_rad = orientation * (PI / 180.0);

        let v_bx = speed * libm::cosf(angle_rad - orientation_rad);
        let v_by = speed * libm::sinf(angle_rad - orientation_rad);

        (-v_by, v_bx)
    }

    /// Constructs the Jacobian matrix for the robot's kinematics.
    ///
    /// The Jacobian matrix relates the robot's body-frame velocity
    /// components to the wheel velocities:
    ///
    /// ```text
    /// J[i, 0] = cos(theta_i)/R
    /// J[i, 1] = sin(theta_i)/R
    /// J[i, 2] = L/R
    /// ```
    ///
    /// # Returns
    /// A 3x3 Jacobian matrix relating robot body-frame velocities to wheel
    /// velocities.
    pub fn construct_jacobian(&self) -> [[f32; 3]; 3] {
        let r = self.wheel_radius;
        let l = self.robot_radius;

        let mut j = [[0.0_f32; 3]; 3];
        for (i, &angle) in self.wheel_angles.iter().enumerate() {
            j[i][0] = libm::cosf(angle) / r;
            j[i][1] = libm::sinf(angle) / r;
            j[i][2] = l / r;
        }
        j
    }

    /// Computes the required wheel velocities based on the desired robot
    /// movement.
    ///
    /// # Parameters
    /// - `speed`: Linear speed in meters per second.
    /// - `angle`: Movement direction in degrees, relative to the global frame.
    /// - `orientation`: Robot's orientation in degrees, relative to the global
    ///   frame.
    /// - `omega`: Angular velocity in radians per second.
    ///
    /// # Returns
    /// An array of wheel velocities `[v1, v2, v3]`, where each value
    /// corresponds to the rotational velocity of an individual wheel in
    /// radians per second.
    pub fn compute_wheel_velocities(
        &self,
        speed: f32,
        angle: f32,
        orientation: f32,
        omega: f32,
    ) -> [f32; 3] {
        let (v_bx, v_by) = Self::convert_to_body_frame(speed, angle, orientation);
        let v = [v_bx, v_by, omega];
        let j = self.construct_jacobian();

        let mut wheel_velocities = [0.0_f32; 3];

        /// Clamps near-zero values to zero to reduce numerical noise.
        fn clamp_small(
            value: f32,
            epsilon: f32,
        ) -> f32 {
            if value.abs() < epsilon {
                0.0
            } else {
                value
            }
        }

        for i in 0..3 {
            wheel_velocities[i] =
                clamp_small(j[i][0] * v[0] + j[i][1] * v[1] + j[i][2] * v[2], 1e-6);
        }
        wheel_velocities
    }
}
