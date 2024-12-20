use core::f32::consts::PI;
use libm;
use micromath::F32Ext;

pub struct WheelKinematics {
    /// Radius of each wheel (m)
    wheel_radius: f32,
    /// Radius from center of the robot to each wheel (m)
    robot_radius: f32,
    wheel_angles: [f32; 3],
}
impl WheelKinematics {
    /// Create a new 'WheelKinematics' instance
    ///
    /// # Parameters
    /// - 'wheel_radius': The radius of an individual onmi-wheel.
    /// - 'base_radius': The radius of the circle on which the wheels lie.

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

    pub fn convert_to_body_frame(
        speed: f32,
        angle: f32,
        orientation: f32,
    ) -> (f32, f32) {
        let angle_rad = angle * (PI / 180.0);
        let orientation_rad = orientation * (PI / 180.0);

        let v_bx_0 = speed * libm::cosf(angle_rad - orientation_rad);
        let v_by_0 = speed * libm::sinf(angle_rad - orientation_rad);

        (-v_by_0, v_bx_0)
    }

    /// Constructs the Jacobian matrix:
    /// J[i, 0] = cos(theta_i)/R
    /// J[i, 1] = sin(theta_i)/R
    /// J[i, 2] = L/R
    ///
    /// For each wheel i:
    ///   θ_i = wheel_angles[i]
    ///   R = wheel_radius
    ///   L = robot_radius
    ///
    /// Returns a 3x3 matrix:
    /// [
    ///   [cos(θ1)/R, sin(θ1)/R, L/R],
    ///   [cos(θ2)/R, sin(θ2)/R, L/R],
    ///   [cos(θ3)/R, sin(θ3)/R, L/R]
    /// ]

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

    pub fn compute_wheel_positions(
        &self,
        orientation: f32,
    ) -> [(f32, f32); 3] {
        let orientation_rad = orientation * (PI / 180.0);
        let mut positions = [(0.0, 0.0); 3];

        for (i, &angle) in self.wheel_angles.iter().enumerate() {
            let rotated_angle = angle + orientation_rad;
            let x = self.robot_radius * libm::cosf(rotated_angle);
            let y = self.robot_radius * libm::sinf(rotated_angle);
            positions[i] = (x, y);
        }
        positions
    }

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
