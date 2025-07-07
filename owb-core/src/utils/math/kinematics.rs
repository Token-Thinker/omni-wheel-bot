#[allow(dead_code)]
use core::f32::consts::PI;
use libm;

/// Represents the kinematics of a 3-wheeled omni-wheel robot
pub struct EmbodiedKinematics {
    /// Radius of each wheel (m)
    wheel_radius: f32,
    /// Robot center-to-wheel distance (m)
    robot_radius: f32,
    /// Precomputed wheel mounting angles (rad)
    wheel_angles: [f32; 3],
}

impl EmbodiedKinematics {
    /// Instantiate with a given wheel and robot radii.
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

    /// Transform global vector (speed, angle) into body-frame (vx, vy)
    pub fn convert_to_body_frame(
        speed: f32,
        angle: f32,
        orientation: f32,
    ) -> (f32, f32) {
        let a = angle * (PI / 180.0);
        let o = orientation * (PI / 180.0);
        let vx = speed * libm::cosf(a - o);
        let vy = speed * libm::sinf(a - o);
        (-vy, vx)
    }

    /// Build Jacobian J such that ω_wheels = J * [vx, vy, ω_body]
    pub fn construct_jacobian(&self) -> [[f32; 3]; 3] {
        let r = self.wheel_radius;
        let l = self.robot_radius;
        let mut j = [[0.0; 3]; 3];
        for (i, &t) in self.wheel_angles.iter().enumerate() {
            j[i][0] = libm::cosf(t) / r;
            j[i][1] = libm::sinf(t) / r;
            j[i][2] = l / r;
        }
        j
    }

    /// Invert measured wheel speeds back to body-frame velocity
    pub fn compute_body_velocity(
        &self,
        wheel_velocity: [f32; 3],
    ) -> (f32, f32, f32) {
        let j = self.construct_jacobian();
        let inv = invert_3x3(j);
        let vx = inv[0][0] * wheel_velocity[0]
            + inv[0][1] * wheel_velocity[1]
            + inv[0][2] * wheel_velocity[2];
        let vy = inv[1][0] * wheel_velocity[0]
            + inv[1][1] * wheel_velocity[1]
            + inv[1][2] * wheel_velocity[2];
        let w = inv[2][0] * wheel_velocity[0]
            + inv[2][1] * wheel_velocity[1]
            + inv[2][2] * wheel_velocity[2];
        (vx, vy, w)
    }

    /// Compute individual wheel speeds given desired motion
    pub fn compute_wheel_velocities(
        &self,
        speed: f32,
        angle: f32,
        orientation: f32,
        omega: f32,
    ) -> [f32; 3] {
        let (vx, vy) = Self::convert_to_body_frame(speed, angle, orientation);
        let v = [vx, vy, omega];
        let j = self.construct_jacobian();
        let mut out = [0.0; 3];
        fn clamp_small(
            v: f32,
            eps: f32,
        ) -> f32 {
            if v.abs() < eps {
                0.0
            } else {
                v
            }
        }
        for i in 0..3 {
            out[i] = clamp_small(j[i][0] * v[0] + j[i][1] * v[1] + j[i][2] * v[2], 1e-6);
        }
        out
    }
}

/// Invert a 3×3 matrix via cofactor expansion
fn invert_3x3(m: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    let inv_det = 1.0 / det;
    [
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ]
}
