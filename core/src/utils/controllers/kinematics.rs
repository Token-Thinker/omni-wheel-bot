#[allow(dead_code)]
use core::f32::consts::PI;
use libm;

/// Represents the kinematics of a 3-wheeled omni-wheel robot and includes
/// PID control logic for wheel-speed regulation.
pub struct WheelKinematics {
    /// Radius of each wheel (m)
    wheel_radius: f32,
    /// Robot center-to-wheel distance (m)
    robot_radius: f32,
    /// Precomputed wheel mounting angles (rad)
    wheel_angles: [f32; 3],
}

impl WheelKinematics {
    /// Instantiate with a given wheel and robot radii.
    pub fn new(wheel_radius: f32, robot_radius: f32) -> Self {
        let wheel_angles = [PI / 3.0, PI, 5.0 * PI / 3.0];
        Self { wheel_radius, robot_radius, wheel_angles }
    }

    /// Transform global vector (speed, angle) into body-frame (vx, vy)
    pub fn convert_to_body_frame(speed: f32, angle: f32, orientation: f32) -> (f32, f32) {
        let a = angle * (PI / 180.0);
        let o = orientation * (PI / 180.0);
        let vx = speed * libm::cosf(a - o);
        let vy = speed * libm::sinf(a - o);
        (-vy, vx)
    }

    /// Build Jacobian J such that ω_wheels = J * [vx, vy, ω_body]
    pub fn construct_jacobian(&self) -> [[f32;3];3] {
        let r = self.wheel_radius;
        let l = self.robot_radius;
        let mut j = [[0.0;3];3];
        for (i,&t) in self.wheel_angles.iter().enumerate() {
            j[i][0] = libm::cosf(t) / r;
            j[i][1] = libm::sinf(t) / r;
            j[i][2] = l / r;
        }
        j
    }

    /// Invert measured wheel speeds back to body-frame velocity
    pub fn compute_body_velocity(&self, wheel_velocity: [f32;3]) -> (f32, f32, f32) {
        let j = self.construct_jacobian();
        let inv = invert_3x3(j);
        let vx = inv[0][0] * wheel_velocity[0] + inv[0][1] * wheel_velocity[1] + inv[0][2] * wheel_velocity[2];
        let vy = inv[1][0] * wheel_velocity[0] + inv[1][1] * wheel_velocity[1] + inv[1][2] * wheel_velocity[2];
        let w = inv[2][0] * wheel_velocity[0] + inv[2][1] * wheel_velocity[1] + inv[2][2] * wheel_velocity[2];
        (vx, vy, w)
    }

    /// Compute individual wheel speeds given desired motion
    pub fn compute_wheel_velocities(
        &self,
        speed: f32,
        angle: f32,
        orientation: f32,
        omega: f32,
    ) -> [f32;3] {
        let (vx, vy) = Self::convert_to_body_frame(speed, angle, orientation);
        let v = [vx, vy, omega];
        let j = self.construct_jacobian();
        let mut out = [0.0;3];
        fn clamp_small(v: f32, eps: f32) -> f32 { if v.abs() < eps {0.0} else {v} }
        for i in 0..3 {
            out[i] = clamp_small(j[i][0]*v[0] + j[i][1]*v[1] + j[i][2]*v[2], 1e-6);
        }
        out
    }

    /// Sensorless cascade control: outer-loop PID on body rates (vx,vy,ω_body)
    /// using IMU accel/gyro, mapping back to wheel commands via Jacobian.
    pub fn sensorless_control(
        &self,
        accel: (f32, f32),           // (ax, ay) in m/s²
        gyro_z: f32,                 // ω_body measured (rad/s)
        desired: (f32, f32, f32),    // (vx*, vy*, ω_body*) setpoints
        vel_state: &mut (f32, f32),  // integrator state for vx, vy
        pids: &mut [PidController; 3],
    ) -> [f32;3] {
        // 1) Integrate body accel → velocity estimate
        let dt = pids[0].dt;
        vel_state.0 += accel.0 * dt;
        vel_state.1 += accel.1 * dt;

        // 2) Construct measured body rates
        let measured = [ vel_state.0, vel_state.1, gyro_z ];

        // 3) Outer-loop PID on body rates
        let des = [ desired.0, desired.1, desired.2 ];
        let mut corr = [0.0;3];
        for i in 0..3 {
            let err = des[i] - measured[i];
            corr[i] = pids[i].update(err);
        }

        // 4) Map body-rate corrections → wheel commands
        let j = self.construct_jacobian();
        let mut wheel_cmds = [0.0;3];
        for i in 0..3 {
            wheel_cmds[i] = j[i][0]*corr[0]
                + j[i][1]*corr[1]
                + j[i][2]*corr[2];
        }
        wheel_cmds
    }
}

/// Invert a 3×3 matrix via cofactor expansion
fn invert_3x3(m: [[f32;3];3]) -> [[f32;3];3] {
    let det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
        - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
        + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    let inv_det = 1.0 / det;
    [
        [ (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * inv_det,
            -(m[0][1]*m[2][2] - m[0][2]*m[2][1]) * inv_det,
            (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * inv_det ],
        [-(m[1][0]*m[2][2] - m[1][2]*m[2][0]) * inv_det,
            (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * inv_det,
            -(m[0][0]*m[1][2] - m[0][2]*m[1][0]) * inv_det ],
        [ (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * inv_det,
            -(m[0][0]*m[2][1] - m[0][1]*m[2][0]) * inv_det,
            (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * inv_det ],
    ]
}

/// Discrete PID controller with anti-windup and derivative filtering
pub struct PidController {
    pub kp: f32, pub ki: f32, pub kd: f32,
    pub prev_err: f32, pub integral: f32, pub prev_der: f32,
    pub dt: f32,
    pub i_min: f32, pub i_max: f32,
    pub out_min: f32, pub out_max: f32,
    pub alpha: f32,
}

impl PidController {
    /// - `alpha` ∈ [0.0,1.0] for derivative LPF
    /// - `(i_min,i_max)` clamp integral
    /// - `(o_min,o_max)` clamp output
    pub fn new(
        kp: f32, ki: f32, kd: f32, dt: f32,
        alpha: f32,
        integral_limits: (f32,f32),
        output_limits:   (f32,f32),
    ) -> Self {
        let (i_min,i_max) = integral_limits;
        let (o_min,o_max) = output_limits;
        Self { kp, ki, kd,
            prev_err:0.0, integral:0.0, prev_der:0.0,
            dt, i_min, i_max, out_min:o_min, out_max:o_max,
            alpha }
    }

    /// Compute PID output
    pub fn update(&mut self, error: f32) -> f32 {
        // integral with clamp
        self.integral = (self.integral + error*self.dt)
            .clamp(self.i_min, self.i_max);
        // derivative
        let raw_der = (error - self.prev_err)/self.dt;
        let der = self.alpha*raw_der + (1.0-self.alpha)*self.prev_der;
        self.prev_der = der;
        // PID sum
        let mut out = self.kp*error + self.ki*self.integral + self.kd*der;
        // clamp output
        out = out.clamp(self.out_min, self.out_max);
        self.prev_err = error;
        out
    }

    /// Reset history on set-point change
    pub fn reset(&mut self) {
        self.prev_err = 0.0;
        self.integral = 0.0;
        self.prev_der = 0.0;
    }
}
