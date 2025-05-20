/// A simple discrete PID controller.
pub struct PidController
{
    kp: f32,
    ki: f32,
    kd: f32,
    prev_err: f32,
    integral: f32,
    dt: f32,
}

impl PidController
{
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        dt: f32,
    ) -> Self
    {
        Self {
            kp,
            ki,
            kd,
            prev_err: 0.0,
            integral: 0.0,
            dt,
        }
    }

    /// Compute control output for the current error.
    pub fn update(
        &mut self,
        error: f32,
    ) -> f32
    {
        self.integral += error * self.dt;
        let derivative = (error - self.prev_err) / self.dt;
        self.prev_err = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    /// Reset integrator and derivative history.
    pub fn reset(&mut self)
    {
        self.prev_err = 0.0;
        self.integral = 0.0;
    }
}
