//! Shared control helpers such as PID loops.

use core::f32::consts::TAU;

use crate::motion::encoder::VelocityEstimator;

/// Smallest interval the derivative term is computed over.
///
/// dt is derived from millisecond timestamps, so anything below this is a
/// rounding artifact rather than a real elapsed time, and a difference quotient
/// over it is pure amplification. The control loop nominally runs at 50 ms, so
/// this is three orders of magnitude below any legitimate interval.
const MIN_DERIVATIVE_DT_S: f32 = 1e-4;

/// Simple clamped PID controller for wheel-speed or other scalar loops.
#[derive(Clone, Copy, Debug)]
pub struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    last_error: f32,
    integral_limit: f32,
    output_limit: f32,
    first_update: bool,
}

impl PidController {
    pub const fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        integral_limit: f32,
        output_limit: f32,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            last_error: 0.0,
            integral_limit,
            output_limit,
            first_update: true,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.first_update = true;
    }

    pub fn update(
        &mut self,
        setpoint: f32,
        measured: f32,
        dt_s: f32,
    ) -> f32 {
        let dt = if dt_s.is_finite() {
            dt_s.max(1e-6)
        }
        else {
            1e-3
        };
        let error = (setpoint - measured).clamp(-1.0, 1.0);

        self.integral =
            (self.integral + error * dt).clamp(-self.integral_limit, self.integral_limit);

        let derivative = if self.first_update {
            self.first_update = false;
            0.0
        }
        else if dt < MIN_DERIVATIVE_DT_S {
            // Callers derive dt from millisecond timestamps, so two updates
            // inside the same millisecond yield dt == 0.0, floored to 1e-6
            // above. Dividing by that scales the error difference by up to 1e6
            // and saturates the output to +/-output_limit — a full-scale PWM
            // spike from a rounding artifact. The P and I terms remain valid at
            // any dt, so only the difference quotient is suppressed.
            // `encoder.rs` already takes this position for dt <= 0.
            0.0
        }
        else {
            (error - self.last_error) / dt
        };

        self.last_error = error;

        (self.kp * error + self.ki * self.integral + self.kd * derivative)
            .clamp(-self.output_limit, self.output_limit)
    }
}

/// One wheel's most recent PID terms in normalized wheel-space.
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelPidTerms {
    pub target_norm: f32,
    pub measured_norm: f32,
    pub error_norm: f32,
    pub output_norm: f32,
}

/// Three-wheel PID helper using encoder deltas as measured feedback.
#[derive(Clone, Copy, Debug)]
pub struct TriWheelPidController {
    controllers: [PidController; 3],
    estimator: VelocityEstimator,
    max_wheel_rad_s: f32,
    last_terms: [WheelPidTerms; 3],
}

impl TriWheelPidController {
    pub fn new(
        counts_per_revolution: f32,
        max_wheel_rad_s: f32,
        kp: f32,
        ki: f32,
        kd: f32,
        integral_limit: f32,
        output_limit: f32,
    ) -> Self {
        Self {
            controllers: [PidController::new(kp, ki, kd, integral_limit, output_limit); 3],
            estimator: VelocityEstimator::new(counts_per_revolution),
            max_wheel_rad_s,
            last_terms: [WheelPidTerms::default(); 3],
        }
    }

    pub fn reset(&mut self) {
        for controller in &mut self.controllers {
            controller.reset();
        }
        self.last_terms = [WheelPidTerms::default(); 3];
    }

    /// Clear one wheel's integrator and derivative history, leaving the other
    /// wheels' loops untouched. Used when a single wheel is commanded to stop
    /// so its integrator does not wind up against the coast-down.
    pub fn reset_wheel(
        &mut self,
        index: usize,
    ) {
        if let Some(controller) = self.controllers.get_mut(index) {
            controller.reset();
        }
        if let Some(terms) = self.last_terms.get_mut(index) {
            *terms = WheelPidTerms::default();
        }
    }

    pub fn update_from_counts(
        &mut self,
        target_norm: [f32; 3],
        delta_counts: [i32; 3],
        dt_s: f32,
    ) -> [f32; 3] {
        let mut outputs = [0.0; 3];

        for idx in 0..self.controllers.len() {
            let measured_rps = self.estimator.rev_per_sec(delta_counts[idx], dt_s);
            let measured_rad_s = measured_rps * TAU;
            let measured_norm = if self.max_wheel_rad_s > 0.0 {
                (measured_rad_s / self.max_wheel_rad_s).clamp(-1.0, 1.0)
            }
            else {
                0.0
            };
            let target = target_norm[idx].clamp(-1.0, 1.0);
            let output = self.controllers[idx].update(target, measured_norm, dt_s);

            self.last_terms[idx] = WheelPidTerms {
                target_norm: target,
                measured_norm,
                error_norm: (target - measured_norm).clamp(-1.0, 1.0),
                output_norm: output,
            };
            outputs[idx] = output;
        }

        outputs
    }

    pub fn last_terms(&self) -> [WheelPidTerms; 3] { self.last_terms }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Regression guard: with `dt` floored at 1e-6 and then divided into, two
    /// updates inside one millisecond turned a 0.001 error change into a
    /// full-scale output spike.
    #[test]
    fn pid_same_millisecond_update_does_not_saturate_output() {
        // kd matches the live board tuning (0.90, 0.20, 0.02).
        let mut pid = PidController::new(0.9, 0.2, 0.02, 1.0, 1.0);
        // Settle first_update and establish last_error at a realistic 50 ms dt.
        pid.update(0.500, 0.0, 0.05);
        // Second update in the same millisecond: dt rounds to 0.0.
        let out = pid.update(0.501, 0.0, 0.0);
        assert!(
            out.abs() < 1.0,
            "output saturated from a zero-dt derivative: {out}"
        );
        // P + I only, so it should track the small error, not slam the limit.
        assert!(out.abs() < 0.8, "unexpectedly large output: {out}");
    }

    #[test]
    fn pid_derivative_still_applies_at_realistic_dt() {
        let mut pid = PidController::new(0.0, 0.0, 1.0, 1.0, 10.0);
        // First call consumes `first_update`, so the D term is skipped there.
        pid.update(0.0, 0.0, 0.05);
        // error moves 0.0 -> 0.5 over 50 ms => derivative 10.0, with kd 1.0.
        let out = pid.update(0.5, 0.0, 0.05);
        assert!(
            (out - 10.0).abs() < 0.01,
            "derivative term was suppressed at a legitimate dt: {out}"
        );
    }

    #[test]
    fn pid_zero_error_outputs_zero() {
        let mut pid = PidController::new(1.0, 0.5, 0.1, 1.0, 1.0);
        let out = pid.update(0.0, 0.0, 0.1);
        assert!(out.abs() < 1e-6);
    }

    #[test]
    fn pid_positive_error_outputs_positive() {
        let mut pid = PidController::new(0.8, 0.0, 0.0, 1.0, 1.0);
        let out = pid.update(0.6, 0.1, 0.1);
        assert!(out > 0.0);
    }

    #[test]
    fn pid_integral_is_clamped() {
        let mut pid = PidController::new(0.0, 10.0, 0.0, 0.2, 10.0);
        let mut out = 0.0;
        for _ in 0..20 {
            out = pid.update(1.0, 0.0, 0.1);
        }
        assert!(out <= 2.0);
    }

    #[test]
    fn pid_reset_clears_state() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 1.0, 1.0);
        let _ = pid.update(1.0, 0.0, 0.1);
        pid.reset();
        let out = pid.update(0.0, 0.0, 0.1);
        assert!(out.abs() < 1e-6);
    }

    #[test]
    fn tri_wheel_reset_wheel_only_clears_that_wheel() {
        let mut pid = TriWheelPidController::new(600.0, 50.0, 0.0, 1.0, 0.0, 10.0, 10.0);
        for _ in 0..5 {
            pid.update_from_counts([0.5, 0.5, 0.0], [0, 0, 0], 0.05);
        }
        pid.reset_wheel(0);
        let out = pid.update_from_counts([0.0, 0.0, 0.0], [0, 0, 0], 0.05);
        assert!(out[0].abs() < 1e-6, "wheel 0 integrator survived reset: {}", out[0]);
        assert!(out[1] > 0.0, "wheel 1 integrator was cleared too: {}", out[1]);
    }

    #[test]
    fn tri_wheel_pid_uses_encoder_feedback() {
        let mut pid = TriWheelPidController::new(600.0, 7.0, 0.9, 0.2, 0.02, 0.5, 1.0);
        let output = pid.update_from_counts([0.5, 0.0, -0.5], [30, 0, -30], 0.1);
        let terms = pid.last_terms();

        assert!(output[0].is_finite());
        assert!(terms[0].measured_norm > 0.0);
        assert!(terms[2].measured_norm < 0.0);
    }
}
