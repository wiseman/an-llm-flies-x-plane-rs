//! PID controller with anti-windup. Mirrors sim_pilot/control/pid.py.

use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct PIDController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integrator_limit: f64,
    pub output_limit: (f64, f64),
    pub integrator: f64,
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64, integrator_limit: f64, output_limit: (f64, f64)) -> Self {
        Self {
            kp,
            ki,
            kd,
            integrator_limit,
            output_limit,
            integrator: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
    }

    pub fn update(&mut self, error: f64, rate_feedback: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }
        self.integrator = clamp(
            self.integrator + error * dt,
            -self.integrator_limit,
            self.integrator_limit,
        );
        let output = self.kp * error + self.ki * self.integrator - self.kd * rate_feedback;
        let (low, high) = self.output_limit;
        let mut clamped = clamp(output, low, high);

        // Back-calculation anti-windup: if the output saturated in the same
        // direction as the error we were integrating, unwind the last
        // accumulation so subsequent ticks don't keep pushing harder.
        if clamped != output
            && ((clamped == high && error > 0.0) || (clamped == low && error < 0.0))
        {
            self.integrator = clamp(
                self.integrator - error * dt,
                -self.integrator_limit,
                self.integrator_limit,
            );
            let recalc = self.kp * error + self.ki * self.integrator - self.kd * rate_feedback;
            clamped = clamp(recalc, low, high);
        }
        clamped
    }
}
