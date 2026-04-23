//! Bank-angle PID + turn-coordination rudder.

use crate::config::PIDGains;
use crate::control::pid::PIDController;
use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct BankController {
    pub gains: PIDGains,
    pub pid: PIDController,
}

impl BankController {
    pub fn new(gains: PIDGains) -> Self {
        Self {
            gains,
            pid: PIDController::new(gains.kp, gains.ki, gains.kd, 15.0, (-1.0, 1.0)),
        }
    }

    pub fn update(&mut self, target_bank_deg: f64, roll_deg: f64, p_rad_s: f64, dt: f64) -> f64 {
        let roll_rate_deg_s = p_rad_s.to_degrees();
        let error = target_bank_deg - roll_deg;
        self.pid.update(error, roll_rate_deg_s, dt)
    }

    pub fn reset(&mut self) {
        self.pid.reset();
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CoordinationController {
    pub beta_gain: f64,
    pub yaw_gain: f64,
    pub bank_feedforward_gain: f64,
}

impl CoordinationController {
    pub fn new() -> Self {
        Self {
            beta_gain: 0.04,
            yaw_gain: 0.03,
            bank_feedforward_gain: 0.015,
        }
    }

    pub fn update(
        &self,
        target_bank_deg: f64,
        roll_deg: f64,
        yaw_rate_rad_s: f64,
        beta_deg: Option<f64>,
        _dt: f64,
    ) -> f64 {
        let yaw_rate_deg_s = yaw_rate_rad_s.to_degrees();
        let slip_term = match beta_deg {
            Some(b) => self.beta_gain * b,
            None => 0.0,
        };
        let rudder = (target_bank_deg - roll_deg) * self.bank_feedforward_gain + slip_term
            - yaw_rate_deg_s * self.yaw_gain;
        clamp(rudder, -1.0, 1.0)
    }
}

impl Default for CoordinationController {
    fn default() -> Self {
        Self::new()
    }
}
