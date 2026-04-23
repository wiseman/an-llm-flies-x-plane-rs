//! Pitch-angle PID.

use crate::config::PIDGains;
use crate::control::pid::PIDController;

#[derive(Debug, Clone)]
pub struct PitchController {
    pub gains: PIDGains,
    pub pid: PIDController,
}

impl PitchController {
    pub fn new(gains: PIDGains) -> Self {
        Self {
            gains,
            pid: PIDController::new(gains.kp, gains.ki, gains.kd, 15.0, (-1.0, 1.0)),
        }
    }

    pub fn update(&mut self, target_pitch_deg: f64, pitch_deg: f64, q_rad_s: f64, dt: f64) -> f64 {
        let pitch_rate_deg_s = q_rad_s.to_degrees();
        let error = target_pitch_deg - pitch_deg;
        self.pid.update(error, pitch_rate_deg_s, dt)
    }
}
