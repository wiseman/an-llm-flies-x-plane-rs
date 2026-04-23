//! Takeoff/rollout rudder controller.
//!
//! Sign conventions:
//!   centerline_error_ft  — positive when right of centerline
//!   track_error_deg      — positive when runway course > current track
//!   yaw_rate_deg_s       — positive when yawing right
//!   output               — positive is right rudder

use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct CenterlineRolloutController {
    pub centerline_gain: f64,
    pub track_gain: f64,
    pub track_integrator_gain: f64,
    pub yaw_rate_gain: f64,
    pub integrator_limit_deg_s: f64,
    integrator: f64,
}

impl CenterlineRolloutController {
    pub fn new() -> Self {
        Self {
            centerline_gain: 0.015,
            track_gain: 0.012,
            track_integrator_gain: 0.025,
            yaw_rate_gain: 0.20,
            integrator_limit_deg_s: 20.0,
            integrator: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
    }

    pub fn integrator(&self) -> f64 {
        self.integrator
    }

    pub fn update(
        &mut self,
        centerline_error_ft: f64,
        track_error_deg: f64,
        yaw_rate_deg_s: f64,
        gs_kt: f64,
        dt: f64,
    ) -> f64 {
        let speed_scale = if gs_kt >= 40.0 { 1.0 } else { 1.4 };
        let centerline_term = -centerline_error_ft * self.centerline_gain * speed_scale;
        let track_term = track_error_deg * self.track_gain;
        self.integrator = clamp(
            self.integrator + track_error_deg * dt,
            -self.integrator_limit_deg_s,
            self.integrator_limit_deg_s,
        );
        let integral_term = self.integrator * self.track_integrator_gain;
        let damping = -yaw_rate_deg_s * self.yaw_rate_gain;
        clamp(centerline_term + track_term + integral_term + damping, -1.0, 1.0)
    }
}

impl Default for CenterlineRolloutController {
    fn default() -> Self {
        Self::new()
    }
}
