//! Flare-phase pitch schedule. Mirrors guidance/flare_profile.py.

use crate::config::FlareConfig;
use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct FlareController {
    pub config: FlareConfig,
}

impl FlareController {
    pub fn new(config: FlareConfig) -> Self {
        Self { config }
    }

    pub fn target_pitch_deg(&self, alt_agl_ft: f64, sink_rate_fpm: f64, ias_error_kt: f64) -> f64 {
        let ratio = clamp(
            (self.config.flare_start_ft - alt_agl_ft) / self.config.flare_start_ft,
            0.0,
            1.0,
        );
        let sink_bias = clamp(((-sink_rate_fpm) - 200.0) / 250.0, 0.0, 2.0);
        let speed_bias = clamp(-ias_error_kt * 0.12, -1.5, 1.0);
        let pitch = 2.5 + ratio * 4.5 + sink_bias + speed_bias;
        clamp(pitch, 1.0, self.config.max_flare_pitch_deg)
    }
}
