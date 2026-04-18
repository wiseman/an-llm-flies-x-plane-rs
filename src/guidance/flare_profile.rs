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
        // Lower base + gentler ramp keep the aircraft settling through the
        // flare instead of leveling off. The old 2.5° base pitched up
        // immediately at flare-start and held the plane in ground effect
        // for 800+ ft of float. 1.0° base with a 3.5° ramp peaks at ~4.5°
        // at touchdown, matching real C172 technique.
        let ratio = clamp(
            (self.config.flare_start_ft - alt_agl_ft) / self.config.flare_start_ft,
            0.0,
            1.0,
        );
        let sink_bias = clamp(((-sink_rate_fpm) - 200.0) / 250.0, 0.0, 2.0);
        let speed_bias = clamp(-ias_error_kt * 0.12, -1.5, 1.0);
        let pitch = 1.0 + ratio * 3.5 + sink_bias + speed_bias;
        clamp(pitch, 0.5, self.config.max_flare_pitch_deg)
    }
}
