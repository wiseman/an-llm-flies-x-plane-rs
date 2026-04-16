//! Simplified TECS (Total Energy Control System). Mirrors
//! sim_pilot/control/tecs_lite.py. Phase-dependent trims and weights keep the
//! commanded pitch / throttle plausible across takeoff, climb, cruise,
//! descent, pattern, base, and final.

use crate::config::TECSGains;
use crate::types::{clamp, FlightPhase};

#[derive(Debug, Clone)]
pub struct TECSLite {
    pub gains: TECSGains,
    pub integrator: f64,
}

impl TECSLite {
    pub fn new(gains: TECSGains) -> Self {
        Self {
            gains,
            integrator: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
    }

    /// Returns (pitch_cmd_deg, throttle_cmd).
    pub fn update(
        &mut self,
        phase: FlightPhase,
        target_alt_ft: f64,
        target_speed_kt: f64,
        alt_ft: f64,
        vs_fpm: f64,
        ias_kt: f64,
        dt: f64,
        throttle_limit: (f64, f64),
    ) -> (f64, f64) {
        let (throttle_trim, pitch_trim) = Self::trim_for_phase(phase);
        let (
            total_alt_w,
            total_spd_w,
            balance_alt_w,
            balance_spd_w,
            min_pitch,
            max_pitch,
        ) = Self::phase_weights(phase);
        let altitude_error_ft = target_alt_ft - alt_ft;
        let speed_error_kt = target_speed_kt - ias_kt;

        let energy_total = total_alt_w * altitude_error_ft + total_spd_w * speed_error_kt;
        self.integrator = clamp(self.integrator + energy_total * dt, -80.0, 80.0);
        let throttle_cmd =
            throttle_trim + self.gains.kp_total * energy_total + self.gains.ki_total * self.integrator;

        let energy_balance = balance_alt_w * altitude_error_ft - balance_spd_w * speed_error_kt;
        let pitch_cmd = pitch_trim + self.gains.kp_balance * energy_balance
            + self.gains.kd_balance * (-vs_fpm);

        (
            clamp(pitch_cmd, min_pitch, max_pitch),
            clamp(throttle_cmd, throttle_limit.0, throttle_limit.1),
        )
    }

    fn trim_for_phase(phase: FlightPhase) -> (f64, f64) {
        match phase {
            FlightPhase::Rotate
            | FlightPhase::InitialClimb
            | FlightPhase::EnrouteClimb
            | FlightPhase::GoAround => (0.85, 6.0),
            FlightPhase::Cruise => (0.58, 2.0),
            FlightPhase::Descent => (0.35, -1.5),
            FlightPhase::PatternEntry | FlightPhase::Downwind => (0.45, 1.0),
            FlightPhase::Base => (0.32, 0.0),
            FlightPhase::Final => (0.28, -0.5),
            _ => (0.5, 1.0),
        }
    }

    fn phase_weights(phase: FlightPhase) -> (f64, f64, f64, f64, f64, f64) {
        match phase {
            FlightPhase::Final => (0.004, 10.0, 3.5, 12.0, -10.0, 10.0),
            FlightPhase::Base => (0.008, 8.5, 2.5, 14.0, -8.0, 10.0),
            FlightPhase::Descent => (0.009, 8.0, 1.0, 14.0, -6.0, 10.0),
            _ => (0.01, 8.0, 1.0, 15.0, -4.0, 12.0),
        }
    }
}
