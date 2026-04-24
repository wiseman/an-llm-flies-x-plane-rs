//! Simplified TECS (Total Energy Control System). Phase-dependent trims
//! and weights keep the commanded pitch / throttle plausible across
//! takeoff, climb, cruise, descent, pattern, base, and final.

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
    #[allow(clippy::too_many_arguments)]
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
        // Anti-windup: accumulate into `next_int` and only accept it if
        // applying the new integrator value does not push the throttle
        // command further into the saturation direction. Otherwise the
        // actuator is already railed and more integration just means
        // more lag when the system eventually recovers.
        let next_int = clamp(self.integrator + energy_total * dt, -80.0, 80.0);
        let unclamped =
            throttle_trim + self.gains.kp_total * energy_total + self.gains.ki_total * next_int;
        let throttle_cmd = clamp(unclamped, throttle_limit.0, throttle_limit.1);
        let pushing_up_into_high = unclamped > throttle_limit.1 && energy_total > 0.0;
        let pushing_down_into_low = unclamped < throttle_limit.0 && energy_total < 0.0;
        if !pushing_up_into_high && !pushing_down_into_low {
            self.integrator = next_int;
        }

        let energy_balance = balance_alt_w * altitude_error_ft - balance_spd_w * speed_error_kt;
        // VS damping is counterproductive on a nominal glidepath (final
        // wants a steady non-zero descent rate). Zero it out for Final so
        // it doesn't oppose the commanded descent.
        let kd_effective = match phase {
            FlightPhase::Final => 0.0,
            _ => self.gains.kd_balance,
        };
        let pitch_cmd = pitch_trim + self.gains.kp_balance * energy_balance
            + kd_effective * (-vs_fpm);

        (clamp(pitch_cmd, min_pitch, max_pitch), throttle_cmd)
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
        // Returns (total_alt_w, total_spd_w, balance_alt_w, balance_spd_w,
        //          min_pitch_deg, max_pitch_deg).
        //
        // `total_*` weights go into the throttle energy equation; their
        // ratio sets how TECS prioritizes altitude vs speed when choosing
        // thrust. Final and Base stay low-weight on altitude because
        // glidepath tracking demands tight speed control and altitude is
        // governed by pitch on a slope. For everything else (Downwind,
        // PatternEntry, Cruise, EnrouteClimb) altitude is a hard
        // constraint — we want within 100 ft of target — so altitude gets
        // meaningful weight (0.05) instead of being drowned out by the
        // speed term.
        match phase {
            FlightPhase::Final => (0.004, 10.0, 6.0, 12.0, -10.0, 10.0),
            FlightPhase::Base => (0.008, 8.5, 2.5, 14.0, -8.0, 10.0),
            FlightPhase::Descent => (0.009, 8.0, 1.0, 14.0, -6.0, 10.0),
            _ => (0.05, 8.0, 1.0, 15.0, -4.0, 12.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_gains() -> TECSGains {
        TECSGains {
            kp_total: 0.003,
            ki_total: 0.0008,
            kp_balance: 0.012,
            kd_balance: 0.01,
        }
    }

    #[test]
    fn anti_windup_freezes_integrator_when_saturated_high() {
        let mut tecs = TECSLite::new(default_gains());
        // Aircraft 1500 ft below a 2000 ft target at the downwind
        // target speed — large positive altitude error sustained. With
        // a tight throttle cap of 0.5 TECS will saturate immediately;
        // the integrator should settle well under the ±80 absolute cap
        // because anti-windup refuses accumulation while saturating.
        for _ in 0..2000 {
            tecs.update(
                FlightPhase::Downwind,
                /* target_alt_ft */ 2000.0,
                /* target_speed_kt */ 90.0,
                /* alt_ft */ 500.0,
                /* vs_fpm */ 0.0,
                /* ias_kt */ 90.0,
                /* dt */ 0.1,
                /* throttle_limit */ (0.0, 0.5),
            );
        }
        // Without anti-windup the integrator would charge right to the
        // +80 clamp. With it, the integrator stops growing as soon as
        // throttle saturates at 0.5 on the first few ticks.
        assert!(
            tecs.integrator.abs() < 20.0,
            "integrator wound up to {} despite saturation",
            tecs.integrator
        );
    }

    #[test]
    fn no_saturation_means_integrator_still_accumulates() {
        let mut tecs = TECSLite::new(default_gains());
        // Small altitude error well within (0.0, 1.0) authority; the
        // controller should never saturate and the integrator should
        // accumulate normally.
        for _ in 0..500 {
            tecs.update(
                FlightPhase::Cruise,
                /* target_alt_ft */ 3000.0,
                /* target_speed_kt */ 110.0,
                /* alt_ft */ 2990.0,
                /* vs_fpm */ 0.0,
                /* ias_kt */ 110.0,
                /* dt */ 0.1,
                /* throttle_limit */ (0.0, 1.0),
            );
        }
        // energy_total ≈ 0.1 per tick → ~5 after 500 ticks of 0.1 s.
        assert!(
            tecs.integrator > 1.0,
            "integrator did not accumulate: {}",
            tecs.integrator
        );
    }
}
