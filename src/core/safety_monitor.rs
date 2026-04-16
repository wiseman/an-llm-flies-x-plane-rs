//! Approach-phase safety checks. Mirrors core/safety_monitor.py.

use crate::config::ConfigBundle;
use crate::types::{clamp, AircraftState, FlightPhase, GuidanceTargets};

#[derive(Debug, Clone)]
pub struct SafetyStatus {
    pub request_go_around: bool,
    pub reason: Option<String>,
    pub bank_limit_deg: f64,
}

#[derive(Debug, Clone)]
pub struct SafetyMonitor {
    pub config: ConfigBundle,
}

impl SafetyMonitor {
    pub fn new(config: ConfigBundle) -> Self {
        Self { config }
    }

    pub fn evaluate(&self, state: &AircraftState, phase: FlightPhase) -> SafetyStatus {
        let bank_limit_deg = self.bank_limit_deg(phase);
        if state.on_ground {
            return SafetyStatus { request_go_around: false, reason: None, bank_limit_deg };
        }
        if phase == FlightPhase::Final && state.alt_agl_ft < 200.0 {
            let cle = state.centerline_error_ft.unwrap_or(0.0).abs();
            let max_cle = (state.alt_agl_ft * 2.0).max(30.0);
            if cle > max_cle {
                let reason = format!(
                    "unstable_lateral cle={:.0}ft agl={:.0}ft limit={:.0}ft",
                    cle, state.alt_agl_ft, max_cle
                );
                return SafetyStatus { request_go_around: true, reason: Some(reason), bank_limit_deg };
            }
        }
        if (phase == FlightPhase::Final || phase == FlightPhase::Roundout)
            && state.alt_agl_ft < 200.0
            && state.vs_fpm < -self.config.limits.unstable_sink_rate_fpm
        {
            let reason = format!(
                "unstable_vertical vs={:.0}fpm agl={:.0}ft",
                state.vs_fpm, state.alt_agl_ft
            );
            return SafetyStatus { request_go_around: true, reason: Some(reason), bank_limit_deg };
        }
        if phase == FlightPhase::Final
            && state.alt_agl_ft < 50.0
            && state.stall_margin < (self.config.limits.min_stall_margin - 0.1).max(1.1)
        {
            let reason = format!(
                "low_energy stall_margin={:.2} ias={:.0}kt agl={:.0}ft",
                state.stall_margin, state.ias_kt, state.alt_agl_ft
            );
            return SafetyStatus { request_go_around: true, reason: Some(reason), bank_limit_deg };
        }
        SafetyStatus { request_go_around: false, reason: None, bank_limit_deg }
    }

    pub fn apply_limits(&self, mut guidance: GuidanceTargets, phase: FlightPhase) -> GuidanceTargets {
        if let Some(b) = guidance.target_bank_deg {
            let limit = self.bank_limit_deg(phase);
            guidance.target_bank_deg = Some(clamp(b, -limit, limit));
        }
        if let Some(p) = guidance.target_pitch_deg {
            guidance.target_pitch_deg = Some(clamp(
                p,
                -self.config.limits.max_pitch_down_deg,
                self.config.limits.max_pitch_up_deg,
            ));
        }
        guidance
    }

    pub fn bank_limit_deg(&self, phase: FlightPhase) -> f64 {
        match phase {
            FlightPhase::Final
            | FlightPhase::Roundout
            | FlightPhase::Flare
            | FlightPhase::Rollout => self.config.limits.max_bank_final_deg,
            FlightPhase::Crosswind
            | FlightPhase::PatternEntry
            | FlightPhase::Downwind
            | FlightPhase::Base => self.config.limits.max_bank_pattern_deg,
            _ => self.config.limits.max_bank_enroute_deg,
        }
    }
}
