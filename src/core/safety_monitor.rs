//! Approach-phase safety checks.

use crate::config::ConfigBundle;
use crate::types::{clamp, AircraftState, FlightPhase, GuidanceTargets};

#[derive(Debug, Clone)]
pub struct SafetyStatus {
    pub request_go_around: bool,
    pub reason: Option<String>,
    pub bank_limit_deg: f64,
}

/// Continuous risk read-out for the TUI. `evaluate()` only fires a
/// binary go-around request; `risk_summary()` reuses the same predicates
/// to expose how close the aircraft is to each limit, plus a single
/// compact "what would trigger a go-around right now" string. All
/// utilizations are in [0, ∞): values < 1 are inside the limit, ≥ 1
/// would trip the corresponding check.
#[derive(Debug, Clone)]
pub struct RiskSummary {
    /// `|roll_deg| / bank_limit_deg` for the current phase.
    pub bank_util: f64,
    /// `|vs_fpm| / unstable_sink_rate_fpm` (descent only — climbs report 0).
    pub descent_util: f64,
    /// Stall-margin band: ≥ 1.0 is safe in steady flight; the
    /// phase-aware go-around predicate fires below `min_stall_margin`
    /// near the runway.
    pub stall_margin: f64,
    /// Lateral-vs-AGL band on final, evaluated as a 3-state classifier
    /// (Safe / Warn / Trigger). Off-final phases report `Safe`.
    pub cte_band: CteBand,
    /// Phase-dependent maximum bank in degrees (mirrors `bank_limit_deg`
    /// returned by `evaluate()`, but always populated, not just on
    /// trigger).
    pub bank_limit_deg: f64,
    /// Phase-dependent descent-rate ceiling in fpm (the threshold the
    /// descent_util ratio is normalized against). Same value as
    /// `config.limits.unstable_sink_rate_fpm` today; surfacing it here
    /// keeps the UI math self-contained.
    pub descent_limit_fpm: f64,
    /// Single-line "what would trigger a go-around right now" string
    /// (e.g. "CTE > 40 ft ∨ AGL < 150 ft on final"). Phase-aware so the
    /// supervisor sees the *active* gate, not the union of all gates.
    pub trigger_text: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CteBand {
    Safe,
    Warn,
    Trigger,
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
        bank_limit_deg_for(&self.config, phase)
    }

    /// Phase-dependent descent ceiling in fpm. Today `evaluate()` uses
    /// the same `unstable_sink_rate_fpm` threshold across phases —
    /// surfacing it as an accessor lets the TUI normalize the descent
    /// utilization band without reaching into the config directly.
    pub fn descent_limit_fpm(&self, _phase: FlightPhase) -> f64 {
        self.config.limits.unstable_sink_rate_fpm
    }

    /// Continuous companion to `evaluate()`: returns per-axis utilization
    /// values plus a human-readable description of the active trigger.
    /// Reuses the same predicates so the TUI's read-out stays
    /// authoritative — if `risk_summary()` reports `Trigger`, the next
    /// `evaluate()` call will fire a go-around.
    pub fn risk_summary(&self, state: &AircraftState, phase: FlightPhase) -> RiskSummary {
        risk_summary_for(&self.config, state, phase)
    }
}

/// Free-standing variant of `SafetyMonitor::risk_summary` that borrows
/// the config instead of owning it. The TUI redraw loop calls this once
/// per frame under a single `pilot.lock()`, avoiding a `ConfigBundle`
/// deep clone (and the second lock that would otherwise be needed to
/// build a temporary `SafetyMonitor`).
pub fn risk_summary_for(
    config: &ConfigBundle,
    state: &AircraftState,
    phase: FlightPhase,
) -> RiskSummary {
    let bank_limit = bank_limit_deg_for(config, phase);
    let bank_util = if bank_limit > 0.0 {
        state.roll_deg.abs() / bank_limit
    } else {
        0.0
    };
    let descent_limit = config.limits.unstable_sink_rate_fpm;
    let descent_util = if descent_limit > 0.0 && state.vs_fpm < 0.0 {
        (-state.vs_fpm) / descent_limit
    } else {
        0.0
    };
    let cte_band = if phase == FlightPhase::Final && !state.on_ground {
        let cle = state.centerline_error_ft.unwrap_or(0.0).abs();
        let max_cle = (state.alt_agl_ft * 2.0).max(30.0);
        if state.alt_agl_ft >= 200.0 {
            CteBand::Safe
        } else if cle > max_cle {
            CteBand::Trigger
        } else if cle > 0.7 * max_cle {
            CteBand::Warn
        } else {
            CteBand::Safe
        }
    } else {
        CteBand::Safe
    };
    let trigger_text = match phase {
        FlightPhase::Final => format!(
            "CTE > 2×AGL ∨ vs < -{:.0} ∨ stall < {:.2} (final, AGL < 200 ft)",
            config.limits.unstable_sink_rate_fpm,
            config.limits.min_stall_margin,
        ),
        FlightPhase::Roundout => format!(
            "vs < -{:.0} (roundout)",
            config.limits.unstable_sink_rate_fpm,
        ),
        _ => format!(
            "bank > {:.0}° (limit per phase) — fires go-around only on final/roundout",
            bank_limit,
        ),
    };
    RiskSummary {
        bank_util,
        descent_util,
        stall_margin: state.stall_margin,
        cte_band,
        bank_limit_deg: bank_limit,
        descent_limit_fpm: descent_limit,
        trigger_text,
    }
}

fn bank_limit_deg_for(config: &ConfigBundle, phase: FlightPhase) -> f64 {
    match phase {
        FlightPhase::Final
        | FlightPhase::Roundout
        | FlightPhase::Flare
        | FlightPhase::Rollout => config.limits.max_bank_final_deg,
        FlightPhase::Crosswind
        | FlightPhase::PatternEntry
        | FlightPhase::Downwind
        | FlightPhase::Base => config.limits.max_bank_pattern_deg,
        _ => config.limits.max_bank_enroute_deg,
    }
}
