//! Flight-phase state machine. Mirrors core/mode_manager.py.
//!
//! Each tick takes the current phase plus the aircraft state and decides what
//! the next phase should be. Pattern geometry and the route manager feed
//! decision gates like "ready for base turn" / "pattern_entry waypoint
//! reached". Keeps the full list of regressions the Python version carries
//! (airborne bailout, stale-AGL ground short-circuit, touch-and-go routing).

use crate::config::ConfigBundle;
use crate::core::safety_monitor::SafetyStatus;
use crate::guidance::pattern_manager::PatternGeometry;
use crate::guidance::route_manager::RouteManager;
use crate::types::{clamp, wrap_degrees_180, AircraftState, FlightPhase};

/// ATC-side clearance state for pattern-phase transitions. Each gate
/// authorizes one transition from running automatically. Default is "all
/// granted" so untowered/CTAF ops behave as before. "I'll call your base"
/// revokes `turn_base`; "turn base now" is handled via
/// [`PatternTriggers`], not by flipping a clearance.
///
/// `land` defaults to `true` and is informational only — the mode
/// machine does not enforce landing clearance. The LLM reads it in the
/// status snapshot and decides whether to go around.
#[derive(Debug, Clone)]
pub struct PatternClearances {
    pub turn_crosswind: bool,
    pub turn_downwind: bool,
    pub turn_base: bool,
    pub turn_final: bool,
    pub land: bool,
    pub landing_runway: Option<String>,
}

impl Default for PatternClearances {
    fn default() -> Self {
        Self {
            turn_crosswind: true,
            turn_downwind: true,
            turn_base: true,
            turn_final: true,
            land: true,
            landing_runway: None,
        }
    }
}

/// One-shot triggers that force a pattern-phase turn immediately,
/// bypassing the automatic geometric readiness gate. Each trigger is
/// consumed (cleared) by `PatternFlyProfile` after the corresponding
/// transition fires.
#[derive(Debug, Clone, Default)]
pub struct PatternTriggers {
    pub turn_crosswind: bool,
    pub turn_downwind: bool,
    pub turn_base: bool,
    pub turn_final: bool,
}

pub struct ModeManagerUpdate<'a> {
    pub phase: FlightPhase,
    pub state: &'a AircraftState,
    pub route_manager: &'a RouteManager,
    pub pattern: &'a PatternGeometry,
    pub safety_status: &'a SafetyStatus,
    pub triggers: &'a PatternTriggers,
    pub clearances: &'a PatternClearances,
    pub force_go_around: bool,
    pub stay_in_pattern: bool,
    pub touch_and_go: bool,
}

#[derive(Debug, Clone)]
pub struct ModeManager {
    pub config: ConfigBundle,
}

impl ModeManager {
    pub fn new(config: ConfigBundle) -> Self {
        Self { config }
    }

    pub fn update(&self, u: &ModeManagerUpdate) -> FlightPhase {
        let phase = u.phase;
        let state = u.state;
        let route_manager = u.route_manager;
        let pattern = u.pattern;
        let safety_status = u.safety_status;
        let triggers = u.triggers;
        let clearances = u.clearances;
        let force_go_around = u.force_go_around;
        let stay_in_pattern = u.stay_in_pattern;
        let touch_and_go = u.touch_and_go;
        if force_go_around || safety_status.request_go_around {
            return FlightPhase::GoAround;
        }

        match phase {
            FlightPhase::Preflight => {
                if !state.on_ground && state.ias_kt >= self.config.performance.vr_kt {
                    return FlightPhase::InitialClimb;
                }
                return FlightPhase::TakeoffRoll;
            }
            FlightPhase::TakeoffRoll => {
                // Airborne bailout: if the wheels are off the ground at or
                // above Vr, advance — a stuck TAKEOFF_ROLL mid-air is bad.
                if !state.on_ground && state.ias_kt >= self.config.performance.vr_kt {
                    return FlightPhase::Rotate;
                }
                let centerline_limit_ft =
                    self.config.limits.unstable_centerline_error_ft * 0.5;
                if state.ias_kt >= self.config.performance.vr_kt
                    && state.centerline_error_ft.unwrap_or(0.0).abs() <= centerline_limit_ft
                {
                    return FlightPhase::Rotate;
                }
                return phase;
            }
            FlightPhase::Rotate => {
                if !state.on_ground && state.alt_agl_ft >= 20.0 && state.vs_fpm > 100.0 {
                    return FlightPhase::InitialClimb;
                }
                return phase;
            }
            FlightPhase::InitialClimb => {
                if stay_in_pattern {
                    let crosswind_turn_agl_ft =
                        (self.config.pattern.altitude_agl_ft - 300.0).max(400.0);
                    let auto_ready =
                        state.alt_agl_ft >= crosswind_turn_agl_ft && state.vs_fpm > 250.0;
                    if triggers.turn_crosswind || (clearances.turn_crosswind && auto_ready) {
                        return FlightPhase::Crosswind;
                    }
                    return phase;
                }
                if state.alt_agl_ft >= 400.0 && state.vs_fpm > 250.0 {
                    return FlightPhase::EnrouteClimb;
                }
                return phase;
            }
            FlightPhase::Crosswind => {
                let Some(y) = state.runway_y_ft else { return phase };
                let downwind_offset_ft = pattern.downwind_y_ft.abs();
                let runway_course_deg = pattern.runway_frame.runway.course_deg;
                let side_sign = if pattern.downwind_y_ft < 0.0 { -1.0 } else { 1.0 };
                let crosswind_course_deg =
                    ((runway_course_deg + side_sign * 90.0).rem_euclid(360.0)) as f64;
                let heading_error_deg =
                    wrap_degrees_180(state.track_deg - crosswind_course_deg).abs();
                let heading_captured = heading_error_deg <= 15.0;
                let at_offset = y.abs() >= downwind_offset_ft * 0.75;
                let auto_ready = heading_captured && at_offset;
                if triggers.turn_downwind || (clearances.turn_downwind && auto_ready) {
                    return FlightPhase::Downwind;
                }
                return phase;
            }
            FlightPhase::EnrouteClimb => {
                if state.alt_msl_ft >= self.config.cruise_altitude_ft() - 150.0 {
                    return FlightPhase::Cruise;
                }
                return phase;
            }
            FlightPhase::Cruise => {
                if let Some(wp) = route_manager.active_waypoint() {
                    if wp.name == "pattern_entry_start" {
                        return FlightPhase::Descent;
                    }
                }
                return phase;
            }
            FlightPhase::Descent => {
                if let Some(wp) = route_manager.active_waypoint() {
                    if wp.name == "pattern_entry_start"
                        && state.position_ft.distance_to(wp.position_ft) <= 2500.0
                        && (state.alt_msl_ft - self.config.pattern_altitude_msl_ft()).abs() <= 450.0
                    {
                        return FlightPhase::PatternEntry;
                    }
                }
                return phase;
            }
            FlightPhase::PatternEntry => {
                if let (Some(rx), Some(ry)) = (state.runway_x_ft, state.runway_y_ft) {
                    let join_reached = rx <= pattern.join_point_runway_ft.x + 400.0
                        && (ry - pattern.downwind_y_ft).abs() <= 900.0;
                    if join_reached
                        || pattern.is_established_on_downwind(
                            state.runway_x_ft,
                            state.runway_y_ft,
                            state.track_deg,
                        )
                    {
                        return FlightPhase::Downwind;
                    }
                }
                return phase;
            }
            FlightPhase::Downwind => {
                let auto_ready = self.downwind_base_turn_ready(state, pattern);
                if triggers.turn_base || (clearances.turn_base && auto_ready) {
                    return FlightPhase::Base;
                }
                return phase;
            }
            FlightPhase::Base => {
                let on_final = pattern.is_established_on_final(
                    state.runway_x_ft,
                    state.runway_y_ft,
                    state.track_deg,
                );
                let leg = pattern.base_leg;
                let path = leg.end_ft - leg.start_ft;
                let rel = state.position_ft - leg.start_ft;
                let path_length = path.length().max(1.0);
                let along = rel.dot(path.normalized());
                let past_65_pct = along >= path_length * 0.65;
                let auto_ready = on_final || past_65_pct;
                if triggers.turn_final || (clearances.turn_final && auto_ready) {
                    return FlightPhase::Final;
                }
                return phase;
            }
            FlightPhase::Final | FlightPhase::Roundout | FlightPhase::Flare => {
                if state.on_ground {
                    return if touch_and_go {
                        FlightPhase::TakeoffRoll
                    } else {
                        FlightPhase::Rollout
                    };
                }
                match phase {
                    FlightPhase::Final => {
                        if state.alt_agl_ft <= self.config.flare.roundout_height_ft {
                            return FlightPhase::Roundout;
                        }
                        return phase;
                    }
                    FlightPhase::Roundout => {
                        if state.alt_agl_ft <= self.config.flare.flare_start_ft {
                            return FlightPhase::Flare;
                        }
                        return phase;
                    }
                    FlightPhase::Flare => return phase,
                    _ => unreachable!(),
                }
            }
            FlightPhase::Rollout => {
                // Advance to RunwayExit once the aircraft has left the
                // runway surface laterally. `PatternFlyProfile` now
                // holds rollout at turnoff speed (not zero), so the
                // old "gs ≤ 5 kt" guard doesn't fire on its own after a
                // nominal landing. The LLM calls engage_park once it's
                // decided where to go; this transition is the clear-
                // of-runway marker that gates the handoff.
                //
                // Thresholds are fixed rather than pulled from a config
                // RunwayConfig because the pilot doesn't carry runway
                // width into the runway frame. 75 ft covers the widest
                // airline-class runways (200 ft wide → half-width 100 ft)
                // only loosely, but for GA ops on 75-150 ft runways it's
                // a clean "you're on the taxiway" signal.
                const RUNWAY_EXIT_OFFSET_FT: f64 = 75.0;
                let lateral_off = state.runway_y_ft.map(f64::abs).unwrap_or(0.0);
                if lateral_off > RUNWAY_EXIT_OFFSET_FT {
                    return FlightPhase::RunwayExit;
                }
                // Preserve the old guard as a fallback so deterministic
                // tests that stop the aircraft on centerline still reach
                // TaxiClear.
                if state.gs_kt <= 5.0 {
                    return FlightPhase::TaxiClear;
                }
                return phase;
            }
            FlightPhase::RunwayExit => {
                // Full stop = clear of runway + brakes down. Alternately,
                // ≥ 150 ft off centerline means we're well onto the
                // taxiway, even if still creeping forward.
                const RUNWAY_CLEAR_OFFSET_FT: f64 = 150.0;
                if state.gs_kt <= 1.0 {
                    return FlightPhase::TaxiClear;
                }
                let lateral_off = state.runway_y_ft.map(f64::abs).unwrap_or(0.0);
                if lateral_off > RUNWAY_CLEAR_OFFSET_FT {
                    return FlightPhase::TaxiClear;
                }
                return phase;
            }
            FlightPhase::TaxiClear => return phase,
            FlightPhase::GoAround => return phase,
        }
    }

    fn downwind_base_turn_ready(&self, state: &AircraftState, pattern: &PatternGeometry) -> bool {
        let Some(rx) = state.runway_x_ft else { return false };
        let nominal_turn_x_ft = pattern.base_turn_x_ft;
        let earliest_turn_x_ft = self.config.pattern.abeam_window_ft;
        let speed_shortfall_kt =
            (self.config.performance.downwind_speed_kt - state.gs_kt).max(0.0);
        let max_relief_ft = (earliest_turn_x_ft - nominal_turn_x_ft).max(0.0);
        let relief_ft = clamp(speed_shortfall_kt * 140.0, 0.0, max_relief_ft);
        let adaptive_turn_x_ft = nominal_turn_x_ft + relief_ft;
        rx <= adaptive_turn_x_ft
    }
}
