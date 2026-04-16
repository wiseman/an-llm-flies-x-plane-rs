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

pub struct ModeManagerUpdate<'a> {
    pub phase: FlightPhase,
    pub state: &'a AircraftState,
    pub route_manager: &'a RouteManager,
    pub pattern: &'a PatternGeometry,
    pub safety_status: &'a SafetyStatus,
    pub turn_base_now: bool,
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

    pub fn update(
        &self,
        phase: FlightPhase,
        state: &AircraftState,
        route_manager: &RouteManager,
        pattern: &PatternGeometry,
        safety_status: &SafetyStatus,
        turn_base_now: bool,
        force_go_around: bool,
        stay_in_pattern: bool,
        touch_and_go: bool,
    ) -> FlightPhase {
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
                    if state.alt_agl_ft >= crosswind_turn_agl_ft && state.vs_fpm > 250.0 {
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
                if heading_captured && at_offset {
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
                if turn_base_now || self.downwind_base_turn_ready(state, pattern) {
                    return FlightPhase::Base;
                }
                return phase;
            }
            FlightPhase::Base => {
                if pattern.is_established_on_final(
                    state.runway_x_ft,
                    state.runway_y_ft,
                    state.track_deg,
                ) {
                    return FlightPhase::Final;
                }
                let leg = pattern.base_leg;
                let path = leg.end_ft - leg.start_ft;
                let rel = state.position_ft - leg.start_ft;
                let path_length = path.length().max(1.0);
                let along = rel.dot(path.normalized());
                if along >= path_length * 0.65 {
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
                if state.gs_kt <= 5.0 {
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
