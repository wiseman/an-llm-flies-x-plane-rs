//! Dead-stick (engine-out) landing profile.
//!
//! Owns all three axes and runs a phase machine subset of `PatternFlyProfile`'s,
//! with throttle pinned to zero throughout. Models the FAA Power-Off 180
//! technique: glide at best-glide speed (`vbg_kt`) toward the **Low Key**
//! fix — abeam touchdown on the pattern side at ~1000 ft AGL — then run
//! Downwind → Base → Final → Roundout → Flare → Rollout via the same
//! `ModeManager` / `SafetyMonitor` machinery `PatternFlyProfile` uses.
//!
//! Two important deltas vs. `PatternFlyProfile`:
//!
//! 1. Throttle is locked at `(0.0, 0.0)` on every phase. TECS reduces to
//!    "pitch for airspeed" since it has no throttle authority.
//! 2. Go-around requests from `SafetyMonitor` are *suppressed* — a powerless
//!    aircraft can't climb. The reason is captured for telemetry but the
//!    phase machine is not allowed to dispatch to `GoAround`.
//!
//! The Descent → PatternEntry transition is handled inside this profile
//! rather than by `ModeManager`, because the standard transition requires
//! altitude convergence (within 450 ft of pattern altitude) which a glide
//! cannot guarantee. This profile transitions purely on horizontal range
//! to the Low Key fix.

use crate::config::ConfigBundle;
use crate::core::mode_manager::{
    ModeManager, ModeManagerUpdate, PatternClearances, PatternTriggers,
};
use crate::core::profiles::{
    guidance_to_contribution, post_landing_handoff, AxisSet, GuidanceProfile, PatternMetadata,
    ProfileTick, TransitionHint,
};
use crate::core::safety_monitor::SafetyMonitor;
use crate::guidance::lateral::L1PathFollower;
use crate::guidance::pattern_manager::{
    build_pattern_geometry, PatternGeometry,
};
use crate::guidance::route_manager::RouteManager;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::types::{
    wrap_degrees_180, wrap_degrees_360, AircraftState, FlightPhase, Glidepath, GuidanceTargets,
    LateralMode, TrafficSide, Vec2, VerticalMode, Waypoint,
};

// FAA Power-Off 180. Three fixes: HighKey overhead the aim-point at
// HIGH_KEY_AGL_FT, a descending spiral toward the pattern side, then
// LowKey abeam the aim-point at pattern altitude — from there a short
// tight Downwind/Base/Final to the threshold.

const DEAD_STICK_HIGH_KEY_AGL_FT: f64 = 1500.0;

/// Spiral exits to Downwind once below this AGL — pattern path from
/// LowKey to threshold (~8500 ft at the current offset, mixed clean/
/// flap config) bleeds ~1200 ft of altitude, so this exit altitude
/// delivers the aircraft to the threshold with little margin either way.
const SPIRAL_EXIT_AGL_FT: f64 = 1500.0;

/// Turn radius for a 30° bank at vbg ≈ V²/(g·tan bank).
const SPIRAL_RADIUS_FT: f64 = 750.0;

/// Engine-out cap on bank — 30°. Steeper turns near vbg eat into stall
/// margin, and there's no power to recover. Clamps the safety-monitor's
/// per-phase limit (enroute 45°, pattern 30°, final 20°) so the
/// enroute value never applies here.
const DEAD_STICK_MAX_BANK_DEG: f64 = 30.0;

/// Arc-length lookahead for orbit following.
const SPIRAL_LOOKAHEAD_FT: f64 = 600.0;

const HIGH_KEY_CAPTURE_RADIUS_FT: f64 = 1500.0;

/// Lateral pattern offset for the dead-stick downwind + base legs. The
/// 30° base-turn radius is ~750 ft (1500 ft diameter); 2500 ft offset
/// leaves ~1000 ft of perpendicular margin so the turn-to-final rolls
/// out on the centerline rather than overshooting it.
pub const DEAD_STICK_PATTERN_OFFSET_FT: f64 = 2500.0;

/// Final-approach AGL below which we deploy full flaps (vs 20° above).
/// Higher than the powered-pattern value: dead-stick needs the full
/// flap drag for most of Final to fit the altitude budget.
const LATE_FLAP_AGL_FT: f64 = 700.0;

/// Final-approach AGL at which the speed target steps from vbg to vref.
/// Above this we keep best glide so the runway stays reachable; below
/// it the field is made and we can trade speed for touchdown attitude.
const SHORT_FINAL_AGL_FT: f64 = 300.0;

const BASE_LEG_FRACTION_GATE: f64 = 0.5;

pub struct DeadStickLandingProfile {
    pub config: ConfigBundle,
    pub runway_frame: RunwayFrame,
    pub pattern: PatternGeometry,
    pub mode_manager: ModeManager,
    pub safety_monitor: SafetyMonitor,
    pub lateral_guidance: L1PathFollower,
    pub route_manager: RouteManager,
    pub phase: FlightPhase,
    pub last_safety_reason: Option<String>,
    /// Runway-frame coords of the Low Key fix (abeam touchdown on the
    /// pattern side). Cached for `low_key_runway_ft()`.
    low_key_runway_ft: Vec2,
    /// World-frame coords of fixed dead-stick fixes — derived from
    /// the runway frame, which is constant for the profile's lifetime.
    /// Cached so the per-tick guidance and capture predicates don't
    /// recompute `to_world_frame` calls every cycle.
    high_key_world_ft: Vec2,
    spiral_center_world_ft: Vec2,
}

impl DeadStickLandingProfile {
    pub fn new(
        config: ConfigBundle,
        runway_frame: RunwayFrame,
        state: &AircraftState,
    ) -> Self {
        let pattern = build_pattern_geometry(
            &runway_frame,
            DEAD_STICK_PATTERN_OFFSET_FT,
            0.0,
            0.0,
        );

        let low_key_runway_ft =
            Vec2::new(runway_frame.touchdown_runway_x_ft(), pattern.downwind_y_ft);
        let low_key_world_ft = runway_frame.to_world_frame(low_key_runway_ft);

        let high_key_world_ft =
            runway_frame.to_world_frame(Vec2::new(runway_frame.touchdown_runway_x_ft(), 0.0));

        // Spiral center sits between LowKey and centerline, offset
        // SPIRAL_RADIUS_FT toward centerline so one tangent of the
        // circle is at LowKey heading downwind direction.
        let side_sign = match runway_frame.runway.traffic_side {
            TrafficSide::Left => -1.0,
            TrafficSide::Right => 1.0,
        };
        let spiral_center_world_ft = runway_frame.to_world_frame(Vec2::new(
            runway_frame.touchdown_runway_x_ft(),
            side_sign * DEAD_STICK_PATTERN_OFFSET_FT - side_sign * SPIRAL_RADIUS_FT,
        ));

        // The "pattern_entry_start" name matches what ModeManager looks
        // for so the standard PatternEntry → Downwind predicates fire.
        let route_manager = RouteManager::new(vec![Waypoint {
            name: "pattern_entry_start".to_string(),
            position_ft: low_key_world_ft,
            altitude_ft: Some(config.pattern_altitude_msl_ft()),
        }]);

        let phase = decide_entry_phase(state, &runway_frame, &pattern);

        Self {
            mode_manager: ModeManager::new(config.clone()),
            safety_monitor: SafetyMonitor::new(config.clone()),
            lateral_guidance: L1PathFollower::new(),
            pattern,
            route_manager,
            phase,
            last_safety_reason: None,
            low_key_runway_ft,
            high_key_world_ft,
            spiral_center_world_ft,
            config,
            runway_frame,
        }
    }

    fn guidance_for_phase(&mut self, state: &AircraftState, phase: FlightPhase) -> GuidanceTargets {
        let bank_limit = self
            .safety_monitor
            .bank_limit_deg(phase)
            .min(DEAD_STICK_MAX_BANK_DEG);
        let perf = &self.config.performance;
        let course = self.runway_frame.runway.course_deg;
        let side_sign = match self.runway_frame.runway.traffic_side {
            TrafficSide::Left => -1.0,
            TrafficSide::Right => 1.0,
        };

        match phase {
            FlightPhase::Descent => {
                // Glide direct to High Key. Arrival heading is whatever
                // the start position dictates; the spiral handles
                // downwind alignment.
                let waypoint = Waypoint {
                    name: "high_key".to_string(),
                    position_ft: self.high_key_world_ft,
                    altitude_ft: Some(
                        self.config.airport.field_elevation_ft + DEAD_STICK_HIGH_KEY_AGL_FT,
                    ),
                };
                let (desired_track, bank_cmd) =
                    self.lateral_guidance.direct_to(state, &waypoint, bank_limit);
                GuidanceTargets {
                    lateral_mode: LateralMode::TrackHold,
                    vertical_mode: VerticalMode::PitchForAirspeed,
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_heading_deg: Some(desired_track),
                    target_waypoint: Some(waypoint),
                    target_speed_kt: Some(perf.vbg_kt),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(0),
                    ..Default::default()
                }
            }
            FlightPhase::PatternEntry => {
                // Orbit a center positioned so one tangent of the
                // circle is at LowKey on downwind heading; circling
                // (CCW for left traffic) bleeds altitude until the
                // exit predicate fires.
                let target = orbit_target_world_ft(
                    state.position_ft,
                    self.spiral_center_world_ft,
                    SPIRAL_RADIUS_FT,
                    SPIRAL_LOOKAHEAD_FT,
                    /* ccw = */ side_sign < 0.0,
                );
                let waypoint = Waypoint {
                    name: "spiral_target".to_string(),
                    position_ft: target,
                    altitude_ft: None,
                };
                let (desired_track, bank_cmd) =
                    self.lateral_guidance.direct_to(state, &waypoint, bank_limit);
                let display_heading = wrap_degrees_360(course + 180.0);
                GuidanceTargets {
                    lateral_mode: LateralMode::TrackHold,
                    vertical_mode: VerticalMode::PitchForAirspeed,
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_heading_deg: Some(display_heading),
                    target_waypoint: Some(waypoint),
                    target_speed_kt: Some(perf.vbg_kt),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(0),
                    ..Default::default()
                }
            }
            FlightPhase::Downwind => {
                let leg = self.pattern.downwind_leg;
                let (desired_track, bank_cmd) =
                    self.lateral_guidance.follow_leg(state, leg, bank_limit);
                let display_heading = wrap_degrees_360(course + 180.0);
                GuidanceTargets {
                    lateral_mode: LateralMode::PathFollow,
                    vertical_mode: VerticalMode::PitchForAirspeed,
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_heading_deg: Some(display_heading),
                    target_path: Some(leg),
                    target_speed_kt: Some(perf.vbg_kt),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(0),
                    ..Default::default()
                }
            }
            FlightPhase::Base => {
                let leg = self.pattern.base_leg;
                let (desired_track, bank_cmd) =
                    self.lateral_guidance.follow_leg(state, leg, bank_limit);
                let display_heading = wrap_degrees_360(course - side_sign * 90.0);
                GuidanceTargets {
                    lateral_mode: LateralMode::PathFollow,
                    vertical_mode: VerticalMode::PitchForAirspeed,
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_heading_deg: Some(display_heading),
                    target_path: Some(leg),
                    target_speed_kt: Some(perf.vbg_kt),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(10),
                    ..Default::default()
                }
            }
            FlightPhase::Final => {
                let leg = self.pattern.final_leg;
                let (desired_track, bank_cmd) =
                    self.lateral_guidance.follow_leg(state, leg, bank_limit);
                let glidepath = Glidepath {
                    slope_deg: 3.0,
                    threshold_crossing_height_ft: 0.0,
                    aimpoint_ft_from_threshold: self.runway_frame.touchdown_runway_x_ft(),
                };
                // Hold best glide all the way down final until short
                // final, then step to vref for the flare. Late flap
                // deployment matches: 20° while we're still protecting
                // glide range, full flaps once the runway is made.
                let on_short_final = state.alt_agl_ft <= SHORT_FINAL_AGL_FT;
                let target_speed = if on_short_final {
                    perf.vref_kt
                } else {
                    perf.vbg_kt
                };
                let flaps = if state.alt_agl_ft > LATE_FLAP_AGL_FT { 20 } else { 30 };
                GuidanceTargets {
                    lateral_mode: LateralMode::PathFollow,
                    vertical_mode: VerticalMode::PitchForAirspeed,
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_heading_deg: Some(course),
                    target_path: Some(leg),
                    target_speed_kt: Some(target_speed),
                    glidepath: Some(glidepath),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(flaps),
                    ..Default::default()
                }
            }
            FlightPhase::Roundout => {
                let (desired_track, bank_cmd) = self.lateral_guidance.follow_leg(
                    state,
                    self.pattern.final_leg,
                    bank_limit.min(8.0),
                );
                let pitch_cmd = 1.5
                    + ((self.config.flare.roundout_height_ft - state.alt_agl_ft).max(0.0)) * 0.08;
                GuidanceTargets {
                    lateral_mode: LateralMode::PathFollow,
                    vertical_mode: VerticalMode::PitchHold,
                    target_path: Some(self.pattern.final_leg),
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_pitch_deg: Some(pitch_cmd),
                    target_speed_kt: Some(perf.vref_kt),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(30),
                    ..Default::default()
                }
            }
            FlightPhase::Flare => {
                let (desired_track, bank_cmd) = self.lateral_guidance.follow_leg(
                    state,
                    self.pattern.final_leg,
                    bank_limit.min(6.0),
                );
                GuidanceTargets {
                    lateral_mode: LateralMode::PathFollow,
                    vertical_mode: VerticalMode::FlareTrack,
                    target_path: Some(self.pattern.final_leg),
                    target_bank_deg: Some(bank_cmd),
                    target_track_deg: Some(desired_track),
                    target_speed_kt: Some(perf.vref_kt - 2.0),
                    throttle_limit: Some((0.0, 0.0)),
                    flaps_cmd: Some(30),
                    brakes: 0.0,
                    ..Default::default()
                }
            }
            FlightPhase::Rollout | FlightPhase::RunwayExit => GuidanceTargets {
                lateral_mode: LateralMode::RolloutCenterline,
                vertical_mode: VerticalMode::PitchHold,
                target_bank_deg: Some(0.0),
                target_heading_deg: Some(course),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 0.0)),
                brakes: 0.75,
                ..Default::default()
            },
            FlightPhase::TaxiClear => GuidanceTargets {
                lateral_mode: LateralMode::RolloutCenterline,
                vertical_mode: VerticalMode::PitchHold,
                target_bank_deg: Some(0.0),
                target_heading_deg: Some(course),
                target_pitch_deg: Some(0.0),
                throttle_limit: Some((0.0, 0.0)),
                brakes: 0.75,
                ..Default::default()
            },
            _ => {
                // Phases the dead-stick profile shouldn't enter — Preflight,
                // TakeoffRoll, Rotate, InitialClimb, Crosswind, EnrouteClimb,
                // Cruise, GoAround. Yield safe zeros so the control loop
                // doesn't blow up if we somehow land here.
                GuidanceTargets {
                    lateral_mode: LateralMode::BankHold,
                    vertical_mode: VerticalMode::PitchHold,
                    target_bank_deg: Some(0.0),
                    target_pitch_deg: Some(0.0),
                    throttle_limit: Some((0.0, 0.0)),
                    ..Default::default()
                }
            }
        }
    }

    pub fn low_key_runway_ft(&self) -> Vec2 {
        self.low_key_runway_ft
    }

    fn captured_at_high_key(&self, state: &AircraftState) -> bool {
        state.position_ft.distance_to(self.high_key_world_ft) <= HIGH_KEY_CAPTURE_RADIUS_FT
    }

    /// Spiral exit predicate: AGL has bled to pattern altitude AND
    /// heading is roughly aligned with downwind direction (so the
    /// transition doesn't dump the aircraft mid-turn pointed the wrong
    /// way).
    fn ready_to_exit_spiral(&self, state: &AircraftState) -> bool {
        if state.alt_agl_ft > SPIRAL_EXIT_AGL_FT {
            return false;
        }
        let downwind_heading = wrap_degrees_360(self.runway_frame.runway.course_deg + 180.0);
        let track_err = wrap_degrees_180(state.track_deg - downwind_heading).abs();
        track_err <= 30.0
    }
}

impl GuidanceProfile for DeadStickLandingProfile {
    fn name(&self) -> &'static str {
        "dead_stick_landing"
    }

    fn owns(&self) -> AxisSet {
        AxisSet::ALL
    }

    fn contribute(&mut self, state: &AircraftState, _dt: f64) -> ProfileTick {
        let previous_phase = self.phase;

        // Suppress go-around: powerless aircraft can't climb. Stash
        // the reason for telemetry; zero the request.
        let mut safety = self.safety_monitor.evaluate(state, self.phase);
        if safety.request_go_around {
            self.last_safety_reason = safety.reason.clone();
            safety.request_go_around = false;
        }

        if self.phase == FlightPhase::Descent {
            if self.captured_at_high_key(state) {
                self.phase = if state.alt_agl_ft > SPIRAL_EXIT_AGL_FT {
                    FlightPhase::PatternEntry
                } else {
                    FlightPhase::Downwind
                };
            }
        } else if self.phase == FlightPhase::PatternEntry {
            if self.ready_to_exit_spiral(state) {
                self.phase = FlightPhase::Downwind;
            }
        } else {
            // Mode_manager's adaptive base-turn relief assumes powered
            // flight: it tightens the pattern when gs is below
            // downwind_speed_kt, but dead-stick is *always* below it,
            // so the relief saturates and base would fire immediately.
            // Gate the turns ourselves on the geometry trigger.
            let mut clearances = PatternClearances::default();
            let rx = state.runway_x_ft.unwrap_or(0.0);
            // Base turn: physically past the nominal base-turn x.
            if rx > self.pattern.base_turn_x_ft {
                clearances.turn_base = false;
            }
            // Final turn: only allow when established on final or
            // most of the way along base.
            let on_final = self.pattern.is_established_on_final(
                state.runway_x_ft,
                state.runway_y_ft,
                state.track_deg,
            );
            let along_base_frac = {
                let leg = self.pattern.base_leg;
                let path = leg.end_ft - leg.start_ft;
                let path_len = path.length().max(1.0);
                let rel = state.position_ft - leg.start_ft;
                (rel.dot(path.normalized()) / path_len).clamp(0.0, 1.0)
            };
            if !on_final && along_base_frac < BASE_LEG_FRACTION_GATE {
                clearances.turn_final = false;
            }
            self.phase = self.mode_manager.update(&ModeManagerUpdate {
                phase: self.phase,
                state,
                route_manager: &self.route_manager,
                pattern: &self.pattern,
                safety_status: &safety,
                triggers: &PatternTriggers::default(),
                clearances: &clearances,
                force_go_around: false,
                stay_in_pattern: true,
                touch_and_go: false,
            });
        }

        let guidance = self.guidance_for_phase(state, self.phase);
        let mut guidance = self.safety_monitor.apply_limits(guidance, self.phase);
        // Belt-and-suspenders: every guidance branch already sets this,
        // but a missing throttle clamp here would re-enable the engine.
        guidance.throttle_limit = Some((0.0, 0.0));

        let hand_off = post_landing_handoff(self.phase, previous_phase, state.gs_kt);

        ProfileTick {
            contribution: guidance_to_contribution(guidance),
            hand_off,
        }
    }

    fn pattern_metadata(&self) -> Option<PatternMetadata> {
        Some(PatternMetadata {
            last_go_around_reason: self.last_safety_reason.clone(),
            airport_ident: self.config.airport.airport.clone(),
            runway_id: self.runway_frame.runway.id.clone(),
            field_elevation_ft: Some(self.config.airport.field_elevation_ft),
            phase: self.phase,
        })
    }

    fn next_transition_hint(&self, state: &AircraftState) -> Option<TransitionHint> {
        let eta_from = |distance_ft: f64| -> Option<f64> {
            if state.gs_kt < 5.0 || distance_ft <= 0.0 {
                return None;
            }
            let gs_fps = state.gs_kt * 1.6878;
            let eta = distance_ft / gs_fps;
            (eta.is_finite() && eta > 0.0).then(|| eta.min(900.0))
        };
        match self.phase {
            FlightPhase::Descent => {
                let dist_to_high_key = state.position_ft.distance_to(self.high_key_world_ft);
                Some(TransitionHint {
                    next_phase: FlightPhase::PatternEntry,
                    condition_text: format!(
                        "within {:.0} ft of high-key fix (now {:.0})",
                        HIGH_KEY_CAPTURE_RADIUS_FT, dist_to_high_key
                    ),
                    eta_s: eta_from(
                        (dist_to_high_key - HIGH_KEY_CAPTURE_RADIUS_FT).max(0.0),
                    ),
                })
            }
            FlightPhase::PatternEntry => Some(TransitionHint {
                next_phase: FlightPhase::Downwind,
                condition_text: format!(
                    "spiral exit: AGL ≤ {:.0} ft and heading toward downwind (now AGL {:.0})",
                    SPIRAL_EXIT_AGL_FT, state.alt_agl_ft
                ),
                eta_s: None,
            }),
            FlightPhase::Downwind => {
                let rx = state.runway_x_ft.unwrap_or(0.0);
                let target_x = self.pattern.base_turn_x_ft;
                let dist_ft = (rx - target_x).max(0.0);
                Some(TransitionHint {
                    next_phase: FlightPhase::Base,
                    condition_text: "abeam base-turn point".to_string(),
                    eta_s: eta_from(dist_ft),
                })
            }
            FlightPhase::Base => Some(TransitionHint {
                next_phase: FlightPhase::Final,
                condition_text: "established on final".to_string(),
                eta_s: None,
            }),
            FlightPhase::Final => {
                let target_agl = self.config.flare.roundout_height_ft;
                let descent_remaining_ft = (state.alt_agl_ft - target_agl).max(0.0);
                let eta = if state.vs_fpm < -10.0 {
                    let descent_fps = (-state.vs_fpm) / 60.0;
                    if descent_fps > 0.5 && descent_remaining_ft > 0.0 {
                        Some((descent_remaining_ft / descent_fps).min(900.0))
                    } else {
                        None
                    }
                } else {
                    None
                };
                Some(TransitionHint {
                    next_phase: FlightPhase::Roundout,
                    condition_text: format!(
                        "AGL ≤ {:.0} ft (now {:.0})",
                        target_agl, state.alt_agl_ft
                    ),
                    eta_s: eta,
                })
            }
            FlightPhase::Roundout => Some(TransitionHint {
                next_phase: FlightPhase::Flare,
                condition_text: format!("AGL ≤ {:.0} ft", self.config.flare.flare_start_ft),
                eta_s: None,
            }),
            FlightPhase::Flare => Some(TransitionHint {
                next_phase: FlightPhase::Rollout,
                condition_text: "wheels on the ground".to_string(),
                eta_s: None,
            }),
            FlightPhase::Rollout => Some(TransitionHint {
                next_phase: FlightPhase::RunwayExit,
                condition_text: "off centerline (>75 ft) or stopped past end".to_string(),
                eta_s: None,
            }),
            FlightPhase::RunwayExit => Some(TransitionHint {
                next_phase: FlightPhase::TaxiClear,
                condition_text: "stopped, off runway".to_string(),
                eta_s: None,
            }),
            _ => None,
        }
    }
}

/// Lyapunov-style orbit target: blends "head inward toward the circle"
/// and "tangent along the circle" using `tanh(radial_error / radius)` —
/// monotonic, never pulls the aircraft the wrong way around.
fn orbit_target_world_ft(
    aircraft_pos_ft: Vec2,
    center_ft: Vec2,
    radius_ft: f64,
    lookahead_ft: f64,
    ccw: bool,
) -> Vec2 {
    let radial = aircraft_pos_ft - center_ft;
    let radial_len = radial.length().max(1.0);
    let radial_unit = radial * (1.0 / radial_len);
    // Rotate radial 90° (CCW for ccw orbit, CW otherwise) to get the
    // tangent direction at the aircraft's projection on the circle.
    let tangent = if ccw {
        Vec2::new(-radial_unit.y, radial_unit.x)
    } else {
        Vec2::new(radial_unit.y, -radial_unit.x)
    };
    let inward = Vec2::new(-radial_unit.x, -radial_unit.y);
    let inward_weight = ((radial_len - radius_ft) / radius_ft.max(1.0)).tanh();
    let tangent_weight = (1.0 - inward_weight.abs()).max(0.0);
    let desired_dir = (tangent * tangent_weight + inward * inward_weight).normalized();
    aircraft_pos_ft + desired_dir * lookahead_ft.max(1.0)
}

/// Pick the entry phase for a fresh dead-stick engagement based on the
/// aircraft's runway-frame position. Made `pub(crate)` so unit tests can
/// exercise the geometry table directly.
pub(crate) fn decide_entry_phase(
    state: &AircraftState,
    runway_frame: &RunwayFrame,
    pattern: &PatternGeometry,
) -> FlightPhase {
    let need_projection = state.runway_x_ft.is_none() || state.runway_y_ft.is_none();
    let projected = need_projection.then(|| runway_frame.to_runway_frame(state.position_ft));
    let rx = state
        .runway_x_ft
        .unwrap_or_else(|| projected.expect("projection computed when needed").x);
    let ry = state
        .runway_y_ft
        .unwrap_or_else(|| projected.expect("projection computed when needed").y);
    let side_sign = match runway_frame.runway.traffic_side {
        TrafficSide::Left => -1.0,
        TrafficSide::Right => 1.0,
    };
    let course = runway_frame.runway.course_deg;
    let track_aligned_with_course =
        wrap_degrees_180(course - state.track_deg).abs() <= 30.0;
    let track_reciprocal = wrap_degrees_180(
        wrap_degrees_360(course + 180.0) - state.track_deg,
    )
    .abs()
        <= 30.0;
    let on_pattern_side = side_sign * ry > 0.0;

    // Final: aligned with runway course on or near the extended centerline,
    // approach side (negative x).
    if rx <= -200.0 && rx >= -15000.0 && ry.abs() <= 300.0 && track_aligned_with_course {
        return FlightPhase::Final;
    }

    // Base: past the base-turn point on the pattern side, generally
    // perpendicular-ish to the runway.
    if rx <= pattern.base_turn_x_ft + 500.0
        && rx >= pattern.base_turn_x_ft - 1500.0
        && on_pattern_side
        && ry.abs() <= 1.5 * pattern.downwind_y_ft.abs()
        && ry.abs() >= 0.2 * pattern.downwind_y_ft.abs()
    {
        return FlightPhase::Base;
    }

    // Downwind: between the join point and base-turn point on the pattern
    // side, parallel to runway in the reciprocal direction.
    let downwind_y_abs = pattern.downwind_y_ft.abs();
    if on_pattern_side
        && (ry.abs() - downwind_y_abs).abs() <= 0.5 * downwind_y_abs
        && rx <= pattern.join_point_runway_ft.x + 500.0
        && rx >= pattern.base_turn_x_ft - 200.0
        && (track_reciprocal || track_aligned_with_course)
    {
        return FlightPhase::Downwind;
    }

    FlightPhase::Descent
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::load_default_config_bundle;
    use crate::types::{AircraftState, Vec2};

    fn airborne_state(position_ft: Vec2, alt_msl: f64, track_deg: f64) -> AircraftState {
        let mut s = AircraftState::synthetic_default();
        s.position_ft = position_ft;
        s.alt_msl_ft = alt_msl;
        s.alt_agl_ft = alt_msl - 500.0;
        s.track_deg = track_deg;
        s.heading_deg = track_deg;
        s.on_ground = false;
        s.gs_kt = 70.0;
        s.ias_kt = 70.0;
        s
    }

    fn fixture_runway_frame() -> RunwayFrame {
        let cfg = load_default_config_bundle();
        RunwayFrame::new(cfg.airport.runway)
    }

    #[test]
    fn entry_phase_far_from_runway_is_descent() {
        let cfg = load_default_config_bundle();
        let rf = fixture_runway_frame();
        let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
        // 5 NM east of the threshold at 5000 ft, heading west.
        let pos = rf.to_world_frame(Vec2::new(0.0, -30000.0));
        let state = airborne_state(pos, 5500.0, 270.0);
        // Clear runway-frame coords so decide_entry_phase falls back to its
        // own runway-frame projection (the synthetic default leaves
        // runway_x_ft / runway_y_ft as None which mirrors the bridge state
        // before the runway is installed in PilotCore).
        let mut state = state;
        state.runway_x_ft = None;
        state.runway_y_ft = None;
        assert_eq!(decide_entry_phase(&state, &rf, &pattern), FlightPhase::Descent);
    }

    #[test]
    fn entry_phase_on_extended_centerline_is_final() {
        let cfg = load_default_config_bundle();
        let rf = fixture_runway_frame();
        let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
        // 5000 ft on final, on centerline, heading runway course (0°).
        let pos = rf.to_world_frame(Vec2::new(-5000.0, 0.0));
        let mut state = airborne_state(pos, 1500.0, rf.runway.course_deg);
        state.runway_x_ft = Some(-5000.0);
        state.runway_y_ft = Some(0.0);
        assert_eq!(decide_entry_phase(&state, &rf, &pattern), FlightPhase::Final);
    }

    #[test]
    fn entry_phase_abeam_base_turn_pattern_side_is_base() {
        let cfg = load_default_config_bundle();
        let rf = fixture_runway_frame();
        let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
        // Pattern side (left → y negative), past base turn x, mid-base
        // (between downwind y and centerline).
        let rf_pos = Vec2::new(pattern.base_turn_x_ft, pattern.downwind_y_ft * 0.5);
        let pos = rf.to_world_frame(rf_pos);
        // Track 90° (heading toward the runway from the west side).
        let mut state = airborne_state(pos, 1300.0, 90.0);
        state.runway_x_ft = Some(rf_pos.x);
        state.runway_y_ft = Some(rf_pos.y);
        assert_eq!(decide_entry_phase(&state, &rf, &pattern), FlightPhase::Base);
    }

    #[test]
    fn entry_phase_on_downwind_pattern_side_is_downwind() {
        let cfg = load_default_config_bundle();
        let rf = fixture_runway_frame();
        let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
        // Mid-downwind on pattern side, heading reciprocal of runway course.
        let mid_x = (pattern.join_point_runway_ft.x + pattern.base_turn_x_ft) * 0.5;
        let rf_pos = Vec2::new(mid_x, pattern.downwind_y_ft);
        let pos = rf.to_world_frame(rf_pos);
        let recip = (rf.runway.course_deg + 180.0).rem_euclid(360.0);
        let mut state = airborne_state(pos, 1500.0, recip);
        state.runway_x_ft = Some(rf_pos.x);
        state.runway_y_ft = Some(rf_pos.y);
        assert_eq!(decide_entry_phase(&state, &rf, &pattern), FlightPhase::Downwind);
    }

    #[test]
    fn contribute_pins_throttle_to_zero() {
        let cfg = load_default_config_bundle();
        let rf = fixture_runway_frame();
        let pattern = build_pattern_geometry(&rf, cfg.pattern.downwind_offset_ft, 0.0, 0.0);
        // Mid-downwind seeding so we engage in Downwind.
        let mid_x = (pattern.join_point_runway_ft.x + pattern.base_turn_x_ft) * 0.5;
        let rf_pos = Vec2::new(mid_x, pattern.downwind_y_ft);
        let pos = rf.to_world_frame(rf_pos);
        let recip = (rf.runway.course_deg + 180.0).rem_euclid(360.0);
        let mut state = airborne_state(pos, 1500.0, recip);
        state.runway_x_ft = Some(rf_pos.x);
        state.runway_y_ft = Some(rf_pos.y);

        let mut profile = DeadStickLandingProfile::new(cfg, rf, &state);
        // A few ticks across phase boundaries — throttle stays at zero.
        for _ in 0..5 {
            let tick = profile.contribute(&state, 0.2);
            let limits = tick
                .contribution
                .throttle_limit
                .expect("dead-stick must always set throttle_limit");
            assert_eq!(limits, (0.0, 0.0));
        }
    }
}
