//! `PlanProfile`: flies an agent-authored plan (see `crate::plan`).
//!
//! Owns all three axes. Lateral steps run in sequence (`.then` chains);
//! altitude and speed are sticky targets that steps and directives set.
//! A pattern step embeds a `PatternFlyProfile` and drives its public turn
//! gates from the plan's triggers, the way `DeadStickLandingProfile`
//! does, so go-arounds, touch-and-goes and the touchdown hand-off to
//! `LandingRolloutProfile` work unchanged. Constraints clamp the altitude
//! target (vertical) or are monitored (lateral), and every notable moment
//! (step done, report due, constraint violated) becomes an event that
//! `PilotCore` turns into a heartbeat.

use std::collections::HashSet;
use std::sync::Arc;

use serde_json::{json, Value};

use crate::config::ConfigBundle;
use crate::core::landing_rollout::LandingPlanHolder;
use crate::core::profiles::{
    AxisSet, GuidanceProfile, PatternFlyProfile, PatternLeg, PatternMetadata, ProfileContribution, ProfileTick,
    TransitionHint,
};
use crate::core::spatial::SpatialOverlay;
use crate::core::traffic::{view as traffic_view, TrafficPicture, TrafficView};
use crate::guidance::geo::{along_track_ft, closest_on_segment, l1_bank_toward, orbit_target_world_ft, point_in_polygon, Polyline};
use crate::guidance::lateral::L1PathFollower;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::plan::compile::{CompiledDelta, RConstraint, RPattern, RReport, RStep, RStepKind, RTrigger};
use crate::plan::cond::{leg_for_phase, EvalCtx, RCond};
use crate::plan::ir::{Dir, Extension, LandKind, Leg};
use crate::plan::resolve::Geom;
use crate::types::{
    clamp, heading_to_vector, vector_to_heading, wrap_degrees_180, wrap_degrees_360, AircraftState, FlightPhase,
    LateralMode, StraightLeg, Vec2, VerticalMode, Waypoint, KT_TO_FPS,
};

/// ACS private-pilot altitude tolerance: an at-or-below/above constraint
/// counts as violated only beyond this.
pub const ALTITUDE_TOLERANCE_FT: f64 = 100.0;
/// Lowest a non-pattern step holds above the ground below (14 CFR 91.119
/// away from congested areas).
const MIN_HOLD_AGL_FT: f64 = 500.0;
/// Aim this far inside an altitude limit so normal TECS wander stays legal.
const ALTITUDE_LIMIT_MARGIN_FT: f64 = 50.0;
const DIRECT_CAPTURE_FT: f64 = 1000.0;
const DIRECT_PASSED_FT: f64 = 4000.0;
const TURN_DONE_DEG: f64 = 5.0;
const TRACK_GAIN: f64 = 0.35;
/// No turns below this after takeoff (AFH): the aircraft holds wings
/// nearly level until it's climbed through it.
const MIN_TURN_AGL_FT: f64 = 400.0;
/// How close to a "remain on this side" line the aircraft may come
/// before it's steered parallel to it.
const LATERAL_BUFFER_FT: f64 = 1500.0;
/// Look this far ahead along the track for a constraint about to be
/// broken.
const LOOKAHEAD_S: f64 = 30.0;
/// Radius of the circle flown short of a line the aircraft may not cross.
const HOLD_SHORT_RADIUS_FT: f64 = 3500.0;

/// On a rectangular course once this close to its track.
const RECT_ON_PATH_FT: f64 = 500.0;
/// Bank limit on a rectangular course (the ACS allows 45° in ground
/// reference maneuvers).
const RECT_MAX_BANK_DEG: f64 = 40.0;

#[derive(Debug, Clone)]
struct StepRun {
    until: Option<RCond>,
    target_track_deg: Option<f64>,
    dir_lock: Option<Dir>,
    orbit_turned_deg: f64,
    orbit_prev_track_deg: f64,
    /// Follow: the line oriented the way it's flown and offset to the
    /// side it's flown on, and the segment being flown.
    follow_line: Option<Polyline>,
    follow_seg: usize,
    /// Circle: turning has started (the aircraft reached the circle).
    circle_on: bool,
    /// Rectangular course: the segment and distance along the path last
    /// tick, whether the aircraft has reached it, and how far it has
    /// flown along it since.
    rect_seg: Option<usize>,
    rect_along_ft: f64,
    rect_on: bool,
    rect_flown_ft: f64,
    /// Direct/toward: the target is across a hard side-of-line
    /// constraint, so the aircraft holds short of the line instead
    /// (the constraint's id, once told).
    holding_short: Option<String>,
}

impl StepRun {
    /// Add the turn since last tick, counting only turning toward `dir`
    /// (orbits and circles).
    fn count_turn(&mut self, dir: Dir, track_deg: f64) {
        let turned = dir.sign() * wrap_degrees_180(track_deg - self.orbit_prev_track_deg);
        if turned > 0.0 {
            self.orbit_turned_deg += turned;
        }
        self.orbit_prev_track_deg = track_deg;
    }
}

/// How far ahead the carrot sits when flying a circle.
fn orbit_lookahead_ft(state: &AircraftState) -> f64 {
    (state.gs_kt * KT_TO_FPS * 8.0).max(800.0)
}

struct PatternRun {
    pf: PatternFlyProfile,
    pattern: RPattern,
    land: LandKind,
    /// Only the takeoff: no crosswind turn; the plan's next step takes
    /// over at 400 ft AGL.
    takeoff_only: bool,
    /// The turn currently armed and its trigger (fresh condition state).
    armed: Option<(Leg, Option<RTrigger>)>,
    last_phase: Option<FlightPhase>,
    leg_since_s: f64,
}

pub struct PlanProfile {
    base_config: ConfigBundle,
    steps: Vec<RStep>,
    idx: usize,
    run: Option<StepRun>,
    pattern: Option<PatternRun>,
    /// Track held when no lateral step is steering (after a turn
    /// completes, between instantaneous steps, after the plan ends).
    hold_track_deg: f64,
    alt_target_ft: f64,
    /// A climb/descend/maintain set `alt_target_ft`; otherwise it's the
    /// altitude the plan started at, and the leg into a pattern entry
    /// descends to pattern altitude by itself.
    alt_explicit: bool,
    /// The altitude floor (MIN_HOLD_AGL_FT) is holding the aircraft up.
    at_altitude_floor: bool,
    speed_kt: f64,
    constraints: Vec<RConstraint>,
    reports: Vec<RReport>,
    gates: HashSet<String>,
    events: Vec<String>,
    prev_pos: Option<crate::types::Vec2>,
    lateral: L1PathFollower,
    t_s: f64,
    /// `speed(...)` while a pattern is flown: replaces its downwind/base
    /// speeds ("keep your speed up", "slowest practical").
    pattern_speed_kt: Option<f64>,
    /// Options ATC granted ("S-turns approved to the west").
    permits: Vec<String>,
    /// Other aircraft, and where they were last tick against the pattern.
    traffic: TrafficPicture,
    traffic_views: Arc<Vec<TrafficView>>,
}

/// The turn the aircraft is waiting to make in `phase`.
fn next_turn(phase: FlightPhase) -> Option<Leg> {
    match phase {
        FlightPhase::InitialClimb => Some(Leg::Crosswind),
        FlightPhase::Crosswind => Some(Leg::Downwind),
        FlightPhase::Downwind => Some(Leg::Base),
        FlightPhase::Base => Some(Leg::Final),
        _ => None,
    }
}

/// The leg whose extension delays the turn onto `turn`.
fn extended_leg_before(turn: Leg) -> Leg {
    match turn {
        Leg::Crosswind => Leg::Upwind,
        Leg::Downwind => Leg::Crosswind,
        Leg::Base => Leg::Downwind,
        Leg::Final => Leg::Base,
        Leg::Upwind => Leg::Upwind,
    }
}

fn pattern_leg(l: Leg) -> Option<PatternLeg> {
    match l {
        Leg::Crosswind => Some(PatternLeg::Crosswind),
        Leg::Downwind => Some(PatternLeg::Downwind),
        Leg::Base => Some(PatternLeg::Base),
        Leg::Final => Some(PatternLeg::Final),
        Leg::Upwind => None,
    }
}

fn set_turn_clearance(pf: &mut PatternFlyProfile, leg: Leg, granted: bool) {
    match leg {
        Leg::Crosswind => pf.clearances.turn_crosswind = granted,
        Leg::Downwind => pf.clearances.turn_downwind = granted,
        Leg::Base => pf.clearances.turn_base = granted,
        Leg::Final => pf.clearances.turn_final = granted,
        Leg::Upwind => {}
    }
}

/// Bank toward `target`, optionally forcing the turn direction until
/// within 5° (the `HeadingHoldProfile` law).
fn track_hold_bank(track_deg: f64, target_deg: f64, lock: &mut Option<Dir>, max_bank: f64) -> f64 {
    let raw = (target_deg - track_deg).rem_euclid(360.0);
    let short = if raw > 180.0 { raw - 360.0 } else { raw };
    if short.abs() < TURN_DONE_DEG {
        *lock = None;
    }
    let err = match lock {
        Some(Dir::Right) => raw,
        Some(Dir::Left) => raw - 360.0,
        None => short,
    };
    clamp(err * TRACK_GAIN, -max_bank, max_bank)
}

impl PlanProfile {
    pub fn new(base_config: ConfigBundle, state: &AircraftState) -> Self {
        let perf = &base_config.performance;
        let speed = clamp((state.ias_kt / 5.0).round() * 5.0, perf.vy_kt, perf.cruise_speed_kt);
        // Low and climbing (just off the runway): keep climbing to
        // pattern altitude rather than levelling off where the plan
        // happened to start.
        let pattern_alt = base_config.pattern_altitude_msl_ft();
        let alt_target = if !state.on_ground && state.alt_msl_ft < pattern_alt - 100.0 && state.vs_fpm > 100.0 {
            pattern_alt
        } else {
            (state.alt_msl_ft / 100.0).round() * 100.0
        };
        Self {
            steps: Vec::new(),
            idx: 0,
            run: None,
            pattern: None,
            hold_track_deg: state.track_deg,
            alt_target_ft: alt_target,
            alt_explicit: false,
            at_altitude_floor: false,
            speed_kt: speed,
            constraints: Vec::new(),
            reports: Vec::new(),
            gates: HashSet::new(),
            events: Vec::new(),
            prev_pos: None,
            lateral: L1PathFollower::new(),
            t_s: 0.0,
            pattern_speed_kt: None,
            permits: Vec::new(),
            traffic: TrafficPicture::default(),
            traffic_views: Arc::new(Vec::new()),
            base_config,
        }
    }

    /// Apply one `fly` call's worth of changes. Returns notes for the tool
    /// result.
    pub fn apply(&mut self, delta: CompiledDelta, state: &AircraftState) -> Vec<String> {
        let mut notes = Vec::new();
        let fresh = self.steps.is_empty() && self.constraints.is_empty() && self.t_s == 0.0;
        if delta.replace {
            if !self.constraints.is_empty() || !self.reports.is_empty() {
                notes.push("earlier constraints and reports dropped (new clearance)".into());
            }
            self.constraints.clear();
            self.reports.clear();
        }
        for id in &delta.cancel {
            let before = self.constraints.len() + self.reports.len();
            self.constraints.retain(|c| &c.id != id);
            self.reports.retain(|r| &r.id != id);
            if self.constraints.len() + self.reports.len() == before {
                notes.push(format!("cancel: no constraint or report {id}"));
            } else {
                notes.push(format!("cancelled {id}"));
            }
        }
        for g in &delta.release {
            self.gates.insert(g.clone());
            notes.push(format!("released ATC gate {g:?}"));
        }
        self.constraints.extend(delta.constraints);
        self.reports.extend(delta.reports);
        self.permits.extend(delta.permits.iter().cloned());
        if delta.go_around {
            match &mut self.pattern {
                Some(run) => {
                    run.pf.go_around();
                    notes.push("going around".into());
                }
                None => notes.push("go_around: no pattern is being flown".into()),
            }
        }

        if let Some(edit) = delta.pattern {
            let mut applied = false;
            if let Some(run) = &mut self.pattern {
                if run.pattern.same_runway(&edit) {
                    apply_pattern_edit(run, edit.clone());
                    applied = true;
                }
            }
            for s in self.steps.iter_mut().skip(self.idx) {
                if let RStepKind::Pattern { pattern, .. } = &mut s.kind {
                    if pattern.same_runway(&edit) {
                        **pattern = edit.clone();
                        applied = true;
                    }
                }
            }
            if !applied {
                notes.push(format!(
                    "pattern edit for {} {} has no pattern step to apply to; add join(p) to fly it",
                    edit.spec.airport, edit.spec.runway
                ));
            }
        }

        if let Some(steps) = delta.steps {
            // Keep a running pattern if the new sequence starts with the
            // same runway's pattern ("continue right traffic").
            let keep = match (&self.pattern, steps.first().map(|s| &s.kind)) {
                (Some(run), Some(RStepKind::Pattern { pattern, .. })) => run.pattern.same_runway(pattern),
                _ => false,
            };
            if !keep {
                self.pattern = None;
            }
            self.steps = steps;
            self.idx = 0;
            self.run = None;
            if keep {
                if let (Some(run), Some(RStepKind::Pattern { pattern, land, .. })) =
                    (&mut self.pattern, self.steps.first().map(|s| s.kind.clone()))
                {
                    apply_pattern_edit(run, *pattern);
                    run.land = land;
                }
                self.run = Some(self.fresh_run(0));
            }
            // Build a leading pattern now rather than on the next tick, so
            // the tools that act on it (plan_arrival, go_around) work in
            // the same turn; the sim may be paused while the LLM thinks.
            if self.run.is_none()
                && matches!(self.steps.first().map(|s| &s.kind), Some(RStepKind::Pattern { .. }) | Some(RStepKind::Takeoff { .. }))
            {
                self.activate(state);
            }
        }

        let had_directives = !delta.directives.is_empty();
        for d in delta.directives {
            self.apply_vertical(&d.kind);
        }
        let flies_pattern_first =
            matches!(self.steps.first().map(|s| &s.kind), Some(RStepKind::Pattern { .. }));
        if fresh && !had_directives && !flies_pattern_first {
            notes.push(match self.entry_ahead() {
                Some((leg, alt)) if !self.alt_explicit && alt < self.alt_target_ft - ALTITUDE_TOLERANCE_FT => format!(
                    "descending to pattern altitude {alt:.0} on the way to the {} entry, at {:.0} kt, unless a \
                     climb/descend/maintain/speed says otherwise",
                    leg.as_str(),
                    self.speed_kt
                ),
                _ => format!(
                    "holding {:.0} ft and {:.0} kt until a climb/descend/speed says otherwise",
                    self.alt_target_ft, self.speed_kt
                ),
            });
        }
        notes
    }

    /// The pattern entry the current non-pattern steps lead into, if any:
    /// its leg and pattern altitude.
    fn entry_ahead(&self) -> Option<(Leg, f64)> {
        if self.is_finished() {
            return None;
        }
        for step in self.steps.iter().skip(self.idx) {
            match &step.kind {
                RStepKind::Pattern { pattern, entry, .. } => {
                    let leg = (*entry)?;
                    let alt = pattern.spec.altitude_ft.unwrap_or_else(|| pattern.config.pattern_altitude_msl_ft());
                    return Some((leg, alt));
                }
                RStepKind::Takeoff { .. } => return None,
                RStepKind::Climb { .. } | RStepKind::Descend { .. } | RStepKind::Maintain { .. } => return None,
                _ => {}
            }
        }
        None
    }

    /// The altitude to fly before constraints: the target, or pattern
    /// altitude on the leg into a pattern entry when nothing set one.
    fn alt_goal(&self) -> f64 {
        match self.entry_ahead() {
            Some((_, alt)) if !self.alt_explicit => self.alt_target_ft.min(alt),
            _ => self.alt_target_ft,
        }
    }

    /// A copy to fly ahead in a preview sim (`plan::preview`): same steps,
    /// conditions and progress. It sees `traffic` instead of the live
    /// picture, so the preview can move traffic on its own clock.
    pub fn preview_copy(&self, traffic: TrafficPicture) -> Self {
        Self {
            base_config: self.base_config.clone(),
            steps: self.steps.clone(),
            idx: self.idx,
            run: self.run.clone(),
            pattern: self.pattern.as_ref().map(|r| PatternRun {
                pf: r.pf.preview_copy(),
                pattern: r.pattern.clone(),
                land: r.land,
                takeoff_only: r.takeoff_only,
                armed: r.armed.clone(),
                last_phase: r.last_phase,
                leg_since_s: r.leg_since_s,
            }),
            hold_track_deg: self.hold_track_deg,
            alt_target_ft: self.alt_target_ft,
            alt_explicit: self.alt_explicit,
            at_altitude_floor: self.at_altitude_floor,
            speed_kt: self.speed_kt,
            constraints: self.constraints.clone(),
            reports: self.reports.clone(),
            gates: self.gates.clone(),
            events: Vec::new(),
            prev_pos: self.prev_pos,
            lateral: self.lateral.clone(),
            t_s: self.t_s,
            pattern_speed_kt: self.pattern_speed_kt,
            permits: self.permits.clone(),
            traffic,
            traffic_views: self.traffic_views.clone(),
        }
    }

    /// Every step flown (holding track and altitude now).
    /// Hard constraints not met right now.
    pub fn violated_constraints(&self) -> impl Iterator<Item = &RConstraint> {
        self.constraints.iter().filter(|c| c.violating && !c.soft)
    }

    /// Every step flown (a plan of only constraints never finishes).
    pub fn is_finished(&self) -> bool {
        !self.steps.is_empty() && self.idx >= self.steps.len()
    }

    /// Sense traffic from `picture` (the pilot's shared picture).
    pub fn with_traffic(mut self, picture: TrafficPicture) -> Self {
        self.traffic = picture;
        self
    }

    /// The runway traffic is placed against: the pattern flown or next,
    /// else the pilot's reference runway.
    fn traffic_frame(&self) -> RunwayFrame {
        match self.upcoming_pattern() {
            Some(p) => p.frame.clone(),
            None => RunwayFrame::new(self.base_config.airport.runway.clone()),
        }
    }

    /// The pattern being flown, else the next pattern step's.
    fn upcoming_pattern(&self) -> Option<&RPattern> {
        self.pattern.as_ref().map(|r| &r.pattern).or_else(|| {
            self.steps.iter().skip(self.idx).find_map(|s| match &s.kind {
                RStepKind::Pattern { pattern, .. } => Some(pattern.as_ref()),
                _ => None,
            })
        })
    }

    /// Traffic descriptions the plan's conditions have bound to targets.
    fn traffic_bindings(&self) -> Vec<(String, String)> {
        let mut out = Vec::new();
        for c in &self.constraints {
            c.pred.traffic_bindings(&mut out);
            if let Some(u) = &c.until {
                u.traffic_bindings(&mut out);
            }
        }
        for r in &self.reports {
            r.when.traffic_bindings(&mut out);
        }
        if let Some(u) = self.run.as_ref().and_then(|r| r.until.as_ref()) {
            u.traffic_bindings(&mut out);
        }
        if let Some((_, Some(t))) = self.pattern.as_ref().and_then(|r| r.armed.as_ref()) {
            for c in [&t.when, &t.earliest, &t.latest].into_iter().flatten() {
                c.traffic_bindings(&mut out);
            }
        }
        out
    }

    /// The runway the plan's current or next pattern uses, as
    /// (airport, runway).
    pub fn pattern_runway(&self) -> Option<(String, String)> {
        let p = self.upcoming_pattern()?;
        Some((p.spec.airport.clone(), p.spec.runway.clone()))
    }

    fn fresh_run(&self, idx: usize) -> StepRun {
        StepRun {
            until: self.steps.get(idx).and_then(|s| s.until.clone()),
            target_track_deg: None,
            dir_lock: None,
            orbit_turned_deg: 0.0,
            orbit_prev_track_deg: 0.0,
            follow_line: None,
            follow_seg: 0,
            circle_on: false,
            rect_seg: None,
            rect_along_ft: 0.0,
            rect_on: false,
            rect_flown_ft: 0.0,
            holding_short: None,
        }
    }

    fn apply_vertical(&mut self, k: &RStepKind) {
        match k {
            RStepKind::Climb { altitude_ft, speed_kt } | RStepKind::Descend { altitude_ft, speed_kt } => {
                self.alt_target_ft = *altitude_ft;
                self.alt_explicit = true;
                if let Some(s) = speed_kt {
                    self.speed_kt = *s;
                }
            }
            RStepKind::Maintain { altitude_ft } => {
                self.alt_target_ft = *altitude_ft;
                self.alt_explicit = true;
            }
            RStepKind::Speed { kt } => {
                self.speed_kt = *kt;
                self.pattern_speed_kt = Some(*kt);
            }
            _ => {}
        }
    }

    fn event(&mut self, text: String) {
        self.events.push(text);
    }

    /// Start step `idx`: capture headings, build the pattern.
    fn activate(&mut self, state: &AircraftState) {
        let mut run = self.fresh_run(self.idx);
        let kind = self.steps[self.idx].kind.clone();
        match &kind {
            RStepKind::Heading { heading_deg, turn } => {
                run.target_track_deg = Some(heading_deg.unwrap_or(state.track_deg));
                run.dir_lock = *turn;
            }
            RStepKind::Turn { dir, by_deg, to_deg } => {
                let target = match (by_deg, to_deg) {
                    (Some(b), _) => state.track_deg + dir.sign() * b,
                    (None, Some(t)) => *t,
                    _ => state.track_deg,
                };
                run.target_track_deg = Some(wrap_degrees_360(target));
                run.dir_lock = Some(*dir);
            }
            RStepKind::Orbit { dir, degrees, .. } => {
                run.orbit_prev_track_deg = state.track_deg;
                // Rolls out (and holds) the entry track turned through.
                run.target_track_deg = Some(wrap_degrees_360(state.track_deg + dir.sign() * degrees));
            }
            RStepKind::Climb { .. } | RStepKind::Descend { .. } | RStepKind::Maintain { .. } | RStepKind::Speed { .. } => {
                self.apply_vertical(&kind);
            }
            RStepKind::Pattern { pattern, entry, land } => {
                let reuse = self.pattern.as_ref().is_some_and(|r| r.pattern.same_runway(pattern) && !r.takeoff_only);
                if !reuse {
                    self.pattern = Some(start_pattern(pattern, *entry, *land, state));
                }
                self.pattern_speed_kt = None;
            }
            RStepKind::Takeoff { pattern } => {
                let mut run = start_pattern(pattern, None, LandKind::FullStop, state);
                run.takeoff_only = true;
                run.pf.clearances.turn_crosswind = false;
                self.pattern = Some(run);
            }
            RStepKind::Follow { line, direction_deg, keep, side_deg, offset_ft, .. } => {
                run.follow_line = Some(orient_and_offset(line, state, *direction_deg, *keep, *side_deg, *offset_ft));
            }
            RStepKind::Circle { .. } => {
                run.orbit_prev_track_deg = state.track_deg;
            }
            RStepKind::RectangularCourse { altitude_ft, .. } => {
                if let Some(a) = altitude_ft {
                    self.alt_target_ft = *a;
                    self.alt_explicit = true;
                }
            }
            RStepKind::Direct { .. } | RStepKind::Toward { .. } => {}
        }
        self.run = Some(run);
    }

    /// Has the active step finished by itself (not counting `until`)?
    fn naturally_done(&mut self, state: &AircraftState) -> bool {
        let Some(run) = &mut self.run else { return false };
        match &self.steps[self.idx].kind {
            RStepKind::Turn { .. } => run.target_track_deg.is_some_and(|t| {
                run.dir_lock.is_none() && wrap_degrees_180(t - state.track_deg).abs() < TURN_DONE_DEG
            }),
            RStepKind::Direct { target } => {
                let p = target.geom.anchor();
                let d = state.position_ft.distance_to(p);
                d < DIRECT_CAPTURE_FT || (d < DIRECT_PASSED_FT && along_track_ft(state.position_ft, state.track_deg, p) < 0.0)
            }
            RStepKind::Orbit { degrees, .. } => run.orbit_turned_deg >= *degrees - 3.0,
            RStepKind::Climb { altitude_ft, .. } | RStepKind::Descend { altitude_ft, .. } => {
                (state.alt_msl_ft - altitude_ft).abs() <= ALTITUDE_TOLERANCE_FT
            }
            RStepKind::Maintain { .. } | RStepKind::Speed { .. } => true,
            RStepKind::Follow { .. } => run.follow_line.as_ref().is_some_and(|l| {
                let n = l.points.len();
                n < 2 || (run.follow_seg + 2 >= n && along_track_ft(state.position_ft, vector_to_heading(l.points[n - 1] - l.points[n - 2]), l.points[n - 1]) < 0.0)
            }),
            RStepKind::Circle { turns, .. } => turns.is_some_and(|n| run.orbit_turned_deg >= 360.0 * n - 5.0),
            RStepKind::RectangularCourse { path, laps, .. } => {
                laps.is_some_and(|n| run.rect_flown_ft >= n * path.length_ft - RECT_ON_PATH_FT)
            }
            RStepKind::Takeoff { .. } => !state.on_ground && state.alt_agl_ft >= MIN_TURN_AGL_FT,
            RStepKind::Heading { .. } | RStepKind::Toward { .. } | RStepKind::Pattern { .. } => false,
        }
    }

    /// A climb/descend the constraints won't let finish ("descend to 1800"
    /// under "remain at or above 2300"): once level at the limit, end the
    /// step so the plan moves on, and say why.
    fn vertical_step_blocked(&mut self, state: &AircraftState) -> bool {
        let target = match &self.steps[self.idx].kind {
            RStepKind::Climb { altitude_ft, .. } | RStepKind::Descend { altitude_ft, .. } => *altitude_ft,
            _ => return false,
        };
        let held = self.clamp_altitude(target);
        if (held - target).abs() <= ALTITUDE_TOLERANCE_FT || (state.alt_msl_ft - held).abs() > ALTITUDE_TOLERANCE_FT {
            return false;
        }
        let text = self.steps[self.idx].text.clone();
        self.event(format!(
            "{text}: held at {held:.0} by a constraint; continuing with the next step, and the {target:.0} target \
             applies once the constraint ends"
        ));
        true
    }

    fn current_leg(&self) -> Option<Leg> {
        self.pattern.as_ref().and_then(|r| leg_for_phase(r.pf.phase))
    }

    fn vertical_limits(&self) -> (f64, f64) {
        let mut lo = f64::NEG_INFINITY;
        let mut hi = f64::INFINITY;
        for c in &self.constraints {
            if !c.enforced() {
                continue;
            }
            match &c.pred {
                RCond::AtOrBelow(a) => hi = hi.min(a - ALTITUDE_LIMIT_MARGIN_FT),
                RCond::AtOrAbove(a) => lo = lo.max(a + ALTITUDE_LIMIT_MARGIN_FT),
                _ => {}
            }
        }
        (lo, hi)
    }

    fn clamp_altitude(&self, alt: f64) -> f64 {
        clamp_to_limits(alt, self.vertical_limits())
    }

    fn update_constraints_and_reports(&mut self, ctx: &EvalCtx) {
        let mut events = Vec::new();
        for c in &mut self.constraints {
            if c.ended {
                continue;
            }
            if let Some(u) = &mut c.until {
                if u.eval(ctx) {
                    c.ended = true;
                    events.push(format!("constraint {} ended: {}", c.id, c.text));
                    continue;
                }
            }
            if !c.started {
                if let Some(a) = &mut c.after {
                    c.started = a.eval(ctx);
                }
                if !c.started {
                    continue;
                }
            }
            let holds = c.pred.holds_with_alt_tolerance(ctx, ALTITUDE_TOLERANCE_FT);
            if !holds && !c.violating {
                c.violating = true;
                events.push(format!(
                    "{} {} {}: {}",
                    if c.soft { "preference" } else { "constraint" },
                    c.id,
                    "not met",
                    c.text
                ));
            } else if holds && c.violating {
                c.violating = false;
                events.push(format!("constraint {} met again: {}", c.id, c.text));
            }
        }
        for r in &mut self.reports {
            if !r.fired && r.when.eval(ctx) {
                r.fired = true;
                events.push(format!("report due ({}): {}", r.id, r.text));
            }
        }
        self.events.extend(events);
    }

    fn open_items(&self) -> Vec<String> {
        let mut out = Vec::new();
        for c in &self.constraints {
            if !c.ended {
                if let Some(u) = &c.until {
                    let mut o = Vec::new();
                    u.open_items(&self.gates, &mut o);
                    out.extend(o.into_iter().map(|s| format!("{} ({})", s, c.id)));
                }
            }
        }
        if let Some(run) = &self.pattern {
            if let Some((leg, trig)) = &run.armed {
                let open_ext = matches!(run.pattern.extensions.get(&extended_leg_before(*leg)), Some(Extension::Open(_)));
                let mut o = Vec::new();
                if let Some(t) = trig {
                    for c in [&t.when, &t.earliest, &t.latest].into_iter().flatten() {
                        c.open_items(&self.gates, &mut o);
                    }
                }
                if open_ext && trig.as_ref().is_none_or(|t| t.when.is_none() && t.latest.is_none()) {
                    o.push(format!(
                        "{} extended open: no turn {} until a later instruction",
                        extended_leg_before(*leg).as_str(),
                        leg.as_str()
                    ));
                }
                out.extend(o.into_iter().map(|s| format!("{s} (turn {})", leg.as_str())));
            }
        }
        if let Some(run) = &self.run {
            if let Some(u) = &run.until {
                u.open_items(&self.gates, &mut out);
            }
        }
        out.sort();
        out.dedup();
        out
    }

    /// Summary for the status payload and the Python context.
    pub fn status_json(&self) -> Value {
        let live_pattern = self.upcoming_pattern();
        let mut gates: Vec<String> = self.gates.iter().cloned().collect();
        gates.sort();
        json!({
            "steps": self.steps.iter().map(|s| s.text.clone()).collect::<Vec<_>>(),
            "active": if self.is_finished() { Value::Null } else { json!(self.idx) },
            "active_step": self.steps.get(self.idx).filter(|_| !self.is_finished()).map(|s| s.text.clone()),
            "complete": self.is_finished(),
            "altitude_target_ft": self.clamp_altitude(self.alt_goal()).round(),
            "speed_target_kt": self.speed_kt.round(),
            "pattern": live_pattern.map(|p| serde_json::to_value(&p.spec).unwrap_or(Value::Null)),
            "pattern_summary": live_pattern.map(|p| p.describe()),
            "next_turn": self.pattern.as_ref().and_then(|r| r.armed.as_ref()).map(|(l, t)| json!({
                "leg": l.as_str(),
                "trigger": t.as_ref().map(|t| t.text.clone()).unwrap_or_else(|| "normal turn point".into()),
            })),
            "constraints": self.constraints.iter().map(|c| json!({
                "id": c.id, "text": c.text,
                "status": if c.ended { "ended" } else if !c.started { "pending" } else if c.violating { "violated" } else { "active" },
                "soft": c.soft,
            })).collect::<Vec<_>>(),
            "reports": self.reports.iter().map(|r| json!({"id": r.id, "text": r.text, "done": r.fired})).collect::<Vec<_>>(),
            "atc_gates_open": gates,
            "traffic_identified": self.traffic_bindings().into_iter().map(|(desc, id)| {
                let w = self.traffic_views.iter().find(|v| v.target.id == id);
                json!({"as": desc, "id": id, "where": w.map(|v| v.where_.clone()).unwrap_or_else(|| "gone (landed)".into())})
            }).collect::<Vec<_>>(),
            "permitted": self.permits,
            "open_items": self.open_items(),
        })
    }

    fn non_pattern_contribution(&mut self, state: &AircraftState, max_bank: f64) -> ProfileContribution {
        // Borrowed out of `self` for the tick (no per-tick copy of a
        // road's or course's points); put back before anything reads it.
        let steps = std::mem::take(&mut self.steps);
        let step = steps.get(self.idx);
        let step_kind = step.map(|s| &s.kind);
        let mut holding_short = false;
        let (mut bank, mut track) = match (step_kind, &mut self.run) {
            (Some(RStepKind::Heading { .. }) | Some(RStepKind::Turn { .. }), Some(run)) => {
                let target = run.target_track_deg.unwrap_or(self.hold_track_deg);
                self.hold_track_deg = target;
                (track_hold_bank(state.track_deg, target, &mut run.dir_lock, max_bank), target)
            }
            (Some(RStepKind::Direct { target }) | Some(RStepKind::Toward { target }), _) => {
                let mut aim = target.geom.anchor();
                match self.hold_short_of(aim) {
                    Some((id, text, center)) => {
                        if self.run.as_ref().is_some_and(|r| r.holding_short.as_deref() != Some(id.as_str())) {
                            if let Some(r) = self.run.as_mut() {
                                r.holding_short = Some(id.clone());
                            }
                            self.event(format!(
                                "{} is across {id} ({text}); circling short of it until that ends",
                                target.label
                            ));
                        }
                        aim = orbit_target_world_ft(state.position_ft, center, HOLD_SHORT_RADIUS_FT, orbit_lookahead_ft(state), false);
                        holding_short = true;
                    }
                    None => {
                        if let Some(r) = self.run.as_mut() {
                            r.holding_short = None;
                        }
                    }
                }
                let wp = Waypoint { name: target.label.clone(), position_ft: aim, altitude_ft: None };
                let (trk, bank) = self.lateral.direct_to(state, &wp, max_bank);
                self.hold_track_deg = trk;
                (bank, trk)
            }
            (Some(RStepKind::Orbit { dir, bank_deg, .. }), Some(run)) => {
                run.count_turn(*dir, state.track_deg);
                self.hold_track_deg = run.target_track_deg.unwrap_or(self.hold_track_deg);
                // The orbit's own bank (compile clamps it to the en-route
                // limit), so a steep turn gets its 45°.
                (dir.sign() * bank_deg, state.track_deg)
            }
            (Some(RStepKind::Follow { .. }), Some(run)) => {
                let (trk, bank) = follow_guidance(&self.lateral, state, run, max_bank);
                self.hold_track_deg = trk;
                (bank, trk)
            }
            (Some(RStepKind::Circle { target, dir, radius_ft, .. }), Some(run)) => {
                let center = target.geom.anchor();
                let dist = state.position_ft.distance_to(center);
                if !run.circle_on && dist <= radius_ft * 1.3 {
                    run.circle_on = true;
                    run.orbit_prev_track_deg = state.track_deg;
                }
                if run.circle_on {
                    run.count_turn(*dir, state.track_deg);
                }
                let carrot = orbit_target_world_ft(state.position_ft, center, *radius_ft, orbit_lookahead_ft(state), *dir == Dir::Left);
                let wp = Waypoint { name: target.label.clone(), position_ft: carrot, altitude_ft: None };
                let (trk, bank) = self.lateral.direct_to(state, &wp, max_bank);
                self.hold_track_deg = trk;
                (bank, trk)
            }
            (Some(RStepKind::RectangularCourse { path, .. }), Some(run)) => {
                let n = path.nearest(state.position_ft, run.rect_seg.filter(|_| run.rect_on));
                let mut lap_done = None;
                if run.rect_on {
                    let d = path.progress(run.rect_along_ft, n.along_ft);
                    if d > 0.0 {
                        let before = (run.rect_flown_ft / path.length_ft).floor();
                        run.rect_flown_ft += d;
                        let after = (run.rect_flown_ft / path.length_ft).floor();
                        if after > before {
                            lap_done = Some(after);
                        }
                    }
                } else if n.distance_ft <= RECT_ON_PATH_FT {
                    run.rect_on = true;
                }
                run.rect_seg = Some(n.segment);
                run.rect_along_ft = n.along_ft;
                // L1 guidance: the carrot is where the path is L1 away
                // (at least a third of L1 ahead, so a far-off aircraft
                // intercepts at an angle rather than square on).
                let l1 = (state.gs_kt * KT_TO_FPS * 5.0).clamp(500.0, 1800.0);
                let ahead = (l1 * l1 - n.distance_ft * n.distance_ft).max(0.0).sqrt().max(l1 / 3.0);
                let carrot = path.point_at(n.along_ft + ahead);
                let (trk, bank) = l1_bank_toward(state.position_ft, state.track_deg, state.gs_kt, carrot, max_bank.min(RECT_MAX_BANK_DEG));
                if let (Some(k), Some(step)) = (lap_done, step) {
                    self.event(format!("{}: lap {k:.0} done", step.text));
                }
                self.hold_track_deg = trk;
                (bank, trk)
            }
            _ => {
                let mut none = None;
                (track_hold_bank(state.track_deg, self.hold_track_deg, &mut none, max_bank), self.hold_track_deg)
            }
        };
        self.steps = steps;
        // Steer clear of lateral constraints (remain north of the 118,
        // over the 101, outside the Class D) before they're broken.
        if let Some(t) = self.enforce_lateral(state, track).filter(|_| !holding_short) {
            track = t;
            let mut none = None;
            bank = track_hold_bank(state.track_deg, t, &mut none, max_bank);
        }
        // No turns until 400 ft AGL after takeoff.
        if !state.on_ground && state.alt_agl_ft < MIN_TURN_AGL_FT && state.vs_fpm > 100.0 {
            bank = clamp(bank, -5.0, 5.0);
        }
        // Never hold lower than MIN_HOLD_AGL_FT above the ground below.
        let floor = state.alt_msl_ft - state.alt_agl_ft + MIN_HOLD_AGL_FT;
        let wanted = self.clamp_altitude(self.alt_goal());
        let at_floor = !state.on_ground && wanted < floor;
        if at_floor && !self.at_altitude_floor {
            self.event(format!(
                "holding {floor:.0} ({MIN_HOLD_AGL_FT:.0} ft above the ground): the plan's altitude {wanted:.0} is too low"
            ));
        }
        self.at_altitude_floor = at_floor;
        let alt = if at_floor { floor } else { wanted };
        let tecs_phase = if alt > state.alt_msl_ft + 200.0 {
            FlightPhase::EnrouteClimb
        } else if alt < state.alt_msl_ft - 200.0 {
            FlightPhase::Descent
        } else {
            FlightPhase::Cruise
        };
        ProfileContribution {
            lateral_mode: Some(LateralMode::TrackHold),
            vertical_mode: Some(VerticalMode::Tecs),
            target_bank_deg: Some(bank),
            target_track_deg: Some(track),
            target_heading_deg: Some(track),
            target_altitude_ft: Some(alt),
            target_speed_kt: Some(self.speed_kt),
            throttle_limit: Some((0.0, 1.0)),
            tecs_phase_override: Some(tecs_phase),
            flaps_cmd: Some(0),
            ..Default::default()
        }
    }

    /// Where to hold when `target` lies across a hard side-of-line
    /// constraint ("direct the golf course" while "remain south of the
    /// 101"): a circle on the allowed side, clear of the line, abeam the
    /// target. Returns the constraint's id and text and the circle's center.
    fn hold_short_of(&self, target: Vec2) -> Option<(String, String, Vec2)> {
        for c in &self.constraints {
            if !c.enforced() {
                continue;
            }
            let RCond::SideOf { feature, side_deg } = &c.pred else { continue };
            let Geom::Line(line) = &feature.geom else { continue };
            if line.is_on_side(target, *side_deg) {
                continue;
            }
            let n = line.nearest(target)?;
            let course = line.local_course_deg(n.point, 0.0)?;
            let away = away_heading(course, *side_deg);
            let center = n.point + heading_to_vector(away, LATERAL_BUFFER_FT + HOLD_SHORT_RADIUS_FT + 500.0);
            return Some((c.id.clone(), c.text.clone(), center));
        }
        None
    }

    /// A track that keeps the aircraft inside its hard lateral
    /// constraints, if `desired_track` would break one within
    /// `LOOKAHEAD_S` (or already has).
    fn enforce_lateral(&self, state: &AircraftState, desired_track: f64) -> Option<f64> {
        let speed = (state.gs_kt * KT_TO_FPS).max(80.0);
        let ahead = state.position_ft + heading_to_vector(desired_track, speed * LOOKAHEAD_S);
        for c in &self.constraints {
            if !c.enforced() {
                continue;
            }
            match &c.pred {
                RCond::SideOf { feature, side_deg } => {
                    let Geom::Line(line) = &feature.geom else { continue };
                    let Some(n) = line.nearest(state.position_ft) else { continue };
                    let on_side = line.is_on_side(state.position_ft, *side_deg);
                    let ahead_ok = line.is_on_side(ahead, *side_deg);
                    let margin = if on_side { n.distance_ft } else { -n.distance_ft };
                    if ahead_ok && margin >= LATERAL_BUFFER_FT {
                        continue;
                    }
                    // Parallel to the line the way the aircraft is going (the
                    // desired track can be perpendicular to it, a tie).
                    let course = line.local_course_deg(state.position_ft, state.track_deg)?;
                    // The way into the allowed side, perpendicular to the line.
                    let away = away_heading(course, *side_deg);
                    let tilt = if margin < 0.0 { 45.0 } else if !ahead_ok { 25.0 } else { 10.0 };
                    let toward_line = heading_to_vector(desired_track, 1.0).dot(heading_to_vector(away, 1.0)) < 0.0;
                    if margin >= LATERAL_BUFFER_FT * 0.5 && !toward_line && ahead_ok {
                        continue;
                    }
                    return Some(turn_toward(course, away, tilt));
                }
                RCond::Over { feature, width_ft } => {
                    let Geom::Line(line) = &feature.geom else { continue };
                    let Some(n) = line.nearest(state.position_ft) else { continue };
                    let course = line.local_course_deg(state.position_ft, state.track_deg)?;
                    if n.distance_ft <= width_ft * 0.5 && wrap_degrees_180(desired_track - course).abs() <= 30.0 {
                        continue;
                    }
                    let to_line = vector_to_heading(n.point - state.position_ft);
                    let tilt = clamp(n.distance_ft / width_ft.max(1.0) * 30.0, 0.0, 45.0);
                    return Some(turn_toward(course, to_line, tilt));
                }
                RCond::Outside { feature } => {
                    let (center, inside_ahead) = match &feature.geom {
                        Geom::Area { center, ring } => (*center, point_in_polygon(ahead, ring)),
                        Geom::Point(p) => (*p, ahead.distance_to(*p) < 1000.0),
                        Geom::Line(_) => continue,
                    };
                    if !inside_ahead {
                        continue;
                    }
                    let radial = vector_to_heading(state.position_ft - center);
                    let (a, b) = (wrap_degrees_360(radial + 90.0), wrap_degrees_360(radial - 90.0));
                    let tangent = if wrap_degrees_180(a - desired_track).abs() < wrap_degrees_180(b - desired_track).abs() { a } else { b };
                    return Some(turn_toward(tangent, radial, 15.0));
                }
                _ => {}
            }
        }
        None
    }
}

/// The heading perpendicular to a line of `course` that points into the
/// `side_deg` side of it.
fn away_heading(course: f64, side_deg: f64) -> f64 {
    let right = wrap_degrees_360(course + 90.0);
    if wrap_degrees_180(side_deg - right).abs() <= 90.0 { right } else { wrap_degrees_360(course - 90.0) }
}

/// `from` turned by up to `deg` toward `to`.
fn turn_toward(from: f64, to: f64, deg: f64) -> f64 {
    let d = wrap_degrees_180(to - from);
    wrap_degrees_360(from + d.signum() * d.abs().min(deg))
}

/// Orient `line` the way it'll be flown and shift it to the side it's
/// flown on.
fn orient_and_offset(
    line: &Polyline,
    state: &AircraftState,
    direction_deg: Option<f64>,
    keep: Option<Dir>,
    side_deg: Option<f64>,
    offset_ft: f64,
) -> Polyline {
    let want = direction_deg.unwrap_or(state.track_deg);
    let mut pts = line.points.clone();
    let local = line
        .nearest(state.position_ft)
        .and_then(|n| line.segments().nth(n.segment))
        .map(|(a, b)| vector_to_heading(b - a))
        .unwrap_or_else(|| line.overall_course_deg());
    if wrap_degrees_180(local - want).abs() > 90.0 {
        pts.reverse();
    }
    let oriented = Polyline::new(pts);
    // Positive offset = right of the direction of travel.
    let signed = match (keep, side_deg) {
        (Some(Dir::Left), _) => offset_ft,
        (Some(Dir::Right), _) => -offset_ft,
        (None, Some(side)) => {
            let course = oriented.local_course_deg(state.position_ft, want).unwrap_or(want);
            if wrap_degrees_180(side - (course + 90.0)).abs() <= 90.0 { offset_ft } else { -offset_ft }
        }
        (None, None) => 0.0,
    };
    if signed == 0.0 {
        return oriented;
    }
    let n = oriented.points.len();
    let seg_right = |i: usize| {
        let d = (oriented.points[i + 1] - oriented.points[i]).normalized();
        Vec2::new(d.y, -d.x)
    };
    let shifted = (0..n)
        .map(|i| {
            let normal = match (i, n) {
                (_, 1) => Vec2::ZERO,
                (0, _) => seg_right(0),
                (i, n) if i == n - 1 => seg_right(n - 2),
                (i, _) => (seg_right(i - 1) + seg_right(i)).normalized(),
            };
            oriented.points[i] + normal * signed
        })
        .collect();
    Polyline::new(shifted)
}

/// L1 along the followed line, advancing segment by segment.
fn follow_guidance(l1: &L1PathFollower, state: &AircraftState, run: &mut StepRun, max_bank: f64) -> (f64, f64) {
    let Some(line) = &run.follow_line else { return (state.track_deg, 0.0) };
    let n = line.points.len();
    if n < 2 {
        return (state.track_deg, 0.0);
    }
    // Move on once abeam the end of the current segment (or closer to a
    // later one).
    loop {
        if run.follow_seg + 2 >= n {
            break;
        }
        let a = line.points[run.follow_seg];
        let b = line.points[run.follow_seg + 1];
        let along = (state.position_ft - a).dot((b - a).normalized());
        let len = a.distance_to(b);
        let lead = (state.gs_kt * KT_TO_FPS * 6.0).max(600.0);
        let next_a = b;
        let next_b = line.points[run.follow_seg + 2];
        let nearer_next = distance_to_segment(state.position_ft, next_a, next_b) < distance_to_segment(state.position_ft, a, b) - 200.0;
        if along > len - lead || nearer_next {
            run.follow_seg += 1;
        } else {
            break;
        }
    }
    let leg = StraightLeg { start_ft: line.points[run.follow_seg], end_ft: line.points[run.follow_seg + 1] };
    l1.follow_leg(state, leg, max_bank)
}

fn distance_to_segment(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    p.distance_to(closest_on_segment(p, a, b).1)
}

/// `alt` within altitude limits (lo, hi); the upper wins if they cross.
fn clamp_to_limits(alt: f64, (lo, hi): (f64, f64)) -> f64 {
    if lo > hi { hi } else { alt.max(lo).min(hi) }
}

fn apply_pattern_edit(run: &mut PatternRun, edit: RPattern) {
    let width_changed = edit.config.pattern.downwind_offset_ft != run.pf.config.pattern.downwind_offset_ft;
    let alt_changed = edit.config.pattern.altitude_agl_ft != run.pf.config.pattern.altitude_agl_ft;
    if width_changed || alt_changed {
        run.pf.config.pattern = edit.config.pattern.clone();
        run.pf.mode_manager.config.pattern = edit.config.pattern.clone();
        run.pf.safety_monitor = crate::core::safety_monitor::SafetyMonitor::new(run.pf.config.clone());
    }
    apply_extensions(&mut run.pf, &edit);
    run.pattern = edit;
    // Re-arm the pending turn with the new trigger.
    run.armed = None;
}

fn apply_extensions(pf: &mut PatternFlyProfile, rp: &RPattern) {
    let dw = match rp.extensions.get(&Leg::Downwind) {
        Some(Extension::Ft(ft)) => *ft,
        _ => pf.config.pattern.default_extension_ft,
    };
    let xw = match rp.extensions.get(&Leg::Crosswind) {
        Some(Extension::Ft(ft)) => *ft,
        _ => 0.0,
    };
    pf.set_leg_extensions(xw, dw);
    pf.square = rp.square();
}

/// Pick where to pick up the pattern from the aircraft's position.
fn entry_phase(pf: &PatternFlyProfile, entry: Option<Leg>, state: &AircraftState) -> FlightPhase {
    let frame = &pf.runway_frame;
    let rf = frame.to_runway_frame(state.position_ft);
    let course = frame.runway.course_deg;
    let recip = wrap_degrees_360(course + 180.0);
    let dw_y = pf.pattern.downwind_y_ft;
    let on_downwind = (rf.y - dw_y).abs() < 2000.0
        && wrap_degrees_180(state.track_deg - recip).abs() < 35.0
        && rf.x > pf.pattern.base_turn_x_ft;
    match entry {
        Some(Leg::Base) => FlightPhase::Base,
        Some(Leg::Final) => FlightPhase::Final,
        Some(Leg::Downwind) => {
            if on_downwind { FlightPhase::Downwind } else { FlightPhase::PatternEntry }
        }
        _ => {
            if state.on_ground {
                FlightPhase::TakeoffRoll
            } else if on_downwind {
                FlightPhase::Downwind
            } else if rf.y.abs() < 800.0 && rf.x < -1500.0 && wrap_degrees_180(state.track_deg - course).abs() < 30.0 {
                FlightPhase::Final
            } else if rf.y.abs() < 1500.0
                && rf.x > -500.0
                && wrap_degrees_180(state.track_deg - course).abs() < 30.0
                && state.alt_agl_ft < pf.config.pattern.altitude_agl_ft - 200.0
            {
                FlightPhase::InitialClimb
            } else {
                FlightPhase::PatternEntry
            }
        }
    }
}

fn start_pattern(rp: &RPattern, entry: Option<Leg>, land: LandKind, state: &AircraftState) -> PatternRun {
    let mut pf = PatternFlyProfile::new(rp.config.clone(), rp.frame.clone());
    apply_extensions(&mut pf, rp);
    pf.phase = entry_phase(&pf, entry, state);
    PatternRun { pf, pattern: rp.clone(), land, takeoff_only: false, armed: None, last_phase: None, leg_since_s: 0.0 }
}

impl PatternRun {
    /// Arm and evaluate the pending turn; set the embedded pattern's gates.
    fn drive(&mut self, ctx: &EvalCtx, state: &AircraftState) {
        let phase = self.pf.phase;
        if self.takeoff_only {
            self.pf.clearances.turn_crosswind = false;
            return;
        }
        if self.land == LandKind::LowApproach && phase == FlightPhase::Final {
            let rf = self.pf.runway_frame.to_runway_frame(state.position_ft);
            if state.alt_agl_ft < 150.0 && rf.x > -1500.0 {
                self.pf.low_approach_go_around();
            }
        }
        let want = next_turn(phase);
        if self.armed.as_ref().map(|(l, _)| Some(*l)) != Some(want) {
            self.armed = want.map(|l| (l, self.pattern.triggers.get(&l).cloned()));
        }
        if self.land == LandKind::TouchAndGo && phase == FlightPhase::Final {
            self.pf.touch_and_go_trigger = true;
        }
        let Some((leg, trig)) = &mut self.armed else { return };
        let leg = *leg;
        let open = matches!(self.pattern.extensions.get(&extended_leg_before(leg)), Some(Extension::Open(_)));
        // An upwind extension holds the crosswind turn until past the
        // departure end plus the extension.
        let upwind_ok = match (leg, self.pattern.extensions.get(&Leg::Upwind)) {
            (Leg::Crosswind, Some(Extension::Ft(ext))) => {
                let rf = self.pf.runway_frame.to_runway_frame(state.position_ft);
                rf.x >= self.pf.runway_frame.runway.length_ft + ext
            }
            _ => true,
        };
        let (when_fired, has_when, earliest_ok, latest_fired) = match trig {
            Some(t) => {
                let w = t.when.as_mut().map(|c| c.eval(ctx));
                let e = t.earliest.as_mut().map(|c| c.eval(ctx)).unwrap_or(true);
                let l = t.latest.as_mut().map(|c| c.eval(ctx)).unwrap_or(false);
                (w.unwrap_or(false), w.is_some(), e, l)
            }
            None => (false, false, true, false),
        };
        // No turn off the upwind below 400 ft AGL, whatever the trigger.
        let floor_ok = leg != Leg::Crosswind || state.alt_agl_ft >= MIN_TURN_AGL_FT;
        let earliest_ok = earliest_ok && upwind_ok && floor_ok;
        if ((when_fired && earliest_ok) || latest_fired) && floor_ok {
            if let Some(pl) = pattern_leg(leg) {
                self.pf.execute_turn(pl);
            }
        }
        set_turn_clearance(&mut self.pf, leg, !open && !has_when && earliest_ok);
    }
}

impl GuidanceProfile for PlanProfile {
    fn name(&self) -> &'static str {
        "plan"
    }

    fn owns(&self) -> AxisSet {
        AxisSet::ALL
    }

    fn contribute(&mut self, state: &AircraftState, dt: f64) -> ProfileTick {
        self.t_s += dt;
        let prev = self.prev_pos.unwrap_or(state.position_ft);
        self.prev_pos = Some(state.position_ft);
        let leg = self.current_leg();
        let leg_time = self.pattern.as_ref().map(|r| r.leg_since_s).unwrap_or(0.0);
        let gates = self.gates.clone();
        let frame = self.traffic_frame();
        let views = Arc::new(self.traffic.snapshot().iter().map(|t| traffic_view(&frame, t)).collect::<Vec<_>>());
        self.traffic_views = views.clone();
        let ctx = EvalCtx {
            position_ft: state.position_ft,
            prev_position_ft: prev,
            track_deg: state.track_deg,
            alt_msl_ft: state.alt_msl_ft,
            alt_agl_ft: state.alt_agl_ft,
            dt,
            leg,
            leg_time_s: leg_time,
            gates: &gates,
            traffic: &views,
        };
        self.update_constraints_and_reports(&ctx);

        // Step sequencing; instantaneous steps fall through in one tick.
        for _ in 0..16 {
            if self.idx >= self.steps.len() {
                break;
            }
            if self.run.is_none() {
                self.activate(state);
            }
            let until_fired = self.run.as_mut().and_then(|r| r.until.as_mut()).map(|u| u.eval(&ctx)).unwrap_or(false);
            let blocked = !until_fired && self.vertical_step_blocked(state);
            if until_fired || blocked || self.naturally_done(state) {
                let text = self.steps[self.idx].text.clone();
                if let Some(t) = self.run.as_ref().and_then(|r| r.target_track_deg) {
                    self.hold_track_deg = t;
                }
                match &self.steps[self.idx].kind {
                    RStepKind::Pattern { .. } => self.pattern = None,
                    RStepKind::Takeoff { pattern } => {
                        self.hold_track_deg = pattern.frame.runway.course_deg;
                        self.alt_target_ft = self.alt_target_ft.max(pattern.config.pattern_altitude_msl_ft());
                        self.speed_kt = self.base_config.performance.vy_kt;
                        self.pattern = None;
                    }
                    _ => {}
                }
                self.idx += 1;
                self.run = None;
                self.event(format!("done: {text}"));
                if self.idx == self.steps.len() {
                    self.event("plan complete; holding present track and altitude".into());
                }
                continue;
            }
            break;
        }

        let pattern_active = self.idx < self.steps.len()
            && matches!(self.steps[self.idx].kind, RStepKind::Pattern { .. } | RStepKind::Takeoff { .. })
            && self.pattern.is_some();
        if pattern_active {
            let limits = self.vertical_limits();
            let run = self.pattern.as_mut().unwrap();
            run.drive(&ctx, state);
            let before = run.pf.phase;
            let mut tick = run.pf.contribute(state, dt);
            let after = run.pf.phase;
            if run.last_phase != Some(after) {
                run.leg_since_s = 0.0;
                run.last_phase = Some(after);
            } else {
                run.leg_since_s += dt;
            }
            if before != FlightPhase::GoAround && after == FlightPhase::GoAround {
                let why = run.pf.last_go_around_reason.clone().unwrap_or_else(|| "unknown".into());
                self.events.push(format!("go-around ({why})"));
            }
            // A go-around's new circuit resets the pattern's extensions;
            // the plan's still apply.
            if before == FlightPhase::GoAround && after != FlightPhase::GoAround {
                let pattern = run.pattern.clone();
                apply_extensions(&mut run.pf, &pattern);
            }
            let c = &mut tick.contribution;
            if let Some(kt) = self.pattern_speed_kt {
                if matches!(after, FlightPhase::PatternEntry | FlightPhase::Downwind | FlightPhase::Base) {
                    c.target_speed_kt = Some(kt);
                }
            }
            if c.vertical_mode == Some(VerticalMode::Tecs) {
                if let Some(a) = c.target_altitude_ft {
                    c.target_altitude_ft = Some(clamp_to_limits(a, limits));
                }
            }
            // The plan's own altitude follows the pattern's, so a step that
            // takes over from it (a 360, a heading) holds where the pattern
            // was, not a target left over from before the takeoff.
            if let (Some(a), false) = (c.target_altitude_ft, state.on_ground) {
                self.alt_target_ft = a;
                self.alt_explicit = false;
            }
            if tick.hand_off.is_some() {
                self.events.push("touchdown; rollout has the aircraft".into());
            }
            return tick;
        }

        let max_bank = self.base_config.limits.max_bank_pattern_deg;
        ProfileTick::contribution_only(self.non_pattern_contribution(state, max_bank))
    }

    fn take_events(&mut self) -> Vec<String> {
        std::mem::take(&mut self.events)
    }

    fn status_json(&self) -> Option<Value> {
        Some(PlanProfile::status_json(self))
    }

    fn is_complete(&self) -> bool {
        self.is_finished()
    }

    fn pattern_metadata(&self) -> Option<PatternMetadata> {
        self.pattern.as_ref().and_then(|r| r.pf.pattern_metadata())
    }

    fn next_transition_hint(&self, state: &AircraftState) -> Option<TransitionHint> {
        self.pattern.as_ref().and_then(|r| r.pf.next_transition_hint(state))
    }

    fn spatial_overlay(&self) -> Option<SpatialOverlay> {
        self.pattern.as_ref().and_then(|r| r.pf.spatial_overlay())
    }

    fn landing_plan_mut(&mut self) -> Option<&mut dyn LandingPlanHolder> {
        self.pattern.as_mut().map(|r| &mut r.pf as &mut dyn LandingPlanHolder)
    }

    fn pattern_fly_mut(&mut self) -> Option<&mut PatternFlyProfile> {
        self.pattern.as_mut().map(|r| &mut r.pf)
    }

    fn mode_line_suffix(&self, _state: &AircraftState) -> Option<String> {
        if self.is_finished() {
            return Some("complete".into());
        }
        self.steps.get(self.idx).map(|s| format!("{}/{} {}", self.idx + 1, self.steps.len(), s.text))
    }

    fn debug_line(&self, state: &AircraftState) -> Option<String> {
        let mut parts = Vec::new();
        if let Some(t) = self.pattern.as_ref().and_then(|r| r.pf.takeoff_status(state)) {
            parts.push(t);
        }
        if let Some(r) = &self.pattern {
            if let Some((l, t)) = &r.armed {
                parts.push(format!(
                    "next turn {}: {}",
                    l.as_str(),
                    t.as_ref().map(|t| t.text.as_str()).unwrap_or("normal")
                ));
            }
        }
        let active: Vec<_> = self.constraints.iter().filter(|c| !c.ended && c.started).map(|c| c.id.as_str()).collect();
        if !active.is_empty() {
            parts.push(format!("constraints {}", active.join(",")));
        }
        (!parts.is_empty()).then(|| format!("plan: {}", parts.join(" · ")))
    }
}
