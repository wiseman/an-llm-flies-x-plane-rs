//! Scripted eval scenarios (`evals/plan/*.yaml`): an airborne or parked
//! start, ATC calls fired by time or by a plan condition, and checks
//! scored over the flown track — "turned base within 1000 ft of the
//! 118", "stayed at or below 1800 until north of the field", "landed on
//! 16R". Conditions and references use the plan IR, so a scenario says
//! where things happen in the same words a plan does.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::path::Path;

use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};

use crate::guidance::runway_geometry::RunwayFrame;
use crate::plan::cond::{distance_to, leg_for_phase, EvalCtx, RCond};
use crate::plan::gazetteer::{Gazetteer, PlaceKind};
use crate::plan::ir::{Cond, Ref};
use crate::plan::resolve::{Geom, Resolver};
use crate::types::{FlightPhase, Vec2, NM_TO_FT};

#[derive(Debug, Clone, Deserialize)]
pub struct Scenario {
    pub id: String,
    /// `research/pilot-api/instructions.yaml` items this scenario tests.
    #[serde(default)]
    pub items: Vec<String>,
    #[serde(default)]
    pub description: Option<String>,
    pub airport: String,
    pub start: Start,
    /// Operator message delivered at the start.
    #[serde(default)]
    pub briefing: Option<String>,
    #[serde(default)]
    pub messages: Vec<ScenarioMessage>,
    #[serde(default)]
    pub max_sim_s: Option<f64>,
    /// End the run (as completed) at this sim time instead of after
    /// landing, for scenarios that don't land.
    #[serde(default)]
    pub end_after_s: Option<f64>,
    #[serde(default)]
    pub checks: Vec<Check>,
    /// Other aircraft flying scripted paths.
    #[serde(default)]
    pub traffic: Vec<ScenarioTraffic>,
    /// Steady wind for the whole run.
    #[serde(default)]
    pub wind: Option<ScenarioWind>,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize)]
pub struct ScenarioWind {
    /// Direction the wind blows from, degrees true.
    pub from_deg: f64,
    pub kt: f64,
}

impl ScenarioWind {
    /// The air's motion, kt (x east, y north).
    pub fn vector_kt(&self) -> Vec2 {
        crate::types::heading_to_vector(self.from_deg + 180.0, self.kt)
    }
}

/// A scripted aircraft: flies `path` at `speed_kt` from `start_s`, sits
/// at the last point for `linger_s` (rolling out), then is gone. Points
/// are in `runway`'s frame (the scenario airport) unless given as `at`.
#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioTraffic {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default)]
    pub callsign: Option<String>,
    #[serde(default)]
    pub runway: Option<String>,
    pub path: Vec<TrafficPoint>,
    pub speed_kt: f64,
    #[serde(default)]
    pub start_s: f64,
    #[serde(default = "default_linger_s")]
    pub linger_s: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum TrafficPoint {
    At { at: Ref, alt_agl_ft: f64 },
    Runway { along_ft: f64, right_ft: f64, alt_agl_ft: f64 },
}

fn default_linger_s() -> f64 {
    30.0
}

/// Scripted traffic resolved to world-frame paths.
#[derive(Debug, Clone, Default)]
pub struct ScriptedTraffic {
    targets: Vec<ScriptedTarget>,
}

#[derive(Debug, Clone)]
struct ScriptedTarget {
    id: String,
    kind: String,
    callsign: Option<String>,
    /// (world position, altitude MSL)
    points: Vec<(Vec2, f64)>,
    speed_fps: f64,
    start_s: f64,
    linger_s: f64,
    field_elevation_ft: f64,
}

impl ScriptedTraffic {
    pub fn new(
        specs: &[ScenarioTraffic],
        airport: &str,
        field_elevation_ft: f64,
        resolver: &mut Resolver,
    ) -> Result<Self> {
        let mut targets = Vec::new();
        for t in specs {
            let mut points = Vec::new();
            for p in &t.path {
                let (r, alt) = match p {
                    TrafficPoint::At { at, alt_agl_ft } => (at.clone(), *alt_agl_ft),
                    TrafficPoint::Runway { along_ft, right_ft, alt_agl_ft } => {
                        let runway = t.runway.clone().ok_or_else(|| anyhow!("traffic {}: runway-frame points need runway:", t.id))?;
                        (
                            Ref::RunwayFrame { airport: airport.to_string(), runway, along_ft: *along_ft, right_ft: *right_ft },
                            *alt_agl_ft,
                        )
                    }
                };
                let f = resolver.resolve(&r).with_context(|| format!("traffic {}", t.id))?;
                points.push((f.geom.anchor(), field_elevation_ft + alt));
            }
            if points.is_empty() {
                return Err(anyhow!("traffic {} has no path", t.id));
            }
            targets.push(ScriptedTarget {
                id: t.id.clone(),
                kind: t.kind.clone(),
                callsign: t.callsign.clone(),
                points,
                speed_fps: t.speed_kt * crate::types::KT_TO_FPS,
                start_s: t.start_s,
                linger_s: t.linger_s,
                field_elevation_ft,
            });
        }
        Ok(Self { targets })
    }


    /// Where everyone is at sim time `t`.
    pub fn at(&self, t: f64) -> Vec<crate::core::traffic::TrafficTarget> {
        self.targets.iter().filter_map(|s| s.at(t)).collect()
    }
}

impl ScriptedTarget {
    fn at(&self, t: f64) -> Option<crate::core::traffic::TrafficTarget> {
        if t < self.start_s {
            return None;
        }
        let mut left = (t - self.start_s) * self.speed_fps;
        let mut pos = self.points[0];
        let mut track = 0.0;
        let mut moving = false;
        for w in self.points.windows(2) {
            let (a, b) = (w[0], w[1]);
            let len = a.0.distance_to(b.0);
            track = crate::types::vector_to_heading(b.0 - a.0);
            if left <= len {
                let f = if len > 0.0 { left / len } else { 1.0 };
                pos = (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
                moving = true;
                break;
            }
            left -= len;
            pos = b;
        }
        if !moving {
            // At the end: sitting there for linger_s, then gone.
            if left / self.speed_fps.max(1.0) > self.linger_s {
                return None;
            }
        }
        let on_ground = pos.1 - self.field_elevation_ft < 5.0;
        Some(crate::core::traffic::TrafficTarget {
            id: self.id.clone(),
            kind: self.kind.clone(),
            callsign: self.callsign.clone(),
            position_ft: pos.0,
            alt_msl_ft: pos.1,
            track_deg: track,
            gs_kt: if moving { self.speed_fps / crate::types::KT_TO_FPS } else { 0.0 },
            on_ground,
        })
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum Start {
    Airborne { airborne: AirborneStart },
    /// On the runway, lined up and stopped, engine running.
    Runway { runway: RunwayStart },
    Parked { parking: String },
}

#[derive(Debug, Clone, Deserialize)]
pub struct RunwayStart {
    pub runway: String,
    #[serde(default)]
    pub along_ft: f64,
}

impl Start {
    /// The runway the start is placed against, if any.
    pub fn runway(&self) -> Option<&str> {
        match self {
            Start::Airborne { airborne } => Some(&airborne.runway),
            Start::Runway { runway } => Some(&runway.runway),
            Start::Parked { .. } => None,
        }
    }
}

/// A start in the air, placed in a runway's frame: `along_ft` down the
/// runway from its approach end (negative = before it), `right_ft` to
/// the right of its course.
#[derive(Debug, Clone, Deserialize)]
pub struct AirborneStart {
    pub runway: String,
    pub along_ft: f64,
    pub right_ft: f64,
    #[serde(default)]
    pub alt_msl_ft: Option<f64>,
    #[serde(default)]
    pub alt_agl_ft: Option<f64>,
    pub heading_deg: f64,
    pub ias_kt: f64,
}

/// One scripted transmission. Fires at `at_s` sim seconds, or when
/// `when` first holds (then after `delay_s`).
#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioMessage {
    #[serde(default)]
    pub at_s: Option<f64>,
    #[serde(default)]
    pub when: Option<Cond>,
    #[serde(default)]
    pub delay_s: f64,
    #[serde(default)]
    pub atc: Option<String>,
    #[serde(default)]
    pub operator: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "check", rename_all = "snake_case")]
pub enum Check {
    /// The first turn onto `leg` (entering that phase from the one before
    /// it) happened within `tol_ft` of `near`.
    Turn { leg: String, near: Ref, #[serde(default = "default_turn_tol")] tol_ft: f64 },
    /// The first turn onto `leg` came after message `message` (0-based)
    /// was sent, and within `within_s` of it.
    TurnAfterMessage { leg: String, message: usize, #[serde(default = "default_within_s")] within_s: f64 },
    /// `holds` (a state condition) held at every sample from when `from`
    /// first holds (default: the first airborne sample) until `until` (if
    /// given) happened. Altitude bounds get the ACS ±100 ft.
    Remain {
        holds: Cond,
        #[serde(default)]
        from: Option<Cond>,
        #[serde(default)]
        until: Option<Cond>,
        #[serde(default)]
        label: Option<String>,
    },
    /// Ground track stayed within `tol_deg` of `heading_deg` from the
    /// start until `until` happened (which must happen).
    HeadingHeld { heading_deg: f64, #[serde(default = "default_heading_tol")] tol_deg: f64, until: Cond },
    /// `cond` happened at some point.
    Reached { cond: Cond, #[serde(default)] label: Option<String> },
    /// Touched down on `runway`, past its threshold and within its first
    /// third, near the centerline.
    Landed { runway: String },
    /// Touched down and took off again (a touch-and-go).
    TouchAndGo,
    /// Turned through `min_deg` (default 330) in one direction within
    /// four minutes: a 360, or circling.
    Orbit { #[serde(default)] dir: Option<crate::plan::ir::Dir>, #[serde(default = "default_orbit_deg")] min_deg: f64 },
    /// Stayed within `radius_ft` of `near` for `min_s` in a row.
    Loiter { near: Ref, radius_ft: f64, min_s: f64 },
    /// After liftoff, never banked more than `max_bank_deg` below
    /// `agl_ft` while climbing (AFH: no turns below 400 ft).
    NoLowTurns { #[serde(default = "default_low_turn_agl")] agl_ft: f64, #[serde(default = "default_low_turn_bank")] max_bank_deg: f64 },
    NoGoAround,
    NoCrash,
    /// A forward slip on final to `runway`: at least `min_s` in a row
    /// banked more than 8° with the track within 10° of the runway's
    /// course, above 150 ft AGL.
    ForwardSlip { runway: String, #[serde(default = "default_slip_s")] min_s: f64 },
    /// A rectangular course around `around` (the field as
    /// `rectangular_course` sizes it): from settling beside it, at least
    /// `min_laps` laps (angle swept around the field) holding a constant
    /// distance from its edges (±`tol_ft` beside the sides; the corners
    /// round off), `offset_ft` if given, else whatever distance was flown,
    /// and within `alt_tol_ft` of the median altitude (ACS: ±100 ft).
    RectangularCourse {
        around: Ref,
        #[serde(default)]
        offset_ft: Option<f64>,
        #[serde(default)]
        size_ft: Option<f64>,
        #[serde(default = "default_rect_tol_ft")]
        tol_ft: f64,
        #[serde(default = "default_rect_alt_tol_ft")]
        alt_tol_ft: f64,
        #[serde(default = "default_rect_laps")]
        min_laps: f64,
    },
    /// A square base to `runway`: the last base leg flown wings level on
    /// a course perpendicular to the runway (±10°) for at least
    /// `min_straight_s`, with the turns onto it from downwind and off it
    /// onto final each done within `max_turn_s`, rolling out within
    /// 300 ft of the centerline.
    SquareBase {
        runway: String,
        #[serde(default = "default_square_straight_s")]
        min_straight_s: f64,
        #[serde(default = "default_square_turn_s")]
        max_turn_s: f64,
    },
    /// Never closer than `min_ft` horizontally to airborne scripted
    /// traffic while within `vertical_ft` of its altitude.
    Separation {
        #[serde(default = "default_sep_ft")]
        min_ft: f64,
        #[serde(default = "default_sep_vertical_ft")]
        vertical_ft: f64,
    },
    /// The pilot said something on the radio containing one of `any`
    /// (case-insensitive), e.g. ["in sight", "have the traffic"], and
    /// none of `none_of` (default: "will report", "looking", "negative").
    /// With `in_sight_of` (a traffic id), only calls made after that
    /// traffic first came within visual range count.
    Radio {
        any: Vec<String>,
        #[serde(default = "default_radio_exclusions")]
        none_of: Vec<String>,
        #[serde(default)]
        in_sight_of: Option<String>,
        #[serde(default)]
        label: Option<String>,
    },
}

fn default_slip_s() -> f64 {
    5.0
}

fn default_rect_tol_ft() -> f64 {
    300.0
}

fn default_rect_alt_tol_ft() -> f64 {
    100.0
}

fn default_rect_laps() -> f64 {
    1.0
}

fn default_square_straight_s() -> f64 {
    8.0
}

fn default_square_turn_s() -> f64 {
    20.0
}

fn default_radio_exclusions() -> Vec<String> {
    vec!["will report".into(), "looking".into(), "negative".into()]
}

fn default_sep_ft() -> f64 {
    crate::core::traffic::SEPARATION_FT
}

fn default_sep_vertical_ft() -> f64 {
    crate::core::traffic::SEPARATION_VERTICAL_FT
}

/// What a run recorded besides the track, for checks that need it.
#[derive(Debug, Clone, Default)]
pub struct ScoreExtras<'a> {
    pub traffic: Option<&'a ScriptedTraffic>,
    /// The pilot's transmissions: (sim time, text).
    pub radio: &'a [(f64, String)],
}

fn default_turn_tol() -> f64 {
    1000.0
}

fn default_within_s() -> f64 {
    10.0
}

fn default_heading_tol() -> f64 {
    10.0
}

fn default_orbit_deg() -> f64 {
    330.0
}

fn default_low_turn_agl() -> f64 {
    350.0
}

fn default_low_turn_bank() -> f64 {
    10.0
}

pub fn load(path: &Path) -> Result<Scenario> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    serde_yaml_ng::from_str(&text).with_context(|| format!("parsing {}", path.display()))
}

/// One recorded tick.
#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub t: f64,
    pub pos: Vec2,
    pub alt_msl_ft: f64,
    pub alt_agl_ft: f64,
    pub track_deg: f64,
    pub phase: Option<FlightPhase>,
    pub on_ground: bool,
    pub crashed: bool,
    pub bank_deg: f64,
    pub vs_fpm: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct CheckResult {
    pub check: String,
    pub pass: bool,
    pub detail: String,
}

/// Scripted messages waiting to fire, with their conditions resolved.
pub struct MessageSchedule {
    pending: Vec<(usize, ScenarioMessage, Option<RCond>, Option<f64>)>,
    pub sent_at: Vec<Option<f64>>,
    prev_pos: Option<Vec2>,
}

impl MessageSchedule {
    pub fn new(messages: &[ScenarioMessage], resolver: &mut Resolver) -> Result<Self> {
        let mut pending = Vec::new();
        for (i, m) in messages.iter().enumerate() {
            let cond = m.when.as_ref().map(|c| RCond::resolve(c, resolver)).transpose()?;
            if m.at_s.is_none() && cond.is_none() {
                return Err(anyhow!("message {i} needs at_s or when"));
            }
            pending.push((i, m.clone(), cond, None));
        }
        Ok(Self { sent_at: vec![None; messages.len()], pending, prev_pos: None })
    }

    /// Messages due this tick, in order.
    pub fn due(&mut self, s: &Sample, dt: f64) -> Vec<(usize, ScenarioMessage)> {
        let prev = self.prev_pos.unwrap_or(s.pos);
        self.prev_pos = Some(s.pos);
        let no_gates = HashSet::new();
        let ctx = s.ctx(prev, dt, &no_gates);
        let mut out = Vec::new();
        for (i, m, cond, fired_at) in self.pending.iter_mut() {
            if fired_at.is_none() {
                let hit = match (m.at_s, cond.as_mut()) {
                    (Some(at), None) => s.t >= at,
                    (None, Some(c)) => c.eval(&ctx),
                    (Some(at), Some(c)) => s.t >= at && c.eval(&ctx),
                    (None, None) => false,
                };
                if hit {
                    *fired_at = Some(s.t);
                }
            }
            if let Some(f) = fired_at {
                if s.t >= *f + m.delay_s && self.sent_at[*i].is_none() {
                    self.sent_at[*i] = Some(s.t);
                    out.push((*i, m.clone()));
                }
            }
        }
        out
    }
}

fn phase_named(leg: &str) -> Result<FlightPhase> {
    FlightPhase::from_str(leg).ok_or_else(|| anyhow!("unknown leg/phase {leg:?}"))
}

/// The phase a turn onto `to` comes from.
fn turn_from(to: FlightPhase) -> Option<FlightPhase> {
    match to {
        FlightPhase::Crosswind => Some(FlightPhase::InitialClimb),
        FlightPhase::Downwind => Some(FlightPhase::Crosswind),
        FlightPhase::Base => Some(FlightPhase::Downwind),
        FlightPhase::Final => Some(FlightPhase::Base),
        _ => None,
    }
}

fn first_turn(samples: &[Sample], to: FlightPhase) -> Option<Sample> {
    let from = turn_from(to);
    samples
        .windows(2)
        .find(|w| w[1].phase == Some(to) && w[0].phase != Some(to) && (from.is_none() || w[0].phase == from))
        .map(|w| w[1])
}

/// Maximal runs of consecutive items where `pred` holds, as inclusive
/// (first, last) index pairs.
fn spans<T>(items: &[T], mut pred: impl FnMut(&T) -> bool) -> Vec<(usize, usize)> {
    let mut out: Vec<(usize, usize)> = Vec::new();
    let mut start = None;
    for (i, x) in items.iter().enumerate() {
        if pred(x) {
            let st = *start.get_or_insert(i);
            match out.last_mut() {
                Some(last) if last.0 == st => last.1 = i,
                _ => out.push((st, i)),
            }
        } else {
            start = None;
        }
    }
    out
}

/// The longest span of `samples` where `pred` holds: (seconds, end time).
fn longest_span(samples: &[Sample], pred: impl FnMut(&Sample) -> bool) -> (f64, f64) {
    spans(samples, pred)
        .into_iter()
        .map(|(a, b)| (samples[b].t - samples[a].t, samples[b].t))
        .fold((0.0, 0.0), |best, x| if x.0 > best.0 { x } else { best })
}

impl Sample {
    /// The condition-evaluation context at this sample.
    fn ctx<'a>(&self, prev_pos: Vec2, dt: f64, gates: &'a HashSet<String>) -> EvalCtx<'a> {
        EvalCtx {
            position_ft: self.pos,
            prev_position_ft: prev_pos,
            track_deg: self.track_deg,
            alt_msl_ft: self.alt_msl_ft,
            alt_agl_ft: self.alt_agl_ft,
            dt,
            leg: self.phase.and_then(leg_for_phase),
            leg_time_s: 0.0,
            gates,
            traffic: &[],
        }
    }
}

fn eval_over<'a>(
    samples: &'a [Sample],
    mut each: impl FnMut(&EvalCtx, &'a Sample) -> bool,
    dt: f64,
) -> Option<&'a Sample> {
    let gates = HashSet::new();
    let mut prev = samples.first().map(|s| s.pos).unwrap_or(Vec2::ZERO);
    for s in samples {
        let ctx = s.ctx(prev, dt, &gates);
        prev = s.pos;
        if each(&ctx, s) {
            return Some(s);
        }
    }
    None
}

pub fn score(
    checks: &[Check],
    samples: &[Sample],
    sent_at: &[Option<f64>],
    resolver: &mut Resolver,
    runway_frame_for: &mut dyn FnMut(&str) -> Result<RunwayFrame>,
    dt: f64,
    extras: &ScoreExtras,
) -> Vec<CheckResult> {
    checks
        .iter()
        .map(|c| match score_one(c, samples, sent_at, resolver, runway_frame_for, dt, extras) {
            Ok(r) => r,
            Err(e) => CheckResult { check: format!("{c:?}"), pass: false, detail: format!("check error: {e:#}") },
        })
        .collect()
}

fn score_one(
    c: &Check,
    samples: &[Sample],
    sent_at: &[Option<f64>],
    resolver: &mut Resolver,
    runway_frame_for: &mut dyn FnMut(&str) -> Result<RunwayFrame>,
    dt: f64,
    extras: &ScoreExtras,
) -> Result<CheckResult> {
    Ok(match c {
        Check::ForwardSlip { runway, min_s } => {
            let course = runway_frame_for(runway)?.runway.course_deg;
            let (best, at) = longest_span(samples, |s| {
                !s.on_ground
                    && s.alt_agl_ft > 150.0
                    && s.bank_deg.abs() > 8.0
                    && crate::types::wrap_degrees_180(s.track_deg - course).abs() < 10.0
            });
            CheckResult {
                check: format!("forward slip on final {runway}"),
                pass: best >= *min_s,
                detail: format!("longest {best:.0} s (ending t={at:.0}s)"),
            }
        }
        Check::RectangularCourse { around, offset_ft, size_ft, tol_ft, alt_tol_ft, min_laps } => {
            let f = resolver.resolve(around)?;
            let name = format!("rectangular course around {}", f.label);
            let rect = crate::plan::compile::course_rect(&f, *size_ft);
            let (u, v) = (rect.u, rect.v());
            // Distance outside the field's edge, for points beside a side
            // (corners, where the track rounds off, don't count).
            let side_dist = |p: Vec2| {
                let d = p - rect.center;
                let (pu, pv) = (d.dot(u).abs(), d.dot(v).abs());
                if pu <= rect.half_u && pv > rect.half_v {
                    Some(pv - rect.half_v)
                } else if pv <= rect.half_v && pu > rect.half_u {
                    Some(pu - rect.half_u)
                } else {
                    None
                }
            };
            let air: Vec<&Sample> = samples.iter().filter(|s| !s.on_ground).collect();
            // The distance flown out from the edges: the median beside the
            // field within a mile of it, or the one asked for.
            let beside: Vec<(&Sample, f64)> =
                air.iter().filter_map(|s| side_dist(s.pos).map(|d| (*s, d))).filter(|(_, d)| *d < 6076.0).collect();
            if beside.is_empty() {
                return Ok(CheckResult { check: name, pass: false, detail: "never flew beside the field".into() });
            }
            let median_of = |mut v: Vec<f64>| {
                v.sort_by(|a, b| a.partial_cmp(b).unwrap());
                v[v.len() / 2]
            };
            let out = offset_ft.unwrap_or_else(|| median_of(beside.iter().map(|(_, d)| *d).collect()));
            let course_alt = median_of(beside.iter().map(|(s, _)| s.alt_msl_ft).collect());
            // Scored from settling (15 s at that distance and the course
            // altitude), not the intercept or the descent to it.
            let settled = |s: &Sample| {
                side_dist(s.pos).is_some_and(|d| (d - out).abs() <= 200.0) && (s.alt_msl_ft - course_alt).abs() <= *alt_tol_ft
            };
            let first = spans(&air, |s| settled(s)).into_iter().find(|(a, b)| air[*b].t - air[*a].t >= 15.0).map(|(a, _)| a);
            let Some(first) = first else {
                return Ok(CheckResult {
                    check: name,
                    pass: false,
                    detail: format!("never settled {out:.0} ft out at {course_alt:.0}"),
                });
            };
            let bearing = |p: Vec2| crate::types::vector_to_heading(p - rect.center);
            let (mut swept, mut prev) = (0.0_f64, bearing(air[first].pos));
            let (mut worst, mut worst_t) = (0.0_f64, air[first].t);
            let mut alts = Vec::new();
            let mut end_t = None;
            for s in &air[first..] {
                let b = bearing(s.pos);
                swept += crate::types::wrap_degrees_180(b - prev);
                prev = b;
                alts.push(s.alt_msl_ft);
                if let Some(d) = side_dist(s.pos) {
                    if (d - out).abs() > worst {
                        worst = (d - out).abs();
                        worst_t = s.t;
                    }
                }
                if swept.abs() >= *min_laps * 360.0 {
                    end_t = Some(s.t);
                    break;
                }
            }
            alts.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let median = alts[alts.len() / 2];
            let alt_dev = (alts[alts.len() - 1] - median).max(median - alts[0]);
            CheckResult {
                check: name,
                pass: end_t.is_some() && worst <= *tol_ft && alt_dev <= *alt_tol_ft,
                detail: format!(
                    "{:.1} laps from t={:.0}s{}, {:.2} nm out from the edges ±{worst:.0} ft (worst t={worst_t:.0}s), altitude {median:.0} ±{alt_dev:.0}",
                    swept.abs() / 360.0,
                    air[first].t,
                    end_t.map(|t| format!(" to t={t:.0}s")).unwrap_or_default(),
                    out / NM_TO_FT
                ),
            }
        }
        Check::SquareBase { runway, min_straight_s, max_turn_s } => {
            use crate::types::wrap_degrees_180 as w180;
            let frame = runway_frame_for(runway)?;
            let course = frame.runway.course_deg;
            let air: Vec<&Sample> = samples.iter().filter(|s| !s.on_ground).collect();
            let near = |s: &Sample, c: f64| w180(s.track_deg - c).abs() <= 10.0;
            let time_near = |c: f64| air.windows(2).filter(|w| near(w[0], c)).map(|w| w[1].t - w[0].t).sum::<f64>();
            let base_course = if time_near(course + 90.0) >= time_near(course - 90.0) { course + 90.0 } else { course - 90.0 };
            // The last wings-level run on the base course.
            let run = spans(&air, |s| near(s, base_course) && s.bank_deg.abs() < 5.0).pop();
            let name = format!("square base to {runway}");
            let Some((a, b)) = run else {
                return Ok(CheckResult { check: name, pass: false, detail: "never flew wings level on a base course".into() });
            };
            let straight_s = air[b].t - air[a].t;
            let into = air[..a].iter().rev().find(|s| near(s, course + 180.0)).map(|s| air[a].t - s.t);
            let out = air[b..].iter().find(|s| near(s, course));
            let out_s = out.map(|s| s.t - air[b].t);
            let rolled_out_y = out.map(|s| frame.to_runway_frame(s.pos).y.abs());
            let fmt = |v: Option<f64>| v.map(|v| format!("{v:.0}")).unwrap_or_else(|| "-".into());
            let pass = straight_s >= *min_straight_s
                && into.is_some_and(|v| v <= *max_turn_s)
                && out_s.is_some_and(|v| v <= *max_turn_s)
                && rolled_out_y.is_some_and(|y| y <= 300.0);
            CheckResult {
                check: name,
                pass,
                detail: format!(
                    "turn onto base {} s, base {straight_s:.0} s wings level, turn to final {} s, rolled out {} ft off centerline",
                    fmt(into),
                    fmt(out_s),
                    fmt(rolled_out_y)
                ),
            }
        }
        Check::Separation { min_ft, vertical_ft } => {
            let name = format!("separation from traffic ({:.1} nm / {vertical_ft:.0} ft)", min_ft / NM_TO_FT);
            let mut worst: Option<(f64, f64, String)> = None;
            if let Some(tr) = extras.traffic {
                for s in samples.iter().filter(|s| !s.on_ground) {
                    for t in tr.at(s.t).iter().filter(|t| !t.on_ground) {
                        if (t.alt_msl_ft - s.alt_msl_ft).abs() >= *vertical_ft {
                            continue;
                        }
                        let d = t.position_ft.distance_to(s.pos);
                        if worst.as_ref().is_none_or(|w| d < w.1) {
                            worst = Some((s.t, d, t.id.clone()));
                        }
                    }
                }
            }
            match worst {
                Some((t, d, id)) => CheckResult {
                    check: name,
                    pass: d >= *min_ft,
                    detail: format!("closest {d:.0} ft from {id} at t={t:.0}s"),
                },
                None => CheckResult { check: name, pass: true, detail: "never within the vertical band".into() },
            }
        }
        Check::Radio { any, none_of, in_sight_of, label } => {
            let name = label.clone().unwrap_or_else(|| format!("said {}", any.join(" / ")));
            let from_t = match (in_sight_of, extras.traffic) {
                (Some(id), Some(tr)) => samples
                    .iter()
                    .find(|s| {
                        tr.at(s.t).iter().any(|t| {
                            &t.id == id && t.position_ft.distance_to(s.pos) <= crate::core::traffic::VISUAL_RANGE_FT
                        })
                    })
                    .map(|s| s.t)
                    .unwrap_or(f64::INFINITY),
                _ => f64::NEG_INFINITY,
            };
            let hit = extras.radio.iter().find(|(t, text)| {
                let l = text.to_lowercase();
                *t >= from_t - 1.0
                    && any.iter().any(|a| l.contains(&a.to_lowercase()))
                    && !none_of.iter().any(|a| l.contains(&a.to_lowercase()))
            });
            match hit {
                Some((t, text)) => CheckResult { check: name, pass: true, detail: format!("t={t:.0}s: {text}") },
                None => CheckResult { check: name, pass: false, detail: format!("{} calls, none matching", extras.radio.len()) },
            }
        }
        Check::Turn { leg, near, tol_ft } => {
            let to = phase_named(leg)?;
            let f = resolver.resolve(near)?;
            let name = format!("turn {leg} within {tol_ft:.0} ft of {}", f.label);
            match first_turn(samples, to) {
                Some(s) => {
                    let d = distance_to(&f.geom, s.pos);
                    CheckResult { check: name, pass: d <= *tol_ft, detail: format!("turned at t={:.0}s, {d:.0} ft away", s.t) }
                }
                None => CheckResult { check: name, pass: false, detail: format!("never turned {leg}") },
            }
        }
        Check::TurnAfterMessage { leg, message, within_s } => {
            let to = phase_named(leg)?;
            let name = format!("turn {leg} within {within_s:.0} s after message {message}");
            let sent = sent_at.get(*message).copied().flatten();
            match (first_turn(samples, to), sent) {
                (Some(s), Some(m)) => CheckResult {
                    check: name,
                    pass: s.t >= m && s.t - m <= *within_s,
                    detail: format!("message at {m:.0}s, turned at {:.0}s", s.t),
                },
                (None, _) => CheckResult { check: name, pass: false, detail: format!("never turned {leg}") },
                (Some(s), None) => CheckResult {
                    check: name,
                    pass: false,
                    detail: format!("turned at {:.0}s but the message was never sent", s.t),
                },
            }
        }
        Check::Remain { holds, from, until, label } => {
            let mut h = RCond::resolve(holds, resolver)?;
            let mut f = from.as_ref().map(|f| RCond::resolve(f, resolver)).transpose()?;
            let mut started = f.is_none();
            let mut u = until.as_ref().map(|u| RCond::resolve(u, resolver)).transpose()?;
            let name = label.clone().unwrap_or_else(|| {
                format!(
                    "remain {}{}",
                    crate::plan::compile::cond_text(holds),
                    until.as_ref().map(|u| format!(" until {}", crate::plan::compile::cond_text(u))).unwrap_or_default()
                )
            });
            let airborne: Vec<Sample> = samples.iter().copied().filter(|s| !s.on_ground).collect();
            let mut ended_at = None;
            let mut worst: Option<(f64, f64)> = None;
            let violated = eval_over(
                &airborne,
                |ctx, s| {
                    if let Some(u) = u.as_mut() {
                        if u.eval(ctx) {
                            ended_at.get_or_insert(s.t);
                        }
                    }
                    if ended_at.is_some() {
                        return false;
                    }
                    if !started {
                        started = f.as_mut().is_some_and(|f| f.eval(ctx));
                        if !started {
                            return false;
                        }
                    }
                    let ok = h.holds_with_alt_tolerance(ctx, crate::core::plan_profile::ALTITUDE_TOLERANCE_FT);
                    if !ok && worst.is_none() {
                        worst = Some((s.t, s.alt_msl_ft));
                    }
                    !ok
                },
                dt,
            );
            match violated {
                Some(s) => CheckResult {
                    check: name,
                    pass: false,
                    detail: format!("broken at t={:.0}s (alt {:.0})", s.t, s.alt_msl_ft),
                },
                None => CheckResult {
                    check: name,
                    pass: true,
                    detail: match (until, ended_at) {
                        (Some(_), Some(t)) => format!("held until it ended at t={t:.0}s"),
                        (Some(_), None) => "held; the until never happened".into(),
                        _ => "held throughout".into(),
                    },
                },
            }
        }
        Check::HeadingHeld { heading_deg, tol_deg, until } => {
            let mut u = RCond::resolve(until, resolver)?;
            let name = format!(
                "track {heading_deg:03.0}±{tol_deg:.0} until {}",
                crate::plan::compile::cond_text(until)
            );
            let mut worst = 0.0_f64;
            // Skip the first few seconds: the handover may start mid-turn.
            let ended = eval_over(
                samples,
                |ctx, s| {
                    if u.eval(ctx) {
                        return true;
                    }
                    if s.t > 5.0 {
                        worst = worst.max(crate::types::wrap_degrees_180(s.track_deg - heading_deg).abs());
                    }
                    false
                },
                dt,
            );
            match ended {
                Some(s) => CheckResult {
                    check: name,
                    pass: worst <= *tol_deg,
                    detail: format!("until t={:.0}s; worst deviation {worst:.0}°", s.t),
                },
                None => CheckResult { check: name, pass: false, detail: "the until never happened".into() },
            }
        }
        Check::Reached { cond, label } => {
            let mut rc = RCond::resolve(cond, resolver)?;
            let name = label.clone().unwrap_or_else(|| format!("reached: {}", crate::plan::compile::cond_text(cond)));
            match eval_over(samples, |ctx, _| rc.eval(ctx), dt) {
                Some(s) => CheckResult { check: name, pass: true, detail: format!("at t={:.0}s", s.t) },
                None => CheckResult { check: name, pass: false, detail: "never".into() },
            }
        }
        Check::Landed { runway } => {
            let frame = runway_frame_for(runway)?;
            let name = format!("landed on {runway}");
            let td = samples.windows(2).find(|w| !w[0].on_ground && w[1].on_ground).map(|w| w[1]);
            match td {
                Some(s) => {
                    let rf = frame.to_runway_frame(s.pos);
                    let thr = frame.runway.displaced_threshold_ft;
                    CheckResult {
                        check: name,
                        pass: frame.touchdown_in_zone(rf),
                        detail: format!("touched down {:.0} ft past the threshold, {:.0} ft off centerline", rf.x - thr, rf.y),
                    }
                }
                None => CheckResult { check: name, pass: false, detail: "no touchdown".into() },
            }
        }
        Check::TouchAndGo => {
            let td = samples.windows(2).position(|w| !w[0].on_ground && w[1].on_ground);
            let again = td.and_then(|i| samples[i..].iter().find(|s| !s.on_ground && s.alt_agl_ft > 300.0));
            CheckResult {
                check: "touch and go".into(),
                pass: again.is_some(),
                detail: match (td, again) {
                    (Some(i), Some(s)) => format!("touched down t={:.0}s, back up t={:.0}s", samples[i + 1].t, s.t),
                    (Some(i), None) => format!("touched down t={:.0}s and stayed down", samples[i + 1].t),
                    _ => "no touchdown".into(),
                },
            }
        }
        Check::Orbit { dir, min_deg } => {
            // 1 Hz signed track changes, right turns positive.
            let mut turns: Vec<(f64, f64)> = Vec::new();
            let mut last: Option<&Sample> = None;
            for s in samples.iter().filter(|s| !s.on_ground) {
                if let Some(l) = last {
                    if s.t - l.t < 1.0 {
                        continue;
                    }
                    turns.push((s.t, crate::types::wrap_degrees_180(s.track_deg - l.track_deg)));
                }
                last = Some(s);
            }
            let sign = match dir {
                Some(crate::plan::ir::Dir::Left) => -1.0,
                _ => 1.0,
            };
            let mut best = 0.0_f64;
            let mut best_at = 0.0;
            // One continuous turn: a run ends after 10 s of straight
            // flight, so the pattern's base and final turns don't add to
            // a 360 flown on downwind.
            for i in 0..turns.len() {
                let mut sum = 0.0;
                let mut straight_s = 0.0;
                for (t, d) in &turns[i..] {
                    if *t - turns[i].0 > 240.0 {
                        break;
                    }
                    straight_s = if d.abs() < 1.0 { straight_s + 1.0 } else { 0.0 };
                    if straight_s >= 10.0 {
                        break;
                    }
                    sum += d;
                    let v = if dir.is_some() { sign * sum } else { sum.abs() };
                    if v > best {
                        best = v;
                        best_at = turns[i].0;
                    }
                }
            }
            let name = format!("{} {min_deg:.0}° turn", match dir {
                Some(crate::plan::ir::Dir::Left) => "left",
                Some(crate::plan::ir::Dir::Right) => "right",
                None => "a",
            });
            CheckResult { check: name, pass: best >= *min_deg, detail: format!("{best:.0}° from t={best_at:.0}s") }
        }
        Check::Loiter { near, radius_ft, min_s } => {
            let f = resolver.resolve(near)?;
            let center = f.geom.anchor();
            let (best, _) = longest_span(samples, |s| s.pos.distance_to(center) <= *radius_ft);
            CheckResult {
                check: format!("stayed within {:.1} nm of {} for {min_s:.0} s", radius_ft / NM_TO_FT, f.label),
                pass: best >= *min_s,
                detail: format!("longest {best:.0} s"),
            }
        }
        Check::NoLowTurns { agl_ft, max_bank_deg } => {
            let liftoff = samples.windows(2).position(|w| w[0].on_ground && !w[1].on_ground).unwrap_or(0);
            let bad = samples[liftoff..]
                .iter()
                .find(|s| !s.on_ground && s.alt_agl_ft < *agl_ft && s.vs_fpm > 100.0 && s.bank_deg.abs() > *max_bank_deg);
            CheckResult {
                check: format!("no turns below {agl_ft:.0} ft AGL after takeoff"),
                pass: bad.is_none(),
                detail: bad
                    .map(|s| format!("banked {:.0}° at {:.0} ft AGL, t={:.0}s", s.bank_deg, s.alt_agl_ft, s.t))
                    .unwrap_or_else(|| "none".into()),
            }
        }
        Check::NoGoAround => {
            let ga = samples.iter().find(|s| s.phase == Some(FlightPhase::GoAround));
            CheckResult {
                check: "no go-around".into(),
                pass: ga.is_none(),
                detail: ga.map(|s| format!("went around at t={:.0}s", s.t)).unwrap_or_else(|| "none".into()),
            }
        }
        Check::NoCrash => {
            let c = samples.iter().find(|s| s.crashed);
            CheckResult {
                check: "no crash".into(),
                pass: c.is_none(),
                detail: c.map(|s| format!("crashed at t={:.0}s", s.t)).unwrap_or_else(|| "ok".into()),
            }
        }
    })
}

/// Ground track over the place list: roads, areas, runways, the track
/// colored by phase, and markers where messages went out and turns began.
pub fn write_ground_svg(
    path: &Path,
    title: &str,
    samples: &[Sample],
    sent_at: &[Option<f64>],
    gazetteer: &Gazetteer,
    resolver: &mut Resolver,
    runways: &[(String, Vec2, Vec2)],
    traffic: Option<&ScriptedTraffic>,
) -> Result<()> {
    if samples.is_empty() {
        return Ok(());
    }
    let (mut lo, mut hi) = (samples[0].pos, samples[0].pos);
    for s in samples {
        lo = Vec2::new(lo.x.min(s.pos.x), lo.y.min(s.pos.y));
        hi = Vec2::new(hi.x.max(s.pos.x), hi.y.max(s.pos.y));
    }
    for (_, a, b) in runways {
        for p in [a, b] {
            lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
        }
    }
    let pad = 6000.0;
    lo = lo - Vec2::new(pad, pad);
    hi = hi + Vec2::new(pad, pad);
    let w_ft = hi.x - lo.x;
    let h_ft = hi.y - lo.y;
    let width = 900.0;
    let scale = width / w_ft;
    let height = h_ft * scale;
    let px = |p: Vec2| ((p.x - lo.x) * scale, (hi.y - p.y) * scale);
    let inside = |p: Vec2| p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y;

    let mut svg = String::new();
    let _ = write!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0}" height="{height:.0}" viewBox="0 0 {width:.0} {height:.0}" font-family="sans-serif" font-size="11"><rect width="100%" height="100%" fill="#fbfaf7"/>"##
    );
    // Places.
    for place in &gazetteer.places {
        let Ok(f) = resolver.resolve(&Ref::Place { name: place.name.clone() }) else { continue };
        match &f.geom {
            Geom::Line(l) => {
                let pts: Vec<String> = l.points.iter().map(|p| { let (x, y) = px(*p); format!("{x:.1},{y:.1}") }).collect();
                let _ = write!(svg, r##"<polyline points="{}" fill="none" stroke="#c9a227" stroke-width="3" opacity="0.7"/>"##, pts.join(" "));
                if let Some(p) = l.points.iter().find(|p| inside(**p)) {
                    let (x, y) = px(*p);
                    let _ = write!(svg, r##"<text x="{:.1}" y="{:.1}" fill="#8a6d0b">{}</text>"##, x + 4.0, y - 4.0, xml(&f.label));
                }
            }
            Geom::Area { ring, center } if place.kind == PlaceKind::Area && inside(*center) => {
                let pts: Vec<String> = ring.iter().map(|p| { let (x, y) = px(*p); format!("{x:.1},{y:.1}") }).collect();
                let _ = write!(svg, r##"<polygon points="{}" fill="#9cc3a4" opacity="0.35" stroke="#5f8f69"/>"##, pts.join(" "));
                let (x, y) = px(*center);
                let _ = write!(svg, r##"<text x="{x:.1}" y="{y:.1}" fill="#3e6b48">{}</text>"##, xml(&f.label));
            }
            Geom::Point(p) if inside(*p) => {
                let (x, y) = px(*p);
                let _ = write!(svg, r##"<circle cx="{x:.1}" cy="{y:.1}" r="3" fill="#5f8f69"/><text x="{:.1}" y="{y:.1}" fill="#3e6b48">{}</text>"##, x + 5.0, xml(&f.label));
            }
            _ => {}
        }
    }
    // Runways.
    for (id, a, b) in runways {
        let (x1, y1) = px(*a);
        let (x2, y2) = px(*b);
        let _ = write!(svg, r##"<line x1="{x1:.1}" y1="{y1:.1}" x2="{x2:.1}" y2="{y2:.1}" stroke="#333" stroke-width="4"/><text x="{:.1}" y="{:.1}">{}</text>"##, x1 + 5.0, y1, xml(id));
    }
    // Track, by phase.
    let color = |p: Option<FlightPhase>| match p {
        Some(FlightPhase::PatternEntry) => "#7a5cc4",
        Some(FlightPhase::Crosswind) | Some(FlightPhase::InitialClimb) => "#2a9d8f",
        Some(FlightPhase::Downwind) => "#1f5f8b",
        Some(FlightPhase::Base) => "#e76f51",
        Some(FlightPhase::Final) | Some(FlightPhase::Roundout) | Some(FlightPhase::Flare) => "#d62828",
        Some(FlightPhase::GoAround) => "#ff00aa",
        _ => "#555555",
    };
    for w in samples.windows(2) {
        let (x1, y1) = px(w[0].pos);
        let (x2, y2) = px(w[1].pos);
        let _ = write!(svg, r##"<line x1="{x1:.1}" y1="{y1:.1}" x2="{x2:.1}" y2="{y2:.1}" stroke="{}" stroke-width="2"/>"##, color(w[1].phase));
    }
    // Traffic: dashed purple tracks, and where each target was when the
    // aircraft changed phase (a dot), to check a follower's base turn.
    if let Some(tr) = traffic {
        let mut tracks: std::collections::BTreeMap<String, Vec<(f64, f64)>> = Default::default();
        for s in samples {
            for t in tr.at(s.t) {
                tracks.entry(t.id.clone()).or_default().push(px(t.position_ft));
            }
        }
        for (id, pts) in &tracks {
            let line: Vec<String> = pts.iter().map(|(x, y)| format!("{x:.1},{y:.1}")).collect();
            let _ = write!(svg, r##"<polyline points="{}" fill="none" stroke="#7b2cbf" stroke-width="1.5" stroke-dasharray="5,4"/>"##, line.join(" "));
            if let Some((x, y)) = pts.first() {
                let _ = write!(svg, r##"<text x="{:.1}" y="{:.1}" fill="#7b2cbf" font-weight="bold">{}</text>"##, x + 5.0, y - 5.0, xml(id));
            }
        }
        for w in samples.windows(2) {
            if w[0].phase != w[1].phase && w[1].phase.is_some() {
                for t in tr.at(w[1].t) {
                    let (x, y) = px(t.position_ft);
                    let _ = write!(svg, r##"<circle cx="{x:.1}" cy="{y:.1}" r="3" fill="#7b2cbf"/>"##);
                }
            }
        }
    }
    for w in samples.windows(2) {
        if w[0].phase != w[1].phase {
            if let Some(p) = w[1].phase {
                let (x, y) = px(w[1].pos);
                let _ = write!(svg, r##"<circle cx="{x:.1}" cy="{y:.1}" r="3.5" fill="white" stroke="{}"/><text x="{:.1}" y="{:.1}" fill="{}">{}</text>"##, color(Some(p)), x + 5.0, y + 12.0, color(Some(p)), p.value());
            }
        }
    }
    for (i, t) in sent_at.iter().enumerate() {
        let Some(t) = t else { continue };
        if let Some(s) = samples.iter().find(|s| s.t >= *t) {
            let (x, y) = px(s.pos);
            let _ = write!(svg, r##"<rect x="{:.1}" y="{:.1}" width="8" height="8" fill="#111"/><text x="{:.1}" y="{:.1}" font-weight="bold">msg {i}</text>"##, x - 4.0, y - 4.0, x + 6.0, y - 6.0);
        }
    }
    let _ = write!(svg, r##"<text x="10" y="18" font-size="14" font-weight="bold">{}</text>"##, xml(title));
    let nm = NM_TO_FT * scale;
    let _ = write!(svg, r##"<line x1="10" y1="{:.1}" x2="{:.1}" y2="{:.1}" stroke="#333" stroke-width="2"/><text x="10" y="{:.1}">1 nm</text></svg>"##, height - 12.0, 10.0 + nm, height - 12.0, height - 16.0);
    std::fs::write(path, svg).with_context(|| format!("writing {}", path.display()))
}

fn xml(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}
