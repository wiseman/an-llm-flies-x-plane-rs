//! Compile a `PlanDelta` (from the Python front end) into runtime form:
//! references resolved, pattern runways turned into configs, conditions
//! ready to evaluate.

use std::collections::HashMap;

use anyhow::{anyhow, bail, Result};

use crate::config::ConfigBundle;
use crate::guidance::runway_geometry::RunwayFrame;
use crate::plan::cond::RCond;
use crate::plan::ir::{
    Cond, Constraint, Dir, Extension, LandKind, Leg, PatternSpec, PlanDelta, Ref, Report, Step, StepKind,
};
use crate::plan::resolve::{Feature, Geom, Resolver};
use crate::types::{Runway, TrafficSide, Vec2, NM_TO_FT};

/// Touchdown zone given to runways looked up from apt.dat (plans and the
/// runway tools alike): the aim point is half of it, 600 ft past the
/// landing threshold, room for the flare float without overrunning short
/// strips.
pub const TOUCHDOWN_ZONE_FT: f64 = 1200.0;

#[derive(Debug, Clone)]
pub struct RTrigger {
    pub when: Option<RCond>,
    pub earliest: Option<RCond>,
    pub latest: Option<RCond>,
    pub text: String,
}

#[derive(Debug, Clone)]
pub struct RPattern {
    pub spec: PatternSpec,
    pub config: ConfigBundle,
    pub frame: RunwayFrame,
    pub triggers: HashMap<Leg, RTrigger>,
    pub extensions: HashMap<Leg, Extension>,
}

impl RPattern {
    pub fn same_runway(&self, other: &RPattern) -> bool {
        self.spec.airport.eq_ignore_ascii_case(&other.spec.airport)
            && self.spec.runway.eq_ignore_ascii_case(&other.spec.runway)
    }

    /// The same runway and the same side: a pattern that can be amended
    /// in flight. Left traffic for the runway being flown right is a new
    /// pattern, not an edit.
    pub fn same_circuit(&self, other: &RPattern) -> bool {
        self.same_runway(other) && self.spec.side == other.spec.side
    }

    pub fn describe(&self) -> String {
        let mut bits = vec![format!(
            "{} traffic {} {}",
            self.spec.side.as_str(),
            self.spec.airport,
            self.spec.runway
        )];
        for leg in [Leg::Upwind, Leg::Crosswind, Leg::Downwind] {
            if let Some(e) = self.extensions.get(&leg) {
                bits.push(match e {
                    Extension::Ft(ft) => format!("{} extended {:.1} nm", leg.as_str(), ft / NM_TO_FT),
                    Extension::Open(_) => format!("{} extended open", leg.as_str()),
                });
            }
        }
        for leg in [Leg::Crosswind, Leg::Downwind, Leg::Base, Leg::Final] {
            if let Some(t) = self.triggers.get(&leg) {
                bits.push(format!("turn {}: {}", leg.as_str(), t.text));
            }
        }
        if let Some(w) = self.spec.width_ft {
            bits.push(format!("downwind {:.1} nm out", w / NM_TO_FT));
        }
        let sq = self.square();
        if sq.into_base {
            bits.push("square base (both turns)".to_string());
        } else if sq.into_final {
            bits.push("square turn to final".to_string());
        }
        bits.join(", ")
    }

    pub fn square(&self) -> crate::core::profiles::SquareCorners {
        let base = self.spec.square.contains(&Leg::Base);
        crate::core::profiles::SquareCorners { into_base: true, into_final: base || self.spec.square.contains(&Leg::Final) }
    }
}

#[derive(Debug, Clone)]
pub enum RStepKind {
    Heading { heading_deg: Option<f64>, turn: Option<Dir> },
    Turn { dir: Dir, by_deg: Option<f64>, to_deg: Option<f64> },
    Direct { target: Feature },
    Toward { target: Feature },
    Orbit { dir: Dir, degrees: f64, bank_deg: f64 },
    Follow {
        label: String,
        line: crate::guidance::geo::Polyline,
        direction_deg: Option<f64>,
        keep: Option<Dir>,
        side_deg: Option<f64>,
        offset_ft: f64,
    },
    Circle { target: Feature, dir: Dir, radius_ft: f64, turns: Option<f64> },
    RectangularCourse {
        label: String,
        path: crate::guidance::geo::ClosedPath,
        dir: Dir,
        offset_ft: f64,
        laps: Option<f64>,
        altitude_ft: Option<f64>,
    },
    Takeoff { pattern: Box<RPattern> },
    Climb { altitude_ft: f64, speed_kt: Option<f64> },
    Descend { altitude_ft: f64, speed_kt: Option<f64> },
    Maintain { altitude_ft: f64 },
    Speed { kt: f64 },
    Pattern { pattern: Box<RPattern>, entry: Option<Leg>, land: LandKind },
}

impl RStepKind {
    pub fn is_lateral(&self) -> bool {
        matches!(
            self,
            RStepKind::Heading { .. }
                | RStepKind::Turn { .. }
                | RStepKind::Direct { .. }
                | RStepKind::Toward { .. }
                | RStepKind::Orbit { .. }
                | RStepKind::Follow { .. }
                | RStepKind::Circle { .. }
                | RStepKind::RectangularCourse { .. }
                | RStepKind::Takeoff { .. }
                | RStepKind::Pattern { .. }
        )
    }

    /// The runway-bound pattern a step flies (pattern or takeoff).
    pub fn pattern(&self) -> Option<&RPattern> {
        match self {
            RStepKind::Pattern { pattern, .. } | RStepKind::Takeoff { pattern } => Some(pattern),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RStep {
    pub kind: RStepKind,
    pub until: Option<RCond>,
    pub text: String,
}

#[derive(Debug, Clone)]
pub struct RConstraint {
    pub id: String,
    pub text: String,
    pub pred: RCond,
    pub until: Option<RCond>,
    pub after: Option<RCond>,
    pub soft: bool,
    /// `after` has happened (or there is none).
    pub started: bool,
    /// `until` has happened: the constraint no longer applies.
    pub ended: bool,
    pub violating: bool,
}

impl RConstraint {
    /// Steering and altitude limits apply: a hard constraint, started
    /// and not yet ended.
    pub fn enforced(&self) -> bool {
        !self.soft && self.started && !self.ended
    }
}

#[derive(Debug, Clone)]
pub struct RReport {
    pub id: String,
    pub text: String,
    pub when: RCond,
    pub fired: bool,
}

#[derive(Debug, Clone, Default)]
pub struct CompiledDelta {
    pub steps: Option<Vec<RStep>>,
    pub directives: Vec<RStep>,
    pub pattern: Option<RPattern>,
    pub constraints: Vec<RConstraint>,
    pub reports: Vec<RReport>,
    pub release: Vec<String>,
    pub cancel: Vec<String>,
    pub replace: bool,
    pub go_around: bool,
    pub permits: Vec<String>,
}

impl CompiledDelta {
    /// The pattern runway the pilot's reference frame should follow: the
    /// first pattern step's, else the live-pattern edit's.
    pub fn reference_pattern(&self) -> Option<&RPattern> {
        self.steps
            .iter()
            .flatten()
            .find_map(|s| s.kind.pattern())
            .or(self.pattern.as_ref())
    }
}

pub fn compile(delta: &PlanDelta, r: &mut Resolver, base: &ConfigBundle) -> Result<CompiledDelta> {
    let steps = match &delta.steps {
        Some(steps) => Some(steps.iter().map(|s| compile_step(s, r, base)).collect::<Result<Vec<_>>>()?),
        None => None,
    };
    let directives = delta
        .directives
        .iter()
        .map(|s| compile_step(s, r, base))
        .collect::<Result<Vec<_>>>()?;
    if let Some(d) = directives.iter().find(|d| d.kind.is_lateral()) {
        bail!("internal: lateral step {:?} sent as a directive", d.text);
    }
    let pattern = delta.pattern.as_ref().map(|p| compile_pattern(p, r, base)).transpose()?;
    let constraints = delta.constraints.iter().map(|c| compile_constraint(c, r)).collect::<Result<Vec<_>>>()?;
    let reports = delta.reports.iter().map(|c| compile_report(c, r)).collect::<Result<Vec<_>>>()?;
    Ok(CompiledDelta {
        steps,
        directives,
        pattern,
        constraints,
        reports,
        release: delta.release.clone(),
        cancel: delta.cancel.clone(),
        replace: delta.replace,
        go_around: delta.go_around,
        permits: delta.permits.clone(),
    })
}

fn opt_cond(c: &Option<Cond>, r: &mut Resolver) -> Result<Option<RCond>> {
    c.as_ref().map(|c| RCond::resolve(c, r)).transpose()
}

fn compile_step(s: &Step, r: &mut Resolver, base: &ConfigBundle) -> Result<RStep> {
    let until = opt_cond(&s.until, r)?;
    let kind = match &s.kind {
        StepKind::Heading { heading_deg, turn } => RStepKind::Heading { heading_deg: *heading_deg, turn: *turn },
        StepKind::Turn { dir, by_deg, to_deg } => {
            if by_deg.is_none() == to_deg.is_none() {
                bail!("turn needs exactly one of by_deg or to_deg");
            }
            RStepKind::Turn { dir: *dir, by_deg: *by_deg, to_deg: *to_deg }
        }
        StepKind::Direct { target } => RStepKind::Direct { target: point_feature(target, r)? },
        StepKind::Toward { target } => RStepKind::Toward { target: point_feature(target, r)? },
        StepKind::Orbit { dir, degrees, bank_deg } => RStepKind::Orbit {
            dir: *dir,
            degrees: *degrees,
            bank_deg: bank_deg.unwrap_or(25.0).clamp(10.0, base.limits.max_bank_enroute_deg),
        },
        StepKind::Follow { line, direction_deg, keep, side_deg, offset_ft } => {
            let f = r.resolve(line)?;
            let Geom::Line(l) = f.geom else {
                bail!("follow() needs a line (a road, the shoreline); {} is a {}", f.label, f.geom.kind());
            };
            if keep.is_some() && side_deg.is_some() {
                bail!("follow() takes keep= or side=, not both");
            }
            let default_offset = if keep.is_some() || side_deg.is_some() { 2000.0 } else { 0.0 };
            RStepKind::Follow {
                label: f.label,
                line: l,
                direction_deg: *direction_deg,
                keep: *keep,
                side_deg: *side_deg,
                offset_ft: offset_ft.unwrap_or(default_offset).abs(),
            }
        }
        StepKind::Circle { target, dir, radius_ft, turns } => RStepKind::Circle {
            target: point_feature(target, r)?,
            dir: *dir,
            radius_ft: radius_ft.unwrap_or(3000.0).clamp(1200.0, 12000.0),
            turns: *turns,
        },
        StepKind::RectangularCourse { target, dir, offset_ft, laps, size_ft, altitude_ft } => {
            let f = point_feature(target, r)?;
            let offset = offset_ft.unwrap_or(RECT_DEFAULT_OFFSET_FT);
            if !(300.0..=NM_TO_FT).contains(&offset) {
                bail!("rectangular course offset {offset:.0} ft is outside 300 ft - 1 nm");
            }
            if let Some(s) = size_ft {
                if !(1000.0..=3.0 * NM_TO_FT).contains(s) {
                    bail!("rectangular course size {s:.0} ft is outside 1000 ft - 3 nm");
                }
            }
            if let Some(n) = laps {
                if !(0.5..=10.0).contains(n) {
                    bail!("rectangular course laps {n} is outside 0.5-10");
                }
            }
            let rect = course_rect(&f, *size_ft);
            RStepKind::RectangularCourse {
                label: f.label.clone(),
                path: crate::guidance::geo::rectangular_course(&rect, offset, RECT_CORNER_RADIUS_FT, *dir == Dir::Left),
                dir: *dir,
                offset_ft: offset,
                laps: *laps,
                altitude_ft: *altitude_ft,
            }
        }
        StepKind::Takeoff { airport, runway } => {
            let spec = PatternSpec {
                airport: airport.clone(),
                runway: runway.clone(),
                side: Dir::Left,
                altitude_ft: None,
                width_ft: None,
                extend: Default::default(),
                turns: Default::default(),
                square: Vec::new(),
            };
            RStepKind::Takeoff { pattern: Box::new(compile_pattern(&spec, r, base)?) }
        }
        StepKind::Climb { altitude_ft, speed_kt } => RStepKind::Climb { altitude_ft: *altitude_ft, speed_kt: *speed_kt },
        StepKind::Descend { altitude_ft, speed_kt } => {
            RStepKind::Descend { altitude_ft: *altitude_ft, speed_kt: *speed_kt }
        }
        StepKind::Maintain { altitude_ft } => RStepKind::Maintain { altitude_ft: *altitude_ft },
        StepKind::Speed { kt } => RStepKind::Speed { kt: *kt },
        StepKind::Pattern { pattern, entry, land } => {
            if matches!(entry, Some(Leg::Upwind) | Some(Leg::Crosswind)) {
                bail!("entering on the {} isn't supported yet; join the pattern instead", entry.unwrap().as_str());
            }
            RStepKind::Pattern { pattern: Box::new(compile_pattern(pattern, r, base)?), entry: *entry, land: *land }
        }
    };
    let mut text = describe_step(&kind);
    match (&s.until, &kind) {
        // A vertical step ended at once only sets the new target: "start
        // down to 1800" while the next step flies on.
        (Some(Cond::Now), RStepKind::Climb { altitude_ft, .. }) => text = format!("start climbing to {altitude_ft:.0}"),
        (Some(Cond::Now), RStepKind::Descend { altitude_ft, .. }) => text = format!("start descending to {altitude_ft:.0}"),
        (Some(u), _) => text.push_str(&format!(" until {}", cond_text(u))),
        (None, _) => {}
    }
    Ok(RStep { kind, until, text })
}

/// How far out a rectangular course flies from the field's edges by
/// default (AFH: a quarter to a half mile).
pub const RECT_DEFAULT_OFFSET_FT: f64 = 0.25 * NM_TO_FT;
/// Corner radius of a rectangular course: about 25° of bank at 90 kt in
/// still air, steeper downwind.
pub const RECT_CORNER_RADIUS_FT: f64 = 1500.0;
/// Side of the square flown around a point with no size given.
const RECT_POINT_SIZE_FT: f64 = 3000.0;

/// The field a rectangular course goes around: the area's smallest
/// bounding rectangle, or a `size_ft` (default 3000 ft) square centered
/// on it, squared to north.
pub fn course_rect(f: &Feature, size_ft: Option<f64>) -> crate::guidance::geo::Rect {
    use crate::guidance::geo::{bounding_rect, Rect};
    let square = |s: f64| Rect { center: f.geom.anchor(), u: Vec2::new(0.0, 1.0), half_u: s / 2.0, half_v: s / 2.0 };
    match (&f.geom, size_ft) {
        (_, Some(s)) => square(s),
        (Geom::Area { ring, .. }, None) => bounding_rect(ring),
        _ => square(RECT_POINT_SIZE_FT),
    }
}

fn point_feature(target: &Ref, r: &mut Resolver) -> Result<Feature> {
    let f = r.resolve(target)?;
    if matches!(f.geom, Geom::Line(_)) {
        bail!("{} is a line; fly to a point or area on it (an interchange, a landmark)", f.label);
    }
    Ok(f)
}

/// A runway end as the pilot's `Runway`, in world feet about `georef`.
pub fn runway_from_info(rw: &crate::plan::resolve::RunwayInfo, georef: crate::sim::xplane_bridge::GeoReference, side: TrafficSide) -> Runway {
    Runway {
        id: Some(rw.ident.clone()),
        threshold_ft: crate::sim::xplane_bridge::geodetic_offset_ft(rw.lat, rw.lon, georef),
        course_deg: rw.course_deg,
        length_ft: rw.length_ft,
        touchdown_zone_ft: TOUCHDOWN_ZONE_FT,
        displaced_threshold_ft: rw.displaced_threshold_ft,
        width_ft: rw.width_ft,
        traffic_side: side,
    }
}

pub fn compile_pattern(p: &PatternSpec, r: &mut Resolver, base: &ConfigBundle) -> Result<RPattern> {
    let apt = r.airport(&p.airport)?.clone();
    let rw = apt
        .runway(&p.runway)
        .ok_or_else(|| anyhow!("runway {} not found at {}", p.runway, p.airport))?
        .clone();
    let side = match p.side {
        Dir::Left => TrafficSide::Left,
        Dir::Right => TrafficSide::Right,
    };
    let runway = runway_from_info(&rw, r.georef, side);
    let mut config = base.clone();
    config.airport.airport = Some(apt.ident.clone());
    config.airport.field_elevation_ft = apt.elevation_ft;
    config.airport.runway = runway.clone();
    if let Some(alt) = p.altitude_ft {
        let agl = alt - apt.elevation_ft;
        if !(400.0..=3000.0).contains(&agl) {
            bail!("pattern altitude {alt:.0} is {agl:.0} ft above the field; expected 400-3000");
        }
        config.pattern.altitude_agl_ft = agl;
    }
    if let Some(w) = p.width_ft {
        if !(1500.0..=15000.0).contains(&w) {
            bail!("pattern width {:.0} ft is outside 1500-15000 ft", w);
        }
        config.pattern.downwind_offset_ft = w;
    }
    let mut triggers = HashMap::new();
    for leg in [Leg::Crosswind, Leg::Downwind, Leg::Base, Leg::Final] {
        if let Some(t) = p.turns.get(leg) {
            let mut parts = Vec::new();
            if let Some(c) = &t.when {
                parts.push(cond_text(c));
            }
            if let Some(c) = &t.earliest {
                parts.push(format!("not before {}", cond_text(c)));
            }
            if let Some(c) = &t.latest {
                parts.push(format!("no later than {}", cond_text(c)));
            }
            triggers.insert(
                leg,
                RTrigger {
                    when: opt_cond(&t.when, r)?,
                    earliest: opt_cond(&t.earliest, r)?,
                    latest: opt_cond(&t.latest, r)?,
                    text: parts.join(", "),
                },
            );
        }
    }
    if let Some(l) = p.square.iter().find(|l| !matches!(l, Leg::Base | Leg::Final)) {
        bail!("only base and final can be squared, not {}", l.as_str());
    }
    let mut extensions = HashMap::new();
    for (leg, e) in [(Leg::Upwind, p.extend.upwind), (Leg::Crosswind, p.extend.crosswind), (Leg::Downwind, p.extend.downwind)] {
        if let Some(e) = e {
            if let Extension::Ft(ft) = e {
                if !(0.0..=10.0 * NM_TO_FT).contains(&ft) {
                    bail!("{} extension {:.0} ft is outside 0-10 nm", leg.as_str(), ft);
                }
            }
            extensions.insert(leg, e);
        }
    }
    Ok(RPattern { spec: p.clone(), frame: RunwayFrame::new(runway), config, triggers, extensions })
}

fn compile_constraint(c: &Constraint, r: &mut Resolver) -> Result<RConstraint> {
    let after = opt_cond(&c.after, r)?;
    Ok(RConstraint {
        id: c.id.clone(),
        text: c.text.clone(),
        pred: RCond::resolve(&c.pred, r)?,
        until: opt_cond(&c.until, r)?,
        started: after.is_none(),
        after,
        soft: c.soft,
        ended: false,
        violating: false,
    })
}

fn compile_report(c: &Report, r: &mut Resolver) -> Result<RReport> {
    Ok(RReport { id: c.id.clone(), text: c.text.clone(), when: RCond::resolve(&c.when, r)?, fired: false })
}


pub fn describe_step(k: &RStepKind) -> String {
    match k {
        RStepKind::Heading { heading_deg: None, .. } => "present heading".into(),
        RStepKind::Heading { heading_deg: Some(h), turn } => match turn {
            Some(d) => format!("turn {} heading {:03.0}", Dir::as_str(*d), h),
            None => format!("heading {:03.0}", h),
        },
        RStepKind::Turn { dir, by_deg: Some(b), .. } => format!("turn {} {:.0}°", Dir::as_str(*dir), b),
        RStepKind::Turn { dir, to_deg, .. } => format!("turn {} heading {:03.0}", Dir::as_str(*dir), to_deg.unwrap_or(0.0)),
        RStepKind::Direct { target } => format!("direct {}", target.label),
        RStepKind::Toward { target } => format!("toward {}", target.label),
        RStepKind::Orbit { dir, degrees, bank_deg } if *bank_deg >= 40.0 => {
            format!("steep turn {} {:.0} ({:.0}° bank)", Dir::as_str(*dir), degrees, bank_deg)
        }
        RStepKind::Orbit { dir, degrees, .. } => format!("{} {:.0}", Dir::as_str(*dir), degrees),
        RStepKind::Follow { label, direction_deg, keep, side_deg, offset_ft, .. } => {
            let mut t = format!("follow {label}");
            if let Some(d) = direction_deg {
                t.push_str(&format!(" {}bound", side_name(*d)));
            }
            if let Some(k) = keep {
                t.push_str(&format!(", {:.1} nm off the {} wing", offset_ft / NM_TO_FT, Dir::as_str(*k)));
            } else if let Some(sd) = side_deg {
                t.push_str(&format!(", {:.1} nm {} of it", offset_ft / NM_TO_FT, side_name(*sd)));
            }
            t
        }
        RStepKind::Circle { target, dir, radius_ft, turns } => format!(
            "circle {} {} ({:.1} nm){}",
            target.label,
            Dir::as_str(*dir),
            radius_ft / NM_TO_FT,
            turns.map(|n| format!(", {n:.0} times")).unwrap_or_default()
        ),
        RStepKind::RectangularCourse { label, dir, offset_ft, laps, altitude_ft, .. } => format!(
            "rectangular course around {label}, {:.2} nm out, {} turns{}{}",
            offset_ft / NM_TO_FT,
            Dir::as_str(*dir),
            laps.map(|n| format!(", {n:.0} laps")).unwrap_or_default(),
            altitude_ft.map(|a| format!(", at {a:.0}")).unwrap_or_default()
        ),
        RStepKind::Takeoff { pattern } => format!("take off {} {}", pattern.spec.airport, pattern.spec.runway),
        RStepKind::Climb { altitude_ft, .. } => format!("climb to {:.0}", altitude_ft),
        RStepKind::Descend { altitude_ft, .. } => format!("descend to {:.0}", altitude_ft),
        RStepKind::Maintain { altitude_ft } => format!("maintain {:.0}", altitude_ft),
        RStepKind::Speed { kt } => format!("speed {:.0} kt", kt),
        RStepKind::Pattern { pattern, entry, land } => format!(
            "{} {}{}",
            match entry {
                Some(l) => format!("enter on {},", l.as_str()),
                None => "join".to_string(),
            },
            pattern.describe(),
            match land {
                LandKind::FullStop => ", full stop",
                LandKind::TouchAndGo => ", touch and go",
                LandKind::LowApproach => ", low approach",
            }
        ),
    }
}

pub fn ref_text(r: &Ref) -> String {
    match r {
        Ref::Place { name } => name.clone(),
        Ref::LatLon { lat, lon } => format!("{lat:.4},{lon:.4}"),
        Ref::Runway { airport, runway, part } => format!("{airport} {runway} {}", part.replace('_', " ")),
        Ref::Airport { ident } => ident.clone(),
        Ref::Intersection { a, b } => format!("{}/{} interchange", ref_text(a), ref_text(b)),
        Ref::RunwayFrame { airport, runway, along_ft, right_ft } => {
            format!("{airport} {runway} +{:.1}/{:+.1} nm", along_ft / NM_TO_FT, right_ft / NM_TO_FT)
        }
    }
}

fn side_name(deg: f64) -> String {
    match deg.rem_euclid(360.0).round() as i64 {
        0 => "north".into(),
        90 => "east".into(),
        180 => "south".into(),
        270 => "west".into(),
        d => format!("{d}° side"),
    }
}

pub fn cond_text(c: &Cond) -> String {
    match c {
        Cond::Now => "now".into(),
        Cond::Asap => "as soon as able".into(),
        Cond::Never => "never".into(),
        Cond::Atc { token } => format!("ATC calls {token}"),
        Cond::Advised => "advised".into(),
        Cond::At { target, .. } => format!("at {}", ref_text(target)),
        Cond::Abeam { target } => format!("abeam {}", ref_text(target)),
        Cond::Crossing { target } => format!("crossing {}", ref_text(target)),
        Cond::Past { target } => format!("past {}", ref_text(target)),
        Cond::SideOf { target, side_deg } => format!("{} of {}", side_name(*side_deg), ref_text(target)),
        Cond::Within { target, distance_ft } => format!("within {:.1} nm of {}", distance_ft / NM_TO_FT, ref_text(target)),
        Cond::Inside { target } => format!("inside {}", ref_text(target)),
        Cond::Outside { target } => format!("outside {}", ref_text(target)),
        Cond::Over { target, .. } => format!("over {}", ref_text(target)),
        Cond::Reaching { altitude_ft } => format!("reaching {altitude_ft:.0}"),
        Cond::AtOrBelow { altitude_ft } => format!("at or below {altitude_ft:.0}"),
        Cond::AtOrAbove { altitude_ft } => format!("at or above {altitude_ft:.0}"),
        Cond::AglBelow { ft } => format!("below {ft:.0} AGL"),
        Cond::AglAbove { ft } => format!("above {ft:.0} AGL"),
        Cond::DistanceFlown { ft } => format!("after {:.1} nm", ft / NM_TO_FT),
        Cond::Elapsed { s } => format!("after {s:.0} s"),
        Cond::OnLeg { leg } => format!("on {}", leg.as_str()),
        Cond::Turning { leg } => format!("turning {}", leg.as_str()),
        Cond::Established { leg } => format!("established on {}", leg.as_str()),
        Cond::AngleBehindWing { target, deg } => format!("{} {deg:.0}° behind the wing", ref_text(target)),
        Cond::InSight { traffic } => format!("{traffic} in sight"),
        Cond::Passed { traffic } => format!("past {traffic}"),
        Cond::Behind { traffic, distance_ft: None } => format!("behind {traffic}"),
        Cond::Behind { traffic, distance_ft: Some(d) } => format!("{:.1} nm behind {traffic}", d / NM_TO_FT),
        Cond::Following { traffic } => format!("{traffic} past you on final (or landed)"),
        Cond::All { conds } => conds.iter().map(cond_text).collect::<Vec<_>>().join(" and "),
        Cond::Any { conds } => conds.iter().map(cond_text).collect::<Vec<_>>().join(" or "),
        Cond::Not { of } => format!("not {}", cond_text(of)),
    }
}

/// Runway-frame position of a world point, for summaries ("1.9 nm N of
/// the 16R threshold").
pub fn where_from(frame: &RunwayFrame, p: Vec2) -> String {
    let d = p - frame.runway.threshold_ft;
    let dist_nm = d.length() / NM_TO_FT;
    let b = crate::types::vector_to_heading(d);
    const C: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    format!(
        "{:.1} nm {} of the {} threshold",
        dist_nm,
        C[(((b + 22.5) / 45.0).floor() as usize) % 8],
        frame.runway.id.as_deref().unwrap_or("runway")
    )
}
