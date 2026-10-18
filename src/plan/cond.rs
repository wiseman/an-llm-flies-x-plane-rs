//! Runtime conditions: `ir::Cond` with its references resolved to
//! geometry, plus the per-condition memory that event conditions
//! (crossing, abeam, past, reaching) need between ticks.

use std::collections::HashSet;

use anyhow::Result;

use crate::core::traffic::{bind, TrafficView, VISUAL_RANGE_FT};
use crate::guidance::geo::{along_track_ft, angle_behind_wing_deg, closest_on_segment, point_in_polygon};
use crate::plan::ir::{Cond, Leg};
use crate::plan::resolve::{Feature, Geom, Resolver};
use crate::types::{heading_to_vector, Vec2, NM_TO_FT};

/// "At" a point counts within this radius, or abeam it within
/// `AT_ABEAM_MAX_FT` when the point is off the track.
const AT_RADIUS_FT: f64 = 1000.0;
const AT_ABEAM_MAX_FT: f64 = 2.0 * NM_TO_FT;
const OVER_LINE_WIDTH_FT: f64 = 1500.0;
const REACHING_TOL_FT: f64 = 100.0;
/// "Established" on a leg: on it this long with the track settled.
pub const ESTABLISHED_S: f64 = 8.0;

/// The pattern leg a flight phase is flying.
pub fn leg_for_phase(p: crate::types::FlightPhase) -> Option<Leg> {
    use crate::types::FlightPhase as F;
    match p {
        F::InitialClimb | F::GoAround => Some(Leg::Upwind),
        F::Crosswind => Some(Leg::Crosswind),
        F::Downwind => Some(Leg::Downwind),
        F::Base => Some(Leg::Base),
        F::Final | F::Roundout | F::Flare => Some(Leg::Final),
        _ => None,
    }
}

/// What a condition can see each tick.
pub struct EvalCtx<'a> {
    pub position_ft: Vec2,
    pub prev_position_ft: Vec2,
    pub track_deg: f64,
    pub alt_msl_ft: f64,
    pub alt_agl_ft: f64,
    pub dt: f64,
    /// Current pattern leg, if a pattern is being flown.
    pub leg: Option<Leg>,
    /// Seconds on the current leg.
    pub leg_time_s: f64,
    /// ATC gates opened so far.
    pub gates: &'a HashSet<String>,
    /// Sensed traffic, placed against the pattern being flown.
    pub traffic: &'a [TrafficView],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrafficTest {
    InSight,
    Passed,
    Behind(f64),
    Following,
}

#[derive(Debug, Clone)]
pub enum RCond {
    Const(bool),
    Gate(String),
    At { feature: Feature, radius_ft: f64, prev_along: Option<f64>, latched: bool },
    Abeam { feature: Feature, prev_along: Option<f64>, latched: bool },
    Crossing { feature: Feature, prev_inside: Option<bool>, prev_along: Option<f64>, latched: bool },
    Past { feature: Feature, latched: bool },
    SideOf { feature: Feature, side_deg: f64 },
    Within { feature: Feature, distance_ft: f64 },
    Inside { feature: Feature },
    Outside { feature: Feature },
    Over { feature: Feature, width_ft: f64 },
    Reaching { altitude_ft: f64, start_above: Option<bool>, latched: bool },
    AtOrBelow(f64),
    AtOrAbove(f64),
    AglBelow(f64),
    AglAbove(f64),
    DistanceFlown { ft: f64, flown: f64 },
    Elapsed { s: f64, t: f64 },
    OnLeg(Leg),
    Turning { leg: Leg, latched: bool },
    Established(Leg),
    AngleBehindWing { feature: Feature, deg: f64 },
    /// A traffic condition. `id` is the target the description bound to
    /// (once one matched); a bound target that disappears has landed and
    /// gone.
    Traffic { desc: String, id: Option<String>, test: TrafficTest },
    All(Vec<RCond>),
    Any(Vec<RCond>),
    Not(Box<RCond>),
}

impl RCond {
    pub fn resolve(c: &Cond, r: &mut Resolver) -> Result<RCond> {
        Ok(match c {
            Cond::Now | Cond::Asap => RCond::Const(true),
            Cond::Never => RCond::Const(false),
            Cond::Atc { token } => RCond::Gate(token.clone()),
            Cond::Advised => RCond::Gate("advised".into()),
            Cond::At { target, radius_ft } => RCond::At {
                feature: r.resolve(target)?,
                radius_ft: radius_ft.unwrap_or(AT_RADIUS_FT),
                prev_along: None,
                latched: false,
            },
            Cond::Abeam { target } => RCond::Abeam { feature: r.resolve(target)?, prev_along: None, latched: false },
            Cond::Crossing { target } => {
                RCond::Crossing { feature: r.resolve(target)?, prev_inside: None, prev_along: None, latched: false }
            }
            Cond::Past { target } => RCond::Past { feature: r.resolve(target)?, latched: false },
            Cond::SideOf { target, side_deg } => RCond::SideOf { feature: r.resolve(target)?, side_deg: *side_deg },
            Cond::Within { target, distance_ft } => {
                RCond::Within { feature: r.resolve(target)?, distance_ft: *distance_ft }
            }
            Cond::Inside { target } => RCond::Inside { feature: r.resolve(target)? },
            Cond::Outside { target } => RCond::Outside { feature: r.resolve(target)? },
            Cond::Over { target, width_ft } => {
                RCond::Over { feature: r.resolve(target)?, width_ft: width_ft.unwrap_or(OVER_LINE_WIDTH_FT) }
            }
            Cond::Reaching { altitude_ft } => {
                RCond::Reaching { altitude_ft: *altitude_ft, start_above: None, latched: false }
            }
            Cond::AtOrBelow { altitude_ft } => RCond::AtOrBelow(*altitude_ft),
            Cond::AtOrAbove { altitude_ft } => RCond::AtOrAbove(*altitude_ft),
            Cond::AglBelow { ft } => RCond::AglBelow(*ft),
            Cond::AglAbove { ft } => RCond::AglAbove(*ft),
            Cond::DistanceFlown { ft } => RCond::DistanceFlown { ft: *ft, flown: 0.0 },
            Cond::Elapsed { s } => RCond::Elapsed { s: *s, t: 0.0 },
            Cond::OnLeg { leg } => RCond::OnLeg(*leg),
            Cond::Turning { leg } => RCond::Turning { leg: *leg, latched: false },
            Cond::Established { leg } => RCond::Established(*leg),
            Cond::AngleBehindWing { target, deg } => {
                RCond::AngleBehindWing { feature: r.resolve(target)?, deg: *deg }
            }
            Cond::InSight { traffic } => RCond::traffic(traffic, TrafficTest::InSight),
            Cond::Passed { traffic } => RCond::traffic(traffic, TrafficTest::Passed),
            Cond::Behind { traffic, distance_ft } => {
                RCond::traffic(traffic, TrafficTest::Behind(distance_ft.unwrap_or(0.0)))
            }
            Cond::Following { traffic } => RCond::traffic(traffic, TrafficTest::Following),
            Cond::All { conds } => RCond::All(conds.iter().map(|c| RCond::resolve(c, r)).collect::<Result<_>>()?),
            Cond::Any { conds } => RCond::Any(conds.iter().map(|c| RCond::resolve(c, r)).collect::<Result<_>>()?),
            Cond::Not { of } => RCond::Not(Box::new(RCond::resolve(of, r)?)),
        })
    }

    fn traffic(desc: &str, test: TrafficTest) -> RCond {
        RCond::Traffic { desc: desc.to_string(), id: None, test }
    }

    /// Traffic this condition has bound: (description, target id).
    pub fn traffic_bindings(&self, out: &mut Vec<(String, String)>) {
        match self {
            RCond::Traffic { desc, id: Some(id), .. } => {
                if !out.iter().any(|(d, i)| d == desc && i == id) {
                    out.push((desc.clone(), id.clone()));
                }
            }
            RCond::All(cs) | RCond::Any(cs) => cs.iter().for_each(|k| k.traffic_bindings(out)),
            RCond::Not(k) => k.traffic_bindings(out),
            _ => {}
        }
    }

    /// The places this condition refers to (the road a turn waits for).
    pub fn features<'a>(&'a self, out: &mut Vec<&'a Feature>) {
        match self {
            RCond::At { feature, .. }
            | RCond::Abeam { feature, .. }
            | RCond::Crossing { feature, .. }
            | RCond::Past { feature, .. }
            | RCond::SideOf { feature, .. }
            | RCond::Within { feature, .. }
            | RCond::Inside { feature }
            | RCond::Outside { feature }
            | RCond::Over { feature, .. }
            | RCond::AngleBehindWing { feature, .. } => {
                if !out.iter().any(|f| f.label == feature.label) {
                    out.push(feature);
                }
            }
            RCond::All(cs) | RCond::Any(cs) => cs.iter().for_each(|c| c.features(out)),
            RCond::Not(c) => c.features(out),
            _ => {}
        }
    }

    /// `eval`, with altitude limits (here or inside All/Any) allowed
    /// `tol_ft` of slack: "at or below 1800" holds up to 1800 + tol.
    pub fn holds_with_alt_tolerance(&mut self, c: &EvalCtx, tol_ft: f64) -> bool {
        match self {
            RCond::AtOrBelow(a) => c.alt_msl_ft <= *a + tol_ft,
            RCond::AtOrAbove(a) => c.alt_msl_ft >= *a - tol_ft,
            RCond::All(cs) => cs.iter_mut().fold(true, |acc, k| k.holds_with_alt_tolerance(c, tol_ft) & acc),
            RCond::Any(cs) => cs.iter_mut().fold(false, |acc, k| k.holds_with_alt_tolerance(c, tol_ft) | acc),
            other => other.eval(c),
        }
    }

    /// Advance one tick and say whether the condition holds. Every child
    /// of a combinator is evaluated (no short-circuit) so latches stay
    /// current.
    pub fn eval(&mut self, c: &EvalCtx) -> bool {
        match self {
            RCond::Const(b) => *b,
            RCond::Gate(token) => c.gates.contains(token.as_str()),
            RCond::At { feature, radius_ft, prev_along, latched } => {
                if !*latched {
                    *latched = match &feature.geom {
                        Geom::Point(p) => {
                            let d = c.position_ft.distance_to(*p);
                            d <= *radius_ft || (abeam_edge(prev_along, c, *p) && d <= AT_ABEAM_MAX_FT)
                        }
                        Geom::Line(l) => l.crossed_by(c.prev_position_ft, c.position_ft),
                        Geom::Area { center, ring } => {
                            point_in_polygon(c.position_ft, ring)
                                || (abeam_edge(prev_along, c, *center)
                                    && c.position_ft.distance_to(*center) <= AT_ABEAM_MAX_FT)
                        }
                    };
                }
                *latched
            }
            RCond::Abeam { feature, prev_along, latched } => {
                let p = feature.geom.anchor();
                if abeam_edge(prev_along, c, p) {
                    *latched = true;
                }
                *latched
            }
            RCond::Crossing { feature, prev_inside, prev_along, latched } => {
                if !*latched {
                    *latched = match &feature.geom {
                        Geom::Line(l) => l.crossed_by(c.prev_position_ft, c.position_ft),
                        Geom::Area { ring, .. } => {
                            let inside = point_in_polygon(c.position_ft, ring);
                            let edge = prev_inside.is_some_and(|was| was != inside);
                            *prev_inside = Some(inside);
                            edge
                        }
                        Geom::Point(p) => abeam_edge(prev_along, c, *p),
                    };
                }
                *latched
            }
            RCond::Past { feature, latched } => {
                if !*latched {
                    *latched = match &feature.geom {
                        Geom::Line(l) => l.crossed_by(c.prev_position_ft, c.position_ft),
                        // A point already behind the aircraft is past it,
                        // whether or not it was ever ahead.
                        g => along_track_ft(c.position_ft, c.track_deg, g.anchor()) <= 0.0,
                    };
                }
                *latched
            }
            RCond::SideOf { feature, side_deg } => is_on_side(&feature.geom, c.position_ft, *side_deg),
            RCond::Within { feature, distance_ft } => distance_to(&feature.geom, c.position_ft) <= *distance_ft,
            RCond::Inside { feature } => inside(&feature.geom, c.position_ft),
            RCond::Outside { feature } => !inside(&feature.geom, c.position_ft),
            RCond::Over { feature, width_ft } => match &feature.geom {
                Geom::Area { ring, .. } => point_in_polygon(c.position_ft, ring),
                g => distance_to(g, c.position_ft) <= *width_ft,
            },
            RCond::Reaching { altitude_ft, start_above, latched } => {
                let above = c.alt_msl_ft > *altitude_ft;
                if start_above.is_none() {
                    *start_above = Some(above);
                }
                if (c.alt_msl_ft - *altitude_ft).abs() <= REACHING_TOL_FT || *start_above != Some(above) {
                    *latched = true;
                }
                *latched
            }
            RCond::AtOrBelow(a) => c.alt_msl_ft <= *a,
            RCond::AtOrAbove(a) => c.alt_msl_ft >= *a,
            RCond::AglBelow(h) => c.alt_agl_ft < *h,
            RCond::AglAbove(h) => c.alt_agl_ft > *h,
            RCond::DistanceFlown { ft, flown } => {
                *flown += c.position_ft.distance_to(c.prev_position_ft);
                *flown >= *ft
            }
            RCond::Elapsed { s, t } => {
                *t += c.dt;
                *t >= *s
            }
            RCond::OnLeg(leg) => c.leg == Some(*leg),
            RCond::Turning { leg, latched } => {
                if c.leg == Some(*leg) {
                    *latched = true;
                }
                *latched
            }
            RCond::Established(leg) => c.leg == Some(*leg) && c.leg_time_s >= ESTABLISHED_S,
            RCond::AngleBehindWing { feature, deg } => {
                angle_behind_wing_deg(c.position_ft, c.track_deg, feature.geom.anchor()) >= *deg
            }
            RCond::Traffic { desc, id, test } => {
                if id.is_none() {
                    *id = bind(desc, c.traffic, c.position_ft);
                    if id.is_none() {
                        return false;
                    }
                }
                let Some(v) = id.as_ref().and_then(|i| c.traffic.iter().find(|v| &v.target.id == i)) else {
                    // Bound once and gone now: landed and cleared.
                    return *test != TrafficTest::InSight;
                };
                let t = &v.target;
                let behind_ft = -along_track_ft(t.position_ft, t.track_deg, c.position_ft);
                match test {
                    TrafficTest::InSight => c.position_ft.distance_to(t.position_ft) <= VISUAL_RANGE_FT,
                    TrafficTest::Passed => behind_ft >= 0.0,
                    TrafficTest::Behind(d) => behind_ft >= *d,
                    TrafficTest::Following => v.leg == Some(Leg::Final) && behind_ft >= 0.0,
                }
            }
            RCond::All(cs) => cs.iter_mut().fold(true, |acc, k| k.eval(c) & acc),
            RCond::Any(cs) => cs.iter_mut().fold(false, |acc, k| k.eval(c) | acc),
            RCond::Not(k) => !k.eval(c),
        }
    }

    /// Things this condition is waiting on that nothing on board can
    /// make happen: ATC gates not yet opened, unsensed traffic.
    pub fn open_items(&self, gates: &HashSet<String>, out: &mut Vec<String>) {
        match self {
            RCond::Gate(t) if !gates.contains(t.as_str()) => out.push(format!("waiting for ATC: {t}")),
            RCond::Traffic { desc, id: None, .. } => {
                out.push(format!("traffic not identified: {desc} (no matching target sensed)"))
            }
            RCond::All(cs) | RCond::Any(cs) => cs.iter().for_each(|k| k.open_items(gates, out)),
            RCond::Not(k) => k.open_items(gates, out),
            _ => {}
        }
    }
}

/// Abeam `p`: the along-track distance to it went from ahead to behind
/// since the last tick.
fn abeam_edge(prev_along: &mut Option<f64>, c: &EvalCtx, p: Vec2) -> bool {
    let along = along_track_ft(c.position_ft, c.track_deg, p);
    let edge = prev_along.is_some_and(|prev| prev > 0.0 && along <= 0.0);
    *prev_along = Some(along);
    edge
}

pub fn inside(g: &Geom, p: Vec2) -> bool {
    match g {
        Geom::Area { ring, .. } => point_in_polygon(p, ring),
        Geom::Point(q) => p.distance_to(*q) <= AT_RADIUS_FT,
        Geom::Line(l) => l.nearest(p).is_some_and(|n| n.distance_ft <= OVER_LINE_WIDTH_FT),
    }
}

pub fn distance_to(g: &Geom, p: Vec2) -> f64 {
    match g {
        Geom::Point(q) => p.distance_to(*q),
        Geom::Line(l) => l.nearest(p).map(|n| n.distance_ft).unwrap_or(f64::INFINITY),
        Geom::Area { ring, .. } => {
            if point_in_polygon(p, ring) {
                0.0
            } else {
                (0..ring.len())
                    .map(|i| p.distance_to(closest_on_segment(p, ring[i], ring[(i + 1) % ring.len()]).1))
                    .fold(f64::INFINITY, f64::min)
            }
        }
    }
}

/// On the `side_deg` side of a feature: of a line by ray casting, of a
/// point by the half-plane through it, of an area beyond its extent.
pub fn is_on_side(g: &Geom, p: Vec2, side_deg: f64) -> bool {
    let dir = heading_to_vector(side_deg, 1.0);
    match g {
        Geom::Line(l) => l.is_on_side(p, side_deg),
        Geom::Point(q) => (p - *q).dot(dir) > 0.0,
        Geom::Area { ring, .. } => {
            let extent = ring.iter().map(|v| v.dot(dir)).fold(f64::NEG_INFINITY, f64::max);
            p.dot(dir) > extent
        }
    }
}
