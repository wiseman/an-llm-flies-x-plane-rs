//! One `fly(code)` call end to end, minus the profile engage: build the
//! Python context, run the code, compile the delta, and summarize it.
//! The tool layer and the tests share this.

use anyhow::{anyhow, Result};
use serde_json::{json, Value};

use crate::config::ConfigBundle;
use crate::plan::compile::{compile, where_from, CompiledDelta, RPattern, RStepKind};
use crate::plan::cond::RCond;
use crate::plan::gazetteer::Gazetteer;
use crate::plan::python::run_plan_code;
use crate::plan::resolve::{AirportData, AirportSource, Geom, Resolver};
use crate::sim::xplane_bridge::GeoReference;

/// Everything the Python side may look at.
pub fn plan_context(
    apt: &AirportData,
    config: &ConfigBundle,
    gazetteer: &Gazetteer,
    plan_status: Option<&Value>,
    traffic: &[Value],
) -> Value {
    let perf = &config.performance;
    let places: Vec<Value> = gazetteer
        .places
        .iter()
        .map(|p| {
            let [lat, lon] = p.anchor();
            json!({"name": p.name, "kind": p.kind.as_str(), "aliases": p.aliases, "lat": lat, "lon": lon})
        })
        .collect();
    let runways: Vec<Value> = apt
        .runways
        .iter()
        .map(|r| json!({"ident": r.ident, "course_deg": r.course_deg, "length_ft": r.length_ft, "lat": r.lat, "lon": r.lon}))
        .collect();
    json!({
        "airport": apt.ident,
        "airport_latlon": [apt.lat, apt.lon],
        "field_elevation_ft": apt.elevation_ft,
        "runways": runways,
        "places": places,
        "speeds": {
            "Vr": perf.vr_kt, "Vx": perf.vx_kt, "Vy": perf.vy_kt, "Vref": perf.vref_kt, "Vapp": perf.vapp_kt,
            "Vbg": perf.vbg_kt, "Vso": perf.vso_landing_kt,
            "PATTERN_SPEED": perf.downwind_speed_kt, "CRUISE_SPEED": perf.cruise_speed_kt,
            "SLOWEST_PRACTICAL": perf.vapp_kt + 5.0, "BEST_FORWARD": perf.cruise_speed_kt,
        },
        "pattern_defaults": {
            "altitude_agl_ft": config.pattern.altitude_agl_ft,
            "downwind_offset_ft": config.pattern.downwind_offset_ft,
        },
        "plan": plan_status.cloned().unwrap_or(Value::Null),
        // Sensed traffic (the status payload's list), for traffic().
        "traffic": traffic,
        // The runway the pilot last took off from or flew to, for
        // heading(RUNWAY) when no pattern is in the plan.
        "reference_runway": config.airport.runway.id,
        "reference_runway_course_deg": config.airport.runway.id.as_ref().map(|_| config.airport.runway.course_deg),
    })
}

pub struct Prepared {
    pub delta: CompiledDelta,
    pub printed: String,
    /// Human-readable reading of what will be flown, plus warnings.
    pub summary: Vec<String>,
    /// The code asked `can(...)`: preview it, don't fly it.
    pub preview: bool,
}

/// Run `code` and compile what it built. `Err` carries a message for the
/// agent (plan errors, unknown places, Python failures).
#[allow(clippy::too_many_arguments)]
pub fn prepare(
    code: &str,
    airport_ident: &str,
    config: &ConfigBundle,
    gazetteer: &Gazetteer,
    airports: &dyn AirportSource,
    georef: GeoReference,
    plan_status: Option<&Value>,
) -> Result<Prepared> {
    prepare_with_traffic(code, airport_ident, config, gazetteer, airports, georef, plan_status, &[])
}

/// `prepare`, with the sensed traffic the plan code can name.
#[allow(clippy::too_many_arguments)]
pub fn prepare_with_traffic(
    code: &str,
    airport_ident: &str,
    config: &ConfigBundle,
    gazetteer: &Gazetteer,
    airports: &dyn AirportSource,
    georef: GeoReference,
    plan_status: Option<&Value>,
    traffic: &[Value],
) -> Result<Prepared> {
    let apt = airports.airport(airport_ident)?;
    let context = plan_context(&apt, config, gazetteer, plan_status, traffic);
    let outcome = run_plan_code(code, &context)?;
    if !outcome.ok {
        let mut msg = outcome.error.unwrap_or_else(|| "plan code failed".into());
        if !outcome.printed.is_empty() {
            msg.push_str(&format!("\nprinted before the error:\n{}", outcome.printed.trim_end()));
        }
        return Err(anyhow!(msg));
    }
    let delta = outcome.delta.ok_or_else(|| anyhow!("plan code produced no result"))?;
    let preview = delta.preview;
    let mut resolver = Resolver::new(georef, gazetteer, airports);
    let compiled = compile(&delta, &mut resolver, config)?;
    let summary = summarize(&compiled);
    Ok(Prepared { delta: compiled, printed: outcome.printed, summary, preview })
}

/// What will be flown, in words, with the checks a controller would
/// expect a pilot to make ("that trigger is behind you").
pub fn summarize(d: &CompiledDelta) -> Vec<String> {
    let mut out = Vec::new();
    if d.replace {
        out.push("new clearance: earlier constraints and reports dropped".into());
    }
    if let Some(steps) = &d.steps {
        if steps.is_empty() {
            out.push("steps: none (holding present track and altitude)".into());
        }
        for (i, s) in steps.iter().enumerate() {
            out.push(format!("step {}: {}", i + 1, s.text));
            if let RStepKind::Pattern { pattern, .. } = &s.kind {
                out.extend(pattern_notes(pattern));
            }
        }
    }
    for s in &d.directives {
        out.push(format!("now: {}", s.text));
    }
    if let Some(p) = &d.pattern {
        out.push(format!("pattern: {}", p.describe()));
        out.extend(pattern_notes(p));
    }
    for c in &d.constraints {
        out.push(format!("{}: {}", c.id, c.text));
    }
    for r in &d.reports {
        out.push(format!("{}: report {:?}", r.id, r.text));
    }
    for p in &d.permits {
        out.push(format!("permitted: {p}"));
    }
    if d.go_around {
        out.push("go around now".into());
    }
    for g in &d.release {
        out.push(format!("released: {g}"));
    }
    for c in &d.cancel {
        out.push(format!("cancel: {c}"));
    }
    if out.is_empty() {
        out.push("nothing to change".into());
    }
    out
}

/// Where the pattern's turn triggers sit relative to the runway, and
/// whether the pattern's geometry can meet them.
fn pattern_notes(p: &RPattern) -> Vec<String> {
    let mut out = Vec::new();
    for (leg, t) in &p.triggers {
        for c in [&t.when, &t.earliest, &t.latest].into_iter().flatten() {
            if let Some(note) = locate(c, p, *leg) {
                out.push(format!("  turn {}: {}", leg.as_str(), note));
            }
        }
    }
    out.sort();
    out
}

fn locate(c: &RCond, p: &RPattern, leg: crate::plan::ir::Leg) -> Option<String> {
    let f = match c {
        RCond::At { feature, .. }
        | RCond::Abeam { feature, .. }
        | RCond::Crossing { feature, .. }
        | RCond::Past { feature, .. } => feature,
        _ => return None,
    };
    // The pattern's own runway (its threshold, midfield): where it is
    // relative to that runway says nothing.
    if f.label.starts_with(&format!("{} {} ", p.spec.airport, p.spec.runway)) {
        return None;
    }
    let frame = &p.frame;
    match &f.geom {
        Geom::Line(l) if leg == crate::plan::ir::Leg::Base => {
            // Where the downwind track meets the line, if it does.
            let side = p.frame.runway.traffic_side.sign();
            let dw_y = side * (p.config.pattern.downwind_offset_ft);
            let a = frame.to_world_frame(crate::types::Vec2::new(30_000.0, dw_y));
            let b = frame.to_world_frame(crate::types::Vec2::new(-60_000.0, dw_y));
            let track = crate::guidance::geo::Polyline::new(vec![a, b]);
            match crate::plan::resolve::polyline_intersection(&track, l) {
                Some(x) => Some(format!("{} meets the downwind track {}", f.label, where_from(frame, x))),
                None => Some(format!("warning: {} doesn't cross the downwind track within 10 nm", f.label)),
            }
        }
        Geom::Line(_) => None,
        g => Some(format!("{} is {}", f.label, where_from(frame, g.anchor()))),
    }
}
