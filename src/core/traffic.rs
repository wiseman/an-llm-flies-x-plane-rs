//! Other aircraft, as the pilot knows them: a shared picture of targets
//! (scripted in evals, empty live until a sensor feeds it), where each one
//! is relative to a runway's pattern, and how a controller's description
//! ("the Cherokee on a 3 mile final") picks one out.

use std::sync::Arc;

use parking_lot::RwLock;
use serde::Serialize;

use crate::guidance::runway_geometry::RunwayFrame;
use crate::plan::ir::Leg;
use crate::types::{vector_to_heading, wrap_degrees_180, wrap_degrees_360, Vec2, NM_TO_FT};

/// How far off traffic counts as seen ("report traffic in sight").
pub const VISUAL_RANGE_FT: f64 = 3.0 * NM_TO_FT;
/// Closer than this horizontally to airborne traffic, within
/// `SEPARATION_VERTICAL_FT` of its altitude, is too close (the plan
/// preview's warning and the eval's separation check).
pub const SEPARATION_FT: f64 = 2500.0;
pub const SEPARATION_VERTICAL_FT: f64 = 400.0;
/// Targets further than this aren't listed or bound.
const LIST_RANGE_FT: f64 = 12.0 * NM_TO_FT;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct TrafficTarget {
    pub id: String,
    /// Aircraft type as a controller would say it ("Cherokee", "Falcon").
    pub kind: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub callsign: Option<String>,
    /// World frame, feet.
    #[serde(skip)]
    pub position_ft: Vec2,
    pub alt_msl_ft: f64,
    pub track_deg: f64,
    pub gs_kt: f64,
    pub on_ground: bool,
}

/// The current targets, shared between whatever senses them and the
/// profiles and tools that read them.
#[derive(Debug, Clone, Default)]
pub struct TrafficPicture(Arc<RwLock<Vec<TrafficTarget>>>);

impl TrafficPicture {
    pub fn set(&self, targets: Vec<TrafficTarget>) {
        *self.0.write() = targets;
    }

    pub fn snapshot(&self) -> Vec<TrafficTarget> {
        self.0.read().clone()
    }
}

/// A target placed against a runway's pattern.
#[derive(Debug, Clone)]
pub struct TrafficView {
    pub target: TrafficTarget,
    pub leg: Option<Leg>,
    /// Miles from the threshold while on final (0 on the runway).
    pub final_nm: Option<f64>,
    /// "on a 2.1 mile final 16R", "on right downwind 16R", "".
    pub where_: String,
}

/// Where `t` is in `frame`'s pattern, by position and track.
pub fn view(frame: &RunwayFrame, t: &TrafficTarget) -> TrafficView {
    let rw = &frame.runway;
    let p = frame.to_runway_frame(t.position_ft);
    let rel = wrap_degrees_180(t.track_deg - rw.course_deg);
    let side = |y: f64| if y > 0.0 { "right" } else { "left" };
    let id = rw.id.clone().unwrap_or_default();
    let (leg, final_nm, where_) = if t.on_ground {
        if p.y.abs() < 300.0 && p.x > -500.0 && p.x < rw.length_ft + 500.0 {
            (Some(Leg::Final), Some(0.0), format!("on runway {id}"))
        } else {
            (None, None, "on the ground".to_string())
        }
    } else if p.x < rw.length_ft && p.y.abs() < 1500.0_f64.max(0.15 * p.x.abs()) && rel.abs() < 30.0 {
        let nm = (-p.x).max(0.0) / NM_TO_FT;
        let w = if nm < 0.8 { format!("on short final {id}") } else { format!("on a {nm:.1} mile final {id}") };
        (Some(Leg::Final), Some(nm), w)
    } else if p.x >= 0.0 && p.y.abs() < 2500.0 && rel.abs() < 30.0 {
        (Some(Leg::Upwind), None, format!("departing {id}"))
    } else if rel.abs() > 150.0 && p.y.abs() > 1500.0 && p.y.abs() < 12000.0 {
        (Some(Leg::Downwind), None, format!("on {} downwind {id}", side(p.y)))
    } else if (rel.abs() - 90.0).abs() < 40.0 && p.y.abs() < 12000.0 {
        // Heading toward the centerline before the threshold: base;
        // away from it past the departure end: crosswind.
        let toward = (p.y > 0.0) == (rel < 0.0);
        if toward && p.x < 0.0 {
            (Some(Leg::Base), None, format!("on {} base {id}", side(p.y)))
        } else if !toward && p.x > rw.length_ft * 0.5 {
            (Some(Leg::Crosswind), None, format!("on {} crosswind {id}", side(p.y)))
        } else {
            (None, None, String::new())
        }
    } else {
        (None, None, String::new())
    };
    TrafficView { target: t.clone(), leg, final_nm, where_ }
}

/// "2 o'clock, 1.4 nm, 300 ft below".
pub fn relative(own_pos: Vec2, own_track_deg: f64, own_alt_ft: f64, t: &TrafficTarget) -> String {
    let d = t.position_ft - own_pos;
    let clock = ((wrap_degrees_360(vector_to_heading(d) - own_track_deg) / 30.0).round() as i64).rem_euclid(12);
    let clock = if clock == 0 { 12 } else { clock };
    let dalt = t.alt_msl_ft - own_alt_ft;
    let alt = if dalt.abs() < 150.0 {
        "same altitude".to_string()
    } else {
        format!("{:.0} ft {}", (dalt.abs() / 100.0).round() * 100.0, if dalt > 0.0 { "above" } else { "below" })
    };
    format!("{clock} o'clock, {:.1} nm, {alt}", d.length() / NM_TO_FT)
}

pub fn in_list_range(own_pos: Vec2, t: &TrafficTarget) -> bool {
    own_pos.distance_to(t.position_ft) <= LIST_RANGE_FT
}

/// Aircraft types controllers name; a description naming one of these
/// can't mean a target of another type.
const TYPE_WORDS: [&str; 34] = [
    "cessna", "skyhawk", "skylane", "cardinal", "caravan", "citation", "piper", "cherokee", "archer", "warrior",
    "seneca", "aztec", "mooney", "bonanza", "baron", "king air", "beech", "cirrus", "diamond", "falcon", "gulfstream",
    "learjet", "lear", "challenger", "hawker", "phenom", "embraer", "pilatus", "robinson", "helicopter", "jet",
    "twin", "decathlon", "citabria",
];

const LEG_WORDS: [(&str, Leg); 5] = [
    ("final", Leg::Final),
    ("base", Leg::Base),
    ("downwind", Leg::Downwind),
    ("upwind", Leg::Upwind),
    ("crosswind", Leg::Crosswind),
];

/// The target a controller's description most likely means, by id, type
/// or callsign, leg, and miles on final. A description that names another
/// target's type never binds to this one. With nothing distinctive in the
/// description, the only target nearby is it.
pub fn bind(desc: &str, views: &[TrafficView], own_pos: Vec2) -> Option<String> {
    let d = desc.to_lowercase();
    let words: Vec<&str> = d.split(|c: char| !c.is_alphanumeric() && c != '.').filter(|w| !w.is_empty()).collect();
    if let Some(v) = views.iter().find(|v| v.target.id.to_lowercase() == d) {
        return Some(v.target.id.clone());
    }
    let near: Vec<&TrafficView> = views.iter().filter(|v| in_list_range(own_pos, &v.target)).collect();
    let names = |v: &TrafficView| {
        let mut n = vec![v.target.kind.to_lowercase()];
        if let Some(c) = &v.target.callsign {
            n.push(c.to_lowercase());
        }
        n
    };
    let named_any = near.iter().any(|v| names(v).iter().any(|n| d.contains(n.as_str())))
        || TYPE_WORDS.iter().any(|w| words.contains(w) || (w.contains(' ') && d.contains(w)));
    let said_leg = LEG_WORDS.iter().find(|(w, _)| words.contains(w)).map(|(_, l)| *l);
    let said_miles = words.windows(2).find_map(|w| {
        (w[1].starts_with("mile") || w[1] == "nm").then(|| w[0].parse::<f64>().ok()).flatten()
    });
    let mut best: Option<(f64, &TrafficView)> = None;
    for v in &near {
        let mine = names(v).iter().any(|n| d.contains(n.as_str()));
        if named_any && !mine {
            continue;
        }
        let mut score = if mine { 3.0 } else { 0.0 };
        match (said_leg, v.leg) {
            (Some(l), Some(vl)) if l == vl => score += 2.0,
            (Some(_), _) if !mine => score -= 1.0,
            _ => {}
        }
        if let (Some(m), Some(f)) = (said_miles, v.final_nm) {
            score += if (m - f).abs() <= 1.5 { 2.0 } else { -2.0 };
        }
        if near.len() == 1 {
            score += 0.5;
        }
        if score > 0.0 && best.is_none_or(|(s, _)| score > s) {
            best = Some((score, v));
        }
    }
    best.map(|(_, v)| v.target.id.clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{heading_to_vector, Runway, TrafficSide};

    fn frame() -> RunwayFrame {
        RunwayFrame::new(Runway {
            id: Some("16R".into()),
            threshold_ft: Vec2::ZERO,
            course_deg: 180.0,
            length_ft: 8000.0,
            touchdown_zone_ft: 1000.0,
            displaced_threshold_ft: 0.0,
            width_ft: 75.0,
            traffic_side: TrafficSide::Right,
        })
    }

    fn target(id: &str, kind: &str, rf: Vec2, track: f64) -> TrafficTarget {
        let f = frame();
        TrafficTarget {
            id: id.into(),
            kind: kind.into(),
            callsign: None,
            position_ft: f.to_world_frame(rf),
            alt_msl_ft: 1500.0,
            track_deg: track,
            gs_kt: 80.0,
            on_ground: false,
        }
    }

    #[test]
    fn places_targets_in_the_pattern_and_binds_descriptions() {
        let f = frame();
        let views: Vec<TrafficView> = [
            target("t1", "Cherokee", Vec2::new(-3.0 * NM_TO_FT, 0.0), 180.0),
            target("t2", "Cessna", Vec2::new(-6000.0, 6000.0), 90.0),
            target("t3", "Mooney", Vec2::new(4000.0, 6000.0), 0.0),
        ]
        .iter()
        .map(|t| view(&f, t))
        .collect();
        assert_eq!(views[0].leg, Some(Leg::Final));
        assert!(views[0].where_.contains("3.0 mile final"), "{}", views[0].where_);
        assert_eq!(views[1].leg, Some(Leg::Base), "{}", views[1].where_);
        assert_eq!(views[2].leg, Some(Leg::Downwind), "{}", views[2].where_);

        let me = f.to_world_frame(Vec2::new(2000.0, 6000.0));
        assert_eq!(bind("Cherokee on a 3 mile final", &views, me).as_deref(), Some("t1"));
        assert_eq!(bind("the traffic on base", &views, me).as_deref(), Some("t2"));
        assert_eq!(bind("Cessna ahead on base", &views, me).as_deref(), Some("t2"));
        assert_eq!(bind("t3", &views, me).as_deref(), Some("t3"));
        // Names a type nobody is: unbound.
        assert_eq!(bind("the Falcon on a 6 mile final", &views, me), None);
        let one = vec![views[0].clone()];
        assert_eq!(bind("that traffic", &one, me).as_deref(), Some("t1"));
    }

    #[test]
    fn relative_position_reads_like_a_traffic_call() {
        let t = TrafficTarget {
            id: "t".into(),
            kind: "Cessna".into(),
            callsign: None,
            position_ft: heading_to_vector(60.0, NM_TO_FT * 2.0),
            alt_msl_ft: 1200.0,
            track_deg: 0.0,
            gs_kt: 90.0,
            on_ground: false,
        };
        assert_eq!(relative(Vec2::ZERO, 0.0, 1500.0, &t), "2 o'clock, 2.0 nm, 300 ft below");
    }
}
