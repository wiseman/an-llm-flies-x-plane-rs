//! Fly a plan ahead before committing to it: "can you make a short
//! approach?", "will I be behind the Cherokee?". A copy of the plan
//! profile flies the simple aircraft model from the current state while
//! sensed traffic moves on (straight ahead, or down final to a landing),
//! and the run comes back as a short timeline and a list of problems.
//! The simple model isn't X-Plane, and there's no wind: the preview is
//! for "does this plan make sense", not exact numbers.

use crate::config::ConfigBundle;
use crate::core::mission_manager::PilotCore;
use crate::core::plan_profile::PlanProfile;
use crate::core::profiles::LOW_APPROACH_REASON;
use crate::core::traffic::{view, TrafficPicture, TrafficTarget, SEPARATION_FT, SEPARATION_VERTICAL_FT};
use crate::guidance::runway_geometry::RunwayFrame;
use crate::plan::ir::Leg;
use crate::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use crate::types::{heading_to_vector, AircraftState, FlightPhase, Vec2, KT_TO_FPS, NM_TO_FT};

const DT: f64 = 0.2;
/// Both on final closer than this (in trail, or cutting in ahead) is a
/// problem.
const FINAL_SPACING_FT: f64 = 0.75 * NM_TO_FT;

#[derive(Debug, Clone, Default)]
pub struct Preview {
    /// What happened, in order ("+1:25 base, 1.3 nm before the 16R threshold").
    pub timeline: Vec<String>,
    /// What a pilot would object to (closer than 0.4 nm to traffic, a
    /// go-around, landing long). Empty: the plan looks flyable.
    pub problems: Vec<String>,
    /// Sim seconds flown.
    pub flown_s: f64,
}

impl Preview {
    pub fn lines(&self) -> Vec<String> {
        let mut out = vec![format!(
            "preview ({}, simple model, no wind):",
            fmt_t(self.flown_s)
        )];
        out.extend(self.timeline.iter().map(|l| format!("  {l}")));
        if self.problems.is_empty() {
            out.push("  verdict: looks flyable".into());
        } else {
            out.push("  problems:".into());
            out.extend(self.problems.iter().map(|p| format!("    - {p}")));
        }
        out
    }
}

fn fmt_t(t: f64) -> String {
    let s = t.max(0.0).round() as i64;
    format!("+{}:{:02}", s / 60, s % 60)
}

/// Where `p` is against `frame`'s runway: "1.3 nm before the 16R
/// threshold, 1.1 nm right of centerline".
fn where_rf(frame: &RunwayFrame, p: Vec2) -> String {
    let rf = frame.to_runway_frame(p);
    let id = frame.runway.id.clone().unwrap_or_default();
    let along = if rf.x < 0.0 {
        format!("{:.1} nm before the {id} threshold", -rf.x / NM_TO_FT)
    } else if rf.x <= frame.runway.length_ft {
        format!("{:.0} ft down {id}", rf.x)
    } else {
        format!("{:.1} nm past the {id} departure end", (rf.x - frame.runway.length_ft) / NM_TO_FT)
    };
    if rf.y.abs() < 300.0 {
        along
    } else {
        format!("{along}, {:.1} nm {} of centerline", rf.y.abs() / NM_TO_FT, if rf.y > 0.0 { "right" } else { "left" })
    }
}

/// Moves sensed traffic on from where it is now: down final to a stop
/// on the runway (then gone after 30 s), otherwise straight ahead.
struct TrafficMover {
    start: Vec<TrafficTarget>,
    frame: RunwayFrame,
    field_elevation_ft: f64,
}

impl TrafficMover {
    fn at(&self, t: f64) -> Vec<TrafficTarget> {
        let mut out = Vec::new();
        for s in &self.start {
            let v = view(&self.frame, s);
            let speed = s.gs_kt * KT_TO_FPS;
            let mut x = s.clone();
            if v.leg == Some(Leg::Final) {
                // Lands about 1000 ft past the threshold and rolls 2000 ft.
                let rf = self.frame.to_runway_frame(s.position_ft);
                let to_td = (1000.0 - rf.x).max(0.0);
                let flown = speed * t;
                if s.on_ground || flown >= to_td {
                    let t_td = if speed > 1.0 { to_td / speed } else { 0.0 };
                    let rolled = (flown - to_td).max(0.0).min(2000.0);
                    if t - t_td > 30.0 + 2000.0 / speed.max(1.0) {
                        continue;
                    }
                    x.position_ft = self.frame.to_world_frame(Vec2::new(rf.x.max(1000.0) + rolled, 0.0));
                    x.alt_msl_ft = self.field_elevation_ft;
                    x.on_ground = true;
                } else {
                    let f = flown / to_td.max(1.0);
                    x.position_ft = s.position_ft + heading_to_vector(s.track_deg, flown);
                    x.alt_msl_ft = s.alt_msl_ft + (self.field_elevation_ft - s.alt_msl_ft) * f;
                }
            } else {
                x.position_ft = s.position_ft + heading_to_vector(s.track_deg, speed * t);
            }
            out.push(x);
        }
        out
    }
}

/// Fly `profile` (the plan with the change applied) from `state` for up
/// to `horizon_s`, or until it lands and stops or completes.
pub fn preview(
    config: &ConfigBundle,
    mut profile: PlanProfile,
    state: &AircraftState,
    traffic: Vec<TrafficTarget>,
    horizon_s: f64,
) -> Preview {
    let frame = RunwayFrame::new(config.airport.runway.clone());
    let field = config.airport.field_elevation_ft;
    let mover = TrafficMover { start: traffic, frame: frame.clone(), field_elevation_ft: field };
    let picture = TrafficPicture::default();
    picture.set(mover.at(0.0));
    profile = profile.with_traffic(picture.clone());

    let model = SimpleAircraftModel::new(config.clone(), Vec2::ZERO);
    let mut pilot = PilotCore::new(config.clone());
    pilot.traffic = picture.clone();
    let mut raw = DynamicsState {
        position_ft: state.position_ft,
        altitude_ft: state.alt_msl_ft,
        agl_ft: None,
        heading_deg: state.heading_deg,
        roll_deg: state.roll_deg,
        pitch_deg: state.pitch_deg,
        ias_kt: state.ias_kt,
        throttle_pos: state.throttle_pos,
        on_ground: state.on_ground,
        time_s: 0.0,
        p_rad_s: state.p_rad_s,
        q_rad_s: state.q_rad_s,
        r_rad_s: state.r_rad_s,
        ground_velocity_ft_s: state.ground_velocity_ft_s,
        vertical_speed_ft_s: state.vs_fpm / 60.0,
        flap_index: state.flap_index,
        gear_down: state.gear_down,
        sideslip_deg: 0.0,
        parking_brake: 0.0,
    };
    let _ = pilot.update(&raw, DT);
    let mut out = Preview::default();
    if let Err(e) = pilot.engage_profile(Box::new(profile)) {
        out.problems.push(format!("couldn't start the preview: {e:?}"));
        return out;
    }

    let mut last_phase: Option<FlightPhase> = None;
    let mut seen_events = 0u64;
    let mut was_airborne = !state.on_ground;
    let mut closest: Option<(f64, f64, String)> = None;
    let mut closest_on_final: Option<(f64, f64, String, bool)> = None;
    let mut traffic_landed: std::collections::HashSet<String> =
        mover.at(0.0).iter().filter(|x| x.on_ground).map(|x| x.id.clone()).collect();
    // (time, who) of each target's touchdown during the preview.
    let mut traffic_touchdowns: Vec<(f64, String)> = Vec::new();
    let mut went_around = false;
    // Hard constraints broken at some point (each reported once).
    let mut violated: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut landed_at: Option<f64> = None;
    let mut complete_at: Option<f64> = None;
    while raw.time_s < horizon_s {
        picture.set(mover.at(raw.time_s));
        let (st, cmds) = pilot.update(&raw, DT);
        model.step(&mut raw, &cmds, DT);
        let t = raw.time_s;

        let phase = pilot.phase();
        if last_phase != Some(phase) {
            let shown = !matches!(phase, FlightPhase::Preflight | FlightPhase::Roundout | FlightPhase::Flare);
            if shown {
                out.timeline.push(format!("{} {}, {}", fmt_t(t), phase.value(), where_rf(&frame, st.position_ft)));
            }
            if phase == FlightPhase::GoAround {
                // A low approach's planned go-around isn't a problem.
                let reason = pilot.pattern_fly_mut().and_then(|pf| pf.last_go_around_reason.clone());
                went_around |= reason.as_deref() != Some(LOW_APPROACH_REASON);
            }
            last_phase = Some(phase);
        }
        if let Some(snap) = &pilot.latest_snapshot {
            for (seq, text) in &snap.events {
                if *seq > seen_events {
                    seen_events = *seq;
                    if text.starts_with("done:") || text.contains("touchdown") {
                        continue;
                    }
                    out.timeline.push(format!("{} {text}", fmt_t(t)));
                }
            }
        }
        if let Some(plan) = pilot.profile_mut::<PlanProfile>() {
            for c in plan.violated_constraints() {
                if violated.insert(c.id.clone()) {
                    out.problems.push(format!("{} constraint {} not met: {}", fmt_t(t), c.id, c.text));
                }
            }
        }
        let now = picture.snapshot();
        note_traffic_landings(&now, t, &mut traffic_landed, &mut traffic_touchdowns, &mut out);
        if !raw.on_ground && matches!(phase, FlightPhase::Final) {
            for x in now.iter().filter(|x| !x.on_ground && view(&frame, x).leg == Some(Leg::Final)) {
                let d = x.position_ft.distance_to(raw.position_ft);
                // Ahead of us if it's closer to the threshold.
                let ahead = frame.to_runway_frame(x.position_ft).x > frame.to_runway_frame(raw.position_ft).x;
                if closest_on_final.as_ref().is_none_or(|c| d < c.1) {
                    closest_on_final = Some((t, d, format!("{} {}", x.id, x.kind), ahead));
                }
            }
        }
        if !raw.on_ground {
            for x in now.iter().filter(|x| !x.on_ground) {
                if (x.alt_msl_ft - raw.altitude_ft).abs() < SEPARATION_VERTICAL_FT {
                    let d = x.position_ft.distance_to(raw.position_ft);
                    if closest.as_ref().is_none_or(|c| d < c.1) {
                        closest = Some((t, d, format!("{} {}", x.id, x.kind)));
                    }
                }
            }
        }
        if was_airborne && raw.on_ground && landed_at.is_none() {
            landed_at = Some(t);
            let rf = frame.to_runway_frame(raw.position_ft);
            let rw = &frame.runway;
            let past = rf.x - rw.displaced_threshold_ft;
            out.timeline.push(format!(
                "{} touchdown {:.0} ft past the {} threshold, {:.0} ft off centerline",
                fmt_t(t),
                past,
                rw.id.clone().unwrap_or_default(),
                rf.y
            ));
            if !frame.touchdown_in_zone(rf) {
                out.problems.push(format!(
                    "touches down {:.0} ft past the threshold ({:.0} ft off centerline): outside the first third / off the runway",
                    past, rf.y
                ));
            }
        }
        was_airborne = !raw.on_ground;
        if landed_at.is_some() && st.gs_kt < 5.0 {
            break;
        }
        let done = pilot.profile_mut::<PlanProfile>().is_none_or(|p| p.is_finished());
        if done && complete_at.is_none() && landed_at.is_none() {
            complete_at = Some(t);
        }
        if complete_at.is_some_and(|c| t - c > 30.0) {
            break;
        }
        if raw.altitude_ft < field - 5.0 {
            out.problems.push(format!("{} below the field elevation (crash)", fmt_t(t)));
            break;
        }
    }
    out.flown_s = raw.time_s;
    // Keep the traffic moving after you've stopped, to see who lands
    // behind you.
    if let Some(td) = landed_at {
        let mut t = raw.time_s;
        while t < td + 90.0 {
            t += 1.0;
            note_traffic_landings(&mover.at(t), t, &mut traffic_landed, &mut traffic_touchdowns, &mut out);
        }
    }
    if went_around {
        out.problems.push("goes around".into());
    }
    if let Some((t, d, who)) = closest {
        let line = format!("closest traffic: {:.1} nm from {who} at {}", d / NM_TO_FT, fmt_t(t));
        if d < SEPARATION_FT {
            out.problems.push(line);
        } else {
            out.timeline.push(line);
        }
    }
    // Landing order on the runway: someone landing right behind you
    // means you cut in; landing right after someone means the runway may
    // not be clear yet.
    if let Some(td) = landed_at {
        for (tt, who) in &traffic_touchdowns {
            let gap = tt - td;
            if (0.0..90.0).contains(&gap) {
                out.problems.push(format!("lands {gap:.0} s ahead of {who}, which lands behind you: cutting in"));
            } else if (-30.0..0.0).contains(&gap) {
                out.problems.push(format!("lands only {:.0} s after {who}; the runway may not be clear", -gap));
            }
        }
    }
    if let Some((t, d, who, ahead)) = closest_on_final {
        if d < FINAL_SPACING_FT {
            out.problems.push(format!(
                "on final {:.1} nm {} {who} at {}",
                d / NM_TO_FT,
                if ahead { "behind" } else { "ahead of (cutting in front of)" },
                fmt_t(t)
            ));
        }
    }
    if landed_at.is_none() && complete_at.is_none() {
        let open = pilot
            .latest_snapshot
            .as_ref()
            .and_then(|s| s.plan_status.as_ref())
            .and_then(|p| p.get("open_items"))
            .and_then(|v| v.as_array())
            .map(|a| a.iter().filter_map(|v| v.as_str()).collect::<Vec<_>>().join("; "))
            .unwrap_or_default();
        out.timeline.push(if open.is_empty() {
            format!("still flying the plan at {}", fmt_t(raw.time_s))
        } else {
            format!("still flying at {}, waiting on: {open} (the preview never opens ATC gates)", fmt_t(raw.time_s))
        });
    }
    out
}

/// Note each target's first time on the ground: in the timeline and as
/// (time, who) for the landing-order check.
fn note_traffic_landings(
    targets: &[TrafficTarget],
    t: f64,
    landed: &mut std::collections::HashSet<String>,
    touchdowns: &mut Vec<(f64, String)>,
    out: &mut Preview,
) {
    for x in targets.iter().filter(|x| x.on_ground) {
        if landed.insert(x.id.clone()) {
            out.timeline.push(format!("{} {} {} lands", fmt_t(t), x.id, x.kind));
            touchdowns.push((t, format!("{} {}", x.id, x.kind)));
        }
    }
}
