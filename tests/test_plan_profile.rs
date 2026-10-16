//! Plans flown in the simple sim, no LLM: Python plan code -> compiled
//! plan -> `PlanProfile` -> `SimpleAircraftModel`, at KVNY with the
//! hardcoded place list. Asserts where turns happen relative to the named
//! features and that constraints hold.

use xplane_pilot::config::{load_default_config_bundle, ConfigBundle};
use xplane_pilot::core::mission_manager::PilotCore;
use xplane_pilot::core::plan_profile::PlanProfile;
use xplane_pilot::guidance::geo::Polyline;
use xplane_pilot::guidance::runway_geometry::RunwayFrame;
use xplane_pilot::plan::fixtures::{kvny_georef, KvnyFixture};
use xplane_pilot::plan::gazetteer::GAZETTEER;
use xplane_pilot::plan::ir::Ref;
use xplane_pilot::plan::resolve::{AirportSource, Geom, Resolver};
use xplane_pilot::eval_scenario::{ScenarioTraffic, ScriptedTraffic};
use xplane_pilot::plan::session::prepare;
use xplane_pilot::sim::simple_dynamics::{DynamicsState, SimpleAircraftModel};
use xplane_pilot::types::{heading_to_vector, AircraftState, FlightPhase, Vec2, KT_TO_FPS, NM_TO_FT};

const DT: f64 = 0.2;

thread_local! {
    /// Wind (the way the air moves, kt) for the next `fly`.
    static WIND_KT: std::cell::Cell<Vec2> = const { std::cell::Cell::new(Vec2::ZERO) };
}

/// An amendment sent mid-flight when `when` first holds.
struct Amend {
    when: Box<dyn Fn(&AircraftState, FlightPhase, f64) -> bool>,
    code: &'static str,
    sent_at: Option<f64>,
}

#[derive(Default)]
struct Flight {
    /// (t, phase, world position, alt msl)
    samples: Vec<(f64, FlightPhase, Vec2, f64)>,
    /// (t, agl, bank, track, on_ground)
    extra: Vec<(f64, f64, f64, f64, bool)>,
    events: Vec<String>,
    touchdown_rf: Option<Vec2>,
    amend_times: Vec<f64>,
}

impl Flight {
    /// First sample in `phase` after being in `before`.
    fn transition(&self, before: FlightPhase, phase: FlightPhase) -> Option<(f64, Vec2, f64)> {
        self.samples
            .windows(2)
            .find(|w| w[0].1 == before && w[1].1 == phase)
            .map(|w| (w[1].0, w[1].2, w[1].3))
    }

    fn phases(&self) -> Vec<FlightPhase> {
        let mut out: Vec<FlightPhase> = Vec::new();
        for s in &self.samples {
            if out.last() != Some(&s.1) {
                out.push(s.1);
            }
        }
        out
    }
}

fn line(name: &str) -> Polyline {
    let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &KvnyFixture);
    match r.resolve(&Ref::Place { name: name.into() }).unwrap().geom {
        Geom::Line(l) => l,
        g => panic!("{name} is a {}", g.kind()),
    }
}

fn prep(code: &str, base: &ConfigBundle, status: Option<&serde_json::Value>) -> xplane_pilot::plan::session::Prepared {
    prepare(code, "KVNY", base, &GAZETTEER, &KvnyFixture, kvny_georef(), status)
        .unwrap_or_else(|e| panic!("plan code failed: {e:#}"))
}

/// Fly `code` from an airborne start given in the 16R runway frame.
fn fly(code: &str, start_rf: Vec2, heading_deg: f64, alt_msl_ft: f64, ias_kt: f64, amends: Vec<Amend>, max_t: f64) -> Flight {
    fly_from(code, start_rf, heading_deg, alt_msl_ft, ias_kt, false, amends, max_t, None)
}

/// `fly`, optionally starting on the ground (lined up, stopped).
#[allow(clippy::too_many_arguments)]
fn fly_from(
    code: &str,
    start_rf: Vec2,
    heading_deg: f64,
    alt_msl_ft: f64,
    ias_kt: f64,
    on_ground: bool,
    mut amends: Vec<Amend>,
    max_t: f64,
    traffic: Option<&ScriptedTraffic>,
) -> Flight {
    let base = load_default_config_bundle();
    // The 16R frame, from a throwaway pattern, anchors the start position.
    let frame_cfg = prep("p = pattern(apt.runway('16R'), RIGHT)\njoin(p)", &base, None)
        .delta
        .reference_pattern()
        .unwrap()
        .config
        .clone();
    let prepared = prep(code, &base, None);
    for line in &prepared.summary {
        eprintln!("plan: {line}");
    }
    let config = prepared.delta.reference_pattern().map(|p| p.config.clone()).unwrap_or(frame_cfg.clone());
    let frame = RunwayFrame::new(frame_cfg.airport.runway.clone());
    let model = SimpleAircraftModel::new(config.clone(), WIND_KT.with(|w| w.get()));
    let mut pilot = PilotCore::new(config.clone());
    let mut raw = DynamicsState {
        position_ft: frame.to_world_frame(start_rf),
        altitude_ft: alt_msl_ft,
        agl_ft: None,
        heading_deg,
        roll_deg: 0.0,
        pitch_deg: 2.0,
        ias_kt,
        throttle_pos: if on_ground { 0.0 } else { 0.5 },
        on_ground,
        time_s: 0.0,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ground_velocity_ft_s: heading_to_vector(heading_deg, ias_kt * KT_TO_FPS),
        vertical_speed_ft_s: 0.0,
        flap_index: 0,
        gear_down: true,
        sideslip_deg: 0.0,
        parking_brake: 0.0,
    };
    let (est, _) = pilot.update(&raw, DT);
    if let Some(t) = traffic {
        pilot.traffic.set(t.at(0.0));
    }
    let mut profile = PlanProfile::new(base.clone(), &est).with_traffic(pilot.traffic.clone());
    profile.apply(prepared.delta, &est);
    pilot.engage_profile(Box::new(profile)).unwrap();

    let mut f = Flight::default();
    let mut was_airborne = !on_ground;
    let mut seen_events = 0u64;
    while raw.time_s < max_t {
        if let Some(t) = traffic {
            pilot.traffic.set(t.at(raw.time_s));
        }
        let (state, cmds) = pilot.update(&raw, DT);
        model.step(&mut raw, &cmds, DT);
        let phase = pilot.phase();
        if std::env::var("PLAN_TRACE").is_ok() && (raw.time_s % 5.0) < DT {
            let rf = frame.to_runway_frame(state.position_ft);
            let g = pilot.latest_snapshot.as_ref().and_then(|s| s.last_guidance.clone());
            eprintln!(
                "t={:5.0} {:?} x={:7.0} y={:6.0} agl={:5.0} ias={:4.1} vs={:5.0} thr={:.2} pitch={:4.1} bank={:5.1} slip={:.2} tgt_alt={:?} tgt_spd={:?} mode={:?}",
                raw.time_s, phase, rf.x, rf.y, state.alt_agl_ft, state.ias_kt, state.vs_fpm, cmds.throttle, state.pitch_deg, state.roll_deg, cmds.slip,
                g.as_ref().and_then(|g| g.target_altitude_ft).map(|a| a.round()), g.as_ref().and_then(|g| g.target_speed_kt),
                g.as_ref().map(|g| g.vertical_mode),
            );
        }
        f.samples.push((raw.time_s, phase, state.position_ft, state.alt_msl_ft));
        f.extra.push((raw.time_s, state.alt_agl_ft, state.roll_deg, state.track_deg, raw.on_ground));
        if let Some(snap) = &pilot.latest_snapshot {
            for (seq, text) in &snap.events {
                if *seq > seen_events {
                    f.events.push(format!("{:.0}s {text}", raw.time_s));
                    seen_events = *seq;
                }
            }
        }
        for a in amends.iter_mut().filter(|a| a.sent_at.is_none()) {
            if (a.when)(&state, phase, raw.time_s) {
                let status = pilot.latest_snapshot.as_ref().and_then(|s| s.plan_status.clone());
                let d = prep(a.code, &base, status.as_ref());
                pilot
                    .profile_mut::<PlanProfile>()
                    .expect("plan profile active")
                    .apply(d.delta, &state);
                a.sent_at = Some(raw.time_s);
                f.amend_times.push(raw.time_s);
            }
        }
        if was_airborne && raw.on_ground {
            f.touchdown_rf = Some(frame.to_runway_frame(raw.position_ft));
        }
        was_airborne = !raw.on_ground;
        if raw.on_ground && state.gs_kt < 5.0 && raw.time_s > 60.0 {
            break;
        }
    }
    for e in &f.events {
        eprintln!("event: {e}");
    }
    f
}

fn assert_landed_16r(f: &Flight) {
    let td = f.touchdown_rf.unwrap_or_else(|| panic!("no touchdown; phases {:?}", f.phases()));
    assert!(td.x > 1000.0 && td.x < 4500.0, "touchdown {:.0} ft down 16R", td.x);
    assert!(td.y.abs() < 40.0, "touchdown {:.0} ft off centerline", td.y);
}

const DOWNWIND_START: (f64, f64) = (4000.0, 6000.0); // abeam midfield, right traffic 16R

#[test]
fn b1_turns_base_at_the_118() {
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\np.turn('base', when=crossing(road('the 118')))\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        80.0,
        vec![],
        900.0,
    );
    let (_, pos, _) = f.transition(FlightPhase::Downwind, FlightPhase::Base).expect("turned base");
    let d = line("CA-118").nearest(pos).unwrap().distance_ft;
    assert!(d < 800.0, "base turn {d:.0} ft from the 118");
    assert_landed_16r(&f);
}

#[test]
fn a1_open_downwind_waits_for_the_call() {
    // Extend downwind, I'll call your base; the call comes 2 nm past the
    // normal base point.
    let amend = Amend {
        when: Box::new(|s: &AircraftState, ph, _| ph == FlightPhase::Downwind && s.runway_x_ft.unwrap_or(0.0) < -6000.0 - 2.0 * NM_TO_FT),
        code: "release('base')",
        sent_at: None,
    };
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\np.extend('downwind', OPEN)\np.turn('base', when=atc('base'))\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        80.0,
        vec![amend],
        900.0,
    );
    let released = *f.amend_times.first().expect("ATC call sent");
    let (t_base, _, _) = f.transition(FlightPhase::Downwind, FlightPhase::Base).expect("turned base");
    assert!(t_base >= released, "turned base at {t_base:.0}s before the call at {released:.0}s");
    assert!(t_base - released < 2.0, "turned base {:.1}s after the call", t_base - released);
    assert_landed_16r(&f);
}

#[test]
fn e1_altitude_limit_until_the_118() {
    // Northbound well west of the field, climbing to 3000, limited to
    // 2000 until north of the 118.
    let start = Vec2::new(4000.0, 12_000.0);
    let f = fly(
        "heading(355)\nclimb(to=3000)\nremain(at_or_below(2000)).until(north_of(road('the 118')))",
        start,
        355.0,
        1500.0,
        85.0,
        vec![],
        400.0,
    );
    let road = line("CA-118");
    let mut max_south = f64::MIN;
    let mut north_alts = Vec::new();
    for (_, _, pos, alt) in &f.samples {
        if road.is_on_side(*pos, 0.0) {
            north_alts.push(*alt);
        } else {
            max_south = max_south.max(*alt);
        }
    }
    assert!(max_south <= 2100.0, "{max_south:.0} ft before the 118");
    let top = north_alts.iter().cloned().fold(f64::MIN, f64::max);
    assert!(top > 2800.0, "only reached {top:.0} ft after the 118");
    assert!(f.events.iter().any(|e| e.contains("ended")), "{:?}", f.events);
}

#[test]
fn c9_present_heading_to_the_118_then_join() {
    // 6 nm north of the field heading 200, cross the 118, then right
    // traffic 16R.
    let start = Vec2::new(-6.0 * NM_TO_FT, 2000.0);
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\nheading(PRESENT).until(crossing(road('the 118'))).then(join(p))",
        start,
        200.0,
        2300.0,
        90.0,
        vec![],
        1200.0,
    );
    let road = line("CA-118");
    // Held 200 until the crossing.
    let crossing = f.samples.windows(2).position(|w| road.crossed_by(w[0].2, w[1].2)).expect("crossed the 118");
    for w in f.samples[..crossing].windows(2) {
        let trk = xplane_pilot::types::vector_to_heading(w[1].2 - w[0].2);
        assert!(xplane_pilot::types::wrap_degrees_180(trk - 200.0).abs() < 8.0, "track {trk:.0} before the 118");
    }
    assert!(f.phases().contains(&FlightPhase::Downwind), "{:?}", f.phases());
    assert_landed_16r(&f);
}

#[test]
fn plain_pattern_lands() {
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        80.0,
        vec![],
        900.0,
    );
    assert_landed_16r(&f);
}

#[test]
fn long_downwind_extension_lands() {
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\np.extend('downwind', nm(2.8))\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        80.0,
        vec![],
        900.0,
    );
    assert_landed_16r(&f);
}

// ---------- M2: departures, entries, roads, holds, constraints ----------

fn place_geom(name: &str) -> Geom {
    let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &KvnyFixture);
    r.resolve(&Ref::Place { name: name.into() }).unwrap().geom
}

#[test]
fn takeoff_then_left_downwind_departure() {
    // E1 the library way: no turn below 400 AGL, crosswind east, then
    // north on the downwind, at or below 1800 until north of the field.
    let f = fly_from(
        "p = pattern(apt.runway('16R'), LEFT)\ntakeoff(apt.runway('16R')).then(depart(p, via=DOWNWIND))\nremain(at_or_below(1800)).until(north_of(apt))\nclimb(to=3500)",
        Vec2::new(10.0, 0.0),
        175.5,
        800.0,
        0.0,
        true,
        vec![],
        500.0,
        None,
    );
    let max_low_bank = f
        .extra
        .iter()
        .filter(|e| !e.4 && e.1 < 350.0)
        .map(|e| e.2.abs())
        .fold(0.0, f64::max);
    assert!(max_low_bank < 8.0, "banked {max_low_bank:.0}° below 350 AGL");
    let last = f.extra.last().unwrap();
    assert!(xplane_pilot::types::wrap_degrees_180(last.3 - 355.5).abs() < 15.0, "final track {:.0}", last.3);
    // East of the runway (left traffic for 16R).
    let east = f.samples.iter().any(|s| s.2.x > 3000.0);
    assert!(east, "never went east of the runway");
    let field_north = 4000.0; // a little north of 16R's threshold
    let max_alt_south = f.samples.iter().filter(|s| s.2.y < field_north).map(|s| s.3).fold(0.0, f64::max);
    assert!(max_alt_south < 1900.0, "{max_alt_south:.0} ft before north of the field");
}

#[test]
fn follow_the_405_keep_it_off_the_left_wing() {
    // Northbound up the valley east of the 405.
    let start = Vec2::new(4.0 * NM_TO_FT, -3000.0); // south of the field, east of 16R
    let f = fly(
        "follow(road('the 405'), direction=NORTH, keep=LEFT).until(north_of(road('Roscoe')))",
        start,
        355.0,
        2500.0,
        90.0,
        vec![],
        400.0,
    );
    let road = line("I-405");
    let roscoe = line("Roscoe Boulevard");
    // Once settled, and until the step ends at Roscoe, every sample sits
    // east of the 405, 800-3500 ft off.
    let settled: Vec<_> = f.samples.iter().skip(150).take_while(|s| !roscoe.is_on_side(s.2, 0.0)).collect();
    assert!(!settled.is_empty());
    for s in &settled {
        let n = road.nearest(s.2).unwrap();
        assert!(road.is_on_side(s.2, 90.0), "west of the 405 at t={:.0}", s.0);
        assert!(n.distance_ft > 800.0 && n.distance_ft < 3500.0, "{:.0} ft off at t={:.0}", n.distance_ft, s.0);
    }
}

#[test]
fn remain_south_of_the_101_is_enforced() {
    // Heading 030 would cross the 101; the constraint keeps the aircraft
    // south of it.
    let start = Vec2::new(5.0 * NM_TO_FT, 8000.0);
    let f = fly("heading(30)\nremain(south_of(road('the 101')))", start, 30.0, 2500.0, 90.0, vec![], 300.0);
    let road = line("US-101");
    for s in &f.samples {
        assert!(!road.is_on_side(s.2, 0.0), "north of the 101 at t={:.0}", s.0);
    }
}

#[test]
fn hold_over_lake_balboa() {
    let f = fly("hold_over(place('Lake Balboa'))", Vec2::new(4000.0, 6000.0), 175.0, 2500.0, 90.0, vec![], 400.0);
    let Geom::Area { center, .. } = place_geom("Lake Balboa") else { panic!() };
    let late: Vec<_> = f.samples.iter().filter(|s| s.0 > 200.0).collect();
    for s in &late {
        assert!(s.2.distance_to(center) < 1.2 * NM_TO_FT, "{:.0} ft from the lake at t={:.0}", s.2.distance_to(center), s.0);
    }
}

#[test]
fn low_approach_then_straight_out() {
    // 3 nm final for 16R.
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\njoin(p, land=LOW_APPROACH, on_go=depart(p, via=STRAIGHT_OUT))",
        Vec2::new(-3.0 * NM_TO_FT, 0.0),
        175.5,
        1750.0,
        70.0,
        vec![],
        300.0,
    );
    assert!(f.touchdown_rf.is_none(), "touched down");
    let min_agl = f.extra.iter().map(|e| e.1).fold(f64::MAX, f64::min);
    assert!(min_agl < 250.0, "lowest {min_agl:.0} AGL");
    let last = f.extra.last().unwrap();
    assert!(last.1 > 500.0, "not climbing out: {:.0} AGL", last.1);
    assert!(xplane_pilot::types::wrap_degrees_180(last.3 - 175.5).abs() < 10.0, "track {:.0}", last.3);
    assert!(f.events.iter().any(|e| e.contains("done: join")), "{:?}", f.events);
}

#[test]
fn enter_downwind_on_the_45() {
    // Inbound from the northeast for left traffic 16R (downwind east of
    // the runway, northbound).
    let f = fly(
        "p = pattern(apt.runway('16R'), LEFT)\nenter(p.downwind, via=FORTY_FIVE)",
        Vec2::new(4.0 * NM_TO_FT, -5.0 * NM_TO_FT),
        290.0,
        2300.0,
        95.0,
        vec![],
        1200.0,
    );
    assert!(f.phases().contains(&FlightPhase::Downwind), "{:?}", f.phases());
    // The entry heads in on the 45 (about 310° for 16R's 355° downwind).
    let entry: Vec<_> = f.samples.iter().filter(|s| s.1 == FlightPhase::PatternEntry).collect();
    assert!(entry.len() > 20, "no 45 leg");
    let a = entry[entry.len() / 3].2;
    let b = entry[entry.len() - 1].2;
    let trk = xplane_pilot::types::vector_to_heading(b - a);
    assert!(xplane_pilot::types::wrap_degrees_180(trk - 310.0).abs() < 20.0, "entry track {trk:.0}");
    let td = f.touchdown_rf.expect("landed");
    assert!(td.y.abs() < 40.0);
}

#[test]
fn go_around_then_back_around() {
    let amend = Amend {
        when: Box::new(|s: &AircraftState, ph, _| ph == FlightPhase::Final && s.alt_agl_ft < 400.0),
        code: "go_around()",
        sent_at: None,
    };
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        80.0,
        vec![amend],
        1500.0,
    );
    assert!(f.phases().contains(&FlightPhase::GoAround), "{:?}", f.phases());
    assert_landed_16r(&f);
}

#[test]
fn overhead_entry_under_an_altitude_floor_keeps_moving() {
    // C2: "fly over the field at or above 2300, then enter right downwind
    // 34L", read as a floor until base. The entry's descent can't finish
    // under the floor; the plan must carry on to the downwind anyway
    // instead of holding the crossing heading out of the area.
    let f = fly(
        "p = pattern(apt.runway('34L'), RIGHT)\n\
         remain(at_or_above(2300)).until(turning(p.base))\n\
         direct(apt).then(enter(p.downwind, via=OVERHEAD)).then(join(p))",
        Vec2::new(4000.0, 5.0 * NM_TO_FT),
        85.0,
        2500.0,
        95.0,
        vec![],
        1500.0,
    );
    let downwind_t = f.samples.iter().find(|s| s.1 == FlightPhase::Downwind).map(|s| s.0);
    assert!(downwind_t.is_some_and(|t| t < 700.0), "downwind at {downwind_t:?}; phases {:?}", f.phases());
    assert!(f.touchdown_rf.is_some(), "no touchdown; phases {:?}", f.phases());
}

#[test]
fn descent_blocked_by_a_floor_ends_and_says_so() {
    let f = fly(
        "remain(at_or_above(2300))\ndescend(to=1800).then(turn(LEFT, to=270))",
        Vec2::new(4000.0, 5.0 * NM_TO_FT),
        0.0,
        2500.0,
        95.0,
        vec![],
        200.0,
    );
    assert!(f.events.iter().any(|e| e.contains("held at 2350")), "{:?}", f.events);
    let last = f.extra.last().unwrap();
    assert!(xplane_pilot::types::wrap_degrees_180(last.3 - 270.0).abs() < 10.0, "track {:.0}", last.3);
    assert!(f.samples.iter().all(|s| s.3 > 2200.0), "went below the floor");
}

#[test]
fn direct_across_a_line_holds_short_until_released() {
    // D2: "remain south of the 101 until advised, proceed direct the
    // Woodley golf course" (north of the 101). Circle south of the 101
    // near the golf course rather than sliding along the freeway, then go
    // on once the constraint is cancelled.
    let road = line("US-101");
    let golf = place_geom("Woodley Lakes Golf Course").anchor();
    let f = fly(
        "direct(place('the Woodley golf course'))\nremain(south_of(road('the 101')))",
        Vec2::new(24304.0, 18228.0),
        60.0,
        2500.0,
        95.0,
        vec![Amend { when: Box::new(|_, _, t| t >= 420.0), code: "cancel('c1')", sent_at: None }],
        700.0,
    );
    assert!(f.events.iter().any(|e| e.contains("circling short")), "{:?}", f.events);
    for s in f.samples.iter().filter(|s| s.0 < 420.0) {
        assert!(!road.is_on_side(s.2, 0.0), "north of the 101 at t={:.0}", s.0);
        if s.0 > 200.0 {
            let d = s.2.distance_to(golf);
            assert!(d < 3.0 * NM_TO_FT, "{:.1} nm from the golf course at t={:.0}", d / NM_TO_FT, s.0);
        }
    }
    assert!(f.events.iter().any(|e| e.contains("done: direct Woodley")), "{:?}", f.events);
}

#[test]
fn leg_into_a_base_entry_descends_to_pattern_altitude() {
    // C6-style: inbound 1200 ft above pattern altitude, "enter right base
    // 16R over <point>" with no altitude given. Arrive at pattern
    // altitude for the base, not 1000 ft high and landing long.
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\n\
         direct(apt.runway('16R').at(along=nm(-1.2), right=nm(1.2))).then(enter(p.base))",
        Vec2::new(-5.0 * NM_TO_FT, 5.0 * NM_TO_FT),
        135.0,
        3000.0,
        95.0,
        vec![],
        900.0,
    );
    let first_base = f.samples.iter().find(|s| s.1 == FlightPhase::Base).expect("flew base");
    assert!(first_base.3 < 1800.0 + 200.0, "entered base at {:.0} ft", first_base.3);
    assert_landed_16r(&f);
}

fn scripted(yaml: &str) -> ScriptedTraffic {
    let specs: Vec<ScenarioTraffic> = serde_yaml_ng::from_str(yaml).unwrap();
    let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &KvnyFixture);
    let elev = KvnyFixture.airport("KVNY").unwrap().elevation_ft;
    ScriptedTraffic::new(&specs, "KVNY", elev, &mut r).unwrap()
}

/// Closest approach to airborne traffic within 400 ft vertically.
fn min_separation_ft(f: &Flight, t: &ScriptedTraffic) -> f64 {
    let mut min = f64::INFINITY;
    for s in &f.samples {
        for x in t.at(s.0).iter().filter(|x| !x.on_ground) {
            if (x.alt_msl_ft - s.3).abs() < 400.0 {
                min = min.min(x.position_ft.distance_to(s.2));
            }
        }
    }
    min
}

#[test]
fn number_two_follows_the_cherokee_on_a_3_mile_final() {
    // H3: on right downwind 16R, "follow the Cherokee on a 3 mile final".
    // The base turn waits until the Cherokee is past abeam on final, then
    // the aircraft lands after it with room to spare.
    let traffic = scripted(
        "- {id: t1, type: Cherokee, runway: 16R, speed_kt: 75, path: [{along_ft: -18228, right_ft: 0, alt_agl_ft: 950}, {along_ft: 1200, right_ft: 0, alt_agl_ft: 0}, {along_ft: 3500, right_ft: 0, alt_agl_ft: 0}]}",
    );
    let f = fly_from(
        "p = pattern(apt.runway('16R'), RIGHT)\nt = traffic('Cherokee on a 3 mile final')\np.follow(t)\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        90.0,
        false,
        vec![],
        900.0,
        Some(&traffic),
    );
    let (t_base, pos, _) = f.transition(FlightPhase::Downwind, FlightPhase::Base).expect("turned base");
    // The Cherokee passes abeam about 1.2 nm before the threshold.
    let frame = frame_16r();
    let rf = frame.to_runway_frame(pos);
    assert!(rf.x < -5000.0, "turned base {:.0} ft before the threshold at t={t_base:.0}; should wait for the Cherokee", -rf.x);
    let tdz = f.touchdown_rf.expect("landed");
    assert!(tdz.x > 1000.0 && tdz.x < 4500.0 && tdz.y.abs() < 40.0, "touchdown {tdz:?}");
    let sep = min_separation_ft(&f, &traffic);
    assert!(sep > 2500.0, "came within {sep:.0} ft of the Cherokee");
    assert!(f.events.iter().all(|e| !e.contains("go-around")), "{:?}", f.events);
}

#[test]
fn follow_with_nothing_sensed_extends_until_atc_calls_base() {
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\np.follow(traffic('the Falcon on a 6 mile final'))\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        90.0,
        vec![Amend { when: Box::new(|_, _, t| t >= 150.0), code: "release('base')", sent_at: None }],
        900.0,
    );
    let (t_base, _, _) = f.transition(FlightPhase::Downwind, FlightPhase::Base).expect("turned base");
    assert!(t_base >= 150.0, "turned base at {t_base:.0} before ATC called it");
    assert!(t_base < 160.0, "base {t_base:.0} long after the call");
    assert!(f.touchdown_rf.is_some());
}

/// Preview `code` from the downwind start with the H3 Cherokee.
fn preview_h3(code: &str) -> xplane_pilot::plan::preview::Preview {
    let traffic = scripted(
        "- {id: t1, type: Cherokee, runway: 16R, speed_kt: 75, path: [{along_ft: -18228, right_ft: 0, alt_agl_ft: 950}, {along_ft: 1200, right_ft: 0, alt_agl_ft: 0}, {along_ft: 3500, right_ft: 0, alt_agl_ft: 0}]}",
    );
    let base = load_default_config_bundle();
    let prepared = prep(code, &base, None);
    let config = prepared.delta.reference_pattern().unwrap().config.clone();
    let frame = RunwayFrame::new(config.airport.runway.clone());
    let mut pilot = PilotCore::new(config.clone());
    let raw = DynamicsState {
        position_ft: frame.to_world_frame(Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1)),
        altitude_ft: 1800.0,
        agl_ft: None,
        heading_deg: 355.5,
        roll_deg: 0.0,
        pitch_deg: 2.0,
        ias_kt: 90.0,
        throttle_pos: 0.5,
        on_ground: false,
        time_s: 0.0,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ground_velocity_ft_s: heading_to_vector(355.5, 90.0 * KT_TO_FPS),
        vertical_speed_ft_s: 0.0,
        flap_index: 0,
        gear_down: true,
        sideslip_deg: 0.0,
        parking_brake: 0.0,
    };
    let (est, _) = pilot.update(&raw, DT);
    let mut profile = PlanProfile::new(base.clone(), &est);
    profile.apply(prepared.delta, &est);
    let pv = xplane_pilot::plan::preview::preview(&config, profile, &est, traffic.at(0.0), 900.0);
    for l in pv.lines() {
        eprintln!("{l}");
    }
    pv
}

#[test]
fn preview_says_following_works_and_turning_base_now_doesnt() {
    let follow = preview_h3("p = pattern(apt.runway('16R'), RIGHT)\np.follow(traffic('Cherokee on a 3 mile final'))\njoin(p)");
    assert!(follow.problems.is_empty(), "{:?}", follow.problems);
    assert!(follow.timeline.iter().any(|l| l.contains("touchdown")), "{:?}", follow.timeline);
    let cut_in = preview_h3("p = pattern(apt.runway('16R'), RIGHT)\np.turn('base', when=NOW)\njoin(p)");
    assert!(cut_in.problems.iter().any(|p| p.contains("t1 Cherokee")), "{:?}", cut_in.problems);
}

#[test]
fn short_approach_turns_base_abeam_the_threshold_and_lands() {
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\np.final_length(SHORT)\njoin(p)",
        Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1),
        355.5,
        1800.0,
        85.0,
        vec![],
        900.0,
    );
    let (_, pos, _) = f.transition(FlightPhase::Downwind, FlightPhase::Base).expect("turned base");
    let frame = frame_16r();
    let rf = frame.to_runway_frame(pos);
    assert!(rf.x > -1500.0, "base turn {:.0} ft before the threshold", -rf.x);
    assert_landed_16r(&f);
}

#[test]
fn steep_turn_meets_the_acs_standards() {
    // P25: 45° bank (±5), entry altitude ±100 ft, roll out on the entry
    // heading ±10°.
    let f = fly("steep_turn(LEFT)", Vec2::new(-5.0 * NM_TO_FT, 6.0 * NM_TO_FT), 90.0, 3500.0, 95.0, vec![], 120.0);
    assert!(f.events.iter().any(|e| e.contains("done: steep turn left 360")), "{:?}", f.events);
    let done_t: f64 = f.events.iter().find(|e| e.contains("done: steep turn")).unwrap().split('s').next().unwrap().parse().unwrap();
    let during: Vec<_> = f.extra.iter().zip(&f.samples).filter(|(e, _)| e.0 > 8.0 && e.0 < done_t - 5.0).collect();
    let max_bank = during.iter().map(|(e, _)| e.2.abs()).fold(0.0, f64::max);
    let min_bank = during.iter().map(|(e, _)| e.2.abs()).fold(90.0, f64::min);
    let alt_dev = f.samples.iter().filter(|s| s.0 < done_t + 10.0).map(|s| (s.3 - 3500.0).abs()).fold(0.0, f64::max);
    eprintln!("bank {min_bank:.1}..{max_bank:.1}, alt dev {alt_dev:.0}");
    assert!(max_bank <= 50.0 && min_bank >= 40.0, "bank {min_bank:.1}..{max_bank:.1}");
    assert!(alt_dev <= 100.0, "altitude off by {alt_dev:.0} ft");
    let after = f.extra.iter().find(|e| e.0 >= done_t + 8.0).expect("flew on after the turn");
    assert!(xplane_pilot::types::wrap_degrees_180(after.3 - 90.0).abs() <= 10.0, "rolled out on {:.0}", after.3);
}

#[test]
fn high_on_final_slips_down_and_lands_in_the_zone() {
    // On a 1.5 nm final about 850 ft above the 3° path: the forward slip
    // (P24) takes the height off without gaining airspeed, comes out
    // before the roundout, and the landing is in the first third.
    let f = fly(
        "p = pattern(apt.runway('16R'), RIGHT)\njoin(p)",
        Vec2::new(-1.5 * NM_TO_FT, 0.0),
        175.5,
        800.0 + 1350.0,
        75.0,
        vec![],
        400.0,
    );
    let slipping: Vec<_> = f.extra.iter().filter(|e| !e.4 && e.2.abs() > 8.0 && e.1 > 300.0).collect();
    eprintln!("slipping samples {} touchdown {:?}", slipping.len(), f.touchdown_rf);
    assert!(!slipping.is_empty(), "never banked into a slip");
    // Track stays on the centerline while slipping.
    for e in &slipping {
        assert!(xplane_pilot::types::wrap_degrees_180(e.3 - 175.5).abs() < 15.0, "track {:.0} while slipping", e.3);
    }
    // Wings level by the roundout.
    for e in f.extra.iter().filter(|e| !e.4 && e.1 < 40.0) {
        assert!(e.2.abs() < 6.0, "bank {:.1} at {:.0} ft", e.2, e.1);
    }
    assert_landed_16r(&f);
}

#[test]
fn rectangular_course_holds_the_track_in_a_crosswind() {
    // P23: constant distance from the field's edges, constant altitude,
    // correcting for a 10 kt west wind. Two laps around Pierce College.
    use xplane_pilot::guidance::geo::rectangular_course;
    use xplane_pilot::plan::compile::{course_rect, RECT_CORNER_RADIUS_FT, RECT_DEFAULT_OFFSET_FT};
    use xplane_pilot::plan::resolve::Feature;
    let pierce = Feature { label: "Pierce College".into(), geom: place_geom("Pierce College") };
    let path = rectangular_course(&course_rect(&pierce, None), RECT_DEFAULT_OFFSET_FT, RECT_CORNER_RADIUS_FT, true);
    let frame = frame_16r();
    let start = frame.to_runway_frame(pierce.geom.anchor() + Vec2::new(2.0 * NM_TO_FT, 0.0));
    WIND_KT.with(|w| w.set(heading_to_vector(90.0, 10.0)));
    let f = fly(
        "rectangular_course(place('Pierce College'), LEFT, laps=2, altitude=agl(1000))",
        start,
        270.0,
        1800.0,
        90.0,
        vec![],
        900.0,
    );
    WIND_KT.with(|w| w.set(Vec2::ZERO));
    assert!(f.events.iter().any(|e| e.contains("done: rectangular course")), "{:?}", f.events);
    let done_t: f64 = f.events.iter().find(|e| e.contains("done: rectangular course")).unwrap().split('s').next().unwrap().parse().unwrap();
    // From first reaching the track until the end of lap 2.
    let on_t = f.samples.iter().find(|s| path.nearest(s.2, None).distance_ft < 300.0).expect("reached the course").0;
    let lap: Vec<_> = f.samples.iter().zip(&f.extra).filter(|(s, _)| s.0 > on_t + 30.0 && s.0 <= done_t).collect();
    let max_off = lap.iter().map(|(s, _)| path.nearest(s.2, None).distance_ft).fold(0.0, f64::max);
    let max_bank = lap.iter().map(|(_, e)| e.2.abs()).fold(0.0, f64::max);
    let mut alts: Vec<f64> = lap.iter().map(|(s, _)| s.3).collect();
    alts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let alt_spread = alts.last().unwrap() - alts[0];
    eprintln!("on the course at {on_t:.0}s, lap 2 at {done_t:.0}s: max off {max_off:.0} ft, max bank {max_bank:.0}, altitude {:.0}..{:.0}", alts[0], alts.last().unwrap());
    assert!(max_off <= 300.0, "{max_off:.0} ft off the course");
    assert!(max_bank <= 45.0, "banked {max_bank:.0}");
    assert!(alt_spread <= 200.0, "altitude varied {alt_spread:.0} ft");
}

fn frame_16r() -> RunwayFrame {
    RunwayFrame::new(
        prep("p = pattern(apt.runway('16R'), RIGHT)\njoin(p)", &load_default_config_bundle(), None)
            .delta
            .reference_pattern()
            .unwrap()
            .frame
            .runway
            .clone(),
    )
}

/// (turn onto base s, wings-level base s, turn to final s, rollout |y|)
/// for the last base, scored like the scenario's square-base check.
fn square_metrics(f: &Flight) -> (f64, f64, f64, f64) {
    use xplane_pilot::types::wrap_degrees_180 as w180;
    let frame = frame_16r();
    let course = frame.runway.course_deg;
    let base_course = course - 90.0; // right traffic
    let air: Vec<_> = f.samples.iter().zip(&f.extra).filter(|(_, e)| !e.4).collect();
    let near = |trk: f64, c: f64| w180(trk - c).abs() <= 10.0;
    let mut run = None;
    let mut start = None;
    for (i, (_, e)) in air.iter().enumerate() {
        if near(e.3, base_course) && e.2.abs() < 5.0 {
            let st = *start.get_or_insert(i);
            run = Some((st, i));
        } else {
            start = None;
        }
    }
    let (a, b) = run.expect("a wings-level base");
    let t = |i: usize| air[i].0 .0;
    let into = air[..a].iter().rev().find(|(_, e)| near(e.3, course + 180.0)).map(|(s, _)| t(a) - s.0).unwrap_or(f64::INFINITY);
    let out = air[b..].iter().find(|(_, e)| near(e.3, course)).expect("turned final");
    (into, t(b) - t(a), out.0 .0 - t(b), frame.to_runway_frame(out.0 .2).y.abs())
}

#[test]
fn square_base_turns_definitely_and_rolls_out_on_each_leg() {
    // A5/A8: "square your base turn and final". The squared corners are
    // quick, steady-bank turns that roll out on base and on the
    // centerline; the default pattern rounds both off.
    let start = Vec2::new(DOWNWIND_START.0, DOWNWIND_START.1);
    let sq = fly("p = pattern(apt.runway('16R'), RIGHT)\np.square('base')\njoin(p)", start, 355.5, 1800.0, 80.0, vec![], 900.0);
    let (into, straight, out, y) = square_metrics(&sq);
    eprintln!("square: onto base {into:.0}s, base {straight:.0}s, to final {out:.0}s, rollout {y:.0} ft");
    assert!(into <= 20.0 && out <= 20.0, "turns took {into:.0} s and {out:.0} s");
    assert!(straight >= 15.0, "only {straight:.0} s wings level on base");
    assert!(y <= 300.0, "rolled out {y:.0} ft off the centerline");
    assert_landed_16r(&sq);

    let plain = fly("p = pattern(apt.runway('16R'), RIGHT)\njoin(p)", start, 355.5, 1800.0, 80.0, vec![], 900.0);
    let (_, _, out, _) = square_metrics(&plain);
    assert!(out > 20.0, "the default turn to final is a gradual intercept ({out:.0} s)");
}
