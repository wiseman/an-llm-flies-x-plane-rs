//! Port of tests/test_heartbeat_pump.py.

use std::sync::Arc;

use crossbeam_channel::{unbounded, TryRecvError};
use parking_lot::Mutex;

use xplane_pilot::bus::SimBus;
use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::core::mission_manager::{PilotCore, StatusSnapshot};
use xplane_pilot::core::profiles::HeadingHoldProfile;
use xplane_pilot::live_runner::{FakeClock, HeartbeatPump};
use xplane_pilot::llm::conversation::{IncomingMessage, IncomingSource};
use xplane_pilot::types::{ActuatorCommands, AircraftState, FlightPhase, Vec2};

fn make_snapshot(
    profiles: &[&str],
    phase: Option<FlightPhase>,
    alt_msl_ft: f64,
    go_around_reason: Option<&str>,
) -> StatusSnapshot {
    let state = AircraftState {
        t_sim: 42.0,
        dt: 0.2,
        position_ft: Vec2::ZERO,
        alt_msl_ft,
        alt_agl_ft: (alt_msl_ft - 425.0).max(0.0),
        pitch_deg: 0.0,
        roll_deg: 0.0,
        heading_deg: 270.0,
        track_deg: 270.0,
        p_rad_s: 0.0,
        q_rad_s: 0.0,
        r_rad_s: 0.0,
        ias_kt: 90.0,
        tas_kt: 90.0,
        gs_kt: 90.0,
        vs_fpm: 0.0,
        ground_velocity_ft_s: Vec2::ZERO,
        flap_index: 0,
        gear_down: true,
        on_ground: false,
        throttle_pos: 0.55,
        runway_id: None,
        runway_dist_remaining_ft: None,
        runway_x_ft: None,
        runway_y_ft: None,
        centerline_error_ft: None,
        threshold_abeam: false,
        distance_to_touchdown_ft: None,
        stall_margin: 2.0,
    };
    StatusSnapshot {
        t_sim: 42.0,
        active_profiles: profiles.iter().map(|s| s.to_string()).collect(),
        phase,
        state,
        last_commands: ActuatorCommands {
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
            throttle: 0.55,
            flaps: None,
            gear_down: Some(true),
            brakes: 0.0,
            pivot_brake: 0.0,
        },
        last_guidance: None,
        go_around_reason: go_around_reason.map(|s| s.to_string()),
        airport_ident: None,
        runway_id: None,
        field_elevation_ft: None,
        debug_lines: Vec::new(),
        completed_profiles: Vec::new(),
    }
}

fn drain(rx: &crossbeam_channel::Receiver<IncomingMessage>) -> Vec<IncomingMessage> {
    let mut msgs = Vec::new();
    loop {
        match rx.try_recv() {
            Ok(m) => msgs.push(m),
            Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => return msgs,
        }
    }
}

fn setup() -> (
    Arc<Mutex<PilotCore>>,
    Arc<FakeClock>,
    crossbeam_channel::Receiver<IncomingMessage>,
    HeartbeatPump,
) {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(Mutex::new(PilotCore::new(cfg)));
    let clock = Arc::new(FakeClock::new());
    let (tx, rx) = unbounded::<IncomingMessage>();
    let pump = HeartbeatPump::new(pilot.clone(), None, tx, 30.0, clock.clone());
    (pilot, clock, rx, pump)
}

fn seed(pilot: &Arc<Mutex<PilotCore>>, profiles: &[&str], phase: Option<FlightPhase>) {
    pilot.lock().latest_snapshot = Some(make_snapshot(profiles, phase, 1500.0, None));
}

#[test]
fn no_heartbeat_without_snapshot() {
    let (_, _, rx, pump) = setup();
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
}

#[test]
fn first_tick_after_snapshot_only_seeds_state() {
    let (pilot, _, rx, pump) = setup();
    seed(&pilot, &["idle_lateral", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
}

#[test]
fn profile_change_within_debounce_is_suppressed() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["idle_lateral", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    clock.advance_secs(0.5);
    seed(&pilot, &["heading_hold", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
}

#[test]
fn profile_change_past_debounce_fires() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["idle_lateral", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    clock.advance_secs(3.0);
    seed(&pilot, &["heading_hold", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    let msgs = drain(&rx);
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].source, IncomingSource::Heartbeat);
    assert!(msgs[0].text.contains("profiles"));
    assert!(msgs[0].text.contains("heading_hold"));
    assert!(msgs[0].text.contains("idle_lateral"));
}

#[test]
fn phase_change_fires_heartbeat() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["pattern_fly"], Some(FlightPhase::Downwind));
    pump.check_and_emit();
    clock.advance_secs(3.0);
    seed(&pilot, &["pattern_fly"], Some(FlightPhase::Base));
    pump.check_and_emit();
    let msgs = drain(&rx);
    assert_eq!(msgs.len(), 1);
    assert!(msgs[0].text.contains("downwind"));
    assert!(msgs[0].text.contains("base"));
}

#[test]
fn profile_completion_fires_heartbeat() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["taxi"], None);
    pump.check_and_emit();
    clock.advance_secs(3.0);
    let mut snap = make_snapshot(&["taxi"], None, 425.0, None);
    snap.completed_profiles = vec!["taxi".to_string()];
    pilot.lock().latest_snapshot = Some(snap);
    pump.check_and_emit();
    let msgs = drain(&rx);
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].source, IncomingSource::Heartbeat);
    assert!(msgs[0].text.contains("completed: taxi"));
}

#[test]
fn already_completed_profile_does_not_refire_heartbeat() {
    let (pilot, clock, rx, pump) = setup();
    let mut snap = make_snapshot(&["taxi"], None, 425.0, None);
    snap.completed_profiles = vec!["taxi".to_string()];
    pilot.lock().latest_snapshot = Some(snap);
    pump.check_and_emit();
    // Same completion state a few ticks later — no new completion, no fire.
    clock.advance_secs(3.0);
    let mut snap2 = make_snapshot(&["taxi"], None, 425.0, None);
    snap2.completed_profiles = vec!["taxi".to_string()];
    pilot.lock().latest_snapshot = Some(snap2);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
}

#[test]
fn periodic_heartbeat_fires_after_idle_interval() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["idle_lateral", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    clock.advance_secs(25.0);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
    clock.advance_secs(10.0);
    pump.check_and_emit();
    let msgs = drain(&rx);
    assert_eq!(msgs.len(), 1);
    assert!(msgs[0].text.contains("periodic"));
}

#[test]
fn user_input_resets_idle_timer() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["idle_lateral", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    clock.advance_secs(25.0);
    pump.record_user_input();
    clock.advance_secs(10.0);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
    clock.advance_secs(25.0);
    pump.check_and_emit();
    assert_eq!(drain(&rx).len(), 1);
}

#[test]
fn periodic_heartbeat_resets_its_own_timer() {
    let (pilot, clock, rx, pump) = setup();
    seed(&pilot, &["idle_lateral", "idle_vertical", "idle_speed"], None);
    pump.check_and_emit();
    clock.advance_secs(35.0);
    pump.check_and_emit();
    assert_eq!(drain(&rx).len(), 1);
    clock.advance_secs(10.0);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());
    clock.advance_secs(25.0);
    pump.check_and_emit();
    assert_eq!(drain(&rx).len(), 1);
}

#[test]
fn heartbeat_embeds_status_json() {
    let (pilot, clock, rx, pump) = setup();
    pilot.lock().latest_snapshot = Some(make_snapshot(
        &["heading_hold", "altitude_hold", "speed_hold"],
        None,
        2500.0,
        None,
    ));
    pump.check_and_emit();
    clock.advance_secs(35.0);
    pump.check_and_emit();
    let msgs = drain(&rx);
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].source, IncomingSource::Heartbeat);
    assert!(msgs[0].text.contains(" | status="));
    let payload_start = msgs[0].text.find(" | status=").unwrap() + " | status=".len();
    let payload: serde_json::Value = serde_json::from_str(&msgs[0].text[payload_start..]).unwrap();
    assert_eq!(
        payload["active_profiles"],
        serde_json::json!(["heading_hold", "altitude_hold", "speed_hold"])
    );
    assert_eq!(payload["alt_msl_ft"], serde_json::json!(2500.0));
    assert_eq!(payload["throttle_pos"], serde_json::json!(0.55));
}

#[test]
fn go_around_transition_pushes_safety_log_line() {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(Mutex::new(PilotCore::new(cfg)));
    let clock = Arc::new(FakeClock::new());
    let bus = SimBus::new(false);
    let (tx, _rx) = unbounded::<IncomingMessage>();
    let pump = HeartbeatPump::new(pilot.clone(), Some(bus.clone()), tx, 30.0, clock.clone());
    pilot.lock().latest_snapshot = Some(make_snapshot(&["pattern_fly"], Some(FlightPhase::Final), 1500.0, None));
    pump.check_and_emit();
    clock.advance_secs(3.0);
    pilot.lock().latest_snapshot = Some(make_snapshot(
        &["pattern_fly"],
        Some(FlightPhase::GoAround),
        1500.0,
        Some("unstable_lateral cle=3500ft agl=199ft limit=398ft"),
    ));
    pump.check_and_emit();
    let tail = bus.log_tail(50);
    let matching: Vec<&String> = tail.iter().filter(|l| l.contains("go_around triggered")).collect();
    assert_eq!(matching.len(), 1);
    assert!(matching[0].contains("unstable_lateral"));
}

#[test]
fn integration_heartbeat_sees_profile_change_from_live_pilot_update() {
    let cfg = load_default_config_bundle();
    let pilot = Arc::new(Mutex::new(PilotCore::new(cfg.clone())));
    let clock = Arc::new(FakeClock::new());
    let (tx, rx) = unbounded::<IncomingMessage>();
    let pump = HeartbeatPump::new(pilot.clone(), None, tx, 30.0, clock.clone());

    let model = xplane_pilot::sim::simple_dynamics::SimpleAircraftModel::new(cfg, Vec2::ZERO);
    let raw = model.initial_state();
    pilot.lock().update(&raw, 0.2);
    pump.check_and_emit();
    assert!(drain(&rx).is_empty());

    pilot
        .lock()
        .engage_profile(Box::new(HeadingHoldProfile::new(270.0, 25.0, None).unwrap()));
    pilot.lock().update(&raw, 0.2);
    clock.advance_secs(3.0);
    pump.check_and_emit();
    let msgs = drain(&rx);
    assert_eq!(msgs.len(), 1);
    assert!(msgs[0].text.contains("engaged: heading_hold"));
    assert!(msgs[0].text.contains("disengaged: idle_lateral"));
}
