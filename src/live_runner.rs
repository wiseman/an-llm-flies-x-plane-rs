//! Live X-Plane runner: bootstrap probe, bridge, pilot core, LLM worker
//! thread, heartbeat pump, control loop, and (optionally) interactive TUI.
//! Mirrors sim_pilot/live_runner.py.

use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc, Mutex,
};

use crossbeam_channel::Sender;

use crate::bus::SimBus;
use crate::config::{AirportConfig, ConfigBundle};
use crate::core::mission_manager::PilotCore;
use crate::llm::conversation::IncomingMessage;
use crate::sim::xplane_bridge::{BootstrapSample, M_TO_FT};
use crate::types::{FlightPhase, Runway};

/// A trivial monotonic clock abstraction so tests can advance virtual time.
pub trait Clock: Send + Sync {
    fn now_secs_f64(&self) -> f64;
}

pub struct RealClock;
impl Clock for RealClock {
    fn now_secs_f64(&self) -> f64 {
        static START: once_cell::sync::Lazy<std::time::Instant> =
            once_cell::sync::Lazy::new(std::time::Instant::now);
        START.elapsed().as_secs_f64()
    }
}

/// Test clock: a mutable counter in fractional seconds.
pub struct FakeClock {
    pub now_ms: AtomicU64,
}
impl FakeClock {
    pub fn new() -> Self { Self { now_ms: AtomicU64::new(0) } }
    pub fn advance_secs(&self, dt: f64) {
        let ms = (dt * 1000.0) as u64;
        self.now_ms.fetch_add(ms, Ordering::Relaxed);
    }
}
impl Clock for FakeClock {
    fn now_secs_f64(&self) -> f64 {
        self.now_ms.load(Ordering::Relaxed) as f64 / 1000.0
    }
}

#[derive(Default)]
struct HeartbeatState {
    last_user_input_s: f64,
    last_heartbeat_s: f64,
    last_seen_phase: Option<FlightPhase>,
    last_seen_profiles: Option<Vec<String>>,
    last_seen_completed: Option<Vec<String>>,
    last_seen_has_crashed: Option<bool>,
    /// While `clock.now_secs_f64() < idle_suppress_until_s`, the idle-cadence
    /// heartbeat is suppressed. Set by the LLM via `sleep(suppress_idle_heartbeat_s=...)`
    /// when it's confident the current state is stable for a while. State-change
    /// heartbeats (phase/profile/crash-edge) and inbound user/ATC messages still
    /// wake the LLM immediately regardless of this field.
    idle_suppress_until_s: f64,
}

pub struct HeartbeatPump {
    pub pilot: Arc<parking_lot::Mutex<PilotCore>>,
    pub bus: Option<SimBus>,
    pub bridge: Option<Arc<dyn crate::llm::tools::ToolBridge>>,
    pub airspace: Option<crate::data::airspace_query::AirspaceSource>,
    pub input_queue: Sender<IncomingMessage>,
    pub heartbeat_interval_s: f64,
    pub poll_interval_s: f64,
    pub min_interval_s: f64,
    pub clock: Arc<dyn Clock>,
    state: Mutex<HeartbeatState>,
}

impl HeartbeatPump {
    pub fn new(
        pilot: Arc<parking_lot::Mutex<PilotCore>>,
        bus: Option<SimBus>,
        input_queue: Sender<IncomingMessage>,
        heartbeat_interval_s: f64,
        clock: Arc<dyn Clock>,
    ) -> Self {
        let now = clock.now_secs_f64();
        Self {
            pilot,
            bus,
            bridge: None,
            airspace: None,
            input_queue,
            heartbeat_interval_s,
            poll_interval_s: 0.5,
            min_interval_s: 2.0,
            clock,
            state: Mutex::new(HeartbeatState {
                last_user_input_s: now,
                last_heartbeat_s: now,
                last_seen_phase: None,
                last_seen_profiles: None,
                last_seen_completed: None,
                last_seen_has_crashed: None,
                idle_suppress_until_s: 0.0,
            }),
        }
    }

    /// LLM hint (via `sleep(suppress_idle_heartbeat_s=N)`): skip the idle-cadence
    /// heartbeat for `seconds` of wall time. Non-positive values clear any
    /// existing suppression; the newest call always wins (no min/max merging)
    /// so the LLM can shorten an earlier, over-eager hint.
    pub fn suppress_idle_heartbeat_for_seconds(&self, seconds: f64) {
        let now = self.clock.now_secs_f64();
        let mut st = self.state.lock().unwrap();
        st.idle_suppress_until_s = if seconds > 0.0 { now + seconds } else { 0.0 };
    }

    pub fn with_bridge(mut self, bridge: Arc<dyn crate::llm::tools::ToolBridge>) -> Self {
        self.bridge = Some(bridge);
        self
    }

    pub fn with_airspace(
        mut self,
        src: crate::data::airspace_query::AirspaceSource,
    ) -> Self {
        self.airspace = Some(src);
        self
    }

    pub fn record_user_input(&self) {
        let now = self.clock.now_secs_f64();
        self.state.lock().unwrap().last_user_input_s = now;
    }

    pub fn check_and_emit(&self) {
        let now = self.clock.now_secs_f64();
        let snapshot_opt = self.pilot.lock().latest_snapshot.clone();
        let Some(snapshot) = snapshot_opt else { return };
        let current_phase = snapshot.phase;
        let current_profiles: Vec<String> = snapshot.active_profiles.clone();
        let current_completed: Vec<String> = snapshot.completed_profiles.clone();
        let current_has_crashed = self
            .bridge
            .as_deref()
            .and_then(|b| b.get_dataref_value(crate::sim::datarefs::HAS_CRASHED.name))
            .map(|v| v >= 0.5);

        let mut st = self.state.lock().unwrap();
        if st.last_seen_profiles.is_none() {
            st.last_seen_phase = current_phase;
            st.last_seen_profiles = Some(current_profiles);
            st.last_seen_completed = Some(current_completed);
            st.last_seen_has_crashed = current_has_crashed;
            return;
        }
        let phase_changed = st.last_seen_phase != current_phase;
        let profiles_changed = st.last_seen_profiles.as_deref() != Some(current_profiles.as_slice());
        // New completions only — if a profile that was already complete
        // stays complete, no need to re-wake the LLM.
        let prev_completed = st.last_seen_completed.as_deref().unwrap_or(&[]);
        let new_completions: Vec<String> = current_completed
            .iter()
            .filter(|n| !prev_completed.contains(n))
            .cloned()
            .collect();
        let completions_changed = !new_completions.is_empty();
        let crash_edge = matches!(
            (st.last_seen_has_crashed, current_has_crashed),
            (Some(false), Some(true)) | (None, Some(true))
        );

        if phase_changed || profiles_changed || completions_changed || crash_edge {
            if phase_changed
                && current_phase == Some(FlightPhase::GoAround)
                && st.last_seen_phase != Some(FlightPhase::GoAround)
            {
                if let Some(bus) = &self.bus {
                    let reason = snapshot.go_around_reason.clone().unwrap_or_else(|| "unknown".to_string());
                    bus.push_log(format!("[safety] go_around triggered: {}", reason));
                }
            }
            if crash_edge {
                if let Some(bus) = &self.bus {
                    bus.push_log("[safety] has_crashed -> 1 (X-Plane reports aircraft crashed)".to_string());
                }
            }
            // Crashes bypass the min-interval throttle — we always want the
            // LLM to hear about a crash the moment it happens.
            if crash_edge || now - st.last_heartbeat_s >= self.min_interval_s {
                let mut reason = describe_change(
                    st.last_seen_phase,
                    current_phase,
                    st.last_seen_profiles.as_deref().unwrap_or(&[]),
                    &current_profiles,
                    &new_completions,
                    snapshot.go_around_reason.as_deref(),
                );
                if crash_edge {
                    if reason == "state changed" {
                        reason = "aircraft crashed".to_string();
                    } else {
                        reason = format!("aircraft crashed; {}", reason);
                    }
                }
                st.last_heartbeat_s = now;
                let status = crate::llm::tools::build_status_payload(
                    Some(&snapshot),
                    self.bridge.as_deref(),
                    self.airspace.as_ref(),
                );
                let text = format!("{} | status={}", reason, serde_json::to_string(&status).unwrap());
                let _ = self.input_queue.send(IncomingMessage::heartbeat(text));
            }
            st.last_seen_phase = current_phase;
            st.last_seen_profiles = Some(current_profiles);
            st.last_seen_completed = Some(current_completed);
            st.last_seen_has_crashed = current_has_crashed;
            return;
        }

        let last_input = st.last_user_input_s.max(st.last_heartbeat_s);
        if now - last_input >= self.heartbeat_interval_s && now >= st.idle_suppress_until_s {
            st.last_heartbeat_s = now;
            // Periodic check-in now threads the bridge through so the
            // airspace section stays populated on the "nothing happened
            // in heartbeat_interval" path — that's exactly when the LLM
            // most benefits from proactive airspace awareness on a long
            // stable leg.
            let status = crate::llm::tools::build_status_payload(
                Some(&snapshot),
                self.bridge.as_deref(),
                self.airspace.as_ref(),
            );
            let text = format!(
                "periodic check-in | status={}",
                serde_json::to_string(&status).unwrap()
            );
            let _ = self.input_queue.send(IncomingMessage::heartbeat(text));
        }
    }
}

fn describe_change(
    old_phase: Option<FlightPhase>,
    new_phase: Option<FlightPhase>,
    old_profiles: &[String],
    new_profiles: &[String],
    new_completions: &[String],
    go_around_reason: Option<&str>,
) -> String {
    let mut parts: Vec<String> = Vec::new();
    if old_phase != new_phase {
        let old = old_phase.map(|p| p.value().to_string()).unwrap_or_else(|| "none".to_string());
        let new = new_phase.map(|p| p.value().to_string()).unwrap_or_else(|| "none".to_string());
        if new_phase == Some(FlightPhase::GoAround) && go_around_reason.is_some() {
            parts.push(format!("phase changed: {} -> {} ({})", old, new, go_around_reason.unwrap()));
        } else {
            parts.push(format!("phase changed: {} -> {}", old, new));
        }
    }
    if old_profiles != new_profiles {
        use std::collections::BTreeSet;
        let olds: BTreeSet<&String> = old_profiles.iter().collect();
        let news: BTreeSet<&String> = new_profiles.iter().collect();
        let added: Vec<String> = news.difference(&olds).map(|s| s.to_string()).collect();
        let removed: Vec<String> = olds.difference(&news).map(|s| s.to_string()).collect();
        let mut profile_parts: Vec<String> = Vec::new();
        if !added.is_empty() {
            profile_parts.push(format!("engaged: {}", added.join(", ")));
        }
        if !removed.is_empty() {
            profile_parts.push(format!("disengaged: {}", removed.join(", ")));
        }
        if !profile_parts.is_empty() {
            parts.push(format!("profiles {}", profile_parts.join("; ")));
        }
    }
    if !new_completions.is_empty() {
        parts.push(format!("completed: {}", new_completions.join(", ")));
    }
    if parts.is_empty() {
        "state changed".to_string()
    } else {
        parts.join("; ")
    }
}

/// Derive a live `ConfigBundle` from a one-shot probe of the running sim.
///
/// The runway frame anchors at the current aircraft heading / position; the
/// airport/runway IDs are cleared (the agent looks them up via sql_query);
/// field elevation falls out as `MSL - AGL` at the probe moment.
pub fn bootstrap_config_from_sample(
    base: ConfigBundle,
    sample: BootstrapSample,
) -> ConfigBundle {
    let course_deg = sample.posi.heading_deg;
    let field_elevation_ft = sample.posi.altitude_msl_m * M_TO_FT - sample.alt_agl_ft;
    let new_runway = Runway {
        id: None,
        course_deg,
        ..base.airport.runway.clone()
    };
    let airport = AirportConfig {
        airport: None,
        field_elevation_ft,
        runway: new_runway,
        mission: base.airport.mission.clone(),
    };
    ConfigBundle { airport, ..base }
}

// ---------------------------------------------------------------------------
// Full live X-Plane runtime glue
// ---------------------------------------------------------------------------

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use crossbeam_channel::unbounded;
use parking_lot::Mutex as PLMutex;

use crate::bus::FileLog;
use crate::core::profiles::PatternFlyProfile;
use crate::llm::conversation::{run_conversation_loop, PilotMode};
use crate::llm::responses_client::ResponsesClient;
use crate::llm::tools::{ToolBridge, ToolContext};
use crate::sim::xplane_bridge::{probe_bootstrap_sample, GeoReference, XPlaneWebBridge};
use crate::tui::{format_snapshot_display, run_tui};
use crate::types::clamp;

#[derive(Debug, Clone)]
pub struct LiveRunConfig {
    pub xplane_host: String,
    pub xplane_port: u16,
    pub llm_model: String,
    /// When `Some`, passed to the Responses API as `reasoning.effort`. When
    /// `None`, the `reasoning` field is omitted entirely.
    pub llm_reasoning_effort: Option<String>,
    pub atc_messages: Vec<String>,
    pub interactive: bool,
    pub control_hz: f64,
    pub status_interval_s: f64,
    pub engage_profile: String,
    pub apt_dat_cache_dir: Option<PathBuf>,
    pub log_file_path: Option<PathBuf>,
    /// CSV + KML target paths for the flight-track recorder. `None`
    /// disables the recorder. CSV is streamed once per second; KML is
    /// emitted on clean shutdown.
    pub track_paths: Option<(PathBuf, PathBuf)>,
    /// Session name used in the KML Document/name element. Lines up with
    /// the timestamp in the log/csv/kml filenames.
    pub session_stem: String,
    pub heartbeat_interval_s: f64,
    pub heartbeat_enabled: bool,
    pub voice_enabled: bool,
    /// Initial pilot persona. Can be swapped at runtime via the `/mode`
    /// TUI command.
    pub pilot_mode: PilotMode,
}

pub fn run_live_xplane(base_config: ConfigBundle, runtime: LiveRunConfig) -> Result<()> {
    let sample = probe_bootstrap_sample(
        &runtime.xplane_host,
        runtime.xplane_port,
        5,
    )
    .context("probing X-Plane bootstrap sample")?;
    let live_config = bootstrap_config_from_sample(base_config, sample);

    let bridge = Arc::new(
        XPlaneWebBridge::new(
            GeoReference {
                threshold_lat_deg: sample.posi.lat_deg,
                threshold_lon_deg: sample.posi.lon_deg,
            },
            &runtime.xplane_host,
            runtime.xplane_port,
            5,
        )
        .context("opening X-Plane web bridge")?,
    );
    let tool_bridge: Arc<dyn ToolBridge> = bridge.clone();

    let pilot_arc = Arc::new(PLMutex::new(PilotCore::new(live_config.clone())));
    engage_startup_profile(&pilot_arc, &live_config, &runtime.engage_profile)?;

    let file_log = runtime
        .log_file_path
        .as_ref()
        .map(|p| FileLog::new(p).map(Arc::new))
        .transpose()
        .context("opening file log")?;
    let bus = match file_log {
        Some(f) => SimBus::with_file_log(!runtime.interactive, f),
        None => SimBus::new(!runtime.interactive),
    };
    if let Some(p) = &runtime.log_file_path {
        bus.push_log(format!("log_file={}", p.display()));
    }
    bus.push_log(format!(
        "bridge connected host={} port={}",
        runtime.xplane_host, runtime.xplane_port
    ));
    let airport_label = live_config
        .airport
        .airport
        .clone()
        .unwrap_or_else(|| "(unset)".to_string());
    let runway_label = live_config
        .airport
        .runway
        .id
        .clone()
        .unwrap_or_else(|| "(unset)".to_string());
    bus.push_log(format!(
        "pilot_reference airport={} runway={} course={:.0} field_elev={:.0}ft",
        airport_label,
        runway_label,
        live_config.airport.runway.course_deg,
        live_config.airport.field_elevation_ft,
    ));
    if runtime.engage_profile != "idle" {
        bus.push_log(format!("startup profile engaged: {}", runtime.engage_profile));
    }

    let (input_tx, input_rx) = unbounded::<IncomingMessage>();
    for msg in &runtime.atc_messages {
        let _ = input_tx.send(IncomingMessage::atc(msg.clone()));
    }

    let llm_client = {
        let mut c = ResponsesClient::new(runtime.llm_model.clone());
        c.reasoning_effort = runtime.llm_reasoning_effort.clone();
        Arc::new(c)
    };
    let cache_stats = llm_client.cache_stats.clone();
    // One DuckDB connection slot, shared between the LLM's tool handlers
    // (via ToolContext.runway_conn) and the heartbeat airspace query. Both
    // serialize on the mutex — DuckDB connections aren't safe for
    // concurrent use across threads.
    let shared_duck_conn: Arc<Mutex<Option<duckdb::Connection>>> = Arc::new(Mutex::new(None));
    let airspace_source = runtime.apt_dat_cache_dir.as_ref().and_then(|dir| {
        if dir.join(crate::data::parquet::AIRSPACES_FILE).exists() {
            Some(crate::data::airspace_query::AirspaceSource {
                conn: shared_duck_conn.clone(),
                cache_dir: dir.clone(),
            })
        } else {
            None
        }
    });
    if airspace_source.is_some() {
        bus.push_log("airspace awareness enabled".to_string());
    }
    let heartbeat_pump = if runtime.heartbeat_enabled {
        let clock: Arc<dyn Clock> = Arc::new(RealClock);
        let mut pump_builder = HeartbeatPump::new(
            pilot_arc.clone(),
            Some(bus.clone()),
            input_tx.clone(),
            runtime.heartbeat_interval_s,
            clock,
        )
        .with_bridge(bridge.clone() as Arc<dyn ToolBridge>);
        if let Some(src) = &airspace_source {
            pump_builder = pump_builder.with_airspace(src.clone());
        }
        let pump = Arc::new(pump_builder);
        bus.push_log(format!(
            "heartbeat pump started interval={:.0}s",
            runtime.heartbeat_interval_s
        ));
        Some(pump)
    } else {
        None
    };

    let tool_ctx = Arc::new(ToolContext {
        pilot: pilot_arc.clone(),
        bridge: Some(tool_bridge),
        config: Arc::new(PLMutex::new(live_config.clone())),
        recent_broadcasts: Arc::new(PLMutex::new(Vec::new())),
        apt_dat_cache_dir: runtime.apt_dat_cache_dir.clone(),
        bus: Some(bus.clone()),
        runway_conn: shared_duck_conn.clone(),
        heartbeat_pump: heartbeat_pump.clone(),
    });

    let llm_stop = Arc::new(AtomicBool::new(false));
    let llm_thread = {
        let client = llm_client.clone();
        let ctx = tool_ctx.clone();
        let rx = input_rx.clone();
        let stop = llm_stop.clone();
        let bus_clone = bus.clone();
        let initial_mode = runtime.pilot_mode;
        thread::Builder::new()
            .name("llm-worker".into())
            .spawn(move || {
                run_conversation_loop(
                    &*client,
                    &ctx,
                    &rx,
                    stop,
                    60,
                    120,
                    initial_mode,
                    Some(&bus_clone),
                );
            })?
    };

    let track_recorder: Option<Arc<PLMutex<crate::track::TrackRecorder>>> =
        match &runtime.track_paths {
            Some((csv, _kml)) => match crate::track::TrackRecorder::new(csv) {
                Ok(r) => {
                    bus.push_log(format!("track_csv={} (1 Hz)", csv.display()));
                    Some(Arc::new(PLMutex::new(r)))
                }
                Err(e) => {
                    bus.push_log(format!("track recorder disabled: {e}"));
                    None
                }
            },
            None => None,
        };

    let control_stop = Arc::new(AtomicBool::new(false));

    if runtime.interactive {
        let pilot_for_loop = pilot_arc.clone();
        let bridge_for_loop = bridge.clone();
        let bus_for_loop = bus.clone();
        let runtime_for_loop = runtime.clone();
        let stop_for_loop = control_stop.clone();
        let pump_for_loop = heartbeat_pump.clone();
        let track_for_loop = track_recorder.clone();
        let control_thread = thread::Builder::new().name("control-loop".into()).spawn(move || {
            run_control_loop(
                bridge_for_loop,
                pilot_for_loop,
                bus_for_loop,
                runtime_for_loop,
                stop_for_loop,
                pump_for_loop,
                track_for_loop,
            );
        })?;
        let ptt = if runtime.voice_enabled {
            match std::env::var("DEEPGRAM_API_KEY").ok() {
                Some(key) if !key.is_empty() => {
                    match crate::transcribe::PttController::spawn(
                        key,
                        Some(bus.clone()),
                        pilot_arc.clone(),
                        tool_ctx.clone(),
                        tool_ctx.bridge.clone(),
                    ) {
                        Ok(c) => {
                            bus.push_log(
                                "voice: ptt ready (deepgram, hold space / tab)".to_string(),
                            );
                            Some(Arc::new(c))
                        }
                        Err(e) => {
                            bus.push_log(format!("voice: disabled ({e})"));
                            None
                        }
                    }
                }
                _ => {
                    bus.push_log("voice: disabled (no DEEPGRAM_API_KEY)".to_string());
                    None
                }
            }
        } else {
            None
        };
        let tui_result = run_tui(
            bus.clone(),
            input_tx.clone(),
            control_stop.clone(),
            pilot_arc.clone(),
            heartbeat_pump.clone(),
            Some(cache_stats),
            ptt,
        );
        control_stop.store(true, Ordering::Release);
        llm_stop.store(true, Ordering::Release);
        let _ = control_thread.join();
        let _ = llm_thread.join();
        finalize_track(&track_recorder, &runtime, &bus);
        bridge.close();
        bus.close();
        tui_result?;
    } else {
        run_control_loop(
            bridge.clone(),
            pilot_arc.clone(),
            bus.clone(),
            runtime.clone(),
            control_stop.clone(),
            heartbeat_pump.clone(),
            track_recorder.clone(),
        );
        llm_stop.store(true, Ordering::Release);
        let _ = llm_thread.join();
        finalize_track(&track_recorder, &runtime, &bus);
        bridge.close();
        bus.close();
    }
    Ok(())
}

/// Flush the in-memory point buffer to KML and log the outcome. Called
/// from both the interactive and headless shutdown paths. Best-effort:
/// failures are logged but not propagated, since the CSV is the durable
/// artifact.
fn finalize_track(
    recorder: &Option<Arc<PLMutex<crate::track::TrackRecorder>>>,
    runtime: &LiveRunConfig,
    bus: &SimBus,
) {
    let Some(rec) = recorder else { return };
    let Some((_, kml_path)) = &runtime.track_paths else {
        return;
    };
    let guard = rec.lock();
    match guard.write_kml(kml_path, &runtime.session_stem) {
        Ok(_) => bus.push_log(format!(
            "track_kml={} ({} points)",
            kml_path.display(),
            guard.len()
        )),
        Err(e) => bus.push_log(format!("track_kml write failed: {e}")),
    }
}

fn run_control_loop(
    bridge: Arc<XPlaneWebBridge>,
    pilot: Arc<PLMutex<PilotCore>>,
    bus: SimBus,
    runtime: LiveRunConfig,
    stop: Arc<AtomicBool>,
    heartbeat_pump: Option<Arc<HeartbeatPump>>,
    track_recorder: Option<Arc<PLMutex<crate::track::TrackRecorder>>>,
) {
    let mut last_state_time_s: Option<f64> = None;
    let target_period = Duration::from_secs_f64(1.0 / runtime.control_hz.max(1.0));
    let mut next_status_s = Instant::now() + Duration::from_secs_f64(runtime.status_interval_s);
    let track_period = Duration::from_secs(1);
    let mut next_track_s = Instant::now();
    while !stop.load(Ordering::Acquire) {
        let loop_start = Instant::now();
        let raw_state = match bridge.read_state() {
            Ok(s) => s,
            Err(e) => {
                bus.push_log(format!("[bridge] read error: {}", e));
                thread::sleep(target_period);
                continue;
            }
        };
        let dt = resolve_dt(raw_state.time_s, last_state_time_s, target_period.as_secs_f64());
        last_state_time_s = Some(raw_state.time_s);
        let commands = {
            let mut p = pilot.lock();
            let result = std::panic::AssertUnwindSafe(|| p.update(&raw_state, dt));
            let (_, commands) = result();
            commands
        };
        if let Err(e) = bridge.write_commands(&commands) {
            bus.push_log(format!("[bridge] write error: {}", e));
        }
        if let Some(pump) = heartbeat_pump.as_ref() {
            pump.check_and_emit();
        }
        if Instant::now() >= next_status_s {
            let snap = pilot.lock().latest_snapshot.clone();
            bus.push_status(format_snapshot_display(snap.as_ref()));
            next_status_s = Instant::now() + Duration::from_secs_f64(runtime.status_interval_s);
        }
        if let Some(recorder) = track_recorder.as_ref() {
            if Instant::now() >= next_track_s {
                sample_track(recorder, &bridge, &pilot, &bus);
                next_track_s = Instant::now() + track_period;
            }
        }
        let elapsed = loop_start.elapsed();
        if elapsed < target_period {
            thread::sleep(target_period - elapsed);
        }
    }
}

/// Pull a single position fix from the bridge + pilot snapshot and hand
/// it to the track recorder. Runs on the control-loop cadence, throttled
/// by the caller to ~1 Hz. Bridge lookup misses (pre-dataref resolution,
/// or just before the first update arrives) silently skip the sample —
/// the recorder will catch the next tick.
fn sample_track(
    recorder: &Arc<PLMutex<crate::track::TrackRecorder>>,
    bridge: &Arc<XPlaneWebBridge>,
    pilot: &Arc<PLMutex<PilotCore>>,
    bus: &SimBus,
) {
    let tool_bridge: &dyn crate::llm::tools::ToolBridge = bridge.as_ref();
    let lat = tool_bridge.get_dataref_value(crate::sim::datarefs::LATITUDE_DEG.name);
    let lon = tool_bridge.get_dataref_value(crate::sim::datarefs::LONGITUDE_DEG.name);
    let (Some(lat), Some(lon)) = (lat, lon) else {
        return;
    };
    let snap = pilot.lock().latest_snapshot.clone();
    let Some(snap) = snap else { return };
    let state = &snap.state;
    let pt = crate::track::TrackPoint {
        wall_time: chrono::Utc::now(),
        t_sim: state.t_sim,
        lat_deg: lat,
        lon_deg: lon,
        alt_msl_ft: state.alt_msl_ft,
        alt_agl_ft: state.alt_agl_ft,
        heading_deg: state.heading_deg,
        track_deg: state.track_deg,
        ias_kt: state.ias_kt,
        gs_kt: state.gs_kt,
        vs_fpm: state.vs_fpm,
        phase: snap
            .phase
            .map(|p| p.value().to_string())
            .unwrap_or_else(|| "none".to_string()),
        on_ground: state.on_ground,
    };
    if let Err(e) = recorder.lock().record(pt) {
        bus.push_log(format!("[track] write error: {e}"));
    }
}

fn engage_startup_profile(
    pilot: &Arc<PLMutex<PilotCore>>,
    config: &ConfigBundle,
    name: &str,
) -> Result<()> {
    match name.to_ascii_lowercase().as_str() {
        "idle" => Ok(()),
        "pattern_fly" => {
            let rf = pilot.lock().runway_frame.clone();
            pilot
                .lock()
                .engage_profile(Box::new(PatternFlyProfile::new(config.clone(), rf)));
            Ok(())
        }
        other => Err(anyhow::anyhow!(
            "Unknown --engage-profile value {:?}; expected one of: idle, pattern_fly",
            other
        )),
    }
}

fn resolve_dt(current: f64, last: Option<f64>, fallback: f64) -> f64 {
    let Some(last) = last else { return fallback };
    let dt = current - last;
    if dt <= 1e-6 {
        return fallback;
    }
    clamp(dt, 0.02, 0.5)
}
