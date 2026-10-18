//! Runs a demo take on the control loop: fires the script's cues
//! (scripted ATC) as their triggers come true, stages each one (OBS
//! scene, what the terminal shows, X-Plane camera), speaks ATC through
//! the radio, and logs a timeline for editing and B-roll.
//!
//! Never blocks the loop: X-Plane commands, the radio and OBS each run on
//! their own thread. ATC waits for a clear frequency; the pilot hears a
//! call (gets the message) when its audio ends, as on a real radio.

use std::collections::VecDeque;
use std::io::Write;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

use crossbeam_channel::{Receiver, Sender};
use parking_lot::Mutex as PLMutex;

use crate::bus::{LogKind, SimBus};
use crate::demo::obs::ObsHandle;
use crate::demo::phraseology::spoken_form;
use crate::demo::radio_voice::{RadioEvent, RadioHandle, Speaker, TxJob};
use crate::demo::script::{CueActions, DemoScript, RecordAction, Roll, TuiView};
use crate::demo::xplane::XpCommander;
use crate::eval_scenario::{CueState, MessageSchedule, Sample, ScenarioMessage, ScheduleInputs};
use crate::llm::conversation::IncomingMessage;
use crate::llm::tools::ToolBridge;

/// From the operator (TUI keys, `/cue`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoCommand {
    /// Send the next cue now.
    Next,
    /// Pause or resume the cues.
    Hold,
    /// Drop the next cue without sending it.
    Skip,
}

/// From the pilot's tools.
#[derive(Debug, Clone)]
pub enum DemoEvent {
    Fly,
    PilotTx(String),
}

/// To the TUI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoUi {
    View(TuiView),
}

/// For the TUI's cue hint.
#[derive(Debug, Clone, Default)]
pub struct DemoStatus {
    pub next: Option<(usize, String, Option<CueState>)>,
    pub total: usize,
    pub held: bool,
    pub standby: bool,
}

/// What the pilot's tools reach: the radio (to speak the pilot's
/// transmissions) and the director's event channel.
pub struct DemoHooks {
    pub radio: Option<RadioHandle>,
    pub events: Sender<DemoEvent>,
}

impl DemoHooks {
    pub fn pilot_transmitted(&self, text: &str) {
        if let Some(r) = &self.radio {
            r.transmit(TxJob { id: 0, speaker: Speaker::Pilot, spoken: spoken_form(text) });
        }
        let _ = self.events.send(DemoEvent::PilotTx(text.to_string()));
    }

    pub fn flew_a_plan(&self) {
        let _ = self.events.send(DemoEvent::Fly);
    }
}

/// Everything the director drives besides the pilot's inbox.
#[derive(Default)]
pub struct Stage {
    pub xp: Option<XpCommander>,
    pub bridge: Option<Arc<dyn ToolBridge>>,
    pub radio: Option<RadioHandle>,
    pub obs: Option<ObsHandle>,
    pub ui: Option<Sender<DemoUi>>,
}

/// An ATC call waiting for (or on) the radio.
struct Pending {
    cue: usize,
    text: String,
    /// The radio's id for it once it's been handed over.
    tx_id: Option<u64>,
}

pub struct DemoDirector {
    script: DemoScript,
    schedule: MessageSchedule,
    input_tx: Sender<IncomingMessage>,
    bus: SimBus,
    radio_log: Arc<PLMutex<Vec<(f64, String)>>>,
    control_rx: Receiver<DemoCommand>,
    events_rx: Receiver<DemoEvent>,
    stage: Stage,
    status: Arc<PLMutex<DemoStatus>>,
    timeline: Option<std::fs::File>,
    /// Sim time of the first tick: cue times count from here.
    t0: Option<f64>,
    rolled: bool,
    standby: bool,
    held: bool,
    was_airborne: bool,
    /// ATC calls in order: waiting for a clear radio, or on the air.
    outbox: VecDeque<Pending>,
    next_tx_id: u64,
    /// What a `hold_s` restores, and when (take time).
    reverts: Vec<(f64, CueActions)>,
    scene: Option<String>,
    view: Option<TuiView>,
    wall_start: Instant,
    last_t: f64,
}

#[allow(clippy::too_many_arguments)]
impl DemoDirector {
    pub fn new(
        script: DemoScript,
        schedule: MessageSchedule,
        input_tx: Sender<IncomingMessage>,
        bus: SimBus,
        radio_log: Arc<PLMutex<Vec<(f64, String)>>>,
        control_rx: Receiver<DemoCommand>,
        events_rx: Receiver<DemoEvent>,
        stage: Stage,
        timeline_path: Option<&Path>,
    ) -> Self {
        let timeline = timeline_path.and_then(|p| std::fs::File::create(p).ok());
        let status = Arc::new(PLMutex::new(DemoStatus { total: script.cues.len(), ..Default::default() }));
        Self {
            script,
            schedule,
            input_tx,
            bus,
            radio_log,
            control_rx,
            events_rx,
            stage,
            status,
            timeline,
            t0: None,
            rolled: false,
            standby: false,
            held: false,
            was_airborne: false,
            outbox: VecDeque::new(),
            next_tx_id: 1,
            reverts: Vec::new(),
            scene: None,
            view: None,
            wall_start: Instant::now(),
            last_t: 0.0,
        }
    }

    pub fn status(&self) -> Arc<PLMutex<DemoStatus>> {
        self.status.clone()
    }

    /// Paused in standby, waiting for the first cue's key: the control
    /// loop leaves the controls alone.
    pub fn is_standby(&self) -> bool {
        self.standby
    }

    /// Before the take: the briefing to the pilot, the start camera, and
    /// the ATC audio rendered ahead.
    pub fn prepare(&mut self) {
        if let Some(b) = &self.script.briefing {
            self.bus.push_log_kind(LogKind::Operator, b.clone());
            let _ = self.input_tx.send(IncomingMessage::operator(b.clone()));
        }
        if let (Some(xp), Some(cam)) = (&self.stage.xp, &self.script.xplane.initial_camera) {
            xp.command(cam, 0.0);
        }
        if let Some(r) = &self.stage.radio {
            for c in &self.script.cues {
                if let Some(text) = &c.msg.atc {
                    r.prewarm(Speaker::Atc, &c.act.say.clone().unwrap_or_else(|| spoken_form(text)));
                }
            }
        }
        self.log(format!("demo {}: {} cues", self.script.id, self.script.cues.len()));
    }

    /// One control-loop tick. `s.t` is X-Plane's sim time.
    pub fn tick(&mut self, s: &Sample, dt: f64) {
        let t0 = *self.t0.get_or_insert(s.t);
        let rel = Sample { t: s.t - t0, ..*s };
        self.last_t = rel.t;
        let (mut force, mut skip) = (false, false);
        for c in self.control_rx.try_iter().collect::<Vec<_>>() {
            match c {
                DemoCommand::Next => force = true,
                DemoCommand::Skip => skip = true,
                DemoCommand::Hold => {
                    self.held = !self.held;
                    self.log(if self.held { "cues held".into() } else { "cues resumed".into() });
                }
            }
        }
        // Standby: settle on the autopilot, then pause until the key.
        if self.script.xplane.roll == Roll::Manual && !self.rolled && !self.standby && rel.t >= self.script.xplane.settle_s {
            self.standby = true;
            if let Some(xp) = &self.stage.xp {
                xp.command("sim/operation/pause_on", 0.0);
            }
            self.log("standby: paused; F6 rolls the take".into());
        }
        self.radio_events();
        self.release_atc();
        if !self.held || force || skip {
            let tx: Vec<f64> = self.radio_log.lock().iter().map(|(t, _)| t - t0).filter(|t| *t >= 0.0).collect();
            let inputs = ScheduleInputs { pilot_tx_times: &tx, force_next: force || skip, rehearse_manual: false };
            for (i, msg) in self.schedule.due_with(&rel, dt, &inputs) {
                if skip {
                    skip = false;
                    self.log(format!("skipped cue {}", self.cue_name(i)));
                    continue;
                }
                self.fire(i, &msg, rel.t);
            }
        }
        for e in self.events_rx.try_iter().collect::<Vec<_>>() {
            match e {
                DemoEvent::Fly => {
                    self.record("fly", serde_json::json!({}));
                    if let Some(h) = self.script.hooks.on_fly.clone() {
                        self.stage_actions(&h, rel.t);
                    }
                }
                DemoEvent::PilotTx(text) => self.record("pilot_tx", serde_json::json!({"text": text})),
            }
        }
        if self.was_airborne && s.on_ground {
            self.record("touchdown", serde_json::json!({}));
            if let Some(h) = self.script.hooks.on_touchdown.clone() {
                self.stage_actions(&h, rel.t);
            }
        }
        self.was_airborne = !s.on_ground;
        let due: Vec<CueActions> = {
            let (now, later): (Vec<_>, Vec<_>) = self.reverts.drain(..).partition(|(at, _)| *at <= rel.t);
            self.reverts = later;
            now.into_iter().map(|(_, a)| a).collect()
        };
        for a in due {
            self.stage_actions(&a, rel.t);
        }
        self.update_status();
    }

    fn cue_name(&self, i: usize) -> String {
        let c = &self.script.cues[i].msg;
        c.id.clone().unwrap_or_else(|| format!("#{i}"))
    }

    fn fire(&mut self, i: usize, msg: &ScenarioMessage, t: f64) {
        if !self.rolled {
            self.rolled = true;
            if self.standby {
                if let Some(xp) = &self.stage.xp {
                    xp.command("sim/operation/pause_off", 0.0);
                }
                self.standby = false;
            }
            self.wall_start = Instant::now();
            self.record("roll", serde_json::json!({}));
            self.log("rolling".into());
        }
        self.record("cue", serde_json::json!({"cue": self.cue_name(i), "atc": msg.atc, "operator": msg.operator}));
        let act = self.script.cues[i].act.clone();
        self.stage_actions(&act, t);
        if let Some(text) = &msg.operator {
            self.bus.push_log_kind(LogKind::Operator, text.clone());
            let _ = self.input_tx.send(IncomingMessage::operator(text.clone()));
        }
        if let Some(text) = &msg.atc {
            self.outbox.push_back(Pending { cue: i, text: text.clone(), tx_id: None });
            self.release_atc();
        }
    }

    /// Put the next ATC call on the air once the frequency is clear (with
    /// no radio, deliver it at once).
    fn release_atc(&mut self) {
        let Some(radio) = self.stage.radio.clone() else {
            while let Some(p) = self.outbox.pop_front() {
                self.on_air(&p.text);
                self.heard(&p.text);
            }
            return;
        };
        let Some(front) = self.outbox.front() else { return };
        if front.tx_id.is_some() || !radio.is_clear(self.script.audio.min_gap_s) {
            return;
        }
        let spoken = self.script.cues[front.cue].act.say.clone().unwrap_or_else(|| spoken_form(&front.text));
        let id = self.next_tx_id;
        self.next_tx_id += 1;
        self.outbox.front_mut().unwrap().tx_id = Some(id);
        radio.transmit(TxJob { id, speaker: Speaker::Atc, spoken });
    }

    /// The front call's audio started (show it) or ended (the pilot has
    /// heard it). Pilot transmissions (id 0) need nothing.
    fn radio_events(&mut self) {
        let Some(radio) = self.stage.radio.clone() else { return };
        for ev in radio.events() {
            if let RadioEvent::Started { speaker: Speaker::Pilot, spoken, .. } = &ev {
                self.record("pilot_audio_start", serde_json::json!({"spoken": spoken}));
                continue;
            }
            let Some(front) = self.outbox.front() else { continue };
            let text = front.text.clone();
            match ev {
                RadioEvent::Started { id, spoken, .. } if front.tx_id == Some(id) => {
                    self.on_air(&text);
                    self.record("atc_audio_start", serde_json::json!({"spoken": spoken}));
                }
                RadioEvent::Done(id) if front.tx_id == Some(id) => {
                    self.outbox.pop_front();
                    self.heard(&text);
                }
                _ => {}
            }
        }
    }

    /// The call starts: on screen.
    fn on_air(&mut self, text: &str) {
        self.bus.push_radio(format!("[atc] {text}"));
        self.record("atc_start", serde_json::json!({"text": text}));
        if let (Some(obs), Some(input)) = (&self.stage.obs, &self.script.obs.caption_input) {
            obs.caption(input, text);
        }
    }

    /// The call ends: the pilot has heard it.
    fn heard(&mut self, text: &str) {
        self.record("atc_end", serde_json::json!({"text": text}));
        let _ = self.input_tx.send(IncomingMessage::atc(text.to_string()));
    }

    /// Scene, terminal view, camera, commands, datarefs, recording.
    fn stage_actions(&mut self, a: &CueActions, t: f64) {
        let mut revert = CueActions::default();
        if let Some(scene) = &a.scene {
            let name = self.script.obs.scene(scene);
            if let Some(obs) = &self.stage.obs {
                obs.scene(&name);
            }
            revert.scene = self.scene.replace(name.clone());
            self.record("scene", serde_json::json!({"scene": name}));
        }
        if let Some(v) = a.tui {
            if let Some(ui) = &self.stage.ui {
                let _ = ui.send(DemoUi::View(v));
            }
            revert.tui = self.view.replace(v);
        }
        if let Some(h) = a.hold_s {
            if revert.scene.is_some() || revert.tui.is_some() {
                self.reverts.push((t + h, revert));
            }
        }
        if let Some(xp) = self.stage.xp.clone() {
            if let Some(cam) = &a.camera {
                xp.command(cam, 0.0);
                self.record("camera", serde_json::json!({"camera": cam}));
            }
            for c in &a.xplane_commands {
                xp.command(&c.name, c.duration_s);
            }
        }
        if !a.datarefs.is_empty() {
            if let Some(b) = &self.stage.bridge {
                let w: Vec<(String, f64)> = a.datarefs.iter().map(|(k, v)| (k.clone(), *v)).collect();
                if let Err(e) = b.write_dataref_values(&w) {
                    self.bus.push_log_kind(LogKind::Error, format!("[demo] datarefs: {e}"));
                }
            }
        }
        if let Some(r) = a.record {
            if let Some(obs) = &self.stage.obs {
                match r {
                    RecordAction::Start => obs.start_record(),
                    RecordAction::Stop => obs.stop_record(),
                    RecordAction::Chapter => obs.chapter(&format!("t={t:.0}")),
                }
            }
            self.record("record", serde_json::json!({"action": format!("{r:?}").to_lowercase()}));
        }
    }

    fn update_status(&mut self) {
        let next = self.schedule.next_pending().map(|(i, m, st)| {
            let name = m.id.clone().unwrap_or_else(|| m.atc.clone().unwrap_or_default().chars().take(24).collect());
            (i, name, Some(st))
        });
        let mut s = self.status.lock();
        s.next = next;
        s.held = self.held;
        s.standby = self.standby;
    }

    fn log(&self, text: String) {
        self.bus.push_log_kind(LogKind::Demo, text);
    }

    /// One timeline line: wall clock, take time, what happened.
    fn record(&mut self, kind: &str, mut detail: serde_json::Value) {
        let Some(f) = self.timeline.as_mut() else { return };
        detail["kind"] = serde_json::json!(kind);
        detail["t"] = serde_json::json!((self.last_t * 10.0).round() / 10.0);
        detail["wall_utc"] = serde_json::json!(chrono::Utc::now().to_rfc3339());
        detail["since_roll_s"] = serde_json::json!(if self.rolled { self.wall_start.elapsed().as_secs_f64() } else { 0.0 });
        let _ = writeln!(f, "{detail}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::llm::conversation::IncomingSource;
    use crate::plan::fixtures::{kvny_georef, KvnyFixture};
    use crate::plan::gazetteer::GAZETTEER;
    use crate::plan::resolve::Resolver;
    use crate::types::{FlightPhase, Vec2};
    use crossbeam_channel::unbounded;

    const SCRIPT: &str = r#"
id: t
airport: KVNY
start: {airborne: {runway: 16R, along_ft: 3000, right_ft: 5500, alt_msl_ft: 1800, heading_deg: 355.5, ias_kt: 80}}
briefing: "brief"
xplane: {settle_s: 2}
hooks:
  on_fly: {tui: log, hold_s: 5}
cues:
  - {id: action, manual: true, tui: spatial}
  - {id: one, when: {cond: elapsed, s: 3}, atc: "Skyhawk 2SP, extend downwind"}
  - {id: two, after_readback: true, delay_s: 2, atc: "Skyhawk 2SP, turn base"}
"#;

    fn sample(t: f64) -> Sample {
        Sample {
            t,
            pos: Vec2::ZERO,
            alt_msl_ft: 1800.0,
            alt_agl_ft: 1000.0,
            track_deg: 355.0,
            phase: Some(FlightPhase::Downwind),
            on_ground: false,
            crashed: false,
            bank_deg: 0.0,
            vs_fpm: 0.0,
        }
    }

    #[test]
    fn standby_roll_cues_readbacks_and_the_fly_hook() {
        let script: DemoScript = serde_yaml_ng::from_str(SCRIPT).unwrap();
        let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &KvnyFixture);
        let schedule = MessageSchedule::with_order(&script.messages(), &mut r, true).unwrap();
        let (input_tx, input_rx) = unbounded();
        let (control_tx, control_rx) = unbounded();
        let (events_tx, events_rx) = unbounded();
        let (ui_tx, ui_rx) = unbounded();
        let radio_log = Arc::new(PLMutex::new(Vec::new()));
        let dir = std::env::temp_dir().join(format!("demo-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let timeline = dir.join("t.demo.jsonl");
        let stage = Stage { ui: Some(ui_tx), ..Default::default() };
        let mut d = DemoDirector::new(
            script,
            schedule,
            input_tx,
            SimBus::new(false),
            radio_log.clone(),
            control_rx,
            events_rx,
            stage,
            Some(&timeline),
        );
        d.prepare();
        let first = input_rx.try_recv().unwrap();
        assert_eq!((first.source, first.text.as_str()), (IncomingSource::Operator, "brief"));

        // X-Plane's clock starts at 1000 s; the take's at the first tick.
        let base = 1000.0;
        for k in 0..30 {
            d.tick(&sample(base + k as f64 * 0.5), 0.5);
        }
        assert!(d.is_standby(), "settled, then standby");
        assert!(input_rx.try_recv().is_err(), "nothing before the roll");

        control_tx.send(DemoCommand::Next).unwrap();
        d.tick(&sample(base + 15.0), 0.0);
        assert!(!d.is_standby(), "F6 rolled the take");
        assert_eq!(ui_rx.try_recv().unwrap(), DemoUi::View(TuiView::Spatial));
        let mut t = base + 15.0;
        let mut atc = Vec::new();
        for _ in 0..40 {
            t += 0.5;
            d.tick(&sample(t), 0.5);
            while let Ok(m) = input_rx.try_recv() {
                atc.push((t, m.text));
            }
            if atc.len() == 1 && radio_log.lock().is_empty() {
                // The pilot reads back.
                radio_log.lock().push((t + 0.25, "extend downwind, 2SP".into()));
            }
        }
        assert_eq!(atc.len(), 2, "{atc:?}");
        assert_eq!(atc[0].1, "Skyhawk 2SP, extend downwind");
        assert!((atc[0].0 - (base + 15.0) - 3.0).abs() <= 1.0, "3 s after the roll: {atc:?}");
        assert!(atc[1].0 >= atc[0].0 + 2.0, "after the readback + 2 s: {atc:?}");

        // A fly call shows the log, then the view comes back after hold_s.
        events_tx.send(DemoEvent::Fly).unwrap();
        d.tick(&sample(t + 0.5), 0.5);
        assert_eq!(ui_rx.try_recv().unwrap(), DemoUi::View(TuiView::Log));
        for k in 0..12 {
            d.tick(&sample(t + 1.0 + k as f64 * 0.5), 0.5);
        }
        assert_eq!(ui_rx.try_recv().unwrap(), DemoUi::View(TuiView::Spatial));

        let lines = std::fs::read_to_string(&timeline).unwrap();
        for kind in ["roll", "cue", "atc_start", "atc_end", "fly"] {
            assert!(lines.contains(&format!("\"kind\":\"{kind}\"")), "{kind} missing:\n{lines}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}
