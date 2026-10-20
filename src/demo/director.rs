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
use crate::eval_scenario::{MessageSchedule, Sample, ScenarioMessage, ScheduleInputs};
use crate::llm::conversation::IncomingMessage;

/// From the operator (TUI keys, `/cue`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoCommand {
    /// Send the next cue now.
    Next,
    /// Pause or resume the cues.
    Hold,
    /// Drop the next cue without sending it.
    Skip,
    /// You switched the terminal's view yourself: it stays there.
    View(TuiView),
}

/// From the pilot's tools and the TUI.
#[derive(Debug, Clone)]
pub enum DemoEvent {
    /// The pilot flew a plan.
    Fly,
    /// ATC (you, from the TUI) called the pilot.
    AtcCall(String),
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
    }

    pub fn flew_a_plan(&self) {
        let _ = self.events.send(DemoEvent::Fly);
    }
}

/// Everything the director drives besides the pilot's inbox.
#[derive(Default)]
pub struct Stage {
    pub xp: Option<XpCommander>,
    pub radio: Option<RadioHandle>,
    pub obs: Option<ObsHandle>,
    /// What the terminal shows.
    pub ui: Option<Sender<TuiView>>,
}

/// An ATC call waiting for (or on) the radio.
struct Pending {
    text: String,
    spoken: String,
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
    timeline: Option<std::fs::File>,
    /// Pilot transmissions already in the timeline.
    radio_seen: usize,
    /// Sim time of the first tick: cue times count from here.
    t0: Option<f64>,
    rolled: bool,
    /// OBS is recording (so the end of the take stops it).
    recording: bool,
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
        Self {
            script,
            schedule,
            input_tx,
            bus,
            radio_log,
            control_rx,
            events_rx,
            stage,
            timeline,
            radio_seen: 0,
            t0: None,
            rolled: false,
            recording: false,
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

    /// Paused in standby, waiting for the first cue's key.
    #[cfg(test)]
    fn is_standby(&self) -> bool {
        self.standby
    }

    /// Before the take: the start camera, and the ATC audio rendered
    /// ahead.
    pub fn prepare(&mut self) {
        if let (Some(xp), Some(cam)) = (&self.stage.xp, &self.script.xplane.initial_camera) {
            xp.command(cam, 0.0);
        }
        if let Some(r) = &self.stage.radio {
            for spoken in self.script.cues.iter().filter_map(|c| c.spoken()) {
                r.prewarm(Speaker::Atc, &spoken);
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
                DemoCommand::View(v) => {
                    self.view = Some(v);
                    for (_, r) in &mut self.reverts {
                        r.tui = None;
                    }
                }
            }
        }
        // Settle on the autopilot, then roll (auto) or pause until the
        // key (manual).
        if !self.rolled && !self.standby && rel.t >= self.script.xplane.settle_s {
            match self.script.xplane.roll {
                Roll::Auto => self.roll(),
                Roll::Manual => {
                    self.standby = true;
                    if let Some(xp) = &self.stage.xp {
                        xp.command("sim/operation/pause_on", 0.0);
                    }
                    self.log("standby: paused; F6 rolls the take".into());
                }
            }
        }
        self.radio_events();
        self.release_atc();
        // The pilot's transmissions: into the timeline, and the latest
        // for readback cues.
        let (new_tx, last_tx) = {
            let log = self.radio_log.lock();
            let new: Vec<String> = log[self.radio_seen.min(log.len())..].iter().map(|(_, m)| m.clone()).collect();
            self.radio_seen = log.len();
            (new, log.last().map(|(t, _)| t - t0))
        };
        for text in new_tx {
            self.record("pilot_tx", serde_json::json!({"text": text}));
        }
        if !self.held || force || skip {
            let inputs = ScheduleInputs { last_pilot_tx_s: last_tx, force_next: force || skip, rehearse_manual: false };
            for (i, msg) in self.schedule.due(&rel, dt, &inputs) {
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
                DemoEvent::AtcCall(text) => self.record("atc_call", serde_json::json!({"text": text})),
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
    }

    fn cue_name(&self, i: usize) -> String {
        let c = &self.script.cues[i].msg;
        c.id.clone().unwrap_or_else(|| format!("#{i}"))
    }

    /// The take starts: out of standby, recording (if the script says),
    /// and the briefing to the pilot.
    fn roll(&mut self) {
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
        if self.script.obs.record {
            self.start_recording();
        }
        if let Some(b) = self.script.briefing.clone() {
            self.bus.push_log_kind(LogKind::Operator, b.clone());
            let _ = self.input_tx.send(IncomingMessage::operator(b));
        }
        if let Some(a) = self.script.hooks.on_roll.clone() {
            self.stage_actions(&a, self.last_t);
        }
    }

    fn start_recording(&mut self) {
        if let Some(obs) = &self.stage.obs {
            obs.start_record();
            self.recording = true;
        }
        self.record("record", serde_json::json!({"action": "start"}));
    }

    fn fire(&mut self, i: usize, msg: &ScenarioMessage, t: f64) {
        if !self.rolled {
            self.roll();
        }
        self.record("cue", serde_json::json!({"cue": self.cue_name(i), "atc": msg.atc, "operator": msg.operator}));
        let act = self.script.cues[i].act.clone();
        self.stage_actions(&act, t);
        if let Some(text) = &msg.operator {
            self.bus.push_log_kind(LogKind::Operator, text.clone());
            let _ = self.input_tx.send(IncomingMessage::operator(text.clone()));
        }
        if let (Some(text), Some(spoken)) = (&msg.atc, self.script.cues[i].spoken()) {
            self.outbox.push_back(Pending { text: text.clone(), spoken, tx_id: None });
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
        let spoken = front.spoken.clone();
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
                let _ = ui.send(v);
            }
            revert.tui = self.view.replace(v);
        }
        if let Some(h) = a.hold_s {
            // A hold during a hold returns to what showed before the first.
            for (_, r) in &mut self.reverts {
                if revert.tui.is_some() {
                    if let Some(v) = r.tui.take() {
                        revert.tui = Some(v);
                    }
                }
                if revert.scene.is_some() {
                    if let Some(name) = r.scene.take() {
                        revert.scene = Some(name);
                    }
                }
            }
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
            if let Some(xp) = &self.stage.xp {
                xp.write_datarefs(a.datarefs.iter().map(|(k, v)| (k.clone(), *v)).collect());
            }
        }
        match a.record {
            Some(RecordAction::Start) if !self.recording => self.start_recording(),
            Some(RecordAction::Stop) => {
                if let Some(obs) = &self.stage.obs {
                    obs.stop_record();
                }
                self.recording = false;
                self.record("record", serde_json::json!({"action": "stop"}));
            }
            Some(RecordAction::Chapter) => {
                if let Some(obs) = &self.stage.obs {
                    obs.chapter(&format!("t={t:.0}"));
                }
                self.record("record", serde_json::json!({"action": "chapter"}));
            }
            _ => {}
        }
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
        // One write per line.
        let mut line = detail.to_string().into_bytes();
        line.push(b'\n');
        let _ = f.write_all(&line);
    }
}

/// The take ends with the session: stop a recording still running.
impl Drop for DemoDirector {
    fn drop(&mut self) {
        self.record("end", serde_json::json!({}));
        if self.recording {
            if let Some(obs) = &self.stage.obs {
                obs.stop_record_and_wait(std::time::Duration::from_secs(3));
            }
        }
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
    fn a_freeform_take_returns_to_its_view_after_the_code_unless_you_switch() {
        let script: DemoScript = serde_yaml_ng::from_str(
            r#"
id: t
airport: KVNY
start: {airborne: {runway: 16R, along_ft: 3000, right_ft: 5500, alt_msl_ft: 1800, heading_deg: 355.5, ias_kt: 80}}
xplane: {settle_s: 1, roll: auto}
hooks:
  on_roll: {tui: spatial}
  on_fly: {tui: log, hold_s: 3}
"#,
        )
        .unwrap();
        let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &KvnyFixture);
        let schedule = MessageSchedule::with_order(&script.messages(), &mut r, true).unwrap();
        let (input_tx, _input_rx) = unbounded();
        let (control_tx, control_rx) = unbounded();
        let (events_tx, events_rx) = unbounded();
        let (ui_tx, ui_rx) = unbounded();
        let stage = Stage { ui: Some(ui_tx), ..Default::default() };
        let mut d = DemoDirector::new(
            script,
            schedule,
            input_tx,
            SimBus::new(false),
            Arc::new(PLMutex::new(Vec::new())),
            control_rx,
            events_rx,
            stage,
            None,
        );
        d.prepare();
        let mut t = 1000.0;
        for _ in 0..10 {
            d.tick(&sample(t), 0.5);
            t += 0.5;
        }
        assert_eq!(ui_rx.try_iter().collect::<Vec<_>>(), [TuiView::Spatial], "rolled into the spatial view");
        events_tx.send(DemoEvent::Fly).unwrap();
        for _ in 0..10 {
            d.tick(&sample(t), 0.5);
            t += 0.5;
        }
        assert_eq!(ui_rx.try_iter().collect::<Vec<_>>(), [TuiView::Log, TuiView::Spatial]);

        // Two plans in a row: back to the spatial view, not stuck on the code.
        events_tx.send(DemoEvent::Fly).unwrap();
        d.tick(&sample(t), 0.5);
        t += 0.5;
        events_tx.send(DemoEvent::Fly).unwrap();
        for _ in 0..10 {
            d.tick(&sample(t), 0.5);
            t += 0.5;
        }
        assert_eq!(ui_rx.try_iter().last(), Some(TuiView::Spatial));

        // Switching the view yourself during the code: it stays switched.
        events_tx.send(DemoEvent::Fly).unwrap();
        d.tick(&sample(t), 0.5);
        t += 0.5;
        control_tx.send(DemoCommand::View(TuiView::Spatial)).unwrap();
        for _ in 0..10 {
            d.tick(&sample(t), 0.5);
            t += 0.5;
        }
        assert_eq!(ui_rx.try_iter().collect::<Vec<_>>(), [TuiView::Log], "no switch back after yours");
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
        let first = input_rx.try_recv().unwrap();
        assert_eq!((first.source, first.text.as_str()), (IncomingSource::Operator, "brief"), "briefed at the roll");
        assert_eq!(ui_rx.try_recv().unwrap(), TuiView::Spatial);
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
        assert_eq!(ui_rx.try_recv().unwrap(), TuiView::Log);
        for k in 0..12 {
            d.tick(&sample(t + 1.0 + k as f64 * 0.5), 0.5);
        }
        assert_eq!(ui_rx.try_recv().unwrap(), TuiView::Spatial);

        // Your calls from the TUI go in the timeline too.
        events_tx.send(DemoEvent::AtcCall("Skyhawk 2SP, turn base now".into())).unwrap();
        d.tick(&sample(t + 8.0), 0.5);
        drop(d);
        let lines = std::fs::read_to_string(&timeline).unwrap();
        assert!(lines.contains("turn base now"), "{lines}");
        for kind in ["roll", "cue", "atc_start", "atc_end", "fly", "atc_call", "end"] {
            assert!(lines.contains(&format!("\"kind\":\"{kind}\"")), "{kind} missing:\n{lines}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}

#[cfg(test)]
mod freeform_tests {
    use super::*;
    use crate::plan::fixtures::{kvny_georef, KvnyFixture};
    use crate::plan::gazetteer::GAZETTEER;
    use crate::plan::resolve::Resolver;
    use crate::types::Vec2;
    use crossbeam_channel::unbounded;

    #[test]
    fn a_take_with_no_cues_rolls_after_settling_and_briefs_the_pilot() {
        let script: DemoScript = serde_yaml_ng::from_str(
            r#"
id: f
airport: KVNY
start: {airborne: {runway: 16R, along_ft: 3000, right_ft: 5500, heading_deg: 355.5, ias_kt: 80}}
briefing: "call the tower"
xplane: {settle_s: 3, roll: auto}
obs: {record: true}
"#,
        )
        .unwrap();
        let mut r = Resolver::new(kvny_georef(), &GAZETTEER, &KvnyFixture);
        let schedule = MessageSchedule::with_order(&script.messages(), &mut r, true).unwrap();
        let (input_tx, input_rx) = unbounded();
        let (_control_tx, control_rx) = unbounded();
        let (_events_tx, events_rx) = unbounded();
        let mut d = DemoDirector::new(
            script,
            schedule,
            input_tx,
            SimBus::new(false),
            Arc::new(PLMutex::new(Vec::new())),
            control_rx,
            events_rx,
            Stage::default(),
            None,
        );
        d.prepare();
        let s = |t: f64| Sample {
            t,
            pos: Vec2::ZERO,
            alt_msl_ft: 1800.0,
            alt_agl_ft: 1000.0,
            track_deg: 355.0,
            phase: None,
            on_ground: false,
            crashed: false,
            bank_deg: 0.0,
            vs_fpm: 0.0,
        };
        d.tick(&s(500.0), 0.5);
        d.tick(&s(502.0), 0.5);
        assert!(input_rx.try_recv().is_err(), "settling");
        d.tick(&s(503.5), 0.5);
        assert!(d.rolled);
        assert_eq!(input_rx.try_recv().unwrap().text, "call the tower");
    }
}
