//! A demo script (`demo/*.yaml`): an eval scenario's start, briefing and
//! checks, plus cues (scripted ATC with triggers and staging actions),
//! how to set X-Plane up, the radio voices, OBS scenes and B-roll passes.
//! `to_scenario` turns it into a scenario so the same cues can be
//! rehearsed offline with the eval harness.

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

use crate::eval_scenario::{Check, Scenario, ScenarioMessage, ScenarioWind, Start};

#[derive(Debug, Clone, Deserialize)]
pub struct DemoScript {
    pub id: String,
    #[serde(default)]
    pub description: Option<String>,
    pub airport: String,
    pub start: Start,
    /// Operator message sent to the pilot when the take rolls (so its
    /// first call is on the recording).
    #[serde(default)]
    pub briefing: Option<String>,
    /// Cues without `after` follow the one before them.
    #[serde(default = "yes")]
    pub sequential: bool,
    #[serde(default)]
    pub max_sim_s: Option<f64>,
    #[serde(default)]
    pub end_after_s: Option<f64>,
    /// Checks for an offline rehearsal.
    #[serde(default)]
    pub checks: Vec<Check>,
    #[serde(default)]
    pub wind: Option<ScenarioWind>,
    #[serde(default)]
    pub xplane: XPlaneSetup,
    #[serde(default)]
    pub obs: ObsSetup,
    #[serde(default)]
    pub audio: AudioSetup,
    #[serde(default)]
    pub hooks: Hooks,
    /// Scripted ATC; none for a freeform take, where you're the tower.
    #[serde(default)]
    pub cues: Vec<DemoCue>,
    #[serde(default)]
    pub broll: Vec<BrollPass>,
}

fn yes() -> bool {
    true
}

/// A cue: when it goes out (the scenario message's triggers) and what
/// happens then besides the transmission.
#[derive(Debug, Clone, Deserialize)]
pub struct DemoCue {
    #[serde(flatten)]
    pub msg: ScenarioMessage,
    #[serde(flatten)]
    pub act: CueActions,
}

impl DemoCue {
    /// What the radio voice says for its ATC call: `say`, else the call
    /// in spoken radio form.
    pub fn spoken(&self) -> Option<String> {
        let text = self.msg.atc.as_ref()?;
        Some(self.act.say.clone().unwrap_or_else(|| crate::demo::phraseology::spoken_form(text)))
    }
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct CueActions {
    /// What the radio voice says, when it differs from the text.
    #[serde(default)]
    pub say: Option<String>,
    /// OBS scene (an alias from `obs.scenes`, or a scene name).
    #[serde(default)]
    pub scene: Option<String>,
    /// What the terminal shows.
    #[serde(default)]
    pub tui: Option<TuiView>,
    /// Undo the scene/tui change after this long.
    #[serde(default)]
    pub hold_s: Option<f64>,
    /// X-Plane view command (`sim/view/chase`, ...).
    #[serde(default)]
    pub camera: Option<String>,
    #[serde(default)]
    pub xplane_commands: Vec<XpCommand>,
    #[serde(default)]
    pub datarefs: BTreeMap<String, f64>,
    #[serde(default)]
    pub record: Option<RecordAction>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TuiView {
    Spatial,
    Log,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RecordAction {
    Start,
    Stop,
    Chapter,
}

#[derive(Debug, Clone, Deserialize)]
pub struct XpCommand {
    pub name: String,
    /// Hold it this long (camera moves); 0 = a single press.
    #[serde(default)]
    pub duration_s: f64,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct XPlaneSetup {
    /// Aircraft path for flight init; default the C172 SP.
    #[serde(default)]
    pub aircraft: Option<String>,
    #[serde(default)]
    pub day_of_year: Option<u32>,
    /// Local time, hours (16.5 = 4:30 pm).
    #[serde(default)]
    pub time_in_24_hours: Option<f64>,
    /// X-Plane flight-init `weather` block, passed through.
    #[serde(default)]
    pub weather: Option<serde_json::Value>,
    /// Written after flight init (radios, lights).
    #[serde(default)]
    pub datarefs: BTreeMap<String, f64>,
    /// Seconds to let the autopilot settle before standby.
    #[serde(default = "default_settle_s")]
    pub settle_s: f64,
    #[serde(default)]
    pub roll: Roll,
    /// View command to start in.
    #[serde(default)]
    pub initial_camera: Option<String>,
}

fn default_settle_s() -> f64 {
    8.0
}

/// How a take starts: paused in standby until the first cue's key
/// (`manual`), or rolling by itself once the flight has settled
/// (`auto`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Roll {
    #[default]
    Manual,
    Auto,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct ObsSetup {
    /// Alias -> OBS scene name (`main: "Main"`).
    #[serde(default)]
    pub scenes: BTreeMap<String, String>,
    /// OBS text source showing the current transmission.
    #[serde(default)]
    pub caption_input: Option<String>,
    /// Record from the roll until the take ends (quitting the TUI).
    #[serde(default)]
    pub record: bool,
}

impl ObsSetup {
    /// The OBS scene an alias (or a literal scene name) means.
    pub fn scene(&self, name: &str) -> String {
        self.scenes.get(name).cloned().unwrap_or_else(|| name.to_string())
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct AudioSetup {
    #[serde(default = "Voice::atc")]
    pub atc: Voice,
    #[serde(default = "Voice::pilot")]
    pub pilot: Voice,
    /// Silence between transmissions.
    #[serde(default = "default_min_gap_s")]
    pub min_gap_s: f64,
}

impl Default for AudioSetup {
    fn default() -> Self {
        Self { atc: Voice::atc(), pilot: Voice::pilot(), min_gap_s: default_min_gap_s() }
    }
}

fn default_min_gap_s() -> f64 {
    1.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TtsBackend {
    Elevenlabs,
    Kokoro,
    Say,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Voice {
    pub backend: TtsBackend,
    /// ElevenLabs voice id, Kokoro voice name, or `say` voice.
    pub voice: String,
    #[serde(default)]
    pub speed: Option<f64>,
    /// ElevenLabs `voice_settings`, passed through.
    #[serde(default)]
    pub settings: Option<serde_json::Value>,
    /// Radio filter: `atc` (more noise) or `pilot` (cleaner sidetone).
    #[serde(default)]
    pub preset: Option<String>,
}

impl Voice {
    fn atc() -> Self {
        Self { backend: TtsBackend::Kokoro, voice: "am_michael".into(), speed: Some(1.1), settings: None, preset: Some("atc".into()) }
    }

    fn pilot() -> Self {
        Self { backend: TtsBackend::Kokoro, voice: "am_adam".into(), speed: Some(1.05), settings: None, preset: Some("pilot".into()) }
    }
}

/// Staging on pilot events.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct Hooks {
    /// The pilot flew a plan (show the code).
    #[serde(default)]
    pub on_fly: Option<CueActions>,
    #[serde(default)]
    pub on_touchdown: Option<CueActions>,
    /// The take rolls (the starting view, so `hold_s` has one to return to).
    #[serde(default)]
    pub on_roll: Option<CueActions>,
}

/// A B-roll pass over the take's replay: camera shots on the take's
/// timeline.
#[derive(Debug, Clone, Deserialize)]
pub struct BrollPass {
    pub name: String,
    pub shots: Vec<BrollShot>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct BrollShot {
    /// When, in the take:
    /// - `start` (the roll);
    /// - `cue:<id>`;
    /// - `atc:<words>`: the first ATC call containing them ("atc:base");
    /// - `pilot:<words>`: the first pilot transmission containing them;
    /// - `event:<kind>`: `touchdown`, `fly` (the pilot flew a plan), ...
    ///
    /// Any of them takes `#n` for the nth match (`event:fly#2`).
    pub at: String,
    #[serde(default)]
    pub offset_s: f64,
    pub camera: String,
    #[serde(default)]
    pub commands: Vec<XpCommand>,
}

impl DemoScript {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        serde_yaml_ng::from_str(&text).with_context(|| format!("parsing {}", path.display()))
    }

    /// The scenario that rehearses this script offline.
    pub fn to_scenario(&self) -> Scenario {
        Scenario {
            id: self.id.clone(),
            items: Vec::new(),
            description: self.description.clone(),
            airport: self.airport.clone(),
            start: self.start.clone(),
            briefing: self.briefing.clone(),
            messages: self.messages(),
            max_sim_s: self.max_sim_s,
            end_after_s: self.end_after_s,
            checks: self.checks.clone(),
            traffic: Vec::new(),
            sequential: self.sequential,
            wind: self.wind,
        }
    }

    pub fn messages(&self) -> Vec<ScenarioMessage> {
        self.cues.iter().map(|c| c.msg.clone()).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_freeform_demo_records_from_the_roll_with_no_cues() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("demo/kvny-16r-freeform.yaml");
        let d = DemoScript::load(&path).unwrap();
        assert!(d.cues.is_empty());
        assert!(d.obs.record);
        assert_eq!(d.xplane.roll, Roll::Auto);
        assert!(d.hooks.on_fly.as_ref().is_some_and(|h| h.scene.is_none()), "scenes are yours");
        assert!(d.hooks.on_touchdown.is_none(), "the camera stays in the cockpit");
        assert_eq!(d.hooks.on_roll.and_then(|h| h.tui), Some(TuiView::Spatial), "the code view has a view to return to");
    }

    #[test]
    fn the_kvny_demo_parses_and_rehearses_as_a_scenario() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("demo/kvny-16r.yaml");
        let d = DemoScript::load(&path).unwrap();
        assert!(d.sequential);
        assert_eq!(d.xplane.roll, Roll::Auto, "the take rolls by itself");
        assert!(d.cues.iter().all(|c| !c.msg.manual), "no cue waits on a key");
        assert!(d.cues.iter().any(|c| c.msg.after_readback));
        let s = d.to_scenario();
        assert_eq!(s.messages.len(), d.cues.len());
        assert_eq!(s.airport, "KVNY");
        let mut r = crate::plan::resolve::Resolver::new(
            crate::plan::fixtures::kvny_georef(),
            &crate::plan::gazetteer::GAZETTEER,
            &crate::plan::fixtures::KvnyFixture,
        );
        crate::eval_scenario::MessageSchedule::with_order(&s.messages, &mut r, s.sequential).expect("every cue resolves");
    }
}
