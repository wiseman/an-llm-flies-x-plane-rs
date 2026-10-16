//! Plan IR: the JSON the Python vocabulary (`python/pilot.py`) emits and
//! `PlanProfile` flies. Mirrors research/pilot-api/DRAFT.md §2.
//!
//! Distances are feet, altitudes MSL feet, speeds knots, angles degrees
//! true. The Python side converts `nm()`, `agl()` and named speeds before
//! anything reaches here.

use serde::{Deserialize, Serialize};

/// A reference to something on the map. Resolved once, in
/// `plan::resolve`, to a point, line or area in world-frame feet.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "ref", rename_all = "snake_case")]
pub enum Ref {
    /// Gazetteer entry by canonical name (the Python side maps aliases).
    Place { name: String },
    #[serde(rename = "latlon")]
    LatLon { lat: f64, lon: f64 },
    /// Runway anatomy: `threshold`, `aim_point`, `midfield`,
    /// `departure_end` (points); `centerline`, `extended_centerline`
    /// (lines).
    Runway { airport: String, runway: String, part: String },
    /// The airport as an area (its runways' footprint plus a margin).
    Airport { ident: String },
    /// Where two lines meet ("the 405/101 interchange").
    Intersection { a: Box<Ref>, b: Box<Ref> },
    /// A point in a runway's frame: `along_ft` past its approach end
    /// (negative = before it), `right_ft` right of its course. How the
    /// pattern library places entry points ("the 45", "abeam midfield").
    RunwayFrame { airport: String, runway: String, along_ft: f64, right_ft: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Dir {
    Left,
    Right,
}

impl Dir {
    pub fn as_str(self) -> &'static str {
        match self {
            Dir::Left => "left",
            Dir::Right => "right",
        }
    }

    /// +1 for right (clockwise), -1 for left.
    pub fn sign(self) -> f64 {
        match self {
            Dir::Left => -1.0,
            Dir::Right => 1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Leg {
    Upwind,
    Crosswind,
    Downwind,
    Base,
    Final,
}

impl Leg {
    pub fn as_str(self) -> &'static str {
        match self {
            Leg::Upwind => "upwind",
            Leg::Crosswind => "crosswind",
            Leg::Downwind => "downwind",
            Leg::Base => "base",
            Leg::Final => "final",
        }
    }
}

/// When something happens. Event-like conditions (crossing, abeam, past)
/// latch: once they've happened since the condition was armed, they stay
/// true.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "cond", rename_all = "snake_case")]
pub enum Cond {
    Now,
    /// As soon as able. Executed like `Now`; the agent may say "unable".
    Asap,
    Never,
    /// A gate the controller opens later ("I'll call your base"). Opened
    /// by `release(token)` in a later `fly` call.
    Atc { token: String },
    /// Until the controller lifts the instruction. Same mechanism as
    /// `Atc`, token `"advised"`.
    Advised,
    /// Over a point (within `radius_ft`, default 1000) or, for points off
    /// the track, abeam it at its closest approach; inside an area; or
    /// crossing a line.
    At { target: Ref, #[serde(default)] radius_ft: Option<f64> },
    Abeam { target: Ref },
    Crossing { target: Ref },
    /// Beyond a point (abeam and continuing) or across a line.
    Past { target: Ref },
    /// On the `side_deg` side of a feature (0 = north of it).
    SideOf { target: Ref, side_deg: f64 },
    Within { target: Ref, distance_ft: f64 },
    Inside { target: Ref },
    Outside { target: Ref },
    /// Within 100 ft of an altitude, or through it.
    Reaching { altitude_ft: f64 },
    AtOrBelow { altitude_ft: f64 },
    AtOrAbove { altitude_ft: f64 },
    /// Over a line (within `width_ft` either side, default 1500), inside
    /// an area, or within `width_ft` of a point.
    Over { target: Ref, #[serde(default)] width_ft: Option<f64> },
    AglBelow { ft: f64 },
    AglAbove { ft: f64 },
    DistanceFlown { ft: f64 },
    Elapsed { s: f64 },
    /// The pattern leg is the current leg.
    OnLeg { leg: Leg },
    /// The turn onto `leg` has started.
    Turning { leg: Leg },
    /// On `leg` and settled on its track.
    Established { leg: Leg },
    AngleBehindWing { target: Ref, deg: f64 },
    /// Traffic conditions. `traffic` is a controller's description ("the
    /// Cherokee on a 3 mile final") or a target id from the status; it
    /// binds to a sensed target when one matches. Unbound traffic never
    /// satisfies these, and shows as an open item.
    InSight { traffic: String },
    /// The traffic has passed abeam: the aircraft is behind it along the
    /// traffic's track (or it has landed and gone).
    Passed { traffic: String },
    /// At least `distance_ft` behind the traffic along its track.
    Behind { traffic: String, #[serde(default)] distance_ft: Option<f64> },
    /// Following the traffic in the pattern: it's on final (or on the
    /// runway) and has passed abeam, or it has landed and gone. The
    /// follower's base-turn cue.
    Following { traffic: String },
    All { conds: Vec<Cond> },
    Any { conds: Vec<Cond> },
    Not { of: Box<Cond> },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Constraint {
    pub id: String,
    /// Human-readable reading for status and the eval transcript.
    pub text: String,
    /// The state to stay in: a state condition (altitude bound, side of a
    /// line, over, inside/outside, within, or a combination).
    pub pred: Cond,
    #[serde(default)]
    pub until: Option<Cond>,
    /// Only enforced once this happens (`remain(...).after(...)`).
    #[serde(default)]
    pub after: Option<Cond>,
    /// `prefer(...)`: a soft constraint, reported but never enforced.
    #[serde(default)]
    pub soft: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Report {
    pub id: String,
    pub text: String,
    pub when: Cond,
}

/// Distance or "until a trigger says otherwise".
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Extension {
    Ft(f64),
    Open(OpenMarker),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OpenMarker {
    Open,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TurnTrigger {
    #[serde(default)]
    pub when: Option<Cond>,
    #[serde(default)]
    pub earliest: Option<Cond>,
    #[serde(default)]
    pub latest: Option<Cond>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Extensions {
    #[serde(default)]
    pub upwind: Option<Extension>,
    #[serde(default)]
    pub crosswind: Option<Extension>,
    #[serde(default)]
    pub downwind: Option<Extension>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Turns {
    #[serde(default)]
    pub crosswind: Option<TurnTrigger>,
    #[serde(default)]
    pub downwind: Option<TurnTrigger>,
    #[serde(default)]
    pub base: Option<TurnTrigger>,
    #[serde(default, rename = "final")]
    pub final_: Option<TurnTrigger>,
}

impl Turns {
    pub fn get(&self, leg: Leg) -> Option<&TurnTrigger> {
        match leg {
            Leg::Crosswind => self.crosswind.as_ref(),
            Leg::Downwind => self.downwind.as_ref(),
            Leg::Base => self.base.as_ref(),
            Leg::Final => self.final_.as_ref(),
            Leg::Upwind => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PatternSpec {
    pub airport: String,
    pub runway: String,
    pub side: Dir,
    /// MSL; `None` = field elevation + the aircraft's pattern altitude.
    #[serde(default)]
    pub altitude_ft: Option<f64>,
    /// Downwind offset from the runway; `None` = aircraft default.
    #[serde(default)]
    pub width_ft: Option<f64>,
    #[serde(default)]
    pub extend: Extensions,
    #[serde(default)]
    pub turns: Turns,
    /// Legs to fly square: `base` squares the turns onto and off base,
    /// `final` just the turn onto final.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub square: Vec<Leg>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LandKind {
    FullStop,
    TouchAndGo,
    /// Down to about 100 ft over the runway, then go around.
    LowApproach,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "step", rename_all = "snake_case")]
pub enum StepKind {
    /// `heading_deg: None` = present heading (captured when the step
    /// starts). `turn` forces the direction of the turn onto it.
    Heading { #[serde(default)] heading_deg: Option<f64>, #[serde(default)] turn: Option<Dir> },
    /// Relative turn (`by_deg`) or turn to a heading (`to_deg`). Done when
    /// the new heading is reached.
    Turn { dir: Dir, #[serde(default)] by_deg: Option<f64>, #[serde(default)] to_deg: Option<f64> },
    /// Fly to a point; done overhead (or at closest approach).
    Direct { target: Ref },
    /// Head toward a point without needing to reach it.
    Toward { target: Ref },
    /// Orbit in place: `degrees` of turn (360 = a "360").
    Orbit { dir: Dir, degrees: f64, #[serde(default)] bank_deg: Option<f64> },
    /// Fly along a line (a road, the shoreline). `direction_deg` picks
    /// which way along it (default: the way the aircraft is heading).
    /// `keep` puts the line off that wing; `side_deg` puts the aircraft
    /// on that side of it ("stay north of the freeway, follow it");
    /// neither = over it. `offset_ft` is how far off (default 2000 with
    /// keep/side). Done at the line's end.
    Follow {
        line: Ref,
        #[serde(default)]
        direction_deg: Option<f64>,
        #[serde(default)]
        keep: Option<Dir>,
        #[serde(default)]
        side_deg: Option<f64>,
        #[serde(default)]
        offset_ft: Option<f64>,
    },
    /// Circle a point (`turns: None` = until something ends the step).
    Circle { target: Ref, dir: Dir, #[serde(default)] radius_ft: Option<f64>, #[serde(default)] turns: Option<f64> },
    /// A rectangular course around an area (or a `size_ft` square on a
    /// point): its edges grown by `offset_ft`, corners rounded, flown
    /// `laps` times (`None` = until something ends the step), at
    /// `altitude_ft` when given.
    RectangularCourse {
        target: Ref,
        dir: Dir,
        #[serde(default)]
        offset_ft: Option<f64>,
        #[serde(default)]
        laps: Option<f64>,
        #[serde(default)]
        size_ft: Option<f64>,
        #[serde(default)]
        altitude_ft: Option<f64>,
    },
    /// Take off from the runway the aircraft is lined up on; done at
    /// 400 ft AGL, climbing on runway heading.
    Takeoff { airport: String, runway: String },
    Climb { altitude_ft: f64, #[serde(default)] speed_kt: Option<f64> },
    Descend { altitude_ft: f64, #[serde(default)] speed_kt: Option<f64> },
    Maintain { altitude_ft: f64 },
    Speed { kt: f64 },
    /// Fly the pattern: join it from wherever the aircraft is (or at
    /// `entry`), then land.
    Pattern {
        pattern: PatternSpec,
        #[serde(default)]
        entry: Option<Leg>,
        #[serde(default = "default_land")]
        land: LandKind,
    },
}

fn default_land() -> LandKind {
    LandKind::FullStop
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Step {
    #[serde(flatten)]
    pub kind: StepKind,
    #[serde(default)]
    pub until: Option<Cond>,
}

/// What one `fly(code)` call produced. Empty fields leave the live plan
/// alone, so a call can amend one thing (`p.turn("base", when=NOW)`)
/// without restating the rest.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PlanDelta {
    /// New step sequence replacing the remaining one; `None` keeps it.
    #[serde(default)]
    pub steps: Option<Vec<Step>>,
    /// Vertical/speed steps stated on their own (not inside a `.then`
    /// chain): they apply immediately, alongside the lateral sequence.
    #[serde(default)]
    pub directives: Vec<Step>,
    /// The live pattern (`p`), edited. Applies to every pattern step for
    /// the same runway, flying or queued.
    #[serde(default)]
    pub pattern: Option<PatternSpec>,
    #[serde(default)]
    pub constraints: Vec<Constraint>,
    #[serde(default)]
    pub reports: Vec<Report>,
    /// ATC gates opened ("base", "advised", ...).
    #[serde(default)]
    pub release: Vec<String>,
    /// Constraint/report ids to drop.
    #[serde(default)]
    pub cancel: Vec<String>,
    /// `plan.replace(...)`: drop all earlier constraints and reports.
    #[serde(default)]
    pub replace: bool,
    /// Go around now (the pattern being flown).
    #[serde(default)]
    pub go_around: bool,
    /// Options ATC granted that the pilot may use ("S-turns approved to
    /// the west"), recorded for the pilot's own reference.
    #[serde(default)]
    pub permits: Vec<String>,
    /// `can(...)`: a question, not a clearance. Preview the change and
    /// report; fly nothing.
    #[serde(default)]
    pub preview: bool,
}

impl PlanDelta {
    pub fn is_empty(&self) -> bool {
        self.steps.is_none()
            && self.directives.is_empty()
            && self.pattern.is_none()
            && self.constraints.is_empty()
            && self.reports.is_empty()
            && self.release.is_empty()
            && self.cancel.is_empty()
            && !self.replace
            && !self.go_around
            && self.permits.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_python_shaped_json() {
        let json = r#"{
          "steps": [
            {"step": "heading", "heading_deg": null, "until": {"cond": "crossing", "target": {"ref": "place", "name": "US-101"}}},
            {"step": "pattern", "pattern": {"airport": "KVNY", "runway": "16R", "side": "right",
               "extend": {"downwind": "open"},
               "turns": {"base": {"when": {"cond": "atc", "token": "base"}}}}}
          ],
          "constraints": [{"id": "c1", "text": "at or below 1800", "pred": {"cond": "at_or_below", "altitude_ft": 1800},
                           "until": {"cond": "side_of", "target": {"ref": "airport", "ident": "KVNY"}, "side_deg": 0}}],
          "release": ["base"]
        }"#;
        let d: PlanDelta = serde_json::from_str(json).unwrap();
        let steps = d.steps.unwrap();
        assert_eq!(steps.len(), 2);
        match &steps[1].kind {
            StepKind::Pattern { pattern, land, .. } => {
                assert_eq!(pattern.extend.downwind, Some(Extension::Open(OpenMarker::Open)));
                assert!(pattern.turns.base.is_some());
                assert_eq!(*land, LandKind::FullStop);
            }
            other => panic!("expected pattern, got {other:?}"),
        }
        assert_eq!(d.release, vec!["base"]);
    }

    #[test]
    fn extension_feet_round_trips() {
        let e: Extension = serde_json::from_str("6076.0").unwrap();
        assert_eq!(e, Extension::Ft(6076.0));
        assert_eq!(serde_json::to_string(&Extension::Open(OpenMarker::Open)).unwrap(), "\"open\"");
    }
}
