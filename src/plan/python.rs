//! Run agent-written plan code in a CPython subprocess.
//!
//! `pilot.py` (the vocabulary) and `runner.py` are embedded and written to
//! a per-process temp dir on first use. Each call starts `python3 -I
//! runner.py`, sends `{code, context}` on stdin and reads `{ok, delta,
//! printed, error}` from stdout. The runner withholds imports and file
//! access from the plan code, which contains mistakes but is not a
//! sandbox.

use std::io::{Read, Write};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use once_cell::sync::OnceCell;
use serde::Deserialize;
use serde_json::Value;

use crate::plan::ir::PlanDelta;

pub const PILOT_PY: &str = include_str!("python/pilot.py");
pub const RUNNER_PY: &str = include_str!("python/runner.py");

const TIMEOUT: Duration = Duration::from_secs(3);
const MAX_OUTPUT_BYTES: usize = 256 * 1024;

#[derive(Debug, Clone, Deserialize)]
pub struct PythonOutcome {
    pub ok: bool,
    #[serde(default)]
    pub delta: Option<PlanDelta>,
    #[serde(default)]
    pub printed: String,
    #[serde(default)]
    pub error: Option<String>,
}

/// The interpreter: `SIM_PILOT_PYTHON`, else `python3` on PATH.
pub fn python_executable() -> String {
    std::env::var("SIM_PILOT_PYTHON").unwrap_or_else(|_| "python3".to_string())
}

fn script_dir() -> Result<&'static PathBuf> {
    static DIR: OnceCell<PathBuf> = OnceCell::new();
    DIR.get_or_try_init(|| {
        let dir = std::env::temp_dir().join(format!("sim-pilot-plan-{}", std::process::id()));
        std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display()))?;
        std::fs::write(dir.join("pilot.py"), PILOT_PY)?;
        std::fs::write(dir.join("runner.py"), RUNNER_PY)?;
        Ok(dir)
    })
}

/// Run `code` against `context` (see `tools::plan_context`).
pub fn run_plan_code(code: &str, context: &Value) -> Result<PythonOutcome> {
    let dir = script_dir()?;
    let mut child = Command::new(python_executable())
        .arg("-I")
        .arg(dir.join("runner.py"))
        .current_dir(dir)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .with_context(|| format!("starting {} (set SIM_PILOT_PYTHON to override)", python_executable()))?;
    let request = serde_json::json!({ "code": code, "context": context });
    {
        let mut stdin = child.stdin.take().ok_or_else(|| anyhow!("no stdin"))?;
        stdin.write_all(request.to_string().as_bytes())?;
    }
    let mut stdout = child.stdout.take().ok_or_else(|| anyhow!("no stdout"))?;
    let reader = std::thread::spawn(move || {
        let mut buf = Vec::new();
        let _ = (&mut stdout).take(MAX_OUTPUT_BYTES as u64 + 1).read_to_end(&mut buf);
        buf
    });
    let start = Instant::now();
    let status = loop {
        if let Some(status) = child.try_wait()? {
            break status;
        }
        if start.elapsed() > TIMEOUT {
            let _ = child.kill();
            let _ = child.wait();
            return Ok(PythonOutcome {
                ok: false,
                delta: None,
                printed: String::new(),
                error: Some(format!(
                    "plan code ran longer than {}s and was stopped (plans should only build a plan, not loop)",
                    TIMEOUT.as_secs()
                )),
            });
        }
        std::thread::sleep(Duration::from_millis(5));
    };
    let out = reader.join().map_err(|_| anyhow!("reading python output"))?;
    if out.len() > MAX_OUTPUT_BYTES {
        return Err(anyhow!("plan code printed more than {} KB", MAX_OUTPUT_BYTES / 1024));
    }
    if out.is_empty() {
        let mut err = String::new();
        if let Some(mut e) = child.stderr.take() {
            let _ = e.read_to_string(&mut err);
        }
        return Err(anyhow!("python exited ({status}) without a result: {}", err.trim()));
    }
    serde_json::from_slice(&out).with_context(|| {
        format!("parsing runner output: {}", String::from_utf8_lossy(&out[..out.len().min(500)]))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> Value {
        serde_json::json!({
            "airport": "KVNY", "airport_latlon": [34.2098, -118.49], "field_elevation_ft": 800.0,
            "runways": [{"ident": "16R", "course_deg": 175.5, "length_ft": 8020.0, "lat": 34.21891, "lon": -118.4912}],
            "places": [{"name": "CA-118", "kind": "road", "aliases": ["the 118"], "lat": 34.266, "lon": -118.48}],
            "speeds": {"Vy": 74.0, "SLOWEST_PRACTICAL": 67.0, "BEST_FORWARD": 95.0},
            "pattern_defaults": {"downwind_offset_ft": 6000.0},
            "plan": {"pattern": {"airport": "KVNY", "runway": "16R", "side": "right", "extend": {}, "turns": {}},
                     "constraints": [], "reports": []}
        })
    }

    #[test]
    fn runs_code_and_parses_the_delta() {
        let out = run_plan_code(
            "p.turn('base', when=crossing(road('the 118')))\nclimb(to=agl(1000), at=Vy)\nprint(p)",
            &ctx(),
        )
        .unwrap();
        assert!(out.ok, "{:?}", out.error);
        let d = out.delta.unwrap();
        assert!(d.pattern.unwrap().turns.base.is_some());
        assert_eq!(d.directives.len(), 1);
        assert!(out.printed.contains("turn base"), "{}", out.printed);
    }

    /// Translations from research/pilot-api/DRAFT.md §3 (the subset the
    /// executor supports) run and come back as a valid plan.
    #[test]
    fn draft_translations_compile() {
        let snippets = [
            "p.extend('downwind', OPEN); p.turn('base', when=atc('base'))",
            "p.extend('downwind', nm(1))",
            "p.extend('upwind', nm(0.5))",
            "p.width(TIGHT)",
            "p.turn('base', when=crossing(road('the 118')))",
            "p.turn('base', when=NOW)",
            "p.turn('base', latest=crossing(road('the 118')), earliest=angle_behind_wing(rwy.threshold) >= 45)",
            "heading(PRESENT).until(crossing(road('the 118'))).then(join(p))",
            "turn(LEFT, by=20).then(enter(p.base)); remain(north_of(road('the 118'))).until(turning(p.base))",
            "remain(at_or_below(1800)).until(north_of(apt))",
            "remain(at_or_above(2000)).until(turning(p.base))",
            "remain(at_or_below(2500) | west_of(latlon(34.1, -118.6)))",
            "turn(LEFT, to=230); remain(at_or_above(4000))",
            "orbit(RIGHT, 360).then(join(p))",
            "climb(to=agl(1500), at=Vy)",
            "speed(SLOWEST_PRACTICAL)",
            "report('3 miles', when=within(nm(3), rwy.threshold))",
            "direct(latlon(34.25, -118.45)).then(join(p))",
            "plan.replace(); turn(RIGHT, to=170)",
            "print(p.final); print(p)",
            "takeoff(rwy).then(depart(p, via=DOWNWIND)); remain(at_or_below(1800)).until(north_of(apt))",
            "takeoff(rwy).then(heading(RUNWAY).until(crossing(road('the 118')))).then(turn(RIGHT, to=270))",
            "depart(p, via=FORTY_FIVE)",
            "depart(pattern(rwy, LEFT), via=CROSSWIND)",
            "enter(p.downwind, via=FORTY_FIVE)",
            "enter(p.downwind, via=MIDFIELD_CROSSWIND)",
            "enter(p.downwind, via=OVERHEAD)",
            "enter(p.base)",
            "join(p, land=OPTION, on_go=depart(p, via=STRAIGHT_OUT))",
            "land(rwy, LOW_APPROACH)",
            "go_around()",
            "go_around(); p.turn('crosswind', when=ASAP)",
            "plan.change_runway(rwy)",
            "follow(road('the 118'), direction=EAST, keep=LEFT).until(abeam(rwy.threshold)).then(join(p))",
            "follow(road('the 118'), side=NORTH, offset=nm(0.5))",
            "circle(rwy.midfield, LEFT, times=2)",
            "hold_over(rwy.at(along=nm(1), right=nm(-2)))",
            "s_turns(first=RIGHT, n=2)",
            "permit(s_turns(first=LEFT)); permit(turn(RIGHT, by=90))",
            "prefer(over(road('the 118')))",
            "t = traffic('Cherokee on a 3 mile final'); p.follow(t); report('traffic in sight', when=in_sight(t))",
            "p.turn('base', when=behind(traffic('the Cessna'), nm(0.5)))",
            "p.turn('downwind', when=NOW); p.follow(traffic('Falcon on a 6 mile final'))",
            "print(traffic('t1'))",
            "can(p.final_length(SHORT))",
            "p.final_length(nm(1.5)); p.final_length(NORMAL)",
            "p.extend('downwind', nm(1)); p.square('base')",
            "p.square(p.final)",
            "rectangular_course(latlon(34.18, -118.58), LEFT, laps=2, altitude=agl(1000))",
            "rectangular_course(rwy.at(along=nm(3), right=nm(2)), RIGHT, offset=nm(0.5), size=nm(0.5))",
        ];
        for code in snippets {
            let out = run_plan_code(code, &ctx()).unwrap();
            assert!(out.ok, "{code:?}: {:?}", out.error);
            assert!(out.delta.is_some(), "{code:?}");
        }
    }

    #[test]
    fn reports_errors_and_stops_runaway_code() {
        let out = run_plan_code("place('Hollywood Sign')", &ctx()).unwrap();
        assert!(!out.ok);
        assert!(out.error.unwrap().contains("unknown place"));
        let out = run_plan_code("while True:\n    pass", &ctx()).unwrap();
        assert!(!out.ok);
        assert!(out.error.unwrap().contains("longer than"));
    }
}
