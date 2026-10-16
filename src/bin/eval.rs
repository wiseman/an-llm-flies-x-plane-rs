//! LLM-pilot eval harness entry point.
//!
//! Runs one eval scenario end-to-end against the deterministic simple
//! sim: spawn the plane at a specified parking spot, deliver a mission
//! brief, let the LLM drive through takeoff / pattern / landing / taxi
//! back to parking, and report structured metrics. Designed for
//! iteration on the system prompt or tool surface where the live
//! X-Plane cost is prohibitive.

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, Context, Result};
use chrono::Local;
use clap::{Parser, ValueEnum};

use xplane_pilot::config::load_default_config_bundle;
use xplane_pilot::data::{apt_dat, parquet as data_parquet};
use xplane_pilot::eval_runner::{run_eval, EvalConfig};
use xplane_pilot::llm::backend::LlmProvider;
use xplane_pilot::llm::conversation::{IncomingMessage, PilotMode};
use xplane_pilot::tui::parse_input_source;

#[derive(Parser, Debug)]
#[command(about = "LLM-pilot eval harness (simple sim, sim-time-driven)")]
struct Cli {
    /// ICAO airport ident (e.g. KWHP). Used by the harness to locate
    /// the parking spot — the LLM itself starts cold and rediscovers
    /// the airport via sql_query tools.
    #[arg(long, required_unless_present = "scenario")]
    airport: Option<String>,

    /// apt.dat parking spot name (e.g. "Ramp 7", "FBO Parking"). Must
    /// exist at `--airport`; parking names are not unique globally.
    #[arg(long, required_unless_present = "scenario")]
    parking: Option<String>,

    /// A scripted scenario (evals/plan/*.yaml): start state, ATC calls
    /// fired by time or condition, end condition, and checks scored over
    /// the flown track. Replaces --airport/--parking/--initial-prompt.
    #[arg(long)]
    scenario: Option<PathBuf>,

    /// LLM vendor — mirrors the x-plane harness's flag name.
    #[arg(long = "pilot-llm-provider", value_enum, default_value_t = LlmProvider::OpenAi)]
    pilot_llm_provider: LlmProvider,

    /// Model id for the selected provider. Falls back to the provider's
    /// default when omitted. Mirrors the x-plane harness's flag name.
    #[arg(long = "pilot-llm-model")]
    pilot_llm_model: Option<String>,

    /// Optional reasoning-effort hint (low / medium / high). Mirrors
    /// the x-plane harness's flag name.
    #[arg(long = "pilot-llm-reasoning-effort")]
    pilot_llm_reasoning_effort: Option<String>,

    /// Pilot persona; selects the system prompt.
    #[arg(long = "pilot-mode", value_enum, default_value_t = CliPilotMode::Normal)]
    pilot_mode: CliPilotMode,

    /// The single startup message delivered to the LLM. Prefix with
    /// `[atc]` / `atc:` to deliver it as an ATC transmission (same
    /// parsing as the x-plane harness's TUI input); otherwise it goes
    /// in as an operator message. Required — the LLM starts cold and
    /// this is its only context beyond the system prompt.
    ///
    /// Examples:
    ///   --initial-prompt "Take off, fly a left pattern, land, park."
    ///   --initial-prompt "[atc] Skyhawk 2SP, cleared for takeoff runway 12, left closed traffic"
    #[arg(long = "initial-prompt", required_unless_present = "scenario")]
    initial_prompt: Option<String>,

    /// Stop the eval after this many LLM turns. Historical max
    /// observed on real runs is 123 — the default is tighter than
    /// that by design: successful eval flights should be efficient,
    /// and an over-budget run is a signal worth catching. Defaults to
    /// 100, or 120 with --sim-atc (each controller exchange costs the
    /// pilot a readback and an action).
    #[arg(long)]
    max_turns: Option<usize>,

    /// Stop the eval after this many cumulative output tokens across
    /// all turns. Output tokens dominate cost; historical max observed
    /// is ~10,400 — default sits just above that to flag runs that
    /// drift high. Set 0 to disable.
    #[arg(long, default_value_t = 15_000)]
    max_output_tokens: u64,

    /// Stop the eval after this many simulated seconds.
    #[arg(long, default_value_t = 1800.0)]
    max_sim_seconds: f64,

    /// Sim timestep.
    #[arg(long, default_value_t = 0.2)]
    dt: f64,

    /// Heartbeat idle-cadence in sim-seconds. Default is looser than
    /// the live x-plane harness (30 s) because eval runs are
    /// turn-budget sensitive: a 300 s cadence means `sleep(null)`
    /// effectively yields 300 sim-seconds to the active profiles
    /// before the next heartbeat wakes the model.
    #[arg(long, default_value_t = 300.0)]
    heartbeat_interval: f64,

    /// Output directory for log / transcript / summary.json. A
    /// per-session subdirectory (eval-<stamp>) is created inside.
    #[arg(long, default_value = "output")]
    output_dir: PathBuf,

    /// Optional explicit apt.dat path. Falls back to the default
    /// X-Plane 12 install location.
    #[arg(long)]
    apt_dat_path: Option<PathBuf>,

    /// Optional explicit airspace.txt path. Falls back to the default
    /// X-Plane 12 install location.
    #[arg(long)]
    airspace_txt_path: Option<PathBuf>,

    /// Print the LLM's assistant text, tool calls/results, heartbeats,
    /// and startup info to stdout as the run progresses. The log file
    /// is written regardless of this flag.
    #[arg(long, short = 'v')]
    verbose: bool,

    /// Simulate ATC: a Claude Code controller (on your subscription)
    /// answers the pilot's radio calls at the airport. The sim runs at
    /// 1x while a call waits for its answer.
    #[arg(long)]
    sim_atc: bool,

    /// Model for --sim-atc.
    #[arg(long, default_value = xplane_pilot::llm::sim_atc::DEFAULT_SIM_ATC_MODEL)]
    sim_atc_model: String,
}

#[derive(Copy, Clone, Debug, ValueEnum)]
enum CliPilotMode {
    Normal,
    Realistic,
}

impl From<CliPilotMode> for PilotMode {
    fn from(m: CliPilotMode) -> Self {
        match m {
            CliPilotMode::Normal => PilotMode::Normal,
            CliPilotMode::Realistic => PilotMode::Realistic,
        }
    }
}

fn load_dotenv(path: &Path) {
    let Ok(text) = fs::read_to_string(path) else {
        return;
    };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            let k = k.trim();
            let v = v.trim();
            if !k.is_empty() && std::env::var_os(k).is_none() {
                std::env::set_var(k, v);
            }
        }
    }
}

fn resolve_apt_dat_cache(
    explicit_apt: Option<PathBuf>,
    explicit_airspace: Option<PathBuf>,
) -> Result<PathBuf> {
    let apt = explicit_apt.or_else(apt_dat::default_apt_dat_path).ok_or_else(|| {
        anyhow!("apt.dat not found; pass --apt-dat-path or install X-Plane 12 at the standard location")
    })?;
    if !apt.exists() {
        return Err(anyhow!("apt.dat not found at {}", apt.display()));
    }
    let airspace = explicit_airspace
        .or_else(xplane_pilot::data::airspace::default_airspace_txt_path)
        .filter(|p| p.exists());
    let cache = data_parquet::resolve(&apt, airspace.as_deref())
        .with_context(|| format!("building apt.dat parquet cache from {}", apt.display()))?;
    Ok(cache.dir)
}

fn main() -> Result<()> {
    load_dotenv(Path::new(".env"));
    let args = Cli::parse();

    let provider = args.pilot_llm_provider;
    let model = args
        .pilot_llm_model
        .clone()
        .unwrap_or_else(|| provider.default_model().to_string());
    let pilot_mode: PilotMode = args.pilot_mode.into();

    let apt_dat_cache_dir =
        resolve_apt_dat_cache(args.apt_dat_path.clone(), args.airspace_txt_path.clone())?;

    let session_stem = format!("eval-{}", Local::now().format("%Y%m%d-%H%M%S"));

    let scenario = match &args.scenario {
        Some(p) => Some(xplane_pilot::eval_scenario::load(p)?),
        None => None,
    };
    // Scenario runs often start together (run_plan_scenarios.py), so the
    // id keeps their directories apart.
    let session_stem = match &scenario {
        Some(s) => format!("{session_stem}-{}", s.id),
        None => session_stem,
    };
    let session_dir = args.output_dir.join(&session_stem);
    if let Some(s) = &scenario {
        return run_scenario(args, s.clone(), provider, model, pilot_mode, apt_dat_cache_dir, session_stem, session_dir);
    }
    let initial_prompt = args.initial_prompt.clone().unwrap_or_default();
    let (source, text) = parse_input_source(&initial_prompt);
    let mission_complete_reminder = "When the aircraft reaches the end state described in the mission \
                                     above (parked at a gate, at altitude, holding short, whatever was \
                                     asked), call mission_complete(success=true, summary=...) instead of \
                                     sleep — sleep waits for more work, mission_complete ends the run. \
                                     If you decide the mission cannot be completed, call \
                                     mission_complete(success=false, ...).";
    let startup_messages = vec![
        match source.as_str() {
            "atc" => IncomingMessage::atc(text.clone()),
            _ => IncomingMessage::operator(text.clone()),
        },
        IncomingMessage::operator(mission_complete_reminder.to_string()),
    ];

    println!("eval session: {}", session_stem);
    println!("session dir:  {}", session_dir.display());
    println!("airport:      {}", args.airport.as_deref().unwrap_or(""));
    println!("parking:      {:?}", args.parking.as_deref().unwrap_or(""));
    println!("llm:          {} / {}", provider.as_str(), model);
    println!("mode:         {}", pilot_mode.label());
    if args.sim_atc {
        println!("sim atc:      claude-code / {}", args.sim_atc_model);
    }
    println!("prompt ({}): {}", source, text);
    println!();

    let base = load_default_config_bundle();
    let cfg = EvalConfig {
        airport: args.airport.clone().unwrap_or_default(),
        parking_spot: args.parking.clone().unwrap_or_default(),
        apt_dat_cache_dir,
        llm_provider: provider,
        llm_model: model,
        llm_reasoning_effort: args.pilot_llm_reasoning_effort,
        startup_messages,
        pilot_mode,
        max_turns: args.max_turns.unwrap_or(if args.sim_atc { 120 } else { 100 }),
        max_output_tokens: args.max_output_tokens,
        max_sim_seconds: args.max_sim_seconds,
        dt: args.dt,
        heartbeat_interval_s: args.heartbeat_interval,
        output_dir: session_dir,
        session_stem,
        verbose: args.verbose,
        sim_atc_model: args.sim_atc.then_some(args.sim_atc_model.clone()),
        scenario: None,
    };

    let result = run_eval(base, cfg)?;
    report(&result)
}

#[allow(clippy::too_many_arguments)]
fn run_scenario(
    args: Cli,
    scenario: xplane_pilot::eval_scenario::Scenario,
    provider: LlmProvider,
    model: String,
    pilot_mode: PilotMode,
    apt_dat_cache_dir: PathBuf,
    session_stem: String,
    session_dir: PathBuf,
) -> Result<()> {
    let parking = match &scenario.start {
        xplane_pilot::eval_scenario::Start::Parked { parking } => parking.clone(),
        _ => String::new(),
    };
    let mut startup_messages = Vec::new();
    if let Some(b) = &scenario.briefing {
        startup_messages.push(IncomingMessage::operator(b.clone()));
    }
    println!("eval session: {}", session_stem);
    println!("session dir:  {}", session_dir.display());
    println!("scenario:     {} {:?}", scenario.id, scenario.items);
    println!("airport:      {}", scenario.airport);
    println!("llm:          {} / {}", provider.as_str(), model);
    println!();
    let cfg = EvalConfig {
        airport: scenario.airport.clone(),
        parking_spot: parking,
        apt_dat_cache_dir,
        llm_provider: provider,
        llm_model: model,
        llm_reasoning_effort: args.pilot_llm_reasoning_effort,
        startup_messages,
        pilot_mode,
        max_turns: args.max_turns.unwrap_or(60),
        max_output_tokens: args.max_output_tokens,
        max_sim_seconds: scenario.max_sim_s.unwrap_or(args.max_sim_seconds),
        dt: args.dt,
        heartbeat_interval_s: args.heartbeat_interval,
        output_dir: session_dir,
        session_stem,
        verbose: args.verbose,
        sim_atc_model: args.sim_atc.then_some(args.sim_atc_model),
        scenario: Some(scenario),
    };
    let result = run_eval(load_default_config_bundle(), cfg)?;
    if !result.checks.is_empty() {
        println!("---");
        for c in &result.checks {
            println!("{} {:<55} {}", if c.pass { "PASS" } else { "FAIL" }, c.check, c.detail);
        }
    }
    if let Some(p) = &result.ground_svg {
        println!("ground_svg:      {}", p.display());
    }
    let failed = result.checks.iter().filter(|c| !c.pass).count();
    report(&result)?;
    if failed > 0 {
        return Err(anyhow!("{failed} of {} checks failed", result.checks.len()));
    }
    Ok(())
}

fn report(result: &xplane_pilot::eval_runner::EvalResult) -> Result<()> {
    println!("---");
    println!("outcome:         {:?}", result.outcome.source);
    println!("success:         {}", result.outcome.success);
    println!("summary:         {}", result.outcome.summary);
    println!("turns:           {}", result.turn_count);
    println!(
        "tokens:          in={} cached={} written={} out={}",
        result.total_input_tokens,
        result.total_cached_tokens,
        result.total_cache_creation_tokens,
        result.total_output_tokens
    );
    println!("sim_duration_s:  {:.1}", result.sim_duration_s);
    println!("wall_duration_s: {:.1}", result.wall_duration_s);
    println!(
        "final_phase:     {}",
        result
            .final_phase
            .map(|p| p.value().to_string())
            .unwrap_or_else(|| "(none)".to_string())
    );
    println!("crashed:         {}", result.crashed);
    println!("max_alt_agl_ft:  {:.0}", result.max_alt_agl_ft);
    println!(
        "tool_calls:      {} ({} failed)",
        result.tool_stats.total, result.tool_stats.failed,
    );
    if !result.tool_stats.per_tool.is_empty() {
        println!("tool_breakdown:  {}", result.tool_stats.format_breakdown());
    }
    if let Some(p) = &result.summary_json {
        println!("summary:         {}", p.display());
    }
    if let Some(p) = &result.track_csv {
        println!("track_csv:       {}", p.display());
    }
    if let Some(p) = &result.track_kml {
        println!("track_kml:       {}", p.display());
    }
    if result.outcome.success {
        Ok(())
    } else {
        Err(anyhow!(
            "eval failed: {:?} — {}",
            result.outcome.source,
            result.outcome.summary
        ))
    }
}
