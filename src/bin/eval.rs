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
    #[arg(long)]
    airport: String,

    /// apt.dat parking spot name (e.g. "Ramp 7", "FBO Parking"). Must
    /// exist at `--airport`; parking names are not unique globally.
    #[arg(long)]
    parking: String,

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
    #[arg(long = "initial-prompt")]
    initial_prompt: String,

    /// Stop the eval after this many LLM turns. Historical max
    /// observed on real runs is 123 — the default is tighter than
    /// that by design: successful eval flights should be efficient,
    /// and an over-budget run is a signal worth catching.
    #[arg(long, default_value_t = 100)]
    max_turns: usize,

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
        .unwrap_or_else(|| provider.default_model().to_string());
    let pilot_mode: PilotMode = args.pilot_mode.into();

    let apt_dat_cache_dir =
        resolve_apt_dat_cache(args.apt_dat_path, args.airspace_txt_path)?;

    let session_stem = format!("eval-{}", Local::now().format("%Y%m%d-%H%M%S"));
    let session_dir = args.output_dir.join(&session_stem);

    let (source, text) = parse_input_source(&args.initial_prompt);
    let startup_messages = vec![match source.as_str() {
        "atc" => IncomingMessage::atc(text.clone()),
        _ => IncomingMessage::operator(text.clone()),
    }];

    println!("eval session: {}", session_stem);
    println!("session dir:  {}", session_dir.display());
    println!("airport:      {}", args.airport);
    println!("parking:      {:?}", args.parking);
    println!("llm:          {} / {}", provider.as_str(), model);
    println!("mode:         {}", pilot_mode.label());
    println!("prompt ({}): {}", source, text);
    println!();

    let base = load_default_config_bundle();
    let cfg = EvalConfig {
        airport: args.airport,
        parking_spot: args.parking,
        apt_dat_cache_dir,
        llm_provider: provider,
        llm_model: model,
        llm_reasoning_effort: args.pilot_llm_reasoning_effort,
        startup_messages,
        pilot_mode,
        max_turns: args.max_turns,
        max_output_tokens: args.max_output_tokens,
        max_sim_seconds: args.max_sim_seconds,
        dt: args.dt,
        heartbeat_interval_s: args.heartbeat_interval,
        output_dir: session_dir,
        session_stem,
        verbose: args.verbose,
    };

    let result = run_eval(base, cfg)?;

    println!("---");
    println!("outcome:         {:?}", result.outcome.source);
    println!("success:         {}", result.outcome.success);
    println!("summary:         {}", result.outcome.summary);
    println!("turns:           {}", result.turn_count);
    println!(
        "tokens:          in={} cached={} out={}",
        result.total_input_tokens, result.total_cached_tokens, result.total_output_tokens
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
    if let Some(p) = &result.summary_json {
        println!("summary:         {}", p.display());
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
