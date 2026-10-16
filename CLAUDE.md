# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Live X-Plane run (default backend; requires X-Plane 12 with web API on
# 8086, OPENAI_API_KEY in .env). Runway truth auto-parses from the
# detected X-Plane 12 apt.dat into zstd GeoParquet under
# ~/.cache/sim_pilot/apt-<hash>/; pass --apt-dat-path to override the
# autodetected location. The interactive TUI is default; add --headless
# for scripted / non-interactive runs (otherwise the process blocks on
# the TUI).
cargo run --release --                                # default: --backend xplane, TUI on
cargo run --release -- --headless --operator-message "..." --atc-message "..." # scripted / agent invocation (sent in argv order)
cargo run --release -- --pilot-llm-model gpt-5.4-mini-2026-03-17

# Claude Code as the pilot (no API key; needs the `claude` CLI logged in).
cargo run --release -- --pilot-llm-provider claude-code --pilot-llm-model claude-sonnet-5-5

# Plan scenarios: scripted ATC against the simple sim, scored by checks.
cargo run --release --bin eval -- --scenario evals/plan/B1-turn-base-at-the-118.yaml --pilot-llm-provider claude-code --pilot-llm-model claude-sonnet-5-5 -v
uv run scripts/eval/run_plan_scenarios.py          # all of evals/plan, pass/fail table

# Offline deterministic simulation (no X-Plane, no OpenAI key).
cargo run --release -- --backend simple
cargo run --release -- --backend simple --crosswind-kt 10 --log-csv output/flight.csv --plots-dir output/plots

# Tests.
cargo test                                 # full suite (~350 tests)
cargo test --lib                           # just the library unit tests
cargo test --test test_scenario            # single integration test file
cargo test --test test_mode_transitions downwind_does_not_skip  # single test by name

# Build checks.
cargo check --lib
cargo build --release
```

`.env` is read from the current working directory (best-effort stdlib loader in `main.rs`). Set `OPENAI_API_KEY=sk-...` there — it's only required for `--backend xplane`.

## Architecture

### Layered control stack (bottom-up)

Core principle: **the flight control loop is deterministic; the LLM never touches actuators**. Flow is top-down:

1. **LLM tool-call loop** (`src/llm/`) — `conversation::run_conversation_loop` consumes an `IncomingMessage` from a `crossbeam-channel` and drives an API backend (`LlmBackend`: OpenAI, Anthropic, Gemini) with `tool_schemas()` from `llm/tools.rs`. The `claude-code` provider instead runs `claude_code::run_claude_code_loop`: one headless `claude -p` child per session that runs its own agent loop, with our tools served over an in-process MCP server (`llm/mcp_server.rs`). `llm/brain.rs::PilotBrain` picks the worker; both share `conversation::execute_tool_call`, so tool logging and counters are identical. Each tool handler mutates `PilotCore` state under `parking_lot::Mutex`. The system prompt lives verbatim in `src/llm/system_prompt.md` and is loaded via `include_str!`.

2. **Profile composer** (`src/core/mission_manager.rs::PilotCore`) — owns `active_profiles: Vec<Box<dyn GuidanceProfile>>`. Each tick, `update()` calls `contribute()` on every active profile and merges the resulting `ProfileContribution`s by axis ownership (`Axis::Lateral` / `Vertical` / `Speed`). `engage_profile` auto-displaces any existing profile that owns a conflicting axis, unless that profile's `preemption()` refuses (then nothing is engaged and a `PreemptionRefused` comes back); `disengage_profile` applies the same check and re-adds idle profiles for orphaned axes. `engage_profiles` runs the whole swap under one logical step so a multi-profile engage doesn't orphan axes mid-way. Tools reach a concrete profile with `PilotCore::profile_mut::<T>()` (a safe `Any` downcast), and the landing's exit/arrival plan through `PilotCore::landing_plan_mut()` (the `LandingPlanHolder` trait, implemented by `pattern_fly` before touchdown and `landing_rollout` after). Tools that fly against a newly looked-up runway engage with `engage_profiles_with_config`, so the runway becomes the pilot's reference only if the engage goes through. Cockpit settings outside the flight controls (parking brake, flap handle) are `CockpitSetting`s on `PilotCore`: the latest request from a tool or a profile wins, and each change goes out once in `ActuatorCommands`.

3. **Guidance profiles** (`src/core/profiles.rs`) — `IdleLateralProfile`/`IdleVerticalProfile`/`IdleSpeedProfile` are the default floor. `HeadingHoldProfile`/`AltitudeHoldProfile`/`SpeedHoldProfile` are single-axis. `TakeoffProfile` and `PatternFlyProfile` own all three axes. `PatternFlyProfile` wraps the airborne phase machine (PREFLIGHT → TAKEOFF_ROLL → ROTATE → … → FLARE, plus GO_AROUND) and holds `ModeManager`, `SafetyMonitor`, `RouteManager`, `PatternGeometry`, `L1PathFollower`. At touchdown it (and `DeadStickLandingProfile`) hands off to `LandingRolloutProfile` (`src/core/landing_rollout.rs`), which owns the ground phases ROLLOUT → RUNWAY_EXIT → TAXI_CLEAR, flies any chosen runway exit, refuses preemption while rolling or mid-exit, and at TAXI_CLEAR hands off to idle or to a pre-planned `ArrivalPlan` taxi (from `plan_arrival`).

4. **Low-level control** (`src/control/`) — `BankController` / `PitchController` PID loops with rate damping, `TECSLite` (altitude + speed → pitch + throttle), `CenterlineRolloutController` for takeoff/rollout rudder. `PilotCore::commands_from_guidance` assembles the final `ActuatorCommands`.

**Key invariants:**
- `PilotCore::update(raw_state, dt)` is the single per-tick entry point, backend-agnostic.
- The control loop never calls out to the LLM. Tool handlers briefly lock the pilot; the OpenAI HTTP call runs on a separate thread (`llm-worker`) outside the lock.
- Every profile must own every axis it touches; `engage_profile` enforces unique per-axis ownership.
- A profile that mustn't be interrupted (e.g. a landing rollout still rolling) says so via `GuidanceProfile::preemption()`; never add ad-hoc "is X active?" guards in tool handlers. Hand-offs bypass the check (a profile replacing itself consents).
- `contribute()` returns a `ProfileTick` with an optional `hand_off: Vec<Box<dyn GuidanceProfile>>`. This is how `PatternFlyProfile` and `DeadStickLandingProfile` hand the ground roll to `LandingRolloutProfile` at touchdown, and how `LandingRolloutProfile` hands off to idle or a planned taxi — the replacement happens *after* all profiles have ticked so no profile re-enters `PilotCore` mid-tick. (After a go-around, `PatternFlyProfile` climbs to pattern altitude and re-enters the pattern itself.)

### Plans

In the air the LLM flies with `fly(code)` (research/pilot-api/DRAFT.md). The code is Python in a pilot/ATC vocabulary (`src/plan/python/pilot.py`, documented in the system prompt, `src/llm/system_prompt.md`) that *builds a plan*; it runs once per clearance in a `python3 -I` subprocess (`plan/python.rs`; imports and file access withheld, not a sandbox) and emits the JSON IR in `plan/ir.rs`. `plan/compile.rs` resolves references — runways from apt.dat, roads and landmarks from the hardcoded place lists in `config/places/*.yaml` (`plan/gazetteer.rs`) — and `core/plan_profile.rs::PlanProfile` flies it. It owns all axes, sequences steps, clamps altitude to `remain(...)` limits, and for pattern steps embeds a `PatternFlyProfile` whose turn gates (`clearances`/`triggers`) it drives from the plan's conditions (`plan/cond.rs`), so go-arounds and the rollout hand-off are unchanged. Profile events (`GuidanceProfile::take_events`: report due, constraint not met, step done) ride the status snapshot and wake the LLM through `HeartbeatPump`.

### Two execution backends

- **`simple`** (default) — `sim/scenario.rs::ScenarioRunner` drives `SimpleAircraftModel` (point-mass dynamics in `sim/simple_dynamics.rs`; wheel physics — kinematic nose wheel, tire grip limit, split brakes, rolling/breakaway friction — in `sim/ground_physics.rs`, tuned by `ground_handling:` in the aircraft YAML) at 0.2 s steps. Used by tests + for offline CSV logging / SVG plotting (`sim/logging.rs`, `sim/plotting.rs`).
- **`xplane`** — `live_runner::run_live_xplane` drives real X-Plane 12 via the web API. `sim/xplane_bridge.rs::XPlaneWebBridge` does a one-shot REST call to `/api/capabilities` and `/datarefs?filter[name]=...` to resolve session-local dataref IDs, then opens a WebSocket to `/api/v3`. Incoming `dataref_update_values` are *deltas* — only changed fields — so the bridge merges each update into a cached map. Writes go out as `dataref_set_values`. State/command specs live in `sim/datarefs.rs`.

Both backends share `load_default_config_bundle()`, `PilotCore`, `RunwayFrame`, and `estimate_aircraft_state`. The live runner anchors the pilot at startup via `bootstrap_config_from_sample`, which fills in course from current heading and field elevation from `MSL - AGL`.

### Runtime plumbing (live backend)

- **`SimBus`** (`src/bus.rs`) — thread-safe output bus with status (single-line overwrite) / log (scrolling) / radio (ATC transmissions) channels. Optional `FileLog` writes timestamped transcripts to `output/sim_pilot-*.log`.
- **`tui.rs`** — ratatui-based interactive TUI with four panes (status / log / radio / input). Runs on the main thread by default (pass `--headless` to disable); the control loop moves to its own thread. **`Paragraph::line_count` + `.scroll()`** is used to keep the log/radio panes pinned to the bottom when lines wrap (requires ratatui's `unstable-rendered-line-info` feature — called out in `Cargo.toml`).
- **`HeartbeatPump`** (`src/live_runner.rs`) — wakes the LLM worker on phase or profile changes and after `heartbeat_interval_s` of idle. It sends only the reason; `conversation::coalesce_queued` merges queued heartbeats and attaches a fresh `status={...}` JSON blob at delivery, so the LLM doesn't need to call `get_status` just to see state. Both pilot brains (API and Claude Code) share that receive loop (`for_each_incoming`).
- **`ToolBridge` trait** (`src/llm/tools.rs`) — abstract interface for dataref reads/writes, implemented by `XPlaneWebBridge` and by `FakeBridge` in `tests/test_tool_dispatch.rs`. Tool handlers never see the concrete bridge type.

### Runway-frame coordinates

Everything inside the pilot reasons in a **runway frame** where `x` points down the runway centerline and the threshold is at the origin. `guidance/runway_geometry.rs::RunwayFrame` converts between this frame and the world frame. Pattern geometry (`guidance/pattern_manager.rs`) is built in runway-frame coords and only converted out when feeding `L1PathFollower`. For live X-Plane, a `GeoReference` (lat/lon anchor) handles the flat-earth geodetic-to-feet conversion.

### Runway / airport / comm / taxi data

`src/data/apt_dat.rs` streams X-Plane's `Global Scenery/Global Airports/Earth
nav data/apt.dat` into five Rust structs:

- `ParsedAirport` — rows `1`/`16`/`17` plus `1302 icao_code`/`iata_code`/
  `faa_code`/`datum_lat`/`datum_lon`. Airports missing 1302 `datum_*` are
  backfilled with the centroid of their runway endpoints so every airport
  has a lat/lon.
- `ParsedRunway` — row `100`. Per-end heading and runway length are derived
  from the two endpoint coordinates (initial bearing, haversine). Per-end
  elevation inherits the airport header's single elevation field since
  apt.dat doesn't carry per-end elevation.
- `ParsedComm` — rows `1050..=1056` (ATIS / UNICOM / CD / GND / TWR / APP /
  DEP). Frequency is stored as MHz (6-digit kHz divided by 1000).
- `ParsedTaxiNode` / `ParsedTaxiEdge` — rows `1201` (nodes) and `1202`
  (edges), with 1204 active-zone rows stitched onto the preceding edge.
  8,265 airports globally have a taxi network. Ground-vehicle edges
  (row 1206) are intentionally ignored — those are the baggage-cart paths.

`src/guidance/taxi_route.rs` loads one airport's graph on demand from the
DuckDB parquet views and runs a stage-indexed Dijkstra so constraints like
"take A then D to runway 31" are honoured. Unnamed connector edges before
the first named taxiway and after the last are allowed (gate lead-ins,
runway hold-out). `nearest_node` deliberately filters to nodes that are
incident on at least one 1202 edge — apt.dat shares node ids across the
aircraft graph and the ground-vehicle graph, and an unfiltered nearest
lookup would sometimes snap the aircraft to a baggage-cart-only node and
the planner would declare "no route". Every taxi tool (preview or engage)
plans through `taxi_route::plan_from`, which also owns the runway-start
rules (start on a node ahead, no backtaxi lead-in). Parking routes end at
a lead-in point back along the spot's heading
(`TaxiProfile::pose_lead_in_point`, sized from the nose-wheel turn
radius); the pose phase rolls in along the heading and, if it passes the
spot unsettled, re-approaches via a lead-in point instead of circling.

`src/data/parquet.rs::resolve(apt_dat_path)` parses the source, feeds rows
into an in-memory DuckDB via the `Appender` API, Hilbert-sorts each geometry-
bearing table, and writes three zstd parquet files to
`~/.cache/sim_pilot/apt-<hash>/airports.parquet` (plus `runways.parquet`,
`comms.parquet`). The cache is rebuilt whenever any file is older than
apt.dat. Full build is ~0.8 s for the 360 MB global source (release); cache
reopen is <10 ms.

Only scalar columns are stored in parquet (lat/lon doubles, idents, etc.).
The GEOMETRY columns the LLM queries — `airports.arp`,
`runways.centerline`/`le_threshold`/`he_threshold` — are added back as
computed `ST_Point`/`ST_MakeLine` expressions in `tools.rs::open_apt_dat_parquet`'s
views. This keeps the parquet files portable and avoids a duckdb-rs panic
on `SELECT *` when GEOMETRY columns come off disk. Tests live in-module
(apt_dat parser) and alongside (parquet builder).

### Configuration

All aircraft/airport/gain values live in YAML under `config/` and fold into the public `ConfigBundle` via `src/config.rs`. The YAML files are **embedded via `include_str!`** so the binary is self-contained; on-disk overrides go through `load_config_bundle`.

## Miscellaneous notes

Each release in `CHANGES.md` opens with a **NOTAMs** section — a short, user-facing, pilot-flavored summary of what actually lands in the operator's hands ("Taxi reliably gets you all the way to the hold-short…"). It sits above the usual Added/Changed/Fixed buckets. When cutting a release, fill this out in plain operator language, not changelog-ese; the Added/Changed/Fixed sections stay for the code-level detail. Same rule applies to `NOTAMS.md`, which carries the operator-language summary standalone.

Don't name tools, profiles, or phases in NOTAMs (no `engage_park`, `sleep()`, `pattern_fly`, `RunwayExit`, `idle_*`, etc.). Those are implementation detail and belong in Added/Changed/Fixed. NOTAMs describe what the operator *sees and does*, not the code that delivers it. CLI flags the operator actually types (`--headless`, `/mode`) are fine.

`SQL_QUERY_DESCRIPTION` in `src/llm/tools.rs` includes the sentence "ALWAYS prefix spatial functions with ST_" — without it the LLM calls `POINT(...)` against DuckDB and fails.

Speech: audio streams to Deepgram (Nova 3) over WebSocket while holding space (operator) or tab (ATC, auto-prefixed `[atc] `). Mic capture is `cpal` + a linear resampler to 16 kHz mono PCM16 sent as raw binary frames; the WebSocket client is `src/transcribe/deepgram_ws.rs`; the state machine is `src/transcribe/ptt.rs`. Each PTT hold opens a fresh WS — Nova 3's `keyterm` query params are bound at connect time, so per-hold vocabulary (nearby airport names/ICAOs, current-airport ATC facility labels, runway idents, taxiway names, phase-specific phraseology) is rebuilt on every Start via `src/transcribe/keyterms.rs`. Press/Repeat/Release is detected via Kitty keyboard enhancement flags when the terminal supports them, with a ~180 ms repeat-gap fallback elsewhere. Disable with `--no-voice` or run with no `DEEPGRAM_API_KEY`.

## Testing notes

- Integration tests under `tests/` cover each subsystem (19 test files). `tests/common/mod.rs` provides `default_state()` / `state_with()` helpers for building `AircraftState` fixtures.
- End-to-end smoke test is `test_scenario.rs::nominal_mission_completes_takeoff_to_rollout` — runs the full deterministic mission and checks touchdown position / centerline / sink rate / max final bank against fixed thresholds.
- `test_tool_dispatch.rs::spatial_*` exercises DuckDB's spatial extension — first run downloads it from the DuckDB repo; after that it's cached in `~/.duckdb/extensions/` and runs offline.
