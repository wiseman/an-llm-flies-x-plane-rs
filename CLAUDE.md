# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project origin

This is a **Rust port of the Python `xplane-pilot` project** at `~/src/xplane-pilot/` (package: `sim_pilot`). Every module here has a direct counterpart there — file-level doc comments (`//!`) call out which Python file each Rust module mirrors. When in doubt about intent or behavior, the Python source is the reference.

The Python project README has the user-facing overview (what the app does, tool list, LLM flow). The Python `SPEC.md` has the full design brief for the deterministic pilot.

## Commands

```bash
# Offline deterministic simulation (no X-Plane, no OpenAI key).
cargo run --release --                     # default: zero-wind pattern → landing
cargo run --release -- --crosswind-kt 10 --log-csv output/flight.csv --plots-dir output/plots

# Live X-Plane run (requires X-Plane 12 with web API on 8086, OPENAI_API_KEY in .env).
# Runway truth auto-parses from the detected X-Plane 12 apt.dat into a cached
# CSV under ~/.cache/sim_pilot/; pass --apt-dat-path or --runway-csv-path to override.
cargo run --release -- \
  --backend xplane --interactive-atc \
  --llm-model gpt-5.4-mini-2026-03-17

# Tests.
cargo test                                 # full suite (≈201 tests, ~1s wall-clock)
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

The design philosophy spelled out in the Python `SPEC.md` still applies here: **the flight control loop is deterministic; the LLM never touches actuators**. Flow is top-down:

1. **LLM tool-call loop** (`src/llm/`) — `conversation::run_conversation_loop` consumes an `IncomingMessage` from a `crossbeam-channel` and drives the OpenAI Responses API with `tool_schemas()` from `llm/tools.rs`. Each tool handler mutates `PilotCore` state under `parking_lot::Mutex`. The system prompt lives verbatim in `src/llm/system_prompt.md` and is loaded via `include_str!`.

2. **Profile composer** (`src/core/mission_manager.rs::PilotCore`) — owns `active_profiles: Vec<Box<dyn GuidanceProfile>>`. Each tick, `update()` calls `contribute()` on every active profile and merges the resulting `ProfileContribution`s by axis ownership (`Axis::Lateral` / `Vertical` / `Speed`). `engage_profile` auto-displaces any existing profile that owns a conflicting axis; `disengage_profile` re-adds idle profiles for orphaned axes. `engage_profiles` runs the whole swap under one logical step so callers like `engage_cruise` don't orphan axes mid-way.

3. **Guidance profiles** (`src/core/profiles.rs`) — `IdleLateralProfile`/`IdleVerticalProfile`/`IdleSpeedProfile` are the default floor. `HeadingHoldProfile`/`AltitudeHoldProfile`/`SpeedHoldProfile` are single-axis. `TakeoffProfile` and `PatternFlyProfile` own all three axes. `PatternFlyProfile` wraps the full phase machine (PREFLIGHT → TAKEOFF_ROLL → ROTATE → … → FLARE → ROLLOUT, plus GO_AROUND) and holds `ModeManager`, `SafetyMonitor`, `RouteManager`, `PatternGeometry`, `L1PathFollower`.

4. **Low-level control** (`src/control/`) — `BankController` / `PitchController` PID loops with rate damping, `TECSLite` (altitude + speed → pitch + throttle), `CenterlineRolloutController` for takeoff/rollout rudder. `PilotCore::commands_from_guidance` assembles the final `ActuatorCommands`.

**Key invariants:**
- `PilotCore::update(raw_state, dt)` is the single per-tick entry point, backend-agnostic.
- The control loop never calls out to the LLM. Tool handlers briefly lock the pilot; the OpenAI HTTP call runs on a separate thread (`llm-worker`) outside the lock.
- Every profile must own every axis it touches; `engage_profile` enforces unique per-axis ownership.
- `contribute()` returns a `ProfileTick` with an optional `hand_off: Vec<Box<dyn GuidanceProfile>>`. This is how `PatternFlyProfile` swaps itself for three single-axis holds after a go-around settles — the replacement happens *after* all profiles have ticked so no profile re-enters `PilotCore` mid-tick.

### Two execution backends

- **`simple`** (default) — `sim/scenario.rs::ScenarioRunner` drives `SimpleAircraftModel` (point-mass dynamics in `sim/simple_dynamics.rs`) at 0.2 s steps. Used by tests + for offline CSV logging / SVG plotting (`sim/logging.rs`, `sim/plotting.rs`).
- **`xplane`** — `live_runner::run_live_xplane` drives real X-Plane 12 via the web API. `sim/xplane_bridge.rs::XPlaneWebBridge` does a one-shot REST call to `/api/capabilities` and `/datarefs?filter[name]=...` to resolve session-local dataref IDs, then opens a WebSocket to `/api/v3`. Incoming `dataref_update_values` are *deltas* — only changed fields — so the bridge merges each update into a cached map. Writes go out as `dataref_set_values`. State/command specs live in `sim/datarefs.rs`.

Both backends share `load_default_config_bundle()`, `PilotCore`, `RunwayFrame`, and `estimate_aircraft_state`. The live runner anchors the pilot at startup via `bootstrap_config_from_sample`, which fills in course from current heading and field elevation from `MSL - AGL`.

### Runtime plumbing (live backend)

- **`SimBus`** (`src/bus.rs`) — thread-safe output bus with status (single-line overwrite) / log (scrolling) / radio (ATC transmissions) channels. Optional `FileLog` writes timestamped transcripts to `output/sim_pilot-*.log`.
- **`tui.rs`** — ratatui-based interactive TUI with four panes (status / log / radio / input). Runs on the main thread when `--interactive-atc` is set; the control loop moves to its own thread. **`Paragraph::line_count` + `.scroll()`** is used to keep the log/radio panes pinned to the bottom when lines wrap (requires ratatui's `unstable-rendered-line-info` feature — called out in `Cargo.toml`).
- **`HeartbeatPump`** (`src/live_runner.rs`) — wakes the LLM worker on phase or profile changes and after `heartbeat_interval_s` of idle. Embeds a `build_status_payload` JSON blob in each heartbeat so the LLM doesn't need to call `get_status` just to see state.
- **`ToolBridge` trait** (`src/llm/tools.rs`) — abstract interface for dataref reads/writes, implemented by `XPlaneWebBridge` and by `FakeBridge` in `tests/test_tool_dispatch.rs`. Tool handlers never see the concrete bridge type.

### Runway-frame coordinates

Everything inside the pilot reasons in a **runway frame** where `x` points down the runway centerline and the threshold is at the origin. `guidance/runway_geometry.rs::RunwayFrame` converts between this frame and the world frame. Pattern geometry (`guidance/pattern_manager.rs`) is built in runway-frame coords and only converted out when feeding `L1PathFollower`. For live X-Plane, a `GeoReference` (lat/lon anchor) handles the flat-earth geodetic-to-feet conversion.

### Runway/airport data

`src/data/apt_dat.rs` parses X-Plane's `Global Scenery/Global Airports/Earth
nav data/apt.dat` (rows `1`/`16`/`17` for airports, `100` for land runways,
`1302` for ICAO/IATA/FAA metadata) into the same column layout the DuckDB view
in `llm/tools.rs` expects. Runway length and per-end heading are derived from
the two end coordinates (great-circle distance and initial bearing); per-end
elevation falls back to the airport header's single elevation field since
apt.dat doesn't carry per-end values. `resolve_runways_csv` writes a cached
CSV to `~/.cache/sim_pilot/runways-<hash>.csv` keyed off the apt.dat path and
mtime; rebuilds are ~1 s in release mode for the 360 MB global file. Tests
live in-module.

### Configuration

All aircraft/airport/gain values live in YAML under `config/` and fold into the public `ConfigBundle` via `src/config.rs`. The YAML files are **embedded via `include_str!`** so the binary is self-contained; on-disk overrides go through `load_config_bundle`.

## Behavior parity with the Python reference

When porting behavior changes, check the Python source first: paths are of the form `~/src/xplane-pilot/sim_pilot/<path>.py`. File-level `//!` doc comments in each Rust file name the Python counterpart.

One intentional, documented divergence: `SQL_QUERY_DESCRIPTION` in `src/llm/tools.rs` adds the sentence "ALWAYS prefix spatial functions with ST_" to the otherwise-verbatim Python description — the LLM was observed calling `POINT(...)` against DuckDB and failing. The comment above the constant flags this.

The Python `sim_pilot/speech.py` used batch Whisper. The Rust port instead streams audio to the OpenAI Realtime API (`gpt-4o-mini-transcribe`) while holding space (operator) or tab (ATC, auto-prefixed `[atc] `). Mic capture is `cpal` + a linear resampler to 24 kHz PCM16; the WebSocket client lives in `src/transcribe/realtime_ws.rs`; the state machine in `src/transcribe/ptt.rs`. Press/Repeat/Release is detected via Kitty keyboard enhancement flags when the terminal supports them, with a ~180 ms repeat-gap fallback elsewhere. Disable with `--no-voice` or run with no `OPENAI_API_KEY`.

## Testing notes

- Integration tests under `tests/` mirror the Python `tests/` directory one-to-one (19 test files). `tests/common/mod.rs` provides `default_state()` / `state_with()` helpers analogous to the Python `make_state` factories.
- End-to-end parity is anchored by `test_scenario.rs::nominal_mission_completes_takeoff_to_rollout` — it runs the full deterministic mission and checks touchdown position / centerline / sink rate / max final bank against the Python baseline (differences of ~5% on touchdown x are expected float-precision drift).
- `test_tool_dispatch.rs::spatial_*` exercises DuckDB's spatial extension — first run downloads it from the DuckDB repo; after that it's cached in `~/.duckdb/extensions/` and runs offline.
