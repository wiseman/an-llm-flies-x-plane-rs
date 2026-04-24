# An LLM Flies X-Plane

An LLM Flies X-Plane is an LLM-driven pilot for X-Plane 12 that handles radio
work, taxi and pattern decisions, and high-level mission intent while a
deterministic flight-control stack flies the airplane.

## Concept

The model never writes elevator, aileron, rudder, or throttle directly. A 10 Hz
deterministic control loop owns the actuators at all times. The LLM reads
operator and ATC messages, reasons about the current situation, and calls tools
that engage or reconfigure guidance profiles such as heading hold, cruise,
takeoff, pattern flying, taxi, or line-up-and-wait.

Those profiles own flight-control axes and are merged into a single command set
every tick. `engage_cruise` installs the three single-axis holds atomically;
`engage_takeoff` and `engage_pattern_fly` own all three axes for runway and
traffic-pattern work; `engage_taxi` and `engage_line_up` use the same
deterministic ground controllers to drive the airplane across the airport
surface and onto the runway centerline.

The live backend talks to X-Plane through the built-in web API and uses
X-Plane's own data files as the source of truth. That data is parsed into
DuckDB-backed parquet views for airports, runways, comms, taxiways, parking
spots, and airspaces, so the LLM can query real airport geometry instead of
guessing runway numbers, frequencies, or taxi routes.

## Getting the binary

### Download a prebuilt binary

The pilot runs as a standalone executable — no Rust toolchain required. Grab the
latest release from [GitHub
Releases](https://github.com/wiseman/an-llm-flies-x-plane-rs/releases/latest),
extract the archive, and run `./sim-pilot` (or `sim-pilot.exe` on Windows).

### Building from source

Requires a recent stable Rust toolchain. Always build with `--release` — the
live backend is much less bursty when optimized, and the offline simulator runs
far faster.

```bash
cargo build --release
cargo test  --release        # ~450 tests, runs in ~4 seconds
```

## Running

### Live X-Plane

By default the executable connects to a running X-Plane 12 and needs an API key
for one of the supported LLM model providers in a `.env` file — `OPENAI_API_KEY`
for OpenAI, `ANTHROPIC_API_KEY` for Claude, or `GEMINI_API_KEY` for Gemini.
`DEEPGRAM_API_KEY` is optional, but if you add it then you get voice
transcription.

```bash
cargo run --release --                                      # default: --backend xplane, interactive TUI, OpenAI
cargo run --release -- \
  --pilot-llm-model gpt-5.4-mini-2026-03-17

# Claude (Anthropic)
cargo run --release -- --pilot-llm-provider anthropic
cargo run --release -- --pilot-llm-provider anthropic \
  --pilot-llm-model claude-opus-4-7

# Gemini
cargo run --release -- --pilot-llm-provider gemini
cargo run --release -- --pilot-llm-provider gemini \
  --pilot-llm-model gemini-2.5-pro
```

`--pilot-llm-reasoning-effort low|medium|high` maps to each provider's native
knob: `reasoning.effort` on OpenAI, `thinking.budget_tokens` on Anthropic,
`thinkingConfig.thinkingBudget` on Gemini.

### Offline simulator testing (no X-Plane, no LLM)

Runs a deterministic pattern mission against a point-mass dynamics model. No LLM
of any flavor is invoked — the flight-control stack runs standalone. Nothing to
configure beyond the flag.

```bash
cargo run --release -- --backend simple                     # zero-wind pattern → landing
cargo run --release -- --backend simple --crosswind-kt 10   # 10 kt crosswind
cargo run --release -- --backend simple \
                       --log-csv out/flight.csv \
                       --plots-dir out/plots                # CSV trace + SVG plots
```

Common flags:

| Flag                                                          | Effect                                                                                                                                                                                                                                                                               |
| ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `--headless`                                                  | Skip the TUI; log to stdout and read startup messages from `--atc-message`. Default is the interactive TUI.                                                                                                                                                                          |
| `--apt-dat-path <path>`                                       | Override the autodetected X-Plane 12 `apt.dat`. Caches to `~/.cache/sim_pilot/apt-<hash>/`.                                                                                                                                                                                          |
| `--airspace-txt-path <path>`                                  | Override the autodetected X-Plane 12 `airspace.txt`. Enables the heartbeat `airspace` awareness field.                                                                                                                                                                               |
| `--heartbeat-interval <s>`                                    | Idle cadence for the LLM heartbeat pump (default 30 s). The pump also wakes on phase/profile events.                                                                                                                                                                                 |
| `--no-heartbeat`, `--no-voice`, `--no-track`, `--no-log-file` | Disable individual subsystems.                                                                                                                                                                                                                                                       |
| `--engage-profile pattern_fly`                                | Start with `pattern_fly` already engaged instead of idle holds.                                                                                                                                                                                                                      |
| `--pilot-mode normal\|realistic`                              | Pick the pilot persona at startup. `normal` (default) is the base prompt; `realistic` layers on real-world ATC phraseology, mandatory readbacks, stabilized-approach criteria, and procedural discipline. The mode can also be switched at runtime from the TUI with `/mode <name>`. |

### TUI

The interactive TUI is four stacked panes:

- **STATUS** — phase, active profiles, position, altitude, speed, heading, control surfaces, flap/gear, throttle.
- **LOG** — timestamped transcript of operator messages, ATC, LLM replies, tool calls, and heartbeats. Scrollable.
- **RADIO** — radio transmissions only, labeled with the frequency they went out on.
- **INPUT** — type instructions to the LLM pilot, or act as ATC. emacs/bash-style keybindings; long lines wrap onto additional visual rows.

Keys:

| Key               | Action                                                                                                     |
| ----------------- | ---------------------------------------------------------------------------------------------------------- |
| Enter             | Send the typed line. Prefix with `[atc]` (or the `atc:` colon form) to inject as ATC; default is operator. |
| Hold Space        | Push-to-talk as operator. Releasing finalizes the transcription.                                           |
| Hold Tab          | Push-to-talk as ATC (auto-prefixed `[atc]`).                                                               |
| Ctrl-T            | Toggle log-pane detail (compact vs full).                                                                  |
| PgUp / PgDn / End | Scroll the log pane. Home view auto-pins to the tail.                                                      |

The input pane shows a steady (non-blinking) block cursor when idle. During PTT
the cursor is replaced by an animated audio-level glyph, and a live dimmed
transcription preview streams alongside the text you've typed.

Slash commands (typed into the INPUT pane and submitted with Enter):

Push-to-talk transcription requires the `DEEPGRAM_API_KEY` environment variable
be set. Disable with `--no-voice` or by omitting the key.

### Outputs

Every live run produces four timestamped artifacts in `output/`, sharing one
stem (`sim_pilot-YYYYMMDD-HHMMSS`):

| File   | When written                                        | Contents                                                                                                                                                                |
| ------ | --------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `.log` | Streamed live                                       | Full transcript (operator, ATC, LLM, radio, heartbeats, tool dispatch, system).                                                                                         |
| `.csv` | Streamed live at 1 Hz, flushed per row (crash-safe) | Per-second fixes: wall time, t_sim, lat, lon, alt MSL/AGL, heading, track, IAS, GS, VS, phase, on-ground.                                                               |
| `.kml` | Written on clean shutdown                           | Google-Earth `gx:Track` with absolute MSL altitudes — drag into Earth for 3D replay.                                                                                    |
| `.txt` | Overwritten on every LLM API round                  | Latest pilot-LLM request + response JSON (in the selected provider's native shape) — useful for inspecting tool calls, reasoning, and prompt-cache stats between turns. |

## Tool Catalog

### State And Profile Control

| Tool                   | What it does                                                                                         |
| ---------------------- | ---------------------------------------------------------------------------------------------------- |
| `get_status`           | Returns a JSON snapshot of aircraft state, phase, active profiles, and live position when available. |
| `sleep`                | Ends the current LLM turn while the deterministic pilot keeps flying the active profiles.            |
| `engage_heading_hold`  | Takes ownership of the lateral axis and turns to a target heading, optionally forcing left or right. |
| `engage_altitude_hold` | Takes ownership of the vertical axis and holds a target altitude with TECS.                          |
| `engage_speed_hold`    | Takes ownership of the speed axis and holds a target IAS.                                            |
| `engage_cruise`        | Atomically installs heading, altitude, and speed hold together.                                      |
| `disengage_profile`    | Removes a named profile and lets uncovered axes fall back to idle profiles.                          |
| `list_profiles`        | Lists the currently active profile names.                                                            |

### Takeoff, Pattern, And Landing

| Tool                    | What it does                                                                                                                                      |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `engage_takeoff`        | Verifies runway alignment and starts the deterministic takeoff roll, rotation, and climb.                                                         |
| `takeoff_checklist`     | Reports takeoff readiness, including parking brake, flaps, and other live-state checks.                                                           |
| `engage_pattern_fly`    | Anchors the mission pilot to a real runway and flies a full traffic pattern from a chosen phase.                                                  |
| `extend_pattern_leg`    | Lengthens the downwind or crosswind leg (ATC: "extend your downwind 2 miles").                                                                    |
| `execute_pattern_turn`  | Forces an immediate turn to crosswind/downwind/base/final, rebuilding the next leg from current position.                                         |
| `set_pattern_clearance` | Grants or revokes an ATC clearance gate (turn_crosswind/turn_downwind/turn_base/turn_final/land); revoking holds the aircraft on its current leg. |
| `go_around`             | Commands an immediate go-around during pattern flying.                                                                                            |
| `execute_touch_and_go`  | Arms the next landing to skip rollout braking and transition straight back to takeoff.                                                            |
| `join_pattern`          | Acknowledges a pattern-entry instruction while `pattern_fly` is active.                                                                           |

### Ground, Radio, And Airport Data

| Tool                 | What it does                                                                                                                    |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `tune_radio`         | Tunes `com1` or `com2` to a frequency in MHz.                                                                                   |
| `broadcast_on_radio` | Sends an actual radio transmission; plain text alone does not transmit to ATC.                                                  |
| `set_parking_brake`  | Engages or releases the parking brake.                                                                                          |
| `set_flaps`          | Sets flap handle position to `0`, `10`, `20`, or `30` degrees.                                                                  |
| `engage_line_up`     | Crosses the hold short, enters the runway, aligns to runway heading, and stops in line-up position.                             |
| `engage_taxi`        | Plans and then flies a taxi route to a runway hold-short point with deterministic ground steering.                              |
| `plan_taxi_route`    | Plans a taxi route only, including legs, taxiway sequence, distance, and runway conflict zones.                                 |
| `sql_query`          | Runs read-only SQL against the apt.dat-derived DuckDB views for airports, runways, comms, taxi network, and optional airspaces. |
