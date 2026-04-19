# An LLM Flies X-Plane

An LLM Flies X-Plane is an LLM-driven pilot for X-Plane 12 that handles radio work, taxi and pattern decisions, and high-level mission intent while a deterministic flight-control stack flies the airplane.

## Concept

The model never writes elevator, aileron, rudder, or throttle directly. A 10 Hz deterministic control loop owns the actuators at all times. The LLM reads operator and ATC messages, reasons about the current situation, and calls tools that engage or reconfigure guidance profiles such as heading hold, cruise, takeoff, pattern flying, taxi, or line-up-and-wait.

Those profiles own flight-control axes and are merged into a single command set every tick. `engage_cruise` installs the three single-axis holds atomically; `engage_takeoff` and `engage_pattern_fly` own all three axes for runway and traffic-pattern work; `engage_taxi` and `engage_line_up` use the same deterministic ground controllers to drive the airplane across the airport surface and onto the runway centerline.

The live backend talks to X-Plane through the built-in web API and uses X-Plane's own `apt.dat` as the source of truth. That data is parsed into DuckDB-backed parquet views for airports, runways, comms, taxi nodes, taxi edges, and optional airspaces, so the LLM can query real airport geometry instead of guessing runway numbers, frequencies, or taxi routes.

## Building

Requires a recent stable Rust toolchain. Release builds are recommended — the offline mission runs far faster and the live backend is less bursty when optimized.

```bash
cargo build --release
cargo test  --release        # ~290 tests, runs in ~3 seconds
```

Live-mode DuckDB uses the spatial extension. The first run that touches the `sql_query` tool downloads it from the DuckDB repo into `~/.duckdb/extensions/`; after that it's cached and runs offline.

## Running

### Offline (no X-Plane, no OpenAI key)

The default backend runs a deterministic pattern mission against a point-mass dynamics model. Nothing to configure.

```bash
cargo run --release --                                      # zero-wind pattern → landing
cargo run --release -- --crosswind-kt 10                    # 10 kt crosswind
cargo run --release -- --log-csv out/flight.csv \
                       --plots-dir out/plots                # CSV trace + SVG plots
```

### Live X-Plane

Requires X-Plane 12 running with the built-in web API enabled on port 8086, and an `OPENAI_API_KEY` (a `.env` file in the working directory is read at startup).

```bash
cargo run --release -- \
  --backend xplane \
  --interactive-atc \
  --llm-model gpt-5.4-mini-2026-03-17
```

Common flags:

| Flag                                  | Effect                                                                                                     |
| ------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `--interactive-atc`                   | Launches the TUI. Without it the session is headless and reads `--atc-message` strings on startup.         |
| `--apt-dat-path <path>`               | Override the autodetected X-Plane 12 `apt.dat`. Caches to `~/.cache/sim_pilot/apt-<hash>/`.                |
| `--airspace-txt-path <path>`          | Override the autodetected X-Plane 12 `airspace.txt`. Enables the heartbeat `airspace` awareness field.     |
| `--heartbeat-interval <s>`            | Idle cadence for the LLM heartbeat pump (default 30 s). The pump also wakes on phase/profile events.       |
| `--no-heartbeat`, `--no-voice`, `--no-track`, `--no-log-file` | Disable individual subsystems.                                                       |
| `--engage-profile pattern_fly`        | Start with `pattern_fly` already engaged instead of idle holds.                                            |

### TUI

The interactive TUI is four stacked panes:

- **STATUS** — phase, active profiles, position, altitude, speed, heading, control surfaces, flap/gear, throttle.
- **LOG** — timestamped transcript of operator messages, ATC, LLM replies, tool calls, and heartbeats. Scrollable.
- **RADIO** — radio transmissions only, labeled with the frequency they went out on.
- **INPUT** — single-line editor for typing to the operator or ATC.

Keys:

| Key                  | Action                                                                                                  |
| -------------------- | ------------------------------------------------------------------------------------------------------- |
| Enter                | Send the typed line. Prefix with `[atc]` (or the `atc:` colon form) to inject as ATC; default is operator. |
| Hold Space           | Push-to-talk as operator. Releasing finalizes the transcription.                                        |
| Hold Tab             | Push-to-talk as ATC (auto-prefixed `[atc]`).                                                            |
| Ctrl-T               | Toggle log-pane detail (compact vs full).                                                               |
| Left / Right         | Move the input cursor one character.                                                                    |
| Up / Down            | Walk backward/forward through the submitted-line history. The in-progress draft is preserved.           |
| Ctrl-A / Ctrl-E      | Smart home / end in the input pane. Repeated presses walk across wrapped visual rows.                   |
| Ctrl-K               | Kill from the cursor to the end of the current visual row.                                              |
| PgUp / PgDn / End    | Scroll the log pane. Home view auto-pins to the tail.                                                   |
| Ctrl-C, Ctrl-D, Esc  | Exit cleanly (flushes the flight-track KML).                                                            |

The input pane shows a steady (non-blinking) block cursor when idle. During PTT the cursor is replaced by an animated audio-level glyph, and a live dimmed transcription preview streams alongside the text you've typed.

Voice transcription streams to the OpenAI Realtime API (`gpt-4o-mini-transcribe`) over WebSocket. Disable with `--no-voice` or by not providing an `OPENAI_API_KEY`.

### Outputs

Every live run produces three timestamped artifacts in `output/`, sharing one stem (`sim_pilot-YYYYMMDD-HHMMSS`):

| File   | When written                                           | Contents                                                                            |
| ------ | ------------------------------------------------------ | ----------------------------------------------------------------------------------- |
| `.log` | Streamed live                                          | Full transcript (operator, ATC, LLM, radio, heartbeats, tool dispatch, system).     |
| `.csv` | Streamed live at 1 Hz, flushed per row (crash-safe)    | Per-second fixes: wall time, t_sim, lat, lon, alt MSL/AGL, heading, track, IAS, GS, VS, phase, on-ground. |
| `.kml` | Written on clean shutdown                              | Google-Earth `gx:Track` with absolute MSL altitudes — drag into Earth for 3D replay. |

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

| Tool                   | What it does                                                                                     |
| ---------------------- | ------------------------------------------------------------------------------------------------ |
| `engage_takeoff`       | Verifies runway alignment and starts the deterministic takeoff roll, rotation, and climb.        |
| `takeoff_checklist`    | Reports takeoff readiness, including parking brake, flaps, and other live-state checks.          |
| `engage_pattern_fly`   | Anchors the mission pilot to a real runway and flies a full traffic pattern from a chosen phase. |
| `extend_downwind`      | Pushes the base-turn point farther out while `pattern_fly` is active.                            |
| `turn_base_now`        | Forces an immediate base turn from downwind.                                                     |
| `go_around`            | Commands an immediate go-around during pattern flying.                                           |
| `execute_touch_and_go` | Arms the next landing to skip rollout braking and transition straight back to takeoff.           |
| `cleared_to_land`      | Records a landing clearance for the active pattern.                                              |
| `join_pattern`         | Acknowledges a pattern-entry instruction while `pattern_fly` is active.                          |

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

### Placeholder Schemas

| Tool                  | Status                                                             |
| --------------------- | ------------------------------------------------------------------ |
| `engage_approach`     | Exposed in the schema but currently returns `not yet implemented`. |
| `engage_route_follow` | Exposed in the schema but currently returns `not yet implemented`. |
