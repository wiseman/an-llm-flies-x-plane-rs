# xplane-pilot (Rust)

A Rust port of [xplane-pilot](../xplane-pilot/) — an LLM-driven autopilot for
X-Plane 12 that flies a deterministic traffic pattern while an LLM handles ATC
communication, mission decisions, and high-level pilot intent.

<a href="https://www.youtube.com/watch?v=5Z6APyCtDTM">
  <img src="https://img.youtube.com/vi/5Z6APyCtDTM/maxresdefault.jpg" width="600" alt="Demo video">
</a>

## Design

The flight control loop is deterministic and runs at 10 Hz — the LLM never
touches elevator, aileron, rudder, or throttle directly. Instead, the LLM
interprets operator and ATC messages into high-level actions (take off, fly a
heading, enter the pattern at a specific runway, extend downwind, go around,
execute a touch-and-go) by calling tools that mutate the pilot core's profile
stack. Composable guidance profiles own axes (lateral, vertical, speed) and
are merged each tick into a single set of actuator commands. A
`PatternFlyProfile` wraps the full phase machine (TAKEOFF_ROLL through
TAXI_CLEAR, plus GO_AROUND) and handles the entire traffic pattern
autonomously once engaged. Single-axis profiles (`HeadingHoldProfile`,
`AltitudeHoldProfile`, `SpeedHoldProfile`) can be composed for cross-country
cruise legs. The X-Plane bridge communicates via the built-in web API on port
8086 (REST for setup, WebSocket for real-time dataref reads and writes).

## Tools

| Tool | Description |
|------|-------------|
| `get_status` | JSON snapshot of aircraft state, phase, and active profiles |
| `sleep` | End the LLM's turn; control loop keeps flying active profiles |
| `engage_heading_hold` | Lateral heading hold (optional forced turn direction) |
| `engage_altitude_hold` | Vertical altitude hold via TECS (Total Energy Control System) |
| `engage_speed_hold` | Airspeed target hold |
| `engage_cruise` | Atomic combo: heading + altitude + speed hold in one call |
| `engage_pattern_fly` | Full deterministic pattern pilot anchored at a specific runway |
| `engage_takeoff` | Takeoff sequence: full power, rotate at Vr, climb at Vy |
| `takeoff_checklist` | Pre-takeoff readiness check (parking brake, flaps, gear, etc.) |
| `disengage_profile` | Remove a named profile; idle profiles fill orphaned axes |
| `list_profiles` | List currently active profile names |
| `extend_downwind` | Push the base-turn point further out on the downwind leg |
| `turn_base_now` | Force an immediate base turn (rebuilds base leg at current position) |
| `go_around` | Command an immediate go-around |
| `execute_touch_and_go` | Arm a touch-and-go: next touchdown skips braking and re-takes off |
| `cleared_to_land` | Record a landing clearance |
| `join_pattern` | Acknowledge a pattern-join instruction |
| `tune_radio` | Set a COM radio frequency |
| `broadcast_on_radio` | Transmit a message on a COM radio |
| `set_parking_brake` | Engage or release the parking brake |
| `set_flaps` | Set the flap handle position (0/10/20/30) |
| `sql_query` | Read-only SQL against the worldwide runway/airport database (DuckDB) |

## Running

Requires a recent stable Rust toolchain (`cargo` via [rustup](https://rustup.rs)).

```bash
# Build.
cargo build --release

# Run the offline deterministic simulator (no X-Plane needed).
./target/release/sim-pilot

# Run with a crosswind and write CSV + SVG plots.
./target/release/sim-pilot --crosswind-kt 10 \
  --log-csv output/flight.csv --plots-dir output/plots

# Connect to a live X-Plane 12 instance with the interactive TUI.
./target/release/sim-pilot --backend xplane --interactive-atc \
  --runway-csv-path ../xplane-pilot/data/runways.csv

# Send an initial instruction to the LLM at startup.
./target/release/sim-pilot --backend xplane --interactive-atc \
  --runway-csv-path ../xplane-pilot/data/runways.csv \
  --atc-message "take off, fly one lap in the pattern, then land"

# Run tests.
cargo test
```

## Configuration

The live backend requires:

- **X-Plane 12.1.1+** with the web API enabled on port 8086 (Settings > Data Output > Web Server)
- **An OpenAI API key** for the LLM worker that interprets ATC/operator messages
- **A runways CSV** — the Python project ships one at `../xplane-pilot/data/runways.csv` (sourced from [ourairports](https://ourairports.com/data/)); point `--runway-csv-path` at it or at your own copy

Create a `.env` file in the directory you run from (it is gitignored):

```
OPENAI_API_KEY=sk-...
```

Or export the variable directly:

```bash
export OPENAI_API_KEY=sk-...
```

The offline deterministic simulator (`--backend simple`, the default) does not
require an API key or X-Plane.

## Differences from the Python project

This port keeps feature parity with the Python version for the flight control
loop, LLM tool surface, and deterministic scenario runner. One intentional
omission and one addition:

- **No audio / Whisper transcription.** The Python `sim_pilot/speech.py`
  push-to-talk path is not ported. Operator and ATC input is typed only.
- **One extra sentence in the `sql_query` tool description** — "ALWAYS prefix
  spatial functions with ST_ (e.g. `ST_Point`, not `POINT`)" — added after
  observing the LLM write bare `POINT(...)` against DuckDB and fail. Flagged
  in the source.

Behavior of the deterministic pilot is anchored by the integration test
`tests/test_scenario.rs::nominal_mission_completes_takeoff_to_rollout`, which
matches the Python baseline touchdown position, centerline, sink rate, and
max final bank.
