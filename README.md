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

# Connect to a live X-Plane 12 instance with the interactive TUI. The standard
# X-Plane 12 install is auto-detected on macOS, and apt.dat is parsed into a
# per-user CSV cache on first run (rebuilds whenever apt.dat changes).
./target/release/sim-pilot --backend xplane --interactive-atc

# Point at a specific apt.dat (e.g. non-standard install location).
./target/release/sim-pilot --backend xplane --interactive-atc \
  --apt-dat-path "/path/to/X-Plane 12/Global Scenery/Global Airports/Earth nav data/apt.dat"

# Send an initial instruction to the LLM at startup.
./target/release/sim-pilot --backend xplane --interactive-atc \
  --atc-message "take off, fly one lap in the pattern, then land"

# Run tests.
cargo test
```

## Configuration

The live backend requires:

- **X-Plane 12.1.1+** with the web API enabled on port 8086 (Settings > Data Output > Web Server)
- **An OpenAI API key** for the LLM worker that interprets ATC/operator messages
- **Runway data** — the preferred source is X-Plane's own `apt.dat` (under `Global Scenery/Global Airports/Earth nav data/`). The pilot auto-detects the standard macOS install and parses it into a cached CSV under `~/.cache/sim_pilot/` on first run (~1 s in release mode for the 360 MB global file; cache is rebuilt whenever `apt.dat` is updated). Override with `--apt-dat-path` if your install is in a non-standard location. apt.dat stores every runway endpoint at 8-decimal (≈1 mm) precision and covers ~31k airports / ~38k runways worldwide. As a fallback, `--runway-csv-path` accepts a hand-curated CSV in the [ourairports](https://ourairports.com/data/) schema (e.g. the one the Python project ships at `../xplane-pilot/data/runways.csv`) — useful only if `apt.dat` is unavailable or you want to test against a snapshot, since the ourairports data is rounded to 3 decimals (≈30 m) and has blank coordinates for many small fields

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
loop, LLM tool surface, and deterministic scenario runner. A couple of
differences worth calling out:

- **Streaming push-to-talk instead of batch Whisper.** Hold **space** in the
  interactive TUI to dictate operator input; hold **tab** to dictate ATC
  input (the result is prefixed with `[atc] ` before submission). Partial
  transcripts stream into the input line via the OpenAI Realtime API
  (`gpt-4o-mini-transcribe`); release the key to finalize, then press Enter
  to send. Requires a working microphone and `OPENAI_API_KEY`. Release feel
  is best on Kitty-keyboard-capable terminals (Ghostty, WezTerm, kitty,
  iTerm2 with CSI u enabled); elsewhere there is a ~180 ms release tail.
  Disable with `--no-voice`. The Python version did batch Whisper via
  `sim_pilot/speech.py`.
- **Runway/airport truth comes from X-Plane's `apt.dat`**, not from the
  ourairports CSV the Python project uses. Endpoints are 8-decimal (vs. the
  CSV's 3), per-runway heading and length are derived from the endpoint
  geodesy (vs. stored-and-often-rounded in the CSV), and every runway has
  non-null coordinates. The CSV path is still accepted as a fallback.
- **Extensions to the `sql_query` tool description** — additions over the
  Python version: "ALWAYS prefix spatial functions with ST_", a warning that
  `ST_Distance_Sphere` only accepts `POINT`s (it errors on a `LINESTRING`),
  and a worked crosstrack / along-track example for off-centerline offset.
  Each was added after observing the LLM stumble on the corresponding query
  against DuckDB. Flagged in the source.

Behavior of the deterministic pilot is anchored by the integration test
`tests/test_scenario.rs::nominal_mission_completes_takeoff_to_rollout`, which
matches the Python baseline touchdown position, centerline, sink rate, and
max final bank.
