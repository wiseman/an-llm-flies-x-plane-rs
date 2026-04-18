# An LLM Flies X-Plane

Rust port of `xplane-pilot`: an LLM-driven pilot for X-Plane 12 that handles radio work, taxi and pattern decisions, and high-level mission intent while a deterministic flight-control stack flies the airplane.

## Concept

The model never writes elevator, aileron, rudder, or throttle directly. A 10 Hz deterministic control loop owns the actuators at all times. The LLM reads operator and ATC messages, reasons about the current situation, and calls tools that engage or reconfigure guidance profiles such as heading hold, cruise, takeoff, pattern flying, taxi, or line-up-and-wait.

Those profiles own flight-control axes and are merged into a single command set every tick. `engage_cruise` installs the three single-axis holds atomically; `engage_takeoff` and `engage_pattern_fly` own all three axes for runway and traffic-pattern work; `engage_taxi` and `engage_line_up` use the same deterministic ground controllers to drive the airplane across the airport surface and onto the runway centerline.

The live backend talks to X-Plane through the built-in web API and uses X-Plane's own `apt.dat` as the source of truth. That data is parsed into DuckDB-backed parquet views for airports, runways, comms, taxi nodes, taxi edges, and optional airspaces, so the LLM can query real airport geometry instead of guessing runway numbers, frequencies, or taxi routes.

## Tool Catalog

### State And Profile Control

| Tool | What it does |
| --- | --- |
| `get_status` | Returns a JSON snapshot of aircraft state, phase, active profiles, and live position when available. |
| `sleep` | Ends the current LLM turn while the deterministic pilot keeps flying the active profiles. |
| `engage_heading_hold` | Takes ownership of the lateral axis and turns to a target heading, optionally forcing left or right. |
| `engage_altitude_hold` | Takes ownership of the vertical axis and holds a target altitude with TECS. |
| `engage_speed_hold` | Takes ownership of the speed axis and holds a target IAS. |
| `engage_cruise` | Atomically installs heading, altitude, and speed hold together. |
| `disengage_profile` | Removes a named profile and lets uncovered axes fall back to idle profiles. |
| `list_profiles` | Lists the currently active profile names. |

### Takeoff, Pattern, And Landing

| Tool | What it does |
| --- | --- |
| `engage_takeoff` | Verifies runway alignment and starts the deterministic takeoff roll, rotation, and climb. |
| `takeoff_checklist` | Reports takeoff readiness, including parking brake, flaps, and other live-state checks. |
| `engage_pattern_fly` | Anchors the mission pilot to a real runway and flies a full traffic pattern from a chosen phase. |
| `extend_downwind` | Pushes the base-turn point farther out while `pattern_fly` is active. |
| `turn_base_now` | Forces an immediate base turn from downwind. |
| `go_around` | Commands an immediate go-around during pattern flying. |
| `execute_touch_and_go` | Arms the next landing to skip rollout braking and transition straight back to takeoff. |
| `cleared_to_land` | Records a landing clearance for the active pattern. |
| `join_pattern` | Acknowledges a pattern-entry instruction while `pattern_fly` is active. |

### Ground, Radio, And Airport Data

| Tool | What it does |
| --- | --- |
| `tune_radio` | Tunes `com1` or `com2` to a frequency in MHz. |
| `broadcast_on_radio` | Sends an actual radio transmission; plain text alone does not transmit to ATC. |
| `set_parking_brake` | Engages or releases the parking brake. |
| `set_flaps` | Sets flap handle position to `0`, `10`, `20`, or `30` degrees. |
| `engage_line_up` | Crosses the hold short, enters the runway, aligns to runway heading, and stops in line-up position. |
| `engage_taxi` | Plans and then flies a taxi route to a runway hold-short point with deterministic ground steering. |
| `plan_taxi_route` | Plans a taxi route only, including legs, taxiway sequence, distance, and runway conflict zones. |
| `sql_query` | Runs read-only SQL against the apt.dat-derived DuckDB views for airports, runways, comms, taxi network, and optional airspaces. |

### Placeholder Schemas

| Tool | Status |
| --- | --- |
| `engage_approach` | Exposed in the schema but currently returns `not yet implemented`. |
| `engage_route_follow` | Exposed in the schema but currently returns `not yet implemented`. |
