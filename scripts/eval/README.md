# sim_pilot end-to-end eval harness

A scenario-agnostic runner that drives `sim_pilot` against a live
X-Plane 12 session and captures evidence of what happened. The harness
**does not judge** pass or fail — that's for the operator (human or LLM)
looking at the artifacts afterward.

## Flow

1. **Load** a scenario YAML (`scenarios/*.yaml`).
2. **Apply** it to X-Plane via the web API on `:8086`:
     - `POST /api/v3/flight` with the scenario's `flight_init` block
       (aircraft path + `lle_air_start`/`runway_start`/`ramp_start`/
       engine status).
     - Optional pause, then writes to writable `setup.datarefs`
       (flaps, gear, parking brake, etc.), then unpause.
3. **Start a 1 Hz state-trace CSV** that records ground-truth position,
   altitude, speeds, on-ground flag, `has_crashed`, engine RPM, etc.
4. **Launch** `target/release/sim-pilot --headless --no-voice …` with
   the scenario's pilot args; stream its stdout/stderr into
   `sim_pilot.log`.
5. **Poll** until a termination condition fires:
     - `has_crashed` edge (if `terminate_on_crash`)
     - on-ground + groundspeed below threshold held for N seconds
       (if `terminate_on_stopped`)
     - `timeout_s` (scenario-supplied, capped at 300 s by the harness)
     - sim_pilot subprocess exited on its own
     - Ctrl-C
6. **Kill** sim_pilot, pause X-Plane, and emit a neutral `report.json`:
   termination reason, elapsed, final observed state, paths to
   artifacts. No pass/fail, no assertions.

Exit codes:
- `0` — run completed for any non-error reason
- `2` — harness error (couldn't talk to X-Plane, setup failed, …)

## Prereqs

- X-Plane 12 running with the web API on port 8086. The harness can
  start a fresh flight from the main menu via `POST /api/v3/flight`,
  so X-Plane doesn't need to already be in a flight.
- `OPENAI_API_KEY` in `.env` at the repo root (sim_pilot requires it
  for the live xplane backend).
- `uv` (the harness is a PEP 723 script).
- `target/release/sim-pilot` prebuilt (`cargo build --release`).

## Usage

```bash
uv run scripts/eval/run_scenario.py scripts/eval/scenarios/deadstick_kwhp_12.yaml
```

Smoke-test just the setup without launching sim_pilot:

```bash
uv run scripts/eval/run_scenario.py scripts/eval/scenarios/foo.yaml --setup-only
```

Override the X-Plane endpoint or output location:

```bash
uv run scripts/eval/run_scenario.py scenarios/foo.yaml \
  --host 10.0.0.5 --port 8086 --out-dir /tmp/eval-foo
```

## Artifacts per run

Each run writes to `output/eval/<scenario>-<timestamp>/`:

- `scenario.yaml` — copy of the input.
- `setup.json` — the exact setup block applied.
- `sim_pilot.log` — sim_pilot stdout + stderr.
- `state_trace.csv` — 1 Hz ground-truth from X-Plane.
- `report.json` — termination facts.

## Scenario schema

```yaml
name: my_scenario

goal: |
  One-line description of what this test is meant to exercise. The
  operator reads this to decide what "success" looks like for this run.

description: |
  Longer context — starting conditions, what the pilot is expected to
  do, what failure modes we're watching for.

setup:
  settle_s: 6.0        # seconds to wait after flight_init POST
  pause: false         # pause X-Plane around dataref writes

  flight_init:         # goes verbatim into POST /api/v3/flight `data`
    aircraft:
      path: "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf"
    lle_air_start:     # or runway_start / ramp_start / lle_ground_start
      latitude: 34.27568
      longitude: -118.45540
      elevation_in_meters: 610.0
      heading_true: 118.0
      speed_in_meters_per_second: 33.4
      pitch_in_degrees: -3.0
    engine_status:
      all_engines:
        running: false

  datarefs:            # writable post-load fixups
    sim/cockpit2/controls/flap_handle_request_ratio: 0.333
    sim/cockpit2/controls/gear_handle_down: 1.0

pilot:
  engage_profile: idle            # --engage-profile
  atc_messages:                   # one --atc-message per entry
    - "Cessna 1234 short final 12, cleared to land."
  extra_args:                     # pass-through to sim-pilot
    - --pilot-llm-model
    - gpt-5.4-mini-2026-03-17

termination:
  terminate_on_crash: true
  terminate_on_stopped: true
  stopped_ground_speed_kt: 5.0
  stopped_hold_s: 5.0
  timeout_s: 300.0                # harness hard-caps at 300s regardless
```

## Writing new scenarios

1. Decide what capability you're testing. Describe it in `goal` so the
   operator reviewing the artifacts knows what to look for.
2. Pick a starting state: `runway_start` for takeoff tests,
   `lle_air_start` for in-flight, `ramp_start` for gate-out, etc.
   See the X-Plane
   [Flight Initialization API](https://developer.x-plane.com/article/flight-initialization-api/)
   for schema.
3. Write one or more `atc_messages` that provide the pilot with context
   it needs — the harness does not otherwise prompt the LLM.
4. Run it. Read `report.json` + `state_trace.csv` + `sim_pilot.log` to
   assess what happened.

## Limitations / TODO

- No batch runner — wrap in a shell loop for now.
- Weather init requires all fields per X-Plane — easier to skip the
  `weather` block entirely than to set a partial one.
- Some position datarefs are read-only via REST; use `flight_init`
  rather than `setup.datarefs` for position/velocity/attitude.
