# Plan: Microsoft Flight Simulator 2024 backend

Goal: fly the same pilot (plans, profiles, LLM loop, TUI, voice) in MSFS 2024,
alongside X-Plane 12 and the simple sim, selected with `--backend msfs`.

Decisions already made:

1. **The agent runs on Windows**, on the same machine as MSFS, and talks to
   it through SimConnect (no relay, no network protocol).
2. **How we drive the controls is an experiment.** We try the candidate
   routes and keep what works (Phase 0).
3. **Airport truth comes from the SimConnect Facilities API**, not
   X-Plane's apt.dat, so runways, taxiways, parking and frequencies match
   what is drawn in the sim.
4. **Controller gains get retuned for MSFS** (Phase 5).

SimConnect names below (SimVars, event IDs, facility fields, sign
conventions) are from memory of the MSFS SDK. **Check each one against the
MSFS 2024 SDK docs** before relying on it. Phase 0 exists partly to do that.

---

## Where X-Plane is wired in today

The control stack itself has no simulator in it: `PilotCore::update(raw_state, dt)`
takes a `DynamicsState` and returns `ActuatorCommands`. The X-Plane coupling
is in these places:

| Place | Coupling |
|---|---|
| `src/sim/xplane_bridge.rs` | REST + WebSocket transport, dataref ID resolution, `read_state`, `write_commands`, unit conversion, flap ratio ↔ detent |
| `src/sim/datarefs.rs` | Dataref names; `STATE_` / `COMMAND_` / `BOOTSTRAP_DATAREFS` sets |
| `src/live_runner.rs` | `run_live_xplane` builds `XPlaneWebBridge` directly; `run_control_loop` and `sample_track` take `Arc<XPlaneWebBridge>`; `probe_bootstrap_sample` is X-Plane REST; `LiveRunConfig` has `xplane_host` / `xplane_port` |
| `ToolBridge` (`src/llm/tools.rs:133`) | Keyed by **dataref name strings**: `get_dataref_value("sim/...")` / `write_dataref_values(...)`. Callers in `tools.rs` read lat/lon, heading, sim time, crash flag, fuel (kg, capacity, flow), engine RPM, parking brake, COM1/2, and write the parking brake, flap handle and COM frequency. Also implemented by `simple_bridge.rs` and `FakeBridge` in tests; used by `eval_runner.rs`, `transcribe/keyterms.rs`, `transcribe/ptt.rs`, `demo/xplane.rs` |
| `src/data/apt_dat.rs`, `src/data/parquet.rs` | apt.dat → `ParsedAptDat` → 7 parquet tables (airports, runways, comms, taxi_nodes, taxi_edges, parking_spots, hold_lines) under `~/.cache/sim_pilot/apt-<hash>/` |
| `src/data/airspace.rs` | X-Plane `airspace.txt` (OpenAir) → `airspaces.parquet` |
| `default_apt_dat_path`, `default_airspace_txt_path`, `parquet.rs:142` | `HOME` + macOS install paths |
| `src/demo/*`, `src/bin/demo.rs`, `src/bin/xplane_*` | X-Plane flight init, window capture, dataref writes from demo scripts |

Everything downstream of the parquet tables (the LLM's `sql_query` views,
`taxi_route::load_graph`, hold-line extension, plan compile/resolve) reads
tables, not apt.dat. So a second data source only has to fill the same
tables.

---

## Phase 0: SimConnect spike (Windows)

A throwaway binary, `src/bin/msfs_probe.rs` (behind the `msfs` feature),
answers the questions the rest of the plan depends on. Run it with the
default C172 at a known airport (KVNY 16R).

**Binding choice.** Evaluate, in order:
- generate our own bindings with `bindgen` from the MSFS 2024 SDK's
  `SimConnect.h`, linking `SimConnect.lib` and shipping `SimConnect.dll`
  next to the exe. This gives full control over facility definitions,
  `_EX1` calls and input events.
- the existing crates (`simconnect-sdk`, `simconnect`): check they cover
  facility data and client events against the 2024 SDK. Use one only if it
  does without patching.

Find the SDK through an env var (`MSFS2024_SDK`) in `build.rs`. Builds without
the feature, or on macOS, never touch it.

**Questions to answer and record in `research/msfs/spike.md`:**

1. **State stream.** Subscribe to one data definition at
   `SIMCONNECT_PERIOD_SIM_FRAME`. Measure the delivery rate and jitter, and
   confirm that 10–20 Hz reads are cheap.
2. **Sign and unit conventions** (the usual gotchas): `PLANE PITCH DEGREES`
   (believed positive nose-down), `PLANE BANK DEGREES` (which side is
   positive), body rotation rates (axes, sign, rad/s or deg/s),
   `VELOCITY WORLD X/Z` (east/north, ft/s), true vs magnetic heading.
   Bank, pitch and yaw the aircraft by hand and log the values.
3. **Pause / sim time.** The control loop treats an unchanged `time_s` as
   paused and skips the update. Find a time source that stops when the sim
   is paused (`SIMULATION TIME`, `ABSOLUTE TIME`, frame count) or else use
   the `Pause_EX1` system event to set a paused flag.
4. **Control routes.** For each of elevator, aileron, rudder, throttle,
   left/right brake, flaps, gear and parking brake, try:
   - a. legacy key events: `AXIS_ELEVATOR_SET`, `AXIS_AILERONS_SET`,
     `AXIS_RUDDER_SET` (±16383), `THROTTLE_SET` / `AXIS_THROTTLE_SET`,
     `AXIS_LEFT_BRAKE_SET` / `AXIS_RIGHT_BRAKE_SET`, `FLAPS_SET` or
     `FLAPS_UP` / `FLAPS_INCR`, `GEAR_SET`, `PARKING_BRAKE_SET`;
   - b. writing settable SimVars (`ELEVATOR POSITION`,
     `AILERON POSITION`, `GENERAL ENG THROTTLE LEVER POSITION:1`, …) with
     `SetDataOnSimObject`;
   - c. MSFS 2024 input events (`EnumerateInputEvents` / `SetInputEvent`),
     which some 2024 aircraft may use instead of legacy events.

   For each, record whether it moves the surface, the latency, whether it
   holds or decays between writes, and whether a connected joystick
   overrides it. The goal is the X-Plane `override_joystick_*` equivalent:
   our input wins and the hardware stays out. If nothing gives that, the
   fallback is documented setup ("unbind the yoke axes" or a control
   profile with no axes).
5. **Radios.** Check that `COM_RADIO_SET_HZ` / `COM2_RADIO_SET_HZ` tune
   8.33 kHz spacing and that `COM ACTIVE FREQUENCY:n` reads it back.
6. **Strings.** Read `ATC ID` (tail number) and `TITLE` / `ATC MODEL`
   (aircraft identity) through a string data definition.
7. **Facilities throughput.** Time `RequestFacilitiesList_EX1(AIRPORT)` for
   the whole world. Then time `RequestFacilityData` for one airport with
   runways + frequencies, and for one with taxi points, paths, names and
   parking. Measure KVNY, KLAX and a small strip. The numbers decide the
   caching strategy in Phase 3.
8. **Positioning.** Can we place the aircraft (set `PLANE LATITUDE`/`LONGITUDE`/
   `ALTITUDE`/`HEADING` with the freeze events, or `FlightLoad` a `.FLT`) for
   repeatable tests? This is not needed for flying, but it makes tuning bearable.

Exit: `spike.md` has a table of SimVars with confirmed units and signs, the
chosen control route per axis, the pause signal, and the facility timings.

---

## Phase 1: Sim-neutral bridge interfaces (on macOS, no MSFS needed)

Pure refactoring, tested against the X-Plane and simple backends. It is
worth doing even if MSFS support stalls.

### 1a. Replace dataref strings in `ToolBridge` with semantic calls

```rust
pub trait ToolBridge: Send + Sync {
    fn georef(&self) -> GeoReference;
    fn position(&self) -> Option<(f64, f64)>;          // lat, lon
    fn heading_true_deg(&self) -> Option<f64>;
    fn sim_time_s(&self) -> Option<f64>;
    fn has_crashed(&self) -> bool;
    fn fuel(&self) -> Option<FuelState>;               // kg, capacity kg, flow kg/s
    fn engine_rpm(&self) -> Option<f64>;
    fn parking_brake(&self) -> Option<f64>;
    fn set_parking_brake(&self, ratio: f64) -> Result<()>;
    fn set_flap_handle(&self, setting_deg: i32) -> Result<()>;
    fn com_frequency_mhz(&self, radio: ComRadio) -> Option<f64>;
    fn set_com_frequency_mhz(&self, radio: ComRadio, mhz: f64) -> Result<()>;
    fn aircraft_tail_number(&self) -> Option<&str> { None }
}
```

- Port every caller in `tools.rs` (around lines 271, 306, 366, 457, 480,
  972, 1660, 1726, 1752, 1761, 2048, 3001), plus `live_runner::sample_track`,
  `keyterms.rs`, `ptt.rs` and `eval_runner.rs`.
- `XPlaneWebBridge`, `simple_bridge` and `FakeBridge` implement the new
  methods by looking up the same datarefs as today.
- Raw dataref writes stay X-Plane-only: demo `script.xplane.datarefs`
  goes through an inherent `XPlaneWebBridge` method, not the trait.

### 1b. A sim bridge trait for the control loop

```rust
pub trait SimBridge: ToolBridge {
    fn read_state(&self) -> Result<DynamicsState>;
    fn write_commands(&self, commands: &ActuatorCommands) -> Result<()>;
    /// True when the sim is paused or not in a flight; the loop skips the update.
    fn paused(&self) -> bool { false }
    fn close(&self) {}
}
```

- `run_control_loop` and `sample_track` take `Arc<dyn SimBridge>`. The
  frozen check keeps the `time_s` comparison and also honours `paused()`.
- Split `run_live_xplane` into backend setup (probe the bootstrap sample,
  open the bridge) and a shared `run_live(bridge, sample, config, runtime)`.
- `LiveRunConfig.xplane_host/port` becomes
  `sim: LiveSim::XPlane { host, port } | LiveSim::Msfs`.
- `BootstrapSample` stays as it is; each backend produces one.

### 1c. A pluggable airport data source

- Make `parquet::write_cache` take the row structs (`ParsedAptDat`, or a
  renamed `AirportData` with the same fields) from any source, with a
  cache key the source provides (an apt.dat hash today, an MSFS key later).
- Rename the cache helpers so nothing outside `data/apt_dat.rs` assumes
  apt.dat (`AptDatCache` → `AirportCache`, `apt_dat_cache_dir` →
  `airport_cache_dir`). Keep the table schemas byte-identical, because the
  system prompt and `SQL_QUERY_DESCRIPTION` document them.
- Allow appending one airport's taxi tables to an existing cache (needed
  if Phase 3 fetches taxi data lazily).

### 1d. Portability that Windows needs anyway

- Home and cache dirs: replace `HOME` lookups (`parquet.rs:142`,
  `apt_dat.rs:597`, `airspace.rs:259`, `demo/radio_voice.rs`) with a helper
  that falls back to `USERPROFILE`. Put the cache in `%LOCALAPPDATA%\sim_pilot`
  on Windows.
- Python: `plan/python.rs` defaults to `python3`, which on Windows is often
  the Store stub. Default to `py -3` on Windows (`SIM_PILOT_PYTHON` still
  overrides it).
- Claude Code provider: confirm `Command::new("claude")` finds the Windows
  install (`claude.exe` vs a `.cmd` shim) and add a `--claude-binary`
  override if it doesn't.

Exit: `cargo test` passes; a live X-Plane run and a simple-backend run
behave exactly as before.

---

## Phase 2: `MsfsBridge` (Windows)

`src/sim/msfs/` behind `#[cfg(all(windows, feature = "msfs"))]`:

- `sys.rs`: bindgen output.
- `client.rs`: connect/disconnect, a dispatch thread that pumps
  `SimConnect_GetNextDispatch` (or waits on the event handle), and
  definition / request ID allocation.
- `simvars.rs`: the data definitions. This is the MSFS counterpart of
  `datarefs.rs`.
- `bridge.rs`: `MsfsBridge`, which implements `SimBridge` + `ToolBridge`.

**State.** One struct-shaped data definition at `SIM_FRAME`. The dispatch
thread overwrites a cached copy and `read_state` converts the latest one.
This is the same "latest value wins" model as the X-Plane delta cache.
Planned mapping (units requested in the definition):

| `DynamicsState` | SimVar (to confirm) |
|---|---|
| lat/lon → `position_ft` | `PLANE LATITUDE`, `PLANE LONGITUDE` (degrees) through `geodetic_offset_ft` |
| `altitude_ft` | `PLANE ALTITUDE` (feet) |
| `agl_ft` | `PLANE ALT ABOVE GROUND MINUS CG`, or `PLANE ALT ABOVE GROUND` (feet). Pick by comparing on the ground |
| `heading_deg` | `PLANE HEADING DEGREES TRUE` (degrees) |
| `roll_deg`, `pitch_deg` | `PLANE BANK DEGREES`, `PLANE PITCH DEGREES`, sign-flipped per Phase 0 |
| `ias_kt` | `AIRSPEED INDICATED` (knots) |
| `ground_velocity_ft_s` | `VELOCITY WORLD X` (east), `VELOCITY WORLD Z` (north) (ft/s) |
| `vertical_speed_ft_s` | `VERTICAL SPEED` (ft/s) |
| `p/q/r_rad_s` | `ROTATION VELOCITY BODY X/Y/Z`, remapped to roll/pitch/yaw rates |
| `throttle_pos` | `GENERAL ENG THROTTLE LEVER POSITION:1` (percent → 0..1) |
| `flap_index` | `FLAPS HANDLE INDEX` mapped to detent degrees through the aircraft config |
| `gear_down` | `GEAR HANDLE POSITION` |
| `on_ground` | `SIM ON GROUND` |
| `time_s` | whichever pausable clock Phase 0 picks |
| `parking_brake` | `BRAKE PARKING POSITION` |
| tool reads | `CRASH FLAG`, `FUEL TOTAL QUANTITY WEIGHT` / `FUEL TOTAL CAPACITY` (lbs or gal × density → kg), `ENG FUEL FLOW PPH:1`, `GENERAL ENG RPM:1`, `COM ACTIVE FREQUENCY:1/2`, `ATC ID` |

**Commands.** `write_commands` uses the route Phase 0 picked per axis. Keep
the X-Plane bridge's rules:
- flaps and parking brake go out only when they change
  (`CockpitSetting`); resending them every tick froze the flaps in X-Plane;
- differential braking: left = brakes − pivot, right = brakes + pivot;
- clamp everything and scale to the event range (±16383 or 0..16383).

**Bootstrap.** `probe_bootstrap_sample` equivalent: wait for the first state
frame (timeout 5 s) and fill `BootstrapSample` from it.

**CLI.** `--backend msfs` (a clear error on non-Windows builds or without
the feature). Add `--aircraft-config` if the C172 YAML needs an MSFS variant
(see Phase 5).

Exit: a live run with the startup profile `idle`, with the TUI showing
correct position, attitude, speeds and phase. Then a heading/altitude/speed
hold that holds (badly tuned is fine).

---

## Phase 3: Airport data from the Facilities API

`src/data/msfs_facilities.rs` fills the same `AirportData` rows.

**Global tables** (`airports`, `runways`, `comms`): these must be global,
because the LLM's SQL does nearest-airport and spatial queries.
- Airport list: `RequestFacilitiesList_EX1(AIRPORT)` gives ident, region,
  lat, lon and altitude.
- Runways and frequencies per airport: a facility definition of
  `OPEN AIRPORT` (`LATITUDE`, `LONGITUDE`, `ALTITUDE`, `NAME`, `ICAO`,
  counts) / `OPEN RUNWAY` (centre `LATITUDE`/`LONGITUDE`/`ALTITUDE`,
  `HEADING`, `LENGTH`, `WIDTH`, `SURFACE`, `PRIMARY_NUMBER`/`DESIGNATOR`,
  `SECONDARY_…`, primary/secondary threshold offsets) / `OPEN FREQUENCY`
  (`TYPE`, `FREQUENCY`, `NAME`).
- Derive each runway's le/he thresholds from the centre, heading and
  length. Displaced thresholds come from the threshold offset fields.
  MSFS gives a per-end elevation, which is better than apt.dat's single
  field elevation.
- Map frequency `TYPE` to our `kind` (ATIS / UNICOM / CD / GND / TWR /
  APP / DEP; anything else becomes `OTHER`).

**Per-airport taxi tables** (`taxi_nodes`, `taxi_edges`, `parking_spots`,
`hold_lines`) come from `TAXI_POINT`, `TAXI_PATH`, `TAXI_NAME` and
`TAXI_PARKING`:
- Points and parking are offsets in metres (`BIAS_X`/`BIAS_Z`) from the
  airport reference point. Convert them with `latlon_from_offset_ft`
  around the airport's lat/lon.
- Paths become edges. Taxiway names come from `NAME_INDEX` → `TAXI_NAME`.
  Category comes from the path `TYPE`. Drop vehicle/road paths, the
  counterpart of skipping apt.dat row 1206. MSFS paths are two-way, so
  `direction = "twoway"`.
- Usage (`init` / `end` / `both`): set to `both`, and check that
  `taxi_route` doesn't depend on it in a way that breaks.
- Active zones: apt.dat 1204 rows have no direct counterpart. Synthesize
  them from runway-type paths (the runway they belong to) and the
  hold-short points next to them, so runway-crossing hold-shorts in taxi
  plans (`tools.rs:1837`) still work.
- Hold lines: MSFS gives hold-short *points*, while `hold_lines` stores
  painted *segments*. Synthesize a short segment through each point,
  perpendicular to its path and as wide as the path, so
  `taxi_route::extend_past_hold_lines` keeps working.
- Parking: `TYPE` → kind (gate / tie_down / ramp → misc), name + number →
  `name`, `HEADING` → `heading_true_deg` (check true vs magnetic).

**Caching.** Choose by the Phase 0 timings:
- if fetching every airport's runways and frequencies takes a few minutes,
  build the global tables once and cache them. Key the cache on the
  airport count plus a hash of the airport list; rebuild with
  `--refresh-airports` after a sim or scenery update;
- otherwise build `airports` globally (one call) and fetch runways and
  comms lazily within ~100 nm of the aircraft, refreshing as it moves;
- either way, fetch taxi data lazily per airport: on startup for the
  nearest airport, and in each taxi tool before `load_graph`. Append it to
  the cache.

Facility requests must not block the control loop. They run on the
dispatch thread's request queue, and tools wait on a reply channel with a
timeout.

**Airspaces.** The Facilities API doesn't give us class B/C/D polygons, as
far as I know (to confirm). Keep `airspaces.parquet` optional: accept an
OpenAir `--airspace-txt-path` (X-Plane's file copied over works) and
otherwise run without one.

**Places.** `config/places/*.yaml` (roads and landmarks) is sim-independent.
Check visually that the MSFS scenery puts the same freeways where the
gazetteer says.

**Validation.** `scripts/msfs/compare_airports.py` (uv) diffs the
MSFS-derived tables against an apt.dat-derived cache for KVNY, KSMO, KLAX,
KWHP and KBUR: threshold offsets, headings, lengths, taxiway names present
in one and missing from the other. It is a sanity check, not a gate,
because the two sims really do differ.

Exit: on MSFS, the LLM can find KVNY, its runways and its tower frequency
through SQL. `plan_taxi` from the KVNY ramp to 16R produces a route along
named taxiways that matches the MSFS ground markings, and stops at the
hold line.

---

## Phase 4: Windows bring-up of the rest

- `cargo test` on Windows (the simple backend and the full unit suite).
  Gate tests that need an X-Plane install
  (`resolve_builds_full_cache_against_real_xplane_install`) on the path
  existing, as on macOS.
- TUI under Windows Terminal: check rendering and resize.
- Voice PTT: crossterm on Windows reports key release events natively, so
  check that `ptt.rs` uses them rather than the repeat-gap fallback. Check
  that `cpal` (WASAPI) mic capture resamples correctly.
- DuckDB spatial: the first run downloads the extension; check that it
  caches under the Windows profile.
- The `.env` loader, `output/` paths, and log, CSV and KML writers.
- LLM providers: API backends, then `--pilot-llm-provider claude-code`
  with the Windows `claude` install.

---

## Phase 5: Tuning for MSFS

Aircraft: the MSFS 2024 default C172. Choose the YAML from `ATC MODEL` /
`TITLE` at startup (`aircraft_c172.yaml` stays the X-Plane one, plus a new
`aircraft_c172_msfs.yaml` and, if needed, `controller_gains_msfs.yaml`).
The flap detent table moves into the aircraft config so the bridges stop
hard-coding `[0, 10, 20, 30]`.

Order, with logged CSVs (`--log-csv`) and the plot tooling for each step:

1. Bank and pitch inner loops: step responses in cruise. Watch for
   actuator lag or rate limits that weren't in X-Plane.
2. Heading, altitude and speed holds, then TECS throttle/pitch.
3. Pattern legs and turns (L1). Then approach, flare and touchdown
   (sink rate, float, centreline).
4. Takeoff roll and rotate: centreline rudder, then rotate speed and pitch.
5. Rollout and runway exit: rudder, brakes and the deceleration profile.
6. Taxi: nose-wheel steering response, differential-brake pivot, the
   parking pose. MSFS ground physics differs most from X-Plane here,
   so expect the most work.

Positioning from Phase 0 (or `FlightLoad` of saved `.FLT` files) gives
repeatable starts: downwind at 1,000 ft AGL, short final, holding short of
16R, the ramp.

Exit: the KVNY 16R mission runs end to end (ramp, taxi, takeoff, pattern,
landing, exit, taxi back) with touchdown and centreline numbers in the same
range as X-Plane.

---

## Phase 6: Later / optional

- Live eval scenarios against MSFS: positioning plus the existing
  `evals/plan` checks run against live state. The simple-sim evals stay the
  main regression suite.
- A demo backend: an MSFS counterpart of `demo/xplane.rs` (flight init and
  window capture for OBS). The demo scripts' `xplane.datarefs` would become
  a per-sim block.
- Update the docs: CLAUDE.md commands, README, NOTAMS and CHANGES
  (operator-language NOTAM: "You can now fly in Microsoft Flight Simulator
  2024 on Windows").

---

## Risks and open questions

- **Control authority.** If no route both holds our input and ignores the
  hardware, operators have to unbind their axes. Settle this first in
  Phase 0.
- **Aircraft that ignore legacy events.** MSFS 2024 modular aircraft may
  only respond to input events or B: vars. If the default C172 does this,
  route (c) becomes the main path and is aircraft-specific.
- **Pause and time semantics** affect controller wind-up. Settle this in
  Phase 0.
- **Facilities API completeness.** Taxi names, hold-short flags and runway
  association on paths may be missing at some airports (scenery
  dependent). The fallback is that taxi isn't offered there, just as with
  apt.dat airports that lack a network.
- **Global build time.** Up to ~40k airports' runways over SimConnect may
  be slow. The lazy option covers that.
- **Sim-specific prompt text.** Check `system_prompt.md` and tool
  descriptions for X-Plane mentions (`apt.dat`, dataref names) and make
  them sim-neutral, keeping the prompt byte-stable within a session.
- **Two bridges to maintain.** Keep the X-Plane and simple backends green
  in CI on macOS. The MSFS backend can only be tested by hand on Windows.

## Suggested order of work

Phase 1 (macOS, any time) → Phase 0 spike (Windows) → Phase 2 → Phase 4
(as needed to run Phase 2) → Phase 3 → Phase 5 → Phase 6. Phase 1 needs no
MSFS and can merge on its own.
