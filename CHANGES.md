# Changelog

All notable user-facing changes to this project are documented here.
The format is loosely based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the
project follows [Semantic Versioning](https://semver.org).

## [0.6.0] — 2026-04-27

### NOTAMs

- Fly with Anthropic Claude or Google Gemini, not just OpenAI. Pick
  the brain at startup with `--pilot-llm-provider anthropic` or
  `--pilot-llm-provider gemini` plus a matching `--pilot-llm-model`.
- New cockpit-style display: status pane up top, energy + risk
  alongside, radio and action log side-by-side below, input at the
  bottom. On narrow terminals the radio stacks above the log so ATC
  doesn't get squeezed off the screen.
- Flight sessions render to a standalone HTML transcript via the new
  `sim-pilot-transcript` binary — radio and ATC traffic show up as
  timeline events alongside pilot actions, with a mission-complete
  summary footer.
- A six-model bench-off comparing how OpenAI, Anthropic, and Gemini
  models fly the same scenario, end-to-end on the offline simulator.
  Results table in the README.
- The pilot's thinking is more visible — Gemini reasoning traces
  surface in the action log, and a propeller spinner appears while
  the model is working.
- Pattern altitude is held until ~1.2 nm from the runway before the
  descent kicks in. Fewer "low and slow" base legs.
- Default downwind offset widened — more room to set up the descent.
- Heartbeats no longer fire on top of an in-flight model call; the
  idle clock resets when the pilot finishes thinking.
- Quieter elevator on combined altitude + speed holds — control
  chatter killed.
- LLM provider errors now surface the response body so failures are
  diagnosable instead of just "non-2xx".

### Added

- Provider-neutral pilot-LLM trait. OpenAI Responses (existing),
  Anthropic Messages, and Gemini are all wired through the same
  conversation loop. New flags: `--pilot-llm-provider {openai,
  anthropic, gemini}` and matching model overrides. Anthropic uses
  prompt cache breakpoints placed on stable content for cost
  discipline; Gemini surfaces its thought-trace into the action log.
- End-to-end LLM eval harness on `SimpleAircraftModel`. One CLI
  drives a fixed scenario across multiple model/provider pairs,
  records per-run tool call + failure counts, writes a flight track
  to CSV, and emits a normalized report. Six-model results table
  added to the README.
- `sim-pilot-transcript` CLI renders a session `.log` to HTML.
  Operator turns, pilot replies, tool calls, radio/ATC traffic, and
  heartbeats appear as a single timeline; a mission-complete summary
  footer (touchdown stats, tokens, tool counts) lands at the bottom.
- Narrow-terminal stacked layout for the radio + action-log split.
  Below 100 columns the panes stack vertically (radio 1/4, log 3/4)
  instead of being squeezed side by side.
- Propeller spinner in the action-log pane while the LLM is busy.

### Changed

- TUI rebuilt as the "AI Pilot Console" multi-pane layout — status
  / energy + risk / radio + action log / input. The previous v1 + v2
  layouts are gone; `--tui-v2` is no longer needed.
- Default C172 downwind offset widened from 3500 ft to 6000 ft.
- Pattern altitude is held until within 1.2 nm of the runway before
  the descent gate opens (was: descent gated only on abeam-the-
  numbers + leg geometry).
- Heartbeat pump gates on LLM-busy. The idle timer restarts from the
  end of an LLM call instead of running concurrently with one, so a
  long-thinking turn won't cause an overlapping heartbeat to fire.
- `pattern_fly` auto-releases on a clean post-landing stop. The
  three idle holds take over without an extra operator nudge.
- Pitch derivative gain halved — kills the elevator chatter that
  showed up on combined altitude + speed holds.
- `Axis` ownership inside the profile composer moved from
  `BTreeSet<Axis>` to a small bitmask (`AxisSet`); shared a test
  fixture across the unit tests that used to copy-paste it.
- Tool descriptions and the system prompt tightened.
- ias / gs in the status payload spelled out with units so models
  read them correctly.
- `engage_pattern_fly` `start_phase` doc clarified.
- Heartbeat lines drop the trailing `| status={...}` JSON in the
  pane — the JSON still goes to disk for the LLM's heartbeat replay,
  but the human only sees the human-readable reason.
- Radio / ATC transmissions now appear as first-class timeline
  events in the HTML transcript renderer, not just plain log lines.
- Single-line tool summaries no longer truncated in the transcript.
- Provider + model surfaced on eval-run log headers.
- Anthropic cache-write tokens tracked for eval cost math.

### Fixed

- Anthropic cache hit-rate calculation summed three disjoint input
  counters; previously the rate was understated.
- Eval pilot snapshot uninit race — scenarios prime the snapshot
  before LLM startup so the first turn doesn't see zeros.
- Non-2xx LLM HTTP responses now include the response body in the
  error, not just the status code.
- `tools` shared takeoff-position gate + an override flag for runway
  departures from non-standard configurations (intersection, hold-
  short cluster mismatch).

## [0.5.0] — 2026-04-22

### NOTAMs

- After landing the pilot actually leaves the runway. Rollout
  decelerates to a turnoff speed and holds it instead of braking to
  a dead stop on the centerline; pick your exit and the aircraft is
  stopped clear of the pavement and released to you to taxi.
- The pilot can park. Name a gate or tie-down and the pilot taxis
  there and stops with the nose on the painted heading. Works from
  any airport in the world.
- No more backtaxi down runways. "Taxi to runway X" without an
  intersection reliably lands at X's approach end; mid-field and
  opposite-threshold crossings are no longer silent intersection
  departures.
- Parked facing into a stall? The pilot now pivots in place to line
  up with the taxiway network instead of declaring a dead end.
- Operator replies come through even when the pilot also runs a
  tool. Previously a response that paired commentary with an action
  would drop the commentary on the floor and look like the pilot
  ignored you.
- Log and radio panes restyled — colored left-gutter bars per
  subsystem, operator turns in a subtle "user bubble" background,
  pilot vs ATC transmissions color-coded, errors keep their red
  emphasis.
- The pilot behaves more like a pilot: won't doze off with the
  aircraft coasting uncontrolled, won't pick a runway exit that
  doesn't exist, reads tool-error remediation text and acts on it
  instead of escalating to you, and sticks to stable flares instead
  of bailing for cosmetic reasons.

### Added

- `engage_park(airport_ident, parking_name, via_taxiways=[...])` and
  `plan_park_route(...)` route to named parking spots sourced from
  apt.dat rows 1300 (startup location: gate, tie-down, hangar) and
  1301 (airline-operations metadata: ICAO category, operation type,
  airline codes). Persisted as a Hilbert-sorted `parking_spots`
  parquet table. Schema version bumped to 3 — existing caches rebuild
  on next start.
- `list_runway_exits(airport_ident, runway_ident)` and
  `choose_runway_exit(taxiway_name)` drive the post-landing turnoff.
  Exits are annotated with their position relative to the aircraft
  (ahead / behind / abeam); `choose_runway_exit` validates against
  `list_runway_exits` so the LLM can't silently pick an exit that
  doesn't exist.
- New `FlightPhase::RunwayExit` sits between `Rollout` and
  `TaxiClear`. Entered when lateral offset passes 75 ft, exits at
  ground stop or >150 ft. `PatternFlyProfile` hands off to idle on
  the first `TaxiClear` tick when a preferred exit is set, so the
  LLM picks up a stationary aircraft to taxi or park.
- Scenario-driven eval harness under `scripts/eval/` — PEP 723 `uv`
  script that applies an X-Plane 12.4 scenario via the `flight_init`
  REST API, spawns `sim-pilot --headless`, captures 1 Hz state-trace
  CSV + log, and emits a neutral `report.json`. Three seed
  scenarios: `deadstick_kwhp_12`, `taxi_after_landing_kwhp_12`,
  `taxi_only_kwhp`.
- `AGENTS.md` with repo guidelines for Codex and other agent tooling.

### Changed

- System prompt reorganised around active airmanship. New Core
  principles: aviate then navigate then communicate; `idle_*` means
  "nothing is flying"; a clearance is a pending action, not a
  conversation; stable-approach discipline; "waiting on context" is
  not a plan — query. Plus a per-turn tool-call budget reminder
  (~10 per turn, batch the work).
- Log pane restyled: per-kind colored left-gutter bar (`▎`) plus
  default-color body; errors keep the `!` sigil + red-bold for
  emphasis; operator entries render over a subtle RGB(32,32,38)
  "user bubble" background padded to pane width; a blank line
  separates consecutive operator↔llm turns.
- Radio pane per-source coloring: pilot broadcasts in light cyan,
  ATC/other-traffic in yellow, tuning notes in dim gray.
- `SimBus` log entries carry a typed `LogKind` (operator, llm,
  safety, error, mode, system, tool-call, tokens, heartbeat, voice)
  instead of a parsed bracket-tag. Compact-mode filtering keys off
  the kind. On-disk log file format is unchanged.
- `sleep` refuses when the aircraft is moving and only `idle_*`
  profiles are active. `pattern_fly`-not-active tool errors now
  tell the LLM exactly which tool to call.
- `TaxiProfile` mode-line shows total remaining taxi distance
  alongside the current leg.
- Cruise taxi speed raised 10 → 20 kt.
- `Rollout → TaxiClear` transition requires both `gs ≤ 1 kt` and
  `runway_x_ft > length_ft` — no more "taxi clear" declared while
  still sitting mid-runway.
- Live runner installs a `ctrlc`/`SIGTERM` handler that flips the
  same stop flags the TUI drain path uses, so harness shutdown
  finalises the track KML + closes the bridge instead of hard-
  killing.
- Shared geodetic helpers (`haversine_m`, `initial_bearing_deg`,
  `EARTH_RADIUS_M`) moved to `types.rs`; internal callers no longer
  carry local copies.
- Active-profiles span in the TUI brightened DarkGray → Gray (was
  nearly invisible on dark themes).

### Fixed

- Assistant text silently dropped when a single response contained
  both a message and a function_call. `handle_message` only emitted
  text on the zero-tool-call branch; operators saw the tool fire
  but no reply. Now emitted unconditionally.
- Touchdown no longer slams 75% brakes to a full stop on the runway.
  Preferred-exit rollout brakes modulate to `turnoff_speed_kt`
  (default 15) and hold; full brakes only in the last 500 ft of
  runway if no exit is selected.
- `engage_park` / `engage_taxi` no longer plan backtaxi routes. The
  apt.dat graph planner now carries per-edge `runway` vs
  `taxiway_A..F` category and filters runway-surface edges whose
  bearing is >90° off aircraft heading. Hold-short resolution clamps
  alternate boundary nodes to within 500 ft of the primary
  candidate so mid-field crossings don't win the path-cost race.
  The synthetic pullout lead-in also rejects >135° turns when the
  aircraft is on runway pavement.
- `build_taxi_legs_with_pullout` no longer rejects any >135°-turn
  pullout — only when the aircraft is actually on runway pavement.
  Ramp pirouettes of any angle succeed, so aircraft parked facing
  into a stall can taxi out without a manual reposition.
- Pattern-fly `Rollout → TaxiClear` handoff now always delivers a
  stationary aircraft — the centerline fallback tightened from
  `gs_kt ≤ 5` to `gs_kt ≤ 1`, matching the `RunwayExit` full-stop
  guard.
- Log colorizer no longer allocates a lowercased copy of every
  line; uses a byte-level `contains_ascii_ignore_case` helper.

## [0.4.0] — 2026-04-20

### NOTAMs

- Taxi reliably gets you all the way to the hold-short, including the
  final 90° swing onto the runway heading.
- Intersection departures work end-to-end — "taxi to runway 19 at
  Charlie, cleared for takeoff at Charlie" does the right thing.
- The pilot actually transmits on the radio in realistic mode
  (previously called a tool that didn't exist).
- Realistic mode asks the operator for a tail number on its first
  radio call when it doesn't have one, instead of failing silently.
- Just run the binary — the TUI is default now, no flag needed. Pass
  `--headless` for scripted runs.
- The taxi mode line shows where you're going (`A → B`), not just the
  current leg.
- Voice transcription catches multi-word facility and taxiway names
  more reliably.

### Added

- Per-session Responses API transcript dump — every round (request +
  response) is written to `output/<session>.txt` so you can tail what
  the pilot is actually sending/receiving without scrolling the main
  log.
- Intersection-departure support — `engage_taxi` and `engage_line_up`
  accept an `intersection=` argument for "runway 19 at Charlie"-style
  clearances. `engage_takeoff` reads remaining runway from current
  position and refuses below 1000 ft usable ahead.
- Route summary in the taxi mode line — the full routed path
  (`A → B`) is surfaced instead of only the current leg name.

### Changed

- **Breaking**: `--interactive-atc` removed. The interactive TUI is
  now the default; pass `--headless` to run with stdout echo and
  scripted `--atc-message` strings. The TUI covers operator input,
  voice PTT, and the log/radio panes, so the old name never made
  sense.
- System prompt rewritten. Consolidated duplicate rules
  (facts / where-you-are / iterate / no-menus → one Core principles
  section), moved the airspace dictionary next to heartbeat
  mechanics, flattened the tool reference, trimmed VFR regs, reduced
  CAPS inflation. Base prompt is now neutral of "sim vs real"
  framing — the realistic overlay still layers on phraseology and
  readback discipline when you want it.
- Deepgram keyterms pass compound phrases ("Whiteman Tower") at a
  higher tier than their component words, improving recognition of
  full facility names and multi-word taxiway IDs.
- X-Plane bridge eagerly updates cached dataref values on write, so a
  `get_status` on the same tick reflects what you just set.
- Dropped `engage_approach` / `engage_route_follow` stubs from the
  tool schema — they were advertised, disclaimed, and only returned
  errors.

### Fixed

- Hold-short taxi stalls on live X-Plane. The final turn onto the
  hold-short used to leave the aircraft with brakes on, rudder hard
  over, heading 40°+ off target, and no motion — a C172's nose wheel
  can't pivot a stationary aircraft with differential braking alone.
  The align phase now creeps at `pose_creep_speed_kt` until heading
  is in tolerance, plus related sequencing fixes for the taxi →
  line-up handoff.

## [0.3.0] — 2026-04-19

### Added

- Principled ATC pattern tools and a "closest runway" SQL template
  for the airport-data query layer.

### Changed

- Downwind descent now starts abeam the numbers instead of at an
  arbitrary waypoint further along the leg.
- Safety-monitor bank ceilings raised: enroute 25° → 45°, pattern
  20° → 30°.
- Pitch controller proportional gain lowered to kill the fast
  oscillation that appeared in climb and downwind.
- Rotating conversation history grows unbounded: compaction
  (`MAX_INPUT_CHARS` / `compact_if_needed`) has been removed, so
  long sessions no longer drop oldest turns.
- Terse log mode in the TUI now suppresses the
  `[llm-worker] tokens …` usage lines.

## [0.2.0] — 2026-04-19

### Added

- `--pilot-mode normal|realistic` CLI flag and `/mode <name>` TUI
  slash command. Realistic mode layers real-world ATC phraseology,
  mandatory readbacks, stabilized-approach criteria, and procedural
  discipline onto the base prompt. Startup delivery bakes the overlay
  into the pinned system prompt; runtime `/mode` delivery injects it
  as a `[MODE_SWITCH]` user message so the system-prompt prefix stays
  byte-stable and prompt caching is preserved.
- `parking_brake_set` (bool) and `parking_brake_ratio` (f64) in the
  `get_status` tool output and the heartbeat status payload, so the
  pilot can see brake state without an extra tool call.

### Changed

- **Breaking**: default backend is now `xplane`. `sim-pilot` with no
  flags tries to connect to a running X-Plane 12 web API on port
  8086 and expects `OPENAI_API_KEY` in a `.env`. The offline
  deterministic simulator now requires `--backend simple`.
- **Breaking**: Cargo package renamed `xplane-pilot` →
  `an-llm-flies-xplane`. Release archives change to
  `an-llm-flies-xplane-<target>.tar.xz`/`.zip`. The library name
  (`xplane_pilot`) and binary name (`sim-pilot`) are unchanged, so
  `use xplane_pilot::...` imports and shell invocations still work.
- `engage_taxi` auto-releases the parking brake if it is set,
  instead of erroring. The summary reports the ratio it released.
- Hold-short pose tolerance widened from 10 ft / 5° to 20 ft / 20°
  so the pose-align phase converges reliably on a 172 (the free-
  castering nose wheel cannot reliably pivot in place at idle
  throttle, leaving the old 5° heading tolerance unreachable and
  the profile stuck reporting `is_complete() == false`).
- Go-around lateral track-hold gain raised 0.35 → 1.0 to fight
  P-factor drift from firewalled climb power. Pattern and crosswind
  phases keep the original 0.35.
- System prompt reflowed and trimmed (cosmetic); realistic-overlay
  wording tightened.

### Fixed

- Heartbeat and status now report the correct `go_around_reason` on
  the transition tick. Previously the profile metadata was captured
  before `contribute()` ran, so the tick that transitioned to
  `GoAround` always logged `unknown` even for manual triggers or
  safety-monitor catches.

## [0.1.0] — 2026-04-19

- First tagged release. Prebuilt archives for macOS (aarch64 +
  x86_64), Linux (x86_64-gnu), and Windows (x86_64-msvc) built via
  cargo-dist.
