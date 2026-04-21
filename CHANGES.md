# Changelog

All notable user-facing changes to this project are documented here.
The format is loosely based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the
project follows [Semantic Versioning](https://semver.org).

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
