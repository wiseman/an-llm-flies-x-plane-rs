# Changelog

All notable user-facing changes to this project are documented here.
The format is loosely based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the
project follows [Semantic Versioning](https://semver.org).

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
