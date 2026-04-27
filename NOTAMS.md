# NOTAMs

Operator-level summary of what's landed in each release, in pilot
language. For code-level detail see `CHANGES.md`.

## [0.6.0] — 2026-04-27

- Fly with Anthropic Claude or Google Gemini, not just OpenAI. Pick
  the brain at startup with `--pilot-llm-provider anthropic` or
  `--pilot-llm-provider gemini`.
- New cockpit-style display: status up top, energy + risk alongside,
  radio and action log side-by-side below, input at the bottom. On
  narrow terminals the radio stacks above the log so ATC stays
  visible.
- Flight sessions render to a standalone HTML transcript — radio and
  ATC traffic show up as timeline events alongside pilot actions,
  with a mission-complete summary footer.
- Six-model bench-off comparing OpenAI, Anthropic, and Gemini models
  flying the same scenario end-to-end. Results table in the README.
- The pilot's thinking is more visible — Gemini reasoning traces
  surface in the action log, and a propeller spinner appears while
  the model is working.
- Pattern altitude held until ~1.2 nm from the runway before
  descent. Fewer low-and-slow base legs.
- Default downwind offset widened — more room to set up the descent.
- Quieter elevator on combined altitude + speed holds.

## [0.5.0] — 2026-04-22

- After landing the pilot leaves the runway. Rollout decelerates to
  a turnoff speed, you pick an exit, and the aircraft is stopped
  clear of the pavement, released to you to taxi.
- The pilot can park. Name a gate or tie-down and the pilot taxis
  there and stops on the painted heading.
- No more backtaxi down runways — "taxi to runway X" reliably lands
  at the approach end, not mid-field.
- Operator replies stop getting dropped when the pilot also runs a
  tool. Previously any response that paired commentary with an
  action looked like the pilot ignored you.
- Log and radio panes restyled — colored left-gutter bars per
  subsystem, operator turns in a subtle "user bubble" background,
  pilot vs ATC transmissions color-coded.
- The pilot behaves more like a pilot: won't doze off with the
  aircraft coasting uncontrolled, acts on tool-error remediation
  instead of escalating to you, and sticks to stable flares instead
  of bailing for cosmetic reasons.

## [0.4.0] — 2026-04-20

- Intersection departures work end-to-end — "taxi to runway 19 at
  Charlie, cleared for takeoff at Charlie" does the right thing.

## [0.3.0] — 2026-04-20

- Can now respond better to ATC pacing the pattern — extend a leg, force a turn
- Downwind descent now starts abeam the numbers, not somewhere
  further along.
- Pattern banks can reach 30°, enroute banks 45° — no more refusing
  normal manoeuvres.
- Long sessions no longer silently drop the oldest conversation; the
  pilot keeps full context.

## [0.2.0] — 2026-04-19

- Realistic pilot mode — real-world phraseology, mandatory readbacks,
  stabilized-approach criteria. Switch at startup with
  `--pilot-mode realistic` or mid-session via `/mode realistic`.
- Hold-short line-ups converge reliably
- Much better crosswind compensation

## [0.1.0] — 2026-04-19

First tagged release. The pilot can fly a pattern end-to-end from
X-Plane 12:

- Taxi from the ramp to the hold-short of any runway in the world,
  planned from apt.dat's taxi network.
- Line up, take off, climb, fly the pattern, roundout, flare, land,
  roll out — all through the autopilot; the LLM only makes piloting
  decisions.
- Voice (push-to-talk) for operator and ATC channels via Deepgram.
- Airspace awareness heartbeats — Class B/C/D, restricted, prohibited
  — on current and projected track.
- Interactive TUI with status / log / radio / input panes, plus a
  flight-track CSV + KML written on shutdown.
- Offline deterministic simulator for tests and headless scenarios
  (`--backend simple`).
- Prebuilt binaries for macOS (aarch64 + x86_64), Linux x86_64,
  Windows x86_64.
