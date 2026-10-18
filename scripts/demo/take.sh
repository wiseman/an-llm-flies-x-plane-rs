#!/usr/bin/env bash
# Record a demo take with the pilot settings the demo is tuned for.
#
#   scripts/demo/take.sh demo/kvny-16r.yaml [more sim-pilot flags]
#
# X-Plane 12 must be running (web API on 8086). OBS is optional
# (obs-websocket on 4455; OBS_WS_PASSWORD in .env if it has a password).
# F6 rolls the take and sends the next cue, F7 holds the cues, F8 skips one.
set -euo pipefail
cd "$(dirname "$0")/../.."
script="${1:?usage: scripts/demo/take.sh demo/<script>.yaml [flags]}"
shift
nice -n 5 cargo build --release --bin sim-pilot
exec ./target/release/sim-pilot --demo "$script" \
  --pilot-llm-provider claude-code --pilot-llm-model claude-sonnet-5-5 \
  --pilot-mode realistic "$@"
