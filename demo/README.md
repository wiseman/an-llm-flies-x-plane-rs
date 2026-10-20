# Demo takes

A demo script (`demo/*.yaml`) starts X-Plane in a fixed place and scripts the
ATC side of a flight. Each call fires on a cue: a time, a flight condition, the
pilot's readback, or a turn. Calls are spoken through a radio-filtered voice, and
the pilot's own transmissions are spoken too. The script can also switch OBS
scenes and X-Plane cameras along the way. Every take writes a timeline, which is
used for B-roll and for rebuilding the radio audio.

## One-time setup

- **Voices.** The default is Kokoro, which runs locally from `~/.venvs/kokoro` and
  needs no setup.
  - To use ElevenLabs instead, put `ELEVENLABS_API_KEY=...` in `.env`. Then set
    `backend: elevenlabs` and a `voice:` (an ElevenLabs voice id) under `audio:`
    in the script.
  - Try a voice: `cargo run --release --bin demo -- audition "Skyhawk 2SP, Van Nuys Tower, extend downwind" --speaker atc`.
- **Headset.** Push-to-talk listens on the Mac's default mic, and the radio
  voices play on its default output: `swift scripts/demo/audio_device.swift`
  lists devices, and `... input HyperX` / `... output HyperX` set them.
- **OBS** (28+): Tools → WebSocket Server Settings → enable it.
  - If you set a password, put `OBS_WS_PASSWORD=...` in `.env`.
  - Size X-Plane to 16:9 first: `swift scripts/demo/xplane_window.swift 0 30 2456 1410`.
  - `uv run scripts/demo/obs_setup.py --terminal "<window title>"` builds a
    "sim-pilot demo" scene collection: X-Plane with a terminal inset, the
    reverse, and each alone, plus the headset mic (track 2) and desktop audio
    (track 3). Window captures follow a window id: `take.sh` rebinds the
    X-Plane capture before every take (and won't start if OBS can't see
    X-Plane); re-run the setup yourself after opening a new terminal
    (`--list` shows the window names).
  - The scripted take switches scenes itself, so build the ones it names
    (`obs.scenes`): **Main** (X-Plane plus a terminal inset), **Code**
    (terminal log pane), **X-Plane**, **Terminal**. Add a text source called
    **Radio Caption** to any scene that should show the current transmission.
  - Capture desktop audio so the radio voices are recorded. If you don't,
    `demo mixdown` can rebuild the radio track afterwards.
- **X-Plane commands.** With X-Plane running, `cargo run --release --bin demo -- discover`
  lists the `sim/view` and `sim/replay` commands. Fix any camera names the
  script uses that don't exist.

## A take

```bash
cargo run --release --bin demo -- check demo/kvny-16r.yaml   # cues, audio, OBS, X-Plane
scripts/demo/take.sh demo/kvny-16r.yaml
```

1. The take starts the flight, lets it settle on the autopilot for a few
   seconds, and rolls by itself: the first cue starts recording and the rest
   fire on their own (timers, the pilot's readbacks, the end of the 360,
   touchdown). The TUI looks as it does in any session.
2. When the pilot flies a plan, the terminal shows its code for 12 s, then goes
   back to the spatial view.
3. Unmarked overrides, for a cue whose condition never comes: **F6** sends the
   next cue now, **F7** holds the cues (again to resume), **F8** skips one.
   With `roll: manual` in the script, the take instead pauses after settling
   until F6.
4. Files: `output/sim_pilot-STEM.{log,csv,kml}` and `output/sim_pilot-STEM.demo.jsonl`
   (the timeline).

To rehearse a script offline (no X-Plane, simple sim, no audio), run:

```bash
cargo run --release --bin eval -- --scenario demo/kvny-16r.yaml \
  --pilot-llm-provider claude-code --pilot-llm-model claude-sonnet-5-5 --pilot-mode realistic
```

## A freeform take: you're the tower

```bash
scripts/demo/take.sh demo/kvny-16r-freeform.yaml
```

The script has no cues; you work the pilot live over push-to-talk.

1. X-Plane loads the 172 on a right downwind for 16R in the cockpit view, and
   the TUI opens as usual.
2. After a few seconds of settling on the autopilot, the take rolls by itself:
   OBS starts recording, then the pilot gets its briefing. Its first call to
   you is on the recording.
3. Talk as tower: hold **Tab**, speak, release, then press **Enter**. The
   transcript lands in the input line first, so you can fix a misheard word
   before sending. The pilot reads back aloud in its radio voice.
   - Wear headphones, so your mic doesn't pick up the pilot.
   - Record your mic in OBS: `demo mixdown` only rebuilds the pilot's voice.
4. Switch OBS scenes yourself (scene hotkeys, or the phone remote). The
   terminal opens on the spatial view; when the pilot flies a plan it shows
   the code for 12 s, then goes back. Switch the view yourself (Ctrl-V) and
   it stays where you put it.
5. Printable one-sheet with setup, controls and two ATC scripts:
   `demo/take-sheet.pdf` (from `demo/take-sheet.html`).
6. Quit the TUI to end the take. That stops the recording.

Every ATC call you send goes into the timeline. B-roll shots can be timed to
your calls:
- `at: "atc:base"` is the first call containing "base"; `atc:base#2` is the
  second.
- `pilot:<words>` matches the pilot's transmissions the same way.
- `event:fly` is a plan the pilot flew, `event:touchdown` the landing.

## B-roll and audio

```bash
# In X-Plane: open the replay, rewind to the start of the take, pause.
cargo run --release --bin demo -- broll --take output/sim_pilot-STEM --demo demo/kvny-16r.yaml --pass cut
cargo run --release --bin demo -- mixdown --take output/sim_pilot-STEM --demo demo/kvny-16r.yaml
```

- `broll` records the replay again, cutting cameras at the pass's shots: `start`,
  `cue:<id>` or `event:touchdown`, each with an offset. It works out where the
  replay is in the take by matching the aircraft's position to the take's track.
  It writes `STEM.broll.<pass>.json`.
- `mixdown` writes `STEM.radio.wav`: every transmission, placed where it started,
  measured from the take's recording start.
