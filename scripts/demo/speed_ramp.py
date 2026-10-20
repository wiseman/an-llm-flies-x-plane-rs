# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy", "scipy", "soundfile", "pyyaml", "pyloudnorm"]
# ///
"""Speed-ramp a take's OBS recording: radio at 1x, everything else fast.

    uv run scripts/demo/speed_ramp.py ~/Movies/"2026-10-19 09-44-11.mov" \\
        output/sim_pilot-20261019-094401

Comms are
- the pilot's radio (each `pilot_audio_start` / `atc_audio_start` in the
  take's .demo.jsonl, lasting as long as its clip in the TTS cache, and
  placed by cross-correlating the clip with the desktop-audio track), and
- the human's speech on the mic track while a push-to-talk key was held
  ("voice: listening" in the take's .log, until the call was sent, the key
  released or the next press).

Each run of comms whose gaps are under --merge-gap-s plays at 1x with
--pad-s on each side; the rest plays at --speed. The OBS tracks are
1 = mix, 2 = mic, 3 = desktop audio (X-Plane plus the pilot's radio voice).

The audio is rebuilt from tracks 2 and 3 (--mix tracks): the mic raised so
speech matches the pilot voice's loudness (--mic-gain-db auto, then a
-1 dBFS limiter), the desktop turned down (--desktop-gain-db), and the
pilot's cached clips laid back in where they played (--pilot-overlay) so
the voice keeps its level while X-Plane gets quieter. --mix full uses
OBS's track 1 instead. In the fast stretches the audio is sped up to match
with ffmpeg's atempo (pitch kept), or silent with --fast-audio mute. Every
cut gets a short fade.

Writes "<recording stem> - comms 1x.mp4" next to the recording unless --out
is given. Intermediate files go in --work-dir (a temp dir by default).
--dry-run prints the plan without rendering. Heavy ffmpeg work runs under
`nice -n 19`.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import soundfile as sf
import yaml
from scipy.signal import butter, fftconvolve, resample_poly, sosfilt

FPS = 30
ENV_HOP_S = 0.05
ANALYSIS_SR = 16_000
OUT_SR = 48_000


@dataclass
class Comm:
    start: float  # seconds in the recording
    end: float
    who: str
    text: str
    clip: str | None = None  # the TTS clip that played (radio comms)


def run(cmd: list[str], nice: bool = True) -> None:
    if nice:
        cmd = ["nice", "-n", "19", *cmd]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"command failed: {' '.join(cmd)}\n{r.stderr[-3000:]}")


def probe_duration(path: Path) -> float:
    out = subprocess.check_output(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)], text=True
    )
    return float(out.strip())


# ---------------------------------------------------------------- timeline

def read_timeline(stem: Path) -> list[dict]:
    events = []
    for line in stem.with_suffix(".demo.jsonl").read_text().splitlines():
        if line.strip():
            e = json.loads(line)
            e["wall"] = dt.datetime.fromisoformat(e["wall_utc"])
            events.append(e)
    return events


def recording_t0(events: list[dict]) -> dt.datetime:
    """Same anchor as `demo mixdown`: the record start, else the roll."""
    for e in events:
        if e["kind"] == "record" and e.get("action") == "start":
            return e["wall"]
    for e in events:
        if e["kind"] == "roll":
            return e["wall"]
    sys.exit("timeline has no record start or roll")


def read_log(stem: Path) -> list[tuple[dt.datetime, str]]:
    """(local wall time as an aware datetime, text) for each timestamped line."""
    out = []
    for line in stem.with_suffix(".log").read_text(errors="replace").splitlines():
        m = re.match(r"(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d) (.*)", line)
        if m:
            out.append((dt.datetime.fromisoformat(m[1]).astimezone(), m[2]))
    return out


# ---------------------------------------------------------------- TTS clips

def fnv64(s: str) -> int:
    h = 0xCBF29CE484222325
    for b in s.encode():
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


DEFAULT_VOICES = {
    "atc": {"backend": "kokoro", "voice": "am_michael", "speed": 1.1, "preset": "atc"},
    "pilot": {"backend": "kokoro", "voice": "am_adam", "speed": 1.05, "preset": "pilot"},
}


def find_demo_voices(stem: Path, log: list[tuple[dt.datetime, str]], demo: Path | None) -> dict:
    """The take's audio voices, from --demo or the demo script named in the log."""
    if demo is None:
        demo_id = next((m[1] for _, t in log if (m := re.search(r"\[demo\] demo (\S+?):", t))), None)
        demo_dir = stem.resolve().parent.parent / "demo"
        if demo_id and demo_dir.is_dir():
            for p in sorted(demo_dir.glob("*.yaml")):
                try:
                    if (yaml.safe_load(p.read_text()) or {}).get("id") == demo_id:
                        demo = p
                        break
                except yaml.YAMLError:
                    pass
    voices = {k: dict(v) for k, v in DEFAULT_VOICES.items()}
    if demo:
        audio = (yaml.safe_load(demo.read_text()) or {}).get("audio") or {}
        for k in ("atc", "pilot"):
            if k in audio:
                voices[k] = audio[k]
        print(f"voices from {demo}")
    else:
        print("voices: demo script not found, using defaults")
    return voices


def clip_path(voice: dict, spoken: str) -> Path:
    """Mirror of radio_voice.rs Voices::render's cache key."""
    backend = str(voice["backend"]).capitalize()  # Debug of TtsBackend
    speed = voice.get("speed")
    speed_s = "None" if speed is None else f"Some({float(speed)!r})"
    settings = voice.get("settings")
    settings_s = "" if settings is None else json.dumps(settings, sort_keys=True, separators=(",", ":"))
    preset = voice.get("preset") or "atc"
    key = f"v1|{backend}|{voice['voice']}|{speed_s}|{settings_s}|{preset}|{spoken}"
    return Path.home() / ".cache/sim_pilot/tts" / f"{fnv64(key):016x}.radio.wav"


# ---------------------------------------------------------------- audio

def extract_audio(rec: Path, work: Path) -> dict[str, Path]:
    """Analysis WAVs of the mic and desktop tracks, and the mix at full rate."""
    paths = {"mic": work / "track2_mic.wav", "desk": work / "track3_desk.wav", "mix": work / "track1_mix.wav",
             "mic48": work / "track2_mic48.wav", "desk48": work / "track3_desk48.wav"}
    if all(p.exists() for p in paths.values()):
        return paths
    print("extracting audio tracks ...")
    run([
        "ffmpeg", "-v", "error", "-y", "-i", str(rec),
        "-map", "0:a:1", "-ac", "1", "-ar", str(ANALYSIS_SR), str(paths["mic"]),
        "-map", "0:a:2", "-ac", "1", "-ar", str(ANALYSIS_SR), str(paths["desk"]),
        "-map", "0:a:0", "-ac", "2", "-ar", str(OUT_SR), "-c:a", "pcm_s16le", str(paths["mix"]),
        "-map", "0:a:1", "-ac", "2", "-ar", str(OUT_SR), "-c:a", "pcm_s16le", str(paths["mic48"]),
        "-map", "0:a:2", "-ac", "2", "-ar", str(OUT_SR), "-c:a", "pcm_s16le", str(paths["desk48"]),
    ])
    return paths


def radio_comms(events, t0, desk_wav: Path, voices: dict, search_s: float) -> tuple[list[Comm], float]:
    """Pilot/scripted-ATC transmissions, aligned to the desktop track."""
    desk, sr = sf.read(desk_wav, dtype="float32")
    found = []  # (predicted, measured or None, duration, who, text)
    for e in events:
        who = {"pilot_audio_start": "pilot", "atc_audio_start": "atc-tts"}.get(e["kind"])
        if not who or not e.get("spoken"):
            continue
        pred = (e["wall"] - t0).total_seconds()
        p = clip_path(voices["pilot" if who == "pilot" else "atc"], e["spoken"])
        if not p.exists():
            dur = 0.4 * len(e["spoken"].split()) + 0.6
            print(f"  ! no cached clip for {e['spoken'][:50]!r}; guessing {dur:.1f}s")
            found.append((pred, None, dur, who, e["spoken"], None))
            continue
        clip, csr = sf.read(p, dtype="float32")
        if clip.ndim > 1:
            clip = clip.mean(axis=1)
        if csr != sr:
            clip = resample_poly(clip, sr, csr).astype(np.float32)
        dur = len(clip) / sr
        a = max(int((pred - search_s) * sr), 0)
        b = min(int((pred + search_s) * sr) + len(clip), len(desk))
        seg = desk[a:b]
        if len(seg) <= len(clip):
            found.append((pred, None, dur, who, e["spoken"], str(p)))
            continue
        xc = fftconvolve(seg, clip[::-1], mode="valid")
        energy = np.sqrt(np.maximum(fftconvolve(seg**2, np.ones(len(clip)), mode="valid"), 0)) * np.linalg.norm(clip)
        ncc = xc / (energy + 1e-9)
        i = int(np.argmax(ncc))
        measured = (a + i) / sr if ncc[i] > 0.5 else None
        found.append((pred, measured, dur, who, e["spoken"], str(p)))
        tag = f"{measured - pred:+.3f}s ncc {ncc[i]:.2f}" if measured is not None else f"no match (ncc {ncc[i]:.2f})"
        print(f"  {who:7s} predicted {pred:8.2f}  {tag}  {e['spoken'][:50]}")
    offsets = [f[1] - f[0] for f in found if f[1] is not None]
    offset = float(np.median(offsets)) if offsets else 0.0
    if offsets:
        print(f"radio: {len(offsets)}/{len(found)} clips matched; offset median {offset:+.3f}s "
              f"(range {min(offsets):+.3f}..{max(offsets):+.3f})")
    comms = [Comm(m if m is not None else p + offset, (m if m is not None else p + offset) + d, who, t, clip)
             for p, m, d, who, t, clip in found]
    return comms, offset


def mic_envelope(mic_wav: Path) -> np.ndarray:
    x, sr = sf.read(mic_wav, dtype="float32")
    y = sosfilt(butter(4, [150, 4000], btype="band", fs=sr, output="sos"), x)
    hop = int(ENV_HOP_S * sr)
    n = len(y) // hop
    rms = np.sqrt((y[: n * hop].reshape(n, hop) ** 2).mean(axis=1)) + 1e-9
    return 20 * np.log10(rms)


def mic_segments(db: np.ndarray, edge_db: float, peak_db: float, gap_s: float) -> list[tuple[float, float, float]]:
    """(start, end, peak dB) runs above edge_db, merged across short gaps, that reach peak_db."""
    on = np.concatenate([[False], db > edge_db, [False]])
    d = np.diff(on.astype(int))
    starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    merged: list[list[int]] = []
    for a, b in zip(starts, ends):
        if merged and (a - merged[-1][1]) * ENV_HOP_S < gap_s:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    return [(a * ENV_HOP_S, b * ENV_HOP_S, float(db[a:b].max())) for a, b in merged if db[a:b].max() >= peak_db]


def ptt_windows(log, events, t0, offset, slack_s: float, max_s: float) -> list[tuple[float, float, str]]:
    """Push-to-talk holds, in recording seconds: from each "voice: listening"
    to the call it produced (atc_call / operator message / release), else the
    next press. Log times are whole seconds, so widen by slack_s."""
    ft = lambda w: (w - t0).total_seconds() + offset  # noqa: E731
    marks = []  # (time, kind, text)
    for w, t in log:
        if "voice: listening" in t:
            marks.append((ft(w), "press", ""))
        elif "voice: finalizing" in t:
            marks.append((ft(w) + 1.0, "release", ""))
        elif "[operator]" in t:
            marks.append((ft(w) + 1.0, "sent", t.split("[operator]", 1)[1].strip()))
    for e in events:
        if e["kind"] == "atc_call":
            marks.append((ft(e["wall"]), "sent", e.get("text", "")))
    marks.sort()
    wins = []
    for i, (t, k, _) in enumerate(marks):
        if k != "press":
            continue
        end, text = t + max_s, ""
        for t2, k2, txt in marks[i + 1:]:
            if k2 == "press":
                end = min(end, t2)
                break
            if t2 > t + max_s:
                break
            end, text = t2, txt
            if k2 in ("sent", "release"):
                # A release with the sent text a moment later: take the text too.
                for t3, k3, txt3 in marks[i + 1:]:
                    if k3 == "sent" and t2 <= t3 <= t2 + 1.5:
                        text = txt3
                break
        wins.append((t - slack_s, end + 0.5, text))
    return wins


def human_comms(db, wins, edge_db, peak_db, gap_s) -> tuple[list[Comm], list, list]:
    segs = mic_segments(db, edge_db, peak_db, gap_s)
    comms, unkeyed, used = [], [], set()
    for a, b, pk in segs:
        hit = [i for i, (wa, wb, _) in enumerate(wins) if a < wb and b > wa]
        if hit:
            used.update(hit)
            best = max(hit, key=lambda i: min(b, wins[i][1]) - max(a, wins[i][0]))  # most overlap
            comms.append(Comm(a, b, "human", wins[best][2]))
        else:
            unkeyed.append((a, b, pk))
    silent = [w for i, w in enumerate(wins) if i not in used]
    return comms, unkeyed, silent


# ---------------------------------------------------------------- mix

def db(x: float) -> float:
    return 20 * math.log10(max(x, 1e-12))


def limit(pcm: np.ndarray, work: Path, name: str, ceiling_db: float) -> np.ndarray:
    """ffmpeg's lookahead limiter, delay-compensated so sync is kept."""
    src, dst = work / f"{name}.in.wav", work / f"{name}.out.wav"
    sf.write(src, pcm, OUT_SR, subtype="FLOAT")
    run(["ffmpeg", "-v", "error", "-y", "-i", str(src), "-af",
         f"alimiter=limit={10 ** (ceiling_db / 20):.4f}:attack=5:release=80:level=0:latency=1",
         "-c:a", "pcm_f32le", str(dst)])
    out, _ = sf.read(dst, dtype="float32", always_2d=True)
    src.unlink()
    dst.unlink()
    if len(out) < len(pcm):
        out = np.vstack([out, np.zeros((len(pcm) - len(out), 2), np.float32)])
    return out[: len(pcm)]


def build_track_mix(paths, comms: list[Comm], work: Path, mic_gain_db: float | None, desk_gain_db: float | None,
                    pilot_gain_db: float, overlay: bool, mic_gate: str = "segments",
                    engine_under_lu: float = 20.0) -> Path:
    """Track 2 (mic) raised to the pilot's voice level, track 3 (desktop)
    turned down, and the pilot's clips laid back in where they played so
    the radio voice keeps its level. Prints the levels it measures."""
    import pyloudnorm as pyln

    meter = pyln.Meter(OUT_SR)
    lufs = lambda x: meter.integrated_loudness(x) if len(x) > OUT_SR // 2 else float("nan")  # noqa: E731
    cat = lambda x, ivs: np.concatenate([x[int(a * OUT_SR):int(b * OUT_SR)] for a, b in ivs]) if ivs else x[:0]  # noqa: E731
    mic, _ = sf.read(paths["mic48"], dtype="float32", always_2d=True)
    desk, _ = sf.read(paths["desk48"], dtype="float32", always_2d=True)
    n = min(len(mic), len(desk))
    mic, desk = mic[:n], desk[:n]
    human_iv = [(c.start, c.end) for c in comms if c.who.startswith("human")]
    radio_iv = [(c.start, c.end) for c in comms if c.clip]
    busy = np.zeros(n, bool)
    for c in comms:
        busy[max(int((c.start - 1) * OUT_SR), 0):int((c.end + 1) * OUT_SR)] = True

    # The played clips, placed to the sample and with the gain they played at.
    voices = []  # (sample offset, mono clip, [gain L, gain R])
    for c in comms:
        if not c.clip:
            continue
        clip, csr = sf.read(c.clip, dtype="float32")
        if clip.ndim > 1:
            clip = clip.mean(axis=1)
        if csr != OUT_SR:
            clip = resample_poly(clip, OUT_SR, csr).astype(np.float32)
        a = max(int(c.start * OUT_SR) - 400, 0)
        seg = desk[a:a + len(clip) + 800]
        if len(seg) < len(clip) + 1:
            continue
        i = int(np.argmax(fftconvolve(seg.mean(axis=1), clip[::-1], mode="valid")))
        s = seg[i:i + len(clip)]
        g = [float(np.dot(s[:, ch], clip) / np.dot(clip, clip)) for ch in (0, 1)]
        voices.append((a + i, clip, g))
    pilot_voice = np.concatenate([np.outer(cl, g) for _, cl, g in voices]) if voices else np.zeros((0, 2), np.float32)

    human_raw = lufs(cat(mic, human_iv))
    pilot_lufs = lufs(pilot_voice)
    engine_lufs = lufs(desk[~busy])
    gains = [db(sum(g) / 2) for _, _, g in voices]
    print("\nlevels in the recording (integrated LUFS, BS.1770, stereo):")
    print(f"  track 2 human speech ({sum(b - a for a, b in human_iv):.0f}s): {human_raw:.1f} LUFS, "
          f"peak {db(float(np.abs(cat(mic, human_iv)).max(initial=0))):.1f} dBFS; "
          f"mic between comms {db(float(np.sqrt((mic[~busy] ** 2).mean()))):.1f} dBFS RMS")
    print(f"  track 3 pilot voice alone: {pilot_lufs:.1f} LUFS (clips played at "
          f"{np.median(gains) if gains else float('nan'):+.2f} dB of the cached file)")
    print(f"  track 3 engine/sim between comms: {engine_lufs:.1f} LUFS")
    print(f"  track 3 during pilot transmissions (voice + engine): {lufs(cat(desk, radio_iv)):.1f} LUFS")
    # For tuning OBS's own noise gate: raw mic levels, in 50 ms windows.
    hop = OUT_SR // 20
    def frames_db(x):
        k = len(x) // hop
        return 20 * np.log10(np.sqrt((x[: k * hop].mean(axis=1).reshape(k, hop) ** 2).mean(axis=1)) + 1e-9)
    sp = cat(mic, human_iv)
    quiet = mic[~busy]
    qdb = frames_db(quiet) if len(quiet) > hop else np.array([-200.0])
    if len(sp):
        sdb = frames_db(sp)
        print(f"  track 2 speech, 50 ms RMS: median {np.median(sdb):.1f}, 90th pct {np.percentile(sdb, 90):.1f}, "
              f"max {sdb.max():.1f} dBFS; overall RMS {db(float(np.sqrt((sp ** 2).mean()))):.1f} dBFS")
    print(f"  track 2 between comms, 50 ms RMS: median {np.median(qdb):.1f}, 90th {np.percentile(qdb, 90):.1f}, "
          f"99th {np.percentile(qdb, 99):.1f}, max {qdb.max():.1f} dBFS; "
          f"{(qdb < -80).mean() * 100:.0f}% of it below -80 (gated)")

    target = pilot_lufs + pilot_gain_db
    if mic_gain_db is None:
        mic_gain_db = target - human_raw if math.isfinite(human_raw) and math.isfinite(target) else 0.0
    mic_out = limit(mic * 10 ** (mic_gain_db / 20), work, "mic", -1.0)
    # Limiting shaves a little loudness; make it up once.
    got = lufs(cat(mic_out, human_iv))
    if math.isfinite(got) and math.isfinite(target) and 0.3 < abs(target - got) < 6:
        mic_gain_db += target - got
        mic_out = limit(mic * 10 ** (mic_gain_db / 20), work, "mic", -1.0)
    del mic

    if mic_gate == "segments":
        # Only the detected push-to-talk speech, with a little pre/post-roll and short fades.
        gate = np.zeros(n, np.float32)
        pre, post, fade = int(0.15 * OUT_SR), int(0.30 * OUT_SR), int(0.05 * OUT_SR)
        ramp = np.linspace(0, 1, fade, dtype=np.float32)
        for a, b in human_iv:
            i0, i1 = max(int(a * OUT_SR) - pre, 0), min(int(b * OUT_SR) + post, n)
            gate[i0:i1] = 1.0
            k = min(fade, i1 - i0)
            gate[i0:i0 + k] = np.minimum(gate[i0:i0 + k], ramp[:k])
            gate[i1 - k:i1] = np.minimum(gate[i1 - k:i1], ramp[:k][::-1])
        mic_out *= gate[:, None]
        print(f"mic gated to {len(human_iv)} speech segments ({gate.sum() / OUT_SR:.0f}s open)")

    if desk_gain_db is None:
        desk_gain_db = (target - engine_under_lu) - engine_lufs if math.isfinite(engine_lufs) else 0.0
    gd = 10 ** (desk_gain_db / 20)
    gp = 10 ** (pilot_gain_db / 20)
    mix = mic_out + desk * gd
    del mic_out
    if overlay:
        for off, clip, g in voices:
            end = min(off + len(clip), n)
            mix[off:end] += np.outer(clip[: end - off], g) * (gp - gd)
    print(f"gains: mic {mic_gain_db:+.1f} dB (then limited at -1 dBFS), desktop {desk_gain_db:+.1f} dB "
          f"(engine {engine_under_lu:.0f} LU under the voices if auto), "
          f"pilot voice {pilot_gain_db:+.1f} dB vs as played "
          f"({'re-added from ' + str(len(voices)) + ' clips' if overlay else 'no overlay'})")
    mix = limit(mix, work, "mix", -1.0)

    print("levels in the new mix:")
    voice_out = [np.outer(cl, g) * (gp if overlay else gd) for _, cl, g in voices]
    print(f"  human speech windows: {lufs(cat(mix, human_iv)):.1f} LUFS")
    print(f"  pilot voice: {lufs(np.concatenate(voice_out)) if voice_out else float('nan'):.1f} LUFS; "
          f"pilot windows (voice + engine): {lufs(cat(mix, radio_iv)):.1f} LUFS")
    print(f"  engine between comms: {lufs(desk[~busy] * gd):.1f} LUFS")
    out = work / "track_mix.wav"
    sf.write(out, mix, OUT_SR, subtype="PCM_16")
    return out


# ---------------------------------------------------------------- plan

def plan_segments(comms: list[Comm], total: float, pad: float, merge_gap: float, tail_s: float = 0.0):
    """[(start, end, is_1x)] covering [0, total], on the frame grid. The
    last tail_s seconds are 1x whatever the comms."""
    groups: list[list[float]] = []
    for c in sorted(comms, key=lambda c: c.start):
        if groups and c.start - groups[-1][1] < merge_gap:
            groups[-1][1] = max(groups[-1][1], c.end)
        else:
            groups.append([c.start, c.end])
    last_frame = math.floor(total * FPS)
    snap = lambda t, f: min(max(f(t * FPS), 0), last_frame)  # noqa: E731
    fast = []
    for a, b in groups:
        fa, fb = snap(a - pad, math.floor), snap(b + pad, math.ceil)
        if fast and fa <= fast[-1][1]:
            fast[-1][1] = max(fast[-1][1], fb)
        else:
            fast.append([fa, fb])
    if tail_s > 0:
        fa = snap(total - tail_s, math.floor)
        if fast and fa <= fast[-1][1]:
            fast[-1][1] = last_frame
        else:
            fast.append([fa, last_frame])
    segs, cur = [], 0
    for fa, fb in fast:
        if fa > cur:
            segs.append((cur, fa, False))
        segs.append((fa, fb, True))
        cur = fb
    if cur < last_frame:
        segs.append((cur, last_frame, False))
    return segs  # in frames


def out_frames(seg, speed: int) -> int:
    fa, fb, slow = seg
    return fb - fa if slow else math.ceil((fb - fa) / speed)


# ---------------------------------------------------------------- render

def time_compress(pcm: np.ndarray, n_out: int) -> np.ndarray:
    """Speed `pcm` (48 kHz stereo) up to last exactly n_out samples, keeping
    pitch: ffmpeg atempo, chained since one stage tops out at 2x."""
    if len(pcm) == 0 or n_out == 0:
        return np.zeros((n_out, 2), np.float32)
    factor = len(pcm) / n_out
    stages = []
    while factor > 2.0:
        stages.append(2.0)
        factor /= 2.0
    stages.append(max(factor, 0.5))
    r = subprocess.run(
        ["nice", "-n", "19", "ffmpeg", "-v", "error", "-f", "f32le", "-ar", str(OUT_SR), "-ac", "2", "-i", "pipe:0",
         "-af", ",".join(f"atempo={s:.9f}" for s in stages), "-f", "f32le", "-ar", str(OUT_SR), "-ac", "2", "pipe:1"],
        input=np.ascontiguousarray(pcm, dtype=np.float32).tobytes(), capture_output=True,
    )
    if r.returncode != 0:
        sys.exit(f"atempo failed: {r.stderr.decode()[-2000:]}")
    out = np.frombuffer(r.stdout, dtype=np.float32).reshape(-1, 2).copy()
    # atempo lands within a few ms of the target; pad or trim to the frame.
    if len(out) < n_out:
        out = np.vstack([out, np.zeros((n_out - len(out), 2), np.float32)])
    return out[:n_out]


def render(rec: Path, segs, speed: int, mix_wav: Path, work: Path, out: Path, quality: int, fade_s: float,
           fast_audio: str = "speed", map_video: Path | None = None, map_xy: tuple[int, int] = (0, 0)) -> None:
    if map_video:  # a re-rendered map gets fresh segments
        st = map_video.stat()
        vdir = work / f"video_map_{st.st_size:x}_{int(st.st_mtime):x}"
    else:
        vdir = work / "video"
    vdir.mkdir(exist_ok=True)
    listing = []
    for i, seg in enumerate(segs):
        fa, fb, slow = seg
        n_out = out_frames(seg, speed)
        p = vdir / f"seg{i:03d}_{fa}_{fb}_{'1x' if slow else f'{speed}x'}.mp4"  # named by its range: a changed plan re-renders
        listing.append(f"file '{p}'")
        if p.exists() and p.with_suffix(".ok").exists():
            continue
        vf = "fps=30" if slow else f"select='not(mod(n\\,{speed}))',setpts=N/({FPS}*TB),fps={FPS}"
        print(f"  video {i + 1}/{len(segs)} {'1x ' if slow else f'{speed}x'} "
              f"{fa / FPS:8.2f}-{fb / FPS:8.2f} -> {n_out} frames", flush=True)
        seek = ["-ss", f"{fa / FPS:.6f}", "-t", f"{(fb - fa) / FPS + 0.5:.6f}"]
        if map_video:
            # The map inset is composited on the source timeline, then sped up with it.
            inputs = [*seek, "-i", str(rec), *seek, "-i", str(map_video)]
            graph = ["-filter_complex", f"[0:v][1:v]overlay={map_xy[0]}:{map_xy[1]}:eof_action=repeat,{vf}[v]",
                     "-map", "[v]"]
        else:
            inputs = [*seek, "-i", str(rec)]
            graph = ["-map", "0:v:0", "-vf", vf]
        run([
            "ffmpeg", "-v", "error", "-y", "-hwaccel", "videotoolbox", *inputs,
            *graph, "-an", "-frames:v", str(n_out),
            "-c:v", "h264_videotoolbox", "-q:v", str(quality), "-bf", "0", "-g", "60",
            "-pix_fmt", "yuv420p", "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709",
            "-video_track_timescale", "15360", str(p),
        ])
        p.with_suffix(".ok").touch()
    (vdir / "concat.txt").write_text("\n".join(listing) + "\n")
    video = vdir / "video.mp4"
    run(["ffmpeg", "-v", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(vdir / "concat.txt"),
         "-c", "copy", str(video)])

    # Audio: the mix at 1x, and for fast stretches either the mix time-
    # compressed to fit (fast_audio="speed") or silence ("mute"). Every
    # segment gets short fades at both ends, so the cuts don't click.
    audio = work / "audio.wav"
    spf = OUT_SR // FPS
    with sf.SoundFile(mix_wav) as src, sf.SoundFile(audio, "w", OUT_SR, 2, "PCM_16") as dst:
        for seg in segs:
            fa, fb, slow = seg
            n = out_frames(seg, speed) * spf
            if slow:
                src.seek(min(fa * spf, src.frames))
                buf = src.read(n, dtype="float32", always_2d=True)
            elif fast_audio == "speed":
                src.seek(min(fa * spf, src.frames))
                buf = time_compress(src.read((fb - fa) * spf, dtype="float32", always_2d=True), n)
            else:
                buf = np.zeros((n, 2), np.float32)
            if len(buf) < n:
                buf = np.vstack([buf, np.zeros((n - len(buf), 2), np.float32)])
            buf = buf[:n]
            fade = min(int(fade_s * OUT_SR), n // 2)
            if fade:
                ramp = np.linspace(0, 1, fade, dtype=np.float32)[:, None]
                buf[:fade] *= ramp
                buf[-fade:] *= ramp[::-1]
            dst.write(buf)

    tmp_out = work / "final.mp4"
    run(["ffmpeg", "-v", "error", "-y", "-i", str(video), "-i", str(audio), "-map", "0:v:0", "-map", "1:a:0",
         "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-movflags", "+faststart", str(tmp_out)])
    shutil.move(tmp_out, out)


# ---------------------------------------------------------------- main

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", type=Path, help="the OBS recording (tracks: 1 mix, 2 mic, 3 desktop)")
    ap.add_argument("take", type=Path, help="take stem, e.g. output/sim_pilot-20261019-094401")
    ap.add_argument("--out", type=Path, help='default: "<recording stem> - comms 1x.mp4" beside it')
    ap.add_argument("--work-dir", type=Path, help="intermediate files (default: a temp dir)")
    ap.add_argument("--demo", type=Path, help="demo script for the radio voices (default: from the log)")
    ap.add_argument("--speed", type=int, default=16, help="speed-up outside comms (default 16)")
    ap.add_argument("--pad-s", type=float, default=2.0, help="1x padding around each comm group (default 2)")
    ap.add_argument("--merge-gap-s", type=float, default=10.0, help="keep gaps shorter than this at 1x (default 10)")
    ap.add_argument("--mic-edge-db", default="auto",
                    help="mic level (50 ms RMS, 150-4000 Hz, dBFS) that counts as sound; 'auto' (default) is "
                         "-45 or 3 dB over the room's 99th percentile outside push-to-talk, whichever is higher")
    ap.add_argument("--mic-peak-db", default="auto",
                    help="a mic segment must reach this to count as speech; 'auto' (default) is -40 or "
                         "edge + 8, whichever is higher")
    ap.add_argument("--mic-gap-s", type=float, default=0.8, help="join mic segments across gaps this short (default 0.8)")
    ap.add_argument("--ptt-slack-s", type=float, default=1.5, help="widen each push-to-talk start by this (default 1.5)")
    ap.add_argument("--include-unkeyed", action="store_true", help="also count mic speech outside push-to-talk")
    ap.add_argument("--fast-audio", choices=["speed", "mute"], default="speed",
                    help="fast stretches: the mix sped up to match (default), or silence")
    ap.add_argument("--mix", choices=["tracks", "full"], default="tracks",
                    help="audio source: rebuild from the mic and desktop tracks (default), or OBS's track 1 mix")
    ap.add_argument("--mic-gain-db", default="auto",
                    help="mic (track 2) gain; 'auto' (default) brings speech to the pilot voice's loudness")
    ap.add_argument("--desktop-gain-db", default="auto",
                    help="desktop (track 3: X-Plane + pilot voice) gain; 'auto' (default) puts the sim "
                         "--engine-under-lu below the voices")
    ap.add_argument("--engine-under-lu", type=float, default=20.0,
                    help="with --desktop-gain-db auto: how far under the voices the sim sits (default 20)")
    ap.add_argument("--mic-gate", choices=["segments", "none"], default="segments",
                    help="pass the mic only inside the detected push-to-talk speech (default), or always")
    ap.add_argument("--tail-1x-s", type=float, default=0.0,
                    help="keep the recording's last N seconds at 1x (default 0)")
    ap.add_argument("--pilot-gain-db", type=float, default=0.0,
                    help="pilot voice level vs as it played (default 0)")
    ap.add_argument("--pilot-overlay", action=argparse.BooleanOptionalAction, default=True,
                    help="lay the cached pilot clips back in so the voice keeps its level (default on)")
    ap.add_argument("--map", type=Path,
                    help="moving-map inset video from map_overlay.py, composited upper right "
                         "(output gets ' map' in its name)")
    ap.add_argument("--map-margin", type=int, default=24, help="map inset margin from the top and right (default 24)")
    ap.add_argument("--quality", type=int, default=62, help="h264_videotoolbox -q:v (default 62)")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, don't render")
    args = ap.parse_args()

    rec = args.recording.expanduser().resolve()
    total = probe_duration(rec)
    work = args.work_dir or Path(tempfile.mkdtemp(prefix="speed_ramp-"))
    work.mkdir(parents=True, exist_ok=True)
    out = args.out or rec.with_name(f"{rec.stem} - comms 1x{' map' if args.map else ''}.mp4")
    print(f"recording {rec} ({total:.1f}s); work dir {work}")

    events = read_timeline(args.take)
    t0 = recording_t0(events)
    log = read_log(args.take)
    end_ev = next((e for e in events if e["kind"] == "end"), None)
    if end_ev:
        print(f"timeline: record start {t0.isoformat()}, end at {(end_ev['wall'] - t0).total_seconds():.1f}s "
              f"(recording is {total:.1f}s)")
    voices = find_demo_voices(args.take, log, args.demo)
    wavs = extract_audio(rec, work)

    radio, offset = radio_comms(events, t0, wavs["desk"], voices, search_s=4.0)
    db = mic_envelope(wavs["mic"])
    if db.max() < -90:
        print("!! track 2 (mic) is silent: OBS recorded no mic audio, so the human's calls aren't on this "
              "recording (check the mic source's device in OBS)")
    wins = ptt_windows(log, events, t0, offset, args.ptt_slack_s, max_s=30.0)
    keyed = np.zeros(len(db), bool)
    for wa, wb, _ in wins:
        keyed[max(int(wa / ENV_HOP_S), 0):int(wb / ENV_HOP_S) + 1] = True
    for c in radio:  # the radio voice can bleed into the mic
        keyed[max(int(c.start / ENV_HOP_S), 0):int(c.end / ENV_HOP_S) + 1] = True
    room99 = float(np.percentile(db[~keyed], 99)) if (~keyed).any() else -200.0
    edge = max(-45.0, room99 + 3) if str(args.mic_edge_db).lower() == "auto" else float(args.mic_edge_db)
    peak = max(-40.0, edge + 8) if str(args.mic_peak_db).lower() == "auto" else float(args.mic_peak_db)
    print(f"mic detection: room 99th pct {room99:.1f} dB outside push-to-talk -> edge {edge:.1f}, peak {peak:.1f} dB")
    human, unkeyed, silent = human_comms(db, wins, edge, peak, args.mic_gap_s)
    print(f"mic: {len(human)} speech segments in {len(wins)} push-to-talk holds")
    for a, b, pk in unkeyed:
        print(f"  speech outside push-to-talk {a:8.2f}-{b:8.2f} ({b - a:.1f}s, peak {pk:.1f} dB)"
              f"{'' if args.include_unkeyed else ' - skipped'}")
    for wa, wb, txt in silent:
        print(f"  ! push-to-talk {wa:8.2f}-{wb:8.2f} with no speech found: {txt[:60]!r}")
    if args.include_unkeyed:
        human += [Comm(a, b, "human-unkeyed", "") for a, b, _ in unkeyed]

    comms = sorted(radio + human, key=lambda c: c.start)
    print("\ncomms:")
    for c in comms:
        print(f"  {c.start:8.2f}-{c.end:8.2f} {c.end - c.start:5.1f}s {c.who:7s} {c.text[:70]}")

    segs = plan_segments(comms, total, args.pad_s, args.merge_gap_s, args.tail_1x_s)
    slow = [s for s in segs if s[2]]
    out_total = sum(out_frames(s, args.speed) for s in segs) / FPS
    slow_total = sum(s[1] - s[0] for s in slow) / FPS
    print("\nplan:")
    t_out = 0.0
    for s in segs:
        d_out = out_frames(s, args.speed) / FPS
        print(f"  {'1x ' if s[2] else f'{args.speed}x'} src {s[0] / FPS:8.2f}-{s[1] / FPS:8.2f} "
              f"({(s[1] - s[0]) / FPS:6.1f}s) -> out {t_out:7.2f}-{t_out + d_out:7.2f}")
        t_out += d_out
    print(f"\n{len(slow)} 1x segments, {slow_total:.1f}s at 1x; output {out_total:.1f}s "
          f"({out_total / 60:.1f} min) from {total:.1f}s")
    (work / "plan.json").write_text(json.dumps({
        "recording": str(rec), "take": str(args.take), "offset_s": offset,
        "comms": [c.__dict__ for c in comms],
        "segments": [{"start_s": a / FPS, "end_s": b / FPS, "speed": 1 if s else args.speed} for a, b, s in segs],
    }, indent=1))
    if args.dry_run:
        return
    mix_wav = wavs["mix"]
    if args.mix == "tracks":
        auto = lambda v: None if str(v).lower() == "auto" else float(v)  # noqa: E731
        mix_wav = build_track_mix(wavs, comms, work, auto(args.mic_gain_db), auto(args.desktop_gain_db),
                                  args.pilot_gain_db, args.pilot_overlay, args.mic_gate, args.engine_under_lu)
    print(f"\nrendering -> {out}")
    map_xy = (0, 0)
    if args.map:
        mw, mh = (int(v) for v in subprocess.check_output(
            ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height", "-of",
             "csv=p=0", str(args.map)], text=True).strip().split(","))
        vw = int(subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                          "stream=width", "-of", "csv=p=0", str(rec)], text=True).strip())
        map_xy = (vw - args.map_margin - mw, args.map_margin)
        print(f"map inset {mw}x{mh} at x={map_xy[0]}, y={map_xy[1]}")
    render(rec, segs, args.speed, mix_wav, work, out, args.quality, fade_s=0.03, fast_audio=args.fast_audio,
           map_video=args.map, map_xy=map_xy)
    print(f"done: {out} ({probe_duration(out):.1f}s)")


if __name__ == "__main__":
    main()
