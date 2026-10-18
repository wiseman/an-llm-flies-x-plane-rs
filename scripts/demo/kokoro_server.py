# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "kokoro-onnx",
#     "soundfile",
# ]
# ///
"""Persistent Kokoro TTS worker for the demo radio.

Loads the model once, then reads one JSON request per line on stdin:
    {"text": "...", "voice": "am_michael", "speed": 1.1, "out": "/path/raw.wav"}
writes the WAV, and answers one JSON line on stdout:
    {"ok": true, "out": "/path/raw.wav"}   or   {"ok": false, "error": "..."}
Prints {"ready": true} once the model is loaded.

Run with the Kokoro venv (sim-pilot does this):
    ~/.venvs/kokoro/bin/python scripts/demo/kokoro_server.py
or standalone with `uv run scripts/demo/kokoro_server.py`.
Model files: $KOKORO_MODEL / $KOKORO_VOICES, default ~/.cache/hyperframes/tts/.
"""

import json
import os
import sys
from pathlib import Path


def main() -> None:
    from kokoro_onnx import Kokoro
    import soundfile as sf

    base = Path.home() / ".cache/hyperframes/tts"
    model = os.environ.get("KOKORO_MODEL", str(base / "models/kokoro-v1.0.onnx"))
    voices = os.environ.get("KOKORO_VOICES", str(base / "voices/voices-v1.0.bin"))
    kokoro = Kokoro(model, voices)
    # Warm up so the first real line is fast.
    kokoro.create("Ready.", voice="am_michael", speed=1.0, lang="en-us")
    print(json.dumps({"ready": True}), flush=True)

    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            samples, rate = kokoro.create(
                req["text"],
                voice=req.get("voice", "am_michael"),
                speed=float(req.get("speed", 1.0)),
                lang=req.get("lang", "en-us"),
            )
            sf.write(req["out"], samples, rate)
            print(json.dumps({"ok": True, "out": req["out"]}), flush=True)
        except Exception as e:  # report and keep serving
            print(json.dumps({"ok": False, "error": str(e)}), flush=True)


if __name__ == "__main__":
    main()
