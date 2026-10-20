# /// script
# requires-python = ">=3.11"
# dependencies = ["obsws-python>=1.7"]
# ///
"""Set up OBS for a demo take, over obs-websocket.

    uv run scripts/demo/obs_setup.py --list          # windows and mics OBS can see
    uv run scripts/demo/obs_setup.py --terminal sim-pilot

Builds (or updates) the scene collection "sim-pilot demo", leaving other
collections alone:

- Scenes: "Cockpit + Terminal" (X-Plane with a terminal inset), "Terminal +
  Cockpit" (the reverse), "Cockpit", "Terminal".
- Sources: X-Plane and terminal window captures, the Mac's default mic (the
  one push-to-talk hears), and desktop
  audio (X-Plane and the radio voices).
- Audio tracks: 1 is the mix, 2 the mic alone, 3 desktop audio alone. The
  profile records all three.

Window captures bind to a window id, which changes when the window is
reopened, so re-run this before a take if X-Plane or the terminal window
was restarted. Reads OBS_WS_URL / OBS_WS_PASSWORD from .env.
"""

import argparse
import logging
import os
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

import obsws_python as obs

COLLECTION = "sim-pilot demo"
XPLANE, TERMINAL, MIC, DESKTOP = "X-Plane Window", "Terminal Window", "Headset Mic", "Desktop Audio"
MAIN, REVERSE, COCKPIT, TERM = "Cockpit + Terminal", "Terminal + Cockpit", "Cockpit", "Terminal"
INSET = 0.42  # inset size, as a fraction of the canvas
# The terminal inset over the cockpit gets twice the area: its text has to
# read on a video.
TERMINAL_INSET = INSET * 2 ** 0.5
MARGIN = 24  # px between the inset and the canvas edge
TITLEBAR = 28  # px of macOS window title bar, cropped off every window capture

# Alignment flags (obs_alignment): center is 0.
LEFT, RIGHT, TOP, BOTTOM = 1, 2, 4, 8


def load_env():
    env = Path(__file__).resolve().parents[2] / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            key, sep, value = line.partition("=")
            if sep and not line.lstrip().startswith("#"):
                os.environ.setdefault(key.strip(), value.strip())


def connect():
    url = urlparse(os.environ.get("OBS_WS_URL", "ws://127.0.0.1:4455"))
    return obs.ReqClient(
        host=url.hostname,
        port=url.port or 4455,
        password=os.environ.get("OBS_WS_PASSWORD", ""),
        timeout=10,
    )


def call(c, request, data=None):
    return c.send(request, data, raw=True) or {}


def use_collection(c):
    listing = call(c, "GetSceneCollectionList")
    if listing["currentSceneCollectionName"] == COLLECTION:
        return
    if COLLECTION in listing["sceneCollections"]:
        call(c, "SetCurrentSceneCollection", {"sceneCollectionName": COLLECTION})
    else:
        call(c, "CreateSceneCollection", {"sceneCollectionName": COLLECTION})
    # The switch finishes asynchronously; requests fail until it does.
    for _ in range(50):
        time.sleep(0.2)
        try:
            if call(c, "GetSceneCollectionList")["currentSceneCollectionName"] == COLLECTION:
                call(c, "GetSceneList")
                return
        except Exception:
            pass
    sys.exit(f"OBS didn't switch to the {COLLECTION!r} collection")


def inputs(c):
    return {i["inputName"]: i for i in call(c, "GetInputList")["inputs"]}


def ensure_input(c, scene, name, kind, settings):
    if name in inputs(c):
        call(c, "SetInputSettings", {"inputName": name, "inputSettings": settings, "overlay": True})
    else:
        call(c, "CreateInput", {
            "sceneName": scene, "inputName": name, "inputKind": kind,
            "inputSettings": settings, "sceneItemEnabled": True,
        })


def items(c, input_name, prop):
    """The choices of a list property, retried while ScreenCaptureKit fills them."""
    for _ in range(20):
        got = call(c, "GetInputPropertiesListPropertyItems",
                   {"inputName": input_name, "propertyName": prop})["propertyItems"]
        if got:
            return got
        time.sleep(0.25)
    return []


def pick(choices, needle, what):
    hits = [i for i in choices if needle.lower() in i["itemName"].lower() and i["itemEnabled"]]
    if not hits:
        names = "\n  ".join(i["itemName"] for i in choices)
        sys.exit(f"no {what} matching {needle!r}; OBS sees:\n  {names}")
    if len(hits) > 1:
        print(f"note: several {what}s match {needle!r}, using {hits[0]['itemName']!r}")
    return hits[0]


def scene_item(c, scene, source):
    """The scene item id of `source` in `scene`, adding it if missing."""
    try:
        return call(c, "GetSceneItemId", {"sceneName": scene, "sourceName": source})["sceneItemId"]
    except Exception:
        return call(c, "CreateSceneItem", {"sceneName": scene, "sourceName": source})["sceneItemId"]


def place(c, scene, source, x, y, w, h, align=0, fill=False):
    """Fit `source` into the box, pinned within it by `align`. With `fill`, crop
    the source to the box's shape first, so it covers the box with no bars."""
    item = scene_item(c, scene, source)
    crop = {"cropLeft": 0, "cropRight": 0, "cropTop": TITLEBAR, "cropBottom": 0}
    sw, sh = source_size(c, scene, source)
    sh -= TITLEBAR
    if fill and sw and sh > 0:
        if sw / sh > w / h:
            crop["cropLeft"] = crop["cropRight"] = int((sw - sh * w / h) / 2)
        else:
            cut = int((sh - sw * h / w) / 2)
            crop["cropTop"] += cut
            crop["cropBottom"] = cut
    call(c, "SetSceneItemTransform", {"sceneName": scene, "sceneItemId": item, "sceneItemTransform": {
        "positionX": x, "positionY": y, "alignment": LEFT | TOP, "rotation": 0,
        "boundsType": "OBS_BOUNDS_SCALE_INNER", "boundsWidth": w, "boundsHeight": h,
        "boundsAlignment": align, **crop,
    }})


def bind_window(c, scene, source, needle):
    """Point window capture `source` at the window matching `needle`: one
    titled exactly that, else the largest that captures anything. X-Plane
    also owns a 0x0 window titled "Window", and binding that first match
    recorded black (2026-10-19)."""
    # Never OBS itself: its title carries the collection name.
    hits = [i for i in items(c, source, "window")
            if needle.lower() in i["itemName"].lower() and i["itemEnabled"]
            and not i["itemName"].startswith("[OBS Studio]")]
    if not hits:
        names = "\n  ".join(i["itemName"] for i in items(c, source, "window"))
        sys.exit(f"no window matching {needle!r}; OBS sees:\n  {names}")
    exact = [i for i in hits if i["itemName"].split("] ", 1)[-1].lower() == needle.lower()]
    if exact:
        hits = exact[:1]
    best, best_area = None, 0
    for h in hits:
        call(c, "SetInputSettings", {"inputName": source, "inputSettings": {"window": h["itemValue"]}, "overlay": True})
        w = hgt = 0
        for _ in range(10):  # the capture's size settles once frames arrive
            time.sleep(0.2)
            w, hgt = source_size(c, scene, source)
            if w and hgt:
                break
        if w * hgt > best_area:
            best, best_area = h, w * hgt
    if best is None:
        sys.exit(f"windows matching {needle!r} capture nothing (closed, minimized, or no screen-recording permission?): "
                 + ", ".join(h["itemName"] for h in hits))
    call(c, "SetInputSettings", {"inputName": source, "inputSettings": {"window": best["itemValue"]}, "overlay": True})
    if len(hits) > 1:
        print(f"note: several windows match {needle!r}; using the largest, {best['itemName']!r}")
    return best


def place_xplane(c, cw, ch):
    """X-Plane full frame under the terminal inset, as the inset in the
    reverse scene, and alone."""
    place(c, MAIN, XPLANE, 0, 0, cw, ch, fill=True)
    raise_to_top(c, MAIN, TERMINAL)
    iw, ih = cw * INSET, ch * INSET
    place(c, REVERSE, XPLANE, cw - iw - MARGIN, ch - ih - MARGIN, iw, ih, align=RIGHT | BOTTOM)
    raise_to_top(c, REVERSE, XPLANE)
    place(c, COCKPIT, XPLANE, 0, 0, cw, ch, fill=True)


def raise_to_top(c, scene, source):
    item = scene_item(c, scene, source)
    count = len(call(c, "GetSceneItemList", {"sceneName": scene})["sceneItems"])
    call(c, "SetSceneItemIndex", {"sceneName": scene, "sceneItemId": item, "sceneItemIndex": count - 1})


def source_size(c, scene, source):
    t = call(c, "GetSceneItemTransform",
             {"sceneName": scene, "sceneItemId": scene_item(c, scene, source)})["sceneItemTransform"]
    return int(t["sourceWidth"]), int(t["sourceHeight"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list capturable windows and mics, change nothing")
    # A window capture follows a window id, and a restarted X-Plane has a
    # new window: the capture records black (2026-10-19, two takes). The
    # take script runs this before every take.
    ap.add_argument("--xplane-only", action="store_true",
                    help="only rebind and place the X-Plane capture (the take's preflight)")
    ap.add_argument("--xplane", default="X-Plane", help="X-Plane window name to match")
    ap.add_argument("--terminal", default="ghostty", help="terminal window name to match")
    # "Default" follows the Mac's default input, the mic push-to-talk
    # hears. Bound to a named headset, OBS recorded silence (2026-10-19)
    # the day the headset was unplugged and push-to-talk used another mic.
    ap.add_argument("--mic", default="Default", help="microphone name to match (Default: the Mac's default input)")
    ap.add_argument("--mic-gain-db", type=float, default=12.0, help="mic gain before the compressor")
    # The 2026-10-19 take measured the voice 7 dB under the pilot's with
    # peaks already at -7 dBFS: louder comes from makeup after the
    # compressor (the limiter takes the peaks), not more gain into it.
    ap.add_argument("--mic-makeup-db", type=float, default=7.0, help="compressor output gain")
    args = ap.parse_args()
    load_env()
    # The client logs every failed request; some lookups are expected to miss.
    logging.getLogger("obsws_python").setLevel(logging.CRITICAL)
    c = connect()

    if args.list:
        # Probe with throwaway inputs in the current collection's first scene.
        scene = call(c, "GetCurrentProgramScene")["currentProgramSceneName"]
        for name, kind, prop in [("_probe window", "screen_capture", "window"),
                                 ("_probe mic", "coreaudio_input_capture", "device_id")]:
            call(c, "CreateInput", {"sceneName": scene, "inputName": name, "inputKind": kind,
                                    "inputSettings": {"type": 1}, "sceneItemEnabled": False})
            try:
                print(f"{prop}:")
                for i in items(c, name, prop):
                    print(f"  {i['itemName']}")
            finally:
                call(c, "RemoveInput", {"inputName": name})
        return

    use_collection(c)
    video = call(c, "GetVideoSettings")
    cw, ch = video["baseWidth"], video["baseHeight"]

    if args.xplane_only:
        xp = bind_window(c, MAIN, XPLANE, args.xplane)
        time.sleep(1)
        place_xplane(c, cw, ch)
        print(f"OBS X-Plane capture: {xp['itemName']} {source_size(c, MAIN, XPLANE)}")
        return

    scenes = [s["sceneName"] for s in call(c, "GetSceneList")["scenes"]]
    for s in (MAIN, REVERSE, COCKPIT, TERM):
        if s not in scenes:
            call(c, "CreateScene", {"sceneName": s})

    # Sources: create in MAIN, then bind windows and devices by name.
    for name in (XPLANE, TERMINAL):
        ensure_input(c, MAIN, name, "screen_capture", {"type": 1, "show_cursor": False})
    xp = bind_window(c, MAIN, XPLANE, args.xplane)
    term = bind_window(c, MAIN, TERMINAL, args.terminal)

    ensure_input(c, MAIN, MIC, "coreaudio_input_capture", {})
    mic = pick(items(c, MIC, "device_id"), args.mic, "mic")
    call(c, "SetInputSettings", {"inputName": MIC, "inputSettings": {"device_id": mic["itemValue"]}, "overlay": True})
    ensure_input(c, MAIN, DESKTOP, "sck_audio_capture", {"type": 0})

    # Mic chain: clean, lift, even out, never clip. A headset boom mic is
    # quiet; the gain brings speech up to the radio voices' level.
    existing = [f["filterName"] for f in call(c, "GetSourceFilterList", {"sourceName": MIC})["filters"]]
    for name, kind, settings in [
        ("Noise Suppression", "noise_suppress_filter_v2", {"method": "rnnoise"}),
        ("Gain", "gain_filter", {"db": args.mic_gain_db}),
        ("Compressor", "compressor_filter", {"ratio": 4.0, "threshold": -24.0, "attack_time": 6,
                                             "release_time": 60, "output_gain": args.mic_makeup_db}),
        ("Limiter", "limiter_filter", {"threshold": -3.0, "release_time": 60}),
        # Last, on the finished level: speech here runs about -18 dBFS (50 ms
        # RMS median, EarPods, 2026-10-19), the room between calls up to
        # -35 dBFS at its 99th percentile. Opening at -30 cut the soft start
        # of about 5 calls in 12 (the "S" of "Skyhawk"), so it opens lower
        # and lets some room through: the edit (speed_ramp.py) gates the mic
        # to push-to-talk speech anyway. The hold rides through the gaps
        # between words.
        ("Noise Gate", "noise_gate_filter", {"open_threshold": -38.0, "close_threshold": -45.0,
                                             "attack_time": 5, "hold_time": 250, "release_time": 150}),
    ]:
        if name in existing:
            call(c, "SetSourceFilterSettings", {"sourceName": MIC, "filterName": name, "filterSettings": settings})
        else:
            call(c, "CreateSourceFilter", {"sourceName": MIC, "filterName": name,
                                           "filterKind": kind, "filterSettings": settings})

    # Layouts. The window sources' sizes only settle once they're capturing.
    time.sleep(1)
    tw, th = cw * TERMINAL_INSET, ch * TERMINAL_INSET
    place(c, MAIN, TERMINAL, cw - tw - MARGIN, ch - th - MARGIN, tw, th, align=RIGHT | BOTTOM)
    place(c, REVERSE, TERMINAL, 0, 0, cw, ch)
    place_xplane(c, cw, ch)
    place(c, TERM, TERMINAL, 0, 0, cw, ch)
    for s in (MAIN, REVERSE, COCKPIT, TERM):
        for a in (MIC, DESKTOP):
            scene_item(c, s, a)

    # Tracks: 1 the mix, 2 the mic, 3 desktop audio; record all three.
    # Window captures carry their app's audio too: silence them, or X-Plane
    # lands on every track (twice in the mix).
    off = {str(n): False for n in range(1, 7)}
    for name in (XPLANE, TERMINAL):
        call(c, "SetInputAudioTracks", {"inputName": name, "inputAudioTracks": off})
        call(c, "SetInputMute", {"inputName": name, "inputMuted": True})
    call(c, "SetInputAudioTracks", {"inputName": MIC, "inputAudioTracks": {**off, "1": True, "2": True}})
    call(c, "SetInputAudioTracks", {"inputName": DESKTOP, "inputAudioTracks": {**off, "1": True, "3": True}})
    mode = call(c, "GetProfileParameter", {"parameterCategory": "Output", "parameterName": "Mode"})
    if mode.get("parameterValue") == "Advanced":
        call(c, "SetProfileParameter", {"parameterCategory": "AdvOut", "parameterName": "RecTracks",
                                        "parameterValue": "7"})
    else:
        print("note: output mode is Simple, so the recording has the mix only")

    for s in scenes:
        if s not in (MAIN, REVERSE, COCKPIT, TERM) and s == "Scene":
            call(c, "SetCurrentProgramScene", {"sceneName": MAIN})
            call(c, "RemoveScene", {"sceneName": s})
    call(c, "SetCurrentProgramScene", {"sceneName": MAIN})

    time.sleep(0.5)
    rec = call(c, "GetRecordDirectory")["recordDirectory"]
    print(f"collection {COLLECTION!r}, canvas {cw}x{ch}, recording to {rec}")
    print(f"  {XPLANE}: {xp['itemName']} {source_size(c, COCKPIT, XPLANE)}")
    print(f"  {TERMINAL}: {term['itemName']} {source_size(c, TERM, TERMINAL)}")
    print(f"  {MIC}: {mic['itemName']}")
    print(f"  scenes: {MAIN}, {REVERSE}, {COCKPIT}, {TERM}")


if __name__ == "__main__":
    main()
