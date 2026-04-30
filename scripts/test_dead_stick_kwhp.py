#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""One-shot live test of the dead-stick controller against real KWHP rwy 30.

Spawns the aircraft NE of KWHP with the engine running, then launches the
existing `sim-pilot` binary in headless mode with an ATC message that
prods the LLM to call `engage_dead_stick_landing(KWHP, 30)`. Polls X-Plane
state every 3 s and prints a trajectory table (heading / IAS / AGL / VS).

Run from the repo root:
    uv run scripts/test_dead_stick_kwhp.py
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:8086/api/v3"


def lookup(name: str) -> int | None:
    q = urllib.parse.urlencode({"filter[name]": name})
    r = json.loads(urllib.request.urlopen(f"{BASE}/datarefs?{q}").read())
    return r["data"][0]["id"] if r.get("data") else None


def cmd(name: str, duration: float = 0.0) -> None:
    q = urllib.parse.urlencode({"filter[name]": name})
    cid = json.loads(urllib.request.urlopen(f"{BASE}/commands?{q}").read())["data"][0][
        "id"
    ]
    urllib.request.urlopen(
        urllib.request.Request(
            f"{BASE}/command/{cid}/activate",
            method="POST",
            data=json.dumps({"duration": duration}).encode(),
            headers={"Content-Type": "application/json"},
        )
    )


def spawn_airborne(lat: float, lon: float, elev_m: float, hdg: float, mps: float) -> None:
    body = {
        "data": {
            "aircraft": {
                "path": "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf"
            },
            "lle_air_start": {
                "latitude": lat,
                "longitude": lon,
                "elevation_in_meters": elev_m,
                "heading_true": hdg,
                "speed_in_meters_per_second": mps,
                "pitch_in_degrees": 0.0,
            },
            "engine_status": {"all_engines": {"running": True}},
        }
    }
    urllib.request.urlopen(
        urllib.request.Request(
            f"{BASE}/flight",
            method="POST",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
        )
    )


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    binary = repo_root / "target/release/sim-pilot"
    if not binary.exists():
        print(f"missing {binary}; run `cargo build --release` first", file=sys.stderr)
        return 1

    # Sanity check: X-Plane is up.
    try:
        urllib.request.urlopen(f"http://127.0.0.1:8086/api/capabilities", timeout=2).read()
    except Exception as e:
        print(f"X-Plane web API unreachable on 8086: {e}", file=sys.stderr)
        return 1

    print("[setup] spawning aircraft 1.5 NM NE of KWHP at 4000 MSL …")
    # 1.5 NM NE of KWHP threshold (KWHP rwy 30 thr ~ 34.255N, -118.409W).
    spawn_airborne(
        lat=34.2750,
        lon=-118.3850,
        elev_m=1219.2,  # 4000 ft MSL ≈ 3000 ft above KWHP field elev (1000 ft)
        hdg=350.0,
        mps=38.0,  # ~74 kt
    )
    time.sleep(6)
    cmd("sim/operation/pause_on")

    pos_ids = {n: lookup(n) for n in [
        "sim/flightmodel/position/elevation",
        "sim/flightmodel/position/y_agl",
        "sim/flightmodel/position/psi",
        "sim/flightmodel/position/indicated_airspeed",
        "sim/flightmodel/position/groundspeed",
        "sim/flightmodel/position/vh_ind_fpm",
        "sim/flightmodel/position/true_phi",
        "sim/flightmodel/position/true_theta",
        "sim/cockpit2/engine/actuators/throttle_ratio_all",
        "sim/flightmodel2/misc/has_crashed",
        "sim/flightmodel/position/latitude",
        "sim/flightmodel/position/longitude",
    ]}

    def read(name: str) -> float:
        i = pos_ids[name]
        return json.loads(urllib.request.urlopen(f"{BASE}/datarefs/{i}/value").read())["data"]

    # ATC prompt that should make the LLM engage_dead_stick_landing(KWHP, 30).
    atc = (
        "[ATC] Cessna, this is a simulated emergency. Your engine has "
        "failed. Use find_dead_stick_candidates and then "
        "engage_dead_stick_landing to glide to KWHP runway 30, left "
        "traffic. Do not re-engage any other profile."
    )

    log_path = repo_root / "output" / f"deadstick-kwhp-{int(time.time())}.log"
    log_path.parent.mkdir(exist_ok=True)
    print(f"[setup] launching sim-pilot --headless, log={log_path}")
    pilot = subprocess.Popen(
        [
            str(binary),
            "--backend", "xplane",
            "--headless",
            "--no-voice",
            "--atc-message", atc,
            "--log-file", str(log_path),
        ],
        cwd=str(repo_root),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    # Give the pilot a moment to bootstrap, then unpause.
    time.sleep(4)
    cmd("sim/operation/pause_off")
    print("[run] unpaused; polling X-Plane every 3 s")
    print()
    print(f"{'t':>3s} {'MSL':>5s} {'AGL':>5s} {'IAS':>4s} {'GS':>4s} {'VS':>6s} "
          f"{'hdg':>5s} {'pitch':>5s} {'roll':>5s} {'thr':>4s} {'lat':>9s} {'lon':>10s} crashed")

    try:
        for i in range(80):  # up to 4 minutes
            t = i * 3
            crashed = read("sim/flightmodel2/misc/has_crashed")
            print(
                f"{t:3d} "
                f"{read('sim/flightmodel/position/elevation') * 3.28084:5.0f} "
                f"{read('sim/flightmodel/position/y_agl'):5.0f} "
                f"{read('sim/flightmodel/position/indicated_airspeed'):4.0f} "
                f"{read('sim/flightmodel/position/groundspeed') * 1.94384:4.0f} "
                f"{read('sim/flightmodel/position/vh_ind_fpm'):6.0f} "
                f"{(read('sim/flightmodel/position/psi') % 360):5.0f} "
                f"{read('sim/flightmodel/position/true_theta'):5.1f} "
                f"{read('sim/flightmodel/position/true_phi'):5.1f} "
                f"{read('sim/cockpit2/engine/actuators/throttle_ratio_all'):4.2f} "
                f"{read('sim/flightmodel/position/latitude'):9.5f} "
                f"{read('sim/flightmodel/position/longitude'):10.5f} "
                f"{crashed:.0f}"
            )
            if crashed > 0.5:
                print("[run] aircraft crashed")
                break
            if read("sim/flightmodel/position/y_agl") < 5:
                print("[run] aircraft on the ground")
                break
            time.sleep(3)
    finally:
        print("[teardown] stopping sim-pilot, pausing X-Plane")
        pilot.send_signal(signal.SIGTERM)
        try:
            pilot.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pilot.kill()
        try:
            cmd("sim/operation/pause_on")
        except Exception:
            pass
        print(f"[done] full pilot transcript: {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
