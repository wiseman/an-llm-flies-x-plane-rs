#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Measure how flap deflection changes engine-off glide performance in
X-Plane's flight model. Used to calibrate the simple-sim drag table.

For each flap setting (0/10/20/30°), spawns the C172 airborne at 8000
ft MSL over flat ground, kills the engine (mixture ratio = 0), holds
pitch via PitchHold autopilot to track ~vbg IAS, and samples sink
rate / IAS / glide ratio for 30 seconds. Prints a table.

Run with X-Plane 12 already running:
    uv run scripts/measure_flap_drag.py
"""

import json
import statistics
import sys
import time
import urllib.parse
import urllib.request

BASE = "http://127.0.0.1:8086/api/v3"


def lookup(name):
    q = urllib.parse.urlencode({"filter[name]": name})
    r = json.loads(urllib.request.urlopen(f"{BASE}/datarefs?{q}").read())
    return r["data"][0]["id"] if r.get("data") else None


def read_id(i):
    return json.loads(urllib.request.urlopen(f"{BASE}/datarefs/{i}/value").read())["data"]


def write_id(i, value):
    urllib.request.urlopen(
        urllib.request.Request(
            f"{BASE}/datarefs/{i}/value",
            method="PATCH",
            data=json.dumps({"data": value}).encode(),
            headers={"Content-Type": "application/json"},
        )
    )


def cmd(name):
    q = urllib.parse.urlencode({"filter[name]": name})
    cid = json.loads(urllib.request.urlopen(f"{BASE}/commands?{q}").read())["data"][0]["id"]
    urllib.request.urlopen(
        urllib.request.Request(
            f"{BASE}/command/{cid}/activate",
            method="POST",
            data=json.dumps({"duration": 0}).encode(),
            headers={"Content-Type": "application/json"},
        )
    )


def spawn(lat, lon, elev_m, heading, mps):
    body = {
        "data": {
            "aircraft": {
                "path": "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP.acf"
            },
            "lle_air_start": {
                "latitude": lat,
                "longitude": lon,
                "elevation_in_meters": elev_m,
                "heading_true": heading,
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


def measure_glide(flap_setting_deg, settle_s=18.0, sample_s=20.0, sample_dt=0.5):
    """Spawn, kill engine, set flap, hold target IAS via the X-Plane
    pitch-hold autopilot, and return averaged (ias_kt, sink_fpm, gs_kt,
    glide_ratio) over the sample window."""
    print(f"\n--- flap_setting_deg = {flap_setting_deg} ---")

    # Spawn over ocean (flat terrain) at 8000 ft MSL = 2438 m, heading west,
    # at vbg target = 68 KIAS = 35 m/s.
    spawn(lat=34.0, lon=-119.5, elev_m=2438.0, heading=270.0, mps=35.0)
    time.sleep(6)

    # Resolve datarefs once.
    ids = {n: lookup(n) for n in [
        "sim/flightmodel/position/y_agl",
        "sim/flightmodel/position/elevation",
        "sim/flightmodel/position/indicated_airspeed",
        "sim/flightmodel/position/groundspeed",
        "sim/flightmodel/position/vh_ind_fpm",
        "sim/flightmodel/position/true_theta",
        "sim/cockpit2/controls/flap_handle_request_ratio",
        "sim/cockpit2/engine/actuators/mixture_ratio_all",
        "sim/cockpit2/engine/actuators/throttle_ratio_all",
        "sim/cockpit/autopilot/autopilot_state",
        "sim/cockpit/autopilot/airspeed",
        "sim/cockpit2/autopilot/altitude_dial_ft",
    ]}

    # Kill the engine via mixture cutoff. Throttle to 0 belt-and-suspenders.
    write_id(ids["sim/cockpit2/engine/actuators/mixture_ratio_all"], 0.0)
    write_id(ids["sim/cockpit2/engine/actuators/throttle_ratio_all"], 0.0)

    # Set the requested flap deflection.
    write_id(
        ids["sim/cockpit2/controls/flap_handle_request_ratio"],
        flap_setting_deg / 30.0,
    )

    # Engage the X-Plane autopilot in FLCH-style "fly-to-IAS" mode so
    # pitch tracks vbg automatically. autopilot_state bit 6 (= 64) is
    # "FLCH/IAS hold" on the C172.
    write_id(ids["sim/cockpit/autopilot/airspeed"], 68.0)
    write_id(ids["sim/cockpit/autopilot/autopilot_state"], 64.0)
    # And aim altitude wildly low so the AP keeps pitching for IAS rather
    # than capturing altitude.
    write_id(ids["sim/cockpit2/autopilot/altitude_dial_ft"], 0.0)

    # Settle.
    print(f"settling for {settle_s:.0f}s …")
    time.sleep(settle_s)

    # Sample.
    samples_ias = []
    samples_vs = []
    samples_gs = []
    samples_pitch = []
    end_t = time.time() + sample_s
    while time.time() < end_t:
        samples_ias.append(read_id(ids["sim/flightmodel/position/indicated_airspeed"]))
        samples_vs.append(read_id(ids["sim/flightmodel/position/vh_ind_fpm"]))
        samples_gs.append(read_id(ids["sim/flightmodel/position/groundspeed"]) * 1.94384)
        samples_pitch.append(read_id(ids["sim/flightmodel/position/true_theta"]))
        time.sleep(sample_dt)

    ias = statistics.mean(samples_ias)
    vs = statistics.mean(samples_vs)
    gs = statistics.mean(samples_gs)
    pitch = statistics.mean(samples_pitch)
    sink_fps = -vs / 60.0
    fwd_fps = gs * 1.6878
    glide_ratio = (fwd_fps / sink_fps) if sink_fps > 0.5 else float("inf")
    print(
        f"flap={flap_setting_deg:3d}°  IAS={ias:5.1f} kt  GS={gs:5.1f} kt  "
        f"VS={vs:7.0f} fpm  pitch={pitch:5.1f}°  L/D≈{glide_ratio:5.1f}"
    )
    return {
        "flap_deg": flap_setting_deg,
        "ias_kt": ias,
        "gs_kt": gs,
        "vs_fpm": vs,
        "pitch_deg": pitch,
        "glide_ratio": glide_ratio,
    }


def main():
    try:
        urllib.request.urlopen(f"http://127.0.0.1:8086/api/capabilities", timeout=2).read()
    except Exception as e:
        print(f"X-Plane not reachable on 8086: {e}", file=sys.stderr)
        return 1

    rows = []
    for flap in [0, 10, 20, 30]:
        rows.append(measure_glide(flap))

    # Pause when done.
    try:
        cmd("sim/operation/pause_on")
    except Exception:
        pass

    print("\n=== summary ===")
    print(f"{'flap°':>5s} {'IAS_kt':>7s} {'GS_kt':>6s} {'VS_fpm':>7s} {'pitch°':>7s} {'L/D':>5s}")
    for r in rows:
        print(
            f"{r['flap_deg']:5d} {r['ias_kt']:7.1f} {r['gs_kt']:6.1f} "
            f"{r['vs_fpm']:7.0f} {r['pitch_deg']:7.1f} {r['glide_ratio']:5.1f}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
