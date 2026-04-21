#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "requests",
#     "pyyaml",
# ]
# ///
"""Scenario-agnostic evaluation harness for sim_pilot + live X-Plane 12.

This harness *runs* a scenario and captures evidence. It does NOT judge
success or failure — that's the operator's (or model's) job after the
run. The harness only decides when to *stop recording*.

Given a scenario YAML, the script:

  1. Applies the scenario setup via the X-Plane web API
     (POST /api/v3/flight with `flight_init`, plus optional writable
     `datarefs` and pause/unpause around them).

  2. Starts a 1 Hz state-trace CSV that records lat/lon/alt/ias/gs/phase/
     on_ground/has_crashed every second.

  3. Launches sim_pilot --headless with the scenario's pilot args,
     streaming its stdout/stderr into a run log.

  4. Polls X-Plane every second until a termination condition fires:
       - has_crashed edge (if terminate_on_crash)
       - on-ground + groundspeed below threshold for N seconds
         (if terminate_on_stopped)
       - wall-clock timeout_s
       - sim_pilot subprocess exited on its own
       - external SIGTERM / Ctrl-C

  5. Kills sim_pilot, pauses X-Plane, and writes a neutral report JSON:
     termination reason, final state, artifact paths. No pass/fail.

Artifacts written under <out_dir>/<run_id>/:
  - scenario.yaml    (copy of the input)
  - setup.json       (the exact flight_init body + dataref writes)
  - sim_pilot.log    (sim_pilot stdout/stderr)
  - state_trace.csv  (1 Hz ground-truth from X-Plane)
  - report.json      (termination facts)

Exit codes:
  0 - run completed (any termination reason other than harness error)
  2 - harness error (couldn't talk to X-Plane, setup failed, etc.)

Usage:
  uv run scripts/eval/run_scenario.py scripts/eval/scenarios/deadstick_kwhp_12.yaml
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import requests
import yaml

# ----- X-Plane web API client --------------------------------------------

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8086

EARTH_RADIUS_FT = 20_925_524.9

# Hard safety cap on any single run, regardless of scenario timeout.
MAX_RUN_SECONDS = 300.0


class XPlane:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._id_cache: dict[str, int] = {}

    @property
    def base(self) -> str:
        return f"http://{self.host}:{self.port}/api/v3"

    def resolve_id(self, name: str) -> int:
        cached = self._id_cache.get(name)
        if cached is not None:
            return cached
        r = requests.get(
            f"{self.base}/datarefs",
            params={"filter[name]": name},
            timeout=self.timeout,
        )
        r.raise_for_status()
        data = r.json().get("data") or []
        if not data:
            raise RuntimeError(f"dataref not found: {name}")
        did = int(data[0]["id"])
        self._id_cache[name] = did
        return did

    def read(self, name: str) -> float:
        did = self.resolve_id(name)
        r = requests.get(f"{self.base}/datarefs/{did}/value", timeout=self.timeout)
        r.raise_for_status()
        val = r.json().get("data")
        if isinstance(val, list):
            return float(val[0]) if val else 0.0
        if isinstance(val, bool):
            return 1.0 if val else 0.0
        return float(val or 0.0)

    def write(self, name: str, value: float) -> None:
        did = self.resolve_id(name)
        r = requests.patch(
            f"{self.base}/datarefs/{did}/value",
            json={"data": value},
            timeout=self.timeout,
        )
        if r.status_code >= 400:
            raise RuntimeError(f"write {name} -> {value} failed: {r.status_code} {r.text}")

    def activate_command(self, name: str, duration: float = 0.0) -> None:
        r = requests.get(
            f"{self.base}/commands",
            params={"filter[name]": name},
            timeout=self.timeout,
        )
        r.raise_for_status()
        data = r.json().get("data") or []
        if not data:
            raise RuntimeError(f"command not found: {name}")
        cid = int(data[0]["id"])
        r = requests.post(
            f"{self.base}/command/{cid}/activate",
            json={"duration": duration},
            timeout=self.timeout,
        )
        if r.status_code >= 400:
            raise RuntimeError(f"command {name} failed: {r.status_code} {r.text}")

    def init_flight(self, data: dict[str, Any]) -> None:
        r = requests.post(
            f"{self.base}/flight",
            json={"data": data},
            timeout=self.timeout,
        )
        if r.status_code >= 400:
            raise RuntimeError(f"flight init failed: {r.status_code} {r.text}")

    def wait_for_flight_loaded(self, timeout_s: float = 60.0) -> None:
        """Poll until the flightmodel datarefs exist (world has loaded)."""
        deadline = time.monotonic() + timeout_s
        name = "sim/flightmodel/position/latitude"
        while time.monotonic() < deadline:
            try:
                self._id_cache.pop(name, None)
                self.resolve_id(name)
                return
            except Exception:
                time.sleep(1.0)
        raise RuntimeError(f"flight did not load within {timeout_s}s")


# ----- scenario ----------------------------------------------------------


@dataclasses.dataclass
class Scenario:
    name: str
    description: str
    goal: str
    setup: dict[str, Any]
    pilot: dict[str, Any]
    termination: dict[str, Any]

    @classmethod
    def load(cls, path: Path) -> "Scenario":
        with path.open() as f:
            raw = yaml.safe_load(f)
        return cls(
            name=raw.get("name", path.stem),
            description=(raw.get("description") or "").strip(),
            goal=(raw.get("goal") or "").strip(),
            setup=raw.get("setup") or {},
            pilot=raw.get("pilot") or {},
            termination=raw.get("termination") or {},
        )


# ----- setup -------------------------------------------------------------


def apply_setup(xp: XPlane, setup: dict[str, Any]) -> None:
    flight_init = setup.get("flight_init")
    if flight_init:
        print("[eval] POST /api/v3/flight (starting fresh flight)", flush=True)
        xp.init_flight(flight_init)
        settle_s = float(setup.get("settle_s", 6.0))
        print(f"[eval] waiting {settle_s}s for world load", flush=True)
        xp.wait_for_flight_loaded(timeout_s=max(30.0, settle_s + 10.0))
        time.sleep(settle_s)

    if setup.get("pause", True):
        try:
            print("[eval] sim/operation/pause_on", flush=True)
            xp.activate_command("sim/operation/pause_on")
        except Exception as e:
            print(f"[eval] WARN pause failed: {e}", flush=True)

    writes = setup.get("datarefs") or {}
    for name, value in writes.items():
        print(f"[eval] write {name} = {value}", flush=True)
        try:
            xp.write(name, float(value))
        except Exception as e:
            print(f"[eval] WARN write {name} failed: {e}", flush=True)

    # Optional X-Plane commands (e.g. `sim/starters/engage_starters`).
    # Each entry is either a string name (fired for 0.0 s) or a map
    # {name: str, duration: float}.
    for cmd in setup.get("commands") or []:
        if isinstance(cmd, str):
            name, duration = cmd, 0.0
        else:
            name = cmd["name"]
            duration = float(cmd.get("duration", 0.0))
        print(f"[eval] command {name} (duration={duration}s)", flush=True)
        try:
            xp.activate_command(name, duration=duration)
        except Exception as e:
            print(f"[eval] WARN command {name} failed: {e}", flush=True)

    time.sleep(0.25)
    # Always unpause on exit from setup. X-Plane has been observed to
    # auto-pause after a flight_init POST even when we never asked for a
    # pause, so this runs unconditionally rather than only in the
    # `setup.pause==True` branch.
    try:
        print("[eval] sim/operation/pause_off", flush=True)
        xp.activate_command("sim/operation/pause_off")
    except Exception as e:
        print(f"[eval] WARN unpause failed: {e}", flush=True)


# ----- state trace (1 Hz CSV) --------------------------------------------

STATE_DATAREFS = [
    "sim/flightmodel/position/latitude",
    "sim/flightmodel/position/longitude",
    "sim/flightmodel/position/elevation",
    "sim/flightmodel/position/y_agl",
    "sim/flightmodel/position/psi",
    "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
    "sim/flightmodel/position/local_vx",
    "sim/flightmodel/position/local_vz",
    "sim/flightmodel/position/vh_ind_fpm",
    "sim/flightmodel2/gear/on_ground",
    "sim/flightmodel2/misc/has_crashed",
    "sim/cockpit2/engine/indicators/engine_speed_rpm",
    "sim/cockpit2/controls/flap_handle_deploy_ratio",
    "sim/cockpit2/engine/actuators/throttle_ratio_all",
]


def trace_sample(xp: XPlane) -> dict[str, float]:
    out: dict[str, float] = {}
    for name in STATE_DATAREFS:
        try:
            out[name] = xp.read(name)
        except Exception:
            out[name] = float("nan")
    vx = out.get("sim/flightmodel/position/local_vx", 0.0)
    vz = out.get("sim/flightmodel/position/local_vz", 0.0)
    out["gs_kt"] = math.hypot(vx, vz) * 1.943844
    out["alt_agl_ft"] = out.get("sim/flightmodel/position/y_agl", 0.0) * 3.28084
    out["alt_msl_ft"] = out.get("sim/flightmodel/position/elevation", 0.0) * 3.28084
    return out


def trace_writer_thread(
    xp: XPlane,
    csv_path: Path,
    stop_event: threading.Event,
    samples: list[dict[str, float]],
) -> None:
    """Write one sample/sec to CSV. Appends to `samples` too for the report."""
    headers = [
        "t_wall_s",
        "lat",
        "lon",
        "alt_msl_ft",
        "alt_agl_ft",
        "heading_deg",
        "ias_kt",
        "gs_kt",
        "vs_fpm",
        "on_ground",
        "has_crashed",
        "engine_rpm",
        "flap_ratio",
        "throttle",
    ]
    t0 = time.monotonic()
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(headers)
        while not stop_event.is_set():
            s = trace_sample(xp)
            row = {
                "t_wall_s": round(time.monotonic() - t0, 2),
                "lat": s.get("sim/flightmodel/position/latitude"),
                "lon": s.get("sim/flightmodel/position/longitude"),
                "alt_msl_ft": round(s["alt_msl_ft"], 1),
                "alt_agl_ft": round(s["alt_agl_ft"], 1),
                "heading_deg": round(s.get("sim/flightmodel/position/psi", 0.0), 1),
                "ias_kt": round(s.get("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", 0.0), 1),
                "gs_kt": round(s["gs_kt"], 1),
                "vs_fpm": round(s.get("sim/flightmodel/position/vh_ind_fpm", 0.0), 0),
                "on_ground": int(s.get("sim/flightmodel2/gear/on_ground", 0.0) >= 0.5),
                "has_crashed": int(s.get("sim/flightmodel2/misc/has_crashed", 0.0) >= 0.5),
                "engine_rpm": round(s.get("sim/cockpit2/engine/indicators/engine_speed_rpm", 0.0), 0),
                "flap_ratio": round(s.get("sim/cockpit2/controls/flap_handle_deploy_ratio", 0.0), 2),
                "throttle": round(s.get("sim/cockpit2/engine/actuators/throttle_ratio_all", 0.0), 2),
            }
            samples.append(row)
            w.writerow([row[h] for h in headers])
            f.flush()
            stop_event.wait(1.0)


# ----- pilot subprocess --------------------------------------------------


def spawn_pilot(
    repo: Path,
    scenario: Scenario,
    log_path: Path,
    extra_env: dict[str, str],
) -> subprocess.Popen:
    bin_path = repo / "target" / "release" / "sim-pilot"
    if bin_path.exists():
        args: list[str] = [str(bin_path)]
    else:
        args = ["cargo", "run", "--release", "--"]
    args += ["--headless", "--no-voice"]
    engage = scenario.pilot.get("engage_profile")
    if engage:
        args += ["--engage-profile", engage]
    for msg in scenario.pilot.get("atc_messages") or []:
        args += ["--atc-message", msg]
    args += list(scenario.pilot.get("extra_args") or [])

    env = os.environ.copy()
    env.update(extra_env)

    print(f"[eval] launching: {' '.join(args)}", flush=True)
    log_f = log_path.open("w")
    return subprocess.Popen(
        args,
        cwd=repo,
        env=env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def terminate_pilot(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)


# ----- monitor loop ------------------------------------------------------


def monitor(
    xp: XPlane,
    termination: dict[str, Any],
    proc: subprocess.Popen,
    samples: list[dict[str, float]],
) -> dict[str, Any]:
    """Return a neutral termination record. Never judges success/failure."""
    scenario_timeout_s = float(termination.get("timeout_s", MAX_RUN_SECONDS))
    timeout_s = min(scenario_timeout_s, MAX_RUN_SECONDS)
    if timeout_s < scenario_timeout_s:
        print(
            f"[eval] capping run at {MAX_RUN_SECONDS:.0f}s (scenario asked for {scenario_timeout_s:.0f}s)",
            flush=True,
        )
    stop_kt = float(termination.get("stopped_ground_speed_kt", 5.0))
    stop_hold_s = float(termination.get("stopped_hold_s", 5.0))
    term_on_stopped = bool(termination.get("terminate_on_stopped", True))
    term_on_crash = bool(termination.get("terminate_on_crash", True))

    t0 = time.monotonic()
    stopped_since: float | None = None
    crash_seen = False
    moved_at_least_once = False

    while True:
        elapsed = time.monotonic() - t0

        if proc.poll() is not None:
            return _record("sim_pilot_exited", elapsed, samples, extra={"exit_code": proc.returncode})

        if elapsed > timeout_s:
            return _record("timeout", elapsed, samples)

        # Use the last trace sample rather than hit the API a second time.
        last = samples[-1] if samples else None
        if last is None:
            time.sleep(0.5)
            continue

        if last.get("has_crashed") and not crash_seen:
            crash_seen = True
            if term_on_crash:
                return _record("has_crashed", elapsed, samples)

        # Track whether the aircraft has ever moved — "stopped on ground"
        # is only a meaningful terminal condition if we went somewhere.
        # Without this, scenarios that spawn stationary (lle_ground_start)
        # terminate immediately.
        if (last.get("gs_kt") or 0.0) >= stop_kt:
            moved_at_least_once = True

        if (
            term_on_stopped
            and moved_at_least_once
            and last.get("on_ground")
            and (last.get("gs_kt") or 0.0) < stop_kt
        ):
            if stopped_since is None:
                stopped_since = elapsed
            elif elapsed - stopped_since >= stop_hold_s:
                return _record("stopped_on_ground", elapsed, samples)
        else:
            stopped_since = None

        time.sleep(1.0)


def _record(
    reason: str,
    elapsed_s: float,
    samples: list[dict[str, float]],
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    last = samples[-1] if samples else {}
    rec = {
        "termination_reason": reason,
        "elapsed_s": round(elapsed_s, 1),
        "final_state": last,
    }
    if extra:
        rec.update(extra)
    return rec


# ----- main --------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", type=Path, help="Path to scenario YAML")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument(
        "--repo",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="Repo root (where sim-pilot is invoked).",
    )
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Where to write run artifacts. Default: <repo>/output/eval/<run_id>.",
    )
    ap.add_argument(
        "--setup-only",
        action="store_true",
        help="Apply the scenario setup to X-Plane, then exit (don't launch sim_pilot).",
    )
    args = ap.parse_args()

    scenario = Scenario.load(args.scenario)
    xp = XPlane(host=args.host, port=args.port)

    run_id = f"{scenario.name}-{time.strftime('%Y%m%d-%H%M%S')}"
    out_dir = args.out_dir or (args.repo / "output" / "eval" / run_id)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[eval] scenario: {scenario.name}", flush=True)
    print(f"[eval] run_id:   {run_id}", flush=True)
    print(f"[eval] out_dir:  {out_dir}", flush=True)
    if scenario.goal:
        print(f"[eval] goal:     {scenario.goal}", flush=True)

    # Archive the scenario + resolved setup so the run is self-describing.
    shutil.copy(args.scenario, out_dir / "scenario.yaml")
    (out_dir / "setup.json").write_text(json.dumps(scenario.setup, indent=2))

    try:
        apply_setup(xp, scenario.setup)
    except Exception as e:
        print(f"[eval] ERROR applying setup: {e}", file=sys.stderr)
        return 2

    if args.setup_only:
        print("[eval] --setup-only: not launching sim_pilot", flush=True)
        return 0

    log_path = out_dir / "sim_pilot.log"
    csv_path = out_dir / "state_trace.csv"
    report_path = out_dir / "report.json"

    stop_trace = threading.Event()
    samples: list[dict[str, float]] = []
    trace_thread = threading.Thread(
        target=trace_writer_thread,
        args=(xp, csv_path, stop_trace, samples),
        daemon=True,
    )
    trace_thread.start()

    proc = spawn_pilot(args.repo, scenario, log_path, extra_env={})
    try:
        record = monitor(xp, scenario.termination, proc, samples)
    except KeyboardInterrupt:
        record = _record("sigint", time.monotonic(), samples)
    finally:
        terminate_pilot(proc)
        stop_trace.set()
        trace_thread.join(timeout=5)
        try:
            xp.activate_command("sim/operation/pause_on")
        except Exception:
            pass

    report = {
        "scenario": scenario.name,
        "run_id": run_id,
        "goal": scenario.goal,
        "description": scenario.description,
        "artifacts": {
            "scenario_yaml": str((out_dir / "scenario.yaml").resolve()),
            "setup_json": str((out_dir / "setup.json").resolve()),
            "sim_pilot_log": str(log_path.resolve()),
            "state_trace_csv": str(csv_path.resolve()),
        },
        **record,
    }
    report_path.write_text(json.dumps(report, indent=2))

    print("[eval] REPORT:", flush=True)
    print(json.dumps(report, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
