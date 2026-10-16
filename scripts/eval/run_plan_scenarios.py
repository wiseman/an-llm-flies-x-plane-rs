# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Run the plan-vocabulary scenarios (evals/plan/*.yaml) in the simple sim
and print a pass/fail table.

    uv run scripts/eval/run_plan_scenarios.py                 # all, 4 at a time
    uv run scripts/eval/run_plan_scenarios.py B1 A1 -j 2      # by id prefix

Each run's log, transcript, track and ground-track SVG land in
output/eval-<stamp>/. Builds the eval binary first (release).
"""

import argparse
import json
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def run(path: Path, args) -> dict:
    cmd = [str(ROOT / "target/release/eval"), "--scenario", str(path),
           "--pilot-llm-provider", args.provider]
    if args.model:
        cmd += ["--pilot-llm-model", args.model]
    try:
        proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=args.timeout)
    except subprocess.TimeoutExpired:
        return {"scenario": path.stem, "checks": [("FAIL", "finished", f"no result after {args.timeout}s wall")],
                "turns": None, "out_tokens": None, "svg": None, "outcome": "timed out"}
    out = proc.stdout + proc.stderr
    checks = re.findall(r"^(PASS|FAIL) (.*?)\s{2,}(.*)$", out, re.M)
    summary = re.search(r"^summary:\s+(\S+\.json)$", out, re.M)
    data = {}
    if summary and (ROOT / summary.group(1)).exists():
        data = json.loads((ROOT / summary.group(1)).read_text())
    return {
        "scenario": path.stem,
        "checks": checks,
        "turns": data.get("turns"),
        "out_tokens": data.get("total_output_tokens"),
        "svg": next(iter(re.findall(r"^ground_svg:\s+(\S+)$", out, re.M)), None),
        "outcome": data.get("outcome", {}).get("summary"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("ids", nargs="*", help="scenario id prefixes (default: all)")
    ap.add_argument("-j", "--jobs", type=int, default=4)
    ap.add_argument("--provider", default="claude-code")
    ap.add_argument("--model", default="claude-sonnet-5-5")
    ap.add_argument("--timeout", type=int, default=1800)
    args = ap.parse_args()

    subprocess.run(["cargo", "build", "--release", "--bin", "eval"], cwd=ROOT, check=True)
    paths = sorted((ROOT / "evals/plan").glob("*.yaml"))
    if args.ids:
        paths = [p for p in paths if any(p.stem.startswith(i) for i in args.ids)]
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(lambda p: run(p, args), paths))

    failed = 0
    for r in results:
        ok = r["checks"] and all(c[0] == "PASS" for c in r["checks"])
        failed += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {r['scenario']}  turns={r['turns']} out_tokens={r['out_tokens']}  {r['outcome']}")
        for status, check, detail in r["checks"]:
            print(f"        {status} {check}: {detail}")
        if r["svg"]:
            print(f"        {r['svg']}")
    print(f"\n{len(results) - failed}/{len(results)} scenarios passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
