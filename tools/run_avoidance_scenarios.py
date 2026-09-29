"""Run generated scenario worlds headlessly, one Webots launch per scenario.

    python tools/run_avoidance_scenarios.py --world regression
    python tools/run_avoidance_scenarios.py --world stress --planner v2 --jobs 3
    python tools/run_avoidance_scenarios.py --world stress --cases u_trap,crossing

Needs the `webots` executable (WEBOTS_HOME or PATH).  On Linux without a
display run it through xvfb-run.  Exit status is 0 only if every case passed.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.avoidance_scenarios import WORLDS


def _webots():
    home = os.environ.get("WEBOTS_HOME")
    for candidate in (
        Path(home) / "msys64/mingw64/bin/webots.exe" if home else None,
        Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"),
    ):
        if candidate is not None and candidate.exists():
            return str(candidate)
    return "webots"


def run_case(world, case_name, planner, port, timeout):
    environment = os.environ.copy()
    environment["REGRESSION_CASE"] = case_name
    environment["AVOIDANCE_PLANNER"] = planner
    environment["REGRESSION_TIMEOUT"] = str(timeout)
    environment["PATH"] = str(Path(sys.executable).parent) + os.pathsep + environment.get("PATH", "")
    report = Path(tempfile.gettempdir(), f"techweek-regression-{case_name}.json")
    report.unlink(missing_ok=True)
    log = Path(tempfile.gettempdir(), f"techweek-regression-{planner}-{case_name}.log")
    with log.open("w", encoding="utf-8") as stream:
        process = subprocess.run(
            [_webots(), "--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr",
             f"--port={port}", str(ROOT / "worlds" / f"{world}.wbt")],
            cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace",
        )
    if report.exists():
        result = json.loads(report.read_text(encoding="utf-8"))
    else:
        result = {"case": case_name, "planner": planner, "passed": False, "error": "missing report"}
    result["webots_exit"] = process.returncode
    result["log"] = str(log)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", choices=("regression", "stress"), default="regression")
    parser.add_argument("--planner", choices=("v2", "legacy"), default="v2")
    parser.add_argument("--cases", help="comma separated subset")
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=150.0)
    args = parser.parse_args()
    world = f"avoidance_{args.world}"
    names = args.cases.split(",") if args.cases else list(WORLDS[world])
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = [
            pool.submit(run_case, world, name, args.planner, 12700 + index, args.timeout)
            for index, name in enumerate(names)
        ]
        results = [future.result() for future in futures]
    for result in results:
        summary = {key: result.get(key) for key in (
            "case", "passed", "goal_reached", "contacts", "stalled", "timed_out", "seconds",
            "waypoints", "recoveries", "final_state", "min_center_clearance", "turn_switches",
            "max_no_progress_s", "planner_mean_ms", "planner_max_ms", "error")}
        print(json.dumps(summary), flush=True)
    passed = all(result.get("passed") for result in results)
    print(json.dumps({"world": world, "planner": args.planner, "passed": passed,
                      "passed_count": sum(bool(r.get("passed")) for r in results),
                      "total": len(results)}), flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
