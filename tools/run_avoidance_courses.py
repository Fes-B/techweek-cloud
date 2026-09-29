"""Run generated avoidance courses (and optionally the benchmarks) headlessly.

    python tools/run_avoidance_courses.py                 # all course_*.wbt
    python tools/run_avoidance_courses.py min_ stress_u   # name filters
    python tools/run_avoidance_courses.py --jobs 2 --trace-dir traces

Worlds are produced by tools/build_avoidance_courses.py.  On Linux without a
display the runner wraps Webots in xvfb-run when it is available.  Results are
read from each course's JSON report and printed as one table; the exit code is
non-zero when any course fails.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def webots_command():
    home = os.environ.get("WEBOTS_HOME")
    candidates = []
    if home:
        candidates += [Path(home) / "msys64/mingw64/bin/webots.exe", Path(home) / "webots"]
    candidates.append(Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"))
    for candidate in candidates:
        if candidate.exists():
            command = [str(candidate)]
            break
    else:
        command = ["webots"]
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
        command = ["xvfb-run", "-a"] + command
    return command


def run_world(world, result_dir, trace_dir, timeout):
    name = world.stem.removeprefix("course_")
    environment = os.environ.copy()
    environment["PATH"] = str(Path(sys.executable).parent) + os.pathsep + environment.get("PATH", "")
    environment["COURSE_RESULT_DIR"] = str(result_dir)
    environment.setdefault("LANG", "C.UTF-8")
    if trace_dir is not None:
        environment["AVOID_TRACE"] = str(Path(trace_dir) / f"{name}.jsonl")
    report = Path(result_dir) / f"techweek-course-{name}.json"
    report.unlink(missing_ok=True)
    log = Path(result_dir) / f"techweek-course-{name}.log"
    with log.open("w", encoding="utf-8") as stream:
        try:
            subprocess.run(
                webots_command() + ["--batch", "--mode=fast", "--no-rendering",
                                    "--stdout", "--stderr", "--minimize", str(world)],
                cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT,
                timeout=timeout, check=False,
            )
        except subprocess.TimeoutExpired:
            pass
    if report.exists():
        return name, json.loads(report.read_text(encoding="utf-8")), log
    return name, None, log


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("filters", nargs="*")
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--trace-dir", type=Path)
    parser.add_argument("--result-dir", type=Path, default=Path(tempfile.gettempdir()))
    parser.add_argument("--timeout", type=float, default=3600.0)
    args = parser.parse_args()
    worlds = sorted((ROOT / "worlds").glob("course_*.wbt"))
    if args.filters:
        worlds = [w for w in worlds if any(f in w.stem for f in args.filters)]
    if args.trace_dir is not None:
        args.trace_dir.mkdir(parents=True, exist_ok=True)
    args.result_dir.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(
            lambda world: run_world(world, args.result_dir, args.trace_dir, args.timeout),
            worlds,
        ))
    failed = 0
    columns = ("completed", "reached_waypoints", "total_waypoints", "contacts", "stalled",
               "timed_out", "seconds", "recovery_entries", "max_turn_flips",
               "oscillation_events", "min_lidar_clearance", "planner_mean_ms", "planner_max_ms")
    for name, result, log in results:
        if result is None:
            failed += 1
            print(f"{name:32s} NO RESULT (log: {log})")
            continue
        failed += not result["completed"]
        print(f"{name:32s} " + " ".join(f"{key}={result[key]}" for key in columns))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
