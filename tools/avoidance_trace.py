"""Record planner inputs in Webots and replay them offline.

Diagnostics only.  A controller calls ``TraceRecorder.from_env()`` and, per
frame, ``recorder.record(...)`` with exactly what it passed to
``choose_action``; set AVOIDANCE_TRACE=/path/file.jsonl to enable it.  The
planner is deterministic given its inputs, so replaying the recorded frames
through a fresh planner reproduces the Webots decisions and lets you inspect
any internal state offline:

    python tools/avoidance_trace.py /tmp/practice.jsonl --from 80 --to 90
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class TraceRecorder:
    def __init__(self, path):
        self._stream = Path(path).open("w", encoding="utf-8")

    @classmethod
    def from_env(cls, variable="AVOIDANCE_TRACE"):
        path = os.environ.get(variable)
        return cls(path) if path else None

    def record(self, sim_time, pose, local_goal, ranges, waypoint_id, command, truth=None):
        frame = {
            "t": sim_time, "pose": list(pose), "goal": list(local_goal),
            "ranges": [value if math.isfinite(value) else None for value in ranges],
            "wp": waypoint_id, "cmd": list(command[:2]), "action": command[2],
        }
        if truth is not None:
            frame["truth"] = list(truth)
        self._stream.write(json.dumps(frame) + "\n")
        self._stream.flush()


def load(path):
    frames = []
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            frame = json.loads(line)
            frame["ranges"] = [math.inf if value is None else value for value in frame["ranges"]]
            frames.append(frame)
    return frames


def replay(frames, planner=None, fov=None, callback=None):
    """Feed recorded frames to a fresh ProgressDWA; returns the planner."""
    from avoidance_v2 import ProgressDWA
    from config import LIDAR_FIELD_OF_VIEW

    planner = planner or ProgressDWA(lidar_field_of_view=fov or LIDAR_FIELD_OF_VIEW)
    for frame in frames:
        result = planner.choose_action(
            frame["ranges"], tuple(frame["goal"]), pose=tuple(frame["pose"]),
            sim_time=frame["t"], waypoint_id=frame["wp"],
        )
        if callback is not None:
            callback(frame, result, planner)
    return planner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--from", dest="start", type=float, default=0.0)
    parser.add_argument("--to", dest="end", type=float, default=math.inf)
    parser.add_argument("--every", type=int, default=1)
    parser.add_argument("--fov", type=float, default=None)
    args = parser.parse_args()
    frames = load(args.trace)
    mismatches = 0
    counter = [0]

    def show(frame, result, planner):
        if abs(result[0] - frame["cmd"][0]) > 1e-6 or abs(result[1] - frame["cmd"][1]) > 1e-6:
            nonlocal mismatches
            mismatches += 1
        if not (args.start <= frame["t"] <= args.end):
            return
        counter[0] += 1
        if counter[0] % args.every:
            return
        diag = planner.diagnostics
        dwa = diag.get("dwa") or {}
        best = dwa.get("best") or {}
        print(json.dumps({
            "t": round(frame["t"], 3), "pose": [round(v, 3) for v in frame["pose"]],
            "state": diag.get("state"), "layer": diag.get("layer"), "action": result[2],
            "adm": dwa.get("admissible"), "static_ok": dwa.get("static_ok"),
            "best": {k: round(v, 3) for k, v in best.items()} if best else None,
            "watchdog": diag.get("watchdog"), "recovery": diag.get("recovery"),
            "moving": diag.get("moving"),
        }, default=str))

    replay(frames, fov=args.fov, callback=show)
    print(f"# replayed {len(frames)} frames, command mismatches vs recording: {mismatches}")


if __name__ == "__main__":
    main()
