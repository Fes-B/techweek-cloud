"""Opt-in frame recorder and offline replay for the avoidance benchmarks.

Recording is enabled only when the ``AVOID_TRACE`` environment variable names
an output file; otherwise ``TraceWriter`` does nothing.  Each JSON line holds
exactly the planner inputs of one control step (LiDAR ranges, pose, time,
local goal) plus the selected command, so a Webots failure can be replayed
through ``DynamicWindowAvoidance`` offline with full introspection:

    python tools/avoidance_trace.py trace.jsonl --from 81.0 --to 86.0

Replay feeds the recorded scans in order to a fresh planner (tracker and
state machines rebuild from the same inputs) and prints the planner state and
best candidates for the requested time window.  It does not use Webots.
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


def _finite(value):
    return value if math.isfinite(value) else None


class TraceWriter:
    def __init__(self, path=None):
        path = path if path is not None else os.environ.get("AVOID_TRACE")
        self._stream = open(path, "w", encoding="utf-8") if path else None

    def record(self, now, pose, local_goal, ranges, command, fov, truth=None):
        if self._stream is None:
            return
        self._stream.write(json.dumps({
            "t": now,
            "pose": list(pose),
            "truth": list(truth) if truth is not None else None,
            "goal": list(local_goal),
            "fov": fov,
            "ranges": [_finite(value) for value in ranges],
            "command": [command[0], command[1], command[2]],
        }) + "\n")

    def close(self):
        if self._stream is not None:
            self._stream.close()
            self._stream = None


def load(path):
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            frame = json.loads(line)
            frame["ranges"] = [
                float("inf") if value is None else value
                for value in frame["ranges"]
            ]
            yield frame


def replay(path, start=None, end=None, planner_factory=None):
    """Yield (frame, planner, avoidance_command, selected_command)."""
    from avoidance import DynamicWindowAvoidance
    from main import nominal_waypoint_command, select_control_command

    planner = None
    for frame in load(path):
        if planner is None:
            planner = (planner_factory or DynamicWindowAvoidance)(
                lidar_field_of_view=frame["fov"]
            )
        goal = tuple(frame["goal"])
        avoidance = planner.choose_action(
            frame["ranges"], goal, pose=tuple(frame["pose"]), sim_time=frame["t"]
        )
        selected = select_control_command(
            planner, nominal_waypoint_command(goal), avoidance
        )
        if (start is None or frame["t"] >= start) and (end is None or frame["t"] <= end):
            yield frame, planner, avoidance, selected
        if end is not None and frame["t"] > end:
            break


def _describe(frame, planner, avoidance, selected):
    best = planner.debug_best_candidate or {}
    forward = planner.debug_best_forward_candidate or {}

    def fmt(candidate):
        if not candidate:
            return "-"
        keys = ("v", "w", "score", "goal_distance", "heading", "clearance",
                "headroom", "side", "static_clearance")
        return " ".join(
            f"{key}={candidate[key]:.3f}" for key in keys
            if isinstance(candidate.get(key), float)
        )

    recorded = frame["command"]
    same = (abs(recorded[0] - selected[0]) < 1e-6 and abs(recorded[1] - selected[1]) < 1e-6)
    return (
        f"t={frame['t']:.3f} pose=({frame['pose'][0]:.3f},{frame['pose'][1]:.3f},{frame['pose'][2]:.3f}) "
        f"goal=({frame['goal'][0]:.2f},{frame['goal'][1]:.2f}) "
        f"front={planner._front_distance(frame['ranges']):.3f} "
        f"arc={planner._front_arc_distance(frame['ranges']):.3f} "
        f"phase={planner.recovery_phase} latch={planner._recovery_reentry_latched} "
        f"corner={planner._front_corner_escape_active} backoff={planner.backoff_steps} "
        f"cross={planner.crossing_waiting}/{planner.crossing_commit_active} "
        f"cur=({planner.current_v:.3f},{planner.current_w:.3f}) "
        f"win={tuple(round(x, 3) for x in planner.debug_v_window or ())} "
        f"n={planner.debug_total_candidates}/{planner.debug_static_ok_count}/"
        f"{planner.debug_dynamic_ok_count}/{planner.debug_braking_ok_count} fwd={planner.debug_valid_forward_count} "
        f"moving={len(planner.last_dynamic_obstacles)} "
        f"sel={selected[2]}{'' if same else ' (DIFFERS FROM RECORDING)'}\n"
        f"    best[{fmt(best)}]\n    fwd [{fmt(forward)}]\n    avoid={avoidance[2]}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    parser.add_argument("--from", dest="start", type=float)
    parser.add_argument("--to", dest="end", type=float)
    parser.add_argument("--every", type=int, default=1)
    args = parser.parse_args()
    for index, item in enumerate(replay(args.trace, args.start, args.end)):
        if index % args.every == 0:
            print(_describe(*item))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
