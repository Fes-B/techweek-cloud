"""Deterministic closed-loop 2D harness for local-avoidance regression.

Not a substitute for Webots (no wheel dynamics, slip or caster effects), but
fast and repeatable: it ray-casts the practice 180-beam, 360-degree LiDAR
against axis-aligned boxes (static, or moving on a scripted crossing), runs a
planner exactly as the Webots controllers do, and integrates the commanded
differential-drive velocity at TIME_STEP.  Contact is checked with the real
robot outline (0.25 m body, 0.26 m across the wheels), not the planner circle.

Usage:
    python tools/avoidance_sim.py --planner v2 --case all
    python tools/avoidance_sim.py --planner legacy --case front_left --trace out.jsonl
"""

import argparse
import json
import math
from pathlib import Path
import random
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import LIDAR_FIELD_OF_VIEW, TIME_STEP, WHEEL_RADIUS, WHEEL_TRACK
from tools.avoidance_scenarios import CASES, REGRESSION_CASES, STRESS_CASES


BEAM_COUNT = 180
MAX_RANGE = 4.0
MIN_RANGE = 0.05
LIDAR_NOISE = 0.002  # same standard deviation as the Webots Lidar node
ROBOT_HALF_LENGTH = 0.125
ROBOT_HALF_WIDTH = 0.13  # body 0.09 + wheels to 0.13
WAYPOINT_TOLERANCE = 0.14
STALL_SECONDS = 35.0


class Crossing:
    """Box moving along one axis after the robot passes a trigger, with dwell.

    Same state machine as controllers/extended_crossing_controller.
    """

    def __init__(self, spec):
        self.size = spec["size"]
        self.axis = spec["axis"]
        self.fixed = spec["fixed"]
        self.low = spec["low"]
        self.high = spec["high"]
        self.speed = spec["speed"]
        self.dwell = spec["dwell"]
        self.trigger_index = 0 if spec["trigger_axis"] == "x" else 1
        self.trigger = spec["trigger"]
        from_high = spec.get("start", "min") == "max"
        self.position = self.high if from_high else self.low
        self.direction = -1.0 if from_high else 1.0
        self.state = "waiting"
        self.dwell_until = 0.0

    def step(self, now, dt, robot_pose):
        if self.state == "waiting":
            if robot_pose[self.trigger_index] >= self.trigger:
                self.state = "moving"
        elif self.state == "dwelling":
            if now >= self.dwell_until:
                self.direction *= -1.0
                self.state = "moving"
        else:
            self.position += self.direction * self.speed * dt
            end = self.high if self.direction > 0 else self.low
            if (self.direction > 0 and self.position >= end) or (
                self.direction < 0 and self.position <= end
            ):
                self.position = end
                self.state = "dwelling"
                self.dwell_until = now + self.dwell

    def box(self):
        if self.axis == "x":
            return (self.position, self.fixed, self.size, self.size)
        return (self.fixed, self.position, self.size, self.size)


def ray_box(origin, direction, rect):
    cx, cy, sx, sy = rect
    low, high = 0.0, MAX_RANGE
    for coordinate, component, (minimum, maximum) in zip(
        origin, direction, ((cx - sx / 2, cx + sx / 2), (cy - sy / 2, cy + sy / 2))
    ):
        if abs(component) < 1e-12:
            if coordinate < minimum or coordinate > maximum:
                return math.inf
            continue
        enter = (minimum - coordinate) / component
        leave = (maximum - coordinate) / component
        if enter > leave:
            enter, leave = leave, enter
        low = max(low, enter)
        high = min(high, leave)
        if high < low:
            return math.inf
    return low


def lidar_scan(pose, rects, rng=None):
    x, y, heading = pose
    ranges = []
    for index in range(BEAM_COUNT):
        angle = heading - LIDAR_FIELD_OF_VIEW / 2.0 + LIDAR_FIELD_OF_VIEW * index / (BEAM_COUNT - 1)
        direction = (math.cos(angle), math.sin(angle))
        distance = min((ray_box((x, y), direction, rect) for rect in rects), default=math.inf)
        if math.isfinite(distance) and rng is not None:
            distance += rng.gauss(0.0, LIDAR_NOISE)
        ranges.append(distance if MIN_RANGE <= distance <= MAX_RANGE else math.inf)
    return ranges


def _robot_corners(pose):
    x, y, heading = pose
    c, s = math.cos(heading), math.sin(heading)
    return [
        (x + c * dx - s * dy, y + s * dx + c * dy)
        for dx, dy in ((ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH), (-ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH),
                       (-ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH), (ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH))
    ]


def footprint_hits(pose, rect):
    """Separating-axis test between the rotated robot outline and a box."""
    cx, cy, sx, sy = rect
    robot = _robot_corners(pose)
    obstacle = [(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2),
                (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2)]
    heading = pose[2]
    axes = [(1.0, 0.0), (0.0, 1.0), (math.cos(heading), math.sin(heading)),
            (-math.sin(heading), math.cos(heading))]
    for ax, ay in axes:
        a = [px * ax + py * ay for px, py in robot]
        b = [px * ax + py * ay for px, py in obstacle]
        if max(a) < min(b) or max(b) < min(a):
            return False
    return True


def center_clearance(point, rect):
    cx, cy, sx, sy = rect
    dx = max(abs(point[0] - cx) - sx / 2, 0.0)
    dy = max(abs(point[1] - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def passable(case, clearance=0.25, resolution=0.02, margin=1.0):
    """True when a disk of radius ``clearance`` can reach every waypoint.

    Used to reject stress geometries that no safe controller could pass:
    the planner keeps DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN = 0.22 m,
    so scenarios are required to leave a few centimetres beyond that.
    Moving boxes are ignored (they clear their lane on a schedule).
    """
    import numpy as np
    from collections import deque

    points = [case["start"][:2], *case["waypoints"]]
    xs = [p[0] for p in points] + [b[0] - b[2] / 2 for b in case["boxes"]] + [b[0] + b[2] / 2 for b in case["boxes"]]
    ys = [p[1] for p in points] + [b[1] - b[3] / 2 for b in case["boxes"]] + [b[1] + b[3] / 2 for b in case["boxes"]]
    x0, y0 = min(xs) - margin, min(ys) - margin
    cols = int((max(xs) + margin - x0) / resolution) + 1
    rows = int((max(ys) + margin - y0) / resolution) + 1
    gx, gy = np.meshgrid(x0 + resolution * np.arange(cols), y0 + resolution * np.arange(rows))
    free = np.ones((rows, cols), dtype=bool)
    for cx, cy, sx, sy in case["boxes"]:
        dx = np.maximum(np.abs(gx - cx) - sx / 2, 0.0)
        dy = np.maximum(np.abs(gy - cy) - sy / 2, 0.0)
        free &= np.hypot(dx, dy) > clearance

    def cell(p):
        return int(round((p[1] - y0) / resolution)), int(round((p[0] - x0) / resolution))

    for start, goal in zip(points, points[1:]):
        s, g = cell(start), cell(goal)
        if not free[s] or not free[g]:
            return False
        seen = np.zeros_like(free)
        seen[s] = True
        queue = deque([s])
        while queue:
            r, c = queue.popleft()
            if (r, c) == g:
                break
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True
                    queue.append((nr, nc))
        if not seen[g]:
            return False
    return True


def make_planner(kind):
    if kind == "v2":
        from avoidance_v2 import ProgressDWA
        return ProgressDWA()
    from avoidance import DynamicWindowAvoidance
    return DynamicWindowAvoidance()


def run_case(name, planner_kind="v2", seconds=90.0, seed=7, trace_path=None, case=None):
    from main import goal_in_robot_frame, nominal_waypoint_command, select_control_command

    case = case or CASES[name]
    rng = random.Random(seed) if seed is not None else None
    planner = make_planner(planner_kind)
    movers = [Crossing(spec) for spec in case.get("movers", [])]
    pose = list(case["start"])
    waypoints = case["waypoints"]
    dt = TIME_STEP / 1000.0
    waypoint_index = 0
    contacts = 0
    touching = False
    min_clearance = math.inf
    best_distance = math.inf
    progress_time = 0.0
    compute = []
    labels = {}
    turn_switches = 0
    previous_turn = 0
    stalled = False
    trace = []
    states_seen = set()
    last_state = None

    now = 0.0
    frames = math.ceil(seconds / dt)
    for frame in range(frames):
        now = frame * dt
        for mover in movers:
            mover.step(now, dt, pose)
        rects = list(case["boxes"]) + [mover.box() for mover in movers]
        goal = waypoints[waypoint_index]
        if math.dist(pose[:2], goal) < WAYPOINT_TOLERANCE:
            waypoint_index += 1
            best_distance = math.inf
            progress_time = now
            if waypoint_index == len(waypoints):
                break
            goal = waypoints[waypoint_index]

        ranges = lidar_scan(pose, rects, rng)
        local_goal = goal_in_robot_frame(goal, pose)
        started = time.perf_counter()
        if planner_kind == "v2":
            left, right, action = planner.choose_action(
                ranges, local_goal, pose=tuple(pose), sim_time=now, waypoint_id=waypoint_index
            )
            control = planner.control_label
        else:
            avoidance = planner.choose_action(ranges, local_goal, pose=tuple(pose), sim_time=now)
            left, right, control = select_control_command(
                planner, nominal_waypoint_command(local_goal), avoidance
            )
            action = avoidance[2]
        compute.append(time.perf_counter() - started)
        labels[control] = labels.get(control, 0) + 1
        v = 0.5 * WHEEL_RADIUS * (left + right)
        w = WHEEL_RADIUS * (right - left) / WHEEL_TRACK
        turn = 1 if w > 0.15 else -1 if w < -0.15 else 0
        if turn and previous_turn and turn != previous_turn:
            turn_switches += 1
        if turn:
            previous_turn = turn
        if planner_kind == "v2":
            state = planner.state.value
            states_seen.add(state)
            last_state = state

        if trace_path is not None:
            record = {"t": round(now, 3), "pose": [round(p, 4) for p in pose],
                      "wp": waypoint_index, "v": round(v, 4), "w": round(w, 4),
                      "control": control, "action": action}
            if planner_kind == "v2":
                record["diag"] = _jsonable(planner.diagnostics)
            trace.append(record)

        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] = math.atan2(math.sin(pose[2] + w * dt), math.cos(pose[2] + w * dt))

        rects = list(case["boxes"]) + [mover.box() for mover in movers]
        for rect in rects:
            min_clearance = min(min_clearance, center_clearance(pose[:2], rect))
        hit = any(footprint_hits(pose, rect) for rect in rects)
        if hit and not touching:
            contacts += 1
        touching = hit
        distance = math.dist(pose[:2], waypoints[min(waypoint_index, len(waypoints) - 1)])
        if distance < best_distance - 0.04:
            best_distance = distance
            progress_time = now
        if now - progress_time > STALL_SECONDS:
            stalled = True
            break

    if trace_path is not None:
        Path(trace_path).write_text(
            "\n".join(json.dumps(item, allow_nan=True) for item in trace), encoding="utf-8"
        )
    goal_reached = waypoint_index == len(waypoints)
    result = {
        "case": name,
        "planner": planner_kind,
        "goal_reached": goal_reached,
        "contacts": contacts,
        "stalled": stalled,
        "seconds": round(now, 2),
        "waypoints": f"{waypoint_index}/{len(waypoints)}",
        "min_center_clearance": round(min_clearance, 4),
        "turn_switches": turn_switches,
        "planner_mean_ms": round(1000.0 * sum(compute) / max(1, len(compute)), 3),
        "planner_max_ms": round(1000.0 * max(compute, default=0.0), 3),
        "labels": labels,
        "final_pose": [round(value, 4) for value in pose],
    }
    if planner_kind == "v2":
        result["recoveries"] = planner.recovery_count
        result["final_state"] = last_state
        result["states_seen"] = sorted(states_seen)
    result["passed"] = goal_reached and contacts == 0 and not stalled and (
        planner_kind != "v2" or last_state == "NORMAL_DWA"
    )
    return result


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, float):
        return round(value, 5) if math.isfinite(value) else str(value)
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", choices=("v2", "legacy"), default="v2")
    parser.add_argument("--case", default="all",
                        help="case name, 'all', 'regression' or 'stress'")
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args()
    if args.case == "all":
        names = tuple(CASES)
    elif args.case == "regression":
        names = REGRESSION_CASES
    elif args.case == "stress":
        names = STRESS_CASES
    else:
        names = tuple(args.case.split(","))
    failed = 0
    for name in names:
        trace = args.trace if len(names) == 1 else None
        seconds = max(args.seconds, CASES[name].get("seconds", 0.0))
        result = run_case(name, args.planner, seconds, args.seed, trace)
        failed += not result["passed"]
        print(json.dumps(result), flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
