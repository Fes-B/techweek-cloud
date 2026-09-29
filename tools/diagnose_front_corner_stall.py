"""Deterministic closed-loop reproducer for static front-corner stalls.

This deliberately does not use or modify a Webots world.  It ray-casts the
same 180-beam, 360-degree LiDAR used by the avoidance worlds against one
0.45 m square obstacle, runs the production planner/control arbitration, and
integrates the commanded differential-drive velocity at TIME_STEP.
"""

import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from avoidance import DynamicWindowAvoidance
from config import DWA_ROBOT_RADIUS, LIDAR_FIELD_OF_VIEW, TIME_STEP, WHEEL_RADIUS, WHEEL_TRACK
from main import goal_in_robot_frame, nominal_waypoint_command, select_control_command


CASES = {
    # The diagonal boxes overlap the straight centre line by only 5 cm: their
    # near corner, rather than their centre, is the front-corner hazard.
    "front_left": (0.90, 0.275, 0.45, 0.45),
    "front_right": (0.90, -0.275, 0.45, 0.45),
    "front": (0.90, 0.0, 0.45, 0.45),
    "side": (0.0, 0.65, 0.45, 0.45),
}
GOALS = {
    "front_left": (3.0, 0.0),
    "front_right": (3.0, 0.0),
    # Keep the obstacle directly ahead while giving the local planner a
    # non-degenerate path around it; a goal exactly behind a centred square is
    # a separate, symmetric local-planning problem rather than a corner test.
    "front": (3.0, -0.65),
    "side": (3.0, 0.0),
}
BEAM_COUNT = 180
MAX_RANGE = 4.0


def _ray_box_distance(origin, direction, box):
    cx, cy, width, height = box
    bounds = (
        (cx - width / 2.0, cx + width / 2.0),
        (cy - height / 2.0, cy + height / 2.0),
    )
    low, high = 0.0, MAX_RANGE
    for coordinate, component, (minimum, maximum) in zip(origin, direction, bounds):
        if abs(component) < 1e-12:
            if coordinate < minimum or coordinate > maximum:
                return float("inf")
            continue
        enter = (minimum - coordinate) / component
        leave = (maximum - coordinate) / component
        if enter > leave:
            enter, leave = leave, enter
        low = max(low, enter)
        high = min(high, leave)
        if high < low:
            return float("inf")
    return low if 0.05 <= low <= MAX_RANGE else float("inf")


def lidar_scan(pose, box):
    x, y, heading = pose
    step = DynamicWindowAvoidance._beam_angle_step(LIDAR_FIELD_OF_VIEW, BEAM_COUNT)
    return [
        _ray_box_distance(
            (x, y),
            (
                math.cos(heading - LIDAR_FIELD_OF_VIEW / 2.0 + step * index),
                math.sin(heading - LIDAR_FIELD_OF_VIEW / 2.0 + step * index),
            ),
            box,
        )
        for index in range(BEAM_COUNT)
    ]


def _distance_to_box(point, box):
    x, y = point
    cx, cy, width, height = box
    dx = max(abs(x - cx) - width / 2.0, 0.0)
    dy = max(abs(y - cy) - height / 2.0, 0.0)
    return math.hypot(dx, dy)


def _candidate_copy(candidate):
    if candidate is None:
        return None
    return {
        key: (round(value, 9) if isinstance(value, float) and math.isfinite(value) else value)
        for key, value in candidate.items()
    }


def _in_wrapped_sector(index, center, half_width, count=BEAM_COUNT):
    return any((center + offset) % count == index for offset in range(-half_width, half_width + 1))


def run_case(case_name, seconds=30.0, trace_path=None):
    box = CASES[case_name]
    goal = GOALS[case_name]
    planner = DynamicWindowAvoidance()
    pose = [0.0, 0.0, 0.0]
    dt = TIME_STEP / 1000.0
    frames = []
    contacts = 0
    touching = False
    max_x = pose[0]
    path_follow_after_pass = False
    obstacle_passed = False

    for frame_index in range(math.ceil(seconds / dt)):
        now = frame_index * dt
        ranges = lidar_scan(pose, box)
        goal_local = goal_in_robot_frame(goal, pose)
        front_distance = planner._front_distance(ranges)
        front_arc_distance = planner._front_arc_distance(ranges)
        left_distance, right_distance = planner._side_distances(ranges)
        finite_indices = [index for index, value in enumerate(ranges) if math.isfinite(value)]
        closest_index = min(finite_indices, key=lambda index: ranges[index]) if finite_indices else None
        closest_beam = None
        if closest_index is not None:
            closest_beam = {
                "index": closest_index,
                "angle": -LIDAR_FIELD_OF_VIEW / 2.0 + closest_index * DynamicWindowAvoidance._beam_angle_step(LIDAR_FIELD_OF_VIEW, BEAM_COUNT),
                "range": ranges[closest_index],
                "front": _in_wrapped_sector(closest_index, BEAM_COUNT // 2, max(2, BEAM_COUNT // 36)),
                "front_arc": _in_wrapped_sector(closest_index, BEAM_COUNT // 2, max(2, BEAM_COUNT // 7)),
                "left_side": _in_wrapped_sector(closest_index, 3 * BEAM_COUNT // 4, max(2, BEAM_COUNT // 18)),
                "right_side": _in_wrapped_sector(closest_index, BEAM_COUNT // 4, max(2, BEAM_COUNT // 18)),
            }

        avoidance = planner.choose_action(ranges, goal_local, pose=tuple(pose), sim_time=now)
        nominal = nominal_waypoint_command(goal_local)
        left, right, control = select_control_command(planner, nominal, avoidance)
        command_v = 0.5 * (left + right) * WHEEL_RADIUS
        command_w = (right - left) * WHEEL_RADIUS / WHEEL_TRACK

        record = {
            "frame": frame_index,
            "time": round(now, 6),
            "robot_pose": [round(value, 9) for value in pose],
            "goal_local": [round(value, 9) for value in goal_local],
            "front_distance": front_distance,
            "front_arc_distance": front_arc_distance,
            "left_distance": left_distance,
            "right_distance": right_distance,
            "side_speed_limit": planner.side_speed_limit,
            "recovery_phase": planner.recovery_phase,
            "wall_side": planner.wall_side,
            "wall_escape_active": planner.wall_escape_active,
            "front_corner_escape_active": planner._front_corner_escape_active,
            "closest_obstacle_beam": closest_beam,
            "current_v": planner.current_v,
            "current_w": planner.current_w,
            "v_low": planner.debug_v_window[0] if planner.debug_v_window else None,
            "v_high": planner.debug_v_window[1] if planner.debug_v_window else None,
            "w_low": planner.debug_w_window[0] if planner.debug_w_window else None,
            "w_high": planner.debug_w_window[1] if planner.debug_w_window else None,
            "total_candidates": planner.debug_total_candidates,
            "static_safe": planner.debug_static_ok_count,
            "dynamic_safe": planner.debug_dynamic_ok_count,
            "braking_safe": planner.debug_braking_ok_count,
            "valid_forward_candidates": planner.debug_valid_forward_count,
            "best_candidate": _candidate_copy(planner.debug_best_candidate),
            "best_forward_candidate": _candidate_copy(planner.debug_best_forward_candidate),
            "control": control,
            "command_v": command_v,
            "command_w": command_w,
        }
        frames.append(record)

        pose[0] += command_v * math.cos(pose[2]) * dt
        pose[1] += command_v * math.sin(pose[2]) * dt
        pose[2] = math.atan2(
            math.sin(pose[2] + command_w * dt),
            math.cos(pose[2] + command_w * dt),
        )
        max_x = max(max_x, pose[0])

        contact_now = _distance_to_box(pose[:2], box) <= DWA_ROBOT_RADIUS
        if contact_now and not touching:
            contacts += 1
        touching = contact_now
        obstacle_passed = pose[0] > box[0] + box[2] / 2.0 + DWA_ROBOT_RADIUS
        if obstacle_passed and control == "PATH_FOLLOW":
            path_follow_after_pass = True
        if math.dist(pose[:2], goal) < 0.14:
            break

    if trace_path is not None:
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        with trace_path.open("w", encoding="utf-8") as stream:
            for frame in frames:
                stream.write(json.dumps(frame, allow_nan=True) + "\n")

    last_window = frames[-min(len(frames), math.ceil(5.0 / dt)):]
    recent_progress = (
        math.dist(last_window[0]["robot_pose"][:2], last_window[-1]["robot_pose"][:2])
        if len(last_window) > 1
        else 0.0
    )
    minimum_clearance = min(_distance_to_box(frame["robot_pose"][:2], box) for frame in frames)
    result = {
        "case": case_name,
        "frames": len(frames),
        "final_pose": [round(value, 6) for value in pose],
        "position_progress": round(max_x, 6),
        "recent_5s_progress": round(recent_progress, 6),
        "minimum_clearance": round(minimum_clearance, 6),
        "contacts": contacts,
        "obstacle_passed": obstacle_passed,
        "path_following_returned": path_follow_after_pass,
        "goal_reached": math.dist(pose[:2], goal) < 0.14,
        "stalled": recent_progress < 0.05 and not obstacle_passed,
        "last_frame": frames[-1],
    }
    return result, frames


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=(*CASES, "all"), default="all")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--trace-dir", type=Path)
    args = parser.parse_args()

    names = CASES if args.case == "all" else (args.case,)
    results = []
    for name in names:
        trace = args.trace_dir / f"{name}.jsonl" if args.trace_dir else None
        result, _ = run_case(name, args.seconds, trace)
        results.append(result)
        print(json.dumps(result, indent=2, allow_nan=True))
    return 1 if any(item["contacts"] or item["stalled"] for item in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
