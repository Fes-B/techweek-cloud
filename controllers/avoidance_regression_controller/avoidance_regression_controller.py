"""Scenario runner for worlds/avoidance_regression.wbt and avoidance_stress.wbt.

One Webots launch runs one scenario, selected with REGRESSION_CASE (see
tools/avoidance_scenarios.py).  The robot node is identical to the Extended
world's; before the first physics step the controller teleports it to the
scenario start pose.  The planner sees only encoder/compass odometry
(practice_odometry.EncoderPose) and LiDAR, as in the Extended benchmark.
Supervisor position is used for scoring (contacts, clearance, stall) only.

Environment:
    REGRESSION_CASE      scenario name (required)
    AVOIDANCE_PLANNER    "v2" (default) or "legacy"
    REGRESSION_TIMEOUT   seconds (default 150)
"""

import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import TIME_STEP, WHEEL_RADIUS, WHEEL_TRACK
from main import goal_in_robot_frame, nominal_waypoint_command, select_control_command
from practice_odometry import EncoderPose
from robot_io import RobotIO
from tools.avoidance_scenarios import world_case


WAYPOINT_TOLERANCE = 0.14
STALL_SECONDS = 35.0


def _clearance(point, rect):
    cx, cy, sx, sy = rect
    return math.hypot(max(abs(point[0] - cx) - sx / 2, 0.0), max(abs(point[1] - cy) - sy / 2, 0.0))


def main(robot):
    case_name = os.environ["REGRESSION_CASE"]
    planner_kind = os.environ.get("AVOIDANCE_PLANNER", "v2")
    timeout = float(os.environ.get("REGRESSION_TIMEOUT", "150"))
    case = world_case(case_name)
    result_path = Path(tempfile.gettempdir(), f"techweek-regression-{case_name}.json")
    result_path.unlink(missing_ok=True)

    node = robot.getSelf()
    x, y, theta = case["start"]
    node.getField("translation").setSFVec3f([x, y, 0.04])
    node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, theta])
    node.resetPhysics()
    robot.step(TIME_STEP)

    io = RobotIO(robot)
    if planner_kind == "legacy":
        from avoidance import DynamicWindowAvoidance
        planner = DynamicWindowAvoidance(lidar_field_of_view=io.lidar.getFov())
    else:
        from avoidance_v2 import ProgressDWA
        planner = ProgressDWA(lidar_field_of_view=io.lidar.getFov())
    odometry = EncoderPose(robot, case["start"])
    node.enableContactPointsTracking(TIME_STEP, True)

    waypoints = case["waypoints"]
    waypoint_index = 0
    contacts = 0
    touching = False
    best_distance = math.inf
    progress_time = robot.getTime()
    start_time = robot.getTime()
    min_clearance = math.inf
    compute = []
    labels = {}
    states = {}
    turn_switches = 0
    previous_turn = 0
    max_no_progress = 0.0
    max_position_error = 0.0
    last_log = -1.0
    stalled = False
    timed_out = False

    while robot.step(TIME_STEP) != -1:
        now = robot.getTime() - start_time
        truth_position = node.getPosition()
        truth = (truth_position[0], truth_position[1])
        pose = odometry.update()
        max_position_error = max(max_position_error, math.dist(pose[:2], truth))
        for rect in case["boxes"]:
            min_clearance = min(min_clearance, _clearance(truth, rect))

        if math.dist(pose[:2], waypoints[waypoint_index]) < WAYPOINT_TOLERANCE:
            print(f"[WAYPOINT] t={now:.2f} reached={waypoint_index + 1}/{len(waypoints)}", flush=True)
            waypoint_index += 1
            best_distance = math.inf
            progress_time = now
        contact_now = any(point.getPoint()[2] > 0.01 for point in node.getContactPoints(True))
        if contact_now and not touching:
            contacts += 1
            print(f"[CONTACT] t={now:.2f} pose={pose}", flush=True)
        touching = contact_now

        if waypoint_index < len(waypoints):
            distance = math.dist(truth, waypoints[waypoint_index])
            if distance < best_distance - 0.04:
                best_distance = distance
                progress_time = now
        stalled = waypoint_index < len(waypoints) and now - progress_time > STALL_SECONDS
        timed_out = now > timeout
        goal_reached = waypoint_index == len(waypoints)

        if goal_reached or stalled or timed_out or contacts:
            io.stop()
            final_state = planner.state.value if planner_kind != "legacy" else None
            passed = (goal_reached and contacts == 0 and not stalled and not timed_out
                      and (planner_kind == "legacy" or final_state == "NORMAL_DWA"))
            result = {
                "case": case_name,
                "planner": planner_kind,
                "passed": passed,
                "goal_reached": goal_reached,
                "contacts": contacts,
                "stalled": stalled,
                "timed_out": timed_out,
                "seconds": round(now, 2),
                "waypoints": f"{waypoint_index}/{len(waypoints)}",
                "min_center_clearance": round(min_clearance, 4) if math.isfinite(min_clearance) else None,
                "turn_switches": turn_switches,
                "recoveries": getattr(planner, "recovery_count", None),
                "final_state": final_state,
                "states": states,
                "labels": labels,
                "max_no_progress_s": round(max_no_progress, 3),
                "max_position_error": round(max_position_error, 4),
                "planner_mean_ms": round(1000.0 * sum(compute) / max(1, len(compute)), 3),
                "planner_max_ms": round(1000.0 * max(compute, default=0.0), 3),
            }
            result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
            print("[RESULT] " + json.dumps(result), flush=True)
            robot.step(TIME_STEP)
            robot.simulationQuit(0 if passed else 1)
            return

        goal = waypoints[waypoint_index]
        ranges = io.get_lidar()
        local_goal = goal_in_robot_frame(goal, pose)
        started = time.perf_counter()
        if planner_kind == "legacy":
            avoidance = planner.choose_action(ranges, local_goal, pose=pose, sim_time=now)
            left, right, control = select_control_command(
                planner, nominal_waypoint_command(local_goal), avoidance
            )
            action = avoidance[2]
        else:
            left, right, action = planner.choose_action(
                ranges, local_goal, pose=pose, sim_time=now, waypoint_id=waypoint_index
            )
            control = planner.control_label
            state = planner.state.value
            states[state] = states.get(state, 0) + 1
            max_no_progress = max(max_no_progress, planner.watchdog.no_progress_seconds)
        compute.append(time.perf_counter() - started)
        io.set_wheel_speed(left, right)
        labels[control] = labels.get(control, 0) + 1
        yaw_rate = WHEEL_RADIUS * (right - left) / WHEEL_TRACK
        turn = 1 if yaw_rate > 0.15 else -1 if yaw_rate < -0.15 else 0
        if turn and previous_turn and turn != previous_turn:
            turn_switches += 1
        if turn:
            previous_turn = turn

        if now - last_log >= 1.0:
            print(f"[RUN] t={now:.2f} wp={waypoint_index} pose=({pose[0]:.3f},{pose[1]:.3f},{pose[2]:.3f}) "
                  f"control={control} action={action}", flush=True)
            last_log = now


if __name__ == "__main__":
    from controller import Supervisor

    main(Supervisor())
