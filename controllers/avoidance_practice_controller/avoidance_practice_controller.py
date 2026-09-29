"""Automated functional benchmark for worlds/avoidance_practice.wbt."""

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

from avoidance import DynamicWindowAvoidance
from config import TIME_STEP
from main import (
    goal_in_robot_frame,
    nominal_waypoint_command,
    select_control_command,
)
from practice_odometry import EncoderPose
from robot_io import RobotIO
from tools.avoidance_trace import TraceRecorder


WAYPOINTS = (
    (-4.00, -3.50),  # clear start lane
    (-2.75, -3.85),  # bypass: swing south of the asymmetric block early
    (-2.10, -3.75),  # beyond asymmetric static obstacle, through the open gap
    (-1.10, -3.00),  # corridor entrance / path recovery
    (1.65, -3.00),   # horizontal corridor exit and turn point
    (1.70, -1.00),   # 90-degree corner exit
    (1.70, 0.10),    # crossing waiting approach
    (1.70, 1.55),    # crossing exit
    (1.70, 2.05),    # final-static approach
    (1.00, 2.75),    # bypass: swing well west of the final static block
    (1.70, 4.00),    # open goal area
)
WAYPOINT_ROLES = (
    "clear start",
    "asymmetric static bypass",
    "asymmetric static exit",
    "corridor entrance",
    "corner turn",
    "corner exit",
    "crossing wait line",
    "crossing exit",
    "final static approach",
    "final static bypass",
    "goal",
)
CROSSING_Y = 0.70
MAX_SECONDS = 240.0
STALL_SECONDS = 35.0
RESULT_PATH = Path(
    tempfile.gettempdir(), "techweek-avoidance-practice-result.json"
)
# "legacy": avoidance.DynamicWindowAvoidance + path-follower arbitration.
# "v2": avoidance_v2.ProgressDWA drives directly (the waypoint is its goal).
PLANNER = os.environ.get("AVOIDANCE_PLANNER", "legacy")


def main():
    print(f"[PYTHON] {sys.version.split()[0]} {sys.executable}", flush=True)
    robot = Supervisor()
    io = RobotIO(robot)
    if PLANNER == "v2":
        from avoidance_v2 import ProgressDWA
        planner = ProgressDWA(lidar_field_of_view=io.lidar.getFov())
    else:
        planner = DynamicWindowAvoidance(lidar_field_of_view=io.lidar.getFov())
    print(f"[PLANNER] {PLANNER}", flush=True)
    recorder = TraceRecorder.from_env() if PLANNER == "v2" else None
    odometry = EncoderPose(robot, (-4.8, -3.5, 0.0))
    node = robot.getSelf()
    node.enableContactPointsTracking(TIME_STEP, True)

    waypoint_index = 0
    contacts = 0
    touching = False
    best_distance = float("inf")
    progress_time = robot.getTime()
    last_print = -1.0
    previous_control = None
    previous_action_kind = None
    previous_commit = False
    compute_total = 0.0
    compute_max = 0.0
    compute_steps = 0
    max_position_error = 0.0

    avoidance_override_count = 0
    avoidance_override_events = 0
    crossing_yield_count = 0
    crossing_gap_count = 0
    crossing_commit_count = 0
    crossing_complete_count = 0
    crossing_cancel_count = 0
    emergency_stop_count = 0
    path_recovery_count = 0
    static_override_seen = False
    post_crossing_static_override_seen = False
    path_recovery_after_crossing = False
    goal_reached = False
    stalled = False
    timed_out = False

    while robot.step(TIME_STEP) != -1:
        now = robot.getTime()
        truth_position = node.getPosition()
        orientation = node.getOrientation()
        truth = (
            truth_position[0],
            truth_position[1],
            math.atan2(orientation[3], orientation[0]),
        )
        pose = odometry.update()
        max_position_error = max(
            max_position_error, math.dist(pose[:2], truth[:2])
        )

        if waypoint_index < len(WAYPOINTS):
            distance = math.dist(pose[:2], WAYPOINTS[waypoint_index])
            if distance < 0.14:
                print(
                    f"[WAYPOINT] t={now:.2f} reached={waypoint_index + 1}/"
                    f"{len(WAYPOINTS)} role={WAYPOINT_ROLES[waypoint_index]}",
                    flush=True,
                )
                waypoint_index += 1
                best_distance = float("inf")
                progress_time = now
                if waypoint_index == len(WAYPOINTS):
                    goal_reached = math.dist(truth[:2], WAYPOINTS[-1]) < 0.25

        contact_now = any(
            contact.getPoint()[2] > 0.01
            for contact in node.getContactPoints(True)
        )
        if contact_now and not touching:
            contacts += 1
            print(f"[CONTACT] t={now:.2f} pose={pose}", flush=True)
        touching = contact_now

        if waypoint_index < len(WAYPOINTS):
            measured_distance = math.dist(truth[:2], WAYPOINTS[waypoint_index])
            if measured_distance < best_distance - 0.04:
                best_distance = measured_distance
                progress_time = now
        stalled = waypoint_index < len(WAYPOINTS) and now - progress_time > STALL_SECONDS
        timed_out = now > MAX_SECONDS

        if goal_reached or stalled or timed_out or contacts:
            io.stop()
            expected_flow = (
                static_override_seen
                and crossing_yield_count > 0
                and crossing_gap_count > 0
                and crossing_commit_count > 0
                and crossing_complete_count > 0
                and path_recovery_after_crossing
                and post_crossing_static_override_seen
            )
            passed = (
                goal_reached
                and waypoint_index == len(WAYPOINTS)
                and contacts == 0
                and not stalled
                and not timed_out
                and expected_flow
            )
            result = {
                "planner": PLANNER,
                "completed": passed,
                "goal_reached": goal_reached,
                "contacts": contacts,
                "stalled": stalled,
                "timed_out": timed_out,
                "seconds": now,
                "reached_waypoints": waypoint_index,
                "total_waypoints": len(WAYPOINTS),
                "avoidance_override_count": avoidance_override_count,
                "avoidance_override_events": avoidance_override_events,
                "crossing_yield_count": crossing_yield_count,
                "crossing_gap_count": crossing_gap_count,
                "crossing_commit_count": crossing_commit_count,
                "crossing_complete_count": crossing_complete_count,
                "crossing_cancel_count": crossing_cancel_count,
                "emergency_stop_count": emergency_stop_count,
                "path_recovery_count": path_recovery_count,
                "static_override_seen": static_override_seen,
                "post_crossing_static_override_seen": post_crossing_static_override_seen,
                "path_recovery_after_crossing": path_recovery_after_crossing,
                "position_error": math.dist(pose[:2], truth[:2]),
                "max_position_error": max_position_error,
                "planner_mean_ms": 1000.0 * compute_total / max(1, compute_steps),
                "planner_max_ms": 1000.0 * compute_max,
            }
            RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
            print("[RESULT] " + json.dumps(result), flush=True)
            robot.step(TIME_STEP)
            robot.simulationQuit(0 if passed else 1)
            break

        goal = WAYPOINTS[waypoint_index]
        ranges = io.get_lidar()
        local_goal = goal_in_robot_frame(goal, pose)
        started = time.perf_counter()
        if PLANNER == "v2":
            avoidance_command = planner.choose_action(
                ranges, local_goal, pose=pose, sim_time=now,
                waypoint_id=waypoint_index,
            )
            command = (*avoidance_command[:2], planner.control_label)
            if recorder is not None:
                recorder.record(now, pose, local_goal, ranges, waypoint_index,
                                avoidance_command, truth)
        else:
            avoidance_command = planner.choose_action(
                ranges, local_goal, pose=pose, sim_time=now
            )
            nominal_command = nominal_waypoint_command(local_goal)
            command = select_control_command(
                planner, nominal_command, avoidance_command
            )
        elapsed = time.perf_counter() - started
        compute_total += elapsed
        compute_max = max(compute_max, elapsed)
        compute_steps += 1

        left, right, control = command
        io.set_wheel_speed(left, right)
        action = avoidance_command[2]
        is_avoid = control.startswith("AVOID")
        if is_avoid:
            avoidance_override_count += 1
            if previous_control is None or not previous_control.startswith("AVOID"):
                avoidance_override_events += 1
            if pose[1] < -0.50:
                static_override_seen = True
            if pose[1] > 1.80:
                post_crossing_static_override_seen = True
        elif previous_control is not None and previous_control.startswith("AVOID"):
            path_recovery_count += 1
            print(f"[FLOW] PATH_FOLLOW_RESUMED t={now:.2f}", flush=True)
            if pose[1] > CROSSING_Y + 0.25:
                path_recovery_after_crossing = True

        if "횡단 장애물 안전 거리 대기" in action:
            action_kind = "yield"
        elif "crossing gap confirmation" in action:
            action_kind = "gap"
        elif "crossing commit" in action:
            action_kind = "commit"
        elif left == 0.0 and right == 0.0 and is_avoid:
            action_kind = "emergency_stop"
        else:
            action_kind = "other"

        if action_kind != previous_action_kind:
            if action_kind == "yield":
                crossing_yield_count += 1
                print(f"[FLOW] CROSSING_YIELD t={now:.2f}", flush=True)
            elif action_kind == "gap":
                crossing_gap_count += 1
                print(f"[FLOW] GAP_ACCEPTANCE_CONFIRMING t={now:.2f}", flush=True)
            elif action_kind == "emergency_stop":
                emergency_stop_count += 1

        current_commit = planner.crossing_commit_active
        if current_commit and not previous_commit:
            crossing_commit_count += 1
            print(
                f"[FLOW] CROSSING_COMMIT t={now:.2f} "
                f"speed={planner.current_v:.2f}",
                flush=True,
            )
        elif previous_commit and not current_commit:
            if pose[1] > CROSSING_Y + 0.20:
                crossing_complete_count += 1
                print(f"[FLOW] CROSSING_COMPLETE t={now:.2f}", flush=True)
            else:
                crossing_cancel_count += 1
                print(f"[FLOW] CROSSING_CANCEL t={now:.2f}", flush=True)

        previous_commit = current_commit
        previous_control = control
        previous_action_kind = action_kind

        if now - last_print >= 1.0:
            print(
                f"[RUN] t={now:.2f} waypoint={waypoint_index} "
                f"role={WAYPOINT_ROLES[waypoint_index]} "
                f"distance={math.dist(pose[:2], goal):.2f} pose={pose} "
                f"control={control} action={action}",
                flush=True,
            )
            last_print = now


if __name__ == "__main__":
    from controller import Supervisor

    main()
