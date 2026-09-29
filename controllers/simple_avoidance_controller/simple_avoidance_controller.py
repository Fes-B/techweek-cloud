"""Controller for the deterministic minimal Webots static-avoidance world."""

import json
import math
import os
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import TIME_STEP
from main import goal_in_robot_frame
from robot_io import RobotIO
from simple_avoidance import NORMAL_DWA, ProgressWatchdogDWA


CASES = {
    "front_left": {"obstacle": (0.90, 0.275, 0.25), "goal": (3.20, 0.0)},
    "front_right": {"obstacle": (0.90, -0.275, 0.25), "goal": (3.20, 0.0)},
    "front": {"obstacle": (0.90, 0.0, 0.25), "goal": (3.20, 0.0)},
    "corridor": {"obstacle": None, "goal": (3.20, 0.0)},
}
TIMEOUT_SECONDS = 50.0


def _set_translation(supervisor, def_name, translation):
    node = supervisor.getFromDef(def_name)
    if node is None:
        raise RuntimeError(f"missing DEF {def_name}")
    node.getField("translation").setSFVec3f(list(translation))


def _heading(robot_node):
    orientation = robot_node.getOrientation()
    return math.atan2(orientation[3], orientation[0])


def main(robot):
    case_name = os.environ.get("SIMPLE_DWA_CASE", "front_left")
    if case_name not in CASES:
        raise ValueError(f"unknown SIMPLE_DWA_CASE={case_name!r}")
    case = CASES[case_name]
    inactive = (30.0, 30.0, 0.25)
    _set_translation(robot, "CASE_OBSTACLE", case["obstacle"] or inactive)
    corridor_active = case_name == "corridor"
    _set_translation(
        robot, "CORRIDOR_LEFT", (1.55, 0.60, 0.30) if corridor_active else inactive
    )
    _set_translation(
        robot, "CORRIDOR_RIGHT", (1.55, -0.60, 0.30) if corridor_active else inactive
    )

    io = RobotIO(robot)
    planner = ProgressWatchdogDWA()
    robot_node = robot.getSelf()
    robot_node.enableContactPointsTracking(TIME_STEP)
    goal = case["goal"]
    start = tuple(robot_node.getPosition()[:2])
    contacts = 0
    minimum_goal_distance = math.dist(start, goal)
    maximum_x = start[0]
    last_log = -1.0
    result_path = Path(
        tempfile.gettempdir(), f"techweek-simple-dwa-{case_name}.json"
    )
    result_path.unlink(missing_ok=True)

    while robot.step(TIME_STEP) != -1:
        now = robot.getTime()
        position = robot_node.getPosition()
        pose = (position[0], position[1], _heading(robot_node))
        local_goal = goal_in_robot_frame(goal, pose)
        ranges = io.get_lidar()
        left, right, action = planner.choose_action(
            ranges, local_goal, pose=pose, sim_time=now
        )
        io.set_wheel_speed(left, right)

        contacts += int(bool(robot_node.getContactPoints()))
        goal_distance = math.dist(pose[:2], goal)
        minimum_goal_distance = min(minimum_goal_distance, goal_distance)
        maximum_x = max(maximum_x, pose[0])
        goal_reached = goal_distance < 0.14
        timed_out = now >= TIMEOUT_SECONDS

        if now - last_log >= 0.5:
            print(
                "[SIMPLE] "
                f"case={case_name} t={now:.2f} pose={pose} goal={goal_distance:.3f} "
                f"state={planner.static_state} recoveries={planner.recovery_count} "
                f"window={planner.debug_v_window} forward={planner.debug_valid_forward_count} "
                f"improvement={planner.debug_goal_improvement:.3f} action={action}",
                flush=True,
            )
            last_log = now

        if goal_reached or timed_out:
            io.stop()
            obstacle_passed = maximum_x > 1.35 if case_name != "corridor" else maximum_x > 2.5
            result = {
                "case": case_name,
                "goal_reached": goal_reached,
                "timed_out": timed_out,
                "contacts": contacts,
                "position_progress": round(maximum_x - start[0], 6),
                "minimum_goal_distance": round(minimum_goal_distance, 6),
                "obstacle_passed": obstacle_passed,
                "path_following_returned": planner.static_state == NORMAL_DWA,
                "recovery_count": planner.recovery_count,
                "final_pose": [round(value, 6) for value in pose],
                "passed": (
                    goal_reached
                    and contacts == 0
                    and obstacle_passed
                    and planner.static_state == NORMAL_DWA
                ),
            }
            result_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
            print(f"[RESULT] {json.dumps(result, sort_keys=True)}", flush=True)
            robot.simulationQuit(0 if result["passed"] else 1)
            return


if __name__ == "__main__":
    from controller import Supervisor

    main(Supervisor())
