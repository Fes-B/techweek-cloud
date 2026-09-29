"""Extended endurance/functional benchmark for worlds/avoidance_extended.wbt.

Independent from avoidance_practice_controller.py (the protected smoke test).
Covers a long, varied course -- static obstacles from both sides, a slalom,
a variable-width corridor, an S-curve, a walled U-turn, five independent
dynamic crossings (both directions, fast, slow, and a short-cycle repeated
one), a static+dynamic mixed zone, a LiDAR-corner phantom-track regression
check, a second slalom, and a final static obstacle -- to verify the DWA
avoidance stack stays reliable over many consecutive avoidance episodes
rather than a single one.
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

from avoidance import DynamicWindowAvoidance
from config import TIME_STEP, WHEEL_RADIUS, WHEEL_TRACK
from main import (
    goal_in_robot_frame,
    nominal_waypoint_command,
    select_control_command,
)
from practice_odometry import EncoderPose
from robot_io import RobotIO
from tools.avoidance_trace import TraceWriter


START_POSE = (-9.50, -6.50, 0.0)

WAYPOINTS = (
    (-9.30, -6.50),  # clear start
    (-6.80, -6.50),  # clear exit
    (-4.60, -5.85),  # static-left exit
    (-2.10, -7.15),  # static-right exit
    (-0.85, -6.30),  # slalom 1 past obstacle 1
    (2.65, -6.70),   # slalom 1 past obstacle 2
    (6.15, -6.30),   # slalom 1 past obstacle 3
    (6.50, -6.50),   # corridor approach
    (7.20, -6.50),   # corridor entrance (width 1.60)
    (7.20, -5.10),   # corridor wide-to-mid (width 1.25)
    (7.20, -3.75),   # corridor narrow entry (width 1.15)
    (7.20, -2.40),   # corridor narrow exit (width 1.40)
    (7.20, -1.00),   # corridor exit
    (6.00, -0.60),   # S-curve left turn
    (6.00, 0.80),    # S-curve straight
    (7.20, 1.60),    # S-curve right turn
    (8.30, 1.60),    # U-turn entry
    (8.30, 3.35),    # U-turn apex
    (6.80, 3.35),    # U-turn cross
    (6.80, 1.60),    # U-turn exit
    (4.00, 1.60),    # clear run west
    (1.00, 1.60),    # dynamic-A exit
    (-1.00, 1.60),   # normal gap
    (-3.50, 1.60),   # dynamic-B exit
    (-6.60, 0.95),   # phantom-regression exit
    (-8.00, 1.60),   # turn north entry
    (-8.60, 2.20),   # corridor2 entrance
    (-8.60, 3.20),   # dynamic-C exit
    (-8.60, 4.40),   # dynamic-D exit
    (-8.60, 5.20),   # corridor2 exit
    (-5.80, 7.10),   # mixed exit
    (-3.80, 6.50),   # dynamic-E exit
    (-2.15, 6.30),   # slalom 2 past obstacle 1
    (0.55, 6.70),    # slalom 2 past obstacle 2
    (2.95, 6.30),    # slalom 2 past obstacle 3
    (3.20, 6.50),    # final corridor entrance
    (5.70, 6.50),    # final corridor exit
    (7.45, 7.15),    # final-static exit
    (9.30, 6.50),    # GOAL
)

WAYPOINT_ROLES = (
    "clear start", "clear exit",
    "static-left exit", "static-right exit",
    "slalom1 past obstacle 1", "slalom1 past obstacle 2",
    "slalom1 past obstacle 3",
    "corridor approach", "corridor entrance", "corridor wide-to-mid",
    "corridor narrow entry", "corridor narrow exit", "corridor exit",
    "S-curve left turn", "S-curve straight", "S-curve right turn",
    "U-turn entry", "U-turn apex", "U-turn cross", "U-turn exit",
    "clear run west", "dynamic-A exit", "normal gap", "dynamic-B exit",
    "phantom-regression exit",
    "turn north entry", "corridor2 entrance", "dynamic-C exit",
    "dynamic-D exit", "corridor2 exit", "mixed exit", "dynamic-E exit",
    "slalom2 past obstacle 1", "slalom2 past obstacle 2",
    "slalom2 past obstacle 3",
    "final corridor entrance", "final corridor exit",
    "final-static exit", "GOAL",
)

# (name, first_waypoint_idx, last_waypoint_idx) -- contiguous, 0..len(WAYPOINTS)-1.
SCENARIOS = (
    ("SCENARIO_01_CLEAR", 0, 1),
    ("SCENARIO_02_STATIC_LEFT", 1, 2),
    ("SCENARIO_03_STATIC_RIGHT", 2, 3),
    ("SCENARIO_04_SLALOM", 3, 6),
    ("SCENARIO_05_CORRIDOR", 6, 12),
    ("SCENARIO_06_S_CURVE", 12, 15),
    ("SCENARIO_07_U_TURN", 15, 19),
    ("SCENARIO_08_DYNAMIC_A", 19, 21),
    ("SCENARIO_09_NORMAL_GAP", 21, 22),
    ("SCENARIO_10_DYNAMIC_B", 22, 23),
    ("SCENARIO_11_PHANTOM_REGRESSION", 23, 24),
    ("SCENARIO_12_CORRIDOR2_DYNAMIC_CD", 24, 29),
    ("SCENARIO_13_MIXED_STATIC_DYNAMIC", 29, 30),
    ("SCENARIO_14_DYNAMIC_E_REPEATED", 30, 31),
    ("SCENARIO_15_SLALOM2", 31, 34),
    ("SCENARIO_16_FINAL_CORRIDOR", 34, 36),
    ("SCENARIO_17_FINAL_STATIC", 36, 37),
    ("SCENARIO_18_GOAL", 37, 38),
)

RECOVERY_KEYWORDS = (
    "복구", "제자리 회전", "긴급 회피", "안전 경로 없음", "안전 후진",
)

MAX_SECONDS = 420.0
STALL_SECONDS = 35.0
TOTAL_LAPS = max(1, int(os.environ.get("EXTENDED_LAPS", "1")))
_debug_wp_env = os.environ.get("EXTENDED_DEBUG_WAYPOINT")
DEBUG_WAYPOINT = int(_debug_wp_env) if _debug_wp_env is not None else None
RESULT_PATH = Path(
    tempfile.gettempdir(), "techweek-avoidance-extended-result.json"
)

# Debug-only unlimited-time diagnostic (EXTENDED_UNLIMITED=1).  The official
# benchmark above (MAX_SECONDS, STALL_SECONDS, RESULT_PATH) is unchanged and
# stays the default.  The diagnostic removes the 420 s limit, detects only
# permanent stalls (no 0.04 m progress for DIAGNOSTIC_STALL_SECONDS), records
# where the official 35 s stall rule and the 420 s limit would have fired, and
# writes a separate report.  DIAGNOSTIC_CAP_SECONDS only guarantees the batch
# run terminates.
UNLIMITED_DIAGNOSTIC = os.environ.get("EXTENDED_UNLIMITED") == "1"
DIAGNOSTIC_STALL_SECONDS = float(os.environ.get("EXTENDED_DIAGNOSTIC_STALL", "180"))
DIAGNOSTIC_CAP_SECONDS = float(os.environ.get("EXTENDED_DIAGNOSTIC_CAP", "3600"))
DIAGNOSTIC_RESULT_PATH = Path(
    tempfile.gettempdir(), "techweek-avoidance-extended-unlimited-result.json"
)


def scenario_index_for_waypoint(waypoint_index):
    for scenario_i, (_, first_idx, last_idx) in enumerate(SCENARIOS):
        if first_idx < waypoint_index <= last_idx:
            return scenario_i
    return None


def classify_action(control, action, left, right):
    if left == 0.0 and right == 0.0:
        return "stop"
    if any(keyword in action for keyword in RECOVERY_KEYWORDS):
        return "recovery"
    if control.startswith("AVOID"):
        return "avoidance_override"
    return "path_follow"


def main():
    print(f"[PYTHON] {sys.version.split()[0]} {sys.executable}", flush=True)
    print(f"[LAPS] total_laps={TOTAL_LAPS}", flush=True)
    robot = Supervisor()
    io = RobotIO(robot)
    planner = DynamicWindowAvoidance(lidar_field_of_view=io.lidar.getFov())
    node = robot.getSelf()
    start_pose = START_POSE
    segment_start_index = 0
    segment = os.environ.get("EXTENDED_DIAGNOSTIC_START") if UNLIMITED_DIAGNOSTIC else None
    if segment:
        # Diagnostic only: start the unchanged course at a later waypoint to
        # look for further permanent stalls beyond a known blocked section.
        x, y, yaw, index = (float(value) for value in segment.split(","))
        start_pose = (x, y, yaw)
        segment_start_index = int(index)
        node.getField("translation").setSFVec3f([x, y, 0.04])
        node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, yaw])
        node.resetPhysics()
        print(f"[DIAG] segment start pose={start_pose} waypoint={segment_start_index}",
              flush=True)
    odometry = EncoderPose(robot, start_pose)
    node.enableContactPointsTracking(TIME_STEP, True)
    trace = TraceWriter()  # no-op unless AVOID_TRACE names an output file

    waypoint_index = segment_start_index
    current_lap = 1
    contacts = 0
    touching = False
    best_distance = float("inf")
    progress_time = robot.getTime()
    last_print = -1.0
    previous_control = None
    previous_action_kind = None
    previous_commit = False
    previous_state = None
    state_since = 0.0
    max_stop_seconds = 0.0
    max_recovery_seconds = 0.0
    commit_start_pose = None
    debug_prev_turn_dir = None
    debug_prev_control = None
    debug_switch_pose = None
    debug_switch_count = 0

    frame_counts = {"path_follow": 0, "avoidance_override": 0, "recovery": 0, "stop": 0}
    crossing_yield_count = 0
    crossing_gap_count = 0
    crossing_commit_count = 0
    crossing_complete_count = 0
    crossing_cancel_count = 0
    emergency_stop_count = 0
    dynamic_tracks_detected = 0
    had_dynamic_track = False
    laps_completed = 0
    goal_reached = False
    stalled = False
    timed_out = False
    official_stall_event = None
    official_timeout_waypoint = None

    scenario_stats = [
        {
            "name": name,
            "entered": False,
            "passed": False,
            "enter_time": None,
            "pass_time": None,
            "contacts": 0,
            "avoidance_override_count": 0,
            "stop_frames": 0,
            "recovery_frames": 0,
        }
        for name, _, _ in SCENARIOS
    ]

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

        if waypoint_index < len(WAYPOINTS):
            distance = math.dist(pose[:2], WAYPOINTS[waypoint_index])
            if distance < 0.14:
                print(
                    f"[WAYPOINT] t={now:.2f} lap={current_lap} "
                    f"reached={waypoint_index + 1}/{len(WAYPOINTS)} "
                    f"role={WAYPOINT_ROLES[waypoint_index]}",
                    flush=True,
                )
                for scenario_i, (_, _, last_idx) in enumerate(SCENARIOS):
                    if last_idx == waypoint_index:
                        scenario_stats[scenario_i]["passed"] = True
                        scenario_stats[scenario_i]["pass_time"] = now
                waypoint_index += 1
                best_distance = float("inf")
                progress_time = now
                if waypoint_index == len(WAYPOINTS):
                    goal_reached = math.dist(truth[:2], WAYPOINTS[-1]) < 0.25
                    if current_lap < TOTAL_LAPS and goal_reached:
                        print(f"[LAP] t={now:.2f} lap {current_lap} complete", flush=True)
                        laps_completed += 1
                        current_lap += 1
                        waypoint_index = 0
                        goal_reached = False
                        best_distance = float("inf")
                        progress_time = now
                        for stat in scenario_stats:
                            stat["entered"] = False
                            stat["passed"] = False

        active_scenario_i = scenario_index_for_waypoint(max(1, waypoint_index))
        if active_scenario_i is not None and not scenario_stats[active_scenario_i]["entered"]:
            scenario_stats[active_scenario_i]["entered"] = True
            scenario_stats[active_scenario_i]["enter_time"] = now

        contact_now = any(
            contact.getPoint()[2] > 0.01
            for contact in node.getContactPoints(True)
        )
        if contact_now and not touching:
            contacts += 1
            print(f"[CONTACT] t={now:.2f} pose={pose}", flush=True)
            if active_scenario_i is not None:
                scenario_stats[active_scenario_i]["contacts"] += 1
        touching = contact_now

        if waypoint_index < len(WAYPOINTS):
            measured_distance = math.dist(truth[:2], WAYPOINTS[waypoint_index])
            if measured_distance < best_distance - 0.04:
                best_distance = measured_distance
                progress_time = now
        official_stall = (
            waypoint_index < len(WAYPOINTS) and now - progress_time > STALL_SECONDS
        )
        if UNLIMITED_DIAGNOSTIC:
            if official_stall and official_stall_event is None:
                official_stall_event = (round(now, 3), waypoint_index)
                print(f"[DIAG] official 35 s stall rule would fire t={now:.2f} "
                      f"waypoint={waypoint_index}", flush=True)
            if now > MAX_SECONDS and official_timeout_waypoint is None:
                official_timeout_waypoint = waypoint_index
                print(f"[DIAG] official 420 s limit passed at waypoint={waypoint_index}",
                      flush=True)
            stalled = (
                waypoint_index < len(WAYPOINTS)
                and now - progress_time > DIAGNOSTIC_STALL_SECONDS
            )
            timed_out = now > DIAGNOSTIC_CAP_SECONDS
        else:
            stalled = official_stall
            timed_out = now > MAX_SECONDS

        if goal_reached or stalled or timed_out or contacts:
            io.stop()
            if goal_reached:
                laps_completed += 1
            scenario_passed = sum(1 for stat in scenario_stats if stat["passed"])
            passed = (
                goal_reached
                and waypoint_index == len(WAYPOINTS)
                and contacts == 0
                and not stalled
                and not timed_out
                and scenario_passed == len(SCENARIOS)
            )
            result = {
                "completed": passed,
                "goal_reached": goal_reached,
                "contacts": contacts,
                "stalled": stalled,
                "timed_out": timed_out,
                "seconds": now,
                "laps_completed": laps_completed,
                "total_laps": TOTAL_LAPS,
                "reached_waypoints": waypoint_index,
                "total_waypoints": len(WAYPOINTS),
                "scenario_passed": scenario_passed,
                "scenario_total": len(SCENARIOS),
                "path_follow_frames": frame_counts["path_follow"],
                "avoidance_override_frames": frame_counts["avoidance_override"],
                "recovery_frames": frame_counts["recovery"],
                "stop_frames": frame_counts["stop"],
                "crossing_yield_count": crossing_yield_count,
                "crossing_gap_count": crossing_gap_count,
                "crossing_commit_count": crossing_commit_count,
                "crossing_complete_count": crossing_complete_count,
                "crossing_cancel_count": crossing_cancel_count,
                "emergency_stop_count": emergency_stop_count,
                "dynamic_tracks_detected": dynamic_tracks_detected,
                "max_continuous_stop_seconds": max_stop_seconds,
                "max_recovery_seconds": max_recovery_seconds,
                "scenarios": [
                    {
                        "name": stat["name"],
                        "entered": stat["entered"],
                        "passed": stat["passed"],
                        "enter_time": stat["enter_time"],
                        "pass_time": stat["pass_time"],
                        "contacts": stat["contacts"],
                        "avoidance_override_count": stat["avoidance_override_count"],
                        "stop_frames": stat["stop_frames"],
                        "recovery_frames": stat["recovery_frames"],
                    }
                    for stat in scenario_stats
                ],
            }
            if UNLIMITED_DIAGNOSTIC:
                result = {
                    "mode": "unlimited_diagnostic",
                    **result,
                    "diagnostic_stall_seconds": DIAGNOSTIC_STALL_SECONDS,
                    "diagnostic_cap_reached": timed_out,
                    "stalled_waypoint": waypoint_index if stalled else None,
                    "official_stall_would_fire": official_stall_event,
                    "official_timeout_waypoint": official_timeout_waypoint,
                    "segment_start_index": segment_start_index,
                    "within_official_420s": goal_reached and now <= MAX_SECONDS,
                }
                DIAGNOSTIC_RESULT_PATH.write_text(
                    json.dumps(result, indent=2), encoding="utf-8")
            else:
                RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
            print("[RESULT] " + json.dumps(result), flush=True)
            robot.step(TIME_STEP)
            robot.simulationQuit(0 if passed else 1)
            break

        goal = WAYPOINTS[waypoint_index]
        ranges = io.get_lidar()
        local_goal = goal_in_robot_frame(goal, pose)
        avoidance_command = planner.choose_action(
            ranges, local_goal, pose=pose, sim_time=now
        )
        nominal_command = nominal_waypoint_command(local_goal)
        command = select_control_command(
            planner, nominal_command, avoidance_command
        )

        left, right, control = command
        io.set_wheel_speed(left, right)
        trace.record(now, pose, local_goal, ranges, command, io.lidar.getFov(), truth)
        action = avoidance_command[2]

        if DEBUG_WAYPOINT is not None and waypoint_index == DEBUG_WAYPOINT:
            static_n = sum(1 for o in planner.last_obstacles or () if len(o) == 2)
            dyn_n = sum(1 for o in planner.last_obstacles or () if len(o) == 4)
            front_distance = planner._front_distance(ranges)
            front_arc_distance = planner._front_arc_distance(ranges)
            left_distance, right_distance = planner._side_distances(ranges)
            nominal_left, nominal_right = nominal_command
            nominal_safe = planner.is_command_safe(nominal_left, nominal_right)
            best_is_none = planner.debug_best_vw is None
            control_mode = (
                "PATH_FOLLOW" if control == "PATH_FOLLOW" else
                "RECOVERY_TURN" if planner.recovery_phase in ("turn", "emergency_turn") else
                "WALL_FOLLOW" if planner.recovery_phase == "wall_follow" else
                "BACKOFF" if planner.backoff_steps > 0 else
                "STOP" if (left == 0.0 and right == 0.0) else
                "DWA"
            )
            turn_dir = None
            if abs(right) > 1e-6 or abs(left) > 1e-6:
                cmd_w = WHEEL_RADIUS * (right - left) / WHEEL_TRACK
                if cmd_w > 0.02:
                    turn_dir = "LEFT"
                elif cmd_w < -0.02:
                    turn_dir = "RIGHT"
                else:
                    turn_dir = "STRAIGHT"
            common_fields = (
                f"t={now:.3f} scenario=SCENARIO_04_SLALOM wp={waypoint_index} "
                f"pose=({pose[0]:.4f},{pose[1]:.4f},{pose[2]:.3f}) "
                f"goal_local=({local_goal[0]:.4f},{local_goal[1]:.4f}) "
                f"front={front_distance:.4f} front_arc={front_arc_distance:.4f} "
                f"left={left_distance:.4f} right={right_distance:.4f} "
                f"side_speed_limit={planner.side_speed_limit:.4f} "
                f"fixed_n={static_n} moving_n={dyn_n} "
                f"crossW={planner.crossing_waiting} commit={planner.crossing_commit_active} "
                f"recovery_phase={planner.recovery_phase} wall_side={planner.wall_side} "
                f"wall_escape={planner.wall_escape_active} backoff={planner.backoff_steps} "
                f"current_vw=({planner.current_v:.6f},{planner.current_w:.6f}) "
                f"v_window={planner.debug_v_window} w_window={planner.debug_w_window} "
                f"nominal=({nominal_left:.2f},{nominal_right:.2f}) nominal_safe={nominal_safe} "
                f"total_cand={planner.debug_total_candidates} "
                f"static_ok={planner.debug_static_ok_count} "
                f"dynamic_ok={planner.debug_dynamic_ok_count} "
                f"braking_ok={planner.debug_braking_ok_count} "
                f"best_vw={planner.debug_best_vw} "
                f"best_static_clr={planner.debug_best_static_clearance} "
                f"best_dynamic_clr={planner.debug_best_dynamic_clearance} "
                f"best_score={planner.debug_best_score} best_is_none={best_is_none} "
                f"valid_forward={planner.debug_valid_forward_count} "
                f"best_candidate={planner.debug_best_candidate} "
                f"best_forward={planner.debug_best_forward_candidate} "
                f"control_mode={control_mode} final=({left:.2f},{right:.2f})"
            )
            if turn_dir is not None and debug_prev_turn_dir is not None and turn_dir != debug_prev_turn_dir and "STRAIGHT" not in (turn_dir, debug_prev_turn_dir):
                print(
                    f"[TURN_SWITCH] old_dir={debug_prev_turn_dir} new_dir={turn_dir} "
                    f"old_control_mode={debug_prev_control} new_control_mode={control_mode} "
                    + common_fields,
                    flush=True,
                )
                debug_switch_count += 1
            if turn_dir is not None:
                debug_prev_turn_dir = turn_dir
            debug_prev_control = control_mode
            print("[DBG] " + common_fields, flush=True)

        state = classify_action(control, action, left, right)
        frame_counts[state] += 1
        if active_scenario_i is not None:
            if state == "avoidance_override":
                scenario_stats[active_scenario_i]["avoidance_override_count"] += 1
            elif state == "stop":
                scenario_stats[active_scenario_i]["stop_frames"] += 1
            elif state == "recovery":
                scenario_stats[active_scenario_i]["recovery_frames"] += 1

        if state == previous_state:
            state_since += TIME_STEP / 1000.0
        else:
            state_since = TIME_STEP / 1000.0
        if state == "stop":
            max_stop_seconds = max(max_stop_seconds, state_since)
        elif state == "recovery":
            max_recovery_seconds = max(max_recovery_seconds, state_since)
        previous_state = state

        has_dynamic_track = len(planner.last_dynamic_obstacles) > 0
        if has_dynamic_track and not had_dynamic_track:
            dynamic_tracks_detected += 1
        had_dynamic_track = has_dynamic_track

        if "횡단 장애물 안전 거리 대기" in action:
            action_kind = "yield"
        elif "crossing gap confirmation" in action:
            action_kind = "gap"
        elif left == 0.0 and right == 0.0 and control.startswith("AVOID"):
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
            commit_start_pose = pose
            print(
                f"[FLOW] CROSSING_COMMIT t={now:.2f} "
                f"speed={planner.current_v:.2f}",
                flush=True,
            )
        elif previous_commit and not current_commit:
            forward_progress = 0.0
            if commit_start_pose is not None:
                dx = pose[0] - commit_start_pose[0]
                dy = pose[1] - commit_start_pose[1]
                forward_progress = (
                    dx * math.cos(commit_start_pose[2])
                    + dy * math.sin(commit_start_pose[2])
                )
            if forward_progress >= 0.55:
                crossing_complete_count += 1
                print(f"[FLOW] CROSSING_COMPLETE t={now:.2f} progress={forward_progress:.2f}", flush=True)
            else:
                crossing_cancel_count += 1
                print(f"[FLOW] CROSSING_CANCEL t={now:.2f} progress={forward_progress:.2f}", flush=True)
            commit_start_pose = None

        previous_commit = current_commit
        previous_control = control
        previous_action_kind = action_kind

        if now - last_print >= 1.0:
            print(
                f"[RUN] t={now:.2f} lap={current_lap} waypoint={waypoint_index} "
                f"role={WAYPOINT_ROLES[waypoint_index]} "
                f"distance={math.dist(pose[:2], goal):.2f} pose={pose} "
                f"control={control} action={action}",
                flush=True,
            )
            last_print = now


if __name__ == "__main__":
    from controller import Supervisor

    main()
