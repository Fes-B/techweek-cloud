"""Standalone reproduction harness for the slalom static-corner oscillation.

Builds a synthetic LiDAR scan for the exact frozen pose observed in the
Webots run (robot sitting ~0.27 m west of slalom obstacle 1, heading
oscillating around 0.9 rad) and drives DynamicWindowAvoidance.choose_action()
directly, frame by frame, printing every internal decision variable the
investigation needs. No Webots required.
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from avoidance import DynamicWindowAvoidance
from config import (
    TIME_STEP,
    DWA_TURN_SWITCH_MARGIN,
    DWA_RECOVERY_TRIGGER_DISTANCE,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)
from main import goal_in_robot_frame, nominal_waypoint_command, select_control_command


def wheel_speeds_to_vw(left, right):
    """Inverse of DynamicWindowAvoidance._wheel_speeds."""
    v = WHEEL_RADIUS * (left + right) / 2.0
    w = WHEEL_RADIUS * (right - left) / WHEEL_TRACK
    return v, w

# Obstacle: slalom obstacle 1, world frame.
OBSTACLE_CENTER = (-1.20, -6.825)
OBSTACLE_HALF = 0.225

# Current waypoint target: "slalom1 past obstacle 1".
GOAL = (-0.85, -6.30)

# The frozen pose observed in the Webots log (t=66..88s), pose barely moving.
POSE = (-1.6903600858006922, -6.822809856091514, 0.90)

LIDAR_COUNT = 180
LIDAR_FOV = 2.0 * math.pi
LIDAR_MIN = 0.05
LIDAR_MAX = 4.0


def ray_box_distance(local_angle, robot_pose, box_center, box_half):
    """Distance from the robot to the box along a ray at robot-local angle."""
    x, y, heading = robot_pose
    world_angle = heading + local_angle
    dx, dy = math.cos(world_angle), math.sin(world_angle)
    # Slab method against an axis-aligned box.
    tmin, tmax = 0.0, LIDAR_MAX
    for origin, direction, center in ((x, dx, box_center[0]), (y, dy, box_center[1])):
        lo = center - box_half
        hi = center + box_half
        if abs(direction) < 1e-9:
            if origin < lo or origin > hi:
                return float("inf")
            continue
        t1 = (lo - origin) / direction
        t2 = (hi - origin) / direction
        if t1 > t2:
            t1, t2 = t2, t1
        tmin = max(tmin, t1)
        tmax = min(tmax, t2)
        if tmin > tmax:
            return float("inf")
    if tmin < LIDAR_MIN:
        return float("inf")
    return tmin if tmin <= LIDAR_MAX else float("inf")


def build_scan(robot_pose):
    ranges = []
    denom = max(1, LIDAR_COUNT - 1)
    for i in range(LIDAR_COUNT):
        local_angle = -0.5 * LIDAR_FOV + LIDAR_FOV * i / denom
        d = ray_box_distance(local_angle, robot_pose, OBSTACLE_CENTER, OBSTACLE_HALF)
        ranges.append(d)
    return ranges


def main():
    planner = DynamicWindowAvoidance(lidar_field_of_view=LIDAR_FOV)
    pose = POSE
    sim_time = 0.0
    dt = TIME_STEP / 1000.0

    print(f"DWA_TURN_SWITCH_MARGIN = {DWA_TURN_SWITCH_MARGIN if 'DWA_TURN_SWITCH_MARGIN' in dir() else 'N/A'}")
    print(f"pose={pose} goal={GOAL} obstacle_center={OBSTACLE_CENTER} half={OBSTACLE_HALF}")
    dist_to_center = math.hypot(pose[0] - OBSTACLE_CENTER[0], pose[1] - OBSTACLE_CENTER[1])
    print(f"distance robot->obstacle center = {dist_to_center:.3f} m "
          f"(box half-diagonal ~= {OBSTACLE_HALF*math.sqrt(2):.3f} m)")
    print()

    prev_recovery_turn_sign = None
    prev_wall_side = None
    prev_wall_front_turn_sign = None
    prev_control = None

    header = (
        f"{'t':>6} {'x':>7} {'y':>8} {'hdg':>6} {'cmd_v':>6} {'cmd_w':>6} "
        f"{'int_v':>6} {'int_w':>6} {'front':>6} {'phase':>6} "
        f"{'turn_sign':>9} {'wall_side':>9} {'wfront':>7} {'backoff':>7} "
        f"{'crossW':>6} {'commit':>6} {'dyn_n':>5} {'control':>10} {'action'}"
    )
    print(header)
    print(f"DWA_RECOVERY_TRIGGER_DISTANCE = {DWA_RECOVERY_TRIGGER_DISTANCE}")

    for step in range(400):
        ranges = build_scan(pose)
        local_goal = goal_in_robot_frame(GOAL, pose)
        avoidance_command = planner.choose_action(
            ranges, local_goal, pose=pose, sim_time=sim_time
        )
        nominal_command = nominal_waypoint_command(local_goal)
        left, right, control = select_control_command(
            planner, nominal_command, avoidance_command
        )
        action = avoidance_command[2]
        cmd_v, cmd_w = wheel_speeds_to_vw(left, right)

        recovery_turn_sign = planner.recovery_turn_sign
        wall_side = planner.wall_side
        wall_front_turn_sign = planner.wall_front_turn_sign
        nominal_left, nominal_right = nominal_command
        nominal_safe = planner.is_command_safe(nominal_left, nominal_right)
        front_distance = planner._front_distance(ranges)

        print(
            f"{sim_time:6.2f} {pose[0]:7.3f} {pose[1]:8.3f} {pose[2]:6.2f} "
            f"{cmd_v:6.2f} {cmd_w:6.2f} "
            f"{planner.current_v:6.2f} {planner.current_w:6.2f} "
            f"{front_distance:6.2f} "
            f"{str(planner.recovery_phase):>6} {str(recovery_turn_sign):>9} "
            f"{str(wall_side):>9} {str(wall_front_turn_sign):>7} "
            f"{planner.backoff_steps:7d} {str(planner.crossing_waiting):>6} "
            f"{str(planner.crossing_commit_active):>6} "
            f"{len(planner.last_dynamic_obstacles):5d} {control:>10} {action}"
        )

        if recovery_turn_sign != prev_recovery_turn_sign:
            print(
                f"  >> recovery_turn_sign CHANGED: {prev_recovery_turn_sign} -> "
                f"{recovery_turn_sign}"
            )
        if wall_side != prev_wall_side:
            print(f"  >> wall_side CHANGED: {prev_wall_side} -> {wall_side}")
        if wall_front_turn_sign != prev_wall_front_turn_sign:
            print(
                f"  >> wall_front_turn_sign CHANGED: {prev_wall_front_turn_sign} -> "
                f"{wall_front_turn_sign}"
            )
        if control != prev_control:
            print(
                f"  >> control CHANGED: {prev_control} -> {control} "
                f"(nominal_safe={nominal_safe}, nominal=({nominal_left:.2f},{nominal_right:.2f}), "
                f"final=({left:.2f},{right:.2f}))"
            )

        prev_recovery_turn_sign = recovery_turn_sign
        prev_wall_side = wall_side
        prev_wall_front_turn_sign = wall_front_turn_sign
        prev_control = control

        # Integrate pose with the actual chosen (v, w), unicycle model --
        # matching what EncoderPose/Webots physics would do. This is what
        # was missing from the first pass (frozen pose hid the heading
        # feedback loop that the real run's oscillation depends on).
        px, py, ph = pose
        px += cmd_v * math.cos(ph) * dt
        py += cmd_v * math.sin(ph) * dt
        ph += cmd_w * dt
        pose = (px, py, ph)
        sim_time += dt


if __name__ == "__main__":
    main()
