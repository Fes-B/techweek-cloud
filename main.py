"""Webots practice adapter for the sensor based local avoidance controller."""

import math
import os

from avoidance import DynamicWindowAvoidance
from config import (
    PRACTICE_PATH_HEADING_GAIN,
    PRACTICE_PATH_SPEED,
    PRACTICE_PATH_TURN_RATE,
    PRACTICE_PATH_TURN_THRESHOLD,
    TIME_STEP,
)
from robot_io import RobotIO
from vision import TargetDetector


# Demonstration global path around the known static walls. The local planner
# receives only the next goal; another team can provide that goal from A*.
PRACTICE_WAYPOINTS = (
    (-1.45, 1.45), (1.45, 1.45), (1.45, -1.45),
    (-1.45, -1.45), (-1.45, 0.0),
)


# DIAGNOSTIC ONLY: VISION_FRAME_LOG=1 logs every frame in which a target is
# detected, for verifying detection across image positions and distances.
VISION_FRAME_LOG = os.environ.get("VISION_FRAME_LOG") == "1"


def describe_target(target):
    """One-line summary of a TargetDetector result, for the practice log."""
    return (
        f"shape={target['shape']} "
        f"center=({target['center_x']:.0f},{target['center_y']:.0f}) "
        f"offset=({target['offset_x']:+.3f},{target['offset_y']:+.3f}) "
        f"bbox={target['bbox']} area={target['area']:.0f} "
        f"area_ratio={target['area_ratio']:.5f} "
        f"confidence={target['confidence']:.3f} clipped={target['clipped']}"
    )


def goal_in_robot_frame(goal, pose):
    """Convert a global path waypoint into the avoidance module's input."""
    x, y, heading = pose
    dx, dy = goal[0] - x, goal[1] - y
    cosine, sine = math.cos(heading), math.sin(heading)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def nominal_waypoint_command(local_goal):
    """Practice-only nominal command; production waypoints come from Planning."""
    distance = math.hypot(*local_goal)
    heading_error = math.atan2(local_goal[1], local_goal[0])
    if abs(heading_error) >= PRACTICE_PATH_TURN_THRESHOLD:
        velocity = 0.0
        yaw_rate = math.copysign(PRACTICE_PATH_TURN_RATE, heading_error)
    else:
        velocity = min(PRACTICE_PATH_SPEED, distance)
        yaw_rate = PRACTICE_PATH_HEADING_GAIN * heading_error
    return DynamicWindowAvoidance._wheel_speeds(velocity, yaw_rate)


def select_control_command(avoidance, nominal_command, avoidance_command):
    """Select the path command only when the latest LiDAR scan permits it.

    Uses is_release_ready() rather than a bare is_command_safe() check: a
    single-frame safety flicker right at an obstacle corner must not hand
    control back to the path follower immediately, or the two arbitrate back
    and forth every frame without the robot ever clearing the corner. See
    DynamicWindowAvoidance.is_release_ready() for the hysteresis this adds.
    """
    nominal_left, nominal_right = nominal_command
    if (
        not avoidance.crossing_commit_active
        and avoidance.is_release_ready(nominal_left, nominal_right)
    ):
        return nominal_left, nominal_right, "PATH_FOLLOW"
    avoid_left, avoid_right, action = avoidance_command
    stopped = avoid_left == 0.0 and avoid_right == 0.0
    if (
        not math.isfinite(avoid_left)
        or not math.isfinite(avoid_right)
        or (not stopped and not avoidance.is_command_safe(avoid_left, avoid_right))
    ):
        return 0.0, 0.0, "AVOID FINAL SAFETY STOP"
    return avoid_left, avoid_right, f"AVOID {action.split(' v=', 1)[0]}"


def main():
    from controller import Supervisor

    robot = Supervisor()
    robot_io = RobotIO(robot)
    avoidance = DynamicWindowAvoidance()
    detector = TargetDetector()
    robot_node = robot.getSelf()
    robot_node.enableContactPointsTracking(TIME_STEP)
    waypoint_index = 0
    last_print_time = -1.0
    in_contact = False
    last_control_mode = None
    target_visible = False
    print("[DWA] LiDAR 센서 기반 이동 장애물 회피 시작")

    while robot.step(TIME_STEP) != -1:
        position = robot_node.getPosition()
        orientation = robot_node.getOrientation()
        heading = math.atan2(orientation[3], orientation[0])
        pose = (position[0], position[1], heading)
        goal = PRACTICE_WAYPOINTS[waypoint_index]
        if math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < 0.10:
            if waypoint_index == len(PRACTICE_WAYPOINTS) - 1:
                robot_io.stop()
                print(f"[DEMO] COMPLETE t={robot.getTime():.2f}")
                robot.simulationQuit(0)
                break
            waypoint_index += 1
            goal = PRACTICE_WAYPOINTS[waypoint_index]
            print(f"[GOAL] t={robot.getTime():.2f} next={waypoint_index}")

        ranges = robot_io.get_lidar()
        local_goal = goal_in_robot_frame(goal, pose)
        avoidance_left, avoidance_right, action = avoidance.choose_action(
            ranges, local_goal, pose=pose, sim_time=robot.getTime()
        )
        nominal_left, nominal_right = nominal_waypoint_command(local_goal)
        left, right, control_mode = select_control_command(
            avoidance,
            (nominal_left, nominal_right),
            (avoidance_left, avoidance_right, action),
        )
        robot_io.set_wheel_speed(left, right)

        target = detector.detect(robot_io.get_camera_bgr())
        if VISION_FRAME_LOG and target["detected"]:
            print(
                f"[VDET] t={robot.getTime():.3f} "
                f"pose=({position[0]:.2f},{position[1]:.2f},{heading:.3f}) "
                + describe_target(target),
                flush=True,
            )
        if control_mode != last_control_mode:
            print(f"[CONTROL] {control_mode}")
            last_control_mode = control_mode
        if target["detected"] and not target_visible:
            print("[VISION] TARGET DETECTED " + describe_target(target))
        elif target_visible and not target["detected"]:
            print(f"[VISION] TARGET LOST t={robot.getTime():.2f}")
        target_visible = target["detected"]

        # Contact tracking is diagnostic only and never enters the controller.
        contact_now = any(
            contact.getPoint()[2] > 0.08
            for contact in robot_node.getContactPoints()
        )
        if contact_now and not in_contact:
            print(f"[CONTACT] t={robot.getTime():.2f} pose={position}")
        in_contact = contact_now

        if robot.getTime() - last_print_time >= 0.5:
            distances = robot_io.get_lidar_directions(ranges)
            if distances is None:
                print("[LiDAR] unavailable")
            else:
                print(
                    f"[LiDAR] front={distances['front']:.2f} "
                    f"left={distances['left']:.2f} right={distances['right']:.2f} "
                    f"action={action}"
                )
            if target["detected"]:
                print("[VISION] " + describe_target(target))
            print("[POSE]", robot.getTime(), position)
            last_print_time = robot.getTime()


if __name__ == "__main__":
    main()
