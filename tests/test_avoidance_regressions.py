"""Regressions for the failure modes found while replaying Webots traces.

Each test pins one general mechanism (not a waypoint or a world object):
LiDAR angle model, turn-side symmetry, idle rotation, recovery re-entry,
path-follow release, translation phantoms, blocked backoff and goals that
sit next to an obstacle.
"""

import math
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from avoidance import DynamicWindowAvoidance, _FORWARD_SPEED_THRESHOLD
from config import (
    DWA_BACKOFF_TRIGGER_DISTANCE,
    DWA_IDLE_PENALTY,
    DWA_LINEAR_ACCELERATION,
    DWA_PATH_RELEASE_CONFIRM_STEPS,
    DWA_RECOVERY_TRIGGER_DISTANCE,
    DWA_ROBOT_RADIUS,
    DWA_STATIC_CLEARANCE_MARGIN,
    LIDAR_FIELD_OF_VIEW,
    TIME_STEP,
)
from tools.diagnose_front_corner_stall import lidar_scan


DT = TIME_STEP / 1000.0


def boxes_scan(pose, boxes):
    """Nearest return over several boxes (harness ray caster per box)."""
    scans = [lidar_scan(pose, box) for box in boxes]
    return [min(values) for values in zip(*scans)]


class TestLidarAngleModel(unittest.TestCase):
    def test_full_turn_does_not_repeat_the_first_ray(self):
        # Measured in Webots with the practice robot: 180 rays over a full
        # turn, index 90 straight ahead and index 135 exactly left.
        step = DynamicWindowAvoidance._beam_angle_step(LIDAR_FIELD_OF_VIEW, 180)
        self.assertAlmostEqual(2.0 * math.pi / 180, step, places=4)
        ranges = [float("inf")] * 180
        ranges[90] = 1.0
        ranges[135] = 1.5
        points = DynamicWindowAvoidance._scan_to_points(ranges, LIDAR_FIELD_OF_VIEW)
        self.assertAlmostEqual(1.0, points[0][0], places=3)
        self.assertAlmostEqual(0.0, points[0][1], places=3)
        self.assertAlmostEqual(0.0, points[1][0], places=3)
        self.assertAlmostEqual(1.5, points[1][1], places=3)

    def test_partial_field_of_view_spans_both_edges(self):
        self.assertAlmostEqual(
            math.pi / 2, DynamicWindowAvoidance._beam_angle_step(math.pi, 3)
        )


class TestIsolatedBodyReturn(unittest.TestCase):
    def test_single_ray_inside_the_footprint_does_not_freeze_the_planner(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[115:122] = [0.56] * 7
        ranges[112] = 0.067  # rendering artefact seen next to a view seam
        left, right, _ = planner.choose_action(ranges, (2.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        self.assertNotEqual((0.0, 0.0), (left, right))

    def test_real_close_surfaces_are_kept(self):
        ranges = [float("inf")] * 180
        ranges[100:106] = [0.12] * 6   # a surface spanning several rays
        ranges[50] = 0.30              # isolated but outside the footprint
        cleaned = DynamicWindowAvoidance._reject_isolated_body_returns(ranges)
        self.assertEqual(ranges, cleaned)


class TestTurnSideSymmetry(unittest.TestCase):
    def test_open_sides_turn_away_from_the_nearer_front_half(self):
        planner = DynamicWindowAvoidance()
        left_box = lidar_scan((0.0, 0.0, 0.0), (0.90, 0.275, 0.45, 0.45))
        right_box = lidar_scan((0.0, 0.0, 0.0), (0.90, -0.275, 0.45, 0.45))
        self.assertEqual(-1.0, planner._select_turn_sign(left_box))
        self.assertEqual(1.0, planner._select_turn_sign(right_box))

    def test_symmetric_front_uses_the_goal_side(self):
        planner = DynamicWindowAvoidance()
        centred = lidar_scan((0.0, 0.0, 0.0), (0.90, 0.0, 0.45, 0.45))
        planner._last_goal = (3.0, -0.65)
        self.assertEqual(-1.0, planner._select_turn_sign(centred))
        planner._last_goal = (3.0, 0.65)
        self.assertEqual(1.0, planner._select_turn_sign(centred))

    def test_scan_mirror_gives_mirrored_commands(self):
        box = (0.90, 0.275, 0.45, 0.45)
        mirrored_box = (0.90, -0.275, 0.45, 0.45)
        left = DynamicWindowAvoidance()
        right = DynamicWindowAvoidance()
        for frame in range(20):
            pose = (0.0, 0.0, 0.0)
            a = left.choose_action(lidar_scan(pose, box), (3.0, 0.0), pose, frame * DT)
            b = right.choose_action(lidar_scan(pose, mirrored_box), (3.0, 0.0), pose, frame * DT)
            self.assertAlmostEqual(a[0], b[1], places=9)
            self.assertAlmostEqual(a[1], b[0], places=9)


class TestZeroVelocityWindow(unittest.TestCase):
    def test_first_acceleration_step_counts_as_forward(self):
        # From standstill the window tops out at accel * dt; that step must
        # count as a forward candidate or every stop near an obstacle looks
        # like a zero-velocity corner trap.
        self.assertLess(_FORWARD_SPEED_THRESHOLD, DWA_LINEAR_ACCELERATION * DT)
        planner = DynamicWindowAvoidance()
        scan = lidar_scan((0.0, 0.0, 0.0), (0.90, 0.0, 0.45, 0.45))
        planner._rollout_best_candidate(
            (3.0, 0.0), planner._scan_with_dynamic_obstacles(scan, (0.0, 0.0, 0.0), 0.0),
            [p for p in planner._scan_to_points(scan)], [], True,
        )
        self.assertGreater(planner.debug_valid_forward_count, 0)
        self.assertFalse(planner._front_corner_escape_active)

    def test_turning_in_place_while_facing_the_goal_is_idle(self):
        trajectory_rotate = DynamicWindowAvoidance._simulate(0.0, 0.3)
        trajectory_forward = DynamicWindowAvoidance._simulate(0.012, 0.0)
        rotate = DynamicWindowAvoidance._sensor_score_terms(
            0.0, 0.3, trajectory_rotate, 0.5, (0.16, 0.0))
        forward = DynamicWindowAvoidance._sensor_score_terms(
            0.012, 0.0, trajectory_forward, 0.5, (0.16, 0.0))
        self.assertEqual(-DWA_IDLE_PENALTY, rotate["stop_penalty"])
        self.assertEqual(0.0, forward["stop_penalty"])

    def test_turning_toward_a_goal_behind_is_not_idle(self):
        rotate = DynamicWindowAvoidance._sensor_score_terms(
            0.0, 0.6, DynamicWindowAvoidance._simulate(0.0, 0.6), 0.5, (-1.0, 0.2))
        self.assertEqual(0.0, rotate["stop_penalty"])


class TestHeadroomAndGoalCaps(unittest.TestCase):
    def test_headroom_penalises_only_near_the_margin(self):
        margin = DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
        self.assertEqual(0.0, DynamicWindowAvoidance._static_headroom_term(1.0))
        self.assertLess(DynamicWindowAvoidance._static_headroom_term(margin + 0.01), -0.9)

    def test_goal_next_to_an_obstacle_caps_headroom_and_triggers(self):
        wall = [(0.45, y / 20.0) for y in range(-10, 11)]
        near_goal = (0.30, 0.0)   # 0.15 m in front of the wall
        far_goal = (3.0, 0.0)
        approach = DynamicWindowAvoidance._goal_approach_clearance(near_goal, wall)
        self.assertAlmostEqual(0.15, approach, places=6)
        margin = DWA_ROBOT_RADIUS + DWA_STATIC_CLEARANCE_MARGIN
        self.assertAlmostEqual(
            margin,
            DynamicWindowAvoidance._goal_capped_trigger(DWA_BACKOFF_TRIGGER_DISTANCE, approach),
        )
        far = DynamicWindowAvoidance._goal_approach_clearance(far_goal, wall)
        self.assertEqual(
            DWA_RECOVERY_TRIGGER_DISTANCE,
            DynamicWindowAvoidance._goal_capped_trigger(DWA_RECOVERY_TRIGGER_DISTANCE, far),
        )


class TestRecoveryAndRelease(unittest.TestCase):
    def test_recovery_escape_latches_until_the_front_arc_clears(self):
        planner = DynamicWindowAvoidance()
        planner._leave_recovery_for_dwa()
        box = (0.62, 0.0, 0.45, 0.45)  # front at ~0.40 m (< recovery trigger)
        scan = lidar_scan((0.0, 0.0, 0.0), box)
        planner.choose_action(scan, (3.0, -0.8), (0.0, 0.0, 0.0), 0.0)
        self.assertIsNone(planner.recovery_phase)
        self.assertTrue(planner._recovery_reentry_latched)
        planner.choose_action([float("inf")] * 180, (3.0, -0.8), (0.0, 0.0, 0.0), DT)
        self.assertFalse(planner._recovery_reentry_latched)

    def test_release_waits_until_the_goal_line_clears_the_obstacle(self):
        planner = DynamicWindowAvoidance()
        # Robot beside a box; a rotate-in-place nominal is safe on its own,
        # but the line to the goal runs through the box.
        pose = (0.0, 0.0, 0.0)
        scan = lidar_scan(pose, (0.60, 0.0, 0.45, 0.45))
        planner.choose_action(scan, (2.0, 0.0), pose, 0.0)
        # Isolate the goal-line gate from the distance-triggered recovery.
        planner.recovery_phase = None
        planner.backoff_steps = 0
        planner._front_corner_escape_active = False
        planner._last_rollout_had_no_safe_candidate = False
        planner._wall_escape_cooldown = 0.0
        rotate_left, rotate_right = DynamicWindowAvoidance._wheel_speeds(0.0, 1.2)
        self.assertTrue(planner.is_command_safe(rotate_left, rotate_right))
        planner._path_release_streak = 0
        for _ in range(DWA_PATH_RELEASE_CONFIRM_STEPS + 2):
            self.assertFalse(planner.is_release_ready(rotate_left, rotate_right))
        planner._last_goal = (0.0, 2.0)  # goal now beside the box, line clear
        released = False
        for _ in range(DWA_PATH_RELEASE_CONFIRM_STEPS):
            released = planner.is_release_ready(rotate_left, rotate_right)
        self.assertTrue(released)

    def test_blocked_backoff_yields_to_safe_forward_rollout(self):
        # Front return inside the backoff trigger, straight rear sector open,
        # but every reverse arc passes the static margin: backing off is not
        # executable, so the planner must not STOP forever in the backoff
        # branch while a safe forward rollout exists.
        planner = DynamicWindowAvoidance()
        pose = (0.0, 0.0, 0.0)
        boxes = (
            (0.32, 0.0, 0.10, 0.60),       # thin wall ahead: front 0.27 m
            (-0.20, 0.315, 0.20, 0.20),    # rear-left post, 0.237 m away
            (-0.20, -0.315, 0.20, 0.20),   # rear-right post, 0.237 m away
        )
        scan = boxes_scan(pose, boxes)
        self.assertLess(planner._front_distance(scan), DWA_BACKOFF_TRIGGER_DISTANCE)
        self.assertEqual(float("inf"), planner._rear_distance(scan))
        results = [
            planner.choose_action(scan, (3.0, -1.5), pose, index * DT)
            for index in range(80)
        ]
        stops = sum(1 for left, right, _ in results if left == 0.0 and right == 0.0)
        self.assertLess(stops, len(results))
        self.assertEqual(0, planner.backoff_steps)


class TestTranslationPhantom(unittest.TestCase):
    def test_static_box_passed_at_speed_is_not_tracked_as_moving(self):
        planner = DynamicWindowAvoidance()
        box = (1.0, 0.45, 0.45, 0.45)
        promoted = False
        for frame in range(60):
            pose = (0.16 * frame * DT, 0.0, 0.0)
            obstacles = planner._scan_with_dynamic_obstacles(
                lidar_scan(pose, box), pose, frame * DT)
            promoted = promoted or any(len(point) == 4 for point in obstacles)
        self.assertFalse(promoted)

    def test_crossing_box_is_still_tracked(self):
        planner = DynamicWindowAvoidance()
        tracked = False
        for frame in range(40):
            box = (1.0, 0.8 - 0.30 * frame * DT, 0.28, 0.28)
            obstacles = planner._scan_with_dynamic_obstacles(
                lidar_scan((0.0, 0.0, 0.0), box), (0.0, 0.0, 0.0), frame * DT)
            tracked = tracked or any(len(point) == 4 for point in obstacles)
        self.assertTrue(tracked)


if __name__ == "__main__":
    unittest.main()
