import math
import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from avoidance import DynamicWindowAvoidance, _SCAN_RELEVANT_DISTANCE
from config import (
    DWA_ANGULAR_ACCELERATION,
    DWA_BACKOFF_MAX_SECONDS,
    DWA_DYNAMIC_MIN_SPEED,
    DWA_LINEAR_ACCELERATION,
    DWA_ROBOT_RADIUS,
    DWA_SIDE_CAUTION_SPEED,
    DWA_SIDE_CLEAR_STEPS,
    DWA_SIDE_YIELD_SECONDS,
    MAX_SPEED,
    TIME_STEP,
)
from config import DWA_SIDE_STOP_DISTANCE


class TestDynamicWindowAvoidance(unittest.TestCase):
    def test_moving_away_prediction_does_not_erase_current_occupancy(self):
        trajectory = DynamicWindowAvoidance._simulate(.24, 0.0)
        obstacle = [(.35, 0.0, 1.0, 0.0)]
        clearance = DynamicWindowAvoidance._dynamic_reachable_clearance(trajectory, obstacle)
        self.assertLess(clearance, DWA_ROBOT_RADIUS)

    def test_batch_clearance_matches_scalar_collision_checks(self):
        planner = DynamicWindowAvoidance()
        trajectories = [planner._simulate(v, w) for v, w in ((.1, .3), (.2, -.5), (0, 0))]
        fixed = [(1.0, .2), (.6, -.4)]
        moving = [(1.2, -.5, -.2, .3)]
        static, dynamic = planner._sensor_clearances(trajectories, fixed, moving)
        for index, trajectory in enumerate(trajectories):
            self.assertAlmostEqual(static[index], planner._trajectory_clearance(trajectory[:12], fixed))
            self.assertAlmostEqual(dynamic[index], planner._dynamic_reachable_clearance(trajectory, moving))

    def test_scan_respects_sensor_field_of_view_and_mount_offset(self):
        points = DynamicWindowAvoidance._scan_to_points(
            [1.0, 1.0, 1.0], math.pi, (0.2, 0.0, math.pi / 2))
        for actual, expected in zip(points, [(1.2, 0.0), (0.2, 1.0), (-0.8, 0.0)]):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])

    def test_path_follower_command_selection_uses_latest_scan(self):
        planner = DynamicWindowAvoidance()
        self.assertFalse(planner.is_command_safe(4.0, 4.0))
        clear = [float("inf")] * 180
        planner.choose_action(clear, pose=(0.0, 0.0, 0.0), sim_time=0.0)
        self.assertTrue(planner.is_command_safe(4.0, 4.0))
        self.assertFalse(planner.is_command_safe(float("nan"), 4.0))
        blocked = clear.copy()
        blocked[89:92] = [0.30] * 3
        planner.choose_action(blocked, pose=(0.0, 0.0, 0.0), sim_time=TIME_STEP / 1000.0)
        self.assertFalse(planner.is_command_safe(4.0, 4.0))

    def test_stops_without_lidar(self):
        planner = DynamicWindowAvoidance()
        self.assertEqual((0.0, 0.0), planner.choose_action(None)[:2])
        self.assertEqual((0.0, 0.0), planner.choose_action([])[:2])

    def test_nonfinite_scan_values_produce_finite_bounded_commands(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[30] = float("nan")
        ranges[90] = float("nan")

        left, right, _ = planner.choose_action(
            ranges, pose=(0.0, 0.0, 0.0), sim_time=0.0
        )

        self.assertTrue(math.isfinite(left) and math.isfinite(right))
        self.assertLessEqual(max(abs(left), abs(right)), MAX_SPEED)

    def test_sensor_mode_requires_pose_and_time_together(self):
        planner = DynamicWindowAvoidance()
        with self.assertRaises(ValueError):
            planner.choose_action([float("inf")] * 180, pose=(0.0, 0.0, 0.0))

    def test_accelerates_forward_in_clear_space(self):
        planner = DynamicWindowAvoidance()
        left, right, action = planner.choose_action([float("inf")] * 180)
        self.assertGreater(left, 0.0)
        self.assertAlmostEqual(left, right)
        self.assertIn("DWA", action)

    def test_sensor_mode_keeps_the_acceleration_limited_dynamic_window(self):
        planner = DynamicWindowAvoidance()
        planner.current_v = 0.10
        planner.current_w = 0.20
        planner.choose_action(
            [float("inf")] * 180,
            pose=(0.0, 0.0, 0.0),
            sim_time=0.0,
        )

        dt = TIME_STEP / 1000.0
        self.assertLessEqual(
            abs(planner.current_v - 0.10), DWA_LINEAR_ACCELERATION * dt + 1e-12
        )
        self.assertLessEqual(
            abs(planner.current_w - 0.20), DWA_ANGULAR_ACCELERATION * dt + 1e-12
        )

    def test_close_front_obstacle_starts_and_holds_turn(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[75:106] = [0.30] * 31

        first_left, first_right, _ = planner.choose_action(blocked)
        next_left, next_right, _ = planner.choose_action(blocked)

        self.assertNotEqual((0.0, 0.0), (first_left, first_right))
        self.assertEqual((first_left, first_right), (next_left, next_right))
        self.assertIsNotNone(planner.recovery_turn_sign)

    def test_emergency_turn_holds_for_off_center_obstacle_then_resumes(self):
        planner = DynamicWindowAvoidance()
        blocked_front_arc = [float("inf")] * 180
        blocked_front_arc[67] = 0.20

        first = planner.choose_action(blocked_front_arc)
        second = planner.choose_action(blocked_front_arc)

        self.assertEqual("emergency_turn", planner.recovery_phase)
        self.assertEqual(1.0, planner.recovery_turn_sign)
        self.assertEqual((0.0, 0.0), first[:2])
        self.assertEqual((0.0, 0.0), second[:2])
        self.assertIn("정지", first[2])
        self.assertIn("정지", second[2])

        obstacle_behind = [float("inf")] * 180
        obstacle_behind[64] = 0.80
        resumed = planner.choose_action(obstacle_behind)

        self.assertIsNone(planner.recovery_phase)
        self.assertIn("DWA v=", resumed[2])

    def test_front_arc_covers_oblique_hazards_without_treating_side_as_front(self):
        ranges = [float("inf")] * 180
        ranges[67] = 0.30
        self.assertEqual(0.30, DynamicWindowAvoidance._front_arc_distance(ranges))

        ranges[67] = float("inf")
        ranges[65] = 0.40
        self.assertEqual(0.40, DynamicWindowAvoidance._front_arc_distance(ranges))

        ranges[65] = float("inf")
        ranges[64] = 0.50
        self.assertEqual(float("inf"), DynamicWindowAvoidance._front_arc_distance(ranges))

    def test_recovery_ends_when_front_is_clear(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[75:106] = [0.30] * 31
        planner.choose_action(blocked)

        planner.choose_action([float("inf")] * 180)

        self.assertIsNone(planner.recovery_turn_sign)
        self.assertGreater(planner.current_v, 0.0)

    def test_recovery_turn_waits_until_the_front_arc_clears(self):
        planner = DynamicWindowAvoidance()
        blocked_front = [float("inf")] * 180
        blocked_front[85:96] = [0.30] * 11
        planner.choose_action(blocked_front)

        blocked_oblique = [float("inf")] * 180
        blocked_oblique[65] = 0.40
        left, right, action = planner.choose_action(blocked_oblique)

        self.assertEqual("turn", planner.recovery_phase)
        self.assertIn("저속 전진", action)
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)

        resumed = planner.choose_action([float("inf")] * 180)
        self.assertIsNone(planner.recovery_phase)
        self.assertIn("DWA v=", resumed[2])

    def test_backoff_finishes_into_recovery_instead_of_restarting_path_planning(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[90] = 0.20
        planner.choose_action(blocked)
        max_steps = math.ceil(DWA_BACKOFF_MAX_SECONDS * 1000.0 / TIME_STEP)

        for _ in range(max_steps):
            result = planner.choose_action(blocked)

        self.assertEqual("turn", planner.recovery_phase)
        self.assertIsNotNone(planner.recovery_turn_sign)
        self.assertEqual("DWA 후진 복구 완료", result[2])

    def test_backoff_curves_away_from_the_blocked_side(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[90] = 0.20
        # Keep the side obstacle outside the robot clearance envelope so the
        # expected curved retreat is physically valid.
        blocked[125:146] = [0.80] * 21
        blocked[35:56] = [1.20] * 21

        left, right, action = planner.choose_action(blocked)

        self.assertEqual(-1.0, planner.recovery_turn_sign)
        self.assertLess(left, 0.0)
        self.assertLess(right, 0.0)
        self.assertGreater(left, right)
        self.assertIn("후진 회피", action)

    def test_recovery_follows_adjacent_wall_after_front_clears(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[75:106] = [0.30] * 31
        planner.choose_action(blocked)

        clear_front_near_wall = [float("inf")] * 180
        clear_front_near_wall[130:141] = [0.86] * 11
        left, right, action = planner.choose_action(clear_front_near_wall)

        self.assertEqual("wall_follow", planner.recovery_phase)
        self.assertIn("벽 추종", action)
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)

    def test_wall_follow_releases_after_adjacent_wall_disappears(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[75:106] = [0.30] * 31
        planner.choose_action(blocked)
        clear_front_near_wall = [float("inf")] * 180
        clear_front_near_wall[130:141] = [0.86] * 11
        planner.choose_action(clear_front_near_wall)

        for _ in range(20):
            result = planner.choose_action([float("inf")] * 180)

        self.assertIsNone(planner.recovery_phase)
        self.assertGreater(planner.current_v, 0.0)
        self.assertIn("DWA v=", result[2])

    def test_wall_follow_switches_when_the_other_turn_opens(self):
        planner = DynamicWindowAvoidance()
        planner.wall_side = "left"
        blocked_front = [float("inf")] * 180
        blocked_front[85:96] = [0.30] * 11
        blocked_front[35:56] = [0.95] * 21
        blocked_front[125:146] = [1.80] * 21

        left, right, action = planner._follow_wall(blocked_front, 0.30)
        turn_sign = planner.wall_front_turn_sign
        blocked_front[35:56] = [1.80] * 21
        blocked_front[125:146] = [0.95] * 21
        next_left, next_right, _ = planner._follow_wall(blocked_front, 0.30)

        self.assertIn("저속 전진", action)
        self.assertEqual(1.0, turn_sign)
        self.assertLess(left, right)
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)
        self.assertEqual(-1.0, planner.wall_front_turn_sign)
        self.assertGreater(next_left, next_right)
        self.assertGreater(next_left, 0.0)
        self.assertGreater(next_right, 0.0)

        forward, _, resumed_action = planner._follow_wall(
            [float("inf")] * 180, float("inf")
        )
        self.assertIsNone(planner.wall_front_turn_sign)
        self.assertGreater(forward, 0.0)
        self.assertIn("벽 추종", resumed_action)

    def test_side_obstacle_limits_forward_speed(self):
        planner = DynamicWindowAvoidance()
        planner.current_v = 0.2
        ranges = [float("inf")] * 180
        ranges[35:56] = [DWA_SIDE_STOP_DISTANCE + 0.05] * 21

        planner.choose_action(ranges)

        self.assertGreater(planner.current_v, DWA_SIDE_CAUTION_SPEED)
        self.assertAlmostEqual(
            0.2 - DWA_LINEAR_ACCELERATION * TIME_STEP / 1000.0,
            planner.current_v,
        )

    def test_side_caution_speed_cap_is_reached_without_being_exceeded(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[35:56] = [DWA_SIDE_STOP_DISTANCE + 0.05] * 21

        for _ in range(40):
            planner.choose_action(ranges)

        self.assertAlmostEqual(DWA_SIDE_CAUTION_SPEED, planner.current_v)

    def test_close_side_obstacle_caps_speed_without_freezing_dwa(self):
        planner = DynamicWindowAvoidance()
        planner.current_v = DWA_SIDE_CAUTION_SPEED
        ranges = [float("inf")] * 180
        ranges[35:56] = [DWA_SIDE_STOP_DISTANCE - 0.01] * 21

        for _ in range(20):
            planner.choose_action(ranges)

        self.assertGreater(planner.current_v, 0.0)
        self.assertLessEqual(planner.current_v, DWA_SIDE_CAUTION_SPEED)

    def test_close_side_yield_is_bounded_for_static_obstacles(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[35:56] = [DWA_SIDE_STOP_DISTANCE - 0.01] * 21
        yield_steps = math.ceil(DWA_SIDE_YIELD_SECONDS * 1000.0 / TIME_STEP)

        for _ in range(yield_steps + 20):
            planner.choose_action(ranges)

        self.assertGreater(planner.current_v, 0.0)
        self.assertLessEqual(planner.current_v, DWA_SIDE_CAUTION_SPEED)

    def test_side_yield_applies_to_front_recovery_and_then_resumes(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[85:96] = [0.35] * 11
        ranges[35:56] = [DWA_SIDE_STOP_DISTANCE - 0.01] * 21

        stopped = planner.choose_action(ranges)
        self.assertNotEqual((0.0, 0.0), stopped[:2])
        self.assertGreater(planner.current_v, 0.0)
        self.assertLessEqual(planner.current_v, DWA_SIDE_CAUTION_SPEED)

        yield_steps = math.ceil(DWA_SIDE_YIELD_SECONDS * 1000.0 / TIME_STEP)
        for _ in range(yield_steps + 5):
            resumed = planner.choose_action(ranges)

        self.assertGreater(planner.current_v, 0.0)
        self.assertGreater(resumed[0], 0.0)
        self.assertGreater(resumed[1], 0.0)

    def test_one_clear_scan_does_not_cancel_active_side_yield(self):
        planner = DynamicWindowAvoidance()
        close_side = [float("inf")] * 180
        close_side[35:56] = [DWA_SIDE_STOP_DISTANCE - 0.01] * 21

        planner.choose_action(close_side)
        planner.choose_action([float("inf")] * 180)
        planner.choose_action(close_side)

        self.assertGreater(planner.side_yield_elapsed, 0.0)
        self.assertGreater(planner.current_v, 0.0)
        self.assertLessEqual(planner.current_v, DWA_SIDE_CAUTION_SPEED)
        self.assertLess(
            DWA_SIDE_CLEAR_STEPS,
            math.ceil(DWA_SIDE_YIELD_SECONDS * 1000.0 / TIME_STEP),
        )

    def test_distant_lidar_returns_do_not_change_wheel_command(self):
        near_only = [float("inf")] * 180
        near_only[50:60] = [0.8] * 10
        with_distant_returns = near_only.copy()
        with_distant_returns[110:140] = [_SCAN_RELEVANT_DISTANCE + 0.1] * 30

        near_action = DynamicWindowAvoidance().choose_action(near_only, (2.0, 0.0))
        distant_action = DynamicWindowAvoidance().choose_action(
            with_distant_returns, (2.0, 0.0)
        )

        self.assertEqual(near_action[:2], distant_action[:2])
        self.assertEqual(10, len(DynamicWindowAvoidance._scan_to_points(with_distant_returns)))

    def test_tracker_estimates_perpendicular_lidar_motion(self):
        planner = DynamicWindowAvoidance()
        obstacles = None
        for step, start in enumerate(range(94, 85, -1)):
            scan = [float("inf")] * 180
            scan[start:start + 7] = [1.0] * 7
            obstacles = planner._scan_with_dynamic_obstacles(
                scan, (0.0, 0.0, 0.0), step * TIME_STEP / 1000.0
            )

        moving_point = next(point for point in obstacles if len(point) == 4)
        self.assertLess(moving_point[3], -DWA_DYNAMIC_MIN_SPEED)
        # A measured mover remains dynamic while briefly stopped at a reversal.
        stopped = planner._scan_with_dynamic_obstacles(
            scan, (0.0, 0.0, 0.0), 9 * TIME_STEP / 1000.0)
        self.assertTrue(any(len(point) == 4 for point in stopped))

    def test_tracker_handles_a_temporarily_missing_cluster(self):
        planner = DynamicWindowAvoidance()
        for step, start in enumerate((94, 92, 90, 88)):
            scan = [float("inf")] * 180
            scan[start:start + 7] = [1.0] * 7
            planner._scan_with_dynamic_obstacles(
                scan, (0.0, 0.0, 0.0), step * TIME_STEP / 1000.0
            )

        missing = planner._scan_with_dynamic_obstacles(
            [float("inf")] * 180, (0.0, 0.0, 0.0), 4 * TIME_STEP / 1000.0
        )
        resumed_scan = [float("inf")] * 180
        resumed_scan[86:93] = [1.0] * 7
        resumed = planner._scan_with_dynamic_obstacles(
            resumed_scan, (0.0, 0.0, 0.0), 5 * TIME_STEP / 1000.0
        )

        self.assertEqual([], missing)
        self.assertTrue(all(len(point) in (2, 4) for point in resumed))

    def test_crossing_yields_but_does_not_reverse_when_rear_is_blocked(self):
        planner = DynamicWindowAvoidance()
        crossing = [(0.50, 0.20, 0.0, -0.20)]

        yield_action = planner._evaluate_crossing_hazard(crossing, rear_distance=0.20)

        self.assertEqual((0.0, 0.0), yield_action[:2])
        self.assertEqual(0.0, planner.current_v)

        imminent = [(0.30, 0.0, 0.0, 0.0)]
        blocked = planner._evaluate_crossing_hazard(
            imminent, rear_distance=0.20
        )
        self.assertEqual((0.0, 0.0), blocked[:2])
        self.assertIn("정지", blocked[2])

    def test_clearance_predicts_a_perpendicular_obstacle_crossing(self):
        trajectory = [(0.2, 0.0, 0.0), (0.4, 0.0, 0.0), (0.6, 0.0, 0.0)]
        moving_obstacles = [(0.4, -0.2, 0.0, 1.0)]
        static_obstacles = [(0.4, -0.2)]

        predicted = DynamicWindowAvoidance._trajectory_clearance(
            trajectory, moving_obstacles
        )
        static = DynamicWindowAvoidance._trajectory_clearance(
            trajectory, static_obstacles
        )

        self.assertEqual(0.0, predicted)
        self.assertGreater(static, DWA_ROBOT_RADIUS)

    def test_sensor_velocity_is_only_projected_for_near_term_risk(self):
        trajectory = DynamicWindowAvoidance._simulate(0.1, 0.0)
        crossing = [(0.1, -0.30, 0.0, 0.50)]
        self.assertEqual(
            0.0,
            DynamicWindowAvoidance._trajectory_clearance(trajectory, crossing),
        )

    def test_sensor_escape_moves_away_when_waiting_would_be_hit(self):
        planner = DynamicWindowAvoidance()
        approaching = [(0.55, 0.0, -0.3, 0.0)]
        action = planner._escape_from_dynamic(approaching, (1.0, 0.0))
        self.assertIsNotNone(action)
        chosen = planner._simulate(planner.current_v, planner.current_w)[:8]
        stopped = planner._simulate(0.0, 0.0)[:8]
        self.assertGreater(
            planner._dynamic_reachable_clearance(chosen, approaching),
            planner._dynamic_reachable_clearance(stopped, approaching),
        )

    def test_emergency_does_not_authorize_a_predicted_collision(self):
        planner = DynamicWindowAvoidance()
        approaching = [(0.20, 0.0, -0.8, 0.0)]
        self.assertIsNone(planner._escape_from_dynamic(approaching, (1.0, 0.0)))

    def test_sensor_prediction_allows_for_a_turn_after_observation(self):
        trajectory = DynamicWindowAvoidance._simulate(0.0, 0.0)
        moving_away = [(0.8, 0.0, 0.2, 0.0)]
        clearance = DynamicWindowAvoidance._dynamic_reachable_clearance(
            trajectory, moving_away
        )
        self.assertLess(clearance, 0.8)

    def test_rollout_uses_speed_after_wheel_motor_limit(self):
        speed, yaw_rate, left, right = DynamicWindowAvoidance._achievable_velocity(
            0.24, 1.5
        )
        self.assertLess(speed, 0.24)
        self.assertLess(yaw_rate, 1.5)
        self.assertLessEqual(max(abs(left), abs(right)), MAX_SPEED)

    def test_recovery_yields_when_a_crossing_obstacle_blocks_both_turns(self):
        planner = DynamicWindowAvoidance()
        planner.recovery_phase = "turn"
        planner.recovery_turn_sign = 1.0
        planner.current_v = 0.08
        planner.current_w = 0.60
        crossing_obstacle = [(0.02, -0.30, 0.0, 0.60)]

        result = planner._validate_recovery_action(
            (0.0, 0.0, "recovery"), crossing_obstacle
        )

        self.assertEqual((0.0, 0.0), result[:2])
        self.assertEqual(0.0, planner.current_v)
        self.assertEqual(0.0, planner.current_w)
        self.assertIn("양보", result[2])

    def test_wheel_speeds_respect_motor_limit(self):
        left, right = DynamicWindowAvoidance._wheel_speeds(1.0, 5.0)
        self.assertLessEqual(abs(left), MAX_SPEED)
        self.assertLessEqual(abs(right), MAX_SPEED)
        self.assertTrue(math.isfinite(left) and math.isfinite(right))

    def test_positive_yaw_matches_webots_wheel_orientation(self):
        left, right = DynamicWindowAvoidance._wheel_speeds(0.10, 0.50)
        self.assertLess(left, right)

    def test_no_return_side_is_open_for_recovery_turn(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[35:56] = [0.3] * 21
        self.assertEqual(1.0, planner._select_turn_sign(ranges))

    def test_recovery_switches_when_its_turn_side_closes(self):
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 180
        blocked[75:106] = [0.35] * 31
        planner.choose_action(blocked)
        sign = planner.recovery_turn_sign
        blocked[125:146] = [0.95] * 21
        blocked[35:56] = [2.0] * 21
        planner.choose_action(blocked)
        self.assertEqual(-sign, planner.recovery_turn_sign)

    def test_recovery_reentry_keeps_direction_unless_clearly_better(self):
        """Regression: _start_recovery() used to re-pick recovery_turn_sign
        from a bare instantaneous left/right clearance snapshot every time
        recovery re-entered (e.g. released back to the path follower for one
        frame and immediately re-triggered, which happens repeatedly when
        the normal DWA rollout finds no safe candidate near two nearby
        static obstacles). That snapshot flips easily between two roughly
        symmetric obstacles, producing a fresh LEFT/RIGHT choice -- and
        therefore a fresh turn direction -- on every re-entry, even though
        the geometry has not meaningfully changed. Re-entering recovery must
        keep the existing direction unless the other side is now clearly
        (DWA_TURN_SWITCH_MARGIN) more open."""
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[75:106] = [0.35] * 31  # front blocked, roughly symmetric sides
        planner.choose_action(ranges)
        first_sign = planner.recovery_turn_sign
        self.assertIsNotNone(first_sign)

        # Simulate recovery ending and re-triggering repeatedly (as
        # is_release_ready()'s hysteresis being bypassed for one frame would
        # do) with the same near-symmetric scan each time: the direction
        # must not flip just because _start_recovery() ran again.
        for _ in range(5):
            planner.recovery_phase = None
            planner.backoff_steps = 0
            planner._start_recovery(ranges)
            self.assertEqual(first_sign, planner.recovery_turn_sign)

        # But a re-entry where the other side has since become clearly more
        # open must still switch -- the hysteresis protects against noise,
        # it does not freeze the direction forever.
        wide_open = list(ranges)
        if first_sign > 0.0:
            wide_open[125:146] = [2.0] * 21
        else:
            wide_open[35:56] = [2.0] * 21
        planner.recovery_phase = None
        planner.backoff_steps = 0
        planner._start_recovery(wide_open)
        self.assertEqual(-first_sign, planner.recovery_turn_sign)


if __name__ == "__main__":
    unittest.main()
