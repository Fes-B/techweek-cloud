import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from avoidance import DynamicWindowAvoidance
from config import (
    DWA_CROSSING_COMMIT_SPEED,
    DWA_CROSSING_GAP_CONFIRM_STEPS,
    DWA_PATH_RELEASE_CONFIRM_STEPS,
    DWA_SIDE_CAUTION_SPEED,
    MAX_SPEED,
    PRACTICE_PATH_SPEED,
    TIME_STEP,
)
from main import select_control_command


class TestAvoidanceSafetyContract(unittest.TestCase):
    @staticmethod
    def scan(distance=float("inf"), count=360):
        return [distance] * count

    def assert_command_contract(self, result):
        left, right = result[:2]
        self.assertTrue(math.isfinite(left))
        self.assertTrue(math.isfinite(right))
        self.assertLessEqual(abs(left), MAX_SPEED)
        self.assertLessEqual(abs(right), MAX_SPEED)

    def test_front_obstacle_rejects_an_unsafe_nominal_command(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 180
        ranges[88:93] = [0.30] * 5

        left, right, _ = planner.choose_action(
            ranges, pose=(0.0, 0.0, 0.0), sim_time=TIME_STEP / 1000.0
        )

        self.assertFalse(planner.is_command_safe(MAX_SPEED, MAX_SPEED))
        self.assertTrue(math.isfinite(left) and math.isfinite(right))
        self.assertLessEqual(max(abs(left), abs(right)), MAX_SPEED)

    def test_clear_scan_keeps_a_nominal_forward_command_safe(self):
        planner = DynamicWindowAvoidance()
        planner.choose_action(
            [float("inf")] * 180, pose=(0.0, 0.0, 0.0), sim_time=0.0
        )

        self.assertTrue(planner.is_command_safe(3.0, 3.0))

    def test_command_validation_covers_motion_types_and_invalid_values(self):
        planner = DynamicWindowAvoidance()
        planner.choose_action(self.scan(), pose=(0.0, 0.0, 0.0), sim_time=0.0)

        for command in ((3.0, 3.0), (2.0, 3.0), (-2.0, -2.0), (-2.0, 2.0)):
            with self.subTest(command=command):
                self.assertTrue(planner.is_command_safe(*command))
        for command in (
            (float("nan"), 0.0),
            (float("inf"), 0.0),
            (MAX_SPEED + 0.01, 0.0),
            (0.0, -MAX_SPEED - 0.01),
        ):
            with self.subTest(command=command):
                self.assertFalse(planner.is_command_safe(*command))

    def test_reverse_is_rejected_when_rear_is_blocked(self):
        planner = DynamicWindowAvoidance()
        ranges = self.scan()
        ranges[:8] = [0.20] * 8
        ranges[-8:] = [0.20] * 8
        planner.choose_action(ranges, pose=(0.0, 0.0, 0.0), sim_time=0.0)

        self.assertFalse(planner.is_command_safe(-2.0, -2.0))

    def test_all_directions_at_twenty_centimetres_stops(self):
        planner = DynamicWindowAvoidance()
        result = planner.choose_action(
            self.scan(0.20), pose=(0.0, 0.0, 0.0), sim_time=0.0
        )

        self.assertEqual((0.0, 0.0), result[:2])
        self.assertEqual(0.0, planner.current_v)
        self.assertEqual(0.0, planner.current_w)
        self.assert_command_contract(result)

    def test_repeated_blocked_recovery_remains_stopped_and_bounded(self):
        planner = DynamicWindowAvoidance()
        for step in range(8):
            result = planner.choose_action(
                self.scan(0.20),
                pose=(0.0, 0.0, 0.0),
                sim_time=step * TIME_STEP / 1000.0,
            )
            self.assertEqual((0.0, 0.0), result[:2])
            self.assert_command_contract(result)

    def test_static_scenarios_always_obey_output_contract(self):
        scenarios = {}
        scenarios["clear"] = self.scan()
        front = self.scan()
        front[165:196] = [0.35] * 31
        scenarios["front"] = front
        left = self.scan()
        left[260:281] = [0.35] * 21
        scenarios["left"] = left
        right = self.scan()
        right[80:101] = [0.35] * 21
        scenarios["right"] = right
        both_sides = self.scan()
        both_sides[80:101] = [0.45] * 21
        both_sides[260:281] = [0.45] * 21
        scenarios["both_sides"] = both_sides
        narrow_corridor = self.scan()
        narrow_corridor[75:106] = [0.45] * 31
        narrow_corridor[255:286] = [0.45] * 31
        scenarios["narrow_corridor"] = narrow_corridor
        corner = self.scan()
        corner[165:196] = [0.35] * 31
        corner[250:291] = [0.35] * 41
        scenarios["corner"] = corner

        for name, ranges in scenarios.items():
            with self.subTest(name=name):
                result = DynamicWindowAvoidance().choose_action(ranges)
                self.assert_command_contract(result)
        corridor_result = DynamicWindowAvoidance().choose_action(narrow_corridor)
        self.assertGreater(sum(corridor_result[:2]), 0.0)

    def test_dynamic_mode_preserves_recovery_and_side_speed_limit(self):
        planner = DynamicWindowAvoidance()
        blocked = self.scan()
        blocked[165:196] = [0.35] * 31
        planner.choose_action(blocked, pose=(0.0, 0.0, 0.0), sim_time=0.0)
        phase = planner.recovery_phase
        planner.choose_action(
            blocked,
            pose=(0.0, 0.0, 0.0),
            sim_time=TIME_STEP / 1000.0,
        )
        self.assertEqual(phase, planner.recovery_phase)

        corridor = self.scan()
        corridor[75:106] = [0.45] * 31
        corridor[255:286] = [0.45] * 31
        limited = DynamicWindowAvoidance()
        for step in range(20):
            limited.choose_action(
                corridor,
                pose=(0.0, 0.0, 0.0),
                sim_time=step * TIME_STEP / 1000.0,
            )
        self.assertGreater(limited.current_v, 0.0)
        self.assertLessEqual(limited.current_v, DWA_SIDE_CAUTION_SPEED)

    def test_moving_obstacle_does_not_start_static_recovery(self):
        planner = DynamicWindowAvoidance()
        ranges = self.scan()
        ranges[175:186] = [0.40] * 11
        moving_away = [(0.40, 0.30, 0.0, 0.20)]

        with patch.object(
            planner, "_scan_with_dynamic_obstacles", return_value=moving_away
        ):
            result = planner.choose_action(
                ranges, pose=(0.0, 0.0, 0.0), sim_time=0.0
            )

        self.assertIsNone(planner.recovery_phase)
        self.assertNotIn("복구", result[2])

    def test_active_recovery_releases_when_only_moving_obstacle_remains(self):
        planner = DynamicWindowAvoidance()
        planner.recovery_phase = "turn"
        planner.recovery_turn_sign = 1.0
        ranges = self.scan()
        ranges[175:186] = [0.40] * 11
        moving_away = [(0.40, 0.30, 0.0, 0.20)]

        with patch.object(
            planner, "_scan_with_dynamic_obstacles", return_value=moving_away
        ):
            result = planner.choose_action(
                ranges, pose=(0.0, 0.0, 0.0), sim_time=0.0
            )

        self.assertIsNone(planner.recovery_phase)
        self.assertNotIn("복구", result[2])

    def test_static_obstacle_still_starts_recovery_in_sensor_mode(self):
        planner = DynamicWindowAvoidance()
        ranges = self.scan()
        ranges[175:186] = [0.40] * 11

        with patch.object(
            planner, "_scan_with_dynamic_obstacles", return_value=[(0.40, 0.0)]
        ):
            planner.choose_action(
                ranges, pose=(0.0, 0.0, 0.0), sim_time=0.0
            )

        self.assertEqual("turn", planner.recovery_phase)

    def test_crossing_directions_yield_and_moving_away_releases(self):
        for crossing in (
            [(0.50, -0.20, 0.0, 0.20)],
            [(0.50, 0.20, 0.0, -0.20)],
            [(0.50, 0.10, 0.0, 0.0)],
        ):
            with self.subTest(crossing=crossing):
                result = DynamicWindowAvoidance()._evaluate_crossing_hazard(
                    crossing, rear_distance=float("inf")
                )
                self.assertEqual((0.0, 0.0), result[:2])
                self.assert_command_contract(result)

        moving_away = [(0.50, 0.30, 0.0, 0.20)]
        self.assertIsNone(
            DynamicWindowAvoidance()._evaluate_crossing_hazard(
                moving_away, rear_distance=float("inf")
            )
        )

    def test_crossing_clearing_does_not_drive_through_a_static_wall(self):
        planner = DynamicWindowAvoidance()
        moving = [(0.0, 0.20, 0.0, -0.20)]
        obstacles = moving + [(0.20, 0.0)]

        result = planner._evaluate_crossing_hazard(
            moving, rear_distance=float("inf"), obstacles=obstacles
        )

        self.assertEqual((0.0, 0.0), result[:2])
        self.assert_command_contract(result)

    def test_crossing_gap_is_confirmed_before_commit(self):
        planner = DynamicWindowAvoidance()
        blocking = [(0.50, 0.10, 0.0, 0.0)]
        passed = [(0.50, 0.35, 0.0, 0.10)]

        planner._evaluate_crossing_hazard(blocking, rear_distance=float("inf"))
        first_gap = planner._evaluate_crossing_hazard(
            passed, rear_distance=float("inf"), pose=(0.0, 0.0, 0.0)
        )

        self.assertEqual((0.0, 0.0), first_gap[:2])
        self.assertFalse(planner.crossing_commit_active)
        for _ in range(DWA_CROSSING_GAP_CONFIRM_STEPS - 1):
            committed = planner._evaluate_crossing_hazard(
                passed, rear_distance=float("inf"), pose=(0.0, 0.0, 0.0)
            )

        self.assertTrue(planner.crossing_commit_active)
        self.assertGreater(committed[0], 0.0)
        self.assertGreater(committed[1], 0.0)
        self.assertAlmostEqual(DWA_CROSSING_COMMIT_SPEED, planner.current_v)
        self.assertGreater(planner.current_v, PRACTICE_PATH_SPEED)

    def test_crossing_wait_releases_when_tracked_threat_vanishes_without_departure(self):
        """A stationary track that triggered the wait but was never a real
        mover (e.g. a transient LiDAR cluster near a static corner during a
        sharp turn) fades out of tracking instead of ever registering
        oy*vy > 0. The wait must release once no tracked mover remains in the
        crossing zone -- falling through to normal DWA/recovery -- instead of
        deadlocking forever waiting for a departure that will never come."""
        planner = DynamicWindowAvoidance()
        blocking = [(0.50, 0.10, 0.0, 0.0)]

        planner._evaluate_crossing_hazard(blocking, rear_distance=float("inf"))
        self.assertTrue(planner.crossing_waiting)

        released = planner._evaluate_crossing_hazard(
            [], rear_distance=float("inf"), pose=(0.0, 0.0, 0.0)
        )

        self.assertIsNone(released)
        self.assertFalse(planner.crossing_waiting)
        self.assertFalse(planner.crossing_commit_active)

    def test_crossing_commit_ignores_repeated_yield_redecisions_until_exit(self):
        planner = DynamicWindowAvoidance()
        blocking = [(0.50, 0.10, 0.0, 0.0)]
        passed = [(0.50, 0.35, 0.0, 0.10)]
        returning_but_not_imminent = [(0.50, 0.35, 0.0, -0.10)]

        planner._evaluate_crossing_hazard(blocking, rear_distance=float("inf"))
        for _ in range(DWA_CROSSING_GAP_CONFIRM_STEPS):
            planner._evaluate_crossing_hazard(
                passed, rear_distance=float("inf"), pose=(0.0, 0.0, 0.0)
            )

        action = planner._evaluate_crossing_hazard(
            returning_but_not_imminent,
            rear_distance=float("inf"),
            pose=(0.0, 0.0, 0.0),
        )

        self.assertTrue(planner.crossing_commit_active)
        self.assertGreater(action[0], 0.0)
        self.assertIn("crossing commit", action[2])

        planner._update_crossing_commit_progress((0.80, 0.0, 0.0))
        self.assertFalse(planner.crossing_commit_active)

    def test_crossing_commit_is_cancelled_only_for_imminent_collision(self):
        planner = DynamicWindowAvoidance()
        planner.crossing_commit_active = True
        planner.crossing_commit_remaining = 0.70
        planner.crossing_commit_last_pose = (0.0, 0.0, 0.0)

        result = planner._evaluate_crossing_hazard(
            [(0.22, 0.0, -0.10, 0.0)], rear_distance=0.20
        )

        self.assertFalse(planner.crossing_commit_active)
        self.assertEqual((0.0, 0.0), result[:2])

    def test_crossing_commit_keeps_state_when_common_safety_gate_stops(self):
        planner = DynamicWindowAvoidance()
        planner.crossing_commit_active = True
        planner.crossing_commit_remaining = 0.70
        planner.crossing_commit_last_pose = (0.0, 0.0, 0.0)

        result = planner._evaluate_crossing_hazard(
            [], rear_distance=float("inf"), obstacles=[(0.45, 0.0)]
        )

        self.assertTrue(planner.crossing_commit_active)
        self.assertEqual((0.0, 0.0), result[:2])
        self.assertFalse(planner._velocity_is_safe(
            DWA_CROSSING_COMMIT_SPEED, 0.0, [(0.45, 0.0)]
        ))

    def test_imminent_threat_has_priority_over_crossing_clearing(self):
        clearing = (0.0, 0.20, 0.0, -0.20)
        imminent = (0.30, 0.0, 0.0, 0.0)

        result = DynamicWindowAvoidance()._evaluate_crossing_hazard(
            [clearing, imminent], rear_distance=0.20
        )

        self.assertEqual((0.0, 0.0), result[:2])
        self.assertIn("정지", result[2])

    def test_close_side_threat_never_reverses_into_blocked_rear(self):
        threat = [(0.35, 0.20, 0.0, 0.0)]
        rear_free = DynamicWindowAvoidance()._evaluate_crossing_hazard(
            threat, rear_distance=float("inf")
        )
        self.assertLess(rear_free[0], 0.0)
        self.assertLess(rear_free[1], 0.0)

        rear_blocked = DynamicWindowAvoidance()._evaluate_crossing_hazard(
            threat, rear_distance=0.20
        )
        self.assertEqual((0.0, 0.0), rear_blocked[:2])
        self.assertIn("정지", rear_blocked[2])

    def test_dynamic_track_survives_one_missing_scan(self):
        planner = DynamicWindowAvoidance()
        obstacles = None
        for step, start in enumerate(range(194, 185, -1)):
            scan = self.scan()
            scan[start:start + 7] = [1.0] * 7
            obstacles = planner._scan_with_dynamic_obstacles(
                scan, (0.0, 0.0, 0.0), step * TIME_STEP / 1000.0
            )
        self.assertTrue(any(len(point) == 4 for point in obstacles))

        held = planner._scan_with_dynamic_obstacles(
            self.scan(),
            (0.0, 0.0, 0.0),
            9 * TIME_STEP / 1000.0,
        )
        self.assertTrue(any(len(point) == 4 for point in held))

    def test_dynamic_track_handles_direction_reversal(self):
        planner = DynamicWindowAvoidance()
        observed_lateral_velocities = []
        starts = list(range(194, 185, -1)) + list(range(187, 198))
        for step, start in enumerate(starts):
            scan = self.scan()
            scan[start:start + 7] = [1.0] * 7
            obstacles = planner._scan_with_dynamic_obstacles(
                scan, (0.0, 0.0, 0.0), step * TIME_STEP / 1000.0
            )
            moving = [point for point in obstacles if len(point) == 4]
            if moving:
                observed_lateral_velocities.append(
                    sum(point[3] for point in moving) / len(moving)
                )

        self.assertLess(min(observed_lateral_velocities), 0.0)
        self.assertGreater(max(observed_lateral_velocities), 0.0)

    def test_sensor_robustness_outputs_are_finite_and_bounded(self):
        scans = [
            None,
            [],
            self.scan(),
            [float("nan") if index % 17 == 0 else float("inf") for index in range(360)],
        ]
        single_noisy = self.scan()
        single_noisy[180] = 0.06
        scans.append(single_noisy)
        for ranges in scans:
            with self.subTest(ranges="none" if ranges is None else len(ranges)):
                self.assert_command_contract(DynamicWindowAvoidance().choose_action(ranges))


class TestPathAvoidanceIntegration(unittest.TestCase):
    def test_clear_path_command_is_kept(self):
        planner = DynamicWindowAvoidance()
        avoidance = planner.choose_action([float("inf")] * 360)
        selected = select_control_command(planner, (3.0, 3.0), avoidance)
        self.assertEqual((3.0, 3.0, "PATH_FOLLOW"), selected)

    def test_crossing_commit_has_priority_over_safe_nominal_command(self):
        planner = DynamicWindowAvoidance()
        planner.choose_action([float("inf")] * 360)
        planner.crossing_commit_active = True
        avoidance = planner._crossing_commit_action([])

        selected = select_control_command(planner, (3.0, 3.0), avoidance)

        self.assertEqual(avoidance[:2], selected[:2])
        self.assertTrue(selected[2].startswith("AVOID"))

    def test_front_obstacle_selects_avoidance_command(self):
        planner = DynamicWindowAvoidance()
        ranges = [float("inf")] * 360
        ranges[175:186] = [0.30] * 11
        avoidance = planner.choose_action(ranges)
        selected = select_control_command(planner, (3.0, 3.0), avoidance)
        self.assertEqual(avoidance[:2], selected[:2])
        self.assertTrue(selected[2].startswith("AVOID"))

    def test_path_command_resumes_after_obstacle_disappears(self):
        """A single clear frame must NOT immediately release control back to
        the path follower: a corner-hugging turn can be judged safe for one
        frame and unsafe again the next, and handing control back on that
        single frame ping-pongs PATH_FOLLOW/AVOID every step without ever
        clearing the obstacle. DWA_PATH_RELEASE_CONFIRM_STEPS consecutive
        clear frames must be observed first (see is_release_ready())."""
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 360
        blocked[175:186] = [0.30] * 11
        avoidance = planner.choose_action(blocked)
        self.assertTrue(
            select_control_command(planner, (3.0, 3.0), avoidance)[2].startswith("AVOID")
        )

        clear = [float("inf")] * 360
        for _ in range(DWA_PATH_RELEASE_CONFIRM_STEPS - 1):
            clear_avoidance = planner.choose_action(clear)
            selected = select_control_command(planner, (3.0, 3.0), clear_avoidance)
            self.assertTrue(selected[2].startswith("AVOID"))

        clear_avoidance = planner.choose_action(clear)
        selected = select_control_command(planner, (3.0, 3.0), clear_avoidance)
        self.assertEqual((3.0, 3.0, "PATH_FOLLOW"), selected)

    def test_release_hysteresis_prevents_single_frame_ping_pong(self):
        """Regression test for the static-corner oscillation: a nominal
        command that alternates safe/unsafe every other frame (exactly what
        a corner-hugging turn produces -- turn is judged safe, gets applied,
        the resulting heading change makes the same turn unsafe again one
        frame later) must never release control to the path follower, since
        the safe frames never arrive consecutively."""
        planner = DynamicWindowAvoidance()
        blocked = [float("inf")] * 360
        blocked[175:186] = [0.30] * 11
        clear = [float("inf")] * 360

        released = False
        for i in range(20):
            # Start on an unsafe frame -- the streak begins pre-filled (a
            # path that was never interrupted needs no delay), so the test
            # must first force a real unsafe frame before alternating.
            ranges = blocked if i % 2 == 0 else clear
            avoidance = planner.choose_action(ranges)
            selected = select_control_command(planner, (3.0, 3.0), avoidance)
            if selected[2] == "PATH_FOLLOW":
                released = True
        self.assertFalse(released)

    def test_release_gate_blocks_while_rollout_finds_no_safe_candidate(self):
        """Regression test: the normal DWA rollout finding zero safe
        candidates and falling back to an emergency turn does not set
        recovery_phase, so a guard that only checked recovery_phase missed
        it -- the nominal command could coincidentally look safe for one
        frame mid-fallback and ping-pong control exactly like the
        recovery_phase=="turn" case. Verified directly against
        is_release_ready() since reliably driving choose_action() into the
        best-is-None branch requires a specific geometry."""
        planner = DynamicWindowAvoidance()
        planner.choose_action([float("inf")] * 360)  # populate last_obstacles
        planner._last_rollout_had_no_safe_candidate = True
        for _ in range(10):
            self.assertFalse(planner.is_release_ready(3.0, 3.0))

    def test_release_gate_lets_plain_wall_follow_cruise_break_out_via_hysteresis(self):
        """Regression test for the complementary failure: gating release on
        plain wall-follow cruising (wall_side set, wall_escape_active False)
        made the robot orbit a square obstacle's perimeter forever, since
        each face looks like "a wall is still here" to wall-follow's own
        exit condition. Plain cruising must still release control once the
        nominal command has been safe for DWA_PATH_RELEASE_CONFIRM_STEPS
        consecutive frames, same as the no-recovery-active case."""
        planner = DynamicWindowAvoidance()
        planner.choose_action([float("inf")] * 360)
        planner.wall_side = "right"
        planner.wall_escape_active = False
        planner._path_release_streak = 0
        released = False
        for _ in range(DWA_PATH_RELEASE_CONFIRM_STEPS):
            released = planner.is_release_ready(3.0, 3.0)
        self.assertTrue(released)

    def test_invalid_avoidance_override_fails_safe_to_stop(self):
        planner = DynamicWindowAvoidance()
        blocked = [0.20] * 360
        planner.choose_action(blocked)

        selected = select_control_command(
            planner,
            (3.0, 3.0),
            (float("nan"), MAX_SPEED + 1.0, "invalid"),
        )

        self.assertEqual((0.0, 0.0, "AVOID FINAL SAFETY STOP"), selected)
