"""Unit and closed-loop tests for avoidance_v2 (DWA-primary static avoidance)."""

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import config
from avoidance_v2 import (
    DYNAMIC_SAFE_DISTANCE,
    CostToGoField,
    ProgressDWA,
    ProgressWatchdog,
    StaticAvoidanceState,
    StaticPersistenceFilter,
    filter_isolated_returns,
)
from avoidance import DynamicWindowAvoidance
from config import (
    MAX_SPEED,
    RECOVERY_BACKOFF_DISTANCE,
    RECOVERY_COMMIT_ANGLE,
    TIME_STEP,
    WATCHDOG_GRACE_SECONDS,
    WATCHDOG_LONG_WINDOW_SECONDS,
    WATCHDOG_WINDOW_SECONDS,
    WHEEL_RADIUS,
    WHEEL_TRACK,
)
from tools.avoidance_scenarios import CASES
from tools.avoidance_sim import lidar_scan, passable, run_case


DT = TIME_STEP / 1000.0


def yaw_rate(result):
    return WHEEL_RADIUS * (result[1] - result[0]) / WHEEL_TRACK


def linear(result):
    return 0.5 * WHEEL_RADIUS * (result[0] + result[1])


def local_goal(goal, pose):
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


class Frames:
    """Feed a planner scans of static boxes at chosen (possibly frozen) poses."""

    def __init__(self, boxes, goal=(3.0, 0.0), planner=None):
        self.boxes = boxes
        self.goal = goal
        self.planner = planner or ProgressDWA()
        self.time = 0.0

    def step(self, pose, waypoint_id=0):
        ranges = lidar_scan(pose, self.boxes)
        result = self.planner.choose_action(
            ranges, local_goal(self.goal, pose), pose=pose, sim_time=self.time,
            waypoint_id=waypoint_id,
        )
        self.time += DT
        return result

    def hold(self, pose, seconds, waypoint_id=0):
        result = None
        for _ in range(int(round(seconds / DT))):
            result = self.step(pose, waypoint_id)
        return result


# A box whose near face is 0.30 m ahead: DWA cannot make progress while the
# pose is frozen, so the watchdog must fire.
FRONT_BLOCK = [(0.30 + 0.225, 0.0, 0.45, 0.80)]


class TestSafetyConstantsUnchanged(unittest.TestCase):
    def test_safety_model_constants_match_legacy(self):
        self.assertEqual(0.17, config.DWA_ROBOT_RADIUS)
        self.assertEqual(0.05, config.DWA_STATIC_CLEARANCE_MARGIN)
        self.assertEqual(0.15, config.DWA_DYNAMIC_CLEARANCE_MARGIN)
        self.assertEqual(0.02, config.DWA_EMERGENCY_CLEARANCE_MARGIN)
        self.assertEqual(0.60, config.DWA_LINEAR_ACCELERATION)
        self.assertEqual(4.0, config.DWA_ANGULAR_ACCELERATION)
        self.assertEqual(2.5, config.DWA_HORIZON)
        # The cost-to-go inflation is the static safety distance, not less.
        self.assertAlmostEqual(
            config.DWA_ROBOT_RADIUS + config.DWA_STATIC_CLEARANCE_MARGIN,
            config.NAV_FIELD_INFLATION,
        )


class TestProgressWatchdog(unittest.TestCase):
    def run_watchdog(self, watchdog, poses, goal_distances, paused=None):
        stuck = []
        for index, (pose, goal) in enumerate(zip(poses, goal_distances)):
            stuck.append(watchdog.update(
                DT, pose, goal, paused=bool(paused and paused[index])
            ))
        return stuck

    def test_reports_stuck_after_window_without_progress(self):
        watchdog = ProgressWatchdog()
        frames = int(WATCHDOG_WINDOW_SECONDS / DT) + 2
        stuck = self.run_watchdog(watchdog, [(0.0, 0.0, 0.0)] * frames, [2.0] * frames)
        self.assertFalse(any(stuck[: int(WATCHDOG_WINDOW_SECONDS / DT) - 1]))
        self.assertTrue(stuck[-1])
        self.assertEqual("no pose progress", watchdog.stuck_reason)

    def test_slow_forward_motion_is_progress(self):
        watchdog = ProgressWatchdog()
        frames = int(10.0 / DT)
        speed = 0.05  # m/s, far below cruise speed
        poses = [(speed * DT * i, 0.0, 0.0) for i in range(frames)]
        goals = [2.0 - speed * DT * i for i in range(frames)]
        self.assertFalse(any(self.run_watchdog(watchdog, poses, goals)))

    def test_normal_slow_rotation_is_not_stuck(self):
        watchdog = ProgressWatchdog()
        frames = int(5.0 / DT)  # shorter than the long window
        rate = 0.30  # rad/s in place, e.g. turning towards a new waypoint
        poses = [(0.0, 0.0, math.atan2(math.sin(rate * DT * i), math.cos(rate * DT * i)))
                 for i in range(frames)]
        self.assertFalse(any(self.run_watchdog(watchdog, poses, [2.0] * frames)))

    def test_small_oscillation_is_caught_by_long_window(self):
        watchdog = ProgressWatchdog()
        frames = int((WATCHDOG_LONG_WINDOW_SECONDS + 1.0) / DT)
        # +-6 cm back and forth: re-arms the short criterion every swing.
        poses = [(0.06 * math.sin(2.0 * math.pi * 0.5 * DT * i), 0.0, 0.0) for i in range(frames)]
        stuck = self.run_watchdog(watchdog, poses, [2.0] * frames)
        self.assertTrue(any(stuck))
        self.assertEqual("no net progress (oscillation)", watchdog.stuck_reason)

    def test_pause_freezes_the_timer(self):
        watchdog = ProgressWatchdog()
        pose = (0.0, 0.0, 0.0)
        before = int(1.0 / DT)
        for _ in range(before):
            self.assertFalse(watchdog.update(DT, pose, 2.0))
        for _ in range(int(10.0 / DT)):  # long dynamic yield
            self.assertFalse(watchdog.update(DT, pose, 2.0, paused=True))
        remaining = int(math.ceil((WATCHDOG_WINDOW_SECONDS - 1.0) / DT))
        results = [watchdog.update(DT, pose, 2.0) for _ in range(remaining + 1)]
        self.assertFalse(any(results[:-3]))
        self.assertTrue(results[-1])

    def test_goal_within_tolerance_is_never_stuck(self):
        watchdog = ProgressWatchdog()
        frames = int(10.0 / DT)
        self.assertFalse(any(self.run_watchdog(
            watchdog, [(0.0, 0.0, 0.0)] * frames, [0.10] * frames)))

    def test_grace_period_after_recovery(self):
        watchdog = ProgressWatchdog()
        watchdog.start_grace(WATCHDOG_GRACE_SECONDS)
        frames = int((WATCHDOG_GRACE_SECONDS + WATCHDOG_WINDOW_SECONDS - 0.1) / DT)
        stuck = self.run_watchdog(watchdog, [(0.0, 0.0, 0.0)] * frames, [2.0] * frames)
        self.assertFalse(any(stuck))
        more = self.run_watchdog(watchdog, [(0.0, 0.0, 0.0)] * 10, [2.0] * 10)
        self.assertTrue(any(more))


class TestPlannerWatchdogIntegration(unittest.TestCase):
    def test_frozen_pose_near_obstacle_starts_recovery(self):
        frames = Frames(FRONT_BLOCK)
        pose = (0.0, 0.0, 0.0)
        frames.hold(pose, WATCHDOG_WINDOW_SECONDS - 0.2)
        self.assertIs(StaticAvoidanceState.NORMAL_DWA, frames.planner.state)
        frames.hold(pose, 0.4)
        self.assertIsNot(StaticAvoidanceState.NORMAL_DWA, frames.planner.state)
        self.assertEqual(1, frames.planner.recovery_count)

    def test_waypoint_change_resets_watchdog(self):
        frames = Frames(FRONT_BLOCK)
        pose = (0.0, 0.0, 0.0)
        frames.hold(pose, WATCHDOG_WINDOW_SECONDS - 0.2, waypoint_id=0)
        frames.hold(pose, WATCHDOG_WINDOW_SECONDS - 0.2, waypoint_id=1)
        self.assertIs(StaticAvoidanceState.NORMAL_DWA, frames.planner.state)
        self.assertEqual(0, frames.planner.recovery_count)

    def test_dynamic_yield_pauses_watchdog_and_keeps_static_state(self):
        frames = Frames(FRONT_BLOCK)
        pose = (0.0, 0.0, 0.0)
        yield_action = (0.0, 0.0, "DWA 횡단 장애물 안전 거리 대기")
        with patch.object(frames.planner.dynamic, "override", return_value=yield_action):
            result = frames.hold(pose, 3 * WATCHDOG_WINDOW_SECONDS)
        self.assertEqual((0.0, 0.0), result[:2])
        self.assertEqual("AVOID DYNAMIC", frames.planner.control_label)
        self.assertIs(StaticAvoidanceState.NORMAL_DWA, frames.planner.state)
        self.assertEqual(0, frames.planner.recovery_count)
        self.assertEqual(0.0, frames.planner.watchdog.no_progress_seconds)

    def test_clear_path_never_starts_recovery(self):
        frames = Frames([])
        pose = [0.0, 0.0, 0.0]
        for _ in range(int(8.0 / DT)):
            result = frames.step(tuple(pose))
            pose[0] += linear(result) * DT
        self.assertEqual(0, frames.planner.recovery_count)
        self.assertGreater(pose[0], 1.2)


class TestRecovery(unittest.TestCase):
    def enter_recovery(self, boxes, pose=(0.0, 0.0, 0.0)):
        frames = Frames(boxes)
        frames.hold(pose, WATCHDOG_WINDOW_SECONDS + 0.2)
        self.assertEqual(1, frames.planner.recovery_count)
        return frames

    def test_backoff_uses_odometry_displacement_not_frames(self):
        frames = self.enter_recovery(FRONT_BLOCK)
        self.assertIs(StaticAvoidanceState.RECOVERY_BACKOFF, frames.planner.state)
        # Wheels commanded backwards but the pose does not change: no progress,
        # so the backoff must not "complete" by frame count.
        result = frames.hold((0.0, 0.0, 0.0), 1.0)
        self.assertIs(StaticAvoidanceState.RECOVERY_BACKOFF, frames.planner.state)
        self.assertLess(linear(result), 0.0)
        frames.step((-(RECOVERY_BACKOFF_DISTANCE - 0.02), 0.0, 0.0))
        self.assertIs(StaticAvoidanceState.RECOVERY_BACKOFF, frames.planner.state)
        frames.step((-(RECOVERY_BACKOFF_DISTANCE + 0.01), 0.0, 0.0))
        self.assertIs(StaticAvoidanceState.RECOVERY_COMMIT, frames.planner.state)

    def test_unsafe_rear_skips_backoff(self):
        boxes = FRONT_BLOCK + [(-0.25 - 0.225, 0.0, 0.45, 0.80)]  # 0.25 m behind
        frames = self.enter_recovery(boxes)
        self.assertIs(StaticAvoidanceState.RECOVERY_COMMIT, frames.planner.state)
        result = frames.step((0.0, 0.0, 0.0))
        self.assertGreaterEqual(linear(result), 0.0)

    def test_direction_is_chosen_once_and_kept_until_commit_is_met(self):
        frames = self.enter_recovery(FRONT_BLOCK)
        sign = frames.planner.recovery.sign
        pose = (-(RECOVERY_BACKOFF_DISTANCE + 0.01), 0.0, 0.0)
        frames.step(pose)
        self.assertIs(StaticAvoidanceState.RECOVERY_COMMIT, frames.planner.state)
        # Make the *other* side look much more open; the committed turn keeps
        # its sign until the measured heading change reaches the minimum.
        other_side_open = FRONT_BLOCK + [(-0.3, sign * 0.55, 1.2, 0.2)]
        frames.boxes = other_side_open
        heading = 0.0
        for _ in range(12):
            result = frames.step((pose[0], pose[1], heading))
            self.assertEqual(sign, frames.planner.recovery.sign)
            self.assertGreater(yaw_rate(result) * sign, 0.0)
            heading += 0.4 * RECOVERY_COMMIT_ANGLE / 12 * sign
        self.assertIs(StaticAvoidanceState.RECOVERY_COMMIT, frames.planner.state)

    def test_vetoed_commit_turn_stops_instead_of_flipping(self):
        frames = self.enter_recovery(FRONT_BLOCK)
        planner = frames.planner
        planner.state = StaticAvoidanceState.RECOVERY_COMMIT
        sign = planner.recovery.sign
        with patch.object(planner, "_is_safe", return_value=False):
            result = frames.step((0.0, 0.0, 0.0))
        self.assertEqual((0.0, 0.0), result[:2])
        self.assertEqual(sign, planner.recovery.sign)

    def test_recovery_returns_to_normal_dwa_with_grace(self):
        frames = self.enter_recovery(FRONT_BLOCK)
        planner = frames.planner
        sign = planner.recovery.sign
        back = -(RECOVERY_BACKOFF_DISTANCE + 0.01)
        frames.step((back, 0.0, 0.0))
        turned = sign * (RECOVERY_COMMIT_ANGLE + 0.02)
        frames.step((back, 0.0, turned))
        self.assertEqual("forward", planner.recovery.segment)
        distance = config.RECOVERY_COMMIT_FORWARD_DISTANCE + 0.01
        frames.step((back + distance * math.cos(turned), distance * math.sin(turned), turned))
        self.assertIs(StaticAvoidanceState.NORMAL_DWA, planner.state)
        self.assertIsNone(planner.recovery)
        self.assertGreater(planner.watchdog.grace_remaining, 0.0)

    def test_direction_prefers_the_feasible_side(self):
        # Wall close on the left: turning left and driving on is infeasible.
        boxes = FRONT_BLOCK + [(0.0, 0.33, 1.2, 0.12)]
        frames = self.enter_recovery(boxes)
        self.assertEqual(-1.0, frames.planner.recovery.sign)
        mirrored = FRONT_BLOCK + [(0.0, -0.33, 1.2, 0.12)]
        frames = self.enter_recovery(mirrored)
        self.assertEqual(1.0, frames.planner.recovery.sign)


class TestCostToGoField(unittest.TestCase):
    def test_open_space_is_euclidean(self):
        field = CostToGoField()
        field.build([], (2.0, 1.0), (0.0, 0.0))
        self.assertAlmostEqual(math.hypot(2.0, 1.0), float(field.cost_at(0.0, 0.0)), places=2)
        self.assertAlmostEqual(math.atan2(1.0, 2.0),
                               float(field.descent_heading_at(0.0, 0.0)), places=1)

    def test_obstacle_between_robot_and_goal_adds_detour(self):
        points = [(0.9, y * 0.02) for y in range(-15, 16)]  # 0.6 m wide plate
        field = CostToGoField()
        field.build(points, (2.5, 0.0), (0.0, 0.0))
        self.assertGreater(float(field.cost_at(0.0, 0.0)), 2.5 + 0.1)
        # Moving sideways (around the plate) reduces cost-to-go.
        self.assertLess(float(field.cost_at(0.0, 0.5)) - 0.5, float(field.cost_at(0.0, 0.0)))

    def test_mirror_symmetry(self):
        points = [(0.9, 0.05 + y * 0.02) for y in range(0, 20)]
        mirrored = [(x, -y) for x, y in points]
        a, b = CostToGoField(), CostToGoField()
        a.build(points, (2.5, 0.0), (0.0, 0.0))
        b.build(mirrored, (2.5, 0.0), (0.0, 0.0))
        for y in (0.0, 0.2, 0.4):
            self.assertAlmostEqual(float(a.cost_at(0.3, y)), float(b.cost_at(0.3, -y)), places=6)


class TestCommandContract(unittest.TestCase):
    def assert_contract(self, result):
        left, right = result[:2]
        self.assertTrue(math.isfinite(left) and math.isfinite(right))
        self.assertLessEqual(max(abs(left), abs(right)), MAX_SPEED)

    def test_requires_pose_and_time(self):
        with self.assertRaises(ValueError):
            ProgressDWA().choose_action([math.inf] * 180)

    def test_missing_and_invalid_scans_are_safe(self):
        planner = ProgressDWA()
        for index, ranges in enumerate((
            None, [], [math.inf] * 180,
            [math.nan if i % 17 == 0 else math.inf for i in range(180)],
        )):
            result = planner.choose_action(ranges, (2.0, 0.0), pose=(0.0, 0.0, 0.0),
                                           sim_time=index * DT)
            self.assert_contract(result)

    def test_enclosed_at_twenty_centimetres_stops(self):
        planner = ProgressDWA()
        for step in range(int(3.0 / DT)):
            result = planner.choose_action([0.20] * 180, (2.0, 0.0), pose=(0.0, 0.0, 0.0),
                                           sim_time=step * DT)
            self.assertEqual((0.0, 0.0), result[:2])

    def test_emergency_obstacle_ahead_while_fast(self):
        planner = ProgressDWA()
        planner.current_v = 0.24
        blocked = lidar_scan((0.0, 0.0, 0.0), [(0.25 + 0.225, 0.0, 0.45, 0.45)])
        result = planner.choose_action(blocked, (2.0, 0.0), pose=(0.0, 0.0, 0.0), sim_time=0.0)
        self.assert_contract(result)
        self.assertTrue(result[:2] == (0.0, 0.0) or planner.is_command_safe(*result[:2]))

    def test_output_is_always_vetted_by_the_shared_safety_model(self):
        # Every non-STOP command passes the shared veto -- except the one
        # documented case (_emit / _safer_than_stopping): a dynamic-layer
        # command kept because STOP itself is predicted to be hit.  Even then
        # the static part of the veto must hold.  (The legacy tracker can
        # report a static box face seen from a moving viewpoint as a mover.)
        frames = Frames(CASES["frontal"]["boxes"], goal=(3.2, 0.0))
        planner = frames.planner
        pose = [0.0, 0.0, 0.0]
        for _ in range(int(20.0 / DT)):
            result = frames.step(tuple(pose))
            if result[:2] != (0.0, 0.0) and not planner.is_command_safe(*result[:2]):
                self.assertIn("(safer than stopping)", result[2])
                obstacles = planner.last_obstacles
                self.assertTrue(planner._stopping_is_unsafe(obstacles))
                velocity, rate = linear(result), yaw_rate(result)
                fixed = [point for point in obstacles if len(point) == 2]
                self.assertTrue(planner._toolkit._velocity_is_safe(velocity, rate, fixed))
            pose[0] += linear(result) * math.cos(pose[2]) * DT
            pose[1] += linear(result) * math.sin(pose[2]) * DT
            pose[2] += yaw_rate(result) * DT


class TestDynamicCrossingRegression(unittest.TestCase):
    def test_crossing_obstacle_ahead_is_yielded_to(self):
        planner = ProgressDWA()
        crossing = [(0.50, 0.20, 0.0, -0.20)]
        with patch.object(planner._toolkit, "_scan_with_dynamic_obstacles", return_value=crossing):
            result = planner.choose_action([math.inf] * 180, (2.0, 0.0),
                                           pose=(0.0, 0.0, 0.0), sim_time=0.0)
        self.assertEqual((0.0, 0.0), result[:2])
        self.assertEqual("AVOID DYNAMIC", planner.control_label)
        self.assertTrue(planner.crossing_waiting)
        self.assertIs(StaticAvoidanceState.NORMAL_DWA, planner.state)

    def test_commit_blocked_by_static_geometry_is_cancelled(self):
        planner = ProgressDWA()
        toolkit = planner._toolkit
        toolkit.crossing_commit_active = True
        toolkit.crossing_commit_remaining = 0.70
        toolkit.crossing_commit_last_pose = (0.0, 0.0, 0.0)
        wall = [(0.25 + 0.06, 0.0, 0.12, 1.5)]  # static wall 0.25 m ahead
        frames = Frames(wall, planner=planner)
        result = frames.step((0.0, 0.0, 0.0))
        self.assertEqual((0.0, 0.0), result[:2])
        self.assertTrue(planner.crossing_commit_active)
        frames.hold((0.0, 0.0, 0.0), config.DYNAMIC_COMMIT_STATIC_BLOCK_SECONDS + 0.1)
        self.assertFalse(planner.crossing_commit_active)
        self.assertEqual(1, planner.dynamic.commits_cancelled)

    def test_commit_vetoed_by_a_mover_keeps_waiting(self):
        planner = ProgressDWA()
        toolkit = planner._toolkit
        toolkit.crossing_commit_active = True
        toolkit.crossing_commit_remaining = 0.70
        toolkit.crossing_commit_last_pose = (0.0, 0.0, 0.0)
        mover = [(0.55, 0.05, 0.0, 0.05)]  # slow mover right on the commit path
        with patch.object(toolkit, "_scan_with_dynamic_obstacles", return_value=mover):
            for step in range(int(3.0 / DT)):
                planner.choose_action([math.inf] * 180, (2.0, 0.0), pose=(0.0, 0.0, 0.0),
                                      sim_time=step * DT)
        self.assertEqual(0, planner.dynamic.commits_cancelled)

    def test_moving_away_obstacle_does_not_stop_the_robot(self):
        planner = ProgressDWA()
        moving_away = [(0.50, 0.40, 0.0, 0.30)]
        with patch.object(planner._toolkit, "_scan_with_dynamic_obstacles",
                          return_value=moving_away):
            planner.choose_action([math.inf] * 180, (2.0, 0.0), pose=(0.0, 0.0, 0.0), sim_time=0.0)
        self.assertFalse(planner.crossing_waiting)
        self.assertIs(StaticAvoidanceState.NORMAL_DWA, planner.state)


class TestLidarSpikeFilter(unittest.TestCase):
    def test_isolated_near_return_is_removed(self):
        ranges = [2.0] * 180
        ranges[50] = 0.15
        filtered, removed = filter_isolated_returns(ranges)
        self.assertEqual(1, removed)
        self.assertEqual(math.inf, filtered[50])

    def test_object_seen_by_two_beams_is_kept(self):
        ranges = [2.0] * 180
        ranges[50] = ranges[51] = 0.15
        self.assertEqual((ranges, 0), filter_isolated_returns(ranges))

    def test_far_single_beam_is_kept(self):
        # A single return beyond ~0.28 m can be a real thin object edge.
        ranges = [2.0] * 180
        ranges[50] = 0.50
        self.assertEqual((ranges, 0), filter_isolated_returns(ranges))

    def test_edge_with_one_consistent_neighbour_is_kept(self):
        ranges = [math.inf] * 180
        ranges[50], ranges[51] = 0.15, 0.17
        self.assertEqual(0, filter_isolated_returns(ranges)[1])


class TestStaticPersistenceFilter(unittest.TestCase):
    def test_stationary_phantom_mover_is_demoted_after_persistence_time(self):
        persistence = StaticPersistenceFilter()
        points = [(1.0, -0.1 + 0.02 * i) for i in range(11)]
        phantom = [(x, y, 0.0, 0.2) for x, y in points]
        demoted_at = None
        for step in range(int(3.0 / DT)):
            output, demoted = persistence.filter(points, phantom, (0.0, 0.0, 0.0), step * DT)
            if demoted and demoted_at is None:
                demoted_at = step * DT
                self.assertTrue(all(len(point) == 2 for point in output))
        self.assertIsNotNone(demoted_at)
        self.assertGreaterEqual(demoted_at, config.DYNAMIC_STATIC_PERSISTENCE_SECONDS - DT)
        self.assertLess(demoted_at, config.DYNAMIC_STATIC_PERSISTENCE_SECONDS + 0.2)

    def test_slow_real_mover_is_never_demoted(self):
        # A 0.28 m face sliding sideways at the tracker's minimum speed: its
        # middle cells stay occupied for a while, but most cells are new.
        persistence = StaticPersistenceFilter()
        speed = config.DWA_DYNAMIC_MIN_SPEED
        for step in range(int(8.0 / DT)):
            now = step * DT
            points = [(1.0, -0.64 + speed * now + 0.02 * i) for i in range(15)]
            mover = [(x, y, 0.0, speed) for x, y in points]
            output, demoted = persistence.filter(points, mover, (0.0, 0.0, 0.0), now)
            self.assertEqual(0, demoted, now)
            self.assertEqual(mover, output)

    def test_held_tracks_without_scan_points_are_unchanged(self):
        persistence = StaticPersistenceFilter()
        held = [(1.0, 0.0, 0.0, -0.3)]
        for step in range(int(3.0 / DT)):
            self.assertEqual((held, 0), persistence.filter([], held, (0.0, 0.0, 0.0), step * DT))

    def test_planner_releases_a_static_wall_tracked_as_a_mover(self):
        planner = ProgressDWA()
        toolkit = planner._toolkit
        wall = [(0.90 + 0.06, 0.0, 0.12, 1.2)]

        def phantom_tracker(ranges, pose, sim_time):
            points = toolkit._scan_to_points(ranges, toolkit.lidar_field_of_view,
                                             toolkit.lidar_pose)
            return [(x, y, -0.30, 0.0) if x > 0.0 and abs(y) < 0.3 else (x, y)
                    for x, y in points]

        frames = Frames(wall, planner=planner)
        with patch.object(toolkit, "_scan_with_dynamic_obstacles", side_effect=phantom_tracker):
            frames.step((0.0, 0.0, 0.0))
            self.assertGreater(planner.diagnostics["moving"], 0)
            frames.hold((0.0, 0.0, 0.0), config.DYNAMIC_STATIC_PERSISTENCE_SECONDS + 0.2)
        self.assertEqual(0, planner.diagnostics["moving"])
        self.assertNotEqual("DYNAMIC", planner.layer)


class TestDynamicAnticipation(unittest.TestCase):
    def setUp(self):
        self.planner = ProgressDWA()

    def test_no_movers_means_no_constraint(self):
        self.assertEqual(math.inf, float(self.planner._braking_gap([0.24], [0.0], [])[0]))

    def test_stopping_in_an_approaching_movers_path_is_rejected(self):
        # Mover 1.2 m to the side, closing at 0.4 m/s: it reaches the robot
        # after the legacy 1.2 s prediction window, but a stopped robot here
        # would still be hit.
        gap = self.planner._braking_gap([0.0], [0.0], [(0.1, 1.2, 0.0, -0.4)])
        self.assertLess(float(gap[0]), DYNAMIC_SAFE_DISTANCE)

    def test_receding_mover_does_not_constrain_stopping(self):
        gap = self.planner._braking_gap([0.0], [0.0], [(0.1, 1.2, 0.0, 0.4)])
        self.assertEqual(math.inf, float(gap[0]))

    def test_braking_candidates_stay_admissible_at_cruise_speed(self):
        # A mover will cross 0.6 m ahead -- exactly where every 2.5 s
        # constant-velocity rollout ends.  The robot can still brake before
        # it, so the window must not be empty (it was with an end-of-rollout
        # stop check, which pushed the Extended run into a fleeing refuge).
        planner = self.planner
        planner.field.build([], (3.0, 0.0), (0.0, 0.0))
        planner.current_v = 0.24
        mover = [(0.6, 1.5, 0.0, -0.5)]
        best, info = planner._evaluate_window((0.0, 0.0, 0.0), [], mover,
                                              float(planner.field.cost_at(0.0, 0.0)))
        self.assertIsNotNone(best)
        self.assertGreater(info["admissible"], 0)
        trajectories, _ = planner._rollouts([(0.24, 0.0)])
        self.assertLess(float(planner._rollout_tail_clearance(trajectories, mover)[0]),
                        DYNAMIC_SAFE_DISTANCE)  # still penalised in the score

    def test_approaching_mover_caps_the_cruise_speed(self):
        planner = self.planner
        planner.field.build([], (3.0, 0.0), (0.0, 0.0))
        start_cost = float(planner.field.cost_at(0.0, 0.0))
        planner.current_v = config.DYNAMIC_CAUTION_SPEED
        _, info = planner._evaluate_window((0.0, 0.0, 0.0), [], [(1.5, 0.9, 0.0, -0.2)], start_cost)
        self.assertTrue(info["dynamic_caution"])
        self.assertLessEqual(info["v_window"][1], config.DYNAMIC_CAUTION_SPEED + 1e-9)
        _, info = planner._evaluate_window((0.0, 0.0, 0.0), [], [], start_cost)
        self.assertFalse(info["dynamic_caution"])
        self.assertGreater(info["v_window"][1], config.DYNAMIC_CAUTION_SPEED)


class TestSaferThanStopping(unittest.TestCase):
    SIDE_MOVER = (0.0, 0.6, 0.0, -0.4)  # heading straight at a stationary robot

    def wheels(self, velocity):
        return velocity / WHEEL_RADIUS, velocity / WHEEL_RADIUS

    def test_moving_out_of_the_movers_path_beats_stopping(self):
        planner = ProgressDWA()
        obstacles = [self.SIDE_MOVER]
        self.assertTrue(planner._stopping_is_unsafe(obstacles))
        left, right = self.wheels(0.2)
        self.assertFalse(planner.is_command_safe(left, right))  # legacy veto alone would STOP
        result = planner._emit((left, right, "refuge"), obstacles)
        self.assertEqual((left, right), result[:2])
        self.assertIn("safer than stopping", result[2])

    def test_static_check_is_never_relaxed(self):
        planner = ProgressDWA()
        obstacles = [self.SIDE_MOVER, (0.30, 0.0), (0.30, 0.05), (0.30, -0.05)]
        result = planner._emit((*self.wheels(0.2), "refuge"), obstacles)
        self.assertEqual((0.0, 0.0), result[:2])

    def test_without_movers_an_unsafe_command_is_stopped(self):
        planner = ProgressDWA()
        obstacles = [(0.30, 0.0), (0.30, 0.05), (0.30, -0.05)]
        self.assertFalse(planner._stopping_is_unsafe(obstacles))
        self.assertEqual((0.0, 0.0), planner._emit((*self.wheels(0.2), "x"), obstacles)[:2])


class TestGoalProjection(unittest.TestCase):
    WALL = [(1.0, -0.5 + 0.02 * i) for i in range(51)]

    def test_goal_inside_the_safety_distance_is_projected(self):
        field = CostToGoField()
        field.build(self.WALL, (0.95, 0.0), (0.0, 0.0))
        self.assertTrue(field.goal_projected)
        self.assertEqual((0.95, 0.0), tuple(field.requested_goal))
        clearance = min(math.dist(field.goal, point) for point in self.WALL)
        self.assertGreaterEqual(clearance, config.NAV_FIELD_INFLATION)
        self.assertLessEqual(math.dist(field.goal, (0.95, 0.0)), config.NAV_FIELD_GOAL_SEARCH_RADIUS)
        self.assertLess(float(field.cost_at(*field.goal)), 0.02)

    def test_reachable_goal_is_not_projected(self):
        field = CostToGoField()
        field.build(self.WALL, (0.60, 0.0), (0.0, 0.0))
        self.assertFalse(field.goal_projected)

    def test_watchdog_requires_actually_reaching_the_projected_goal(self):
        # Frozen 8 cm short of the projected goal (inside the normal 0.14 m
        # waypoint tolerance of it): still no progress, so STUCK must fire.
        wall = [(1.0 + 0.06, 0.0, 0.12, 1.0)]
        frames = Frames(wall, goal=(0.95, 0.0))
        frames.hold((0.70, 0.0, 0.0), WATCHDOG_WINDOW_SECONDS + 0.5)
        self.assertTrue(frames.planner.field.goal_projected)
        self.assertLess(math.dist((0.70, 0.0), frames.planner.field.goal), 0.14)
        self.assertGreaterEqual(frames.planner.recovery_count, 1)


class TestStaticAdmissibility(unittest.TestCase):
    def test_matches_the_legacy_veto_rule_without_buffer(self):
        import random
        rng = random.Random(3)
        planner = ProgressDWA()
        steps = max(1, round(1.2 / config.DWA_SIMULATION_STEP))
        for _ in range(100):
            fixed = [(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6)) for _ in range(4)]
            fixed = [point for point in fixed if math.hypot(*point) > 0.175]
            if not fixed:
                continue
            requested = [(rng.uniform(0.0, 0.24), rng.uniform(-1.5, 1.5)) for _ in range(10)]
            trajectories, achieved = planner._rollouts(requested)
            exact = planner._static_admissible(trajectories, fixed, buffer=0.0)
            buffered = planner._static_admissible(trajectories, fixed)
            for index, (v, w, _, _) in enumerate(achieved):
                legacy = DynamicWindowAvoidance._static_motion_is_safe(
                    DynamicWindowAvoidance._simulate(v, w)[:steps], fixed)
                self.assertEqual(legacy, bool(exact[index]))
                self.assertFalse(bool(buffered[index]) and not legacy)

    def test_dwa_does_not_plan_into_the_execution_buffer(self):
        planner = ProgressDWA()
        wall = [(x * 0.02, 0.25) for x in range(-20, 41)]  # left wall, 0.25 m away
        trajectories, _ = planner._rollouts([(0.10, 0.40), (0.10, 0.0)])
        towards, straight = planner._static_admissible(trajectories, wall)
        self.assertFalse(towards)  # would reach < 0.22 + buffer
        self.assertTrue(straight)
        # The shared veto itself accepts the approach.
        self.assertTrue(planner._toolkit._velocity_is_safe(0.10, 0.40, wall))

    def test_inside_the_buffer_driving_along_a_wall_is_allowed(self):
        planner = ProgressDWA()
        wall = [(x * 0.02, 0.225) for x in range(-20, 41)]
        trajectories, _ = planner._rollouts([(0.10, 0.0), (0.10, 0.2)])
        straight, towards = planner._static_admissible(trajectories, wall)
        self.assertTrue(straight)
        self.assertFalse(towards)


class TestPassThroughWaypoint(unittest.TestCase):
    def test_no_braking_when_the_waypoint_is_passed_at_speed(self):
        planner = ProgressDWA()
        planner.current_v = config.STATIC_DWA_MAX_SPEED
        goal = (0.4, 0.0)
        planner.field.build([], goal, (0.0, 0.0))
        best, info = planner._evaluate_window((0.0, 0.0, 0.0), [], [],
                                              float(planner.field.cost_at(0.0, 0.0)))
        self.assertAlmostEqual(config.STATIC_DWA_MAX_SPEED, best["v"], places=3)
        self.assertGreater(info["passes_waypoint"], 0)

    def test_projected_goal_is_still_approached_precisely(self):
        planner = ProgressDWA()
        planner.current_v = config.STATIC_DWA_MAX_SPEED
        wall = [(0.45, -0.5 + 0.02 * i) for i in range(51)]
        planner.field.build(wall, (0.40, 0.0), (0.0, 0.0))
        self.assertTrue(planner.field.goal_projected)
        _, info = planner._evaluate_window((0.0, 0.0, 0.0), wall, [],
                                           float(planner.field.cost_at(0.0, 0.0)))
        self.assertNotIn("passes_waypoint", info)


class TestClosedLoopScenarios(unittest.TestCase):
    """Deterministic 2D closed loop (tools/avoidance_sim.py) with LiDAR noise."""

    @classmethod
    def setUpClass(cls):
        cls.results = {name: run_case(name, "v2", seconds=90.0, seed=7)
                       for name in ("baseline", "front_left", "front_right", "frontal",
                                    "corridor", "corner", "crossing")}
        cls.mirror = {name: run_case(name, "v2", seconds=90.0, seed=None)
                      for name in ("front_left", "front_right")}

    def assert_passed(self, name):
        result = self.results[name]
        self.assertEqual(0, result["contacts"], result)
        self.assertFalse(result["stalled"], result)
        self.assertTrue(result["goal_reached"], result)
        self.assertEqual("NORMAL_DWA", result["final_state"], result)
        self.assertGreaterEqual(result["min_center_clearance"], config.DWA_ROBOT_RADIUS)

    def test_no_obstacle_baseline_needs_no_recovery(self):
        self.assert_passed("baseline")
        self.assertEqual(0, self.results["baseline"]["recoveries"])
        self.assertEqual(["NORMAL_DWA"], self.results["baseline"]["states_seen"])

    def test_front_left(self):
        self.assert_passed("front_left")

    def test_front_right(self):
        self.assert_passed("front_right")

    def test_front_left_and_right_are_mirrored(self):
        left, right = self.mirror["front_left"], self.mirror["front_right"]
        for result in (left, right):
            self.assertTrue(result["passed"], result)
        self.assertAlmostEqual(left["final_pose"][1], -right["final_pose"][1], delta=0.03)
        self.assertAlmostEqual(left["seconds"], right["seconds"], delta=1.0)
        self.assertAlmostEqual(left["min_center_clearance"], right["min_center_clearance"],
                               delta=0.01)

    def test_frontal(self):
        self.assert_passed("frontal")

    def test_corridor(self):
        self.assert_passed("corridor")

    def test_corner(self):
        self.assert_passed("corner")

    def test_dynamic_crossing(self):
        self.assert_passed("crossing")
        self.assertGreater(self.results["crossing"]["labels"].get("AVOID DYNAMIC", 0), 0)


class TestStressScenarios(unittest.TestCase):
    """Stress geometries: must be passable with margin, and v2 must pass them."""

    NAMES = ("slalom_narrow", "alternating_corners", "s_passage", "u_trap",
             "narrow_corridor", "corridor_exit_obstacle", "mixed_sizes",
             "recovery_chain", "static_dynamic_mixed", "side_approach", "endurance")

    def test_every_scenario_is_physically_passable(self):
        for name, case in CASES.items():
            with self.subTest(name=name):
                self.assertTrue(passable(case, clearance=0.25), name)

    def test_stress_scenarios_pass(self):
        for name in self.NAMES:
            with self.subTest(name=name):
                seconds = max(120.0, CASES[name].get("seconds", 0.0))
                result = run_case(name, "v2", seconds=seconds, seed=7)
                self.assertEqual(0, result["contacts"], result)
                self.assertFalse(result["stalled"], result)
                self.assertTrue(result["goal_reached"], result)
                self.assertEqual("NORMAL_DWA", result["final_state"], result)


if __name__ == "__main__":
    unittest.main()
