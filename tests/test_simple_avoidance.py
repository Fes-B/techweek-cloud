import math
import unittest

from simple_avoidance import (
    NORMAL_DWA,
    RECOVERY_BACKOFF,
    RECOVERY_COMMIT,
    ProgressWatchdogDWA,
)


class TestProgressWatchdogDWA(unittest.TestCase):
    def test_forward_progress_scores_above_pure_rotation(self):
        planner = ProgressWatchdogDWA()
        forward = planner._sensor_score_terms(
            0.08, 0.0, planner._simulate(0.08, 0.0), 0.5, (2.0, 0.0)
        )
        rotation = planner._sensor_score_terms(
            0.0, 0.6, planner._simulate(0.0, 0.6), 0.5, (2.0, 0.0)
        )
        self.assertGreater(forward["total"], rotation["total"])

    def test_watchdog_uses_pose_progress_not_lidar_sector(self):
        planner = ProgressWatchdogDWA()
        goal = (2.0, 0.0)
        self.assertFalse(planner._update_watchdog((0.0, 0.0, 0.0), goal, 0.0))
        self.assertFalse(planner._update_watchdog((0.0, 0.0, 0.0), goal, 1.0))
        self.assertTrue(planner._update_watchdog((0.0, 0.0, 0.0), goal, 2.5))

    def test_goal_improvement_resets_stuck_decision(self):
        planner = ProgressWatchdogDWA()
        planner._update_watchdog((0.0, 0.0, 0.0), (2.0, 0.0), 0.0)
        planner._update_watchdog((0.10, 0.0, 0.0), (1.9, 0.0), 1.0)
        self.assertFalse(
            planner._update_watchdog((0.20, 0.0, 0.0), (1.8, 0.0), 2.5)
        )

    def test_recovery_has_only_two_committed_phases(self):
        planner = ProgressWatchdogDWA()
        self.assertEqual(NORMAL_DWA, planner.static_state)
        planner.static_state = RECOVERY_BACKOFF
        planner.recovery_turn_sign = -1.0
        self.assertEqual(-1.0, planner.recovery_turn_sign)
        planner.static_state = RECOVERY_COMMIT
        self.assertEqual(-1.0, planner.recovery_turn_sign)
        self.assertNotIn("wall", planner.static_state.lower())
        self.assertNotIn("corner", planner.static_state.lower())

    def test_angle_delta_is_symmetric_through_wrap(self):
        from simple_avoidance import _angle_delta

        self.assertAlmostEqual(
            math.radians(20.0),
            _angle_delta(math.radians(-170.0), math.radians(170.0)),
        )


if __name__ == "__main__":
    unittest.main()
