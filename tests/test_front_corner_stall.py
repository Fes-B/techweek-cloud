import sys
import unittest
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from tools.diagnose_front_corner_stall import run_case


class TestFrontCornerStallRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = {}
        cls.frames = {}
        for name in ("front_left", "front_right", "front", "side"):
            cls.results[name], cls.frames[name] = run_case(name, seconds=45.0)

    def assert_passed_without_contact(self, name):
        result = self.results[name]
        self.assertEqual(0, result["contacts"])
        self.assertFalse(result["stalled"])
        self.assertTrue(result["obstacle_passed"])
        self.assertTrue(result["path_following_returned"])
        self.assertTrue(result["goal_reached"])

    def test_front_left_diagonal_passes(self):
        self.assert_passed_without_contact("front_left")

    def test_front_right_diagonal_passes(self):
        self.assert_passed_without_contact("front_right")

    def test_diagonal_cases_are_mirrored(self):
        left = self.results["front_left"]
        right = self.results["front_right"]
        self.assertAlmostEqual(
            abs(left["final_pose"][1]), abs(right["final_pose"][1]), delta=0.03
        )
        self.assertAlmostEqual(
            left["minimum_clearance"], right["minimum_clearance"], delta=0.005
        )

    def test_front_obstacle_preserves_avoidance(self):
        self.assert_passed_without_contact("front")

    def test_side_obstacle_does_not_activate_corner_recovery(self):
        self.assert_passed_without_contact("side")
        self.assertFalse(
            any(frame["front_corner_escape_active"] for frame in self.frames["side"])
        )
        self.assertFalse(any(frame["recovery_phase"] for frame in self.frames["side"]))


if __name__ == "__main__":
    unittest.main()
