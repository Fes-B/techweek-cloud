import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from robot_io import RobotIO


class FakeLidar:
    def __init__(self, ranges):
        self.ranges = ranges

    def getRangeImage(self):
        return self.ranges


class FakeCamera:
    def __init__(self, width, height, raw_image):
        self.width = width
        self.height = height
        self.raw_image = raw_image

    def getImage(self):
        return self.raw_image

    def getWidth(self):
        return self.width

    def getHeight(self):
        return self.height


class FakeMotor:
    def __init__(self):
        self.velocity = None

    def getMaxVelocity(self):
        return 10.0

    def setVelocity(self, velocity):
        self.velocity = velocity


class TestRobotIOLidarOrientation(unittest.TestCase):
    def test_reversed_scan_is_normalized_once_before_sector_queries(self):
        raw_ranges = [float("inf")] * 180
        raw_ranges[135] = 0.25
        robot_io = RobotIO.__new__(RobotIO)
        robot_io.lidar = FakeLidar(raw_ranges)

        normalized = robot_io.get_lidar()
        directions = robot_io.get_lidar_directions(normalized)

        self.assertEqual(0.25, directions["right"])
        self.assertEqual(float("inf"), directions["left"])
        self.assertEqual(0.25, raw_ranges[135])

    def test_camera_bgra_is_exposed_as_bgr(self):
        robot_io = RobotIO.__new__(RobotIO)
        robot_io.camera = FakeCamera(2, 1, bytes([1, 2, 3, 99, 4, 5, 6, 88]))

        image = robot_io.get_camera_bgr()

        np.testing.assert_array_equal(
            image, np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        )

    def test_nonfinite_motor_command_fails_safe_to_stop(self):
        robot_io = RobotIO.__new__(RobotIO)
        left_motor = FakeMotor()
        right_motor = FakeMotor()
        robot_io.left_motors = [left_motor]
        robot_io.right_motors = [right_motor]

        robot_io.set_wheel_speed(float("nan"), float("inf"))

        self.assertEqual(0.0, left_motor.velocity)
        self.assertEqual(0.0, right_motor.velocity)


if __name__ == "__main__":
    unittest.main()
