"""Pose adapter contract with encoder distance and independently measured heading."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from config import WHEEL_RADIUS
from practice_odometry import EncoderPose


class Device:
    value = 0.0
    north = (1.0, 0.0, 0.0)

    def enable(self, _):
        pass

    def getValue(self):
        return self.value

    def getValues(self):
        return self.north


class Robot:
    def __init__(self):
        self.devices = {name: Device() for name in
                        ('left wheel sensor', 'right wheel sensor', 'heading compass')}

    def getDevice(self, name):
        return self.devices[name]


class TestPracticeOdometry(unittest.TestCase):
    def test_encoder_translation_and_compass_turn_from_given_start(self):
        robot = Robot()
        adapter = EncoderPose(robot, (2.0, 3.0, 0.0))
        adapter.update()
        left, right = adapter.left, adapter.right
        left.value = right.value = 1.0 / WHEEL_RADIUS
        self.assertAlmostEqual(adapter.update()[0], 3.0)
        adapter.compass.north = (0.0, -1.0, 0.0)
        self.assertAlmostEqual(adapter.update()[2], math.pi / 2)
        left.value = right.value = 2.0 / WHEEL_RADIUS
        x, y, _ = adapter.update()
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, 4.0)


if __name__ == '__main__':
    unittest.main()
