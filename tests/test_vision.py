import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from vision import TargetDetector


class TestTargetDetector(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_detects_large_red_circle(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(image, (160, 120), 35, (0, 0, 255), -1)

        result = self.detector.detect(image)

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(160, result["center_x"], delta=2)
        self.assertAlmostEqual(120, result["center_y"], delta=2)

    def test_rejects_blank_small_noise_and_blue_objects(self):
        blank = np.zeros((240, 320, 3), dtype=np.uint8)
        small_red = blank.copy()
        cv2.circle(small_red, (160, 120), 3, (0, 0, 255), -1)
        blue = blank.copy()
        cv2.circle(blue, (160, 120), 35, (255, 0, 0), -1)

        self.assertFalse(self.detector.detect(blank)["detected"])
        self.assertFalse(self.detector.detect(small_red)["detected"])
        self.assertFalse(self.detector.detect(blue)["detected"])

    def test_selects_the_largest_valid_red_target(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(image, (50, 50), 4, (0, 0, 255), -1)
        cv2.circle(image, (250, 160), 30, (0, 0, 255), -1)

        result = self.detector.detect(image)

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(250, result["center_x"], delta=2)
        self.assertAlmostEqual(160, result["center_y"], delta=2)
