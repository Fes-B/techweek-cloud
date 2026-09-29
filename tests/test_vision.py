"""Target detection contract: red circle or square, robust to noise and borders.

Images are built in BGR, the order robot_io.get_camera_bgr() produces from
Webots' BGRA frames, so pure red is (0, 0, 255).
"""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from vision import TargetDetector


RED = (0, 0, 255)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
WIDTH, HEIGHT = 320, 240
RESULT_FIELDS = {
    "detected", "center_x", "center_y", "bbox", "area", "area_ratio",
    "confidence", "offset_x", "offset_y", "shape", "clipped",
}


def blank():
    return np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)


def with_circle(center, radius, colour=RED):
    image = blank()
    cv2.circle(image, center, radius, colour, -1)
    return image


def with_square(center, half, colour=RED):
    image = blank()
    cv2.rectangle(
        image,
        (center[0] - half, center[1] - half),
        (center[0] + half, center[1] + half),
        colour,
        -1,
    )
    return image


class TestTargetDetectorInput(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_none_input_is_not_detected(self):
        result = self.detector.detect(None)

        self.assertFalse(result["detected"])
        self.assertEqual(RESULT_FIELDS, set(result))

    def test_invalid_images_are_not_detected_instead_of_raising(self):
        cases = {
            "2d grayscale": np.zeros((HEIGHT, WIDTH), dtype=np.uint8),
            "two channels": np.zeros((HEIGHT, WIDTH, 2), dtype=np.uint8),
            "zero size": np.zeros((0, 0, 3), dtype=np.uint8),
            "4d": np.zeros((2, HEIGHT, WIDTH, 3), dtype=np.uint8),
        }
        for name, image in cases.items():
            with self.subTest(name):
                self.assertFalse(self.detector.detect(image)["detected"])

    def test_bgra_four_channel_input_uses_channel_two_as_red(self):
        """Webots frames are BGRA: red is byte offset +2, blue is +0."""
        bgra = np.zeros((HEIGHT, WIDTH, 4), dtype=np.uint8)
        bgra[:, :, 3] = 255
        cv2.circle(bgra, (160, 120), 35, (0, 0, 255, 255), -1)

        self.assertTrue(self.detector.detect(bgra)["detected"])

        blue_bgra = np.zeros((HEIGHT, WIDTH, 4), dtype=np.uint8)
        blue_bgra[:, :, 3] = 255
        cv2.circle(blue_bgra, (160, 120), 35, (255, 0, 0, 255), -1)

        self.assertFalse(self.detector.detect(blue_bgra)["detected"])

    def test_blank_image_has_no_target(self):
        self.assertFalse(self.detector.detect(blank())["detected"])


class TestTargetDetectorShapes(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_centered_red_circle_is_detected_as_a_circle(self):
        result = self.detector.detect(with_circle((160, 120), 35))

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(160, result["center_x"], delta=2)
        self.assertAlmostEqual(120, result["center_y"], delta=2)
        self.assertEqual("circle", result["shape"])
        self.assertAlmostEqual(0.0, result["offset_x"], delta=0.02)
        self.assertAlmostEqual(0.0, result["offset_y"], delta=0.02)
        self.assertFalse(result["clipped"])
        self.assertEqual(RESULT_FIELDS, set(result))

    def test_centered_red_square_is_detected_as_a_square(self):
        result = self.detector.detect(with_square((160, 120), 30))

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(160, result["center_x"], delta=2)
        self.assertAlmostEqual(120, result["center_y"], delta=2)
        self.assertEqual("square", result["shape"])
        self.assertFalse(result["clipped"])

    def test_rotated_red_square_is_still_accepted(self):
        """The square gate is rotation invariant, so a tilted panel still passes."""
        image = blank()
        corners = np.array([[160, 80], [200, 120], [160, 160], [120, 120]], dtype=np.int32)
        cv2.fillPoly(image, [corners], RED)

        result = self.detector.detect(image)

        self.assertTrue(result["detected"])
        self.assertEqual("square", result["shape"])

    def test_circle_and_square_are_both_accepted_without_one_shared_circularity_gate(self):
        circle = self.detector.detect(with_circle((160, 120), 35))
        square = self.detector.detect(with_square((160, 120), 30))

        self.assertTrue(circle["detected"])
        self.assertTrue(square["detected"])
        self.assertNotEqual(circle["shape"], square["shape"])


class TestTargetDetectorPosition(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_target_on_the_left_reports_negative_offset_x(self):
        result = self.detector.detect(with_circle((60, 120), 25))

        self.assertTrue(result["detected"])
        self.assertLess(result["offset_x"], -0.3)
        self.assertAlmostEqual(0.0, result["offset_y"], delta=0.05)

    def test_target_on_the_right_reports_positive_offset_x(self):
        result = self.detector.detect(with_circle((260, 120), 25))

        self.assertTrue(result["detected"])
        self.assertGreater(result["offset_x"], 0.3)

    def test_offsets_stay_within_the_normalised_range(self):
        for center in ((5, 5), (315, 235), (160, 120)):
            with self.subTest(center):
                result = self.detector.detect(with_circle(center, 20))
                self.assertGreaterEqual(result["offset_x"], -1.0)
                self.assertLessEqual(result["offset_x"], 1.0)
                self.assertGreaterEqual(result["offset_y"], -1.0)
                self.assertLessEqual(result["offset_y"], 1.0)

    def test_small_distant_target_is_detected(self):
        result = self.detector.detect(with_circle((160, 120), 6))

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(160, result["center_x"], delta=2)
        self.assertLess(result["area_ratio"], 0.01)

    def test_partially_clipped_target_is_detected_and_flagged(self):
        """Circularity drops at the border; convexity must carry the detection."""
        for center in ((5, 120), (315, 120), (160, 4)):
            with self.subTest(center):
                result = self.detector.detect(with_circle(center, 35))
                self.assertTrue(result["detected"])
                self.assertTrue(result["clipped"])


class TestTargetDetectorRejection(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_tiny_red_noise_is_rejected(self):
        image = blank()
        for center in ((40, 40), (160, 120), (280, 200), (100, 190)):
            cv2.circle(image, center, 2, RED, -1)

        self.assertFalse(self.detector.detect(image)["detected"])

    def test_single_red_pixel_speckle_is_rejected(self):
        image = blank()
        image[60, 60] = RED
        image[61, 200] = RED

        self.assertFalse(self.detector.detect(image)["detected"])

    def test_elongated_red_object_is_rejected(self):
        image = blank()
        cv2.rectangle(image, (110, 114), (210, 126), RED, -1)

        self.assertFalse(self.detector.detect(image)["detected"])

    def test_irregular_red_cross_is_rejected(self):
        image = blank()
        cv2.rectangle(image, (120, 112), (200, 128), RED, -1)
        cv2.rectangle(image, (152, 80), (168, 160), RED, -1)

        self.assertFalse(self.detector.detect(image)["detected"])

    def test_non_red_circles_are_rejected(self):
        for name, colour in (("blue", BLUE), ("green", GREEN)):
            with self.subTest(name):
                result = self.detector.detect(with_circle((160, 120), 35, colour))
                self.assertFalse(result["detected"])

    def test_dark_desaturated_red_is_rejected_by_the_hsv_thresholds(self):
        result = self.detector.detect(with_circle((160, 120), 35, (40, 40, 70)))

        self.assertFalse(result["detected"])


class TestTargetDetectorSelection(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_largest_of_two_valid_targets_is_selected(self):
        image = blank()
        cv2.circle(image, (60, 60), 10, RED, -1)
        cv2.circle(image, (250, 160), 30, RED, -1)

        result = self.detector.detect(image)

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(250, result["center_x"], delta=2)
        self.assertAlmostEqual(160, result["center_y"], delta=2)

    def test_larger_irregular_blob_does_not_beat_a_smaller_clean_target(self):
        """Area alone would pick the cross; shape quality must pick the circle."""
        image = blank()
        cv2.rectangle(image, (20, 100), (140, 140), RED, -1)
        cv2.rectangle(image, (60, 40), (100, 200), RED, -1)
        cv2.circle(image, (250, 120), 22, RED, -1)

        result = self.detector.detect(image)

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(250, result["center_x"], delta=3)
        self.assertEqual("circle", result["shape"])

    def test_elongated_bar_larger_than_the_target_is_not_selected(self):
        image = blank()
        cv2.rectangle(image, (10, 30), (300, 55), RED, -1)
        cv2.circle(image, (200, 170), 20, RED, -1)

        result = self.detector.detect(image)

        self.assertTrue(result["detected"])
        self.assertAlmostEqual(200, result["center_x"], delta=3)
        self.assertAlmostEqual(170, result["center_y"], delta=3)

    def test_selection_is_deterministic_for_identical_repeated_calls(self):
        image = blank()
        cv2.circle(image, (100, 120), 25, RED, -1)
        cv2.circle(image, (220, 120), 25, RED, -1)

        first = self.detector.detect(image)
        second = TargetDetector().detect(image)

        self.assertEqual(first, second)


class TestTargetDetectorConfidence(unittest.TestCase):
    def setUp(self):
        self.detector = TargetDetector()

    def test_confidence_is_bounded_and_not_a_raw_area(self):
        big = self.detector.detect(with_circle((160, 120), 70))
        small = self.detector.detect(with_circle((160, 120), 7))

        for result in (big, small):
            self.assertTrue(result["detected"])
            self.assertGreaterEqual(result["confidence"], 0.0)
            self.assertLessEqual(result["confidence"], 1.0)
        self.assertGreater(big["confidence"], small["confidence"])

    def test_clean_large_target_is_confident(self):
        result = self.detector.detect(with_circle((160, 120), 40))

        self.assertGreater(result["confidence"], 0.6)

    def test_area_ratio_matches_the_image_size(self):
        result = self.detector.detect(with_circle((160, 120), 40))

        self.assertAlmostEqual(
            result["area"] / (WIDTH * HEIGHT), result["area_ratio"], places=9
        )


if __name__ == "__main__":
    unittest.main()
