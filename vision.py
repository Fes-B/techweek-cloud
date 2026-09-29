"""Configurable HSV target detection for the practice camera target."""

import math

import cv2
import numpy as np

from config import (
    VISION_MAX_ASPECT_RATIO,
    VISION_MIN_AREA_PIXELS,
    VISION_MIN_CIRCULARITY,
    VISION_MORPH_KERNEL_SIZE,
    VISION_RED_HIGH_1,
    VISION_RED_HIGH_2,
    VISION_RED_LOW_1,
    VISION_RED_LOW_2,
)


def _not_detected():
    return {"detected": False}


class TargetDetector:
    """Detect the configurable red circular or square practice target."""

    def detect(self, camera_image):
        if camera_image is None:
            return _not_detected()
        image = np.asarray(camera_image)
        if image.ndim != 3 or image.shape[2] < 3:
            raise ValueError("camera_image must be a BGR image with three channels")

        bgr = image[:, :, :3].astype(np.uint8, copy=False)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array(VISION_RED_LOW_1), np.array(VISION_RED_HIGH_1)),
            cv2.inRange(hsv, np.array(VISION_RED_LOW_2), np.array(VISION_RED_HIGH_2)),
        )
        kernel = np.ones((VISION_MORPH_KERNEL_SIZE, VISION_MORPH_KERNEL_SIZE), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < VISION_MIN_AREA_PIXELS:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            aspect_ratio = max(width, height) / max(1, min(width, height))
            perimeter = cv2.arcLength(contour, True)
            circularity = 0.0 if perimeter <= 0.0 else 4.0 * math.pi * area / perimeter**2
            if aspect_ratio > VISION_MAX_ASPECT_RATIO or circularity < VISION_MIN_CIRCULARITY:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0.0:
                continue
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]
            score = area * circularity
            if best is None or score > best[0]:
                best = (score, center_x, center_y, area, circularity, x, y, width, height)

        if best is None:
            return _not_detected()
        _, center_x, center_y, area, circularity, x, y, width, height = best
        image_area = max(1, image.shape[0] * image.shape[1])
        return {
            "detected": True,
            "center_x": center_x,
            "center_y": center_y,
            "area": area,
            "confidence": min(1.0, circularity * area / (0.05 * image_area)),
            "bbox": (x, y, width, height),
        }
