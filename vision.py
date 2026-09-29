"""Configurable HSV target detection for the red circular or square target.

Camera format: Webots returns BGRA frames (verified against the installed
Webots R2025a controller library -- see the VISION note in config.py), so
robot_io.get_camera_bgr() hands this module BGR.  4-channel input is accepted
directly as well.

The detector is deliberately plain OpenCV: two HSV hue ranges for red, light
morphology, then per-contour geometry.  It never raises on a bad frame -- a
control loop must not die because one camera frame was unusable -- and always
returns the same dictionary schema.
"""

import math

import cv2
import numpy as np

from config import (
    VISION_BORDER_MARGIN_PIXELS,
    VISION_CIRCLE_FILL_RANGE,
    VISION_CLIPPED_MAX_ASPECT_RATIO,
    VISION_CLIPPED_MIN_SOLIDITY,
    VISION_CONFIDENT_AREA_PIXELS,
    VISION_MAX_ASPECT_RATIO,
    VISION_MIN_AREA_PIXELS,
    VISION_MIN_CIRCULARITY,
    VISION_MIN_SOLIDITY,
    VISION_MIN_SQUARE_FILL,
    VISION_MORPH_KERNEL_SIZE,
    VISION_RED_HIGH_1,
    VISION_RED_HIGH_2,
    VISION_RED_LOW_1,
    VISION_RED_LOW_2,
)


def _not_detected():
    """Negative result.  Same keys as a positive one so callers never KeyError."""
    return {
        "detected": False,
        "center_x": None,
        "center_y": None,
        "bbox": None,
        "area": 0.0,
        "area_ratio": 0.0,
        "confidence": 0.0,
        "offset_x": 0.0,
        "offset_y": 0.0,
        "shape": None,
        "clipped": False,
    }


def _clamp01(value):
    return 0.0 if value < 0.0 else 1.0 if value > 1.0 else float(value)


def _clamp_signed(value):
    return -1.0 if value < -1.0 else 1.0 if value > 1.0 else float(value)


def _as_bgr(camera_image):
    """Return a contiguous uint8 BGR image, or None if the input is unusable."""
    if camera_image is None:
        return None
    image = np.asarray(camera_image)
    if image.ndim != 3 or image.shape[2] < 3:
        return None
    if image.shape[0] < 1 or image.shape[1] < 1:
        return None
    if image.dtype != np.uint8:
        if not np.issubdtype(image.dtype, np.number):
            return None
        image = np.clip(image, 0, 255).astype(np.uint8)
    # Drop alpha (BGRA -> BGR); cv2 needs a contiguous buffer.
    return np.ascontiguousarray(image[:, :, :3])


class TargetDetector:
    """Detect the configurable red circular or square target."""

    def red_mask(self, bgr):
        """Two-range red HSV mask with light, small-target-preserving cleanup."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array(VISION_RED_LOW_1), np.array(VISION_RED_HIGH_1)),
            cv2.inRange(hsv, np.array(VISION_RED_LOW_2), np.array(VISION_RED_HIGH_2)),
        )
        if VISION_MORPH_KERNEL_SIZE > 1:
            size = int(VISION_MORPH_KERNEL_SIZE)
            # A cross is gentler than a full square: it still removes isolated
            # speckle but keeps a few-pixel-wide distant target intact.
            kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (size, size))
            # Close before open so a target split by an anti-aliased seam is
            # rejoined before the opening can nibble either half away.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def _measure(self, contour, mask, image_shape):
        """Geometry and colour metrics for one contour, or None if degenerate."""
        area = float(cv2.contourArea(contour))
        if area < VISION_MIN_AREA_PIXELS:
            return None

        x, y, width, height = cv2.boundingRect(contour)
        if width < 1 or height < 1:
            return None

        perimeter = float(cv2.arcLength(contour, True))
        circularity = 0.0 if perimeter <= 0.0 else 4.0 * math.pi * area / perimeter**2

        # Rotation-invariant box: a square fills it at any angle, a disc fills
        # pi/4 of it.  This is what lets circle and square be judged apart
        # instead of forcing both through one circularity threshold.
        (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(contour)
        if rect_w <= 0.0 or rect_h <= 0.0:
            return None
        aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)
        rect_fill = _clamp01(area / (rect_w * rect_h))

        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = _clamp01(area / hull_area) if hull_area > 0.0 else 0.0

        moments = cv2.moments(contour)
        if moments["m00"] > 0.0:
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]
        else:
            center_x = x + 0.5 * width
            center_y = y + 0.5 * height

        rows, columns = image_shape[0], image_shape[1]
        margin = VISION_BORDER_MARGIN_PIXELS
        clipped = (
            x <= margin
            or y <= margin
            or x + width >= columns - margin
            or y + height >= rows - margin
        )

        # Colour quality: how completely the bounding box's red pixels form one
        # solid blob.  A crisp target fills its own contour with red; a noisy
        # red texture does not.
        window = mask[y:y + height, x:x + width]
        red_pixels = float(cv2.countNonZero(window)) if window.size else 0.0
        color_score = _clamp01(area / red_pixels) if red_pixels > 0.0 else 0.0

        return {
            "area": area,
            "bbox": (int(x), int(y), int(width), int(height)),
            "center_x": float(center_x),
            "center_y": float(center_y),
            "circularity": circularity,
            "aspect_ratio": aspect_ratio,
            "rect_fill": rect_fill,
            "solidity": solidity,
            "clipped": clipped,
            "color_score": color_score,
        }

    def _classify(self, metrics):
        """(shape, shape_score) for an accepted candidate, or (None, None)."""
        circle_low, circle_high = VISION_CIRCLE_FILL_RANGE

        if metrics["clipped"]:
            # A clipped target is a convex fragment: its circularity and side
            # ratio are cut by the image border, so only convexity is required
            # and the shape is not claimed.
            if (
                metrics["solidity"] >= VISION_CLIPPED_MIN_SOLIDITY
                and metrics["aspect_ratio"] <= VISION_CLIPPED_MAX_ASPECT_RATIO
            ):
                span = max(1e-6, 1.0 - VISION_CLIPPED_MIN_SOLIDITY)
                score = (metrics["solidity"] - VISION_CLIPPED_MIN_SOLIDITY) / span
                return None, _clamp01(score)
            return None, None

        if metrics["aspect_ratio"] > VISION_MAX_ASPECT_RATIO:
            return None, None
        if metrics["solidity"] < VISION_MIN_SOLIDITY:
            return None, None

        is_circle = (
            metrics["circularity"] >= VISION_MIN_CIRCULARITY
            and circle_low <= metrics["rect_fill"] <= circle_high
        )
        is_square = metrics["rect_fill"] >= VISION_MIN_SQUARE_FILL

        if is_square and not is_circle:
            span = max(1e-6, 1.0 - VISION_MIN_SQUARE_FILL)
            score = (metrics["rect_fill"] - VISION_MIN_SQUARE_FILL) / span
            return "square", _clamp01(score)
        if is_circle and not is_square:
            span = max(1e-6, 1.0 - VISION_MIN_CIRCULARITY)
            score = (metrics["circularity"] - VISION_MIN_CIRCULARITY) / span
            return "circle", _clamp01(score)
        if is_circle and is_square:
            # Shape ambiguous (the fill ranges only overlap if they are
            # reconfigured to overlap): accept the target, claim no shape.
            return None, 0.5
        return None, None

    def detect(self, camera_image):
        bgr = _as_bgr(camera_image)
        if bgr is None:
            return _not_detected()

        rows, columns = bgr.shape[0], bgr.shape[1]
        mask = self.red_mask(bgr)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        for contour in contours:
            metrics = self._measure(contour, mask, bgr.shape)
            if metrics is None:
                continue
            shape, shape_score = self._classify(metrics)
            if shape_score is None:
                continue

            size_score = _clamp01(metrics["area"] / VISION_CONFIDENT_AREA_PIXELS)
            solidity_score = _clamp01((metrics["solidity"] - 0.70) / 0.30)
            quality = _clamp01(
                0.50 * shape_score
                + 0.25 * solidity_score
                + 0.25 * metrics["color_score"]
            )
            # Selection weighs shape/colour quality together with size, so a
            # large ragged blob does not beat a clean target on area alone.
            selection = quality * (0.35 + 0.65 * size_score)
            # Fully deterministic ordering, including exact ties.
            key = (selection, metrics["area"], -metrics["center_x"], -metrics["center_y"])
            if best is None or key > best[0]:
                best = (key, metrics, shape, quality, size_score)

        if best is None:
            return _not_detected()

        _, metrics, shape, quality, size_score = best
        image_area = max(1, rows * columns)
        half_width = max(1.0, columns / 2.0)
        half_height = max(1.0, rows / 2.0)
        return {
            "detected": True,
            "center_x": metrics["center_x"],
            "center_y": metrics["center_y"],
            "bbox": metrics["bbox"],
            "area": metrics["area"],
            "area_ratio": metrics["area"] / image_area,
            # Bounded deterministic quality, never a raw area value.
            "confidence": _clamp01(0.70 * quality + 0.30 * size_score),
            "offset_x": _clamp_signed((metrics["center_x"] - columns / 2.0) / half_width),
            "offset_y": _clamp_signed((metrics["center_y"] - rows / 2.0) / half_height),
            "shape": shape,
            "clipped": metrics["clipped"],
        }

