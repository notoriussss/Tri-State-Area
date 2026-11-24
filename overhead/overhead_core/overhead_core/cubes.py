"""Cube detection in HSV color space within the arena."""

from typing import Any

import cv2
import numpy as np

from overhead_core.constants import (
    DEFAULT_MIN_AREA_PX,
    MAX_CUBE_AREA_PX,
)


def detect_cubes(
    image_bgr: np.ndarray,
    arena_mask: np.ndarray | None,
    hsv_ranges: dict[str, list[tuple[Any, Any]]],
    *,
    min_area_px: int = DEFAULT_MIN_AREA_PX,
    max_area_px: int = MAX_CUBE_AREA_PX,
) -> list[tuple[str, float, float, float]]:
    """Detects colored cubes within the arena bounds.

    Args:
        image_bgr: Raw input BGR image.
        arena_mask: Binary mask with 255 inside the arena and 0 outside (or None).
        hsv_ranges: Dictionary mapping color names to lists of (lower_hsv, upper_hsv) bounds.
        min_area_px: Minimum contour area in pixels to be considered a cube.
        max_area_px: Maximum contour area in pixels; anything larger is rejected as a
            non-cube blob (for example, a robot marker).

    Returns:
        List of detections formatted as (color, u_px, v_px, area_px).
    """
    if image_bgr is None or image_bgr.size == 0:
        return []

    hsv = cv2.cvtColor(
        image_bgr,
        cv2.COLOR_BGR2HSV,
    )
    detections: list[tuple[str, float, float, float]] = []

    for color, ranges in hsv_ranges.items():
        color_mask = np.zeros(
            hsv.shape[:2],
            dtype=np.uint8,
        )
        for lower_b, upper_b in ranges:
            lower_arr = np.asarray(
                lower_b,
                dtype=np.uint8,
            )
            upper_arr = np.asarray(
                upper_b,
                dtype=np.uint8,
            )
            sub_mask = cv2.inRange(
                hsv,
                lower_arr,
                upper_arr,
            )
            color_mask = cv2.bitwise_or(
                color_mask,
                sub_mask,
            )

        if arena_mask is not None:
            color_mask = cv2.bitwise_and(
                color_mask,
                arena_mask,
            )

        contours, _ = cv2.findContours(
            color_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area_px or area > max_area_px:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] <= 0.0:
                continue

            u_px = float(moments["m10"] / moments["m00"])
            v_px = float(moments["m01"] / moments["m00"])
            detections.append(
                (
                    str(color),
                    u_px,
                    v_px,
                    float(area),
                )
            )

    return detections
