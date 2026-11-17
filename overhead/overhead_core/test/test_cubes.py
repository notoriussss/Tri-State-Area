"""Unit tests for cube detection in synthetic images."""

import cv2
import numpy as np
from overhead_core.cubes import detect_cubes

HSV_RANGES = {
    "yellow": [
        ((20, 100, 100), (35, 255, 255)),
    ],
    "red": [
        ((0, 100, 100), (10, 255, 255)),
        ((170, 100, 100), (180, 255, 255)),
    ],
}


def test_should_detect_yellow_and_red_cubes_when_both_present_in_image():
    image = np.zeros(
        (200, 300, 3),
        dtype=np.uint8,
    )
    # BGR yellow square: (50, 50) to (70, 70) -> area 400 px, center (60, 60)
    cv2.rectangle(
        image,
        (50, 50),
        (70, 70),
        (0, 255, 255),
        -1,
    )
    # BGR red square: (150, 100) to (170, 120) -> area 400 px, center (160, 110)
    cv2.rectangle(
        image,
        (150, 100),
        (170, 120),
        (0, 0, 255),
        -1,
    )

    detections = detect_cubes(
        image,
        arena_mask=None,
        hsv_ranges=HSV_RANGES,
        min_area_px=150,
    )

    detected_colors = {d[0] for d in detections}
    assert detected_colors == {"yellow", "red"}
    yellow_det = next(d for d in detections if d[0] == "yellow")
    red_det = next(d for d in detections if d[0] == "red")
    assert np.isclose(yellow_det[1], 60.0, atol=2.0)
    assert np.isclose(yellow_det[2], 60.0, atol=2.0)
    assert np.isclose(red_det[1], 160.0, atol=2.0)
    assert np.isclose(red_det[2], 110.0, atol=2.0)


def test_should_ignore_small_blobs_when_area_below_threshold():
    image = np.zeros(
        (200, 300, 3),
        dtype=np.uint8,
    )
    # Tiny 5x5 yellow square (area 25 < 150)
    cv2.rectangle(
        image,
        (50, 50),
        (55, 55),
        (0, 255, 255),
        -1,
    )

    detections = detect_cubes(
        image,
        arena_mask=None,
        hsv_ranges=HSV_RANGES,
        min_area_px=150,
    )

    assert len(detections) == 0


def test_should_filter_outside_arena_when_mask_provided():
    image = np.zeros(
        (200, 300, 3),
        dtype=np.uint8,
    )
    cv2.rectangle(
        image,
        (50, 50),
        (70, 70),
        (0, 255, 255),
        -1,
    )
    # Mask covers only x >= 100, excluding the cube at x=50..70
    mask = np.zeros(
        (200, 300),
        dtype=np.uint8,
    )
    mask[:, 100:] = 255

    detections = detect_cubes(
        image,
        arena_mask=mask,
        hsv_ranges=HSV_RANGES,
        min_area_px=150,
    )

    assert len(detections) == 0
