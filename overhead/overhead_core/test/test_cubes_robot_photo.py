"""Regression test verifying cube detection on the arena photo containing a robot."""

from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from overhead_core.constants import (
    DEFAULT_CUBE_HSV_RANGES,
    DEFAULT_MIN_AREA_PX,
    MAX_CUBE_AREA_PX,
    CubeColor,
)
from overhead_core.cubes import detect_cubes

FIXTURE_PATH: Path = Path(__file__).parent / "fixtures" / "arena_robot_2026-11-24.jpg"

EXPECTED_COUNTS: dict[CubeColor, int] = {
    CubeColor.RED: 4,
    CubeColor.YELLOW: 6,
    CubeColor.BLUE: 4,
}


def _load_bgr_image(image_path: Path) -> np.ndarray:
    """Loads a BGR image from a path that may contain non-ASCII characters."""
    raw_bytes = np.fromfile(
        str(image_path),
        dtype=np.uint8,
    )
    frame = cv2.imdecode(
        raw_bytes,
        cv2.IMREAD_COLOR,
    )
    assert frame is not None
    return frame


def test_should_reject_robot_triangle_and_detect_cubes_when_robot_photo_given() -> None:
    frame = _load_bgr_image(FIXTURE_PATH)

    detections = detect_cubes(
        frame,
        None,
        DEFAULT_CUBE_HSV_RANGES,
        min_area_px=DEFAULT_MIN_AREA_PX,
        max_area_px=MAX_CUBE_AREA_PX,
    )

    counts_by_color = dict(Counter(CubeColor(det[0]) for det in detections))
    assert counts_by_color == EXPECTED_COUNTS
    assert all(DEFAULT_MIN_AREA_PX <= det[3] <= MAX_CUBE_AREA_PX for det in detections)
