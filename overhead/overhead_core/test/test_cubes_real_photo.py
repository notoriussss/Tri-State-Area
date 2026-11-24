"""Regression tests for cube detection on a real overhead camera photo."""

import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from overhead_core.constants import (
    DEFAULT_CUBE_HSV_RANGES,
    DEFAULT_MIN_AREA_PX,
    CubeColor,
)
from overhead_core.cubes import detect_cubes

FIXTURE_PATH: Path = Path(__file__).parent / "fixtures" / "arena_real_2026-11-24.jpg"
MAX_CENTER_ERROR_PX: float = 8.0

EXPECTED_COUNTS: dict[CubeColor, int] = {
    CubeColor.RED: 5,
    CubeColor.YELLOW: 7,
    CubeColor.BLUE: 4,
}

EXPECTED_CENTERS_PX: dict[CubeColor, tuple[tuple[int, int], ...]] = {
    CubeColor.RED: (
        (466, 554),
        (500, 380),
        (626, 511),
        (782, 87),
        (1006, 104),
    ),
    CubeColor.YELLOW: (
        (84, 243),
        (372, 567),
        (396, 195),
        (587, 97),
        (774, 314),
        (808, 579),
        (1258, 314),
    ),
    CubeColor.BLUE: (
        (259, 366),
        (996, 309),
        (1054, 597),
        (1234, 377),
    ),
}


def _load_bgr_image(image_path: Path) -> np.ndarray:
    """Loads a BGR image from a path that may contain non-ASCII characters."""
    # np.fromfile + cv2.imdecode avoids OpenCV path encoding failures on Windows.
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


def _nearest_center_distance_px(
    detections: list[tuple[str, float, float, float]],
    color: CubeColor,
    expected_xy_px: tuple[int, int],
) -> float:
    """Returns minimum Euclidean distance in pixels to a matching-color detection."""
    expected_u_px, expected_v_px = expected_xy_px
    distances_px = [
        math.hypot(
            det_u_px - expected_u_px,
            det_v_px - expected_v_px,
        )
        for det_color, det_u_px, det_v_px, _ in detections
        if det_color == color
    ]
    if not distances_px:
        return math.inf

    return min(distances_px)


def test_should_detect_expected_cubes_when_given_real_arena_photo() -> None:
    frame = _load_bgr_image(FIXTURE_PATH)

    detections = detect_cubes(
        frame,
        None,
        DEFAULT_CUBE_HSV_RANGES,
        min_area_px=DEFAULT_MIN_AREA_PX,
    )

    counts_by_color = dict(Counter(CubeColor(det[0]) for det in detections))
    assert counts_by_color == EXPECTED_COUNTS
    for color, centers_px in EXPECTED_CENTERS_PX.items():
        for expected_xy_px in centers_px:
            distance_px = _nearest_center_distance_px(
                detections,
                color,
                expected_xy_px,
            )
            assert distance_px <= MAX_CENTER_ERROR_PX
