"""Unit tests for pure arena calibration attempts."""

from pathlib import Path

import cv2
import numpy as np
from overhead_core.arena_calibration import (
    CalibrationResult,
    calibrate_from_frame,
)
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    LOG_CALIBRATION_INVALID_QUAD,
    LOG_NO_FRAME_DURING_CALIBRATION,
)


def _make_synthetic_arena_frame(
    *,
    width_px: int = 1280,
    height_px: int = 720,
    border_bounds: tuple[int, int, int, int] = (140, 60, 1140, 660),
) -> np.ndarray:
    frame = np.full(
        (height_px, width_px, 3),
        230,
        dtype=np.uint8,
    )
    x0, y0, x1, y1 = border_bounds
    cv2.rectangle(
        frame,
        (x0, y0),
        (x1, y1),
        (0, 0, 0),
        20,
    )
    return frame


def test_should_fail_with_no_frame_warning_when_frame_is_none() -> None:
    result = calibrate_from_frame(None)

    assert isinstance(result, CalibrationResult)
    assert result.is_success is False
    assert result.homography is None
    assert result.corners is None
    assert result.warning_message == LOG_NO_FRAME_DURING_CALIBRATION


def test_should_succeed_with_homography_when_synthetic_arena_frame_is_valid() -> None:
    frame = _make_synthetic_arena_frame()

    result = calibrate_from_frame(
        frame,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert result.is_success is True
    assert result.homography is not None
    assert result.homography.shape == (3, 3)
    assert result.corners is not None
    assert result.corners.shape == (4, 2)
    assert result.warning_message is None


def test_should_succeed_with_homography_when_real_arena_photo_is_provided() -> None:
    fixture_path = Path(__file__).parent / "fixtures" / "arena_real_2026-11-24.jpg"
    raw_bytes = np.fromfile(
        str(fixture_path),
        dtype=np.uint8,
    )
    frame = cv2.imdecode(
        raw_bytes,
        cv2.IMREAD_COLOR,
    )
    assert frame is not None

    result = calibrate_from_frame(
        frame,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert result.is_success is True
    assert result.homography is not None
    assert result.homography.shape == (3, 3)
    assert result.warning_message is None


def test_should_fail_with_warning_when_frame_has_no_arena_border() -> None:
    blank_frame = np.full(
        (720, 1280, 3),
        230,
        dtype=np.uint8,
    )

    result = calibrate_from_frame(blank_frame)

    assert result.is_success is False
    assert result.homography is None
    assert result.warning_message is not None


def test_should_fail_with_invalid_quad_warning_when_detected_border_is_square() -> None:
    square_frame = _make_synthetic_arena_frame(
        border_bounds=(300, 75, 850, 625),
    )

    result = calibrate_from_frame(
        square_frame,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert result.is_success is False
    assert result.homography is None
    assert result.corners is not None
    assert result.warning_message == LOG_CALIBRATION_INVALID_QUAD
