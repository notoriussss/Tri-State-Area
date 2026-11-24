"""Pure arena calibration attempt combining corner detection, validation, and homography."""

from dataclasses import dataclass

import cv2
import numpy as np

from overhead_core.calibration import validate_arena_quad
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    LOG_CALIBRATION_FAILED_FMT,
    LOG_CALIBRATION_INVALID_QUAD,
    LOG_NO_FRAME_DURING_CALIBRATION,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
)


@dataclass(frozen=True)
class CalibrationResult:
    """Outcome of a single arena homography calibration attempt."""

    is_success: bool
    homography: np.ndarray | None = None
    corners: np.ndarray | None = None
    warning_message: str | None = None


def calibrate_from_frame(
    frame: np.ndarray | None,
    *,
    arena_width_m: float = ARENA_WIDTH_M,
    arena_height_m: float = ARENA_HEIGHT_M,
) -> CalibrationResult:
    """Detects and validates arena corners and computes the floor homography.

    Args:
        frame: Input BGR camera frame, or None when no frame is available.
        arena_width_m: Physical arena width along world +x in meters.
        arena_height_m: Physical arena height along world +y in meters.

    Returns:
        CalibrationResult containing the 3x3 homography on success, or a
        warning message describing why calibration failed.
    """
    if frame is None:
        return CalibrationResult(
            is_success=False,
            warning_message=LOG_NO_FRAME_DURING_CALIBRATION,
        )

    try:
        corners = detect_arena_corners(frame)
    except (ValueError, cv2.error) as exc:
        return CalibrationResult(
            is_success=False,
            warning_message=LOG_CALIBRATION_FAILED_FMT.format(exc),
        )

    height_px, width_px = frame.shape[:2]
    is_valid = validate_arena_quad(
        corners,
        image_width_px=width_px,
        image_height_px=height_px,
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
    )
    if not is_valid:
        return CalibrationResult(
            is_success=False,
            corners=corners,
            warning_message=LOG_CALIBRATION_INVALID_QUAD,
        )

    homography = compute_homography(
        corners,
        arena_width_m,
        arena_height_m,
    )
    return CalibrationResult(
        is_success=True,
        homography=homography,
        corners=corners,
    )
