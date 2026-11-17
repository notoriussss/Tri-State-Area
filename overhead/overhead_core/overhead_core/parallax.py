"""Parallax projection corrections for points above the arena floor."""

import numpy as np

from overhead_core.constants import (
    CAM_HEIGHT_M,
    CUBE_HEIGHT_M,
    ROBOT_MARKER_HEIGHT_M,
)

__all__ = [
    "CAM_HEIGHT_M",
    "CUBE_HEIGHT_M",
    "ROBOT_MARKER_HEIGHT_M",
    "correct_parallax",
]


def correct_parallax(
    seen_xy: np.ndarray,
    center_xy: np.ndarray,
    *,
    object_height_m: float = 0.0,
    cam_height_m: float = CAM_HEIGHT_M,
) -> np.ndarray:
    """Projects points seen at a given height down to the arena floor.

    Accounts for perspective displacement away from the nadir point directly beneath the camera.

    Args:
        seen_xy: Nx2 points in the arena frame, meters, as the homography returns them.
        center_xy: Point of the floor right below the camera, arena frame, meters.
        object_height_m: Height of the observed feature above the floor, meters.
        cam_height_m: Camera height above the arena floor, meters.

    Returns:
        Nx2 floor points in the arena frame, meters.
    """
    seen = np.asarray(
        seen_xy,
        dtype=np.float64,
    )
    center = np.asarray(
        center_xy,
        dtype=np.float64,
    )

    if seen.size == 0:
        return np.empty_like(seen)

    scale = (cam_height_m - object_height_m) / cam_height_m

    return center + (seen - center) * scale
