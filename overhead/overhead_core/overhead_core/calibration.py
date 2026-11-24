"""Arena quadrilateral validation for honest camera calibration."""

import cv2
import numpy as np

from overhead_core.constants import (
    ARENA_ASPECT_TOLERANCE_RATIO,
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    MIN_ARENA_AREA_FRACTION,
    MIN_CORNER_BOUND_MARGIN_PX,
    QUAD_POINT_DIMS,
    QUAD_VERTEX_COUNT,
)


def _has_valid_quad_order(pts: np.ndarray) -> bool:
    """Checks convexity and [BL, BR, TR, TL] vertex ordering in image coordinates."""
    signed_area = float(cv2.contourArea(pts, oriented=True))
    if signed_area >= 0.0:
        return False

    pts_f64 = pts.astype(np.float64)
    shifted_pts = np.roll(
        pts_f64,
        -1,
        axis=0,
    )
    edges = shifted_pts - pts_f64
    next_edges = np.roll(
        edges,
        -1,
        axis=0,
    )
    cross_products = edges[:, 0] * next_edges[:, 1] - edges[:, 1] * next_edges[:, 0]
    if not bool(np.all(cross_products < 0.0)):
        return False

    bl, br, tr, tl = pts_f64
    is_ordered = br[0] > bl[0] and tr[0] > tl[0] and bl[1] > tl[1] and br[1] > tr[1]
    return bool(is_ordered)


def _has_valid_aspect_ratio(
    pts: np.ndarray,
    *,
    arena_width_m: float,
    arena_height_m: float,
) -> bool:
    """Checks whether the quad width-to-height edge ratio matches the physical arena."""
    bl, br, tr, tl = pts.astype(np.float64)
    bottom_len_px = float(np.linalg.norm(br - bl))
    top_len_px = float(np.linalg.norm(tr - tl))
    left_len_px = float(np.linalg.norm(bl - tl))
    right_len_px = float(np.linalg.norm(br - tr))

    mean_width_px = (bottom_len_px + top_len_px) / 2.0
    mean_height_px = (left_len_px + right_len_px) / 2.0
    if mean_height_px <= 0.0:
        return False

    observed_aspect = mean_width_px / mean_height_px
    expected_aspect = arena_width_m / arena_height_m
    relative_error = abs(observed_aspect - expected_aspect) / expected_aspect
    return relative_error <= ARENA_ASPECT_TOLERANCE_RATIO


def _are_corners_on_image_bounds(
    pts: np.ndarray,
    *,
    image_width_px: int,
    image_height_px: int,
) -> bool:
    """Checks whether all four corners sit on the image perimeter.

    This is what the corner detector returns on a blank or black frame, so it is never a real
    arena border.
    """
    on_x_edge = (pts[:, 0] <= MIN_CORNER_BOUND_MARGIN_PX) | (
        pts[:, 0] >= image_width_px - 1 - MIN_CORNER_BOUND_MARGIN_PX
    )
    on_y_edge = (pts[:, 1] <= MIN_CORNER_BOUND_MARGIN_PX) | (
        pts[:, 1] >= image_height_px - 1 - MIN_CORNER_BOUND_MARGIN_PX
    )
    return bool(np.all(on_x_edge & on_y_edge))


def validate_arena_quad(
    corners: np.ndarray | list[list[float]],
    *,
    image_width_px: int,
    image_height_px: int,
    arena_width_m: float = ARENA_WIDTH_M,
    arena_height_m: float = ARENA_HEIGHT_M,
) -> bool:
    """Validates a detected arena quadrilateral [BL, BR, TR, TL] against image and arena bounds.

    Args:
        corners: 4x2 array or list of corner pixel coordinates ordered [BL, BR, TR, TL].
        image_width_px: Source image width in pixels.
        image_height_px: Source image height in pixels.
        arena_width_m: Physical arena width along world +x in meters.
        arena_height_m: Physical arena height along world +y in meters.

    Returns:
        True if the quad is convex in [BL, BR, TR, TL] order, is not the image perimeter itself,
        covers at least MIN_ARENA_AREA_FRACTION of the image, and matches the physical aspect ratio
        within ARENA_ASPECT_TOLERANCE_RATIO.
    """
    has_invalid_size = (
        image_width_px <= 0 or image_height_px <= 0 or arena_width_m <= 0.0 or arena_height_m <= 0.0
    )
    if has_invalid_size:
        return False

    pts = np.asarray(
        corners,
        dtype=np.float32,
    )
    expected_shape = (
        QUAD_VERTEX_COUNT,
        QUAD_POINT_DIMS,
    )
    if pts.shape != expected_shape or not bool(np.all(np.isfinite(pts))):
        return False

    if not _has_valid_quad_order(pts):
        return False

    is_image_bounds = _are_corners_on_image_bounds(
        pts,
        image_width_px=image_width_px,
        image_height_px=image_height_px,
    )
    if is_image_bounds:
        return False

    quad_area_px = float(cv2.contourArea(pts))
    image_area_px = float(image_width_px * image_height_px)
    if quad_area_px < MIN_ARENA_AREA_FRACTION * image_area_px:
        return False

    return _has_valid_aspect_ratio(
        pts,
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
    )
