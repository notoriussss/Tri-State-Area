"""Triangle marker pose estimation and heading transform."""

import math

import cv2
import numpy as np

from overhead_core.constants import (
    DEFAULT_ROBOT_MIN_AREA_PX,
    TRIANGLE_VERTEX_COUNT,
)
from overhead_core.geometry import (
    image_to_world,
    wrap_angle_rad,
)


def triangle_pose(
    mask: np.ndarray,
    *,
    min_area_px: int = DEFAULT_ROBOT_MIN_AREA_PX,
) -> tuple[float, float, float] | None:
    """Estimates the position and image heading of a triangle marker.

    Finds the largest contour in the mask, approximates it to 3 vertices,
    and identifies the tip as the vertex opposite the shortest side.

    Args:
        mask: Binary single-channel mask (uint8).
        min_area_px: Minimum contour area in pixels to process.

    Returns:
        Tuple of (u_px, v_px, theta_img_rad) or None if no valid triangle is found.
    """
    if mask is None or mask.size == 0:
        return None

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    largest = max(
        contours,
        key=cv2.contourArea,
    )
    if cv2.contourArea(largest) < min_area_px:
        return None

    hull = cv2.convexHull(largest)
    peri = cv2.arcLength(
        hull,
        True,
    )

    poly = None
    for eps_factor in np.linspace(0.01, 0.35, 70):
        approx = cv2.approxPolyDP(
            hull,
            eps_factor * peri,
            True,
        )
        if len(approx) == TRIANGLE_VERTEX_COUNT:
            poly = approx
            break

    if poly is None:
        _, triangle_pts = cv2.minEnclosingTriangle(hull)
        if triangle_pts is None or len(triangle_pts) != TRIANGLE_VERTEX_COUNT:
            return None
        pts = triangle_pts.reshape(
            TRIANGLE_VERTEX_COUNT,
            2,
        ).astype(np.float64)
    else:
        pts = poly.reshape(
            TRIANGLE_VERTEX_COUNT,
            2,
        ).astype(np.float64)

    centroid = np.mean(
        pts,
        axis=0,
    )

    side_lengths = [
        math.hypot(
            pts[1, 0] - pts[2, 0],
            pts[1, 1] - pts[2, 1],
        ),
        math.hypot(
            pts[0, 0] - pts[2, 0],
            pts[0, 1] - pts[2, 1],
        ),
        math.hypot(
            pts[0, 0] - pts[1, 0],
            pts[0, 1] - pts[1, 1],
        ),
    ]

    shortest_idx = int(np.argmin(side_lengths))
    tip = pts[shortest_idx]

    du = tip[0] - centroid[0]
    dv = tip[1] - centroid[1]
    theta_img = math.atan2(
        dv,
        du,
    )

    return (
        float(centroid[0]),
        float(centroid[1]),
        float(theta_img),
    )


def heading_to_world(
    homography: np.ndarray,
    u_px: float,
    v_px: float,
    *,
    theta_img: float = 0.0,
    step_px: float = 20.0,
) -> float:
    """Transforms an image heading angle to a world angle in frame arena.

    Maps the centroid and a point along the heading vector through the homography,
    handling the vertical coordinate flip between image and world frames.

    Args:
        homography: 3x3 homography matrix mapping pixels to world meters.
        u_px: Centroid horizontal coordinate in image pixels.
        v_px: Centroid vertical coordinate in image pixels.
        theta_img: Image heading angle in radians (atan2 in image coordinates).
        step_px: Step distance in pixels along heading to map through homography.

    Returns:
        World heading angle theta in radians, CCW from +x in (-pi, pi].
    """
    tip_u = u_px + step_px * math.cos(theta_img)
    tip_v = v_px + step_px * math.sin(theta_img)

    pts_uv = np.array(
        [
            [u_px, v_px],
            [tip_u, tip_v],
        ],
        dtype=np.float64,
    )

    pts_world = image_to_world(
        homography,
        pts_uv,
    )
    c_world = pts_world[0]
    tip_world = pts_world[1]

    dx = tip_world[0] - c_world[0]
    dy = tip_world[1] - c_world[1]
    world_theta = math.atan2(
        dy,
        dx,
    )
    return wrap_angle_rad(world_theta)
