"""Triangle marker pose estimation and heading transform."""

import math

import cv2
import numpy as np

from overhead_core.constants import (
    DEFAULT_ROBOT_MIN_AREA_PX,
    DEFAULT_TRIANGLE_HEADING_RULE,
    ERR_UNKNOWN_TRIANGLE_HEADING_RULE_FMT,
    TRIANGLE_VERTEX_COUNT,
    TriangleHeadingRule,
)
from overhead_core.geometry import (
    image_to_world,
    wrap_angle_rad,
)

_APPROX_EPS_STEPS: int = 70
_APPROX_EPS_MIN_RATIO: float = 0.01
_APPROX_EPS_MAX_RATIO: float = 0.35


def _validate_heading_rule(heading_rule: TriangleHeadingRule | str) -> TriangleHeadingRule:
    """Validates and normalizes a triangle heading rule value."""
    try:
        return TriangleHeadingRule(heading_rule)
    except ValueError as exc:
        raise ValueError(
            ERR_UNKNOWN_TRIANGLE_HEADING_RULE_FMT.format(heading_rule),
        ) from exc


def _extract_triangle_vertices(hull: np.ndarray) -> np.ndarray | None:
    """Approximates a convex hull as 3 float64 (u, v) triangle vertices."""
    peri = cv2.arcLength(
        hull,
        True,
    )
    poly = None
    for eps_factor in np.linspace(
        _APPROX_EPS_MIN_RATIO,
        _APPROX_EPS_MAX_RATIO,
        _APPROX_EPS_STEPS,
    ):
        approx = cv2.approxPolyDP(
            hull,
            eps_factor * peri,
            True,
        )
        if len(approx) == TRIANGLE_VERTEX_COUNT:
            poly = approx
            break

    if poly is not None:
        return poly.reshape(
            TRIANGLE_VERTEX_COUNT,
            2,
        ).astype(np.float64)

    _, triangle_pts = cv2.minEnclosingTriangle(hull)
    if triangle_pts is None or len(triangle_pts) != TRIANGLE_VERTEX_COUNT:
        return None
    return triangle_pts.reshape(
        TRIANGLE_VERTEX_COUNT,
        2,
    ).astype(np.float64)


def _compute_side_lengths(pts: np.ndarray) -> list[float]:
    """Computes side lengths ordered so index i is the side opposite vertex i."""
    return [
        math.hypot(
            float(pts[1, 0] - pts[2, 0]),
            float(pts[1, 1] - pts[2, 1]),
        ),
        math.hypot(
            float(pts[0, 0] - pts[2, 0]),
            float(pts[0, 1] - pts[2, 1]),
        ),
        math.hypot(
            float(pts[0, 0] - pts[1, 0]),
            float(pts[0, 1] - pts[1, 1]),
        ),
    ]


def _compute_heading_img_rad(
    pts: np.ndarray,
    *,
    centroid: np.ndarray,
    side_lengths: list[float],
    heading_rule: TriangleHeadingRule,
) -> float | None:
    """Computes image-frame heading in radians from 3 vertices using heading_rule."""
    if heading_rule == TriangleHeadingRule.OPPOSITE_SHORTEST_SIDE:
        shortest_idx = int(np.argmin(side_lengths))
        tip = pts[shortest_idx]
        return math.atan2(
            float(tip[1] - centroid[1]),
            float(tip[0] - centroid[0]),
        )

    longest_idx = int(np.argmax(side_lengths))
    apex = pts[longest_idx]
    edge_indices = [idx for idx in range(TRIANGLE_VERTEX_COUNT) if idx != longest_idx]
    edge_a = pts[edge_indices[0]]
    edge_b = pts[edge_indices[1]]
    edge = edge_b - edge_a
    edge_len = math.hypot(
        float(edge[0]),
        float(edge[1]),
    )
    if edge_len <= 0.0:
        return None

    normal = np.array(
        [
            -float(edge[1]) / edge_len,
            float(edge[0]) / edge_len,
        ],
        dtype=np.float64,
    )
    if float(np.dot(normal, apex - edge_a)) < 0.0:
        normal = -normal

    return math.atan2(
        float(normal[1]),
        float(normal[0]),
    )


def triangle_pose(
    mask: np.ndarray,
    *,
    min_area_px: int = DEFAULT_ROBOT_MIN_AREA_PX,
    heading_rule: TriangleHeadingRule = DEFAULT_TRIANGLE_HEADING_RULE,
) -> tuple[float, float, float] | None:
    """Estimates the position and image heading of a triangle marker.

    Finds the largest contour in the mask, checks the area of its convex hull against
    ``min_area_px`` (so wires crossing the marker do not drop a valid triangle),
    approximates the hull to 3 vertices, and computes heading according to ``heading_rule``:

    - ``TriangleHeadingRule.LONGEST_SIDE_NORMAL`` (default for provisional triangles):
      uses the unit normal of the longest edge pointing toward the opposite vertex. This
      fits right-ish triangles mounted with their longest edge along the robot rear and the
      third vertex pointing toward the robot front.
    - ``TriangleHeadingRule.OPPOSITE_SHORTEST_SIDE``: points from the triangle centroid to
      the vertex opposite the shortest edge. This fits elongated isosceles markers.

    Position is always the triangle centroid in image pixels before per-robot offsets.

    Args:
        mask: Binary single-channel mask (uint8).
        min_area_px: Minimum convex-hull area in pixels to process.
        heading_rule: Rule selecting how heading is derived from the triangle vertices.

    Returns:
        Tuple of (u_px, v_px, theta_img_rad) or None if no valid triangle is found.

    Raises:
        ValueError: If heading_rule is not a recognized TriangleHeadingRule value.
    """
    rule = _validate_heading_rule(heading_rule)
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
    hull = cv2.convexHull(largest)
    if cv2.contourArea(hull) < min_area_px:
        return None

    pts = _extract_triangle_vertices(hull)
    if pts is None:
        return None

    centroid = np.mean(
        pts,
        axis=0,
    )
    side_lengths = _compute_side_lengths(pts)
    theta_img = _compute_heading_img_rad(
        pts,
        centroid=centroid,
        side_lengths=side_lengths,
        heading_rule=rule,
    )
    if theta_img is None:
        return None

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
