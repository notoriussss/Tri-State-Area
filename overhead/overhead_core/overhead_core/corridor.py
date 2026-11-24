"""Corridor clearance query against red-cube safety circles."""

import math

from overhead_core.constants import ObstacleKind
from overhead_core.types import (
    CorridorResult,
    Obstacle,
)


def _point_to_segment_distance_m(
    point_xy: tuple[float, float],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
) -> float:
    """Computes Euclidean distance from a point to a line segment in meters."""
    px_m, py_m = point_xy
    ax_m, ay_m = start_xy
    gx_m, gy_m = goal_xy

    seg_dx_m = gx_m - ax_m
    seg_dy_m = gy_m - ay_m
    seg_len_sq = seg_dx_m * seg_dx_m + seg_dy_m * seg_dy_m
    if seg_len_sq <= 0.0:
        return math.hypot(px_m - ax_m, py_m - ay_m)

    rel_x_m = px_m - ax_m
    rel_y_m = py_m - ay_m
    raw_t = (rel_x_m * seg_dx_m + rel_y_m * seg_dy_m) / seg_len_sq
    clamped_t = min(1.0, max(0.0, raw_t))
    closest_x_m = ax_m + clamped_t * seg_dx_m
    closest_y_m = ay_m + clamped_t * seg_dy_m
    return math.hypot(px_m - closest_x_m, py_m - closest_y_m)


def corridor_free(
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    *,
    width_m: float,
    obstacles: list[Obstacle],
) -> CorridorResult:
    """Checks whether a straight corridor is clear of red-cube safety circles.

    Only obstacles with ``kind == ObstacleKind.RED_CUBE`` are evaluated. A red
    obstacle blocks when the distance from its center to the segment
    ``start_xy -> goal_xy`` minus its ``radius_m`` is strictly less than
    ``width_m / 2``. Degenerate segments (``start_xy == goal_xy``) are treated
    as a single point.

    Args:
        start_xy: Corridor start point (x_m, y_m) in the arena frame, meters.
        goal_xy: Corridor goal point (x_m, y_m) in the arena frame, meters.
        width_m: Full corridor width in meters.
        obstacles: Circular obstacles in the arena frame.

    Returns:
        CorridorResult with ``is_free`` and sorted ``blocking_ids``.
    """
    half_width_m = width_m / 2.0
    blocking_ids: list[int] = []

    for obstacle in obstacles:
        if obstacle.kind != ObstacleKind.RED_CUBE:
            continue

        dist_m = _point_to_segment_distance_m(
            (obstacle.x_m, obstacle.y_m),
            start_xy,
            goal_xy,
        )
        if dist_m - obstacle.radius_m < half_width_m:
            blocking_ids.append(obstacle.obstacle_id)

    sorted_ids = tuple(sorted(blocking_ids))
    return CorridorResult(
        is_free=not sorted_ids,
        blocking_ids=sorted_ids,
    )
