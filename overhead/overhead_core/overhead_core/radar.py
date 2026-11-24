"""Robot-centric radar sector distances and ray casting in the arena frame."""

import math

from overhead_core.constants import (
    RADAR_MAX_RANGE_M,
    RADAR_RAYS_PER_SECTOR,
    RADAR_SECTOR_COUNT,
    RAY_ENDPOINT_TOLERANCE,
    RAY_PARALLEL_EPSILON,
    ObstacleKind,
)
from overhead_core.types import (
    Obstacle,
    ObstacleSet,
    RobotPose,
    WallSegment,
)


def spread_angles(
    center_rad: float,
    half_width_rad: float,
    count: int,
) -> list[float]:
    """Generates evenly spaced angles across [center - half_width, center + half_width].

    Args:
        center_rad: Center angle in radians.
        half_width_rad: Half angular span in radians.
        count: Number of angles to generate.

    Returns:
        List of angles in radians.
    """
    if count <= 0:
        return []
    if count == 1:
        return [center_rad]
    step_rad = (2.0 * half_width_rad) / (count - 1)
    start_rad = center_rad - half_width_rad
    return [start_rad + idx * step_rad for idx in range(count)]


def _ray_circle_distance(
    origin_x_m: float,
    origin_y_m: float,
    direction: tuple[float, float],
    *,
    circle: Obstacle,
    max_range_m: float,
) -> float | None:
    """Returns distance along a unit ray to a circle boundary, or None if no hit."""
    if circle.radius_m < 0.0:
        return None

    fx = origin_x_m - circle.x_m
    fy = origin_y_m - circle.y_m
    c = fx * fx + fy * fy - circle.radius_m * circle.radius_m
    if c <= 0.0:
        return 0.0

    dx, dy = direction
    half_b = fx * dx + fy * dy
    if half_b >= 0.0:
        return None

    discriminant = half_b * half_b - c
    if discriminant < 0.0:
        return None

    dist_m = max(
        0.0,
        -half_b - math.sqrt(discriminant),
    )
    if dist_m > max_range_m:
        return None
    return dist_m


def _ray_segment_distance(
    origin_x_m: float,
    origin_y_m: float,
    direction: tuple[float, float],
    *,
    segment: WallSegment,
    max_range_m: float,
) -> float | None:
    """Returns distance along a unit ray to a wall segment, or None if no hit."""
    dx, dy = direction
    sx = segment.x1_m - segment.x0_m
    sy = segment.y1_m - segment.y0_m
    denom = dx * sy - dy * sx
    if abs(denom) < RAY_PARALLEL_EPSILON:
        return None

    qx = segment.x0_m - origin_x_m
    qy = segment.y0_m - origin_y_m
    dist_m = (qx * sy - qy * sx) / denom
    if dist_m < -RAY_ENDPOINT_TOLERANCE or dist_m > max_range_m:
        return None

    u = (qx * dy - qy * dx) / denom
    if u < -RAY_ENDPOINT_TOLERANCE or u > 1.0 + RAY_ENDPOINT_TOLERANCE:
        return None

    return max(
        0.0,
        dist_m,
    )


def _is_closer_hit(
    hit_dist_m: float | None,
    best_dist_m: float,
    best_kind: ObstacleKind | None,
) -> bool:
    """Returns whether a candidate ray intersection improves on the current best hit."""
    if hit_dist_m is None:
        return False
    return hit_dist_m < best_dist_m or best_kind is None


def ray_free_distance(
    origin_x_m: float,
    origin_y_m: float,
    angle_rad: float,
    *,
    obstacles: ObstacleSet,
    max_range_m: float = RADAR_MAX_RANGE_M,
) -> tuple[float, ObstacleKind | None]:
    """Casts a ray in the arena frame and returns the nearest obstacle distance and kind.

    Args:
        origin_x_m: Ray origin x coordinate in the arena frame, meters.
        origin_y_m: Ray origin y coordinate in the arena frame, meters.
        angle_rad: Ray direction in the arena frame, radians CCW from +x.
        obstacles: Circular and wall/zone obstacles in the arena frame.
        max_range_m: Maximum sensing distance in meters.

    Returns:
        Tuple of (distance_m, kind) where distance_m is capped at max_range_m and
        kind is None when no obstacle is hit within range.
    """
    if max_range_m <= 0.0:
        return (0.0, None)

    direction = (
        math.cos(angle_rad),
        math.sin(angle_rad),
    )
    best_dist_m = max_range_m
    best_kind: ObstacleKind | None = None

    for circle in obstacles.circles:
        hit_dist_m = _ray_circle_distance(
            origin_x_m,
            origin_y_m,
            direction,
            circle=circle,
            max_range_m=best_dist_m,
        )
        if _is_closer_hit(
            hit_dist_m,
            best_dist_m,
            best_kind,
        ):
            best_dist_m = float(hit_dist_m)
            best_kind = ObstacleKind(circle.kind)

    for segment in obstacles.walls:
        hit_dist_m = _ray_segment_distance(
            origin_x_m,
            origin_y_m,
            direction,
            segment=segment,
            max_range_m=best_dist_m,
        )
        if _is_closer_hit(
            hit_dist_m,
            best_dist_m,
            best_kind,
        ):
            best_dist_m = float(hit_dist_m)
            best_kind = ObstacleKind(segment.kind)

    return (best_dist_m, best_kind)


def compute_radar(
    pose: RobotPose,
    obstacles: ObstacleSet,
    *,
    max_range_m: float = RADAR_MAX_RANGE_M,
) -> list[float]:
    """Computes 8-sector robot-centric free distances around a robot pose.

    Sector 0 is centered on the robot scoop (+x in robot frame, pose.theta in arena frame),
    and sectors advance counter-clockwise in 45-degree increments.

    Args:
        pose: Ego robot pose in the arena frame (meters, radians).
        obstacles: Circular and wall/zone obstacles in the arena frame.
        max_range_m: Maximum radar sensing range in meters.

    Returns:
        List of RADAR_SECTOR_COUNT free distances in meters, capped at max_range_m.
    """
    sector_width_rad = math.tau / RADAR_SECTOR_COUNT
    half_sector_rad = sector_width_rad / 2.0
    sectors: list[float] = []

    for sector_idx in range(RADAR_SECTOR_COUNT):
        sector_center_rad = pose.theta + sector_idx * sector_width_rad
        ray_angles = spread_angles(
            sector_center_rad,
            half_sector_rad,
            RADAR_RAYS_PER_SECTOR,
        )
        sector_dist_m = min(
            ray_free_distance(
                pose.x,
                pose.y,
                angle_rad,
                obstacles=obstacles,
                max_range_m=max_range_m,
            )[0]
            for angle_rad in ray_angles
        )
        sectors.append(sector_dist_m)

    return sectors
