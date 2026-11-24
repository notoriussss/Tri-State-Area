"""Forward-cone collision level assessment and time-to-collision calculation."""

import math
from dataclasses import dataclass

from overhead_core.constants import (
    COLLISION_CONE_HALF_ANGLE_RAD,
    COLLISION_CONE_RAY_COUNT,
    COLLISION_SLOW_DIST_M,
    COLLISION_SLOW_TTC_S,
    COLLISION_STOP_DIST_M,
    COLLISION_STOP_TTC_S,
    RADAR_MAX_RANGE_M,
    VELOCITY_ZERO_EPSILON,
    CollisionLevel,
    ObstacleKind,
)
from overhead_core.geometry import world_to_robot
from overhead_core.radar import (
    ray_free_distance,
    spread_angles,
)
from overhead_core.types import (
    CollisionAssessment,
    ObstacleSet,
    RobotPose,
)

_LEVEL_SEVERITY: dict[CollisionLevel, int] = {
    CollisionLevel.CLEAR: 0,
    CollisionLevel.SLOW: 1,
    CollisionLevel.STOP: 2,
}


@dataclass(frozen=True)
class CollisionConfig:
    """Thresholds and ray sampling parameters for collision assessment."""

    cone_half_angle_rad: float = COLLISION_CONE_HALF_ANGLE_RAD
    cone_ray_count: int = COLLISION_CONE_RAY_COUNT
    max_range_m: float = RADAR_MAX_RANGE_M
    slow_dist_m: float = COLLISION_SLOW_DIST_M
    stop_dist_m: float = COLLISION_STOP_DIST_M
    slow_ttc_s: float = COLLISION_SLOW_TTC_S
    stop_ttc_s: float = COLLISION_STOP_TTC_S


def closest_approach_time_s(
    rel_x_m: float,
    rel_y_m: float,
    *,
    rel_vx_mps: float,
    rel_vy_mps: float,
    radius_m: float,
) -> float | None:
    """Computes first contact time between two constant-velocity circles.

    Models the ego robot at the origin and a circle of ``radius_m`` at
    ``(rel_x_m, rel_y_m)`` moving at ``(rel_vx_mps, rel_vy_mps)`` relative to the robot.

    Args:
        rel_x_m: Obstacle x minus robot x in the arena frame, meters.
        rel_y_m: Obstacle y minus robot y in the arena frame, meters.
        rel_vx_mps: Obstacle vx minus robot vx in the arena frame, m/s.
        rel_vy_mps: Obstacle vy minus robot vy in the arena frame, m/s.
        radius_m: Combined safety radius at which contact occurs, meters.

    Returns:
        Time in seconds until first contact (0.0 if already overlapping), or None
        if the circles never touch or are already diverging.
    """
    if radius_m < 0.0:
        return None

    dist_sq = rel_x_m * rel_x_m + rel_y_m * rel_y_m
    radius_sq = radius_m * radius_m
    if dist_sq <= radius_sq:
        return 0.0

    speed_sq = rel_vx_mps * rel_vx_mps + rel_vy_mps * rel_vy_mps
    if speed_sq <= VELOCITY_ZERO_EPSILON:
        return None

    half_b = rel_x_m * rel_vx_mps + rel_y_m * rel_vy_mps
    if half_b >= 0.0:
        return None

    c = dist_sq - radius_sq
    discriminant = half_b * half_b - speed_sq * c
    if discriminant < 0.0:
        return None

    ttc_s = (-half_b - math.sqrt(discriminant)) / speed_sq
    return max(
        0.0,
        ttc_s,
    )


def _distance_level(
    free_dist_m: float,
    config: CollisionConfig,
) -> CollisionLevel:
    """Maps minimum forward free distance to a collision severity level."""
    if free_dist_m < config.stop_dist_m:
        return CollisionLevel.STOP
    if free_dist_m <= config.slow_dist_m:
        return CollisionLevel.SLOW
    return CollisionLevel.CLEAR


def _time_level(
    ttc_s: float | None,
    config: CollisionConfig,
) -> CollisionLevel:
    """Maps time to collision to a collision severity level."""
    if ttc_s is None:
        return CollisionLevel.CLEAR
    if ttc_s < config.stop_ttc_s:
        return CollisionLevel.STOP
    if ttc_s <= config.slow_ttc_s:
        return CollisionLevel.SLOW
    return CollisionLevel.CLEAR


def _nearest_cone_hit(
    pose: RobotPose,
    obstacles: ObstacleSet,
    *,
    config: CollisionConfig,
) -> tuple[float, ObstacleKind | None, float, float]:
    """Samples the forward cone and returns free distance and nearest hit details."""
    relative_bearings = spread_angles(
        0.0,
        config.cone_half_angle_rad,
        config.cone_ray_count,
    )
    free_dist_m = config.max_range_m
    nearest_kind: ObstacleKind | None = None
    nearest_dist_m = -1.0
    nearest_bearing_rad = 0.0

    for bearing_rad in relative_bearings:
        hit_dist_m, hit_kind = ray_free_distance(
            pose.x,
            pose.y,
            pose.theta + bearing_rad,
            obstacles=obstacles,
            max_range_m=config.max_range_m,
        )
        free_dist_m = min(free_dist_m, hit_dist_m)
        if hit_kind is None:
            continue
        is_tie = hit_dist_m == nearest_dist_m and abs(bearing_rad) < abs(nearest_bearing_rad)
        is_better_hit = nearest_kind is None or hit_dist_m < nearest_dist_m or is_tie
        if is_better_hit:
            nearest_kind = hit_kind
            nearest_dist_m = hit_dist_m
            nearest_bearing_rad = bearing_rad

    return (
        free_dist_m,
        nearest_kind,
        nearest_dist_m,
        nearest_bearing_rad,
    )


def _min_cone_ttc_s(
    pose: RobotPose,
    velocity: tuple[float, float],
    obstacles: ObstacleSet,
    *,
    config: CollisionConfig,
) -> float | None:
    """Computes minimum time to collision among circle obstacles in the forward cone."""
    min_ttc_s: float | None = None
    robot_pose_tuple = (
        pose.x,
        pose.y,
        pose.theta,
    )

    for circle in obstacles.circles:
        _, _, _, bearing_rad = world_to_robot(
            robot_pose_tuple,
            (circle.x_m, circle.y_m),
        )
        if abs(bearing_rad) > config.cone_half_angle_rad:
            continue

        ttc_s = closest_approach_time_s(
            circle.x_m - pose.x,
            circle.y_m - pose.y,
            rel_vx_mps=circle.vx_mps - velocity[0],
            rel_vy_mps=circle.vy_mps - velocity[1],
            radius_m=circle.radius_m,
        )
        if ttc_s is None:
            continue
        if min_ttc_s is None or ttc_s < min_ttc_s:
            min_ttc_s = ttc_s

    return min_ttc_s


def assess_collision(
    pose: RobotPose,
    velocity: tuple[float, float],
    obstacles: ObstacleSet,
    *,
    config: CollisionConfig | None = None,
) -> CollisionAssessment:
    """Assesses forward-cone collision risk for a robot given arena obstacles.

    Evaluates free distance across rays spanning +-cone_half_angle_rad around the scoop
    and time to collision across circular obstacles whose bearing lies in that cone.

    Args:
        pose: Ego robot pose in the arena frame (meters, radians).
        velocity: Ego robot velocity (vx_mps, vy_mps) in the arena frame, m/s.
        obstacles: Circular and wall/zone obstacles in the arena frame.
        config: Optional collision thresholds and ray configuration.

    Returns:
        CollisionAssessment combining distance and time-to-collision severity levels.
    """
    cfg = config if config is not None else CollisionConfig()
    free_dist_m, nearest_kind, nearest_dist_m, nearest_bearing_rad = _nearest_cone_hit(
        pose,
        obstacles,
        config=cfg,
    )
    ttc_s = _min_cone_ttc_s(
        pose,
        velocity,
        obstacles,
        config=cfg,
    )
    dist_level = _distance_level(
        free_dist_m,
        cfg,
    )
    ttc_level = _time_level(
        ttc_s,
        cfg,
    )
    final_level = max(
        (dist_level, ttc_level),
        key=lambda lvl: _LEVEL_SEVERITY[lvl],
    )
    return CollisionAssessment(
        level=final_level,
        time_to_collision_s=ttc_s if ttc_s is not None else -1.0,
        nearest_kind=nearest_kind,
        nearest_dist_m=nearest_dist_m,
        nearest_bearing_rad=nearest_bearing_rad,
    )
