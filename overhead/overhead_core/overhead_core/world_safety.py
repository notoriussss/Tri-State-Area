"""Pure arena-wide safety evaluation combining obstacles, radar, and collision checks."""

from dataclasses import dataclass, field

from overhead_core.collision import assess_collision
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    DEFAULT_ZONE_RECT,
)
from overhead_core.obstacles import (
    ObstacleConfig,
    build_obstacles_for,
)
from overhead_core.radar import compute_radar
from overhead_core.types import (
    Obstacle,
    RobotPose,
    RobotSafety,
    SafetyResult,
)

_ARENA_SCOPE_SENTINEL: str = "__arena__"


@dataclass(frozen=True)
class SafetyConfig:
    """Configuration for arena-wide obstacle and per-robot safety evaluation."""

    arena_width_m: float = ARENA_WIDTH_M
    arena_height_m: float = ARENA_HEIGHT_M
    zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT
    zone_occupant: str | None = None
    is_gate_closed: bool = False
    carrying_robots: frozenset[str] = field(default_factory=frozenset)


def _to_obstacle_config(config: SafetyConfig) -> ObstacleConfig:
    """Converts a SafetyConfig into an ObstacleConfig."""
    return ObstacleConfig(
        arena_width_m=config.arena_width_m,
        arena_height_m=config.arena_height_m,
        zone_rect=config.zone_rect,
        zone_occupant=config.zone_occupant,
        is_gate_closed=config.is_gate_closed,
        carrying_robots=config.carrying_robots,
    )


def _build_arena_circles(
    *,
    cubes: list[tuple[int, str, float, float, float]],
    robot_poses: list[RobotPose],
    velocities: dict[str, tuple[float, float]],
    obstacle_config: ObstacleConfig,
) -> list[Obstacle]:
    """Builds all cube and robot safety circles for the whole arena without walls."""
    robot_names = {pose.name for pose in robot_poses}
    excluded_name = _ARENA_SCOPE_SENTINEL
    while excluded_name in robot_names:
        excluded_name = f"{excluded_name}_"

    return build_obstacles_for(
        excluded_name,
        cubes=cubes,
        robots=robot_poses,
        velocities=velocities,
        config=obstacle_config,
    ).circles


def compute_safety(
    robot_poses: list[RobotPose],
    cubes: list[tuple[int, str, float, float, float]],
    *,
    velocities: dict[str, tuple[float, float]],
    config: SafetyConfig,
) -> SafetyResult:
    """Computes arena obstacle circles, per-robot radar sectors, and collision levels.

    Args:
        robot_poses: Active robot poses in the arena frame (meters, radians).
        cubes: Tracked cubes as (track_id, color, x_m, y_m, confidence) in arena frame.
        velocities: Mapping from robot name to (vx_mps, vy_mps) in arena frame.
        config: Arena bounds, delivery zone state, and carrying status.

    Returns:
        SafetyResult containing arena-wide circle obstacles and per-robot safety outputs.
    """
    obstacle_config = _to_obstacle_config(config)
    circles = _build_arena_circles(
        cubes=cubes,
        robot_poses=robot_poses,
        velocities=velocities,
        obstacle_config=obstacle_config,
    )

    per_robot: dict[str, RobotSafety] = {}
    for pose in robot_poses:
        obstacle_set = build_obstacles_for(
            pose.name,
            cubes=cubes,
            robots=robot_poses,
            velocities=velocities,
            config=obstacle_config,
        )
        sectors = compute_radar(
            pose,
            obstacle_set,
        )
        velocity = velocities.get(
            pose.name,
            (0.0, 0.0),
        )
        assessment = assess_collision(
            pose,
            velocity,
            obstacle_set,
        )
        per_robot[pose.name] = RobotSafety(
            sectors=sectors,
            assessment=assessment,
        )

    return SafetyResult(
        circles=circles,
        per_robot=per_robot,
    )
