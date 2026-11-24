"""Obstacle and safety circle construction for collision and radar queries."""

from dataclasses import dataclass, field

from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CARRY_EXTENSION_M,
    CUBE_SAFETY_M,
    DEFAULT_ZONE_RECT,
    RED_CUBE_SAFETY_M,
    ROBOT_SAFETY_M,
    CubeColor,
    ObstacleKind,
)
from overhead_core.types import (
    Obstacle,
    ObstacleSet,
    RobotPose,
    WallSegment,
)


@dataclass(frozen=True)
class ObstacleConfig:
    """Configuration for building arena obstacles around a target robot."""

    arena_width_m: float = ARENA_WIDTH_M
    arena_height_m: float = ARENA_HEIGHT_M
    zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT
    zone_occupant: str | None = None
    is_gate_closed: bool = False
    carrying_robots: frozenset[str] = field(default_factory=frozenset)


def _build_cube_circles(
    cubes: list[tuple[int, str, float, float, float]],
) -> list[Obstacle]:
    """Converts tracked cubes into circular obstacles with color-dependent safety radii."""
    circles: list[Obstacle] = []
    for track_id, color, x_m, y_m, _confidence in cubes:
        is_red = color == CubeColor.RED
        kind = ObstacleKind.RED_CUBE if is_red else ObstacleKind.CUBE
        radius_m = RED_CUBE_SAFETY_M if is_red else CUBE_SAFETY_M
        circles.append(
            Obstacle(
                kind=kind,
                obstacle_id=track_id,
                x_m=x_m,
                y_m=y_m,
                radius_m=radius_m,
                vx_mps=0.0,
                vy_mps=0.0,
            )
        )
    return circles


def _build_robot_circles(
    robot_name: str,
    *,
    robots: list[RobotPose],
    velocities: dict[str, tuple[float, float]],
    carrying_robots: frozenset[str],
) -> list[Obstacle]:
    """Converts other robots into circular obstacles with alphabetical stable IDs."""
    sorted_robots = sorted(
        robots,
        key=lambda robot: robot.name,
    )
    circles: list[Obstacle] = []
    for robot_idx, robot in enumerate(sorted_robots):
        if robot.name == robot_name:
            continue
        radius_m = ROBOT_SAFETY_M
        if robot.name in carrying_robots:
            radius_m += CARRY_EXTENSION_M
        vx_mps, vy_mps = velocities.get(
            robot.name,
            (0.0, 0.0),
        )
        circles.append(
            Obstacle(
                kind=ObstacleKind.ROBOT,
                obstacle_id=robot_idx,
                x_m=robot.x,
                y_m=robot.y,
                radius_m=radius_m,
                vx_mps=vx_mps,
                vy_mps=vy_mps,
            )
        )
    return circles


def _rectangle_segments(
    kind: ObstacleKind,
    rect: tuple[float, float, float, float],
) -> list[WallSegment]:
    """Builds four border segments around an axis-aligned rectangle."""
    x0_m, y0_m, x1_m, y1_m = rect
    return [
        WallSegment(
            kind=kind,
            x0_m=x0_m,
            y0_m=y0_m,
            x1_m=x1_m,
            y1_m=y0_m,
        ),
        WallSegment(
            kind=kind,
            x0_m=x1_m,
            y0_m=y0_m,
            x1_m=x1_m,
            y1_m=y1_m,
        ),
        WallSegment(
            kind=kind,
            x0_m=x1_m,
            y0_m=y1_m,
            x1_m=x0_m,
            y1_m=y1_m,
        ),
        WallSegment(
            kind=kind,
            x0_m=x0_m,
            y0_m=y1_m,
            x1_m=x0_m,
            y1_m=y0_m,
        ),
    ]


def _build_walls(
    robot_name: str,
    config: ObstacleConfig,
) -> list[WallSegment]:
    """Builds arena boundary walls and conditional delivery zone border segments."""
    arena_rect = (
        0.0,
        0.0,
        config.arena_width_m,
        config.arena_height_m,
    )
    walls = _rectangle_segments(
        ObstacleKind.WALL,
        arena_rect,
    )
    is_occupant_allowed = not config.is_gate_closed and robot_name == config.zone_occupant
    if not is_occupant_allowed:
        walls.extend(
            _rectangle_segments(
                ObstacleKind.ZONE,
                config.zone_rect,
            )
        )
    return walls


def build_obstacles_for(
    robot_name: str,
    *,
    cubes: list[tuple[int, str, float, float, float]],
    robots: list[RobotPose],
    velocities: dict[str, tuple[float, float]],
    config: ObstacleConfig,
) -> ObstacleSet:
    """Builds circular and segment obstacles in the arena frame for one robot.

    Excludes the queried robot from circular obstacles. Other robots receive stable
    ``obstacle_id`` values equal to their 0-based index in the alphabetically sorted
    list of all robot names in ``robots``.

    Args:
        robot_name: Name of the ego robot for which obstacles are constructed.
        cubes: Tracked cubes as (track_id, color, x_m, y_m, confidence) in arena frame.
        robots: Observed robot poses in arena frame.
        velocities: Mapping from robot name to (vx_mps, vy_mps) in arena frame.
        config: Arena dimensions, delivery zone state, and carrying status.

    Returns:
        ObstacleSet containing all relevant circular obstacles and wall/zone segments.
    """
    cube_circles = _build_cube_circles(cubes)
    robot_circles = _build_robot_circles(
        robot_name,
        robots=robots,
        velocities=velocities,
        carrying_robots=config.carrying_robots,
    )
    walls = _build_walls(
        robot_name,
        config,
    )
    return ObstacleSet(
        circles=cube_circles + robot_circles,
        walls=walls,
    )
