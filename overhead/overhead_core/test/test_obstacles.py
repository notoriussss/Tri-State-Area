"""Unit tests for obstacle and safety circle construction."""

import math

from overhead_core.constants import (
    CARRY_EXTENSION_M,
    CUBE_SAFETY_M,
    RED_CUBE_SAFETY_M,
    ROBOT_SAFETY_M,
    CubeColor,
    ObstacleKind,
    RobotName,
)
from overhead_core.obstacles import (
    ObstacleConfig,
    build_obstacles_for,
)
from overhead_core.types import RobotPose


def _default_config(
    *,
    zone_occupant: str | None = None,
    is_gate_closed: bool = False,
    carrying_robots: frozenset[str] = frozenset(),
) -> ObstacleConfig:
    return ObstacleConfig(
        arena_width_m=1.20,
        arena_height_m=0.80,
        zone_rect=(1.20, 0.25, 1.35, 0.45),
        zone_occupant=zone_occupant,
        is_gate_closed=is_gate_closed,
        carrying_robots=carrying_robots,
    )


def test_should_assign_red_and_normal_safety_radii_when_cubes_present() -> None:
    cubes = [
        (1, CubeColor.RED, 0.30, 0.20, 0.9),
        (2, CubeColor.BLUE, 0.50, 0.40, 0.9),
        (3, CubeColor.YELLOW, 0.70, 0.60, 0.8),
    ]

    obstacle_set = build_obstacles_for(
        RobotName.PHINEAS,
        cubes=cubes,
        robots=[],
        velocities={},
        config=_default_config(),
    )

    assert len(obstacle_set.circles) == 3
    by_id = {c.obstacle_id: c for c in obstacle_set.circles}
    assert by_id[1].kind == ObstacleKind.RED_CUBE
    assert math.isclose(
        by_id[1].radius_m,
        RED_CUBE_SAFETY_M,
    )
    assert by_id[2].kind == ObstacleKind.CUBE
    assert math.isclose(
        by_id[2].radius_m,
        CUBE_SAFETY_M,
    )
    assert by_id[3].kind == ObstacleKind.CUBE
    assert math.isclose(
        by_id[3].radius_m,
        CUBE_SAFETY_M,
    )


def test_should_exclude_self_and_include_other_robot_velocity_when_building_obstacles() -> None:
    robots = [
        RobotPose(
            name=RobotName.PHINEAS,
            x=0.20,
            y=0.30,
            theta=0.0,
            confidence=1.0,
        ),
        RobotPose(
            name=RobotName.FERB,
            x=0.80,
            y=0.50,
            theta=1.0,
            confidence=0.9,
        ),
    ]
    velocities = {
        RobotName.PHINEAS: (0.10, 0.0),
        RobotName.FERB: (-0.25, 0.15),
    }

    obstacle_set = build_obstacles_for(
        RobotName.PHINEAS,
        cubes=[],
        robots=robots,
        velocities=velocities,
        config=_default_config(),
    )

    assert len(obstacle_set.circles) == 1
    other = obstacle_set.circles[0]
    assert other.kind == ObstacleKind.ROBOT
    assert other.obstacle_id == 0
    assert math.isclose(
        other.x_m,
        0.80,
    )
    assert math.isclose(
        other.y_m,
        0.50,
    )
    assert math.isclose(
        other.radius_m,
        ROBOT_SAFETY_M,
    )
    assert other.vx_mps == -0.25
    assert other.vy_mps == 0.15


def test_should_extend_radius_when_other_robot_is_carrying_cube() -> None:
    robots = [
        RobotPose(
            name=RobotName.PHINEAS,
            x=0.20,
            y=0.30,
            theta=0.0,
            confidence=1.0,
        ),
        RobotPose(
            name=RobotName.FERB,
            x=0.80,
            y=0.50,
            theta=0.0,
            confidence=1.0,
        ),
    ]

    obstacle_set = build_obstacles_for(
        RobotName.FERB,
        cubes=[],
        robots=robots,
        velocities={},
        config=_default_config(
            carrying_robots=frozenset({RobotName.PHINEAS}),
        ),
    )

    assert len(obstacle_set.circles) == 1
    phineas_circle = obstacle_set.circles[0]
    assert phineas_circle.obstacle_id == 1
    assert math.isclose(
        phineas_circle.radius_m,
        ROBOT_SAFETY_M + CARRY_EXTENSION_M,
    )


def test_should_build_four_arena_wall_segments_when_config_given() -> None:
    obstacle_set = build_obstacles_for(
        RobotName.PHINEAS,
        cubes=[],
        robots=[],
        velocities={},
        config=_default_config(
            zone_occupant=RobotName.PHINEAS,
            is_gate_closed=False,
        ),
    )

    wall_segments = [w for w in obstacle_set.walls if w.kind == ObstacleKind.WALL]
    assert len(wall_segments) == 4
    expected_coords = {
        (0.0, 0.0, 1.20, 0.0),
        (1.20, 0.0, 1.20, 0.80),
        (1.20, 0.80, 0.0, 0.80),
        (0.0, 0.80, 0.0, 0.0),
    }
    actual_coords = {(w.x0_m, w.y0_m, w.x1_m, w.y1_m) for w in wall_segments}
    assert actual_coords == expected_coords


def test_should_include_zone_segments_only_for_non_occupant_when_gate_open() -> None:
    config = _default_config(
        zone_occupant=RobotName.PHINEAS,
        is_gate_closed=False,
    )

    occupant_set = build_obstacles_for(
        RobotName.PHINEAS,
        cubes=[],
        robots=[],
        velocities={},
        config=config,
    )
    non_occupant_set = build_obstacles_for(
        RobotName.FERB,
        cubes=[],
        robots=[],
        velocities={},
        config=config,
    )

    occupant_zone = [w for w in occupant_set.walls if w.kind == ObstacleKind.ZONE]
    non_occupant_zone = [w for w in non_occupant_set.walls if w.kind == ObstacleKind.ZONE]
    assert len(occupant_zone) == 0
    assert len(non_occupant_zone) == 4


def test_should_include_zone_segments_for_everyone_when_gate_closed() -> None:
    config = _default_config(
        zone_occupant=RobotName.PHINEAS,
        is_gate_closed=True,
    )

    occupant_set = build_obstacles_for(
        RobotName.PHINEAS,
        cubes=[],
        robots=[],
        velocities={},
        config=config,
    )
    non_occupant_set = build_obstacles_for(
        RobotName.FERB,
        cubes=[],
        robots=[],
        velocities={},
        config=config,
    )

    occupant_zone = [w for w in occupant_set.walls if w.kind == ObstacleKind.ZONE]
    non_occupant_zone = [w for w in non_occupant_set.walls if w.kind == ObstacleKind.ZONE]
    assert len(occupant_zone) == 4
    assert len(non_occupant_zone) == 4
