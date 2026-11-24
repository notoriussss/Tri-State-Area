"""Unit tests for arena-wide world safety evaluation."""

import math

from overhead_core.constants import (
    COLLISION_STOP_DIST_M,
    RADAR_SECTOR_COUNT,
    RED_CUBE_SAFETY_M,
    CollisionLevel,
    CubeColor,
    ObstacleKind,
    RobotName,
)
from overhead_core.types import RobotPose
from overhead_core.world_safety import (
    SafetyConfig,
    compute_safety,
)


def _pose(
    name: str,
    x_m: float,
    y_m: float,
    *,
    theta_rad: float = 0.0,
) -> RobotPose:
    return RobotPose(
        name=name,
        x=x_m,
        y=y_m,
        theta=theta_rad,
        confidence=1.0,
    )


def test_should_return_stop_for_both_when_two_robots_move_head_on() -> None:
    robot_poses = [
        _pose(RobotName.PHINEAS, 0.30, 0.35, theta_rad=0.0),
        _pose(RobotName.FERB, 0.90, 0.35, theta_rad=math.pi),
    ]
    velocities = {
        RobotName.PHINEAS: (0.4, 0.0),
        RobotName.FERB: (-0.4, 0.0),
    }

    result = compute_safety(
        robot_poses,
        [],
        velocities=velocities,
        config=SafetyConfig(),
    )

    assert len(result.circles) == 2
    assert all(circle.kind == ObstacleKind.ROBOT for circle in result.circles)
    assert result.per_robot[RobotName.PHINEAS].assessment.level == CollisionLevel.STOP
    assert result.per_robot[RobotName.FERB].assessment.level == CollisionLevel.STOP


def test_should_return_clear_with_eight_sectors_when_one_robot_alone_in_empty_arena() -> None:
    robot_poses = [
        _pose(RobotName.PHINEAS, 0.70, 1.00),
    ]
    config = SafetyConfig(
        arena_width_m=2.0,
        arena_height_m=2.0,
        zone_rect=(2.0, 0.80, 2.20, 1.20),
    )

    result = compute_safety(
        robot_poses,
        [],
        velocities={},
        config=config,
    )

    phineas_safety = result.per_robot[RobotName.PHINEAS]
    assert [circle.kind for circle in result.circles] == [ObstacleKind.ROBOT]
    assert len(phineas_safety.sectors) == RADAR_SECTOR_COUNT
    assert phineas_safety.assessment.level == CollisionLevel.CLEAR


def test_should_lower_sector_zero_when_red_cube_is_straight_ahead() -> None:
    robot_poses = [
        _pose(RobotName.PHINEAS, 0.20, 0.35),
    ]
    cubes = [
        (1, CubeColor.RED, 0.55, 0.35, 1.0),
    ]
    config = SafetyConfig()

    empty_result = compute_safety(
        robot_poses,
        [],
        velocities={},
        config=config,
    )
    cube_result = compute_safety(
        robot_poses,
        cubes,
        velocities={},
        config=config,
    )

    empty_sector_zero = empty_result.per_robot[RobotName.PHINEAS].sectors[0]
    cube_sector_zero = cube_result.per_robot[RobotName.PHINEAS].sectors[0]
    expected_dist_m = 0.55 - 0.20 - RED_CUBE_SAFETY_M
    assert cube_sector_zero < empty_sector_zero
    assert math.isclose(
        cube_sector_zero,
        expected_dist_m,
        abs_tol=1e-6,
    )
    assert len(cube_result.circles) == 2  # the red cube and the robot itself
    assert cube_result.circles[0].kind == ObstacleKind.RED_CUBE


def test_should_block_zone_for_non_occupant_but_allow_occupant_when_gate_open() -> None:
    zone_rect = (0.85, 0.90, 1.10, 1.10)
    robot_x_m = zone_rect[0] - (COLLISION_STOP_DIST_M / 2.0)
    robot_poses = [
        _pose(
            RobotName.PHINEAS,
            robot_x_m,
            1.00,
        ),
    ]
    occupant_config = SafetyConfig(
        arena_width_m=2.0,
        arena_height_m=2.0,
        zone_rect=zone_rect,
        zone_occupant=RobotName.PHINEAS,
        is_gate_closed=False,
    )
    non_occupant_config = SafetyConfig(
        arena_width_m=2.0,
        arena_height_m=2.0,
        zone_rect=zone_rect,
        zone_occupant=RobotName.FERB,
        is_gate_closed=False,
    )

    occupant_result = compute_safety(
        robot_poses,
        [],
        velocities={},
        config=occupant_config,
    )
    non_occupant_result = compute_safety(
        robot_poses,
        [],
        velocities={},
        config=non_occupant_config,
    )

    occupant_assessment = occupant_result.per_robot[RobotName.PHINEAS].assessment
    non_occupant_assessment = non_occupant_result.per_robot[RobotName.PHINEAS].assessment
    assert occupant_assessment.level == CollisionLevel.CLEAR
    assert non_occupant_assessment.level == CollisionLevel.STOP
    assert non_occupant_assessment.nearest_kind == ObstacleKind.ZONE


def test_should_block_zone_even_for_occupant_when_gate_is_closed() -> None:
    zone_rect = (0.85, 0.90, 1.10, 1.10)
    robot_x_m = zone_rect[0] - (COLLISION_STOP_DIST_M / 2.0)
    robot_poses = [
        _pose(
            RobotName.PHINEAS,
            robot_x_m,
            1.00,
        ),
    ]
    config = SafetyConfig(
        arena_width_m=2.0,
        arena_height_m=2.0,
        zone_rect=zone_rect,
        zone_occupant=RobotName.PHINEAS,
        is_gate_closed=True,
    )

    result = compute_safety(
        robot_poses,
        [],
        velocities={},
        config=config,
    )

    assessment = result.per_robot[RobotName.PHINEAS].assessment
    assert assessment.level == CollisionLevel.STOP
    assert assessment.nearest_kind == ObstacleKind.ZONE
