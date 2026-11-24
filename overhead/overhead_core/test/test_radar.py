"""Unit tests for robot-centric radar sector computation and ray casting."""

import math

from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    RADAR_MAX_RANGE_M,
    RADAR_SECTOR_COUNT,
    ObstacleKind,
    RobotName,
)
from overhead_core.obstacles import (
    ObstacleConfig,
    build_obstacles_for,
)
from overhead_core.radar import (
    compute_radar,
    ray_free_distance,
)
from overhead_core.types import (
    Obstacle,
    ObstacleSet,
    RobotPose,
    WallSegment,
)


def _pose(
    x_m: float,
    y_m: float,
    theta_rad: float = 0.0,
) -> RobotPose:
    return RobotPose(
        name=RobotName.PHINEAS,
        x=x_m,
        y=y_m,
        theta=theta_rad,
        confidence=1.0,
    )


def _circle(
    x_m: float,
    y_m: float,
    radius_m: float,
    *,
    kind: ObstacleKind = ObstacleKind.RED_CUBE,
) -> Obstacle:
    return Obstacle(
        kind=kind,
        obstacle_id=1,
        x_m=x_m,
        y_m=y_m,
        radius_m=radius_m,
        vx_mps=0.0,
        vy_mps=0.0,
    )


def test_should_return_wall_distances_when_arena_is_empty() -> None:
    pose = _pose(
        ARENA_WIDTH_M / 2.0,
        ARENA_HEIGHT_M / 2.0,
        0.0,
    )
    obstacles = build_obstacles_for(
        RobotName.PHINEAS,
        cubes=[],
        robots=[],
        velocities={},
        config=ObstacleConfig(
            zone_occupant=RobotName.PHINEAS,
        ),
    )

    sectors = compute_radar(
        pose,
        obstacles,
    )

    assert len(sectors) == RADAR_SECTOR_COUNT
    assert math.isclose(
        sectors[0],
        ARENA_WIDTH_M / 2.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[2],
        ARENA_HEIGHT_M / 2.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[4],
        ARENA_WIDTH_M / 2.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[6],
        ARENA_HEIGHT_M / 2.0,
        abs_tol=1e-6,
    )


def test_should_place_circle_ahead_in_sector_zero_when_in_front_of_scoop() -> None:
    pose = _pose(0.20, 0.35, 0.0)
    obstacles = ObstacleSet(
        circles=[_circle(0.70, 0.35, 0.17)],
        walls=[],
    )

    sectors = compute_radar(
        pose,
        obstacles,
    )

    assert math.isclose(
        sectors[0],
        0.33,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[2],
        RADAR_MAX_RANGE_M,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[4],
        RADAR_MAX_RANGE_M,
        abs_tol=1e-6,
    )


def test_should_place_circle_on_left_in_sector_two_when_at_positive_y() -> None:
    pose = _pose(0.50, 0.20, 0.0)
    obstacles = ObstacleSet(
        circles=[
            _circle(
                0.50,
                0.50,
                0.05,
                kind=ObstacleKind.CUBE,
            ),
        ],
        walls=[],
    )

    sectors = compute_radar(
        pose,
        obstacles,
    )

    assert math.isclose(
        sectors[2],
        0.25,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[0],
        RADAR_MAX_RANGE_M,
        abs_tol=1e-6,
    )


def test_should_place_circle_behind_in_sector_four_when_at_negative_x() -> None:
    pose = _pose(0.70, 0.35, 0.0)
    obstacles = ObstacleSet(
        circles=[_circle(0.30, 0.35, 0.10)],
        walls=[],
    )

    sectors = compute_radar(
        pose,
        obstacles,
    )

    assert math.isclose(
        sectors[4],
        0.30,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors[0],
        RADAR_MAX_RANGE_M,
        abs_tol=1e-6,
    )


def test_should_return_zero_when_robot_center_is_inside_circle() -> None:
    pose = _pose(0.50, 0.35, 0.0)
    obstacles = ObstacleSet(
        circles=[_circle(0.55, 0.35, 0.17)],
        walls=[],
    )

    sectors = compute_radar(
        pose,
        obstacles,
    )

    assert all(dist == 0.0 for dist in sectors)


def test_should_move_obstacle_to_matching_sector_when_robot_rotates_ninety_degrees() -> None:
    obstacles = ObstacleSet(
        circles=[_circle(0.50, 0.65, 0.05)],
        walls=[],
    )
    facing_east = _pose(0.50, 0.25, 0.0)
    facing_north = _pose(
        0.50,
        0.25,
        math.pi / 2.0,
    )
    facing_south = _pose(
        0.50,
        0.25,
        -math.pi / 2.0,
    )

    sectors_east = compute_radar(
        facing_east,
        obstacles,
    )
    sectors_north = compute_radar(
        facing_north,
        obstacles,
    )
    sectors_south = compute_radar(
        facing_south,
        obstacles,
    )

    assert math.isclose(
        sectors_east[2],
        0.35,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors_north[0],
        0.35,
        abs_tol=1e-6,
    )
    assert math.isclose(
        sectors_south[4],
        0.35,
        abs_tol=1e-6,
    )


def test_should_cap_distances_at_max_range_when_obstacles_exceed_range() -> None:
    pose = _pose(0.00, 0.35, 0.0)
    empty_obstacles = ObstacleSet(
        circles=[],
        walls=[],
    )
    far_circle_obstacles = ObstacleSet(
        circles=[_circle(1.00, 0.35, 0.10)],
        walls=[],
    )

    empty_sectors = compute_radar(
        pose,
        empty_obstacles,
    )
    capped_sectors = compute_radar(
        pose,
        far_circle_obstacles,
        max_range_m=0.40,
    )

    assert empty_sectors == [RADAR_MAX_RANGE_M] * RADAR_SECTOR_COUNT
    assert capped_sectors == [0.40] * RADAR_SECTOR_COUNT


def test_should_return_nearest_hit_kind_when_casting_single_ray() -> None:
    obstacles = ObstacleSet(
        circles=[
            _circle(
                0.50,
                0.35,
                0.17,
                kind=ObstacleKind.RED_CUBE,
            ),
        ],
        walls=[
            WallSegment(
                kind=ObstacleKind.WALL,
                x0_m=1.00,
                y0_m=0.00,
                x1_m=1.00,
                y1_m=0.70,
            ),
        ],
    )

    hit_dist, hit_kind = ray_free_distance(
        0.00,
        0.35,
        0.0,
        obstacles=obstacles,
        max_range_m=RADAR_MAX_RANGE_M,
    )
    miss_dist, miss_kind = ray_free_distance(
        0.00,
        0.35,
        math.pi,
        obstacles=obstacles,
        max_range_m=RADAR_MAX_RANGE_M,
    )

    assert math.isclose(
        hit_dist,
        0.33,
        abs_tol=1e-6,
    )
    assert hit_kind == ObstacleKind.RED_CUBE
    assert math.isclose(
        miss_dist,
        RADAR_MAX_RANGE_M,
        abs_tol=1e-6,
    )
    assert miss_kind is None
