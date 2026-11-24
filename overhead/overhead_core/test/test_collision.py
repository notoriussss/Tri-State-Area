"""Unit tests for forward-cone collision level and time-to-collision logic."""

import math

from overhead_core.collision import (
    assess_collision,
    closest_approach_time_s,
)
from overhead_core.constants import (
    COLLISION_SLOW_DIST_M,
    COLLISION_SLOW_TTC_S,
    COLLISION_STOP_DIST_M,
    COLLISION_STOP_TTC_S,
    RED_CUBE_SAFETY_M,
    ROBOT_SAFETY_M,
    CollisionLevel,
    ObstacleKind,
    RobotName,
)
from overhead_core.types import (
    Obstacle,
    ObstacleSet,
    RobotPose,
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


def _obstacle(
    x_m: float,
    y_m: float,
    radius_m: float,
    *,
    kind: ObstacleKind = ObstacleKind.ROBOT,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
) -> Obstacle:
    return Obstacle(
        kind=kind,
        obstacle_id=1,
        x_m=x_m,
        y_m=y_m,
        radius_m=radius_m,
        vx_mps=vx_mps,
        vy_mps=vy_mps,
    )


def test_should_compute_contact_time_when_circles_are_on_collision_course() -> None:
    ttc_s = closest_approach_time_s(
        1.0,
        0.0,
        rel_vx_mps=-0.5,
        rel_vy_mps=0.0,
        radius_m=0.2,
    )

    assert ttc_s is not None
    assert math.isclose(
        ttc_s,
        1.6,
        abs_tol=1e-6,
    )


def test_should_return_none_when_circles_miss_on_parallel_paths() -> None:
    ttc_s = closest_approach_time_s(
        1.0,
        0.5,
        rel_vx_mps=-0.5,
        rel_vy_mps=0.0,
        radius_m=0.2,
    )

    assert ttc_s is None


def test_should_return_zero_when_circles_are_already_overlapping() -> None:
    ttc_s = closest_approach_time_s(
        0.1,
        0.0,
        rel_vx_mps=-0.5,
        rel_vy_mps=0.0,
        radius_m=0.2,
    )

    assert ttc_s == 0.0


def test_should_return_none_when_circles_are_moving_apart() -> None:
    ttc_s = closest_approach_time_s(
        1.0,
        0.0,
        rel_vx_mps=0.5,
        rel_vy_mps=0.0,
        radius_m=0.2,
    )

    assert ttc_s is None


def test_should_return_stop_when_head_on_robots_reach_under_stop_ttc() -> None:
    free_dist_m = COLLISION_SLOW_DIST_M + COLLISION_STOP_DIST_M
    target_ttc_s = COLLISION_STOP_TTC_S / 2.0
    half_speed_mps = (free_dist_m / target_ttc_s) / 2.0
    pose = _pose(
        0.0,
        0.35,
        0.0,
    )
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                free_dist_m + ROBOT_SAFETY_M,
                0.35,
                ROBOT_SAFETY_M,
                vx_mps=-half_speed_mps,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (half_speed_mps, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.STOP
    assert math.isclose(
        assessment.time_to_collision_s,
        target_ttc_s,
        abs_tol=1e-6,
    )
    assert assessment.nearest_kind == ObstacleKind.ROBOT
    assert math.isclose(
        assessment.nearest_dist_m,
        free_dist_m,
        abs_tol=1e-6,
    )


def test_should_return_slow_when_head_on_robots_approach_at_moderate_speed() -> None:
    free_dist_m = COLLISION_SLOW_DIST_M + COLLISION_STOP_DIST_M
    target_ttc_s = (COLLISION_STOP_TTC_S + COLLISION_SLOW_TTC_S) / 2.0
    half_speed_mps = (free_dist_m / target_ttc_s) / 2.0
    pose = _pose(
        0.0,
        0.35,
        0.0,
    )
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                free_dist_m + ROBOT_SAFETY_M,
                0.35,
                ROBOT_SAFETY_M,
                vx_mps=-half_speed_mps,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (half_speed_mps, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.SLOW
    assert math.isclose(
        assessment.time_to_collision_s,
        target_ttc_s,
        abs_tol=1e-6,
    )


def test_should_return_clear_when_robots_are_diverging() -> None:
    pose = _pose(0.0, 0.35, 0.0)
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                0.9,
                0.35,
                ROBOT_SAFETY_M,
                vx_mps=0.4,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (0.1, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.CLEAR
    assert assessment.time_to_collision_s == -1.0


def test_should_not_raise_level_when_obstacle_is_behind_robot() -> None:
    pose = _pose(0.5, 0.35, 0.0)
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                0.25,
                0.35,
                ROBOT_SAFETY_M,
                vx_mps=0.5,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (0.0, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.CLEAR
    assert assessment.time_to_collision_s == -1.0
    assert assessment.nearest_kind is None
    assert assessment.nearest_dist_m == -1.0


def test_should_ignore_obstacle_when_located_seventy_degrees_to_the_side() -> None:
    pose = _pose(0.0, 0.0, 0.0)
    angle_rad = math.radians(70.0)
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                0.50 * math.cos(angle_rad),
                0.50 * math.sin(angle_rad),
                0.05,
                kind=ObstacleKind.CUBE,
                vx_mps=-0.2,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (0.2, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.CLEAR
    assert assessment.time_to_collision_s == -1.0
    assert assessment.nearest_kind is None
    assert assessment.nearest_dist_m == -1.0


def test_should_return_slow_when_static_obstacle_is_twenty_five_cm_ahead() -> None:
    free_dist_m = 0.25
    pose = _pose(
        0.0,
        0.35,
        0.0,
    )
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                free_dist_m + RED_CUBE_SAFETY_M,
                0.35,
                RED_CUBE_SAFETY_M,
                kind=ObstacleKind.RED_CUBE,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (0.0, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.SLOW
    assert assessment.time_to_collision_s == -1.0
    assert assessment.nearest_kind == ObstacleKind.RED_CUBE
    assert math.isclose(
        assessment.nearest_dist_m,
        free_dist_m,
        abs_tol=1e-6,
    )
    assert math.isclose(
        assessment.nearest_bearing_rad,
        0.0,
        abs_tol=1e-6,
    )


def test_should_return_stop_when_static_obstacle_is_ten_cm_ahead() -> None:
    free_dist_m = 0.10
    pose = _pose(
        0.0,
        0.35,
        0.0,
    )
    obstacles = ObstacleSet(
        circles=[
            _obstacle(
                free_dist_m + RED_CUBE_SAFETY_M,
                0.35,
                RED_CUBE_SAFETY_M,
                kind=ObstacleKind.RED_CUBE,
            ),
        ],
        walls=[],
    )

    assessment = assess_collision(
        pose,
        (0.0, 0.0),
        obstacles,
    )

    assert assessment.level == CollisionLevel.STOP
    assert assessment.time_to_collision_s == -1.0
    assert assessment.nearest_kind == ObstacleKind.RED_CUBE
    assert math.isclose(
        assessment.nearest_dist_m,
        free_dist_m,
        abs_tol=1e-6,
    )
    assert math.isclose(
        assessment.nearest_bearing_rad,
        0.0,
        abs_tol=1e-6,
    )


def test_should_return_slow_or_clear_when_red_cube_gap_is_twenty_or_thirty_five_cm() -> None:
    pose = _pose(
        0.0,
        0.35,
        0.0,
    )
    slow_gap_m = 0.20
    clear_gap_m = 0.35
    slow_obstacles = ObstacleSet(
        circles=[
            _obstacle(
                slow_gap_m + RED_CUBE_SAFETY_M,
                0.35,
                RED_CUBE_SAFETY_M,
                kind=ObstacleKind.RED_CUBE,
            ),
        ],
        walls=[],
    )
    clear_obstacles = ObstacleSet(
        circles=[
            _obstacle(
                clear_gap_m + RED_CUBE_SAFETY_M,
                0.35,
                RED_CUBE_SAFETY_M,
                kind=ObstacleKind.RED_CUBE,
            ),
        ],
        walls=[],
    )

    slow_assessment = assess_collision(
        pose,
        (0.0, 0.0),
        slow_obstacles,
    )
    clear_assessment = assess_collision(
        pose,
        (0.0, 0.0),
        clear_obstacles,
    )

    assert slow_assessment.level == CollisionLevel.SLOW
    assert math.isclose(
        slow_assessment.nearest_dist_m,
        slow_gap_m,
        abs_tol=1e-6,
    )
    assert clear_assessment.level == CollisionLevel.CLEAR
    assert math.isclose(
        clear_assessment.nearest_dist_m,
        clear_gap_m,
        abs_tol=1e-6,
    )
