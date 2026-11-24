"""Unit tests for the pure corridor clearance query."""

from overhead_core.constants import (
    RED_CUBE_SAFETY_M,
    ObstacleKind,
)
from overhead_core.corridor import corridor_free
from overhead_core.types import Obstacle


def _obstacle(
    obstacle_id: int,
    x_m: float,
    y_m: float,
    *,
    kind: ObstacleKind = ObstacleKind.RED_CUBE,
    radius_m: float = RED_CUBE_SAFETY_M,
) -> Obstacle:
    return Obstacle(
        kind=kind,
        obstacle_id=obstacle_id,
        x_m=x_m,
        y_m=y_m,
        radius_m=radius_m,
        vx_mps=0.0,
        vy_mps=0.0,
    )


def test_should_return_free_when_no_red_obstacles_intersect_corridor() -> None:
    obstacles = [
        _obstacle(1, 0.50, 0.80),
    ]

    result = corridor_free(
        (0.0, 0.20),
        (1.0, 0.20),
        width_m=0.20,
        obstacles=obstacles,
    )

    assert result.is_free is True
    assert result.blocking_ids == ()


def test_should_block_when_red_circle_lies_on_corridor_segment() -> None:
    obstacles = [
        _obstacle(4, 0.50, 0.35),
    ]

    result = corridor_free(
        (0.0, 0.35),
        (1.0, 0.35),
        width_m=0.20,
        obstacles=obstacles,
    )

    assert result.is_free is False
    assert result.blocking_ids == (4,)


def test_should_return_free_when_red_circle_is_just_outside_half_width() -> None:
    width_m = 0.20
    radius_m = 0.17
    clearance_offset_m = radius_m + width_m / 2.0 + 0.01
    obstacles = [
        _obstacle(
            2,
            0.50,
            0.35 + clearance_offset_m,
            radius_m=radius_m,
        ),
    ]

    result = corridor_free(
        (0.0, 0.35),
        (1.0, 0.35),
        width_m=width_m,
        obstacles=obstacles,
    )

    assert result.is_free is True
    assert result.blocking_ids == ()


def test_should_measure_to_segment_end_when_red_circle_is_beyond_endpoint() -> None:
    obstacles = [
        _obstacle(
            7,
            1.30,
            0.35,
            radius_m=0.17,
        ),
    ]

    result = corridor_free(
        (0.0, 0.35),
        (1.0, 0.35),
        width_m=0.20,
        obstacles=obstacles,
    )

    assert result.is_free is True
    assert result.blocking_ids == ()


def test_should_return_sorted_blocking_ids_when_several_red_circles_block() -> None:
    obstacles = [
        _obstacle(12, 0.30, 0.35),
        _obstacle(3, 0.70, 0.35),
        _obstacle(8, 0.50, 0.90),
        _obstacle(5, 0.50, 0.40),
    ]

    result = corridor_free(
        (0.0, 0.35),
        (1.0, 0.35),
        width_m=0.20,
        obstacles=obstacles,
    )

    assert result.is_free is False
    assert result.blocking_ids == (
        3,
        5,
        12,
    )


def test_should_ignore_non_red_obstacles_when_checking_corridor() -> None:
    obstacles = [
        _obstacle(1, 0.20, 0.35, kind=ObstacleKind.ROBOT),
        _obstacle(2, 0.40, 0.35, kind=ObstacleKind.CUBE),
        _obstacle(3, 0.60, 0.35, kind=ObstacleKind.WALL),
        _obstacle(4, 0.80, 0.35, kind=ObstacleKind.ZONE),
    ]

    result = corridor_free(
        (0.0, 0.35),
        (1.0, 0.35),
        width_m=0.20,
        obstacles=obstacles,
    )

    assert result.is_free is True
    assert result.blocking_ids == ()


def test_should_handle_degenerate_segment_as_point_when_start_equals_goal() -> None:
    obstacles = [
        _obstacle(
            9,
            0.65,
            0.50,
            radius_m=0.17,
        ),
        _obstacle(
            10,
            0.90,
            0.50,
            radius_m=0.17,
        ),
    ]

    result = corridor_free(
        (0.50, 0.50),
        (0.50, 0.50),
        width_m=0.20,
        obstacles=obstacles,
    )

    assert result.is_free is False
    assert result.blocking_ids == (9,)
