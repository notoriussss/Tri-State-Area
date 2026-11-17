"""Unit tests for virtual sensors and proximity events."""

import math

from overhead_core.virtual_sensors import compute

ZONE_RECT = (1.0, 0.2, 1.2, 0.4)


def test_should_emit_entered_zone_event_when_robot_enters_rect():
    robots = {"phineas": (1.1, 0.3)}
    cubes = []
    prev_in_zone = set()

    result = compute(
        robots,
        cubes,
        zone_rect=ZONE_RECT,
        prev_in_zone=prev_in_zone,
    )

    events = result["per_robot"]["phineas"][2]
    assert "entered:zone" in events
    assert "phineas" in result["robots_in_zone"]
    assert "phineas" in result["in_zone"]


def test_should_emit_left_zone_event_when_robot_leaves_rect():
    robots = {"phineas": (0.5, 0.3)}
    cubes = []
    prev_in_zone = {"phineas"}

    result = compute(
        robots,
        cubes,
        zone_rect=ZONE_RECT,
        prev_in_zone=prev_in_zone,
    )

    events = result["per_robot"]["phineas"][2]
    assert "left:zone" in events
    assert "phineas" not in result["robots_in_zone"]


def test_should_emit_near_red_event_when_red_cube_within_distance():
    robots = {"phineas": (0.50, 0.30)}
    cubes = [
        (7, "red", 0.60, 0.30),
    ]

    result = compute(
        robots,
        cubes,
        zone_rect=ZONE_RECT,
        near_red_m=0.17,
    )

    nearest_id, nearest_dist, events = result["per_robot"]["phineas"]
    assert nearest_id == 7
    assert math.isclose(
        nearest_dist,
        0.10,
    )
    assert "near:red:7" in events


def test_should_return_negative_distance_when_fewer_than_two_robots():
    robots = {"phineas": (0.5, 0.3)}

    result = compute(
        robots,
        [],
        zone_rect=ZONE_RECT,
    )

    assert result["robot_distance"] == -1.0


def test_should_return_default_red_info_when_no_red_cubes_present():
    robots = {"phineas": (0.5, 0.3)}
    cubes = [
        (1, "blue", 0.55, 0.3),
    ]

    result = compute(
        robots,
        cubes,
        zone_rect=ZONE_RECT,
    )

    nearest_id, nearest_dist, events = result["per_robot"]["phineas"]
    assert nearest_id == 0
    assert nearest_dist == -1.0
    assert len(events) == 0
