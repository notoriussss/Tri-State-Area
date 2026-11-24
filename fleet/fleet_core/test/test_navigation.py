"""Unit tests for guided push planning, red avoidance (D7), and transit detours."""

import math

import pytest
from fleet_core.constants import (
    RED_STRICT_RADIUS_M,
    RED_TRANSIT_RADIUS_M,
    PushMode,
)
from fleet_core.navigation import (
    NavigationConfig,
    compute_approach_pose,
    distance_point_to_segment,
    is_corridor_clear,
    is_inside_virtual_margin,
    plan_push,
    plan_transit_waypoints,
)


def test_should_place_approach_pose_behind_cube_facing_goal() -> None:
    cube_xy = (0.50, 0.35)
    goal_xy = (0.85, 0.35)

    approach = compute_approach_pose(cube_xy, goal_xy, offset_m=0.14)

    assert approach.x_m == pytest.approx(0.36)
    assert approach.y_m == pytest.approx(0.35)
    assert approach.theta_rad == pytest.approx(0.0)


def test_should_allow_delivery_notch_only_when_zone_entry_permitted() -> None:
    inside_notch = (1.20, 0.35)

    blocked = is_inside_virtual_margin(inside_notch, allow_zone_entry=False)
    allowed = is_inside_virtual_margin(inside_notch, allow_zone_entry=True)

    assert blocked is False
    assert allowed is True


def test_should_select_direct_push_when_corridor_is_free_of_red_cubes() -> None:
    cube_xy = (0.50, 0.35)
    goal_xy = (0.85, 0.35)
    red_cubes = ((0.65, 0.55),)

    plan = plan_push(cube_xy, goal_xy, red_cubes)

    assert plan.mode == PushMode.DIRECT
    assert plan.approach_pose is not None
    assert plan.waypoints == (goal_xy,)


def test_should_select_two_stage_break_when_red_cube_blocks_push_corridor() -> None:
    cube_xy = (0.45, 0.35)
    goal_xy = (0.85, 0.35)
    blocking_red = ((0.65, 0.35),)

    plan = plan_push(cube_xy, goal_xy, blocking_red)

    assert plan.mode == PushMode.TWO_STAGE
    assert plan.approach_pose is not None
    assert len(plan.waypoints) == 2
    assert plan.waypoints[-1] == goal_xy
    assert is_corridor_clear(
        (plan.approach_pose.x_m, plan.approach_pose.y_m),
        plan.waypoints[0],
        blocking_red,
    )


def test_should_select_border_sweep_when_direct_approach_falls_outside_margin() -> None:
    cube_near_left_tape = (0.14, 0.35)
    goal_xy = (0.85, 0.35)

    plan = plan_push(cube_near_left_tape, goal_xy, red_positions=())

    assert plan.mode == PushMode.BORDER_SWEEP
    assert plan.approach_pose is not None
    assert is_inside_virtual_margin(
        (plan.approach_pose.x_m, plan.approach_pose.y_m),
    )
    assert len(plan.waypoints) == 2


def test_should_differentiate_d7_radii_when_red_cube_is_14cm_from_approach() -> None:
    cube_xy = (0.50, 0.35)
    goal_xy = (0.85, 0.35)
    # Approach is at (0.36, 0.35); red at (0.36, 0.49) is 14 cm away.
    nearby_red = ((0.36, 0.49),)
    cfg_12cm = NavigationConfig(red_transit_radius_m=RED_TRANSIT_RADIUS_M)
    cfg_17cm = NavigationConfig(red_transit_radius_m=RED_STRICT_RADIUS_M)

    plan_12cm = plan_push(cube_xy, goal_xy, nearby_red, config=cfg_12cm)
    plan_17cm = plan_push(cube_xy, goal_xy, nearby_red, config=cfg_17cm)

    assert plan_12cm.mode == PushMode.DIRECT
    assert plan_17cm.mode != PushMode.DIRECT


def test_should_detour_around_red_obstacle_when_planning_transit_waypoints() -> None:
    start_xy = (0.25, 0.35)
    goal_xy = (0.80, 0.35)
    obstacles = ((0.52, 0.35, RED_TRANSIT_RADIUS_M),)

    waypoints = plan_transit_waypoints(start_xy, goal_xy, obstacles)

    assert len(waypoints) == 2
    assert waypoints[-1] == goal_xy
    detour = waypoints[0]
    assert math.hypot(detour[0] - 0.52, detour[1] - 0.35) >= RED_TRANSIT_RADIUS_M
    assert distance_point_to_segment((0.52, 0.35), start_xy, detour) >= RED_TRANSIT_RADIUS_M
