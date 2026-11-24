"""Unit tests for distributed mission selection, crossing penalties, and parking."""

import pytest
from fleet_core.constants import (
    CubeColor,
    PlannerState,
    RobotName,
)
from fleet_core.mission import (
    default_mission_config,
    estimate_peer_cross_penalty,
    is_cube_in_delivery_zone,
    select_target_cube,
)
from fleet_core.types import (
    CubeObservation,
    PlanSnapshot,
    Pose2D,
)


def test_should_filter_only_own_color_and_ignore_delivered_cubes_when_selecting() -> None:
    cfg = default_mission_config(RobotName.PHINEAS.value)
    own_pose = Pose2D(x_m=0.25, y_m=0.40, theta_rad=0.0)
    cubes = (
        CubeObservation(1, CubeColor.YELLOW, 0.35, 0.40, 0.95),
        CubeObservation(2, CubeColor.RED, 0.60, 0.20, 0.95),
        CubeObservation(3, CubeColor.BLUE, 1.20, 0.35, 0.95),
        CubeObservation(4, CubeColor.BLUE, 0.50, 0.45, 0.95),
    )

    decision = select_target_cube(own_pose, cubes, now_s=1.0, config=cfg)

    assert is_cube_in_delivery_zone(cubes[2]) is True
    assert decision.should_park is False
    assert decision.target_cube is not None
    assert decision.target_cube.cube_id == 4


def test_should_keep_current_target_via_hysteresis_when_other_cube_is_slightly_closer() -> None:
    cfg = default_mission_config(RobotName.PHINEAS.value)
    own_pose = Pose2D(x_m=0.30, y_m=0.45, theta_rad=0.0)
    cubes = (
        CubeObservation(10, CubeColor.BLUE, 0.50, 0.45, 0.95),
        CubeObservation(11, CubeColor.BLUE, 0.48, 0.45, 0.95),
    )

    decision = select_target_cube(
        own_pose,
        cubes,
        now_s=1.0,
        config=cfg,
        current_target_id=10,
    )

    assert decision.target_cube is not None
    assert decision.target_cube.cube_id == 10


def test_should_apply_cross_penalty_only_when_peer_plan_is_fresh() -> None:
    cfg = default_mission_config(RobotName.PHINEAS.value)
    own_path = ((0.30, 0.20), (0.80, 0.50))
    peer_plan = PlanSnapshot(
        name=RobotName.FERB.value,
        state=PlannerState.PUSH,
        target_cube_id=5,
        target_pose=Pose2D(0.85, 0.25, 0.0),
        path=((0.30, 0.50), (0.80, 0.20)),
        received_at_s=10.0,
    )

    fresh_pen = estimate_peer_cross_penalty(own_path, peer_plan, now_s=10.2, config=cfg)
    stale_pen = estimate_peer_cross_penalty(own_path, peer_plan, now_s=10.8, config=cfg)

    assert fresh_pen == pytest.approx(cfg.cross_penalty_m)
    assert stale_pen == pytest.approx(0.0)


def test_should_prefer_direct_cube_over_red_blocked_cube_when_scoring() -> None:
    cfg = default_mission_config(RobotName.PHINEAS.value)
    own_pose = Pose2D(x_m=0.25, y_m=0.45, theta_rad=0.0)
    cubes = (
        CubeObservation(1, CubeColor.BLUE, 0.45, 0.45, 0.95),
        CubeObservation(2, CubeColor.RED, 0.65, 0.45, 0.95),
        CubeObservation(3, CubeColor.BLUE, 0.48, 0.22, 0.95),
    )

    decision = select_target_cube(own_pose, cubes, now_s=1.0, config=cfg)

    assert decision.target_cube is not None
    assert decision.target_cube.cube_id == 3


def test_should_signal_parked_when_no_reachable_own_cubes_remain() -> None:
    cfg = default_mission_config(RobotName.FERB.value)
    own_pose = Pose2D(x_m=0.50, y_m=0.35, theta_rad=0.0)
    cubes = (
        CubeObservation(1, CubeColor.BLUE, 0.45, 0.45, 0.95),
        CubeObservation(2, CubeColor.YELLOW, 1.21, 0.35, 0.95),
    )

    decision = select_target_cube(own_pose, cubes, now_s=1.0, config=cfg)

    assert decision.should_park is True
    assert decision.target_cube is None
