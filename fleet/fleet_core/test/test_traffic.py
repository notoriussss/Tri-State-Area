"""Unit tests for navigation-light P2P anti-collision and 20 Hz step validation."""

import math

import pytest
from fleet_core.constants import (
    PlannerState,
    RobotName,
    TrafficAction,
    TrafficSituation,
)
from fleet_core.traffic import (
    TrafficConfig,
    TrafficCoordinator,
    classify_encounter,
    is_step_safe,
)
from fleet_core.types import (
    PlanSnapshot,
    Pose2D,
)


def test_should_give_priority_to_restricted_robot_carrying_cube() -> None:
    own = Pose2D(x_m=0.50, y_m=0.35, theta_rad=0.0)
    peer = Pose2D(x_m=0.60, y_m=0.15, theta_rad=math.pi / 2.0)

    own_pushing = classify_encounter(
        own,
        peer,
        own_state=PlannerState.PUSH,
        peer_state=PlannerState.GO_TO_APPROACH,
    )
    peer_pushing = classify_encounter(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        peer_state=PlannerState.DELIVER,
    )

    assert own_pushing == TrafficSituation.HOLD_PRIORITY
    assert peer_pushing == TrafficSituation.SEE_RED


def test_should_yield_on_red_and_hold_on_green_when_equal_priority() -> None:
    own = Pose2D(x_m=0.50, y_m=0.35, theta_rad=0.0)
    peer_on_right = Pose2D(x_m=0.60, y_m=0.15, theta_rad=math.pi / 2.0)
    peer_on_left = Pose2D(x_m=0.60, y_m=0.55, theta_rad=-math.pi / 2.0)

    right_sit = classify_encounter(
        own,
        peer_on_right,
        own_state=PlannerState.GO_TO_APPROACH,
        peer_state=PlannerState.GO_TO_APPROACH,
    )
    left_sit = classify_encounter(
        own,
        peer_on_left,
        own_state=PlannerState.GO_TO_APPROACH,
        peer_state=PlannerState.GO_TO_APPROACH,
    )

    assert right_sit == TrafficSituation.SEE_RED
    assert left_sit == TrafficSituation.SEE_GREEN


def test_should_command_starboard_sidestep_when_head_on_and_starboard_clear() -> None:
    coord = TrafficCoordinator(TrafficConfig(robot_name=RobotName.FERB.value))
    own = Pose2D(x_m=0.50, y_m=0.45, theta_rad=0.0)
    peer = Pose2D(x_m=0.72, y_m=0.45, theta_rad=math.pi)

    decision = coord.evaluate(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        red_positions=(),
        now_s=1.0,
    )

    assert decision.situation == TrafficSituation.HEAD_ON
    assert decision.action == TrafficAction.SIDESTEP_STARBOARD
    assert decision.must_wait is False
    assert decision.evasion_cmd is not None
    assert decision.evasion_cmd.vy_mps < 0.0
    assert decision.evasion_cmd.vx_mps == pytest.approx(0.0)


def test_should_wait_instead_of_sidestep_when_red_cube_blocks_starboard() -> None:
    coord = TrafficCoordinator(TrafficConfig(robot_name=RobotName.FERB.value))
    own = Pose2D(x_m=0.50, y_m=0.45, theta_rad=0.0)
    peer = Pose2D(x_m=0.72, y_m=0.45, theta_rad=math.pi)
    red_on_starboard = ((0.50, 0.34),)

    safe = is_step_safe(own, (0.0, -0.18), red_on_starboard)
    decision = coord.evaluate(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        red_positions=red_on_starboard,
        now_s=1.0,
    )

    assert safe is False
    assert decision.situation == TrafficSituation.HEAD_ON
    assert decision.action == TrafficAction.WAIT
    assert decision.must_wait is True
    assert decision.evasion_cmd is None


def test_should_reverse_after_deadlock_timeout_on_ferb_before_phineas() -> None:
    ferb_coord = TrafficCoordinator(TrafficConfig(robot_name=RobotName.FERB.value))
    phineas_coord = TrafficCoordinator(TrafficConfig(robot_name=RobotName.PHINEAS.value))
    own = Pose2D(x_m=0.50, y_m=0.20, theta_rad=0.0)
    peer = Pose2D(x_m=0.72, y_m=0.20, theta_rad=math.pi)
    # Starboard (-y) hits bottom border margin (y = 0.02 < 0.11).

    ferb_t0 = ferb_coord.evaluate(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        red_positions=(),
        now_s=10.0,
    )
    ferb_t3 = ferb_coord.evaluate(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        red_positions=(),
        now_s=13.1,
    )
    phineas_coord.evaluate(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        red_positions=(),
        now_s=10.0,
    )
    phineas_t3 = phineas_coord.evaluate(
        own,
        peer,
        own_state=PlannerState.GO_TO_APPROACH,
        red_positions=(),
        now_s=13.1,
    )

    assert ferb_t0.action == TrafficAction.WAIT
    assert ferb_t3.action == TrafficAction.DEADLOCK_REVERSE
    assert ferb_t3.evasion_cmd is not None
    assert ferb_t3.evasion_cmd.vx_mps < 0.0
    assert phineas_t3.action == TrafficAction.WAIT


def test_should_operate_without_peer_plan_when_peer_plan_is_stale() -> None:
    coord = TrafficCoordinator(TrafficConfig(robot_name=RobotName.PHINEAS.value))
    own = Pose2D(x_m=0.50, y_m=0.45, theta_rad=0.0)
    peer = Pose2D(x_m=0.60, y_m=0.25, theta_rad=math.pi / 2.0)
    stale_plan = PlanSnapshot(
        name=RobotName.FERB.value,
        state=PlannerState.DELIVER,
        target_cube_id=1,
        target_pose=Pose2D(1.20, 0.35, 0.0),
        path=(),
        received_at_s=1.0,
    )

    decision = coord.evaluate(
        own,
        peer,
        own_state=PlannerState.PUSH,
        red_positions=(),
        now_s=5.0,
        peer_plan=stale_plan,
    )

    assert decision.situation == TrafficSituation.HOLD_PRIORITY
    assert decision.action == TrafficAction.HOLD_COURSE
