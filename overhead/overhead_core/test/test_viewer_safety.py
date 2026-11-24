"""Unit tests for the stateful viewer safety holder."""

import math

from overhead_core.constants import (
    ROBOT_HOLD_S,
    CollisionLevel,
    RobotName,
)
from overhead_core.types import RobotPose
from overhead_core.viewer_safety import (
    ViewerSafety,
    build_robot_statuses,
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


def test_should_report_stop_for_both_when_two_robots_face_each_other_half_meter_apart() -> None:
    now_s = 1.0
    holder = ViewerSafety(clock=lambda: now_s)
    phineas = _pose(
        RobotName.PHINEAS.value,
        0.30,
        0.35,
        theta_rad=0.0,
    )
    ferb = _pose(
        RobotName.FERB.value,
        0.80,
        0.35,
        theta_rad=math.pi,
    )

    result = holder.update([phineas, ferb])

    assert result.safety.per_robot[RobotName.PHINEAS.value].assessment.level == CollisionLevel.STOP
    assert result.safety.per_robot[RobotName.FERB.value].assessment.level == CollisionLevel.STOP


def test_should_report_clear_when_one_robot_alone_in_empty_arena() -> None:
    now_s = 1.0
    holder = ViewerSafety(clock=lambda: now_s)
    phineas = _pose(
        RobotName.PHINEAS.value,
        0.58,
        0.05,
        theta_rad=math.pi / 2.0,
    )

    result = holder.update([phineas])

    assert list(result.safety.per_robot.keys()) == [RobotName.PHINEAS.value]
    assert result.safety.per_robot[RobotName.PHINEAS.value].assessment.level == CollisionLevel.CLEAR


def test_should_hold_then_drop_disappeared_robot_after_hold_duration() -> None:
    now_s = 10.0
    holder = ViewerSafety(clock=lambda: now_s)
    phineas = _pose(
        RobotName.PHINEAS.value,
        0.58,
        0.05,
        theta_rad=math.pi / 2.0,
    )
    ferb = _pose(
        RobotName.FERB.value,
        0.80,
        0.35,
        theta_rad=math.pi,
    )

    initial = holder.update([phineas, ferb])
    now_s = 10.0 + (ROBOT_HOLD_S / 2.0)
    held = holder.update([phineas])
    now_s = 10.0 + ROBOT_HOLD_S + 0.05
    dropped = holder.update([phineas])

    assert len(initial.robot_poses) == 2
    assert RobotName.FERB.value in held.safety.per_robot
    assert len(held.robot_poses) == 2
    held_ferb = next(p for p in held.robot_poses if p.name == RobotName.FERB.value)
    assert math.isclose(
        held_ferb.confidence,
        0.5,
    )
    assert RobotName.FERB.value not in dropped.safety.per_robot
    assert [pose.name for pose in dropped.robot_poses] == [RobotName.PHINEAS.value]


def test_should_estimate_velocity_and_build_statuses_across_frames() -> None:
    now_s = 0.0
    holder = ViewerSafety(clock=lambda: now_s)

    holder.update(
        [
            _pose(
                RobotName.PHINEAS.value,
                0.20,
                0.35,
                theta_rad=0.0,
            ),
        ]
    )
    now_s = 0.1
    result = holder.update(
        [
            _pose(
                RobotName.PHINEAS.value,
                0.25,
                0.35,
                theta_rad=0.0,
            ),
        ]
    )
    statuses = build_robot_statuses(result.safety)

    assert result.velocities[RobotName.PHINEAS.value][0] > 0.0
    assert len(statuses) == 1
    assert statuses[0].name == RobotName.PHINEAS.value
