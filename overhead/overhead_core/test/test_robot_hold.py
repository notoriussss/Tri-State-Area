"""Unit tests for the lost robot hold filter."""

import math

from overhead_core.constants import (
    ROBOT_HOLD_S,
    RobotName,
)
from overhead_core.robot_hold import RobotHoldFilter
from overhead_core.types import RobotPose

UNEXPECTED_ROBOT_NAME: str = "candace"


def _make_pose(
    name: str,
    *,
    x: float = 0.5,
    y: float = 0.3,
    theta: float = 0.0,
    confidence: float = 0.9,
) -> RobotPose:
    return RobotPose(
        name=name,
        x=x,
        y=y,
        theta=theta,
        confidence=confidence,
    )


def test_should_pass_seen_robots_through_unchanged_when_all_expected_are_present() -> None:
    expected_names = (
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    )
    hold_filter = RobotHoldFilter(expected_names)
    phineas_pose = _make_pose(
        RobotName.PHINEAS.value,
        x=0.4,
        y=0.2,
        theta=0.5,
        confidence=0.9,
    )
    ferb_pose = _make_pose(
        RobotName.FERB.value,
        x=0.8,
        y=0.5,
        theta=-0.3,
        confidence=0.85,
    )
    seen = {
        RobotName.PHINEAS.value: phineas_pose,
        RobotName.FERB.value: ferb_pose,
    }

    result = hold_filter.update(
        seen,
        now_s=1.0,
    )

    assert result.poses == [
        ferb_pose,
        phineas_pose,
    ]
    assert result.lost_names == ()
    assert result.is_all_seen is True


def test_should_hold_missing_robot_with_half_confidence_when_at_half_hold_time() -> None:
    expected_names = (
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    )
    hold_filter = RobotHoldFilter(
        expected_names,
        hold_s=ROBOT_HOLD_S,
    )
    phineas_pose = _make_pose(
        RobotName.PHINEAS.value,
        x=0.3,
        y=0.4,
        theta=1.2,
        confidence=0.8,
    )
    ferb_pose = _make_pose(
        RobotName.FERB.value,
        x=0.7,
        y=0.2,
        theta=-0.5,
        confidence=0.95,
    )

    hold_filter.update(
        {
            RobotName.PHINEAS.value: phineas_pose,
            RobotName.FERB.value: ferb_pose,
        },
        now_s=1.0,
    )
    result = hold_filter.update(
        {RobotName.FERB.value: ferb_pose},
        now_s=1.0 + (ROBOT_HOLD_S / 2),
    )

    assert result.is_all_seen is False
    assert result.lost_names == (RobotName.PHINEAS.value,)
    assert len(result.poses) == 2
    assert result.poses[0] == ferb_pose
    held_phineas = result.poses[1]
    assert held_phineas.name == RobotName.PHINEAS.value
    assert held_phineas.x == phineas_pose.x
    assert held_phineas.y == phineas_pose.y
    assert held_phineas.theta == phineas_pose.theta
    assert math.isclose(
        held_phineas.confidence,
        0.5 * phineas_pose.confidence,
    )


def test_should_drop_missing_robot_when_elapsed_time_exceeds_hold_s() -> None:
    expected_names = (
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    )
    hold_filter = RobotHoldFilter(
        expected_names,
        hold_s=0.5,
    )
    phineas_pose = _make_pose(RobotName.PHINEAS.value)
    ferb_pose = _make_pose(RobotName.FERB.value)

    hold_filter.update(
        {
            RobotName.PHINEAS.value: phineas_pose,
            RobotName.FERB.value: ferb_pose,
        },
        now_s=0.0,
    )
    result = hold_filter.update(
        {RobotName.FERB.value: ferb_pose},
        now_s=0.6,
    )

    assert result.poses == [ferb_pose]
    assert result.lost_names == (RobotName.PHINEAS.value,)
    assert result.is_all_seen is False


def test_should_reset_hold_state_when_lost_robot_reappears() -> None:
    hold_filter = RobotHoldFilter(
        (RobotName.PHINEAS.value,),
        hold_s=0.5,
    )
    initial_pose = _make_pose(
        RobotName.PHINEAS.value,
        x=0.1,
        confidence=0.8,
    )
    reappeared_pose = _make_pose(
        RobotName.PHINEAS.value,
        x=0.6,
        confidence=0.9,
    )

    hold_filter.update(
        {RobotName.PHINEAS.value: initial_pose},
        now_s=0.0,
    )
    hold_filter.update(
        {},
        now_s=0.6,
    )
    reappeared_result = hold_filter.update(
        {RobotName.PHINEAS.value: reappeared_pose},
        now_s=1.0,
    )
    held_again_result = hold_filter.update(
        {},
        now_s=1.25,
    )

    assert reappeared_result.poses == [reappeared_pose]
    assert reappeared_result.lost_names == ()
    assert reappeared_result.is_all_seen is True
    assert held_again_result.lost_names == (RobotName.PHINEAS.value,)
    assert held_again_result.is_all_seen is False
    assert len(held_again_result.poses) == 1
    assert held_again_result.poses[0].x == reappeared_pose.x
    assert math.isclose(
        held_again_result.poses[0].confidence,
        0.5 * reappeared_pose.confidence,
    )


def test_should_report_never_seen_robot_only_in_lost_names() -> None:
    expected_names = (
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    )
    hold_filter = RobotHoldFilter(expected_names)
    phineas_pose = _make_pose(RobotName.PHINEAS.value)

    result = hold_filter.update(
        {RobotName.PHINEAS.value: phineas_pose},
        now_s=0.0,
    )

    assert result.poses == [phineas_pose]
    assert result.lost_names == (RobotName.FERB.value,)
    assert result.is_all_seen is False


def test_should_ignore_unexpected_robot_name_when_present_in_seen() -> None:
    hold_filter = RobotHoldFilter((RobotName.PHINEAS.value,))
    phineas_pose = _make_pose(RobotName.PHINEAS.value)
    unexpected_pose = _make_pose(UNEXPECTED_ROBOT_NAME)

    result = hold_filter.update(
        {
            RobotName.PHINEAS.value: phineas_pose,
            UNEXPECTED_ROBOT_NAME: unexpected_pose,
        },
        now_s=0.0,
    )

    assert result.poses == [phineas_pose]
    assert result.lost_names == ()
    assert result.is_all_seen is True


def test_should_treat_held_age_as_zero_when_timestamp_goes_backwards() -> None:
    hold_filter = RobotHoldFilter(
        (RobotName.PHINEAS.value,),
        hold_s=0.5,
    )
    phineas_pose = _make_pose(
        RobotName.PHINEAS.value,
        confidence=0.8,
    )

    hold_filter.update(
        {RobotName.PHINEAS.value: phineas_pose},
        now_s=10.0,
    )
    result = hold_filter.update(
        {},
        now_s=9.0,
    )

    assert result.lost_names == (RobotName.PHINEAS.value,)
    assert result.is_all_seen is False
    assert len(result.poses) == 1
    assert math.isclose(
        result.poses[0].confidence,
        phineas_pose.confidence,
    )


def test_should_report_all_seen_false_when_any_robot_is_lost() -> None:
    expected_names = (
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    )
    hold_filter = RobotHoldFilter(
        expected_names,
        hold_s=0.5,
    )
    phineas_pose = _make_pose(RobotName.PHINEAS.value)
    ferb_pose = _make_pose(RobotName.FERB.value)

    all_seen_result = hold_filter.update(
        {
            RobotName.PHINEAS.value: phineas_pose,
            RobotName.FERB.value: ferb_pose,
        },
        now_s=0.0,
    )
    held_result = hold_filter.update(
        {RobotName.PHINEAS.value: phineas_pose},
        now_s=0.25,
    )
    dropped_result = hold_filter.update(
        {RobotName.PHINEAS.value: phineas_pose},
        now_s=0.6,
    )

    assert all_seen_result.is_all_seen is True
    assert held_result.is_all_seen is False
    assert dropped_result.is_all_seen is False
