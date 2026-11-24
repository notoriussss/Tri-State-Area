"""Unit tests for pure manipulator relay routing rules."""

import pytest
from overhead_core.constants import (
    PERRY_HEARTBEAT_TIMEOUT_S,
    RelayDestination,
    RelayMessageKind,
    RobotName,
)
from overhead_core.relay import route_message

_KNOWN_ROBOTS: tuple[str, ...] = (
    RobotName.PHINEAS,
    RobotName.FERB,
)


@pytest.mark.parametrize(
    (
        "kind",
        "robot_name",
        "expected",
    ),
    [
        (
            RelayMessageKind.ZONE_OCCUPANT,
            None,
            (
                (
                    RelayDestination.US,
                    RelayDestination.ROBOT,
                ),
                _KNOWN_ROBOTS,
            ),
        ),
        (
            RelayMessageKind.PERRY_STATE,
            None,
            (
                (RelayDestination.US,),
                (),
            ),
        ),
        (
            RelayMessageKind.ZONE_REQUEST_RESPONSE,
            RobotName.PHINEAS,
            (
                (RelayDestination.ROBOT,),
                (RobotName.PHINEAS,),
            ),
        ),
        (
            RelayMessageKind.ZONE_RELEASE_RESPONSE,
            RobotName.FERB,
            (
                (RelayDestination.ROBOT,),
                (RobotName.FERB,),
            ),
        ),
        (
            RelayMessageKind.ROBOT_ZONE_REQUEST,
            RobotName.PHINEAS,
            (
                (RelayDestination.PERRY,),
                (RobotName.PHINEAS,),
            ),
        ),
        (
            RelayMessageKind.ROBOT_ZONE_RELEASE,
            RobotName.FERB,
            (
                (RelayDestination.PERRY,),
                (RobotName.FERB,),
            ),
        ),
    ],
)
def test_should_route_each_message_kind_according_to_relay_tables(
    kind: RelayMessageKind,
    robot_name: str | None,
    expected: tuple[tuple[RelayDestination, ...], tuple[str, ...]],
) -> None:
    decision = route_message(
        kind,
        known_robots=_KNOWN_ROBOTS,
        robot_name=robot_name,
        age_s=0.1,
    )

    assert decision.is_accepted is True
    assert decision.is_fresh is True
    expected_destinations, expected_targets = expected
    assert decision.destinations == expected_destinations
    assert decision.target_robots == expected_targets


def test_should_drop_stale_heartbeat_while_keeping_accepted_when_past_timeout() -> None:
    decision = route_message(
        RelayMessageKind.ZONE_OCCUPANT,
        known_robots=_KNOWN_ROBOTS,
        robot_name=None,
        age_s=PERRY_HEARTBEAT_TIMEOUT_S + 0.01,
    )

    assert decision.is_accepted is True
    assert decision.is_fresh is False
    assert decision.destinations == ()
    assert decision.target_robots == ()


def test_should_keep_heartbeat_fresh_when_age_equals_timeout() -> None:
    decision = route_message(
        RelayMessageKind.ZONE_OCCUPANT,
        known_robots=_KNOWN_ROBOTS,
        robot_name=None,
        age_s=PERRY_HEARTBEAT_TIMEOUT_S,
    )

    assert decision.is_accepted is True
    assert decision.is_fresh is True
    assert decision.destinations == (
        RelayDestination.US,
        RelayDestination.ROBOT,
    )
    assert decision.target_robots == _KNOWN_ROBOTS


@pytest.mark.parametrize(
    "kind",
    [
        RelayMessageKind.ZONE_REQUEST_RESPONSE,
        RelayMessageKind.ZONE_RELEASE_RESPONSE,
        RelayMessageKind.ROBOT_ZONE_REQUEST,
        RelayMessageKind.ROBOT_ZONE_RELEASE,
    ],
)
def test_should_reject_unknown_robot_for_robot_addressed_messages(
    kind: RelayMessageKind,
) -> None:
    decision = route_message(
        kind,
        known_robots=_KNOWN_ROBOTS,
        robot_name="candace",
        age_s=0.0,
    )

    assert decision.is_accepted is False
    assert decision.destinations == ()
    assert decision.target_robots == ()


@pytest.mark.parametrize(
    "kind",
    [
        RelayMessageKind.ZONE_REQUEST_RESPONSE,
        RelayMessageKind.ZONE_RELEASE_RESPONSE,
        RelayMessageKind.ROBOT_ZONE_REQUEST,
        RelayMessageKind.ROBOT_ZONE_RELEASE,
    ],
)
def test_should_reject_none_robot_name_for_robot_addressed_messages(
    kind: RelayMessageKind,
) -> None:
    decision = route_message(
        kind,
        known_robots=_KNOWN_ROBOTS,
        robot_name=None,
        age_s=0.0,
    )

    assert decision.is_accepted is False
    assert decision.destinations == ()
    assert decision.target_robots == ()
