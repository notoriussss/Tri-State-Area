"""Unit tests for the pure overhead camera state machine."""

import pytest
from overhead_core.state_machine import (
    OverheadEvent,
    OverheadState,
    can_publish,
    next_state,
)


@pytest.mark.parametrize(
    ("from_state", "event", "expected_state"),
    [
        (OverheadState.STARTING, OverheadEvent.CAMERA_OPENED, OverheadState.CALIBRATING),
        (OverheadState.CALIBRATING, OverheadEvent.CORNERS_FOUND, OverheadState.RUNNING),
        (OverheadState.CALIBRATING, OverheadEvent.CORNERS_NOT_FOUND, OverheadState.CALIBRATING),
        (OverheadState.RUNNING, OverheadEvent.ROBOT_LOST, OverheadState.DEGRADED),
        (OverheadState.DEGRADED, OverheadEvent.ALL_ROBOTS_SEEN, OverheadState.RUNNING),
        (OverheadState.RUNNING, OverheadEvent.FRAME_TIMEOUT, OverheadState.NO_IMAGE),
        (OverheadState.DEGRADED, OverheadEvent.FRAME_TIMEOUT, OverheadState.NO_IMAGE),
        (OverheadState.NO_IMAGE, OverheadEvent.FRAME_BACK, OverheadState.RUNNING),
        (OverheadState.RUNNING, OverheadEvent.RECALIBRATE_REQUESTED, OverheadState.CALIBRATING),
        (OverheadState.DEGRADED, OverheadEvent.RECALIBRATE_REQUESTED, OverheadState.CALIBRATING),
    ],
)
def test_should_transition_to_expected_state_when_event_occurs(
    from_state: OverheadState,
    event: OverheadEvent,
    expected_state: OverheadState,
) -> None:
    actual_state = next_state(
        from_state,
        event,
    )

    assert actual_state == expected_state


@pytest.mark.parametrize(
    ("state", "event"),
    [
        (OverheadState.STARTING, OverheadEvent.CORNERS_FOUND),
        (OverheadState.STARTING, OverheadEvent.FRAME_TIMEOUT),
        (OverheadState.STARTING, OverheadEvent.ROBOT_LOST),
        (OverheadState.CALIBRATING, OverheadEvent.CAMERA_OPENED),
        (OverheadState.CALIBRATING, OverheadEvent.ROBOT_LOST),
        (OverheadState.CALIBRATING, OverheadEvent.FRAME_TIMEOUT),
        (OverheadState.CALIBRATING, OverheadEvent.RECALIBRATE_REQUESTED),
        (OverheadState.RUNNING, OverheadEvent.CAMERA_OPENED),
        (OverheadState.RUNNING, OverheadEvent.CORNERS_FOUND),
        (OverheadState.RUNNING, OverheadEvent.ALL_ROBOTS_SEEN),
        (OverheadState.RUNNING, OverheadEvent.FRAME_BACK),
        (OverheadState.DEGRADED, OverheadEvent.CAMERA_OPENED),
        (OverheadState.DEGRADED, OverheadEvent.CORNERS_FOUND),
        (OverheadState.DEGRADED, OverheadEvent.ROBOT_LOST),
        (OverheadState.DEGRADED, OverheadEvent.FRAME_BACK),
        (OverheadState.NO_IMAGE, OverheadEvent.CAMERA_OPENED),
        (OverheadState.NO_IMAGE, OverheadEvent.CORNERS_FOUND),
        (OverheadState.NO_IMAGE, OverheadEvent.ROBOT_LOST),
        (OverheadState.NO_IMAGE, OverheadEvent.RECALIBRATE_REQUESTED),
    ],
)
def test_should_keep_current_state_when_event_is_unlisted(
    state: OverheadState,
    event: OverheadEvent,
) -> None:
    actual_state = next_state(
        state,
        event,
    )

    assert actual_state == state


@pytest.mark.parametrize(
    ("state", "expected"),
    [
        (OverheadState.STARTING, False),
        (OverheadState.CALIBRATING, False),
        (OverheadState.NO_IMAGE, False),
        (OverheadState.RUNNING, True),
        (OverheadState.DEGRADED, True),
    ],
)
def test_should_allow_publishing_only_when_running_or_degraded(
    state: OverheadState,
    expected: bool,
) -> None:
    actual = can_publish(state)

    assert actual is expected
