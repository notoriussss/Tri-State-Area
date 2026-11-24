"""Pure state machine for the overhead camera node."""

from enum import StrEnum


class OverheadState(StrEnum):
    """Operating states of the overhead camera node."""

    STARTING = "STARTING"
    CALIBRATING = "CALIBRATING"
    RUNNING = "RUNNING"
    DEGRADED = "DEGRADED"
    NO_IMAGE = "NO_IMAGE"


class OverheadEvent(StrEnum):
    """Events driving transitions of the overhead camera state machine."""

    CAMERA_OPENED = "CAMERA_OPENED"
    CORNERS_FOUND = "CORNERS_FOUND"
    CORNERS_NOT_FOUND = "CORNERS_NOT_FOUND"
    FRAME_TIMEOUT = "FRAME_TIMEOUT"
    FRAME_BACK = "FRAME_BACK"
    ROBOT_LOST = "ROBOT_LOST"
    ALL_ROBOTS_SEEN = "ALL_ROBOTS_SEEN"
    RECALIBRATE_REQUESTED = "RECALIBRATE_REQUESTED"


_TRANSITIONS: dict[tuple[OverheadState, OverheadEvent], OverheadState] = {
    (OverheadState.STARTING, OverheadEvent.CAMERA_OPENED): OverheadState.CALIBRATING,
    (OverheadState.CALIBRATING, OverheadEvent.CORNERS_FOUND): OverheadState.RUNNING,
    (OverheadState.CALIBRATING, OverheadEvent.CORNERS_NOT_FOUND): OverheadState.CALIBRATING,
    (OverheadState.RUNNING, OverheadEvent.ROBOT_LOST): OverheadState.DEGRADED,
    (OverheadState.DEGRADED, OverheadEvent.ALL_ROBOTS_SEEN): OverheadState.RUNNING,
    (OverheadState.RUNNING, OverheadEvent.FRAME_TIMEOUT): OverheadState.NO_IMAGE,
    (OverheadState.DEGRADED, OverheadEvent.FRAME_TIMEOUT): OverheadState.NO_IMAGE,
    (OverheadState.NO_IMAGE, OverheadEvent.FRAME_BACK): OverheadState.RUNNING,
    (OverheadState.RUNNING, OverheadEvent.RECALIBRATE_REQUESTED): OverheadState.CALIBRATING,
    (OverheadState.DEGRADED, OverheadEvent.RECALIBRATE_REQUESTED): OverheadState.CALIBRATING,
}

_PUBLISHABLE_STATES: frozenset[OverheadState] = frozenset(
    {
        OverheadState.RUNNING,
        OverheadState.DEGRADED,
    }
)


def next_state(
    state: OverheadState,
    event: OverheadEvent,
) -> OverheadState:
    """Computes the next overhead camera state for a given state and event.

    Args:
        state: Current state of the overhead camera node.
        event: Observed event triggering a potential transition.

    Returns:
        Resulting OverheadState from the transition table, or the current state
        when the (state, event) pair has no defined transition.
    """
    return _TRANSITIONS.get(
        (state, event),
        state,
    )


def can_publish(state: OverheadState) -> bool:
    """Returns whether world state and virtual sensors may be published in this state.

    Args:
        state: Current state of the overhead camera node.

    Returns:
        True only when state is RUNNING or DEGRADED.
    """
    return state in _PUBLISHABLE_STATES
