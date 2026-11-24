"""Pure routing rules for the manipulator relay between robots and Perry."""

from dataclasses import dataclass

from overhead_core.constants import (
    PERRY_HEARTBEAT_TIMEOUT_S,
    RelayDestination,
    RelayMessageKind,
)
from overhead_core.types import RelayDecision


@dataclass(frozen=True)
class _RouteSpec:
    """Static routing specification for one relay message kind."""

    destinations: tuple[RelayDestination, ...]
    requires_robot: bool
    broadcast_known_robots: bool
    timeout_s: float | None


_ROUTING_TABLE: dict[RelayMessageKind, _RouteSpec] = {
    RelayMessageKind.ZONE_OCCUPANT: _RouteSpec(
        destinations=(
            RelayDestination.US,
            RelayDestination.ROBOT,
        ),
        requires_robot=False,
        broadcast_known_robots=True,
        timeout_s=PERRY_HEARTBEAT_TIMEOUT_S,
    ),
    RelayMessageKind.PERRY_STATE: _RouteSpec(
        destinations=(RelayDestination.US,),
        requires_robot=False,
        broadcast_known_robots=False,
        timeout_s=None,
    ),
    RelayMessageKind.ZONE_REQUEST_RESPONSE: _RouteSpec(
        destinations=(RelayDestination.ROBOT,),
        requires_robot=True,
        broadcast_known_robots=False,
        timeout_s=None,
    ),
    RelayMessageKind.ZONE_RELEASE_RESPONSE: _RouteSpec(
        destinations=(RelayDestination.ROBOT,),
        requires_robot=True,
        broadcast_known_robots=False,
        timeout_s=None,
    ),
    RelayMessageKind.ROBOT_ZONE_REQUEST: _RouteSpec(
        destinations=(RelayDestination.PERRY,),
        requires_robot=True,
        broadcast_known_robots=False,
        timeout_s=None,
    ),
    RelayMessageKind.ROBOT_ZONE_RELEASE: _RouteSpec(
        destinations=(RelayDestination.PERRY,),
        requires_robot=True,
        broadcast_known_robots=False,
        timeout_s=None,
    ),
}


def _resolve_target_robots(
    spec: _RouteSpec,
    known_robots: tuple[str, ...],
    robot_name: str | None,
) -> tuple[str, ...]:
    """Resolves target robot names for an accepted and fresh relay message."""
    if spec.broadcast_known_robots:
        return tuple(known_robots)
    if spec.requires_robot and robot_name is not None:
        return (robot_name,)
    return ()


def route_message(
    kind: RelayMessageKind,
    *,
    known_robots: tuple[str, ...],
    robot_name: str | None,
    age_s: float,
) -> RelayDecision:
    """Computes relay routing destinations and freshness for a manipulator message.

    Age is measured on the overhead node's own clock since message reception.
    Stale heartbeat data is accepted as recognized but never forwarded. Messages
    addressed from or to an unknown or missing robot name are rejected.

    Args:
        kind: Category of the relay message from Perry or from a robot.
        known_robots: Tuple of valid robot names in the fleet.
        robot_name: Robot associated with the request or response, or None.
        age_s: Elapsed time in seconds on our local clock since reception.

    Returns:
        RelayDecision describing acceptance, freshness, destinations, and target robots.
    """
    spec = _ROUTING_TABLE[kind]
    is_fresh = spec.timeout_s is None or age_s <= spec.timeout_s

    if spec.requires_robot:
        is_known_robot = robot_name is not None and robot_name in known_robots
        if not is_known_robot:
            return RelayDecision(
                is_accepted=False,
                is_fresh=is_fresh,
                destinations=(),
                target_robots=(),
            )

    if not is_fresh:
        return RelayDecision(
            is_accepted=True,
            is_fresh=False,
            destinations=(),
            target_robots=(),
        )

    target_robots = _resolve_target_robots(
        spec,
        known_robots,
        robot_name,
    )
    return RelayDecision(
        is_accepted=True,
        is_fresh=True,
        destinations=spec.destinations,
        target_robots=target_robots,
    )
