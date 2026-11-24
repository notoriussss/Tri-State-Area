"""Perry delivery-zone heartbeat, lease protocol, and free delivery slot selection."""

import math
from dataclasses import dataclass

from fleet_core.constants import (
    CUBE_SIZE_M,
    DEFAULT_DELIVER_SLOT_Y_M,
    SLOT_MIN_SEPARATION_M,
    ZONE_CALL_TIMEOUT_S,
    ZONE_GREEN_LINE_X_M,
    ZONE_HEARTBEAT_TIMEOUT_S,
    ZONE_LEASE_RENEW_S,
    ZONE_MAX_X_M,
    ZONE_MAX_Y_M,
    ZONE_MIN_Y_M,
    ZONE_SLOT_X_CANDIDATES_M,
    ZONE_SLOT_Y_CANDIDATES_M,
    CubeColor,
)
from fleet_core.navigation import Point2D
from fleet_core.types import CubeObservation


@dataclass(frozen=True)
class DeliveryZoneConfig:
    """Physical bounds and cube-spacing rules for Perry's delivery notch."""

    green_line_x_m: float = ZONE_GREEN_LINE_X_M
    max_x_m: float = ZONE_MAX_X_M
    min_y_m: float = ZONE_MIN_Y_M
    max_y_m: float = ZONE_MAX_Y_M
    cube_size_m: float = CUBE_SIZE_M
    min_edge_separation_m: float = SLOT_MIN_SEPARATION_M
    slot_x_candidates_m: tuple[float, ...] = ZONE_SLOT_X_CANDIDATES_M
    slot_y_candidates_m: tuple[float, ...] = ZONE_SLOT_Y_CANDIDATES_M


def is_call_expired(
    sent_at_s: float | None,
    now_s: float,
    *,
    timeout_s: float = ZONE_CALL_TIMEOUT_S,
) -> bool:
    """Returns True when an in-flight zone service call exceeded timeout_s."""
    return sent_at_s is not None and (now_s - sent_at_s) >= timeout_s


def select_free_delivery_slot(
    cubes: tuple[CubeObservation, ...],
    *,
    config: DeliveryZoneConfig | None = None,
) -> Point2D | None:
    """Selects a free slot past the green line with >= 1.5 cm edge clearance.

    Prefers slots closer to the green line and centered in the delivery notch,
    never exceeding config.max_x_m (virtual stop where no black border exists).

    Args:
        cubes: All visible cubes in the arena (only cubes inside the zone matter).
        config: Delivery notch geometry and spacing thresholds.

    Returns:
        Free slot coordinate (x_m, y_m) in arena frame, or None if the zone is full.
    """
    cfg = config or DeliveryZoneConfig()
    zone_cubes = tuple(
        cube
        for cube in cubes
        if cube.x_m >= cfg.green_line_x_m and cfg.min_y_m <= cube.y_m <= cfg.max_y_m
    )
    min_center_dist_m = cfg.cube_size_m + cfg.min_edge_separation_m

    candidates = [
        (slot_x, slot_y)
        for slot_x in cfg.slot_x_candidates_m
        for slot_y in cfg.slot_y_candidates_m
        if cfg.green_line_x_m < slot_x <= cfg.max_x_m
        and cfg.min_y_m <= slot_y <= cfg.max_y_m
    ]
    candidates.sort(
        key=lambda pt: (
            pt[0] - cfg.green_line_x_m,
            abs(pt[1] - DEFAULT_DELIVER_SLOT_Y_M),
        )
    )

    for slot_x, slot_y in candidates:
        is_clear = all(
            math.hypot(slot_x - cube.x_m, slot_y - cube.y_m) >= min_center_dist_m
            for cube in zone_cubes
        )
        if is_clear:
            return (
                slot_x,
                slot_y,
            )

    return None


class ZoneHeartbeat:
    """Tracks liveness of Perry's /zone/occupant heartbeat on the local clock."""

    def __init__(self, timeout_s: float = ZONE_HEARTBEAT_TIMEOUT_S) -> None:
        """Initializes with no heartbeat received yet."""
        self.timeout_s = timeout_s
        self._last_beat_s: float | None = None

    def beat(self, now_s: float) -> None:
        """Records arrival of a /zone/occupant heartbeat at local time now_s."""
        self._last_beat_s = now_s

    def is_alive(self, now_s: float) -> bool:
        """Returns True when a heartbeat arrived within timeout_s."""
        if self._last_beat_s is None:
            return False

        elapsed_s = now_s - self._last_beat_s
        return 0.0 <= elapsed_s <= self.timeout_s


class ZoneClient:
    """Client-side state machine for requesting, renewing, and releasing Perry's zone."""

    def __init__(
        self,
        robot_name: str,
        *,
        renew_period_s: float = ZONE_LEASE_RENEW_S,
    ) -> None:
        """Starts neither wanting nor holding the delivery zone."""
        self.robot_name = robot_name
        self.renew_period_s = renew_period_s
        self.want: bool = False
        self.occupant: str = ""
        self.gate_closed: bool = False
        self.delivered: bool = False
        self.delivered_color: str = ""
        self._was_wanting: bool = False
        self._pending_release: bool = False
        self._last_request_s: float | None = None

    @property
    def holds_zone(self) -> bool:
        """Returns True when Perry reports this robot as the active zone occupant."""
        return self.occupant == self.robot_name

    def is_ready_to_enter(
        self,
        heartbeat: ZoneHeartbeat,
        now_s: float,
    ) -> bool:
        """Returns True when Perry's heartbeat is fresh and the gate allows entry."""
        if not heartbeat.is_alive(now_s):
            return False

        return not self.gate_closed or self.holds_zone

    def is_granted(
        self,
        heartbeat: ZoneHeartbeat,
        now_s: float,
    ) -> bool:
        """Returns True only when holding the lease AND Perry's heartbeat is fresh."""
        return self.holds_zone and heartbeat.is_alive(now_s)

    def update(
        self,
        *,
        want: bool | None = None,
        occupant: str | None = None,
        gate_closed: bool | None = None,
    ) -> None:
        """Updates client state from FSM wishes, /zone/occupant, or /perry/state."""
        if occupant is not None:
            self.occupant = occupant
        if gate_closed is not None:
            self.gate_closed = gate_closed
        if want is not None:
            if self._was_wanting and not want and self.holds_zone:
                self._pending_release = True
            self.want = want
            self._was_wanting = want

    def mark_delivered(self, color: CubeColor | str) -> None:
        """Records that a cube of the given color was delivered into the zone."""
        self.delivered = True
        self.delivered_color = str(color)

    def request_release(self, delivered_color: CubeColor | None = None) -> None:
        """Marks the held zone lease for explicit release after leaving the zone."""
        if delivered_color is not None:
            self.mark_delivered(delivered_color)
        self.want = False
        self._was_wanting = False
        self._pending_release = True

    def should_request(self, now_s: float) -> bool:
        """Returns True when /zone/request must be sent to acquire or renew the lease."""
        if not self.want:
            return False
        if self.gate_closed and not self.holds_zone:
            return False
        if not self.holds_zone or self._last_request_s is None:
            return True

        return (now_s - self._last_request_s) >= self.renew_period_s

    def note_requested(self, now_s: float) -> None:
        """Records the local time a /zone/request call was dispatched."""
        self._last_request_s = now_s

    def should_release(self) -> bool:
        """Returns True when /zone/release must be sent to Perry's arbiter."""
        return self._pending_release or (not self.want and self.holds_zone)

    def pop_release(self) -> tuple[bool, str]:
        """Consumes pending release metadata and returns (delivered, color)."""
        self._pending_release = False
        self._last_request_s = None
        delivered = self.delivered
        color = self.delivered_color
        self.delivered = False
        self.delivered_color = ""
        return delivered, color

    def restore_release(
        self,
        *,
        delivered: bool,
        color: str,
    ) -> None:
        """Re-queues a release after an unanswered /zone/release timeout."""
        self._pending_release = True
        if delivered:
            self.mark_delivered(color)
