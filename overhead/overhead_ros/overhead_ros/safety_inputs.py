"""Thin ROS adapter tracking Perry zone occupant heartbeat and gate status."""

import threading
import time

from multirobot_interfaces.msg import RobotState
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    DEFAULT_ZONE_RECT,
    NO_ZONE_OCCUPANT,
    PERRY_HEARTBEAT_TIMEOUT_S,
)
from overhead_core.world_safety import SafetyConfig
from std_msgs.msg import String


class SafetyInputs:
    """Caches Perry's /zone/occupant heartbeat and /perry/state gate status.

    Measures heartbeat age exclusively on the local node monotonic clock from
    message reception time and builds fail-safe SafetyConfig snapshots.
    """

    def __init__(
        self,
        *,
        arena_width_m: float = ARENA_WIDTH_M,
        arena_height_m: float = ARENA_HEIGHT_M,
        zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT,
        heartbeat_timeout_s: float = PERRY_HEARTBEAT_TIMEOUT_S,
    ) -> None:
        """Initializes arena geometry and empty Perry state.

        Args:
            arena_width_m: Arena width along +x in meters.
            arena_height_m: Arena height along +y in meters.
            zone_rect: Delivery zone bounds (xmin, ymin, xmax, ymax) in meters.
            heartbeat_timeout_s: Maximum allowed age in seconds for /zone/occupant.
        """
        self._arena_width_m = float(arena_width_m)
        self._arena_height_m = float(arena_height_m)
        self._zone_rect = zone_rect
        self._heartbeat_timeout_s = float(heartbeat_timeout_s)
        self._lock = threading.Lock()
        self._zone_occupant: str = NO_ZONE_OCCUPANT
        self._last_occupant_rx_s: float | None = None
        self._gate_closed: bool = False

    def on_zone_occupant(self, msg: String) -> None:
        """Records the latest /zone/occupant string and local reception time."""
        rx_time_s = time.monotonic()
        with self._lock:
            self._zone_occupant = str(msg.data)
            self._last_occupant_rx_s = rx_time_s

    def on_perry_state(self, msg: RobotState) -> None:
        """Records the latest gate_closed flag from /perry/state."""
        with self._lock:
            self._gate_closed = bool(msg.gate_closed)

    def is_heartbeat_fresh(self, now_s: float | None = None) -> bool:
        """Returns whether the latest /zone/occupant heartbeat is within timeout.

        Args:
            now_s: Local monotonic clock timestamp in seconds, or None for current time.

        Returns:
            True if a heartbeat was received and its age is at most heartbeat_timeout_s.
        """
        current_s = now_s if now_s is not None else time.monotonic()
        with self._lock:
            last_rx_s = self._last_occupant_rx_s
        if last_rx_s is None:
            return False
        age_s = current_s - last_rx_s
        return age_s <= self._heartbeat_timeout_s

    def build_config(
        self,
        now_s: float | None = None,
        *,
        carrying_robots: frozenset[str] = frozenset(),
    ) -> SafetyConfig:
        """Builds a SafetyConfig applying the Perry heartbeat and gate safety rule.

        If the /zone/occupant heartbeat is not fresh or gate_closed is True, the zone
        becomes an obstacle for every robot (is_gate_closed=True, zone_occupant=None).

        Args:
            now_s: Local monotonic clock timestamp in seconds, or None for current time.
            carrying_robots: Set of robot names currently carrying a cube.

        Returns:
            SafetyConfig snapshot for compute_safety.
        """
        current_s = now_s if now_s is not None else time.monotonic()
        with self._lock:
            last_rx_s = self._last_occupant_rx_s
            zone_occupant = self._zone_occupant
            gate_closed = self._gate_closed

        is_fresh = last_rx_s is not None and (current_s - last_rx_s) <= self._heartbeat_timeout_s
        if not is_fresh or gate_closed:
            return SafetyConfig(
                arena_width_m=self._arena_width_m,
                arena_height_m=self._arena_height_m,
                zone_rect=self._zone_rect,
                zone_occupant=None,
                is_gate_closed=True,
                carrying_robots=carrying_robots,
            )

        occupant = zone_occupant if zone_occupant != NO_ZONE_OCCUPANT else None
        return SafetyConfig(
            arena_width_m=self._arena_width_m,
            arena_height_m=self._arena_height_m,
            zone_rect=self._zone_rect,
            zone_occupant=occupant,
            is_gate_closed=False,
            carrying_robots=carrying_robots,
        )
