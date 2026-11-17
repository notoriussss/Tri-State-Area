"""Virtual sensor computations based on arena world model."""

import math
from typing import Any

from overhead_core.constants import (
    DEFAULT_ZONE_RECT,
    MIN_ROBOTS_FOR_DISTANCE,
    NEAR_RED_DISTANCE_M,
    CubeColor,
    VirtualSensorEvent,
)


def _compute_robot_distance(
    robots: dict[str, tuple[float, float]],
) -> float:
    """Computes Euclidean distance between the first two robots or returns -1.0."""
    if len(robots) >= MIN_ROBOTS_FOR_DISTANCE:
        names = sorted(robots.keys())
        p0 = robots[names[0]]
        p1 = robots[names[1]]
        return float(
            math.hypot(
                p0[0] - p1[0],
                p0[1] - p1[1],
            )
        )
    return -1.0


def _compute_zone_containment(
    robots: dict[str, tuple[float, float]],
    zone_rect: tuple[float, float, float, float],
) -> tuple[set[str], list[str]]:
    """Identifies which robots are located inside the delivery zone rectangle."""
    z_xmin, z_ymin, z_xmax, z_ymax = zone_rect
    current_in_zone: set[str] = set()
    robots_in_zone: list[str] = []

    for name, (rx, ry) in robots.items():
        is_inside_x = z_xmin <= rx <= z_xmax
        is_inside_y = z_ymin <= ry <= z_ymax
        if is_inside_x and is_inside_y:
            current_in_zone.add(name)
            robots_in_zone.append(name)

    robots_in_zone.sort()
    return current_in_zone, robots_in_zone


def _find_nearest_red(
    rx: float,
    ry: float,
    red_cubes: list[tuple[int, str, float, float]],
) -> tuple[int, float]:
    """Finds the nearest red cube ID and distance in meters to a given robot position."""
    if not red_cubes:
        return 0, -1.0

    nearest_cube = min(
        red_cubes,
        key=lambda c: math.hypot(
            c[2] - rx,
            c[3] - ry,
        ),
    )
    nearest_red_id = int(nearest_cube[0])
    nearest_red_dist = float(
        math.hypot(
            nearest_cube[2] - rx,
            nearest_cube[3] - ry,
        )
    )
    return nearest_red_id, nearest_red_dist


def _build_robot_events(
    name: str,
    *,
    is_inside: bool,
    was_inside: bool,
    nearest_red_id: int,
    nearest_red_dist: float,
    near_red_m: float,
) -> list[str]:
    """Builds zone entry/exit and near-red proximity event strings."""
    events: list[str] = []
    if is_inside and not was_inside:
        events.append(VirtualSensorEvent.ENTERED_ZONE.value)
    elif was_inside and not is_inside:
        events.append(VirtualSensorEvent.LEFT_ZONE.value)

    is_near_red = nearest_red_id != 0 and nearest_red_dist < near_red_m
    if is_near_red:
        events.append(f"{VirtualSensorEvent.NEAR_RED_PREFIX.value}:{nearest_red_id}")

    return events


def _compute_per_robot(
    robots: dict[str, tuple[float, float]],
    red_cubes: list[tuple[int, str, float, float]],
    *,
    current_in_zone: set[str],
    prev_zone_set: set[str],
    near_red_m: float,
) -> dict[str, tuple[int, float, list[str]]]:
    """Computes nearest red cube and event logs for each observed robot."""
    per_robot: dict[str, tuple[int, float, list[str]]] = {}
    for name, (rx, ry) in robots.items():
        nearest_red_id, nearest_red_dist = _find_nearest_red(
            rx,
            ry,
            red_cubes,
        )
        events = _build_robot_events(
            name,
            is_inside=name in current_in_zone,
            was_inside=name in prev_zone_set,
            nearest_red_id=nearest_red_id,
            nearest_red_dist=nearest_red_dist,
            near_red_m=near_red_m,
        )
        per_robot[name] = (
            nearest_red_id,
            nearest_red_dist,
            events,
        )
    return per_robot


def compute(
    robots: dict[str, tuple[float, float]],
    cubes: list[tuple[int, str, float, float]],
    *,
    zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT,
    prev_in_zone: set[str] | None = None,
    near_red_m: float = NEAR_RED_DISTANCE_M,
) -> dict[str, Any]:
    """Computes virtual sensor values including proximity, zones, and events.

    Args:
        robots: Mapping from robot name to its world floor position (x_m, y_m).
        cubes: List of tracked cubes as (track_id, color, x_m, y_m).
        zone_rect: Delivery zone bounding box as (xmin_m, ymin_m, xmax_m, ymax_m).
        prev_in_zone: Set of robot names present in the zone during the previous cycle.
        near_red_m: Threshold distance in meters for near:red events.

    Returns:
        Dictionary containing robot_distance, robots_in_zone, per_robot measurements,
        and current in_zone set.
    """
    prev_zone_set = prev_in_zone if prev_in_zone is not None else set()
    robot_distance = _compute_robot_distance(robots)
    current_in_zone, robots_in_zone = _compute_zone_containment(
        robots,
        zone_rect,
    )

    red_cubes = [c for c in cubes if c[1] == CubeColor.RED.value]
    per_robot = _compute_per_robot(
        robots,
        red_cubes,
        current_in_zone=current_in_zone,
        prev_zone_set=prev_zone_set,
        near_red_m=near_red_m,
    )

    return {
        "robot_distance": robot_distance,
        "robots_in_zone": robots_in_zone,
        "per_robot": per_robot,
        "in_zone": current_in_zone,
    }
