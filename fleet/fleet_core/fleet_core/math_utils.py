"""Pure math helpers shared across fleet_core modules."""

import math

from fleet_core.constants import FULL_TURN_RAD


def clamp(
    value: float,
    low: float,
    high: float,
) -> float:
    """Limits a scalar value to the closed interval [low, high].

    Args:
        value: Input scalar to clamp.
        low: Lower inclusive bound.
        high: Upper inclusive bound.

    Returns:
        Clamped scalar within [low, high].
    """
    return max(
        low,
        min(
            high,
            value,
        ),
    )


def wrap_angle(angle_rad: float) -> float:
    """Wraps an angle to the REP 103 interval (-pi, pi] in radians.

    Args:
        angle_rad: Input angle in radians.

    Returns:
        Equivalent angle in (-pi, pi] radians.
    """
    wrapped = (angle_rad + math.pi) % FULL_TURN_RAD - math.pi

    if wrapped <= -math.pi:
        return math.pi

    return wrapped
