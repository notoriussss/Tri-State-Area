"""Mecanum inverse kinematics (X-roller layout) in REP 103 coordinates."""

from fleet_core.constants import (
    FULL_WHEEL_RATIO,
    WHEEL_ORDER,
    WheelName,
)
from fleet_core.types import (
    MecanumGeometry,
    VelocityCmd,
)

ERR_NON_POSITIVE_RADIUS = "wheel_radius_m must be strictly positive"
ERR_NON_POSITIVE_MAX_SPEED = "max_wheel_rad_s must be strictly positive"
ERR_NON_POSITIVE_LIMIT = "max_abs must be strictly positive"


def max_wheel_rad_s_from_linear(
    max_wheel_speed_mps: float,
    wheel_radius_m: float,
) -> float:
    """Converts linear wheel rim speed to angular speed in rad/s.

    Args:
        max_wheel_speed_mps: Maximum linear rim speed in meters per second.
        wheel_radius_m: Wheel radius in meters.

    Returns:
        Maximum angular wheel speed in radians per second.
    """
    if wheel_radius_m <= 0.0:
        raise ValueError(ERR_NON_POSITIVE_RADIUS)

    return max_wheel_speed_mps / wheel_radius_m


def wheel_speeds(
    cmd: VelocityCmd,
    geometry: MecanumGeometry,
) -> dict[WheelName, float]:
    """Computes angular wheel speeds (rad/s) for a body-frame velocity command.

    Args:
        cmd: Desired body velocity (vx forward, vy left, wz CCW) in REP 103 units.
        geometry: Mecanum chassis dimensions in meters.

    Returns:
        Mapping from each canonical wheel name to its angular speed in rad/s.
    """
    if geometry.wheel_radius_m <= 0.0:
        raise ValueError(ERR_NON_POSITIVE_RADIUS)

    lever_arm_m = geometry.half_wheelbase_m + geometry.half_track_m
    radius_m = geometry.wheel_radius_m
    vx = cmd.vx_mps
    vy = cmd.vy_mps
    wz = cmd.wz_radps

    return {
        WheelName.FL: (vx - vy - lever_arm_m * wz) / radius_m,
        WheelName.FR: (vx + vy + lever_arm_m * wz) / radius_m,
        WheelName.RL: (vx + vy - lever_arm_m * wz) / radius_m,
        WheelName.RR: (vx - vy + lever_arm_m * wz) / radius_m,
    }


def scale_to_limit(
    speeds: dict[WheelName, float],
    max_abs: float,
) -> dict[WheelName, float]:
    """Scales all wheel speeds equally when any wheel exceeds max_abs.

    Args:
        speeds: Wheel speed or ratio mapping.
        max_abs: Maximum permitted absolute value per wheel.

    Returns:
        Scaled mapping preserving the motion direction.
    """
    if max_abs <= 0.0:
        raise ValueError(ERR_NON_POSITIVE_LIMIT)

    peak = max(abs(value) for value in speeds.values())
    if peak <= max_abs or peak == 0.0:
        return dict(speeds)

    factor = max_abs / peak
    return {
        wheel: speeds[wheel] * factor
        for wheel in WHEEL_ORDER
    }


def wheel_ratios(
    cmd: VelocityCmd,
    geometry: MecanumGeometry,
    max_wheel_rad_s: float,
) -> dict[WheelName, float]:
    """Computes normalized wheel ratios in [-1, 1] for a velocity command.

    When any wheel would saturate, all four wheels are scaled down together so
    the resulting holonomic motion direction is preserved.

    Args:
        cmd: Desired body velocity in REP 103 units.
        geometry: Mecanum chassis dimensions in meters.
        max_wheel_rad_s: Maximum angular wheel speed corresponding to ratio 1.0.

    Returns:
        Mapping from each wheel name to its normalized command in [-1.0, 1.0].
    """
    if max_wheel_rad_s <= 0.0:
        raise ValueError(ERR_NON_POSITIVE_MAX_SPEED)

    speeds = wheel_speeds(
        cmd,
        geometry,
    )
    ratios = {
        wheel: speeds[wheel] / max_wheel_rad_s
        for wheel in WHEEL_ORDER
    }
    return scale_to_limit(
        ratios,
        FULL_WHEEL_RATIO,
    )
