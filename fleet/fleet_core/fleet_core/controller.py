"""Holonomic go-to-pose proportional controller with deadband and acceleration ramp."""

import math
from dataclasses import dataclass

from fleet_core.constants import (
    DEFAULT_ANGLE_DEADBAND_RAD,
    DEFAULT_ANGULAR_ACCEL_RADPS2,
    DEFAULT_KP_ANGULAR,
    DEFAULT_KP_LINEAR,
    DEFAULT_LINEAR_ACCEL_MPS2,
    DEFAULT_MAX_ANGULAR_RADPS,
    DEFAULT_MAX_LINEAR_MPS,
    DEFAULT_POS_DEADBAND_M,
)
from fleet_core.math_utils import (
    clamp,
    wrap_angle,
)
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    Pose2D,
    VelocityCmd,
)


@dataclass(frozen=True)
class ControllerConfig:
    """Gains, velocity caps, deadbands, and acceleration limits for pose control."""

    kp_linear: float = DEFAULT_KP_LINEAR
    kp_angular: float = DEFAULT_KP_ANGULAR
    max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS
    max_angular_radps: float = DEFAULT_MAX_ANGULAR_RADPS
    pos_deadband_m: float = DEFAULT_POS_DEADBAND_M
    angle_deadband_rad: float = DEFAULT_ANGLE_DEADBAND_RAD
    max_linear_accel_mps2: float = DEFAULT_LINEAR_ACCEL_MPS2
    max_angular_accel_radps2: float = DEFAULT_ANGULAR_ACCEL_RADPS2


def is_at_pose(
    current: Pose2D,
    target: Pose2D,
    config: ControllerConfig | None = None,
) -> bool:
    """Returns True when both position and heading errors fall inside deadbands.

    Args:
        current: Current robot pose in arena frame (m, m, rad).
        target: Desired robot pose in arena frame (m, m, rad).
        config: Controller thresholds (defaults to ControllerConfig).

    Returns:
        True when the robot has arrived at the target pose.
    """
    cfg = config or ControllerConfig()
    pos_err_m = math.hypot(
        target.x_m - current.x_m,
        target.y_m - current.y_m,
    )
    angle_err_rad = abs(wrap_angle(target.theta_rad - current.theta_rad))

    return pos_err_m <= cfg.pos_deadband_m and angle_err_rad <= cfg.angle_deadband_rad


def compute_go_to_pose(
    current: Pose2D,
    target: Pose2D,
    config: ControllerConfig | None = None,
) -> VelocityCmd:
    """Computes saturated body-frame velocity (vx, vy, wz) to reach target pose.

    Implements:
        e = p_target - p_robot (arena frame)
        [vx, vy] = sat(K * R(-theta) * e) (body frame, preserving direction)
        wz = sat(K_theta * wrap_angle(theta_target - theta))

    Args:
        current: Current robot pose in arena frame (m, m, rad).
        target: Desired robot pose in arena frame (m, m, rad).
        config: Gains, saturation limits, and arrival deadbands.

    Returns:
        Body-frame VelocityCmd in REP 103 units.
    """
    cfg = config or ControllerConfig()
    dx_m = target.x_m - current.x_m
    dy_m = target.y_m - current.y_m
    dist_m = math.hypot(
        dx_m,
        dy_m,
    )

    vx_mps = 0.0
    vy_mps = 0.0
    if dist_m > cfg.pos_deadband_m:
        cos_theta = math.cos(current.theta_rad)
        sin_theta = math.sin(current.theta_rad)
        ex_body = cos_theta * dx_m + sin_theta * dy_m
        ey_body = -sin_theta * dx_m + cos_theta * dy_m
        vx_mps, vy_mps = _saturate_linear(
            cfg.kp_linear * ex_body,
            cfg.kp_linear * ey_body,
            cfg.max_linear_mps,
        )

    angle_err_rad = wrap_angle(target.theta_rad - current.theta_rad)
    wz_radps = 0.0
    if abs(angle_err_rad) > cfg.angle_deadband_rad:
        wz_radps = clamp(
            cfg.kp_angular * angle_err_rad,
            -cfg.max_angular_radps,
            cfg.max_angular_radps,
        )

    return VelocityCmd(
        vx_mps=vx_mps,
        vy_mps=vy_mps,
        wz_radps=wz_radps,
    )


def ramp_velocity(
    previous: VelocityCmd,
    target: VelocityCmd,
    dt_s: float,
    *,
    config: ControllerConfig | None = None,
) -> VelocityCmd:
    """Limits per-cycle velocity deltas to smooth acceleration on mecanum wheels.

    Args:
        previous: Velocity command applied on the previous cycle.
        target: Unramped desired velocity command.
        dt_s: Elapsed cycle time in seconds.
        config: Acceleration limits.

    Returns:
        Ramped VelocityCmd bounded by max_linear_accel_mps2 and max_angular_accel_radps2.
    """
    if dt_s <= 0.0:
        return previous

    cfg = config or ControllerConfig()
    max_dv = cfg.max_linear_accel_mps2 * dt_s
    max_dw = cfg.max_angular_accel_radps2 * dt_s

    dvx = target.vx_mps - previous.vx_mps
    dvy = target.vy_mps - previous.vy_mps
    dv_norm = math.hypot(
        dvx,
        dvy,
    )
    if dv_norm > max_dv and dv_norm > 0.0:
        scale = max_dv / dv_norm
        dvx *= scale
        dvy *= scale

    dwz = clamp(
        target.wz_radps - previous.wz_radps,
        -max_dw,
        max_dw,
    )

    return VelocityCmd(
        vx_mps=previous.vx_mps + dvx,
        vy_mps=previous.vy_mps + dvy,
        wz_radps=previous.wz_radps + dwz,
    )


class PoseController:
    """Stateful go-to-pose controller combining proportional feedback with velocity ramping."""

    def __init__(self, config: ControllerConfig | None = None) -> None:
        """Initializes the controller with zero previous velocity."""
        self._config = config or ControllerConfig()
        self._last_cmd: VelocityCmd = ZERO_VELOCITY_CMD

    def reset(self) -> None:
        """Resets internal ramp state to zero velocity (call after stops)."""
        self._last_cmd = ZERO_VELOCITY_CMD

    def step(
        self,
        current: Pose2D,
        target: Pose2D,
        dt_s: float,
    ) -> VelocityCmd:
        """Computes the ramped body velocity for one control tick.

        Args:
            current: Current robot pose in arena frame.
            target: Desired robot pose in arena frame.
            dt_s: Elapsed time since previous tick in seconds.

        Returns:
            Ramped VelocityCmd for the HAL.
        """
        raw_cmd = compute_go_to_pose(
            current,
            target,
            self._config,
        )
        ramped = ramp_velocity(
            self._last_cmd,
            raw_cmd,
            dt_s,
            config=self._config,
        )
        self._last_cmd = ramped
        return ramped


def _saturate_linear(
    vx_mps: float,
    vy_mps: float,
    max_linear_mps: float,
) -> tuple[float, float]:
    """Scales (vx, vy) jointly so speed never exceeds max_linear_mps."""
    speed_mps = math.hypot(
        vx_mps,
        vy_mps,
    )
    if speed_mps <= max_linear_mps or speed_mps == 0.0:
        return vx_mps, vy_mps

    scale = max_linear_mps / speed_mps
    return vx_mps * scale, vy_mps * scale
