"""Pure manual teleoperation state with safety timeout for the web controller."""

from dataclasses import dataclass

from fleet_core.constants import (
    DEFAULT_MAX_ANGULAR_RADPS,
    DEFAULT_MAX_LINEAR_MPS,
    HAL_WATCHDOG_TIMEOUT_S,
)
from fleet_core.math_utils import clamp
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    VelocityCmd,
)

DEFAULT_MANUAL_TIMEOUT_S: float = HAL_WATCHDOG_TIMEOUT_S


@dataclass(frozen=True)
class ManualControlConfig:
    """Velocity caps and command timeout for browser manual teleoperation."""

    max_linear_mps: float = DEFAULT_MAX_LINEAR_MPS
    max_angular_radps: float = DEFAULT_MAX_ANGULAR_RADPS
    cmd_timeout_s: float = DEFAULT_MANUAL_TIMEOUT_S


class ManualControlState:
    """Tracks AUTO/MANUAL mode and expires stale manual commands safely."""

    def __init__(
        self,
        config: ManualControlConfig | None = None,
        *,
        auto_enabled: bool = False,
    ) -> None:
        """Initializes in safe stopped state with the requested mode."""
        self._config = config or ManualControlConfig()
        self._auto_enabled: bool = auto_enabled
        self._cmd: VelocityCmd = ZERO_VELOCITY_CMD
        self._cmd_time_s: float | None = None

    @property
    def auto_enabled(self) -> bool:
        """Returns True when autonomous 6-pillar navigation is enabled."""
        return self._auto_enabled

    def set_auto(self, enabled: bool) -> None:
        """Switches between AUTO and MANUAL modes, zeroing manual commands."""
        self._auto_enabled = bool(enabled)
        self._cmd = ZERO_VELOCITY_CMD
        self._cmd_time_s = None

    def set_normalized_cmd(
        self,
        vx_ratio: float,
        vy_ratio: float,
        wz_ratio: float,
        *,
        now_s: float,
    ) -> VelocityCmd:
        """Stores a normalized [-1, 1] teleop command scaled to physical limits.

        Args:
            vx_ratio: Forward/backward ratio in [-1.0, 1.0].
            vy_ratio: Left/right strafe ratio in [-1.0, 1.0].
            wz_ratio: CCW/CW rotation ratio in [-1.0, 1.0].
            now_s: Local monotonic timestamp in seconds.

        Returns:
            Scaled physical VelocityCmd in REP 103 units.
        """
        vx = clamp(vx_ratio, -1.0, 1.0) * self._config.max_linear_mps
        vy = clamp(vy_ratio, -1.0, 1.0) * self._config.max_linear_mps
        wz = clamp(wz_ratio, -1.0, 1.0) * self._config.max_angular_radps
        self._cmd = VelocityCmd(
            vx_mps=vx,
            vy_mps=vy,
            wz_radps=wz,
        )
        self._cmd_time_s = now_s
        return self._cmd

    def stop(self) -> None:
        """Immediately zeroes the manual velocity command."""
        self._cmd = ZERO_VELOCITY_CMD
        self._cmd_time_s = None

    def current_cmd(self, now_s: float) -> VelocityCmd:
        """Returns the active manual command or zero when expired."""
        if self._cmd_time_s is None:
            return ZERO_VELOCITY_CMD

        age_s = now_s - self._cmd_time_s
        if age_s < 0.0 or age_s > self._config.cmd_timeout_s:
            return ZERO_VELOCITY_CMD

        return self._cmd
