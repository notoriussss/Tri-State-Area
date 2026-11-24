"""Robot velocity estimation with exponential smoothing and gap handling."""

from dataclasses import dataclass

from overhead_core.constants import (
    MIN_VELOCITY_DT_S,
    VELOCITY_MAX_GAP_S,
    VELOCITY_SMOOTHING_ALPHA,
)


@dataclass(frozen=True)
class _RobotState:
    """Last valid pose and smoothed velocity for a single robot."""

    x_m: float
    y_m: float
    last_seen_s: float
    vx_mps: float
    vy_mps: float


class VelocityEstimator:
    """Estimates arena-frame robot velocities from consecutive poses."""

    def __init__(
        self,
        alpha: float = VELOCITY_SMOOTHING_ALPHA,
        *,
        min_dt_s: float = MIN_VELOCITY_DT_S,
        max_gap_s: float = VELOCITY_MAX_GAP_S,
    ) -> None:
        """Initializes the velocity estimator.

        Args:
            alpha: Exponential smoothing factor in [0, 1] applied to new velocity samples.
            min_dt_s: Minimum elapsed seconds required to compute a finite difference.
            max_gap_s: Maximum elapsed seconds before resetting or dropping stale robot state.
        """
        self._alpha = alpha
        self._min_dt_s = min_dt_s
        self._max_gap_s = max_gap_s
        self._states: dict[str, _RobotState] = {}

    def _update_robot(
        self,
        name: str,
        position: tuple[float, float],
        *,
        now_s: float,
    ) -> tuple[float, float]:
        """Updates velocity state for one observed robot."""
        x_m, y_m = position
        prev = self._states.get(name)
        if prev is None:
            self._states[name] = _RobotState(
                x_m=x_m,
                y_m=y_m,
                last_seen_s=now_s,
                vx_mps=0.0,
                vy_mps=0.0,
            )
            return (0.0, 0.0)

        dt_s = now_s - prev.last_seen_s
        if dt_s < self._min_dt_s:
            return (prev.vx_mps, prev.vy_mps)

        if dt_s > self._max_gap_s:
            self._states[name] = _RobotState(
                x_m=x_m,
                y_m=y_m,
                last_seen_s=now_s,
                vx_mps=0.0,
                vy_mps=0.0,
            )
            return (0.0, 0.0)

        raw_vx_mps = (x_m - prev.x_m) / dt_s
        raw_vy_mps = (y_m - prev.y_m) / dt_s
        vx_mps = self._alpha * raw_vx_mps + (1.0 - self._alpha) * prev.vx_mps
        vy_mps = self._alpha * raw_vy_mps + (1.0 - self._alpha) * prev.vy_mps
        self._states[name] = _RobotState(
            x_m=x_m,
            y_m=y_m,
            last_seen_s=now_s,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
        )
        return (vx_mps, vy_mps)

    def _drop_stale_states(
        self,
        active_positions: dict[str, tuple[float, float]],
        now_s: float,
    ) -> None:
        """Removes stored state for absent robots whose last sighting exceeded the gap limit."""
        expired_names = [
            name
            for name, state in self._states.items()
            if name not in active_positions and now_s - state.last_seen_s > self._max_gap_s
        ]
        for name in expired_names:
            del self._states[name]

    def update(
        self,
        positions: dict[str, tuple[float, float]],
        now_s: float,
    ) -> dict[str, tuple[float, float]]:
        """Updates and returns smoothed arena-frame velocities for observed robots.

        Args:
            positions: Mapping from robot name to (x_m, y_m) in the arena frame, meters.
            now_s: Current timestamp in seconds.

        Returns:
            Mapping from each robot name in ``positions`` to (vx_mps, vy_mps) in m/s.
        """
        self._drop_stale_states(
            positions,
            now_s,
        )
        velocities: dict[str, tuple[float, float]] = {}
        for name, position in positions.items():
            velocities[name] = self._update_robot(
                name,
                position,
                now_s=now_s,
            )
        return velocities
