"""Stateful safety evaluation holder for the standalone overhead web viewer."""

import time
from collections.abc import Callable, Sequence

from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    DEFAULT_ROBOT_NAMES,
    DEFAULT_ZONE_RECT,
    ROBOT_HOLD_S,
)
from overhead_core.robot_hold import RobotHoldFilter
from overhead_core.types import (
    RobotPose,
    RobotSafetyStatus,
    SafetyResult,
    ViewerSafetyResult,
)
from overhead_core.velocity import VelocityEstimator
from overhead_core.world_safety import (
    SafetyConfig,
    compute_safety,
)


def build_robot_statuses(safety: SafetyResult) -> tuple[RobotSafetyStatus, ...]:
    """Extracts serializable per-robot safety status summaries sorted by name.

    Args:
        safety: Arena safety evaluation containing per-robot assessments.

    Returns:
        Tuple of RobotSafetyStatus entries ordered alphabetically by robot name.
    """
    return tuple(
        RobotSafetyStatus(
            name=name,
            level=robot_safety.assessment.level,
            ttc_s=robot_safety.assessment.time_to_collision_s,
            nearest_kind=robot_safety.assessment.nearest_kind,
            nearest_dist_m=robot_safety.assessment.nearest_dist_m,
        )
        for name, robot_safety in sorted(safety.per_robot.items())
    )


class ViewerSafety:
    """Maintains velocity, hold filter, and arena safety state across viewer frames."""

    def __init__(
        self,
        expected_names: tuple[str, ...] = DEFAULT_ROBOT_NAMES,
        *,
        hold_s: float = ROBOT_HOLD_S,
        arena_width_m: float = ARENA_WIDTH_M,
        arena_height_m: float = ARENA_HEIGHT_M,
        zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Initializes velocity estimator, hold filter, and viewer safety config.

        Args:
            expected_names: Robot names tracked by the lost-robot hold filter.
            hold_s: Duration in seconds to hold a temporarily missing robot pose.
            arena_width_m: Arena width along world +x in meters.
            arena_height_m: Arena height along world +y in meters.
            zone_rect: Delivery zone bounds (xmin, ymin, xmax, ymax) in meters.
            clock: Monotonic clock callable returning seconds.
        """
        self._hold_filter = RobotHoldFilter(
            expected_names,
            hold_s=hold_s,
        )
        self._velocity_estimator = VelocityEstimator()
        self._safety_config = SafetyConfig(
            arena_width_m=arena_width_m,
            arena_height_m=arena_height_m,
            zone_rect=zone_rect,
            zone_occupant=None,
            is_gate_closed=False,
            carrying_robots=frozenset(),
        )
        self._clock = clock

    def update(
        self,
        robots: Sequence[RobotPose],
        cubes: Sequence[tuple[int, str, float, float, float]] = (),
        *,
        now_s: float | None = None,
    ) -> ViewerSafetyResult:
        """Updates hold and velocity state and computes arena safety for one frame.

        Args:
            robots: Currently detected robot poses in the arena frame.
            cubes: Detected cubes as (track_id, color, x_m, y_m, confidence).
            now_s: Optional timestamp in seconds overriding the injected clock.

        Returns:
            ViewerSafetyResult containing held poses, velocities, and SafetyResult.
        """
        timestamp_s = now_s if now_s is not None else self._clock()
        seen_by_name = {pose.name: pose for pose in robots}
        hold_result = self._hold_filter.update(
            seen_by_name,
            timestamp_s,
        )
        seen_positions = {name: (pose.x, pose.y) for name, pose in seen_by_name.items()}
        velocities = self._velocity_estimator.update(
            seen_positions,
            timestamp_s,
        )
        safety = compute_safety(
            hold_result.poses,
            list(cubes),
            velocities=velocities,
            config=self._safety_config,
        )
        return ViewerSafetyResult(
            robot_poses=hold_result.poses,
            safety=safety,
            velocities=velocities,
            lost_names=hold_result.lost_names,
            is_all_seen=hold_result.is_all_seen,
        )
