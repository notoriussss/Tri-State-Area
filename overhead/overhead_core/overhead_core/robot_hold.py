"""Lost robot hold filter with linear confidence decay."""

from dataclasses import dataclass

from overhead_core.constants import ROBOT_HOLD_S
from overhead_core.types import RobotPose


@dataclass(frozen=True)
class HoldResult:
    """Filtered robot poses and visibility status for a single frame."""

    poses: list[RobotPose]
    lost_names: tuple[str, ...]
    is_all_seen: bool


class RobotHoldFilter:
    """Holds the last pose of temporarily unseen robots with linear confidence decay."""

    def __init__(
        self,
        expected_names: tuple[str, ...],
        *,
        hold_s: float = ROBOT_HOLD_S,
    ) -> None:
        """Initializes the lost robot hold filter.

        Args:
            expected_names: Names of robots expected in the arena.
            hold_s: Maximum duration in seconds to hold an unseen robot pose.
        """
        self._expected_names: tuple[str, ...] = tuple(dict.fromkeys(expected_names))
        self._hold_s = hold_s
        self._last_seen_pose: dict[str, RobotPose] = {}
        self._last_seen_time_s: dict[str, float] = {}

    def _resolve_missing_robot(
        self,
        name: str,
        now_s: float,
    ) -> RobotPose | None:
        """Returns a confidence-decayed pose while within hold_s, or None when dropped."""
        last_pose = self._last_seen_pose.get(name)
        if last_pose is None:
            return None

        last_seen_s = self._last_seen_time_s[name]
        held_age_s = max(0.0, now_s - last_seen_s)
        is_expired = self._hold_s <= 0.0 or held_age_s > self._hold_s
        if is_expired:
            del self._last_seen_pose[name]
            del self._last_seen_time_s[name]
            return None

        decay_ratio = max(0.0, 1.0 - (held_age_s / self._hold_s))
        return RobotPose(
            name=last_pose.name,
            x=last_pose.x,
            y=last_pose.y,
            theta=last_pose.theta,
            confidence=last_pose.confidence * decay_ratio,
        )

    def update(
        self,
        seen: dict[str, RobotPose],
        now_s: float,
    ) -> HoldResult:
        """Updates filter state with robots seen in the current frame.

        Args:
            seen: Mapping from robot name to observed RobotPose in world coordinates.
            now_s: Current monotonic timestamp in seconds from the caller clock.

        Returns:
            HoldResult with active poses sorted by name, names of robots not seen
            in this frame, and whether all expected robots were seen.
        """
        poses: list[RobotPose] = []
        lost_names: list[str] = []

        for name in self._expected_names:
            pose = seen.get(name)
            if pose is not None:
                self._last_seen_pose[name] = pose
                self._last_seen_time_s[name] = now_s
                poses.append(pose)
                continue

            lost_names.append(name)
            held_pose = self._resolve_missing_robot(
                name,
                now_s,
            )
            if held_pose is not None:
                poses.append(held_pose)

        poses.sort(key=lambda item: item.name)
        return HoldResult(
            poses=poses,
            lost_names=tuple(lost_names),
            is_all_seen=len(lost_names) == 0,
        )
