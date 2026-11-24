"""Pure stateful vision, tracking, virtual sensors, and safety pipeline."""

from dataclasses import dataclass
from typing import Any

import numpy as np

from overhead_core import virtual_sensors
from overhead_core.constants import (
    DEFAULT_ZONE_RECT,
    NEAR_RED_DISTANCE_M,
    ROBOT_HOLD_S,
    ObstacleKind,
)
from overhead_core.robot_hold import RobotHoldFilter
from overhead_core.tracker import CubeTracker
from overhead_core.types import (
    Obstacle,
    RobotPose,
    SafetyResult,
)
from overhead_core.velocity import VelocityEstimator
from overhead_core.world_frame import (
    WorldFrameConfig,
    detect_robot_poses,
    process_cubes,
)
from overhead_core.world_safety import (
    SafetyConfig,
    compute_safety,
)


@dataclass(frozen=True)
class DetectionStepResult:
    """Outputs of a single frame's cube tracking and robot hold step."""

    tracked_cubes: list[tuple[int, str, float, float, float]]
    seen_robots: dict[str, RobotPose]
    robot_poses: list[RobotPose]
    is_all_seen: bool


@dataclass(frozen=True)
class WorldEvaluationResult:
    """Virtual sensors, per-robot safety outputs, and red-cube obstacles for one cycle."""

    vs_result: dict[str, Any]
    safety_result: SafetyResult
    red_obstacles: list[Obstacle]


class WorldPipeline:
    """Runs pure cube tracking, robot pose hold, virtual sensors, and safety checks."""

    def __init__(
        self,
        world_config: WorldFrameConfig,
        *,
        zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT,
        near_red_m: float = NEAR_RED_DISTANCE_M,
        hold_s: float = ROBOT_HOLD_S,
    ) -> None:
        """Initializes stateful trackers and zone memory for the vision loop.

        Args:
            world_config: Vision and parallax geometry configuration.
            zone_rect: Delivery zone bounds (xmin, ymin, xmax, ymax) in meters.
            near_red_m: Distance threshold in meters for near-red sensor events.
            hold_s: Hold duration in seconds before dropping an unseen robot.
        """
        self._world_config = world_config
        self._zone_rect = zone_rect
        self._near_red_m = near_red_m
        self._tracker = CubeTracker()
        self._robot_hold = RobotHoldFilter(
            world_config.robot_names,
            hold_s=hold_s,
        )
        self._velocity_estimator = VelocityEstimator()
        self._prev_in_zone: set[str] = set()

    def detect_and_track(
        self,
        frame: np.ndarray,
        *,
        homography: np.ndarray,
        now_s: float,
    ) -> DetectionStepResult:
        """Detects cubes and robots and updates the cube tracker and robot hold filter.

        Args:
            frame: Input BGR camera frame.
            homography: 3x3 homography mapping image pixels to world meters.
            now_s: Current monotonic timestamp in seconds.

        Returns:
            DetectionStepResult with active tracked cubes, raw seen robots,
            held robot poses, and whether all configured robots are visible.
        """
        tracked_cubes = process_cubes(
            frame,
            homography=homography,
            tracker=self._tracker,
            now_s=now_s,
            config=self._world_config,
        )
        seen_robots = detect_robot_poses(
            frame,
            homography=homography,
            config=self._world_config,
        )
        hold_result = self._robot_hold.update(
            seen_robots,
            now_s,
        )
        return DetectionStepResult(
            tracked_cubes=tracked_cubes,
            seen_robots=seen_robots,
            robot_poses=hold_result.poses,
            is_all_seen=hold_result.is_all_seen,
        )

    def evaluate_sensors_and_safety(
        self,
        detection: DetectionStepResult,
        *,
        now_s: float,
        safety_config: SafetyConfig,
    ) -> WorldEvaluationResult:
        """Computes virtual sensors, robot velocities, and safety circles for one cycle.

        Args:
            detection: Output of detect_and_track for the current frame.
            now_s: Current monotonic timestamp in seconds.
            safety_config: Arena bounds and Perry delivery-zone safety snapshot.

        Returns:
            WorldEvaluationResult containing virtual sensors, per-robot safety
            assessments, and red-cube obstacle circles.
        """
        robots_for_sensors = {pose.name: (pose.x, pose.y) for pose in detection.robot_poses}
        cubes_for_sensors = [(t[0], t[1], t[2], t[3]) for t in detection.tracked_cubes]
        vs_result = virtual_sensors.compute(
            robots_for_sensors,
            cubes_for_sensors,
            zone_rect=self._zone_rect,
            prev_in_zone=self._prev_in_zone,
            near_red_m=self._near_red_m,
        )
        self._prev_in_zone = vs_result["in_zone"]

        seen_positions = {name: (pose.x, pose.y) for name, pose in detection.seen_robots.items()}
        velocities = self._velocity_estimator.update(
            seen_positions,
            now_s,
        )
        safety_result = compute_safety(
            detection.robot_poses,
            detection.tracked_cubes,
            velocities=velocities,
            config=safety_config,
        )
        red_obstacles = [
            circle for circle in safety_result.circles if circle.kind == ObstacleKind.RED_CUBE
        ]
        return WorldEvaluationResult(
            vs_result=vs_result,
            safety_result=safety_result,
            red_obstacles=red_obstacles,
        )
