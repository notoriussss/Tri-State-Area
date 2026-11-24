"""Local-clock world model and short-horizon latency compensation."""

import math
from dataclasses import dataclass

from fleet_core.constants import (
    CAMERA_TIMEOUT_S,
    LATENCY_PREDICTION_THRESHOLD_S,
    MIN_OBSERVATION_CONFIDENCE,
    CubeColor,
)
from fleet_core.math_utils import wrap_angle
from fleet_core.types import (
    CubeObservation,
    Pose2D,
    RobotObservation,
    VelocityCmd,
    VirtualSensorsSnapshot,
    WorldSnapshot,
)


@dataclass(frozen=True)
class WorldModelConfig:
    """Freshness, identity, and latency parameters for WorldModel."""

    robot_name: str
    peer_name: str
    camera_timeout_s: float = CAMERA_TIMEOUT_S
    min_confidence: float = MIN_OBSERVATION_CONFIDENCE
    prediction_threshold_s: float = LATENCY_PREDICTION_THRESHOLD_S


def predict_pose(
    pose: Pose2D,
    cmd: VelocityCmd,
    latency_s: float,
    *,
    threshold_s: float = LATENCY_PREDICTION_THRESHOLD_S,
) -> Pose2D:
    """Projects a robot pose forward when measured camera latency exceeds threshold.

    Args:
        pose: Observed robot pose in arena coordinates (m, m, rad).
        cmd: Last body-frame velocity command applied by the HAL.
        latency_s: Estimated camera-to-control latency in seconds.
        threshold_s: Minimum latency in seconds before prediction activates.

    Returns:
        Predicted Pose2D in arena coordinates, or the original pose below threshold.
    """
    if latency_s <= threshold_s:
        return pose

    cos_theta = math.cos(pose.theta_rad)
    sin_theta = math.sin(pose.theta_rad)
    vx_world = cmd.vx_mps * cos_theta - cmd.vy_mps * sin_theta
    vy_world = cmd.vx_mps * sin_theta + cmd.vy_mps * cos_theta

    return Pose2D(
        x_m=pose.x_m + vx_world * latency_s,
        y_m=pose.y_m + vy_world * latency_s,
        theta_rad=wrap_angle(pose.theta_rad + cmd.wz_radps * latency_s),
    )


class WorldModel:
    """Stores overhead snapshots and evaluates freshness strictly on the local clock."""

    def __init__(self, config: WorldModelConfig) -> None:
        """Initializes an empty world model with no observations received yet."""
        self._config = config
        self._world: WorldSnapshot | None = None
        self._sensors: VirtualSensorsSnapshot | None = None

    def update_world(self, snapshot: WorldSnapshot) -> None:
        """Stores the latest WorldSnapshot tagged with local reception time."""
        self._world = snapshot

    def update_sensors(self, snapshot: VirtualSensorsSnapshot) -> None:
        """Stores the latest VirtualSensorsSnapshot tagged with local reception time."""
        self._sensors = snapshot

    def is_world_fresh(self, now_s: float) -> bool:
        """Returns True when a WorldSnapshot arrived within camera_timeout_s."""
        if self._world is None:
            return False

        age_s = now_s - self._world.received_at_s
        return 0.0 <= age_s <= self._config.camera_timeout_s

    def is_sensors_fresh(self, now_s: float) -> bool:
        """Returns True when VirtualSensors arrived within camera_timeout_s."""
        if self._sensors is None:
            return False

        age_s = now_s - self._sensors.received_at_s
        return 0.0 <= age_s <= self._config.camera_timeout_s

    def world_age_s(self, now_s: float) -> float | None:
        """Returns age in seconds of the latest WorldSnapshot on the local clock."""
        if self._world is None:
            return None

        return max(
            0.0,
            now_s - self._world.received_at_s,
        )

    def robot_observation(
        self,
        name: str,
        now_s: float,
    ) -> RobotObservation | None:
        """Returns a fresh, high-confidence observation for the named robot."""
        if not self.is_world_fresh(now_s) or self._world is None:
            return None

        for robot in self._world.robots:
            is_valid_match = (
                robot.name == name
                and robot.confidence >= self._config.min_confidence
            )
            if is_valid_match:
                return robot

        return None

    def own_pose(
        self,
        now_s: float,
        *,
        last_cmd: VelocityCmd | None = None,
        latency_s: float = 0.0,
    ) -> Pose2D | None:
        """Returns own robot pose, optionally compensated for high camera latency."""
        obs = self.robot_observation(
            self._config.robot_name,
            now_s,
        )
        if obs is None:
            return None

        pose = Pose2D(
            x_m=obs.x_m,
            y_m=obs.y_m,
            theta_rad=wrap_angle(obs.theta_rad),
        )
        if last_cmd is None:
            return pose

        return predict_pose(
            pose,
            last_cmd,
            latency_s,
            threshold_s=self._config.prediction_threshold_s,
        )

    def peer_pose(self, now_s: float) -> Pose2D | None:
        """Returns the peer robot pose when fresh and above confidence threshold."""
        obs = self.robot_observation(
            self._config.peer_name,
            now_s,
        )
        if obs is None:
            return None

        return Pose2D(
            x_m=obs.x_m,
            y_m=obs.y_m,
            theta_rad=wrap_angle(obs.theta_rad),
        )

    def cubes_by_color(
        self,
        color: CubeColor,
        now_s: float,
    ) -> tuple[CubeObservation, ...]:
        """Returns all fresh, high-confidence cubes of the requested color."""
        if not self.is_world_fresh(now_s) or self._world is None:
            return ()

        return tuple(
            cube
            for cube in self._world.cubes
            if cube.color == color and cube.confidence >= self._config.min_confidence
        )

    def all_cubes(self, now_s: float) -> tuple[CubeObservation, ...]:
        """Returns all fresh, high-confidence cubes regardless of color."""
        if not self.is_world_fresh(now_s) or self._world is None:
            return ()

        return tuple(
            cube
            for cube in self._world.cubes
            if cube.confidence >= self._config.min_confidence
        )

    def cube_by_id(
        self,
        cube_id: int,
        now_s: float,
    ) -> CubeObservation | None:
        """Looks up a specific tracked cube ID in the current fresh world state."""
        for cube in self.all_cubes(now_s):
            if cube.cube_id == cube_id:
                return cube

        return None

    def robot_distance_m(self, now_s: float) -> float | None:
        """Returns center-to-center distance between both robots in meters.

        Uses overhead virtual sensors when fresh and valid, otherwise falls back
        to Euclidean distance computed directly from WorldState poses.
        """
        if self.is_sensors_fresh(now_s) and self._sensors is not None:
            if self._sensors.robot_distance_m >= 0.0:
                return self._sensors.robot_distance_m

        own = self.own_pose(now_s)
        peer = self.peer_pose(now_s)
        if own is None or peer is None:
            return None

        return math.hypot(
            peer.x_m - own.x_m,
            peer.y_m - own.y_m,
        )

    def has_virtual_event(
        self,
        event: str,
        now_s: float,
    ) -> bool:
        """Returns True if own robot received the named virtual sensor event."""
        if not self.is_sensors_fresh(now_s) or self._sensors is None:
            return False

        robot_sensors = self._sensors.per_robot.get(self._config.robot_name)
        if robot_sensors is None:
            return False

        return event in robot_sensors.events

    def is_own_in_zone(self, now_s: float) -> bool:
        """Returns True if overhead virtual sensors place own robot inside the zone."""
        if not self.is_sensors_fresh(now_s) or self._sensors is None:
            return False

        return self._config.robot_name in self._sensors.robots_in_zone
