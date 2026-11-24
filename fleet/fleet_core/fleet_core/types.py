"""Shared immutable dataclasses and type definitions for fleet_core."""

from dataclasses import dataclass

from fleet_core.constants import (
    CubeColor,
    PlannerState,
    RobotMode,
)


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the arena frame in meters and radians."""

    x_m: float
    y_m: float
    theta_rad: float


@dataclass(frozen=True)
class VelocityCmd:
    """Body-frame holonomic velocity command in REP 103 units."""

    vx_mps: float
    vy_mps: float
    wz_radps: float


ZERO_VELOCITY_CMD: VelocityCmd = VelocityCmd(
    vx_mps=0.0,
    vy_mps=0.0,
    wz_radps=0.0,
)


@dataclass(frozen=True)
class MecanumGeometry:
    """Chassis dimensions used by mecanum inverse kinematics."""

    wheel_radius_m: float
    half_wheelbase_m: float
    half_track_m: float


@dataclass(frozen=True)
class CubeObservation:
    """Tracked cube observation on the arena floor."""

    cube_id: int
    color: CubeColor
    x_m: float
    y_m: float
    confidence: float


@dataclass(frozen=True)
class RobotObservation:
    """Parallax-corrected robot pose on the arena floor."""

    name: str
    x_m: float
    y_m: float
    theta_rad: float
    confidence: float


@dataclass(frozen=True)
class RobotSensors:
    """Virtual sensor measurements for one robot."""

    name: str
    nearest_red_id: int
    nearest_red_dist_m: float
    events: tuple[str, ...] = ()


@dataclass(frozen=True)
class WorldSnapshot:
    """World state tagged with the local reception timestamp in seconds."""

    received_at_s: float
    cubes: tuple[CubeObservation, ...]
    robots: tuple[RobotObservation, ...]


@dataclass(frozen=True)
class VirtualSensorsSnapshot:
    """Virtual sensors tagged with the local reception timestamp in seconds."""

    received_at_s: float
    robot_distance_m: float
    robots_in_zone: tuple[str, ...]
    per_robot: dict[str, RobotSensors]


@dataclass(frozen=True)
class PlanSnapshot:
    """Peer or self plan snapshot in arena coordinates."""

    name: str
    state: PlannerState
    target_cube_id: int
    target_pose: Pose2D
    path: tuple[tuple[float, float], ...]
    received_at_s: float


@dataclass(frozen=True)
class RobotStatusSnapshot:
    """Onboard HAL health and safety telemetry for /<robot>/status."""

    line_detected: bool
    battery_v: float
    cmd_age_ms: int
    mode: RobotMode
