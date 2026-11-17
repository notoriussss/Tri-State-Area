"""Shared dataclasses and type definitions for overhead_core."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class CubeDetection:
    """Raw cube detection in image coordinates."""

    color: str
    u: float
    v: float
    area: float


@dataclass(frozen=True)
class TrackedCube:
    """Tracked cube observation in world floor coordinates."""

    track_id: int
    color: str
    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class RobotPose:
    """Robot pose in world coordinates with parallax-corrected position."""

    name: str
    x: float
    y: float
    theta: float
    confidence: float


@dataclass(frozen=True)
class RobotSensorsResult:
    """Virtual sensor measurements for a single robot."""

    name: str
    nearest_red_id: int
    nearest_red_dist: float
    events: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class VirtualSensorsResult:
    """Complete virtual sensor computations across the arena."""

    robot_distance: float
    robots_in_zone: list[str]
    per_robot: dict[str, RobotSensorsResult]
    in_zone: set[str]
