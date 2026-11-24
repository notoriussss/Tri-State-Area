"""Shared dataclasses and type definitions for overhead_core."""

from dataclasses import dataclass, field

from overhead_core.constants import (
    CollisionLevel,
    ObstacleKind,
)


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


@dataclass(frozen=True)
class Obstacle:
    """Circular obstacle with position, safety radius, and velocity in arena frame."""

    kind: str
    obstacle_id: int
    x_m: float
    y_m: float
    radius_m: float
    vx_mps: float
    vy_mps: float


@dataclass(frozen=True)
class WallSegment:
    """Line-segment obstacle representing an arena wall or delivery zone border."""

    kind: str
    x0_m: float
    y0_m: float
    x1_m: float
    y1_m: float


@dataclass(frozen=True)
class ObstacleSet:
    """Complete set of circular and segment obstacles for a single robot."""

    circles: list[Obstacle]
    walls: list[WallSegment]


@dataclass(frozen=True)
class CollisionAssessment:
    """Forward-cone collision assessment for a single robot."""

    level: CollisionLevel
    time_to_collision_s: float
    nearest_kind: ObstacleKind | None
    nearest_dist_m: float
    nearest_bearing_rad: float
