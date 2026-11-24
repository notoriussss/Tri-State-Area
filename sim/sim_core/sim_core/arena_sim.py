"""Pure 2D holonomic arena physics, virtual sensors, and mock Perry zone arbiter."""

import math
from dataclasses import dataclass

from fleet_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    PEER_COLLISION_ALERT_M,
    POINT_EPSILON_M,
    RED_STRICT_RADIUS_M,
    ZONE_GREEN_LINE_X_M,
    ZONE_MAX_X_M,
    ZONE_MAX_Y_M,
    ZONE_MIN_Y_M,
    CubeColor,
    RobotName,
    VirtualSensorEvent,
)
from fleet_core.math_utils import wrap_angle
from fleet_core.types import (
    CubeObservation,
    Pose2D,
    RobotObservation,
    RobotSensors,
    VelocityCmd,
    VirtualSensorsSnapshot,
    WorldSnapshot,
)

DEFAULT_LEASE_DURATION_S: float = 60.0
DEFAULT_LINE_SENSOR_AHEAD_M: float = 0.08
DEFAULT_SCOOP_CONTACT_DIST_M: float = 0.10
DEFAULT_SCOOP_HALF_WIDTH_M: float = 0.07
DEFAULT_SCOOP_HOLD_AHEAD_M: float = 0.11
DEFAULT_BUMPER_RADIUS_M: float = 0.095
SIM_CONFIDENCE: float = 1.0


@dataclass
class SimCube:
    """Mutable simulated cube state on the arena floor."""

    cube_id: int
    color: CubeColor
    x_m: float
    y_m: float


class MockPerryArbiter:
    """In-memory mock of Perry's stage-1 delivery-zone lease arbiter."""

    def __init__(self, lease_duration_s: float = DEFAULT_LEASE_DURATION_S) -> None:
        """Initializes an empty zone with zero delivered cubes."""
        self._lease_duration_s = lease_duration_s
        self._holder: str = ""
        self._expires_at_s: float = 0.0
        self.gate_closed: bool = False
        self.delivered_counts: dict[str, int] = {
            color.value: 0
            for color in CubeColor
        }
        self.delivered_total: int = 0

    def occupant(self, now_s: float) -> str:
        """Returns the current zone holder or empty string if expired."""
        if self._holder and now_s >= self._expires_at_s:
            self._holder = ""
        return self._holder

    def request_zone(
        self,
        robot_name: str,
        now_s: float,
    ) -> tuple[bool, str, float]:
        """Handles a /zone/request call with 60 s exclusive lease semantics."""
        holder = self.occupant(now_s)
        if self.gate_closed and holder != robot_name:
            return False, holder, 0.0

        if holder in ("", robot_name):
            self._holder = robot_name
            self._expires_at_s = now_s + self._lease_duration_s
            return True, robot_name, self._lease_duration_s

        return False, holder, 0.0

    def release_zone(
        self,
        robot_name: str,
        *,
        delivered: bool,
        color: str,
    ) -> tuple[bool, int]:
        """Handles a /zone/release call and updates official delivery counters."""
        if self._holder != robot_name:
            return False, self.delivered_total

        self._holder = ""
        self._expires_at_s = 0.0
        if delivered:
            self.delivered_total += 1
            if color in self.delivered_counts:
                self.delivered_counts[color] += 1

        return True, self.delivered_total


class ArenaSimulator:
    """Simulates Phineas, Ferb, pushable cubes, black border tape, and virtual sensors."""

    def __init__(
        self,
        robots: dict[str, Pose2D],
        cubes: list[SimCube],
    ) -> None:
        """Initializes arena entities and collision telemetry."""
        self.robots: dict[str, Pose2D] = dict(robots)
        self.cubes: list[SimCube] = list(cubes)
        self._prev_in_zone: set[str] = set()
        self.min_robot_distance_m: float = math.inf

    def step(
        self,
        commands: dict[str, VelocityCmd],
        dt_s: float,
    ) -> None:
        """Integrates robot poses and cube push contacts forward by dt_s seconds."""
        for name, cmd in commands.items():
            if name not in self.robots:
                continue

            pose = self.robots[name]
            cos_theta = math.cos(pose.theta_rad)
            sin_theta = math.sin(pose.theta_rad)
            dx_world = (cmd.vx_mps * cos_theta - cmd.vy_mps * sin_theta) * dt_s
            dy_world = (cmd.vx_mps * sin_theta + cmd.vy_mps * cos_theta) * dt_s
            new_pose = Pose2D(
                x_m=pose.x_m + dx_world,
                y_m=pose.y_m + dy_world,
                theta_rad=wrap_angle(pose.theta_rad + cmd.wz_radps * dt_s),
            )
            self.robots[name] = new_pose
            self._resolve_cube_pushes(
                new_pose,
                dx_world,
                dy_world,
            )

        self._update_min_robot_distance()

    def is_on_black_line(self, robot_name: str) -> bool:
        """Returns True when the front-bottom line sensor crosses the black border tape.

        The delivery notch on the right edge (x >= ARENA_WIDTH_M and y in
        [ZONE_MIN_Y_M, ZONE_MAX_Y_M]) has no black tape.
        """
        pose = self.robots[robot_name]
        sx = pose.x_m + DEFAULT_LINE_SENSOR_AHEAD_M * math.cos(pose.theta_rad)
        sy = pose.y_m + DEFAULT_LINE_SENSOR_AHEAD_M * math.sin(pose.theta_rad)
        is_in_notch = sx >= ARENA_WIDTH_M and ZONE_MIN_Y_M <= sy <= ZONE_MAX_Y_M
        if is_in_notch:
            return False

        is_inside_tape = 0.0 <= sx <= ARENA_WIDTH_M and 0.0 <= sy <= ARENA_HEIGHT_M
        return not is_inside_tape

    def snapshot(
        self,
        now_s: float,
    ) -> tuple[WorldSnapshot, VirtualSensorsSnapshot]:
        """Generates contract-identical WorldSnapshot and VirtualSensorsSnapshot."""
        cube_obs = tuple(
            CubeObservation(
                cube_id=cube.cube_id,
                color=cube.color,
                x_m=cube.x_m,
                y_m=cube.y_m,
                confidence=SIM_CONFIDENCE,
            )
            for cube in self.cubes
        )
        robot_obs = tuple(
            RobotObservation(
                name=name,
                x_m=pose.x_m,
                y_m=pose.y_m,
                theta_rad=pose.theta_rad,
                confidence=SIM_CONFIDENCE,
            )
            for name, pose in sorted(self.robots.items())
        )
        world = WorldSnapshot(
            received_at_s=now_s,
            cubes=cube_obs,
            robots=robot_obs,
        )
        sensors = self._compute_virtual_sensors(now_s)
        return world, sensors

    def _resolve_cube_pushes(
        self,
        pose: Pose2D,
        dx_world: float,
        dy_world: float,
    ) -> None:
        """Pushes any cube contacting the robot scoop or chassis forward smoothly."""
        cos_theta = math.cos(pose.theta_rad)
        sin_theta = math.sin(pose.theta_rad)

        for cube in self.cubes:
            rel_x = cube.x_m - pose.x_m
            rel_y = cube.y_m - pose.y_m
            ahead_m = cos_theta * rel_x + sin_theta * rel_y
            lateral_m = -sin_theta * rel_x + cos_theta * rel_y
            is_in_scoop_pocket = (
                0.0 <= ahead_m <= DEFAULT_SCOOP_CONTACT_DIST_M
                and abs(lateral_m) <= DEFAULT_SCOOP_HALF_WIDTH_M
            )
            if is_in_scoop_pocket:
                cube.x_m = pose.x_m + DEFAULT_SCOOP_HOLD_AHEAD_M * cos_theta
                cube.y_m = pose.y_m + DEFAULT_SCOOP_HOLD_AHEAD_M * sin_theta
                continue

            dist = math.hypot(
                rel_x,
                rel_y,
            )
            if POINT_EPSILON_M < dist < DEFAULT_BUMPER_RADIUS_M:
                push = DEFAULT_BUMPER_RADIUS_M - dist
                cube.x_m += (rel_x / dist) * push + dx_world * 0.5
                cube.y_m += (rel_y / dist) * push + dy_world * 0.5

    def _update_min_robot_distance(self) -> None:
        """Tracks closest approach between Phineas and Ferb across the simulation."""
        if RobotName.PHINEAS.value not in self.robots or RobotName.FERB.value not in self.robots:
            return

        p1 = self.robots[RobotName.PHINEAS.value]
        p2 = self.robots[RobotName.FERB.value]
        dist = math.hypot(
            p1.x_m - p2.x_m,
            p1.y_m - p2.y_m,
        )
        self.min_robot_distance_m = min(
            self.min_robot_distance_m,
            dist,
        )

    def _compute_virtual_sensors(self, now_s: float) -> VirtualSensorsSnapshot:
        """Computes robot_distance, zone containment, and near:red / near:robot events."""
        robot_dist = -1.0
        names = sorted(self.robots.keys())
        if len(names) >= 2:
            p0 = self.robots[names[0]]
            p1 = self.robots[names[1]]
            robot_dist = math.hypot(
                p0.x_m - p1.x_m,
                p0.y_m - p1.y_m,
            )

        current_in_zone: set[str] = set()
        for name, pose in self.robots.items():
            is_in_zone = (
                ZONE_GREEN_LINE_X_M <= pose.x_m <= ZONE_MAX_X_M
                and ZONE_MIN_Y_M <= pose.y_m <= ZONE_MAX_Y_M
            )
            if is_in_zone:
                current_in_zone.add(name)

        red_cubes = [cube for cube in self.cubes if cube.color == CubeColor.RED]
        per_robot: dict[str, RobotSensors] = {}
        for name, pose in self.robots.items():
            nearest_id, nearest_dist = _nearest_red(
                pose,
                red_cubes,
            )
            events: list[str] = []
            if name in current_in_zone and name not in self._prev_in_zone:
                events.append(VirtualSensorEvent.ENTERED_ZONE.value)
            elif name in self._prev_in_zone and name not in current_in_zone:
                events.append(VirtualSensorEvent.LEFT_ZONE.value)

            if nearest_id != 0 and nearest_dist < RED_STRICT_RADIUS_M:
                events.append(f"{VirtualSensorEvent.NEAR_RED_PREFIX.value}:{nearest_id}")
            if 0.0 <= robot_dist < PEER_COLLISION_ALERT_M:
                events.append(VirtualSensorEvent.NEAR_ROBOT.value)

            per_robot[name] = RobotSensors(
                name=name,
                nearest_red_id=nearest_id,
                nearest_red_dist_m=nearest_dist,
                events=tuple(events),
            )

        self._prev_in_zone = current_in_zone
        return VirtualSensorsSnapshot(
            received_at_s=now_s,
            robot_distance_m=robot_dist,
            robots_in_zone=tuple(sorted(current_in_zone)),
            per_robot=per_robot,
        )


def _nearest_red(
    pose: Pose2D,
    red_cubes: list[SimCube],
) -> tuple[int, float]:
    """Finds the nearest red cube ID and Euclidean distance in meters."""
    if not red_cubes:
        return 0, -1.0

    best = min(
        red_cubes,
        key=lambda cube: math.hypot(
            cube.x_m - pose.x_m,
            cube.y_m - pose.y_m,
        ),
    )
    return best.cube_id, math.hypot(
        best.x_m - pose.x_m,
        best.y_m - pose.y_m,
    )
