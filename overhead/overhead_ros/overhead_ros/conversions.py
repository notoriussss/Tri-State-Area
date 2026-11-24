"""Conversions between overhead_core domain types and tsa_interfaces ROS messages."""

from overhead_core.constants import (
    NO_OBSTACLE_KIND,
    ObstacleKind,
)
from overhead_core.types import Obstacle as CoreObstacle
from overhead_core.types import RobotSafety
from std_msgs.msg import Header
from tsa_interfaces.msg import (
    CubeObservation,
    Obstacle,
    RobotObservation,
    RobotObstacles,
    RobotSensors,
    VirtualSensors,
    WorldState,
)


def to_cube_observation_msg(
    track_id: int,
    color: str,
    x: float,
    *,
    y: float,
    confidence: float,
) -> CubeObservation:
    """Converts a tracked cube observation into a CubeObservation message.

    Args:
        track_id: Unique persistent track identifier.
        color: Cube color string ("blue", "yellow", "red").
        x: Floor x-coordinate in meters in frame arena.
        y: Floor y-coordinate in meters in frame arena.
        confidence: Detection confidence in [0.0, 1.0].

    Returns:
        Populated CubeObservation message.
    """
    msg = CubeObservation()
    msg.id = int(track_id)
    msg.color = str(color)
    msg.x = float(x)
    msg.y = float(y)
    msg.confidence = float(confidence)
    return msg


def to_robot_observation_msg(
    name: str,
    x: float,
    y: float,
    *,
    theta: float,
    confidence: float,
) -> RobotObservation:
    """Converts a robot pose into a RobotObservation message.

    Args:
        name: Robot name ("phineas" or "ferb").
        x: Parallax-corrected floor x-coordinate in meters in frame arena.
        y: Parallax-corrected floor y-coordinate in meters in frame arena.
        theta: Heading angle in radians, CCW from +x in (-pi, pi].
        confidence: Detection confidence in [0.0, 1.0].

    Returns:
        Populated RobotObservation message.
    """
    msg = RobotObservation()
    msg.name = str(name)
    msg.x = float(x)
    msg.y = float(y)
    msg.theta = float(theta)
    msg.confidence = float(confidence)
    return msg


def to_world_state_msg(
    header: Header,
    cubes: list[CubeObservation],
    robots: list[RobotObservation],
) -> WorldState:
    """Combines cube and robot observations into a WorldState message.

    Args:
        header: Standard message header with stamp and frame_id="arena".
        cubes: List of active cube observations.
        robots: List of active robot observations.

    Returns:
        Populated WorldState message.
    """
    msg = WorldState()
    msg.header = header
    msg.cubes = cubes
    msg.robots = robots
    return msg


def to_robot_sensors_msg(
    name: str,
    nearest_red_id: int,
    nearest_red_dist: float,
    *,
    events: list[str] | None = None,
) -> RobotSensors:
    """Builds a per-robot virtual sensors message.

    Args:
        name: Robot name ("phineas" or "ferb").
        nearest_red_id: Nearest red cube ID (0 if none visible).
        nearest_red_dist: Distance in meters to nearest red cube (-1.0 if none).
        events: List of proximity and zone event strings.

    Returns:
        Populated RobotSensors message.
    """
    msg = RobotSensors()
    msg.name = str(name)
    msg.nearest_red_id = int(nearest_red_id)
    msg.nearest_red_dist = float(nearest_red_dist)
    msg.events = events if events is not None else []
    return msg


def to_virtual_sensors_msg(
    header: Header,
    robot_distance: float,
    robots_in_zone: list[str],
    *,
    per_robot: list[RobotSensors],
    obstacles: list[Obstacle] | None = None,
    per_robot_obstacles: list[RobotObstacles] | None = None,
) -> VirtualSensors:
    """Assembles all virtual sensor readings into a VirtualSensors message.

    Args:
        header: Standard message header with stamp and frame_id="arena".
        robot_distance: Euclidean distance between robots in meters (-1.0 if < 2).
        robots_in_zone: List of robot names currently within the delivery zone.
        per_robot: List of RobotSensors messages for each observed robot.
        obstacles: Arena-wide safety circle obstacles (cubes and robots).
        per_robot_obstacles: Radar and collision assessments for each robot.

    Returns:
        Populated VirtualSensors message.
    """
    msg = VirtualSensors()
    msg.header = header
    msg.robot_distance = float(robot_distance)
    msg.robots_in_zone = robots_in_zone
    msg.per_robot = per_robot
    msg.obstacles = obstacles if obstacles is not None else []
    msg.per_robot_obstacles = per_robot_obstacles if per_robot_obstacles is not None else []
    return msg


def to_obstacle_msg(obstacle: CoreObstacle) -> Obstacle:
    """Converts a pure core Obstacle into a tsa_interfaces Obstacle message.

    Args:
        obstacle: Circular safety obstacle in the arena frame.

    Returns:
        Populated Obstacle ROS message.
    """
    msg = Obstacle()
    kind_str = (
        obstacle.kind.value if isinstance(obstacle.kind, ObstacleKind) else str(obstacle.kind)
    )
    msg.kind = str(kind_str)
    msg.id = int(obstacle.obstacle_id)
    msg.x = float(obstacle.x_m)
    msg.y = float(obstacle.y_m)
    msg.radius = float(obstacle.radius_m)
    msg.vx = float(obstacle.vx_mps)
    msg.vy = float(obstacle.vy_mps)
    return msg


def to_robot_obstacles_msg(
    name: str,
    safety: RobotSafety,
) -> RobotObstacles:
    """Converts per-robot safety outputs into a RobotObstacles message.

    Args:
        name: Robot name ("phineas" or "ferb").
        safety: Radar sector distances and collision assessment for the robot.

    Returns:
        Populated RobotObstacles ROS message.
    """
    assessment = safety.assessment
    msg = RobotObstacles()
    msg.name = str(name)
    msg.sectors = [float(dist_m) for dist_m in safety.sectors]
    msg.level = str(assessment.level.value)
    msg.time_to_collision = float(assessment.time_to_collision_s)
    msg.nearest_kind = (
        str(assessment.nearest_kind.value)
        if assessment.nearest_kind is not None
        else NO_OBSTACLE_KIND
    )
    msg.nearest_dist = float(assessment.nearest_dist_m)
    msg.nearest_bearing = float(assessment.nearest_bearing_rad)
    return msg


def located_robot_names_from_world_state(msg: WorldState) -> frozenset[str]:
    """Extracts the set of robot names currently present in a WorldState message.

    Args:
        msg: WorldState message published on /world/state.

    Returns:
        Frozen set of robot names currently located in the arena.
    """
    return frozenset(str(robot.name) for robot in msg.robots)
