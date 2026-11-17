"""Conversions between overhead_core domain types and tsa_interfaces ROS messages."""

from std_msgs.msg import Header
from tsa_interfaces.msg import (
    CubeObservation,
    RobotObservation,
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
) -> VirtualSensors:
    """Assembles all virtual sensor readings into a VirtualSensors message.

    Args:
        header: Standard message header with stamp and frame_id="arena".
        robot_distance: Euclidean distance between robots in meters (-1.0 if < 2).
        robots_in_zone: List of robot names currently within the delivery zone.
        per_robot: List of RobotSensors messages for each observed robot.

    Returns:
        Populated VirtualSensors message.
    """
    msg = VirtualSensors()
    msg.header = header
    msg.robot_distance = float(robot_distance)
    msg.robots_in_zone = robots_in_zone
    msg.per_robot = per_robot
    return msg
