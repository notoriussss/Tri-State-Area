"""ROS 2 topic names, service names, frame IDs, and QoS profiles for fleet_ros."""

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Frame IDs
ARENA_FRAME_ID: str = "arena"

# Shared contract topics
WORLD_STATE_TOPIC: str = "/world/state"
WORLD_SENSORS_TOPIC: str = "/world/sensors"
ZONE_OCCUPANT_TOPIC: str = "/zone/occupant"
PERRY_STATE_TOPIC: str = "/perry/state"

# Shared contract services
ZONE_REQUEST_SERVICE: str = "/zone/request"
ZONE_RELEASE_SERVICE: str = "/zone/release"

# QoS profiles matching docs/contract.md
STATE_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

SENSOR_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

PLAN_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

STATUS_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

OCCUPANT_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def plan_topic_for(robot_name: str) -> str:
    """Returns the contract plan topic name for a collector robot."""
    return f"/{robot_name}/plan"


def status_topic_for(robot_name: str) -> str:
    """Returns the contract status topic name for a collector robot."""
    return f"/{robot_name}/status"
