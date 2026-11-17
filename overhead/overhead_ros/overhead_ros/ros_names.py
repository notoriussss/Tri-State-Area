"""Constants for ROS topic names, service names, frame IDs, and QoS profiles."""

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Frame IDs
ARENA_FRAME_ID: str = "arena"

# Topic names
WORLD_STATE_TOPIC: str = "/world/state"
WORLD_SENSORS_TOPIC: str = "/world/sensors"
WORLD_IMAGE_TOPIC: str = "/world/image"
FLEET_PLAN_TOPIC: str = "/fleet/plan"

# Service names
WORLD_GET_SERVICE: str = "/world/get"
OVERHEAD_RECALIBRATE_SERVICE: str = "/overhead/recalibrate"

# QoS profiles matching contract specifications
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

IMAGE_QOS: QoSProfile = QoSProfile(
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
