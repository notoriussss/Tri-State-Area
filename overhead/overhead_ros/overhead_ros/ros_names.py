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
ZONE_OCCUPANT_TOPIC: str = "/zone/occupant"
PERRY_STATE_TOPIC: str = "/perry/state"

# Service names
WORLD_GET_SERVICE: str = "/world/get"
WORLD_CORRIDOR_FREE_SERVICE: str = "/world/corridor_free"
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

ZONE_OCCUPANT_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

PERRY_STATE_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Node names
OVERHEAD_NODE_NAME: str = "overhead_node"

# Overhead node ROS parameter names
PARAM_CAMERA_INDEX: str = "camera_index"
PARAM_VIDEO_SOURCE: str = "video_source"
PARAM_WIDTH: str = "width"
PARAM_HEIGHT: str = "height"
PARAM_RATE_HZ: str = "rate_hz"
PARAM_CALIBRATION_RETRY_S: str = "calibration_retry_s"
PARAM_ARENA_WIDTH_M: str = "arena_width_m"
PARAM_ARENA_HEIGHT_M: str = "arena_height_m"
PARAM_CAM_HEIGHT_M: str = "cam_height_m"
PARAM_CAMERA_CENTER_WORLD: str = "camera_center_world"
PARAM_ZONE_RECT: str = "zone_rect"
PARAM_ROBOTS: str = "robots"
PARAM_IMAGE_RATE_HZ: str = "image_rate_hz"
PARAM_MIN_AREA_PX: str = "min_area_px"
PARAM_NEAR_RED_M: str = "near_red_m"
PARAM_HSV_BLUE: str = "hsv_blue"
PARAM_HSV_YELLOW: str = "hsv_yellow"
PARAM_HSV_RED_1: str = "hsv_red_1"
PARAM_HSV_RED_2: str = "hsv_red_2"
PARAM_HSV_ROBOT_PHINEAS: str = "hsv_robot_phineas"
PARAM_HSV_ROBOT_FERB: str = "hsv_robot_ferb"
