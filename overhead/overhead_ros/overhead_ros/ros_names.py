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
PERRY_ZONE_REQUEST_SERVICE: str = "/zone/request"
PERRY_ZONE_RELEASE_SERVICE: str = "/zone/release"

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

# VOLATILE on purpose: a late-joining robot must never receive a cached stale occupant.
ROBOT_ZONE_OCCUPANT_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

PERRY_STATE_QOS: QoSProfile = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# Node names
OVERHEAD_NODE_NAME: str = "overhead_node"
MANIPULATOR_RELAY_NODE_NAME: str = "manipulator_relay_node"
ROBOT_LINK_CHECK_NODE_NAME: str = "robot_link_check_node"

# Package resource identifiers for config loading
OVERHEAD_ROS_PACKAGE_NAME: str = "overhead_ros"
CONFIG_DIR_NAME: str = "config"
ROBOTS_CONFIG_FILENAME: str = "robots.yaml"
ROS_PARAMETERS_KEY: str = "ros__parameters"
UTF8_ENCODING: str = "utf-8"

# Per-robot topic suffixes
PLAN_TOPIC_SUFFIX: str = "/plan"
STATUS_TOPIC_SUFFIX: str = "/status"

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
PARAM_MAX_AREA_PX: str = "max_area_px"
PARAM_NEAR_RED_M: str = "near_red_m"
PARAM_ROBOT_DETECTOR: str = "robot_detector"
PARAM_ARUCO_DICTIONARY: str = "aruco_dictionary"
PARAM_ARUCO_IDS: str = "aruco_ids"
PARAM_ARUCO_HEADING_OFFSET_RAD: str = "aruco_heading_offset_rad"
PARAM_HSV_BLUE: str = "hsv_blue"
PARAM_HSV_YELLOW: str = "hsv_yellow"
PARAM_HSV_RED_1: str = "hsv_red_1"
PARAM_HSV_RED_2: str = "hsv_red_2"
PARAM_HSV_ROBOT_PHINEAS_1: str = "hsv_robot_phineas_1"
PARAM_HSV_ROBOT_PHINEAS_2: str = "hsv_robot_phineas_2"
PARAM_HSV_ROBOT_FERB: str = "hsv_robot_ferb"
PARAM_ROBOT_HEADING_OFFSET_RAD_PHINEAS: str = "robot_heading_offset_rad_phineas"
PARAM_ROBOT_HEADING_OFFSET_RAD_FERB: str = "robot_heading_offset_rad_ferb"
PARAM_ROBOT_POSITION_OFFSET_M_PHINEAS: str = "robot_position_offset_m_phineas"
PARAM_ROBOT_POSITION_OFFSET_M_FERB: str = "robot_position_offset_m_ferb"

# Manipulator relay node ROS parameter names
PARAM_PERRY_CALL_TIMEOUT_S: str = "perry_call_timeout_s"
PARAM_HEARTBEAT_TIMEOUT_S: str = "heartbeat_timeout_s"
PARAM_REPUBLISH_RATE_HZ: str = "republish_rate_hz"

# Robot link check node ROS parameter names
PARAM_ROBOT_NAMES: str = "robot_names"
PARAM_ROBOT_HOSTS: str = "robot_hosts"
PARAM_ROBOT_WEB_PORT: str = "robot_web_port"
PARAM_LISTEN_S: str = "listen_s"


def zone_request_service(robot: str) -> str:
    """Builds the per-robot zone request service name."""
    cleaned_robot = robot.strip("/")
    return f"/{cleaned_robot}{PERRY_ZONE_REQUEST_SERVICE}"


def zone_release_service(robot: str) -> str:
    """Builds the per-robot zone release service name."""
    cleaned_robot = robot.strip("/")
    return f"/{cleaned_robot}{PERRY_ZONE_RELEASE_SERVICE}"


def zone_occupant_topic(robot: str) -> str:
    """Builds the per-robot zone occupant topic name."""
    cleaned_robot = robot.strip("/")
    return f"/{cleaned_robot}{ZONE_OCCUPANT_TOPIC}"


def robot_plan_topic(robot: str) -> str:
    """Builds the per-robot plan topic name."""
    cleaned_robot = robot.strip("/")
    return f"/{cleaned_robot}{PLAN_TOPIC_SUFFIX}"


def robot_status_topic(robot: str) -> str:
    """Builds the per-robot status topic name."""
    cleaned_robot = robot.strip("/")
    return f"/{cleaned_robot}{STATUS_TOPIC_SUFFIX}"
