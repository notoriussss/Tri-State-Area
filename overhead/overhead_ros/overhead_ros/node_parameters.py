"""ROS 2 parameter declaration and typed loading for overhead_node."""

from dataclasses import dataclass
from typing import Any

from overhead_core.aruco_robots import ArucoRobotConfig
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CALIBRATION_RETRY_S,
    CAM_HEIGHT_M,
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_ARUCO_HEADING_OFFSET_RAD,
    DEFAULT_ARUCO_IDS,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_CAPTURE_HEIGHT_PX,
    DEFAULT_CAPTURE_WIDTH_PX,
    DEFAULT_CUBE_HSV_RANGES,
    DEFAULT_IMAGE_RATE_HZ,
    DEFAULT_MIN_AREA_PX,
    DEFAULT_RATE_HZ,
    DEFAULT_ROBOT_HSV_RANGES,
    DEFAULT_ROBOT_NAMES,
    DEFAULT_VIDEO_SOURCE,
    DEFAULT_ZONE_RECT,
    HSV_CHANNEL_COUNT,
    MAX_CUBE_AREA_PX,
    NEAR_RED_DISTANCE_M,
    CubeColor,
    RobotDetector,
    RobotName,
)
from overhead_core.world_frame import WorldFrameConfig
from rclpy.node import Node

from overhead_ros.ros_names import (
    PARAM_ARENA_HEIGHT_M,
    PARAM_ARENA_WIDTH_M,
    PARAM_ARUCO_DICTIONARY,
    PARAM_ARUCO_HEADING_OFFSET_RAD,
    PARAM_ARUCO_IDS,
    PARAM_CALIBRATION_RETRY_S,
    PARAM_CAM_HEIGHT_M,
    PARAM_CAMERA_CENTER_WORLD,
    PARAM_CAMERA_INDEX,
    PARAM_HEIGHT,
    PARAM_HSV_BLUE,
    PARAM_HSV_RED_1,
    PARAM_HSV_RED_2,
    PARAM_HSV_ROBOT_FERB,
    PARAM_HSV_ROBOT_PHINEAS,
    PARAM_HSV_YELLOW,
    PARAM_IMAGE_RATE_HZ,
    PARAM_MAX_AREA_PX,
    PARAM_MIN_AREA_PX,
    PARAM_NEAR_RED_M,
    PARAM_RATE_HZ,
    PARAM_ROBOT_DETECTOR,
    PARAM_ROBOTS,
    PARAM_VIDEO_SOURCE,
    PARAM_WIDTH,
    PARAM_ZONE_RECT,
)


@dataclass(frozen=True)
class OverheadNodeParameters:
    """Typed ROS 2 parameter values and derived core configuration for OverheadNode."""

    camera_index: int
    video_source: str
    width_px: int
    height_px: int
    rate_hz: float
    calibration_retry_s: float
    arena_width_m: float
    arena_height_m: float
    zone_rect: tuple[float, float, float, float]
    robots: tuple[str, ...]
    image_rate_hz: float
    near_red_m: float
    world_config: WorldFrameConfig


def _hsv_param_default(
    bounds: tuple[tuple[int, int, int], tuple[int, int, int]],
) -> list[int]:
    """Flattens an HSV (lower, upper) pair into [h, s, v, h, s, v].

    ROS 2 parameters cannot hold nested arrays, so the pair is stored flat.
    """
    return [
        *bounds[0],
        *bounds[1],
    ]


def _read_hsv_pair(
    node: Node,
    param_name: str,
) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Reads a flat [h, s, v, h, s, v] parameter as a pair of integer tuples."""
    raw = [int(v) for v in node.get_parameter(param_name).value]
    return (
        tuple(raw[:HSV_CHANNEL_COUNT]),
        tuple(raw[HSV_CHANNEL_COUNT:]),
    )


def _build_aruco_config(
    node: Node,
    robots: tuple[str, ...],
) -> ArucoRobotConfig:
    """Builds ArucoRobotConfig from declared ArUco ROS parameters."""
    aruco_dictionary = str(node.get_parameter(PARAM_ARUCO_DICTIONARY).value)
    aruco_ids = [int(marker_id) for marker_id in node.get_parameter(PARAM_ARUCO_IDS).value]
    aruco_heading_offset_rad = float(
        node.get_parameter(PARAM_ARUCO_HEADING_OFFSET_RAD).value,
    )
    return ArucoRobotConfig(
        dictionary_name=aruco_dictionary,
        marker_ids=dict(
            zip(
                robots,
                aruco_ids,
                strict=False,
            )
        ),
        heading_offset_rad=aruco_heading_offset_rad,
    )


def _build_world_config(
    node: Node,
    *,
    robots: tuple[str, ...],
    arena_width_m: float,
    arena_height_m: float,
) -> WorldFrameConfig:
    """Builds the pure WorldFrameConfig from declared vision and geometry parameters."""
    cam_height_m = float(node.get_parameter(PARAM_CAM_HEIGHT_M).value)
    center_raw = node.get_parameter(PARAM_CAMERA_CENTER_WORLD).value
    camera_center_world = (
        float(center_raw[0]),
        float(center_raw[1]),
    )
    min_area_px = int(node.get_parameter(PARAM_MIN_AREA_PX).value)
    max_area_px = int(node.get_parameter(PARAM_MAX_AREA_PX).value)
    robot_detector = RobotDetector(
        str(node.get_parameter(PARAM_ROBOT_DETECTOR).value),
    )
    aruco_config = _build_aruco_config(
        node,
        robots,
    )

    hsv_ranges: dict[str, list[tuple[Any, Any]]] = {
        CubeColor.BLUE.value: [_read_hsv_pair(node, PARAM_HSV_BLUE)],
        CubeColor.YELLOW.value: [_read_hsv_pair(node, PARAM_HSV_YELLOW)],
        CubeColor.RED.value: [
            _read_hsv_pair(node, PARAM_HSV_RED_1),
            _read_hsv_pair(node, PARAM_HSV_RED_2),
        ],
    }
    robot_hsv_ranges: dict[str, list[tuple[Any, Any]]] = {
        RobotName.PHINEAS.value: [_read_hsv_pair(node, PARAM_HSV_ROBOT_PHINEAS)],
        RobotName.FERB.value: [_read_hsv_pair(node, PARAM_HSV_ROBOT_FERB)],
    }

    return WorldFrameConfig(
        hsv_ranges=hsv_ranges,
        robot_hsv_ranges=robot_hsv_ranges,
        min_area_px=min_area_px,
        camera_center_world=camera_center_world,
        cam_height_m=cam_height_m,
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
        robot_names=robots,
        max_area_px=max_area_px,
        robot_detector=robot_detector,
        aruco=aruco_config,
    )


def declare_overhead_parameters(node: Node) -> None:
    """Declares all overhead_node ROS parameters using package constants as defaults.

    Args:
        node: Target ROS 2 node on which parameters are declared.
    """
    parameter_defaults: tuple[tuple[str, Any], ...] = (
        (PARAM_CAMERA_INDEX, DEFAULT_CAMERA_INDEX),
        (PARAM_VIDEO_SOURCE, DEFAULT_VIDEO_SOURCE),
        (PARAM_WIDTH, DEFAULT_CAPTURE_WIDTH_PX),
        (PARAM_HEIGHT, DEFAULT_CAPTURE_HEIGHT_PX),
        (PARAM_RATE_HZ, DEFAULT_RATE_HZ),
        (PARAM_CALIBRATION_RETRY_S, CALIBRATION_RETRY_S),
        (PARAM_ARENA_WIDTH_M, ARENA_WIDTH_M),
        (PARAM_ARENA_HEIGHT_M, ARENA_HEIGHT_M),
        (PARAM_CAM_HEIGHT_M, CAM_HEIGHT_M),
        (PARAM_CAMERA_CENTER_WORLD, list(DEFAULT_CAMERA_CENTER_WORLD)),
        (PARAM_ZONE_RECT, list(DEFAULT_ZONE_RECT)),
        (PARAM_ROBOTS, list(DEFAULT_ROBOT_NAMES)),
        (PARAM_IMAGE_RATE_HZ, DEFAULT_IMAGE_RATE_HZ),
        (PARAM_MIN_AREA_PX, DEFAULT_MIN_AREA_PX),
        (PARAM_MAX_AREA_PX, MAX_CUBE_AREA_PX),
        (PARAM_NEAR_RED_M, NEAR_RED_DISTANCE_M),
        (PARAM_ROBOT_DETECTOR, RobotDetector.TRIANGLE.value),
        (PARAM_ARUCO_DICTIONARY, DEFAULT_ARUCO_DICTIONARY),
        (PARAM_ARUCO_IDS, list(DEFAULT_ARUCO_IDS)),
        (PARAM_ARUCO_HEADING_OFFSET_RAD, DEFAULT_ARUCO_HEADING_OFFSET_RAD),
        (
            PARAM_HSV_BLUE,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.BLUE.value][0]),
        ),
        (
            PARAM_HSV_YELLOW,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.YELLOW.value][0]),
        ),
        (
            PARAM_HSV_RED_1,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.RED.value][0]),
        ),
        (
            PARAM_HSV_RED_2,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.RED.value][1]),
        ),
        (
            PARAM_HSV_ROBOT_PHINEAS,
            _hsv_param_default(DEFAULT_ROBOT_HSV_RANGES[RobotName.PHINEAS.value][0]),
        ),
        (
            PARAM_HSV_ROBOT_FERB,
            _hsv_param_default(DEFAULT_ROBOT_HSV_RANGES[RobotName.FERB.value][0]),
        ),
    )
    for param_name, default_value in parameter_defaults:
        node.declare_parameter(
            param_name,
            default_value,
        )


def load_overhead_parameters(node: Node) -> OverheadNodeParameters:
    """Loads declared ROS parameter values into an immutable OverheadNodeParameters.

    Args:
        node: ROS 2 node whose parameters have already been declared.

    Returns:
        Frozen OverheadNodeParameters instance including the derived WorldFrameConfig.
    """
    arena_width_m = float(node.get_parameter(PARAM_ARENA_WIDTH_M).value)
    arena_height_m = float(node.get_parameter(PARAM_ARENA_HEIGHT_M).value)
    zone_raw = node.get_parameter(PARAM_ZONE_RECT).value
    zone_rect = (
        float(zone_raw[0]),
        float(zone_raw[1]),
        float(zone_raw[2]),
        float(zone_raw[3]),
    )
    robots = tuple(str(name) for name in node.get_parameter(PARAM_ROBOTS).value)
    world_config = _build_world_config(
        node,
        robots=robots,
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
    )

    return OverheadNodeParameters(
        camera_index=int(node.get_parameter(PARAM_CAMERA_INDEX).value),
        video_source=str(node.get_parameter(PARAM_VIDEO_SOURCE).value),
        width_px=int(node.get_parameter(PARAM_WIDTH).value),
        height_px=int(node.get_parameter(PARAM_HEIGHT).value),
        rate_hz=float(node.get_parameter(PARAM_RATE_HZ).value),
        calibration_retry_s=float(node.get_parameter(PARAM_CALIBRATION_RETRY_S).value),
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
        zone_rect=zone_rect,
        robots=robots,
        image_rate_hz=float(node.get_parameter(PARAM_IMAGE_RATE_HZ).value),
        near_red_m=float(node.get_parameter(PARAM_NEAR_RED_M).value),
        world_config=world_config,
    )
