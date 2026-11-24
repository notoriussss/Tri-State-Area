"""Pure world-frame vision pipeline extracting tracked cubes and robot poses."""

import math
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from overhead_core.aruco_robots import (
    ArucoRobotConfig,
    detect_aruco_robots,
)
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CAM_HEIGHT_M,
    CUBE_HEIGHT_M,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_CUBE_HSV_RANGES,
    DEFAULT_DETECTION_CONFIDENCE,
    DEFAULT_MIN_AREA_PX,
    DEFAULT_ROBOT_HEADING_OFFSETS_RAD,
    DEFAULT_ROBOT_HSV_RANGES,
    DEFAULT_ROBOT_MIN_AREA_PX,
    DEFAULT_ROBOT_NAMES,
    DEFAULT_ROBOT_POSITION_OFFSETS_M,
    ERR_UNKNOWN_ROBOT_DETECTOR_FMT,
    HEADING_STEP_PX,
    MAX_CUBE_AREA_PX,
    ROBOT_HULL_DILATION_PX,
    ROBOT_MARKER_HEIGHT_M,
    RobotDetector,
)
from overhead_core.cubes import detect_cubes
from overhead_core.geometry import (
    image_to_world,
    wrap_angle_rad,
)
from overhead_core.parallax import correct_parallax
from overhead_core.tracker import CubeTracker
from overhead_core.triangle import heading_to_world, triangle_pose
from overhead_core.types import RobotPose

HsvBoundsMap = dict[str, list[tuple[Any, Any]]]


@dataclass(frozen=True)
class WorldFrameConfig:
    """Configuration for detecting cubes and robots in world floor coordinates."""

    hsv_ranges: HsvBoundsMap = field(
        default_factory=lambda: dict(DEFAULT_CUBE_HSV_RANGES),
    )
    robot_hsv_ranges: HsvBoundsMap = field(
        default_factory=lambda: dict(DEFAULT_ROBOT_HSV_RANGES),
    )
    min_area_px: int = DEFAULT_MIN_AREA_PX
    robot_min_area_px: int = DEFAULT_ROBOT_MIN_AREA_PX
    camera_center_world: tuple[float, float] = DEFAULT_CAMERA_CENTER_WORLD
    cam_height_m: float = CAM_HEIGHT_M
    arena_width_m: float = ARENA_WIDTH_M
    arena_height_m: float = ARENA_HEIGHT_M
    robot_names: tuple[str, ...] = DEFAULT_ROBOT_NAMES
    max_area_px: int = MAX_CUBE_AREA_PX
    robot_detector: RobotDetector = RobotDetector.TRIANGLE
    aruco: ArucoRobotConfig | None = None
    robot_heading_offsets_rad: dict[str, float] = field(
        default_factory=lambda: dict(DEFAULT_ROBOT_HEADING_OFFSETS_RAD),
    )
    robot_position_offsets_m: dict[str, tuple[float, float]] = field(
        default_factory=lambda: dict(DEFAULT_ROBOT_POSITION_OFFSETS_M),
    )


def _build_hsv_mask(
    hsv_frame: np.ndarray,
    ranges: list[tuple[Any, Any]],
) -> np.ndarray:
    """Builds a binary mask combining one or more HSV (lower, upper) bounds."""
    mask = np.zeros(
        hsv_frame.shape[:2],
        dtype=np.uint8,
    )
    for lower_b, upper_b in ranges:
        sub_mask = cv2.inRange(
            hsv_frame,
            np.asarray(
                lower_b,
                dtype=np.uint8,
            ),
            np.asarray(
                upper_b,
                dtype=np.uint8,
            ),
        )
        mask = cv2.bitwise_or(
            mask,
            sub_mask,
        )

    return mask


def _detect_robot_triangle_hulls(
    frame: np.ndarray,
    config: WorldFrameConfig,
) -> list[np.ndarray]:
    """Returns convex hulls of detected robot triangles meeting robot_min_area_px."""
    if frame is None or frame.size == 0:
        return []

    hsv_frame = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV,
    )
    hulls: list[np.ndarray] = []
    for robot_name in config.robot_names:
        ranges = config.robot_hsv_ranges.get(robot_name)
        if not ranges:
            continue

        robot_mask = _build_hsv_mask(
            hsv_frame,
            ranges,
        )
        contours, _ = cv2.findContours(
            robot_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            continue

        largest = max(
            contours,
            key=cv2.contourArea,
        )
        if cv2.contourArea(largest) < config.robot_min_area_px:
            continue

        hulls.append(cv2.convexHull(largest))

    return hulls


def _is_inside_any_hull(
    u_px: float,
    v_px: float,
    hulls: list[np.ndarray],
    *,
    dilation_px: int,
) -> bool:
    """Returns True when (u_px, v_px) lies within dilation_px of any convex hull."""
    margin = -float(dilation_px)
    return any(cv2.pointPolygonTest(hull, (u_px, v_px), True) >= margin for hull in hulls)


def process_cubes(
    frame: np.ndarray,
    *,
    homography: np.ndarray,
    tracker: CubeTracker,
    now_s: float,
    config: WorldFrameConfig | None = None,
) -> list[tuple[int, str, float, float, float]]:
    """Detects cubes, projects them to parallax-corrected world meters, and updates tracks.

    Args:
        frame: Input BGR camera frame.
        homography: 3x3 homography mapping image pixels to world arena meters.
        tracker: Stateful CubeTracker maintaining persistent cube track IDs.
        now_s: Current monotonic timestamp in seconds.
        config: Optional vision and geometry configuration.

    Returns:
        List of active tracked cubes as (track_id, color, x_m, y_m, confidence).
    """
    cfg = config if config is not None else WorldFrameConfig()
    center_arr = np.asarray(
        cfg.camera_center_world,
        dtype=np.float64,
    )
    raw_dets = detect_cubes(
        frame,
        None,
        cfg.hsv_ranges,
        min_area_px=cfg.min_area_px,
        max_area_px=cfg.max_area_px,
    )
    robot_hulls = (
        _detect_robot_triangle_hulls(
            frame,
            cfg,
        )
        if cfg.robot_detector == RobotDetector.TRIANGLE
        else []
    )

    floor_dets: list[tuple[str, float, float, float]] = []
    for color, u_px, v_px, _area in raw_dets:
        # Discard spurious color blobs on the robot roof marker (cubes cannot sit under it).
        if _is_inside_any_hull(
            u_px,
            v_px,
            robot_hulls,
            dilation_px=ROBOT_HULL_DILATION_PX,
        ):
            continue

        seen_world = image_to_world(
            homography,
            [[u_px, v_px]],
        )
        corrected_floor = correct_parallax(
            seen_world,
            center_arr,
            object_height_m=CUBE_HEIGHT_M,
            cam_height_m=cfg.cam_height_m,
        )
        floor_dets.append(
            (
                color,
                float(corrected_floor[0, 0]),
                float(corrected_floor[0, 1]),
                DEFAULT_DETECTION_CONFIDENCE,
            )
        )

    return tracker.update(
        floor_dets,
        now_s,
    )


def _apply_robot_pose_offset(
    x_m: float,
    y_m: float,
    theta_rad: float,
    *,
    heading_offset_rad: float,
    position_offset_m: tuple[float, float],
) -> tuple[float, float, float]:
    """Applies per-robot heading and (forward_m, left_m) calibration offsets."""
    final_theta = wrap_angle_rad(theta_rad + heading_offset_rad)
    forward_m, left_m = position_offset_m
    cos_theta = math.cos(final_theta)
    sin_theta = math.sin(final_theta)
    world_x = x_m + (forward_m * cos_theta - left_m * sin_theta)
    world_y = y_m + (forward_m * sin_theta + left_m * cos_theta)
    return world_x, world_y, final_theta


def _detect_triangle_robot_poses(
    frame: np.ndarray,
    *,
    homography: np.ndarray,
    config: WorldFrameConfig,
) -> dict[str, RobotPose]:
    """Detects robot triangle markers and returns parallax-corrected world poses."""
    center_arr = np.asarray(
        config.camera_center_world,
        dtype=np.float64,
    )
    hsv_frame = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV,
    )

    poses: dict[str, RobotPose] = {}
    for robot_name in config.robot_names:
        ranges = config.robot_hsv_ranges.get(robot_name)
        if not ranges:
            continue

        robot_mask = _build_hsv_mask(
            hsv_frame,
            ranges,
        )
        pose = triangle_pose(
            robot_mask,
            min_area_px=config.robot_min_area_px,
        )
        if pose is None:
            continue

        u_px, v_px, theta_img = pose
        seen_world = image_to_world(
            homography,
            [[u_px, v_px]],
        )
        corrected_floor = correct_parallax(
            seen_world,
            center_arr,
            object_height_m=ROBOT_MARKER_HEIGHT_M,
            cam_height_m=config.cam_height_m,
        )
        theta_world = heading_to_world(
            homography,
            u_px,
            v_px,
            theta_img=theta_img,
            step_px=HEADING_STEP_PX,
        )
        world_x, world_y, final_theta = _apply_robot_pose_offset(
            float(corrected_floor[0, 0]),
            float(corrected_floor[0, 1]),
            theta_world,
            heading_offset_rad=config.robot_heading_offsets_rad.get(
                robot_name,
                0.0,
            ),
            position_offset_m=config.robot_position_offsets_m.get(
                robot_name,
                (0.0, 0.0),
            ),
        )
        poses[robot_name] = RobotPose(
            name=robot_name,
            x=world_x,
            y=world_y,
            theta=final_theta,
            confidence=DEFAULT_DETECTION_CONFIDENCE,
        )

    return poses


def detect_robot_poses(
    frame: np.ndarray,
    *,
    homography: np.ndarray,
    config: WorldFrameConfig | None = None,
) -> dict[str, RobotPose]:
    """Detects robot markers using the configured backend and returns world poses.

    Args:
        frame: Input BGR camera frame.
        homography: 3x3 homography mapping image pixels to world arena meters.
        config: Optional vision and geometry configuration.

    Returns:
        Mapping from detected robot name to its RobotPose in world coordinates.

    Raises:
        ValueError: If config.robot_detector is not a recognized RobotDetector value.
    """
    if frame is None or frame.size == 0:
        return {}

    cfg = config if config is not None else WorldFrameConfig()
    if cfg.robot_detector == RobotDetector.ARUCO:
        return detect_aruco_robots(
            frame,
            homography=homography,
            config=cfg.aruco,
            camera_center_world=cfg.camera_center_world,
            cam_height_m=cfg.cam_height_m,
        )

    if cfg.robot_detector != RobotDetector.TRIANGLE:
        raise ValueError(
            ERR_UNKNOWN_ROBOT_DETECTOR_FMT.format(cfg.robot_detector),
        )

    return _detect_triangle_robot_poses(
        frame,
        homography=homography,
        config=cfg,
    )
