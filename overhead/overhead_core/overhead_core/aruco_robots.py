"""ArUco marker detection and pose estimation for collector robots."""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from overhead_core.constants import (
    ARUCO_DICT_PREFIX,
    ARUCO_ID_FERB,
    ARUCO_ID_PHINEAS,
    CAM_HEIGHT_M,
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_ARUCO_HEADING_OFFSET_RAD,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_DETECTION_CONFIDENCE,
    ERR_UNKNOWN_ARUCO_DICTIONARY_FMT,
    HEADING_STEP_PX,
    QUAD_VERTEX_COUNT,
    ROBOT_MARKER_HEIGHT_M,
    RobotName,
)
from overhead_core.geometry import (
    image_to_world,
    wrap_angle_rad,
)
from overhead_core.parallax import correct_parallax
from overhead_core.triangle import heading_to_world
from overhead_core.types import RobotPose


def _default_marker_ids() -> dict[str, int]:
    """Returns the default mapping from robot names to ArUco marker IDs."""
    return {
        RobotName.PHINEAS.value: ARUCO_ID_PHINEAS,
        RobotName.FERB.value: ARUCO_ID_FERB,
    }


@dataclass(frozen=True)
class ArucoRobotConfig:
    """Configuration for ArUco-based robot pose detection."""

    dictionary_name: str = DEFAULT_ARUCO_DICTIONARY
    marker_ids: dict[str, int] = field(default_factory=_default_marker_ids)
    heading_offset_rad: float = DEFAULT_ARUCO_HEADING_OFFSET_RAD
    marker_height_m: float = ROBOT_MARKER_HEIGHT_M


def _get_aruco_dictionary(dictionary_name: str) -> cv2.aruco.Dictionary:
    """Resolves a predefined OpenCV ArUco dictionary by name."""
    if not dictionary_name.startswith(ARUCO_DICT_PREFIX):
        raise ValueError(
            ERR_UNKNOWN_ARUCO_DICTIONARY_FMT.format(dictionary_name),
        )

    dict_id = getattr(
        cv2.aruco,
        dictionary_name,
        None,
    )
    if not isinstance(dict_id, int) or isinstance(dict_id, bool):
        raise ValueError(
            ERR_UNKNOWN_ARUCO_DICTIONARY_FMT.format(dictionary_name),
        )

    return cv2.aruco.getPredefinedDictionary(dict_id)


def _pose_from_marker_corners(
    robot_name: str,
    marker_corners: np.ndarray,
    *,
    homography: np.ndarray,
    config: ArucoRobotConfig,
    camera_center_arr: np.ndarray,
    cam_height_m: float,
) -> RobotPose:
    """Computes a parallax-corrected RobotPose from one marker's 4 corners."""
    pts = np.asarray(
        marker_corners,
        dtype=np.float64,
    ).reshape(
        QUAD_VERTEX_COUNT,
        2,
    )
    center_uv = np.mean(
        pts,
        axis=0,
    )
    u_px = float(center_uv[0])
    v_px = float(center_uv[1])

    seen_world = image_to_world(
        homography,
        [[u_px, v_px]],
    )
    corrected_floor = correct_parallax(
        seen_world,
        camera_center_arr,
        object_height_m=config.marker_height_m,
        cam_height_m=cam_height_m,
    )

    top_mid = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[3] + pts[2]) / 2.0
    theta_img = math.atan2(
        float(top_mid[1] - bottom_mid[1]),
        float(top_mid[0] - bottom_mid[0]),
    )
    theta_world = heading_to_world(
        homography,
        u_px,
        v_px,
        theta_img=theta_img,
        step_px=HEADING_STEP_PX,
    )
    return RobotPose(
        name=robot_name,
        x=float(corrected_floor[0, 0]),
        y=float(corrected_floor[0, 1]),
        theta=wrap_angle_rad(theta_world + config.heading_offset_rad),
        confidence=DEFAULT_DETECTION_CONFIDENCE,
    )


def detect_aruco_robots(
    frame_bgr: np.ndarray,
    *,
    homography: np.ndarray,
    config: ArucoRobotConfig | None = None,
    camera_center_world: tuple[float, float] = DEFAULT_CAMERA_CENTER_WORLD,
    cam_height_m: float = CAM_HEIGHT_M,
) -> dict[str, RobotPose]:
    """Detects ArUco robot markers and returns parallax-corrected world poses.

    Args:
        frame_bgr: Input BGR camera frame.
        homography: 3x3 homography mapping image pixels to world arena meters.
        config: Optional ArUco detection and marker-ID configuration.
        camera_center_world: Point on the arena floor directly below the camera, meters.
        cam_height_m: Camera height above the arena floor, meters.

    Returns:
        Mapping from configured robot name to its detected RobotPose in world coordinates.

    Raises:
        ValueError: If config.dictionary_name is not a known OpenCV ArUco dictionary.
    """
    cfg = config if config is not None else ArucoRobotConfig()
    dictionary = _get_aruco_dictionary(cfg.dictionary_name)

    is_empty_or_unconfigured = frame_bgr is None or frame_bgr.size == 0 or not cfg.marker_ids
    if is_empty_or_unconfigured:
        return {}

    id_to_robot = {marker_id: robot_name for robot_name, marker_id in cfg.marker_ids.items()}
    gray = cv2.cvtColor(
        frame_bgr,
        cv2.COLOR_BGR2GRAY,
    )
    detector = cv2.aruco.ArucoDetector(
        dictionary,
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return {}

    camera_center_arr = np.asarray(
        camera_center_world,
        dtype=np.float64,
    )
    poses: dict[str, RobotPose] = {}
    for marker_corners, marker_id_arr in zip(
        corners,
        ids,
        strict=True,
    ):
        robot_name = id_to_robot.get(int(np.asarray(marker_id_arr).ravel()[0]))
        if robot_name is None:
            continue

        poses[robot_name] = _pose_from_marker_corners(
            robot_name,
            marker_corners,
            homography=homography,
            config=cfg,
            camera_center_arr=camera_center_arr,
            cam_height_m=cam_height_m,
        )

    return poses
