"""Unit tests for ArUco marker robot pose detection."""

import math

import cv2
import numpy as np
import pytest
from overhead_core.aruco_robots import (
    ArucoRobotConfig,
    detect_aruco_robots,
)
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    ARUCO_ID_FERB,
    ARUCO_ID_PHINEAS,
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_DETECTION_CONFIDENCE,
    RobotName,
)
from overhead_core.geometry import compute_homography

CANVAS_WIDTH_PX: int = 1280
CANVAS_HEIGHT_PX: int = 720
MARKER_SIZE_PX: int = 121
PATCH_SIZE_PX: int = 201
POSITION_TOLERANCE_M: float = 0.01
HEADING_TOLERANCE_RAD: float = math.radians(5.0)


def _make_canvas_homography() -> np.ndarray:
    corners_px = np.array(
        [
            [0.0, float(CANVAS_HEIGHT_PX)],
            [float(CANVAS_WIDTH_PX), float(CANVAS_HEIGHT_PX)],
            [float(CANVAS_WIDTH_PX), 0.0],
            [0.0, 0.0],
        ],
        dtype=np.float32,
    )
    return compute_homography(
        corners_px,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )


def _draw_aruco_on_canvas(
    canvas_bgr: np.ndarray,
    *,
    marker_id: int,
    center_px: tuple[int, int],
    rotation_deg: float = 0.0,
    dictionary_name: str = DEFAULT_ARUCO_DICTIONARY,
) -> None:
    dict_id = getattr(
        cv2.aruco,
        dictionary_name,
    )
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    marker_gray = cv2.aruco.generateImageMarker(
        dictionary,
        marker_id,
        MARKER_SIZE_PX,
    )
    marker_bgr = cv2.cvtColor(
        marker_gray,
        cv2.COLOR_GRAY2BGR,
    )

    patch = np.full(
        (PATCH_SIZE_PX, PATCH_SIZE_PX, 3),
        255,
        dtype=np.uint8,
    )
    pad = (PATCH_SIZE_PX - MARKER_SIZE_PX) // 2
    patch[pad : pad + MARKER_SIZE_PX, pad : pad + MARKER_SIZE_PX] = marker_bgr

    patch_center = (PATCH_SIZE_PX - 1) / 2.0
    rot_mat = cv2.getRotationMatrix2D(
        (patch_center, patch_center),
        rotation_deg,
        1.0,
    )
    rotated_patch = cv2.warpAffine(
        patch,
        rot_mat,
        (PATCH_SIZE_PX, PATCH_SIZE_PX),
        borderValue=(255, 255, 255),
    )

    cx_px, cy_px = center_px
    half = PATCH_SIZE_PX // 2
    canvas_bgr[cy_px - half : cy_px + half + 1, cx_px - half : cx_px + half + 1] = rotated_patch


def _wrapped_angle_diff_rad(
    angle_a: float,
    angle_b: float,
) -> float:
    diff = (angle_a - angle_b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(diff)


@pytest.mark.parametrize(
    ("rotation_deg", "expected_theta_rad"),
    [
        (0.0, math.pi / 2.0),
        (90.0, math.pi),
        (200.0, math.radians(-70.0)),
    ],
)
def test_should_detect_position_and_heading_when_marker_rotated(
    rotation_deg: float,
    expected_theta_rad: float,
) -> None:
    canvas = np.full(
        (CANVAS_HEIGHT_PX, CANVAS_WIDTH_PX, 3),
        255,
        dtype=np.uint8,
    )
    center_px = (400, 300)
    _draw_aruco_on_canvas(
        canvas,
        marker_id=ARUCO_ID_PHINEAS,
        center_px=center_px,
        rotation_deg=rotation_deg,
    )
    homography = _make_canvas_homography()
    config = ArucoRobotConfig(
        dictionary_name=DEFAULT_ARUCO_DICTIONARY,
        marker_ids={RobotName.PHINEAS.value: ARUCO_ID_PHINEAS},
        heading_offset_rad=0.0,
        marker_height_m=0.0,
    )
    expected_x_m = center_px[0] * ARENA_WIDTH_M / CANVAS_WIDTH_PX
    expected_y_m = (CANVAS_HEIGHT_PX - center_px[1]) * ARENA_HEIGHT_M / CANVAS_HEIGHT_PX

    poses = detect_aruco_robots(
        canvas,
        homography=homography,
        config=config,
        camera_center_world=DEFAULT_CAMERA_CENTER_WORLD,
        cam_height_m=1.0,
    )

    assert set(poses.keys()) == {RobotName.PHINEAS.value}
    phineas = poses[RobotName.PHINEAS.value]
    assert math.isclose(
        phineas.x,
        expected_x_m,
        abs_tol=POSITION_TOLERANCE_M,
    )
    assert math.isclose(
        phineas.y,
        expected_y_m,
        abs_tol=POSITION_TOLERANCE_M,
    )
    assert (
        _wrapped_angle_diff_rad(
            phineas.theta,
            expected_theta_rad,
        )
        <= HEADING_TOLERANCE_RAD
    )
    assert math.isclose(
        phineas.confidence,
        DEFAULT_DETECTION_CONFIDENCE,
    )


def test_should_ignore_marker_when_id_not_in_config() -> None:
    canvas = np.full(
        (CANVAS_HEIGHT_PX, CANVAS_WIDTH_PX, 3),
        255,
        dtype=np.uint8,
    )
    _draw_aruco_on_canvas(
        canvas,
        marker_id=7,
        center_px=(640, 360),
    )
    homography = _make_canvas_homography()
    config = ArucoRobotConfig(
        dictionary_name=DEFAULT_ARUCO_DICTIONARY,
        marker_ids={RobotName.PHINEAS.value: ARUCO_ID_PHINEAS},
        heading_offset_rad=0.0,
        marker_height_m=0.0,
    )

    poses = detect_aruco_robots(
        canvas,
        homography=homography,
        config=config,
    )

    assert poses == {}


def test_should_detect_two_robots_when_two_configured_markers_present() -> None:
    canvas = np.full(
        (CANVAS_HEIGHT_PX, CANVAS_WIDTH_PX, 3),
        255,
        dtype=np.uint8,
    )
    phineas_px = (360, 280)
    ferb_px = (920, 440)
    _draw_aruco_on_canvas(
        canvas,
        marker_id=ARUCO_ID_PHINEAS,
        center_px=phineas_px,
        rotation_deg=0.0,
    )
    _draw_aruco_on_canvas(
        canvas,
        marker_id=ARUCO_ID_FERB,
        center_px=ferb_px,
        rotation_deg=90.0,
    )
    homography = _make_canvas_homography()
    config = ArucoRobotConfig(
        dictionary_name=DEFAULT_ARUCO_DICTIONARY,
        marker_ids={
            RobotName.PHINEAS.value: ARUCO_ID_PHINEAS,
            RobotName.FERB.value: ARUCO_ID_FERB,
        },
        heading_offset_rad=0.0,
        marker_height_m=0.0,
    )

    poses = detect_aruco_robots(
        canvas,
        homography=homography,
        config=config,
    )

    assert set(poses.keys()) == {
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    }
    assert math.isclose(
        poses[RobotName.PHINEAS.value].x,
        phineas_px[0] * ARENA_WIDTH_M / CANVAS_WIDTH_PX,
        abs_tol=POSITION_TOLERANCE_M,
    )
    assert math.isclose(
        poses[RobotName.FERB.value].x,
        ferb_px[0] * ARENA_WIDTH_M / CANVAS_WIDTH_PX,
        abs_tol=POSITION_TOLERANCE_M,
    )


def test_should_add_heading_offset_when_configured() -> None:
    canvas = np.full(
        (CANVAS_HEIGHT_PX, CANVAS_WIDTH_PX, 3),
        255,
        dtype=np.uint8,
    )
    _draw_aruco_on_canvas(
        canvas,
        marker_id=ARUCO_ID_PHINEAS,
        center_px=(640, 360),
        rotation_deg=0.0,
    )
    homography = _make_canvas_homography()
    config = ArucoRobotConfig(
        dictionary_name=DEFAULT_ARUCO_DICTIONARY,
        marker_ids={RobotName.PHINEAS.value: ARUCO_ID_PHINEAS},
        heading_offset_rad=-math.pi / 2.0,
        marker_height_m=0.0,
    )

    poses = detect_aruco_robots(
        canvas,
        homography=homography,
        config=config,
    )

    phineas = poses[RobotName.PHINEAS.value]
    assert (
        _wrapped_angle_diff_rad(
            phineas.theta,
            0.0,
        )
        <= HEADING_TOLERANCE_RAD
    )


def test_should_raise_value_error_when_dictionary_name_unknown() -> None:
    canvas = np.full(
        (CANVAS_HEIGHT_PX, CANVAS_WIDTH_PX, 3),
        255,
        dtype=np.uint8,
    )
    homography = _make_canvas_homography()
    config = ArucoRobotConfig(
        dictionary_name="DICT_DOES_NOT_EXIST",
        marker_ids={RobotName.PHINEAS.value: ARUCO_ID_PHINEAS},
        heading_offset_rad=0.0,
        marker_height_m=0.0,
    )

    with pytest.raises(ValueError):
        detect_aruco_robots(
            canvas,
            homography=homography,
            config=config,
        )
