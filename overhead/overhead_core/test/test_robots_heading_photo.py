"""Regression tests for robot headings and phantom-cube exclusion on real arena photos."""

import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CAM_HEIGHT_M,
    CUBE_HEIGHT_M,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_ROBOT_HSV_RANGES,
    DEFAULT_ROBOT_MIN_AREA_PX,
    ROBOT_HULL_DILATION_PX,
    CubeColor,
    RobotName,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    world_to_image,
    wrap_angle_rad,
)
from overhead_core.viewer_pipeline import process_frame
from overhead_core.world_frame import WorldFrameConfig
from overhead_core.world_pipeline import WorldPipeline

FIXTURES_DIR: Path = Path(__file__).parent / "fixtures"
FERB_EAST_FIXTURE_PATH: Path = FIXTURES_DIR / "arena_ferb_east_2026-11-24.jpg"
TWO_ROBOTS_FIXTURE_PATH: Path = FIXTURES_DIR / "arena_two_robots_2026-11-24.jpg"

FERB_HEADING_TOLERANCE_RAD: float = math.radians(15.0)
PHINEAS_HEADING_TOLERANCE_RAD: float = math.radians(20.0)
MAX_CUBE_CENTER_ERROR_PX: float = 20.0
CUBE_WORLD_MATCH_TOLERANCE_M: float = 1e-6
ROBOT_POSE_MATCH_TOLERANCE_M: float = 1e-6
ROBOT_POSE_MATCH_TOLERANCE_RAD: float = 1e-6

EXPECTED_FERB_EAST_CUBE_COUNTS: dict[str, int] = {
    CubeColor.RED.value: 4,
    CubeColor.YELLOW.value: 6,
    CubeColor.BLUE.value: 3,
}
EXPECTED_FERB_EAST_RED_CENTERS_PX: tuple[tuple[float, float], ...] = (
    (678.0, 170.0),
    (464.0, 625.0),
    (718.0, 648.0),
    (1155.0, 457.0),
)


def _load_bgr_image(image_path: Path) -> np.ndarray:
    """Loads a BGR image from a path that may contain non-ASCII characters."""
    raw_bytes = np.fromfile(
        str(image_path),
        dtype=np.uint8,
    )
    frame = cv2.imdecode(
        raw_bytes,
        cv2.IMREAD_COLOR,
    )
    assert frame is not None
    return frame


def _calibrate_homography(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Detects arena corners from the photo and computes the floor homography."""
    corners = detect_arena_corners(frame_bgr)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    return homography, corners


def _world_point_to_image_px(
    x_m: float,
    y_m: float,
    *,
    homography: np.ndarray,
    object_height_m: float,
) -> tuple[float, float]:
    """Inverts parallax and projects a world floor coordinate back to image pixels."""
    scale = (CAM_HEIGHT_M - object_height_m) / CAM_HEIGHT_M
    center_x_m, center_y_m = DEFAULT_CAMERA_CENTER_WORLD
    seen_x_m = center_x_m + (x_m - center_x_m) / scale
    seen_y_m = center_y_m + (y_m - center_y_m) / scale
    uv_px = world_to_image(
        homography,
        [[seen_x_m, seen_y_m]],
    )
    return float(uv_px[0, 0]), float(uv_px[0, 1])


def _extract_robot_hull(
    frame_bgr: np.ndarray,
    robot_name: str,
) -> np.ndarray:
    """Extracts the convex hull of the largest blob matching a robot's HSV range."""
    hsv_frame = cv2.cvtColor(
        frame_bgr,
        cv2.COLOR_BGR2HSV,
    )
    mask = np.zeros(
        hsv_frame.shape[:2],
        dtype=np.uint8,
    )
    for lower_b, upper_b in DEFAULT_ROBOT_HSV_RANGES[robot_name]:
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

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    assert contours
    largest = max(
        contours,
        key=cv2.contourArea,
    )
    hull = cv2.convexHull(largest)
    assert cv2.contourArea(hull) >= DEFAULT_ROBOT_MIN_AREA_PX
    return hull


def test_should_report_ferb_facing_east_when_ferb_east_photo_processed() -> None:
    frame = _load_bgr_image(FERB_EAST_FIXTURE_PATH)
    homography, _corners = _calibrate_homography(frame)
    pipeline = WorldPipeline(WorldFrameConfig())

    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )

    assert RobotName.FERB.value in detection.seen_robots
    ferb_pose = detection.seen_robots[RobotName.FERB.value]
    heading_error_rad = abs(wrap_angle_rad(ferb_pose.theta - 0.0))
    assert heading_error_rad <= FERB_HEADING_TOLERANCE_RAD


def test_should_report_phineas_facing_east_when_two_robots_photo_processed() -> None:
    frame = _load_bgr_image(TWO_ROBOTS_FIXTURE_PATH)
    homography, _corners = _calibrate_homography(frame)
    pipeline = WorldPipeline(WorldFrameConfig())

    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )

    assert RobotName.PHINEAS.value in detection.seen_robots
    phineas_pose = detection.seen_robots[RobotName.PHINEAS.value]
    heading_error_rad = abs(wrap_angle_rad(phineas_pose.theta - 0.0))
    assert heading_error_rad <= PHINEAS_HEADING_TOLERANCE_RAD


def test_should_exclude_phantom_cubes_and_match_viewer_when_ferb_east_photo_processed() -> None:
    frame = _load_bgr_image(FERB_EAST_FIXTURE_PATH)
    homography, corners = _calibrate_homography(frame)
    ferb_hull = _extract_robot_hull(
        frame,
        RobotName.FERB.value,
    )
    pipeline = WorldPipeline(WorldFrameConfig())

    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )
    viewer_output = process_frame(
        frame,
        homography,
        corners_px=corners,
    )

    world_cube_pixels = [
        (
            color,
            *_world_point_to_image_px(
                x_m,
                y_m,
                homography=homography,
                object_height_m=CUBE_HEIGHT_M,
            ),
        )
        for _track_id, color, x_m, y_m, _conf in detection.tracked_cubes
    ]
    for _color, cube_u_px, cube_v_px in world_cube_pixels:
        hull_dist_px = cv2.pointPolygonTest(
            ferb_hull,
            (cube_u_px, cube_v_px),
            True,
        )
        assert hull_dist_px < -float(ROBOT_HULL_DILATION_PX)

    assert Counter(cube[1] for cube in detection.tracked_cubes) == EXPECTED_FERB_EAST_CUBE_COUNTS

    red_pixels = [
        (u_px, v_px) for color, u_px, v_px in world_cube_pixels if color == CubeColor.RED.value
    ]
    assert len(red_pixels) == EXPECTED_FERB_EAST_CUBE_COUNTS[CubeColor.RED.value]
    for expected_u_px, expected_v_px in EXPECTED_FERB_EAST_RED_CENTERS_PX:
        min_error_px = min(
            math.hypot(
                cube_u_px - expected_u_px,
                cube_v_px - expected_v_px,
            )
            for cube_u_px, cube_v_px in red_pixels
        )
        assert min_error_px <= MAX_CUBE_CENTER_ERROR_PX

    assert Counter(cube.color for cube in viewer_output.cubes) == EXPECTED_FERB_EAST_CUBE_COUNTS
    world_sorted = sorted(
        (color, x_m, y_m) for _cid, color, x_m, y_m, _conf in detection.tracked_cubes
    )
    viewer_sorted = sorted((cube.color, cube.x_m, cube.y_m) for cube in viewer_output.cubes)
    assert len(viewer_sorted) == len(world_sorted)
    for (v_color, v_x_m, v_y_m), (w_color, w_x_m, w_y_m) in zip(
        viewer_sorted,
        world_sorted,
        strict=True,
    ):
        assert v_color == w_color
        assert math.isclose(
            v_x_m,
            w_x_m,
            abs_tol=CUBE_WORLD_MATCH_TOLERANCE_M,
        )
        assert math.isclose(
            v_y_m,
            w_y_m,
            abs_tol=CUBE_WORLD_MATCH_TOLERANCE_M,
        )

    viewer_robots = {robot.name: robot for robot in viewer_output.robots}
    assert RobotName.FERB.value in viewer_robots
    world_ferb = detection.seen_robots[RobotName.FERB.value]
    viewer_ferb = viewer_robots[RobotName.FERB.value]
    assert math.isclose(
        viewer_ferb.x_m,
        world_ferb.x,
        abs_tol=ROBOT_POSE_MATCH_TOLERANCE_M,
    )
    assert math.isclose(
        viewer_ferb.y_m,
        world_ferb.y,
        abs_tol=ROBOT_POSE_MATCH_TOLERANCE_M,
    )
    assert (
        abs(wrap_angle_rad(viewer_ferb.theta_rad - world_ferb.theta))
        <= ROBOT_POSE_MATCH_TOLERANCE_RAD
    )
