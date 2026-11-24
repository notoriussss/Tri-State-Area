"""Regression tests for detecting both robots and cubes on the two-robot arena photo."""

import math
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
    ROBOT_MARKER_HEIGHT_M,
    CubeColor,
    RobotName,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    world_to_image,
)
from overhead_core.tracker import CubeTracker
from overhead_core.world_frame import (
    WorldFrameConfig,
    process_cubes,
)
from overhead_core.world_pipeline import WorldPipeline

FIXTURE_PATH: Path = Path(__file__).parent / "fixtures" / "arena_two_robots_2026-11-24.jpg"

EXPECTED_ROBOT_CENTROID_PX: dict[str, tuple[float, float]] = {
    RobotName.PHINEAS.value: (480.0, 360.0),
    RobotName.FERB.value: (245.0, 580.0),
}
EXPECTED_RED_CUBE_COUNT: int = 4
EXPECTED_RED_CUBE_CENTERS_PX: tuple[tuple[float, float], ...] = (
    (483.0, 106.0),
    (767.0, 88.0),
    (1061.0, 486.0),
    (460.0, 620.0),
)
MAX_CENTROID_ERROR_PX: float = 30.0
MAX_CUBE_CENTER_ERROR_PX: float = 15.0
ROBOT_MASK_RADIUS_PX: int = 80
FLOOR_FILL_BGR: tuple[int, int, int] = (180, 180, 180)


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


def _calibrate_homography(frame_bgr: np.ndarray) -> np.ndarray:
    """Detects arena corners from the photo and computes the floor homography."""
    corners = detect_arena_corners(frame_bgr)
    return compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )


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


def _build_hsv_mask(
    hsv_frame: np.ndarray,
    ranges: list[tuple[tuple[int, int, int], tuple[int, int, int]]],
) -> np.ndarray:
    """Builds a binary HSV mask combining one or more (lower, upper) bounds."""
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


def _robot_triangle_hulls(frame_bgr: np.ndarray) -> dict[str, np.ndarray]:
    """Extracts the convex hull of each detected robot triangle in image pixels."""
    hsv_frame = cv2.cvtColor(
        frame_bgr,
        cv2.COLOR_BGR2HSV,
    )
    hulls: dict[str, np.ndarray] = {}
    for robot_name, ranges in DEFAULT_ROBOT_HSV_RANGES.items():
        mask = _build_hsv_mask(
            hsv_frame,
            ranges,
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
        assert cv2.contourArea(largest) >= DEFAULT_ROBOT_MIN_AREA_PX
        hulls[robot_name] = cv2.convexHull(largest)
    return hulls


def _mask_out_robots(frame_bgr: np.ndarray) -> np.ndarray:
    """Returns a copy of the arena photo with both provisional robot triangles painted over."""
    masked = frame_bgr.copy()
    for center_u, center_v in EXPECTED_ROBOT_CENTROID_PX.values():
        cv2.circle(
            masked,
            (int(round(center_u)), int(round(center_v))),
            ROBOT_MASK_RADIUS_PX,
            FLOOR_FILL_BGR,
            -1,
        )
    return masked


def test_should_detect_both_robots_near_centroids_when_two_robots_photo_processed() -> None:
    frame = _load_bgr_image(FIXTURE_PATH)
    homography = _calibrate_homography(frame)
    zero_offset_config = WorldFrameConfig(
        robot_position_offsets_m={
            RobotName.PHINEAS.value: (0.0, 0.0),
            RobotName.FERB.value: (0.0, 0.0),
        },
    )
    pipeline = WorldPipeline(zero_offset_config)

    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )

    assert detection.is_all_seen is True
    assert set(detection.seen_robots.keys()) == {
        RobotName.PHINEAS.value,
        RobotName.FERB.value,
    }
    assert len(detection.robot_poses) == 2
    for robot_name, expected_uv_px in EXPECTED_ROBOT_CENTROID_PX.items():
        pose = detection.seen_robots[robot_name]
        detected_u_px, detected_v_px = _world_point_to_image_px(
            pose.x,
            pose.y,
            homography=homography,
            object_height_m=ROBOT_MARKER_HEIGHT_M,
        )
        error_px = math.hypot(
            detected_u_px - expected_uv_px[0],
            detected_v_px - expected_uv_px[1],
        )
        assert error_px <= MAX_CENTROID_ERROR_PX


def test_should_detect_four_red_cubes_and_exclude_robots_when_two_robots_present() -> None:
    frame = _load_bgr_image(FIXTURE_PATH)
    homography = _calibrate_homography(frame)
    config = WorldFrameConfig()
    pipeline = WorldPipeline(config)
    triangle_hulls = _robot_triangle_hulls(frame)
    frame_without_robots = _mask_out_robots(frame)

    detection = pipeline.detect_and_track(
        frame,
        homography=homography,
        now_s=1.0,
    )
    cubes_without_robots = process_cubes(
        frame_without_robots,
        homography=homography,
        tracker=CubeTracker(),
        now_s=1.0,
        config=config,
    )

    tracked_pixels = [
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
    for hull in triangle_hulls.values():
        for _color, cube_u_px, cube_v_px in tracked_pixels:
            hull_dist_px = cv2.pointPolygonTest(
                hull,
                (cube_u_px, cube_v_px),
                True,
            )
            assert hull_dist_px < -float(ROBOT_HULL_DILATION_PX)

    red_pixels_with_robots = [
        (u_px, v_px) for color, u_px, v_px in tracked_pixels if color == CubeColor.RED.value
    ]
    red_without_robots = [cube for cube in cubes_without_robots if cube[1] == CubeColor.RED.value]
    assert len(red_pixels_with_robots) == EXPECTED_RED_CUBE_COUNT
    assert len(red_without_robots) == EXPECTED_RED_CUBE_COUNT
    for expected_u_px, expected_v_px in EXPECTED_RED_CUBE_CENTERS_PX:
        min_error_px = min(
            math.hypot(
                cube_u_px - expected_u_px,
                cube_v_px - expected_v_px,
            )
            for cube_u_px, cube_v_px in red_pixels_with_robots
        )
        assert min_error_px <= MAX_CUBE_CENTER_ERROR_PX
