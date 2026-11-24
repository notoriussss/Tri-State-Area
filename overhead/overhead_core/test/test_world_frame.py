"""Integration tests for world_frame detection on the real arena photo and synthetic frames."""

import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from overhead_core.aruco_robots import ArucoRobotConfig
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    ARUCO_ID_PHINEAS,
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_DETECTION_CONFIDENCE,
    DEFAULT_ROBOT_MIN_AREA_PX,
    DEFAULT_ZONE_RECT,
    CubeColor,
    RobotDetector,
    RobotName,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    wrap_angle_rad,
)
from overhead_core.tracker import CubeTracker
from overhead_core.triangle import triangle_pose
from overhead_core.world_frame import (
    WorldFrameConfig,
    detect_robot_poses,
    process_cubes,
)


def _load_real_arena_photo() -> tuple[np.ndarray, np.ndarray]:
    fixture_path = Path(__file__).parent / "fixtures" / "arena_real_2026-11-24.jpg"
    raw_bytes = np.fromfile(
        str(fixture_path),
        dtype=np.uint8,
    )
    image = cv2.imdecode(
        raw_bytes,
        cv2.IMREAD_COLOR,
    )
    assert image is not None
    corners = detect_arena_corners(image)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    return image, homography


def test_should_detect_expected_cubes_and_positions_when_real_arena_photo_processed() -> None:
    image, homography = _load_real_arena_photo()
    tracker = CubeTracker()
    config = WorldFrameConfig()

    tracked = process_cubes(
        image,
        homography=homography,
        tracker=tracker,
        now_s=1.0,
        config=config,
    )

    color_counts = Counter(cube[1] for cube in tracked)
    assert color_counts == {
        CubeColor.RED.value: 5,
        CubeColor.YELLOW.value: 7,
        CubeColor.BLUE.value: 4,
    }

    inside_arena = [
        cube
        for cube in tracked
        if 0.0 <= cube[2] <= ARENA_WIDTH_M and 0.0 <= cube[3] <= ARENA_HEIGHT_M
    ]
    outside_arena = [cube for cube in tracked if cube[2] > ARENA_WIDTH_M]
    assert 14 <= len(inside_arena) <= 15
    assert all(0.0 <= c[2] <= ARENA_WIDTH_M for c in inside_arena)
    assert all(0.0 <= c[3] <= ARENA_HEIGHT_M for c in inside_arena)
    assert all(
        DEFAULT_ZONE_RECT[0] <= c[2] <= DEFAULT_ZONE_RECT[2]
        and DEFAULT_ZONE_RECT[1] <= c[3] <= DEFAULT_ZONE_RECT[3]
        for c in outside_arena
    )


def test_should_return_empty_dict_when_no_robot_marker_in_real_arena_photo() -> None:
    image, homography = _load_real_arena_photo()
    config = WorldFrameConfig()

    poses = detect_robot_poses(
        image,
        homography=homography,
        config=config,
    )

    assert poses == {}


def test_should_detect_robot_pose_when_synthetic_triangle_present() -> None:
    image = np.full(
        (600, 900, 3),
        230,
        dtype=np.uint8,
    )
    cv2.rectangle(
        image,
        (40, 30),
        (860, 570),
        (0, 0, 0),
        20,
    )
    triangle_pts = np.array(
        [
            [690, 300],
            [610, 265],
            [610, 335],
        ],
        dtype=np.int32,
    )
    cv2.fillPoly(
        image,
        [triangle_pts],
        (0, 0, 255),
    )
    corners = detect_arena_corners(image)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    config = WorldFrameConfig()

    poses = detect_robot_poses(
        image,
        homography=homography,
        config=config,
    )

    assert set(poses.keys()) == {RobotName.PHINEAS.value}
    phineas = poses[RobotName.PHINEAS.value]
    assert 0.0 < phineas.x < ARENA_WIDTH_M
    assert 0.0 < phineas.y < ARENA_HEIGHT_M
    assert math.isclose(
        phineas.theta,
        0.0,
        abs_tol=0.15,
    )
    assert math.isclose(
        phineas.confidence,
        DEFAULT_DETECTION_CONFIDENCE,
    )


def _make_two_triangle_arena_image(
    *,
    phineas_pts: np.ndarray,
    ferb_pts: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    image = np.full(
        (600, 900, 3),
        230,
        dtype=np.uint8,
    )
    cv2.rectangle(
        image,
        (40, 30),
        (860, 570),
        (0, 0, 0),
        20,
    )
    cv2.fillPoly(
        image,
        [phineas_pts],
        (0, 0, 255),
    )
    if ferb_pts is not None:
        cv2.fillPoly(
            image,
            [ferb_pts],
            (0, 75, 255),
        )
    corners = detect_arena_corners(image)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    return image, homography


def test_should_apply_heading_offset_and_wrap_when_configured() -> None:
    phineas_pts = np.array(
        [
            [450, 220],
            [415, 300],
            [485, 300],
        ],
        dtype=np.int32,
    )
    image, homography = _make_two_triangle_arena_image(phineas_pts=phineas_pts)
    heading_offset_rad = 0.75 * math.pi
    base_poses = detect_robot_poses(
        image,
        homography=homography,
        config=WorldFrameConfig(),
    )
    offset_config = WorldFrameConfig(
        robot_heading_offsets_rad={
            RobotName.PHINEAS.value: heading_offset_rad,
        },
    )

    offset_poses = detect_robot_poses(
        image,
        homography=homography,
        config=offset_config,
    )

    base_phineas = base_poses[RobotName.PHINEAS.value]
    offset_phineas = offset_poses[RobotName.PHINEAS.value]
    expected_theta = wrap_angle_rad(base_phineas.theta + heading_offset_rad)
    assert math.isclose(
        offset_phineas.x,
        base_phineas.x,
        abs_tol=1e-9,
    )
    assert math.isclose(
        offset_phineas.y,
        base_phineas.y,
        abs_tol=1e-9,
    )
    assert -math.pi < offset_phineas.theta <= math.pi
    assert math.isclose(
        offset_phineas.theta,
        expected_theta,
        abs_tol=1e-6,
    )


def test_should_rotate_position_offset_by_final_heading_when_configured() -> None:
    phineas_pts = np.array(
        [
            [350, 300],
            [270, 265],
            [270, 335],
        ],
        dtype=np.int32,
    )
    ferb_pts = np.array(
        [
            [690, 300],
            [610, 265],
            [610, 335],
        ],
        dtype=np.int32,
    )
    image, homography = _make_two_triangle_arena_image(
        phineas_pts=phineas_pts,
        ferb_pts=ferb_pts,
    )
    base_poses = detect_robot_poses(
        image,
        homography=homography,
        config=WorldFrameConfig(),
    )
    heading_offset_rad = math.pi / 2.0
    forward_offset_m = 0.10
    left_offset_m = 0.05
    offset_config = WorldFrameConfig(
        robot_heading_offsets_rad={
            RobotName.PHINEAS.value: heading_offset_rad,
        },
        robot_position_offsets_m={
            RobotName.PHINEAS.value: (forward_offset_m, left_offset_m),
        },
    )

    offset_poses = detect_robot_poses(
        image,
        homography=homography,
        config=offset_config,
    )

    base_phineas = base_poses[RobotName.PHINEAS.value]
    offset_phineas = offset_poses[RobotName.PHINEAS.value]
    final_theta = wrap_angle_rad(base_phineas.theta + heading_offset_rad)
    expected_x = base_phineas.x + (
        forward_offset_m * math.cos(final_theta) - left_offset_m * math.sin(final_theta)
    )
    expected_y = base_phineas.y + (
        forward_offset_m * math.sin(final_theta) + left_offset_m * math.cos(final_theta)
    )
    assert math.isclose(
        offset_phineas.theta,
        final_theta,
        abs_tol=1e-6,
    )
    assert math.isclose(
        offset_phineas.x,
        expected_x,
        abs_tol=1e-6,
    )
    assert math.isclose(
        offset_phineas.y,
        expected_y,
        abs_tol=1e-6,
    )
    assert offset_poses[RobotName.FERB.value] == base_poses[RobotName.FERB.value]


def test_should_ignore_cube_sized_red_blob_when_below_robot_min_area() -> None:
    image = np.full(
        (600, 900, 3),
        230,
        dtype=np.uint8,
    )
    cv2.rectangle(
        image,
        (40, 30),
        (860, 570),
        (0, 0, 0),
        20,
    )
    cv2.rectangle(
        image,
        (400, 280),
        (420, 300),
        (0, 0, 255),
        -1,
    )
    corners = detect_arena_corners(image)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    red_mask = np.zeros(
        (600, 900),
        dtype=np.uint8,
    )
    red_mask[280:300, 400:420] = 255

    poses = detect_robot_poses(
        image,
        homography=homography,
        config=WorldFrameConfig(),
    )
    blob_pose = triangle_pose(
        red_mask,
        min_area_px=DEFAULT_ROBOT_MIN_AREA_PX,
    )

    assert poses == {}
    assert blob_pose is None


def test_should_discard_cube_near_triangle_hull_only_when_triangle_detector_active() -> None:
    phineas_pts = np.array(
        [
            [350, 300],
            [270, 265],
            [270, 335],
        ],
        dtype=np.int32,
    )
    image, homography = _make_two_triangle_arena_image(phineas_pts=phineas_pts)
    # Place a small red blob 10 px past the triangle tip (inside dilated hull) and a far cube
    cv2.rectangle(
        image,
        (360, 292),
        (376, 308),
        (0, 0, 255),
        -1,
    )
    cv2.rectangle(
        image,
        (600, 290),
        (620, 310),
        (0, 0, 255),
        -1,
    )

    triangle_cubes = process_cubes(
        image,
        homography=homography,
        tracker=CubeTracker(),
        now_s=1.0,
        config=WorldFrameConfig(robot_detector=RobotDetector.TRIANGLE),
    )
    aruco_cubes = process_cubes(
        image,
        homography=homography,
        tracker=CubeTracker(),
        now_s=1.0,
        config=WorldFrameConfig(robot_detector=RobotDetector.ARUCO),
    )

    assert len(triangle_cubes) == 1
    assert len(aruco_cubes) == 2


def _make_aruco_arena_image() -> tuple[np.ndarray, np.ndarray]:
    image = np.full(
        (720, 1280, 3),
        245,
        dtype=np.uint8,
    )
    cv2.rectangle(
        image,
        (40, 30),
        (1240, 690),
        (0, 0, 0),
        20,
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_gray = cv2.aruco.generateImageMarker(
        dictionary,
        ARUCO_ID_PHINEAS,
        121,
    )
    marker_bgr = cv2.cvtColor(
        marker_gray,
        cv2.COLOR_GRAY2BGR,
    )
    image[180:301, 280:401] = marker_bgr
    corners = detect_arena_corners(image)
    homography = compute_homography(
        corners,
        ARENA_WIDTH_M,
        ARENA_HEIGHT_M,
    )
    return image, homography


def test_should_dispatch_to_aruco_detector_when_aruco_configured() -> None:
    image, homography = _make_aruco_arena_image()
    config = WorldFrameConfig(
        robot_detector=RobotDetector.ARUCO,
        aruco=ArucoRobotConfig(
            dictionary_name=DEFAULT_ARUCO_DICTIONARY,
            marker_ids={RobotName.PHINEAS.value: ARUCO_ID_PHINEAS},
            heading_offset_rad=0.0,
            marker_height_m=0.0,
        ),
    )

    poses = detect_robot_poses(
        image,
        homography=homography,
        config=config,
    )

    assert set(poses.keys()) == {RobotName.PHINEAS.value}
    phineas = poses[RobotName.PHINEAS.value]
    assert 0.0 < phineas.x < ARENA_WIDTH_M
    assert 0.0 < phineas.y < ARENA_HEIGHT_M
    assert math.isclose(
        phineas.theta,
        math.pi / 2.0,
        abs_tol=0.1,
    )
    assert math.isclose(
        phineas.confidence,
        DEFAULT_DETECTION_CONFIDENCE,
    )


def test_should_not_detect_aruco_marker_when_triangle_detector_selected() -> None:
    image, homography = _make_aruco_arena_image()
    config = WorldFrameConfig(robot_detector=RobotDetector.TRIANGLE)

    poses = detect_robot_poses(
        image,
        homography=homography,
        config=config,
    )

    assert poses == {}
