"""Integration tests for world_frame detection on the real arena photo and synthetic frames."""

import math
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    DEFAULT_DETECTION_CONFIDENCE,
    DEFAULT_ZONE_RECT,
    CubeColor,
    RobotName,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
)
from overhead_core.tracker import CubeTracker
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
            [680, 300],
            [610, 270],
            [610, 330],
        ],
        dtype=np.int32,
    )
    cv2.fillPoly(
        image,
        [triangle_pts],
        (0, 255, 0),
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
