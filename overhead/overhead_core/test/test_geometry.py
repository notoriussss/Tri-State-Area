"""Unit tests for pure geometry functions."""

import math

import cv2
import numpy as np
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    image_to_world,
    rectify,
    world_to_image,
    world_to_robot,
)


def test_should_roundtrip_homography_when_mapping_corners():
    corners_px = np.array(
        [
            [65.0, 1035.0],
            [1740.0, 1050.0],
            [1790.0, 25.0],
            [45.0, 20.0],
        ],
        dtype=np.float64,
    )
    w_m, h_m = 2.4, 1.5
    homography = compute_homography(
        corners_px,
        w_m,
        h_m,
    )

    world_pts = image_to_world(
        homography,
        corners_px,
    )
    expected_world = np.array(
        [
            [0.0, 0.0],
            [w_m, 0.0],
            [w_m, h_m],
            [0.0, h_m],
        ]
    )
    assert np.allclose(
        world_pts,
        expected_world,
        atol=1e-4,
    )

    recovered_px = world_to_image(
        homography,
        world_pts,
    )
    assert np.allclose(
        recovered_px,
        corners_px,
        atol=1e-3,
    )


def test_should_return_straight_ahead_when_robot_faces_target():
    # Robot at (1.0, 0.5) facing +Y (theta = pi/2); target straight ahead at (1.0, 1.5).
    robot_pose = (1.0, 0.5, math.pi / 2.0)
    target_point = (1.0, 1.5)

    x_r, y_r, dist, bearing = world_to_robot(
        robot_pose,
        target_point,
    )

    assert math.isclose(
        x_r,
        1.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        y_r,
        0.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        dist,
        1.0,
        abs_tol=1e-6,
    )
    assert math.isclose(
        bearing,
        0.0,
        abs_tol=1e-6,
    )


def test_should_preserve_expected_shape_when_rectifying():
    corners_px = [
        [0.0, 100.0],
        [200.0, 100.0],
        [200.0, 0.0],
        [0.0, 0.0],
    ]
    homography = compute_homography(
        corners_px,
        2.0,
        1.0,
    )
    img = np.zeros(
        (100, 200, 3),
        dtype=np.uint8,
    )

    rectified_no_margin = rectify(
        img,
        homography,
        px_per_m=50.0,
        margin_m=0.0,
    )
    rectified_with_margin = rectify(
        img,
        homography,
        px_per_m=50.0,
        margin_m=0.1,
    )

    assert rectified_no_margin.shape == (50, 100, 3)
    assert rectified_with_margin.shape == (60, 110, 3)


def test_should_keep_black_border_when_rectifying_with_margin():
    img = np.full(
        (600, 900, 3),
        230,
        np.uint8,
    )
    # Tape 20 px wide, inner edges ~(40, 30) to (860, 570)
    cv2.rectangle(
        img,
        (30, 20),
        (870, 580),
        (0, 0, 0),
        20,
    )
    homography = compute_homography(
        detect_arena_corners(img),
        8.2,
        5.4,
    )

    top = rectify(
        img,
        homography,
        px_per_m=100.0,
        margin_m=0.1,
    )

    assert top[270, 5].max() < 50
    assert top[5, 400].max() < 50
    assert top[270, 20].min() > 200


def test_should_detect_corners_when_synthetic_border_present():
    img = np.full(
        (600, 900, 3),
        230,
        np.uint8,
    )
    cv2.rectangle(
        img,
        (40, 30),
        (860, 570),
        (0, 0, 0),
        20,
    )

    corners = detect_arena_corners(img)

    expected = np.array(
        [
            [50, 560],
            [850, 560],
            [850, 40],
            [50, 40],
        ]
    )
    assert np.allclose(
        corners,
        expected,
        atol=3,
    )
