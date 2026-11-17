"""Unit tests for triangle marker pose estimation and heading transform."""

import math

import cv2
import numpy as np
from overhead_core.geometry import compute_homography
from overhead_core.triangle import heading_to_world, triangle_pose


def test_should_detect_heading_zero_when_triangle_points_right():
    mask = np.zeros(
        (200, 200),
        dtype=np.uint8,
    )
    # Isosceles triangle pointing right: base is 80 px tall at u=60, tip at u=150
    pts = np.array(
        [[150, 100], [60, 60], [60, 140]],
        dtype=np.int32,
    )
    cv2.fillPoly(
        mask,
        [pts],
        255,
    )

    pose = triangle_pose(
        mask,
        min_area_px=50,
    )

    assert pose is not None
    _u, _v, theta_img = pose
    assert math.isclose(
        theta_img,
        0.0,
        abs_tol=0.1,
    )


def test_should_detect_heading_minus_half_pi_when_triangle_points_up():
    mask = np.zeros(
        (200, 200),
        dtype=np.uint8,
    )
    # Isosceles triangle pointing up: base is 80 px wide at v=150, tip at v=50
    pts = np.array(
        [[100, 50], [60, 140], [140, 140]],
        dtype=np.int32,
    )
    cv2.fillPoly(
        mask,
        [pts],
        255,
    )

    pose = triangle_pose(
        mask,
        min_area_px=50,
    )

    assert pose is not None
    _u, _v, theta_img = pose
    assert math.isclose(
        theta_img,
        -math.pi / 2.0,
        abs_tol=0.1,
    )


def test_should_transform_heading_to_world_when_homography_applied():
    # Corners: BL, BR, TR, TL
    corners = np.array(
        [[0.0, 100.0], [100.0, 100.0], [100.0, 0.0], [0.0, 0.0]],
        dtype=np.float32,
    )
    homography = compute_homography(
        corners,
        arena_width_m=1.0,
        arena_height_m=1.0,
    )

    # Heading pointing up in image (-pi/2) corresponds to world +y (+pi/2)
    world_theta_up = heading_to_world(
        homography,
        50.0,
        50.0,
        theta_img=-math.pi / 2.0,
    )
    # Heading pointing right in image (0.0) corresponds to world +x (0.0)
    world_theta_right = heading_to_world(
        homography,
        50.0,
        50.0,
        theta_img=0.0,
    )

    assert math.isclose(
        world_theta_up,
        math.pi / 2.0,
        abs_tol=0.1,
    )
    assert math.isclose(
        world_theta_right,
        0.0,
        abs_tol=0.1,
    )


def test_should_return_none_when_mask_is_empty():
    empty_mask = np.zeros(
        (100, 100),
        dtype=np.uint8,
    )

    result = triangle_pose(empty_mask)

    assert result is None
