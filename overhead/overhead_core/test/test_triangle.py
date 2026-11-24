"""Unit tests for triangle marker pose estimation and heading transform."""

import math

import cv2
import numpy as np
import pytest
from overhead_core.constants import (
    DEFAULT_ROBOT_MIN_AREA_PX,
    TriangleHeadingRule,
)
from overhead_core.geometry import compute_homography
from overhead_core.triangle import heading_to_world, triangle_pose


def test_should_detect_heading_zero_when_isosceles_points_right_with_opposite_shortest():
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
        heading_rule=TriangleHeadingRule.OPPOSITE_SHORTEST_SIDE,
    )

    assert pose is not None
    _u, _v, theta_img = pose
    assert math.isclose(
        theta_img,
        0.0,
        abs_tol=0.1,
    )


def test_should_detect_heading_minus_half_pi_when_isosceles_points_up_with_opposite_shortest():
    mask = np.zeros(
        (200, 200),
        dtype=np.uint8,
    )
    # Isosceles triangle pointing up: base is 80 px wide at v=140, tip at v=50
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
        heading_rule=TriangleHeadingRule.OPPOSITE_SHORTEST_SIDE,
    )

    assert pose is not None
    _u, _v, theta_img = pose
    assert math.isclose(
        theta_img,
        -math.pi / 2.0,
        abs_tol=0.1,
    )


def test_should_detect_east_heading_when_rear_edge_triangle_uses_longest_side_normal():
    mask = np.zeros(
        (600, 200),
        dtype=np.uint8,
    )
    # Exact Ferb triangle vertices measured in arena_ferb_east_2026-11-24.jpg:
    # longest side (131 px) along vertical rear edge, apex pointing east (+u).
    pts = np.array(
        [[50, 372], [84, 459], [53, 503]],
        dtype=np.int32,
    )
    cv2.fillPoly(
        mask,
        [pts],
        255,
    )

    default_pose = triangle_pose(
        mask,
        min_area_px=50,
    )
    old_rule_pose = triangle_pose(
        mask,
        min_area_px=50,
        heading_rule=TriangleHeadingRule.OPPOSITE_SHORTEST_SIDE,
    )

    assert default_pose is not None
    assert old_rule_pose is not None
    u_px, v_px, theta_default = default_pose
    _old_u, _old_v, theta_old = old_rule_pose
    assert math.isclose(
        u_px,
        (50.0 + 84.0 + 53.0) / 3.0,
        abs_tol=2.0,
    )
    assert math.isclose(
        v_px,
        (372.0 + 459.0 + 503.0) / 3.0,
        abs_tol=2.0,
    )
    assert math.isclose(
        theta_default,
        0.0,
        abs_tol=0.1,
    )
    assert abs(theta_old + math.pi / 2.0) < 0.3


@pytest.mark.parametrize(
    ("pts", "expected_theta_img"),
    [
        ([[110, 100], [60, 30], [60, 170]], 0.0),
        ([[100, 50], [30, 100], [170, 100]], -math.pi / 2.0),
        ([[50, 100], [100, 30], [100, 170]], math.pi),
        ([[100, 150], [30, 100], [170, 100]], math.pi / 2.0),
    ],
)
def test_should_return_normal_toward_opposite_vertex_when_longest_side_normal_selected(
    pts: list[list[int]],
    expected_theta_img: float,
):
    mask = np.zeros(
        (220, 220),
        dtype=np.uint8,
    )
    cv2.fillPoly(
        mask,
        [np.array(pts, dtype=np.int32)],
        255,
    )

    pose = triangle_pose(
        mask,
        min_area_px=50,
        heading_rule=TriangleHeadingRule.LONGEST_SIDE_NORMAL,
    )

    assert pose is not None
    _u, _v, theta_img = pose
    diff = (theta_img - expected_theta_img + math.pi) % (2.0 * math.pi) - math.pi
    assert abs(diff) < 0.08


def test_should_use_convex_hull_area_when_wires_notch_triangle_mask():
    mask = np.zeros(
        (300, 300),
        dtype=np.uint8,
    )
    # Full triangle area = 0.5 * 120 * 60 = 3600 px; notch removes 25 * 60 = 1500 px (~2100 px)
    pts = np.array(
        [[140, 150], [80, 90], [80, 210]],
        dtype=np.int32,
    )
    cv2.fillPoly(
        mask,
        [pts],
        255,
    )
    cv2.rectangle(
        mask,
        (80, 120),
        (105, 180),
        0,
        -1,
    )

    pose = triangle_pose(
        mask,
        min_area_px=2800,
    )

    assert pose is not None
    _u, _v, theta_img = pose
    assert math.isclose(
        theta_img,
        0.0,
        abs_tol=0.1,
    )


def test_should_reject_single_cube_sized_blob_when_using_default_min_area():
    mask = np.zeros(
        (200, 200),
        dtype=np.uint8,
    )
    # 25x25 cube blob (625 px) is well below DEFAULT_ROBOT_MIN_AREA_PX (1200 px)
    cv2.rectangle(
        mask,
        (80, 80),
        (105, 105),
        255,
        -1,
    )

    pose = triangle_pose(
        mask,
        min_area_px=DEFAULT_ROBOT_MIN_AREA_PX,
    )

    assert pose is None


def test_should_raise_value_error_when_heading_rule_unknown():
    mask = np.zeros(
        (100, 100),
        dtype=np.uint8,
    )

    with pytest.raises(ValueError):
        triangle_pose(
            mask,
            heading_rule="unknown_rule",  # type: ignore[arg-type]
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
