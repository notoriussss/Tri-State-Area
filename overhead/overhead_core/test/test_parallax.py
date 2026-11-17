"""Unit tests for parallax projection corrections."""

import numpy as np
from overhead_core.constants import (
    CAM_HEIGHT_M,
    CUBE_HEIGHT_M,
    ROBOT_MARKER_HEIGHT_M,
)
from overhead_core.parallax import correct_parallax


def test_should_keep_point_unchanged_when_height_is_zero():
    seen_xy = np.array(
        [[0.8, 0.4]],
        dtype=np.float64,
    )
    center_xy = np.array(
        [0.5825, 0.35],
        dtype=np.float64,
    )

    corrected = correct_parallax(
        seen_xy,
        center_xy,
        object_height_m=0.0,
        cam_height_m=CAM_HEIGHT_M,
    )

    assert np.allclose(
        corrected,
        seen_xy,
    )


def test_should_shift_point_inward_when_robot_marker_has_height():
    seen_xy = np.array(
        [[1.0, 0.35]],
        dtype=np.float64,
    )
    center_xy = np.array(
        [0.5, 0.35],
        dtype=np.float64,
    )
    expected_x = 0.5 + (1.0 - 0.5) * (1.0 - ROBOT_MARKER_HEIGHT_M) / 1.0

    corrected = correct_parallax(
        seen_xy,
        center_xy,
        object_height_m=ROBOT_MARKER_HEIGHT_M,
        cam_height_m=1.0,
    )

    assert np.isclose(
        corrected[0, 0],
        expected_x,
    )
    assert np.isclose(
        corrected[0, 1],
        0.35,
    )


def test_should_shift_cube_inward_when_cube_has_height():
    seen_xy = np.array(
        [[0.9, 0.5]],
        dtype=np.float64,
    )
    center_xy = np.array(
        [0.5, 0.5],
        dtype=np.float64,
    )
    expected_x = 0.5 + (0.9 - 0.5) * (1.0 - CUBE_HEIGHT_M) / 1.0

    corrected = correct_parallax(
        seen_xy,
        center_xy,
        object_height_m=CUBE_HEIGHT_M,
        cam_height_m=1.0,
    )

    assert np.isclose(
        corrected[0, 0],
        expected_x,
    )
    assert np.isclose(
        corrected[0, 1],
        0.5,
    )


def test_should_leave_point_at_nadir_unchanged_when_at_center():
    center_xy = np.array(
        [0.5825, 0.35],
        dtype=np.float64,
    )
    seen_xy = np.array(
        [[0.5825, 0.35]],
        dtype=np.float64,
    )

    corrected = correct_parallax(
        seen_xy,
        center_xy,
        object_height_m=ROBOT_MARKER_HEIGHT_M,
    )

    assert np.allclose(
        corrected,
        center_xy,
    )


def test_should_correct_multiple_points_when_passed_nx2_array():
    center_xy = np.array(
        [0.0, 0.0],
        dtype=np.float64,
    )
    seen_xy = np.array(
        [[1.0, 0.0], [0.0, 2.0]],
        dtype=np.float64,
    )

    corrected = correct_parallax(
        seen_xy,
        center_xy,
        object_height_m=0.1,
        cam_height_m=1.0,
    )

    assert np.allclose(
        corrected,
        [[0.9, 0.0], [0.0, 1.8]],
    )
