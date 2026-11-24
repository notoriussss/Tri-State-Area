"""Unit tests for the pure overhead web viewer image pipeline."""

import math

import cv2
import numpy as np
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    BANNER_HEIGHT_PX,
    DEFAULT_RECTIFY_PX_PER_M,
    RECTIFY_MARGIN_M,
    CubeColor,
    RobotName,
    ViewerView,
)
from overhead_core.viewer_pipeline import (
    calibrate_frame,
    process_frame,
)


def _make_synthetic_arena_image() -> np.ndarray:
    """Creates a synthetic BGR arena image with border, cubes, and a robot triangle."""
    img = np.full(
        (600, 900, 3),
        230,
        dtype=np.uint8,
    )
    # Black tape border: 20 px thick, inner corners [50, 560]..[850, 40]
    cv2.rectangle(
        img,
        (40, 30),
        (860, 570),
        (0, 0, 0),
        20,
    )
    # Yellow cube inside arena
    cv2.rectangle(
        img,
        (200, 200),
        (230, 230),
        (0, 255, 255),
        -1,
    )
    # Red cube inside arena
    cv2.rectangle(
        img,
        (400, 300),
        (430, 330),
        (0, 0, 255),
        -1,
    )
    # Green Phineas triangle pointing right
    triangle_pts = np.array(
        [
            [680, 300],
            [610, 270],
            [610, 330],
        ],
        dtype=np.int32,
    )
    cv2.fillPoly(
        img,
        [triangle_pts],
        (0, 255, 0),
    )
    return img


def test_should_return_calibrated_views_and_detections_when_arena_valid():
    image = _make_synthetic_arena_image()
    expected_rect_w = int(
        round((ARENA_WIDTH_M + 2.0 * RECTIFY_MARGIN_M) * DEFAULT_RECTIFY_PX_PER_M)
    )
    expected_rect_h = int(
        round((ARENA_HEIGHT_M + 2.0 * RECTIFY_MARGIN_M) * DEFAULT_RECTIFY_PX_PER_M)
    )

    homography, corners = calibrate_frame(image)
    output = process_frame(
        image,
        homography,
        corners_px=corners,
    )

    assert homography is not None
    assert corners is not None
    assert output.is_calibrated is True
    assert set(output.views.keys()) == {
        ViewerView.RAW.value,
        ViewerView.RECTIFIED.value,
        ViewerView.MASK.value,
    }
    assert output.views[ViewerView.RAW.value].shape == image.shape
    assert output.views[ViewerView.MASK.value].shape == image.shape
    assert output.views[ViewerView.RECTIFIED.value].shape == (
        expected_rect_h,
        expected_rect_w,
        3,
    )

    cube_colors = {c.color for c in output.cubes}
    assert cube_colors == {
        CubeColor.YELLOW.value,
        CubeColor.RED.value,
    }
    assert all(0.0 < c.x_m < ARENA_WIDTH_M for c in output.cubes)
    assert all(0.0 < c.y_m < ARENA_HEIGHT_M for c in output.cubes)

    assert len(output.robots) == 1
    assert output.robots[0].name == RobotName.PHINEAS.value
    assert math.isclose(
        output.robots[0].theta_rad,
        0.0,
        abs_tol=0.15,
    )


def test_should_show_banner_and_return_no_world_positions_when_uncalibrated():
    image = _make_synthetic_arena_image()

    output = process_frame(
        image,
        None,
    )

    assert output.is_calibrated is False
    assert output.cubes == []
    assert output.robots == []
    assert output.corners_px is None

    raw_view = output.views[ViewerView.RAW.value]
    rect_view = output.views[ViewerView.RECTIFIED.value]
    mask_view = output.views[ViewerView.MASK.value]

    assert raw_view.shape == image.shape
    assert mask_view.shape == image.shape
    assert not np.array_equal(
        raw_view[:BANNER_HEIGHT_PX, :],
        image[:BANNER_HEIGHT_PX, :],
    )
    assert np.any(rect_view[:BANNER_HEIGHT_PX, :] > 0)
    assert np.any(mask_view[:BANNER_HEIGHT_PX, :, 2] > 100)


def test_should_return_none_homography_when_arena_corners_absent():
    blank_image = np.full(
        (480, 640, 3),
        240,
        dtype=np.uint8,
    )

    homography, corners = calibrate_frame(blank_image)

    assert homography is None
    assert corners is None


def test_should_highlight_cube_pixels_when_mask_view_rendered():
    image = np.full(
        (400, 600, 3),
        220,
        dtype=np.uint8,
    )
    cv2.rectangle(
        image,
        (30, 20),
        (570, 380),
        (0, 0, 0),
        20,
    )
    cv2.rectangle(
        image,
        (250, 180),
        (290, 220),
        (255, 0, 0),
        -1,
    )
    homography, corners = calibrate_frame(image)

    output = process_frame(
        image,
        homography,
        corners_px=corners,
    )
    mask_view = output.views[ViewerView.MASK.value]
    cube_pixel = mask_view[200, 270]
    floor_pixel = mask_view[100, 100]

    assert int(cube_pixel[0]) > int(cube_pixel[2])
    assert int(cube_pixel[0]) > int(floor_pixel[0])
