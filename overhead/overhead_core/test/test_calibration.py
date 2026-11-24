"""Unit tests for arena quadrilateral calibration validation."""

from pathlib import Path

import cv2
import numpy as np
from overhead_core.calibration import validate_arena_quad
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
)
from overhead_core.geometry import detect_arena_corners


def test_should_accept_detected_quad_when_real_arena_photo_provided():
    fixture_path = Path(__file__).parent / "fixtures" / "arena_real_2026-11-24.jpg"
    image = cv2.imdecode(
        np.fromfile(
            str(fixture_path),
            np.uint8,
        ),
        cv2.IMREAD_COLOR,
    )
    assert image is not None
    height_px, width_px = image.shape[:2]
    corners = detect_arena_corners(image)

    is_valid = validate_arena_quad(
        corners,
        image_width_px=width_px,
        image_height_px=height_px,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is True


def test_should_accept_quad_when_synthetic_border_matches_arena():
    corners = np.array(
        [
            [140.0, 660.0],
            [1140.0, 660.0],
            [1140.0, 60.0],
            [140.0, 60.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is True


def test_should_reject_quad_when_cropped_to_small_part_of_image():
    corners = np.array(
        [
            [100.0, 300.0],
            [433.0, 300.0],
            [433.0, 100.0],
            [100.0, 100.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is False


def test_should_reject_quad_when_below_minimum_area_fraction():
    corners = np.array(
        [
            [300.0, 540.0],
            [900.0, 540.0],
            [900.0, 180.0],
            [300.0, 180.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is False


def test_should_reject_quad_when_aspect_ratio_is_square():
    corners = np.array(
        [
            [300.0, 625.0],
            [850.0, 625.0],
            [850.0, 75.0],
            [300.0, 75.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is False


def test_should_reject_quad_when_vertices_are_in_wrong_order():
    corners = np.array(
        [
            [140.0, 60.0],
            [1140.0, 60.0],
            [1140.0, 660.0],
            [140.0, 660.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is False


def test_should_reject_quad_when_non_convex_bowtie():
    corners = np.array(
        [
            [140.0, 660.0],
            [1140.0, 660.0],
            [140.0, 60.0],
            [1140.0, 60.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is False


def test_should_reject_quad_when_concave():
    corners = np.array(
        [
            [140.0, 660.0],
            [1140.0, 660.0],
            [300.0, 500.0],
            [140.0, 60.0],
        ],
        dtype=np.float32,
    )

    is_valid = validate_arena_quad(
        corners,
        image_width_px=1280,
        image_height_px=720,
        arena_width_m=ARENA_WIDTH_M,
        arena_height_m=ARENA_HEIGHT_M,
    )

    assert is_valid is False
