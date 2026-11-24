"""Unit tests for the pure rectified safety overlay renderer."""

import math

import numpy as np
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    DEFAULT_RECTIFY_PX_PER_M,
    RECTIFY_MARGIN_M,
    RED_CUBE_SAFETY_M,
    ROBOT_SAFETY_M,
    CollisionLevel,
    CubeColor,
    RobotName,
)
from overhead_core.types import RobotPose
from overhead_core.viewer_overlay import (
    draw_safety_overlay,
    world_to_rectified_px,
)
from overhead_core.world_safety import (
    SafetyConfig,
    compute_safety,
)

RECT_WIDTH_PX: int = int(round((ARENA_WIDTH_M + 2.0 * RECTIFY_MARGIN_M) * DEFAULT_RECTIFY_PX_PER_M))
RECT_HEIGHT_PX: int = int(
    round((ARENA_HEIGHT_M + 2.0 * RECTIFY_MARGIN_M) * DEFAULT_RECTIFY_PX_PER_M)
)


def _blank_rectified() -> np.ndarray:
    return np.zeros(
        (RECT_HEIGHT_PX, RECT_WIDTH_PX, 3),
        dtype=np.uint8,
    )


def _pose(
    name: str,
    x_m: float,
    y_m: float,
    *,
    theta_rad: float = 0.0,
) -> RobotPose:
    return RobotPose(
        name=name,
        x=x_m,
        y=y_m,
        theta=theta_rad,
        confidence=1.0,
    )


def test_should_change_pixels_around_red_cube_safety_circle_and_not_elsewhere() -> None:
    blank = _blank_rectified()
    safety = compute_safety(
        [],
        [(1, CubeColor.RED.value, 0.50, 0.35, 1.0)],
        velocities={},
        config=SafetyConfig(),
    )
    center_u, center_v = world_to_rectified_px(
        0.50,
        0.35,
    )
    radius_px = int(round(RED_CUBE_SAFETY_M * DEFAULT_RECTIFY_PX_PER_M))
    far_u, far_v = world_to_rectified_px(
        0.15,
        0.15,
    )

    annotated = draw_safety_overlay(
        blank,
        safety,
        [],
    )
    circle_patch = annotated[
        center_v - 1 : center_v + 2,
        center_u + radius_px - 1 : center_u + radius_px + 2,
    ]

    assert np.all(blank == 0)
    assert int(circle_patch[:, :, 2].max()) > 0
    assert int(circle_patch[:, :, 2].max()) > int(circle_patch[:, :, 1].max())
    assert int(circle_patch[:, :, 2].max()) > int(circle_patch[:, :, 0].max())
    assert np.array_equal(
        annotated[center_v, center_u],
        np.array([0, 0, 0], dtype=np.uint8),
    )
    assert np.array_equal(
        annotated[far_v, far_u],
        np.array([0, 0, 0], dtype=np.uint8),
    )


def test_should_draw_red_dominant_ring_when_robot_is_stop() -> None:
    blank = _blank_rectified()
    phineas = _pose(
        RobotName.PHINEAS.value,
        0.40,
        0.35,
        theta_rad=0.0,
    )
    ferb = _pose(
        RobotName.FERB.value,
        0.90,
        0.35,
        theta_rad=math.pi,
    )
    safety = compute_safety(
        [phineas, ferb],
        [],
        velocities={},
        config=SafetyConfig(),
    )
    pu, pv = world_to_rectified_px(
        phineas.x,
        phineas.y,
    )
    ring_radius_px = int(round(ROBOT_SAFETY_M * DEFAULT_RECTIFY_PX_PER_M))

    annotated = draw_safety_overlay(
        blank,
        safety,
        [phineas, ferb],
    )
    ring_patch = annotated[
        pv - ring_radius_px - 1 : pv - ring_radius_px + 2,
        pu - 1 : pu + 2,
    ]

    assert safety.per_robot[RobotName.PHINEAS.value].assessment.level == CollisionLevel.STOP
    assert int(ring_patch[:, :, 2].max()) > int(ring_patch[:, :, 1].max())
    assert int(ring_patch[:, :, 2].max()) > int(ring_patch[:, :, 0].max())


def test_should_draw_green_dominant_ring_when_robot_is_clear() -> None:
    blank = _blank_rectified()
    phineas = _pose(
        RobotName.PHINEAS.value,
        0.58,
        0.05,
        theta_rad=math.pi / 2.0,
    )
    safety = compute_safety(
        [phineas],
        [],
        velocities={},
        config=SafetyConfig(),
    )
    pu, pv = world_to_rectified_px(
        phineas.x,
        phineas.y,
    )
    ring_radius_px = int(round(ROBOT_SAFETY_M * DEFAULT_RECTIFY_PX_PER_M))

    annotated = draw_safety_overlay(
        blank,
        safety,
        [phineas],
    )
    ring_patch = annotated[
        pv - 1 : pv + 2,
        pu - ring_radius_px - 1 : pu - ring_radius_px + 2,
    ]

    assert safety.per_robot[RobotName.PHINEAS.value].assessment.level == CollisionLevel.CLEAR
    assert int(ring_patch[:, :, 1].max()) > int(ring_patch[:, :, 2].max())
    assert int(ring_patch[:, :, 1].max()) > int(ring_patch[:, :, 0].max())


def test_should_draw_longer_radar_spokes_in_empty_arena_than_toward_nearby_obstacle() -> None:
    blank = _blank_rectified()
    phineas = _pose(
        RobotName.PHINEAS.value,
        0.35,
        0.35,
        theta_rad=0.0,
    )
    empty_safety = compute_safety(
        [phineas],
        [],
        velocities={},
        config=SafetyConfig(),
    )
    blocked_safety = compute_safety(
        [phineas],
        [(1, CubeColor.RED.value, 0.65, 0.35, 1.0)],
        velocities={},
        config=SafetyConfig(),
    )
    ru, rv = world_to_rectified_px(
        phineas.x,
        phineas.y,
    )

    empty_img = draw_safety_overlay(
        blank,
        empty_safety,
        [phineas],
    )
    blocked_img = draw_safety_overlay(
        blank,
        blocked_safety,
        [phineas],
    )
    empty_cols = np.flatnonzero(
        np.any(
            empty_img[rv - 1 : rv + 2, ru : ru + 50] > 0,
            axis=(0, 2),
        )
    )
    blocked_cols = np.flatnonzero(
        np.any(
            blocked_img[rv - 1 : rv + 2, ru : ru + 50] > 0,
            axis=(0, 2),
        )
    )

    assert int(empty_cols.max()) > int(blocked_cols.max())
