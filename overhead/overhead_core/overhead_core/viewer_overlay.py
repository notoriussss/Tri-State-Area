"""Pure safety overlay rendering for the rectified overhead viewer image."""

import math
from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import numpy as np

from overhead_core.constants import (
    ARENA_HEIGHT_M,
    COLLISION_LEVEL_COLOR_BGR,
    COLLISION_SLOW_DIST_M,
    COLLISION_STOP_DIST_M,
    COLOR_BGR_GRAY,
    COLOR_BGR_RED,
    COLOR_BGR_ZONE_OUTLINE,
    DEFAULT_RECTIFY_PX_PER_M,
    DEFAULT_ZONE_RECT,
    FONT_SCALE_LABEL,
    LABEL_OFFSET_X_PX,
    LABEL_OUTLINE_THICKNESS_PX,
    METERS_TO_CM,
    RADAR_MAX_RANGE_M,
    RADAR_SECTOR_COUNT,
    RADAR_SPOKE_MAX_LEN_PX,
    RADAR_SPOKE_THICKNESS_PX,
    RECTIFY_MARGIN_M,
    ROBOT_SAFETY_M,
    ROBOT_SAFETY_RING_THICKNESS_PX,
    SAFETY_CIRCLE_ALPHA,
    SAFETY_CIRCLE_GRAY_THICKNESS_PX,
    SAFETY_CIRCLE_RED_THICKNESS_PX,
    SAFETY_LABEL_OFFSET_Y_PX,
    VELOCITY_ZERO_EPSILON,
    ZONE_OUTLINE_THICKNESS_PX,
    CollisionLevel,
    ObstacleKind,
)
from overhead_core.types import (
    CollisionAssessment,
    Obstacle,
    RobotPose,
    SafetyResult,
)


@dataclass(frozen=True)
class SafetyOverlayConfig:
    """Geometry and delivery zone configuration for rectified safety overlays."""

    arena_height_m: float = ARENA_HEIGHT_M
    px_per_m: float = DEFAULT_RECTIFY_PX_PER_M
    margin_m: float = RECTIFY_MARGIN_M
    zone_rect: tuple[float, float, float, float] = DEFAULT_ZONE_RECT
    max_range_m: float = RADAR_MAX_RANGE_M


def world_to_rectified_px(
    x_m: float,
    y_m: float,
    *,
    arena_height_m: float = ARENA_HEIGHT_M,
    px_per_m: float = DEFAULT_RECTIFY_PX_PER_M,
    margin_m: float = RECTIFY_MARGIN_M,
) -> tuple[int, int]:
    """Maps world meters (x, y) to rectified image pixel coordinates (u, v).

    Args:
        x_m: World x-coordinate in meters.
        y_m: World y-coordinate in meters.
        arena_height_m: Arena height along world +y in meters.
        px_per_m: Rectified image scale in pixels per meter.
        margin_m: Outer margin around the arena perimeter in meters.

    Returns:
        Pixel coordinates (u, v) in the rectified image.
    """
    u_px = int(round((x_m + margin_m) * px_per_m))
    v_px = int(round((arena_height_m + margin_m - y_m) * px_per_m))
    return u_px, v_px


def _draw_zone_outline(
    image_bgr: np.ndarray,
    *,
    config: SafetyOverlayConfig,
) -> None:
    """Draws the delivery zone rectangle outline on the rectified image."""
    x0_m, y0_m, x1_m, y1_m = config.zone_rect
    top_left_px = world_to_rectified_px(
        x0_m,
        y1_m,
        arena_height_m=config.arena_height_m,
        px_per_m=config.px_per_m,
        margin_m=config.margin_m,
    )
    bottom_right_px = world_to_rectified_px(
        x1_m,
        y0_m,
        arena_height_m=config.arena_height_m,
        px_per_m=config.px_per_m,
        margin_m=config.margin_m,
    )
    cv2.rectangle(
        image_bgr,
        top_left_px,
        bottom_right_px,
        COLOR_BGR_ZONE_OUTLINE,
        ZONE_OUTLINE_THICKNESS_PX,
    )


def _draw_obstacle_circles(
    image_bgr: np.ndarray,
    circles: Sequence[Obstacle],
    *,
    config: SafetyOverlayConfig,
) -> None:
    """Draws semi-transparent safety circles for non-robot obstacles."""
    overlay = image_bgr.copy()
    has_circles = False
    for circle in circles:
        if circle.kind == ObstacleKind.ROBOT:
            continue
        has_circles = True
        is_red = circle.kind == ObstacleKind.RED_CUBE
        color = COLOR_BGR_RED if is_red else COLOR_BGR_GRAY
        thickness = SAFETY_CIRCLE_RED_THICKNESS_PX if is_red else SAFETY_CIRCLE_GRAY_THICKNESS_PX
        center_px = world_to_rectified_px(
            circle.x_m,
            circle.y_m,
            arena_height_m=config.arena_height_m,
            px_per_m=config.px_per_m,
            margin_m=config.margin_m,
        )
        radius_px = max(
            1,
            int(round(circle.radius_m * config.px_per_m)),
        )
        cv2.circle(
            overlay,
            center_px,
            radius_px,
            color,
            thickness,
            lineType=cv2.LINE_AA,
        )

    if not has_circles:
        return

    drawn_mask = np.any(
        overlay != image_bgr,
        axis=2,
    )
    if not np.any(drawn_mask):
        return

    blended = cv2.addWeighted(
        overlay,
        SAFETY_CIRCLE_ALPHA,
        image_bgr,
        1.0 - SAFETY_CIRCLE_ALPHA,
        0.0,
    )
    image_bgr[drawn_mask] = blended[drawn_mask]


def _sector_color(free_dist_m: float) -> tuple[int, int, int]:
    """Maps a single radar sector's free distance to a severity BGR color."""
    if free_dist_m < COLLISION_STOP_DIST_M:
        return COLLISION_LEVEL_COLOR_BGR[CollisionLevel.STOP]
    if free_dist_m <= COLLISION_SLOW_DIST_M:
        return COLLISION_LEVEL_COLOR_BGR[CollisionLevel.SLOW]
    return COLLISION_LEVEL_COLOR_BGR[CollisionLevel.CLEAR]


def _draw_radar_spokes(
    image_bgr: np.ndarray,
    *,
    pose: RobotPose,
    sectors: Sequence[float],
    config: SafetyOverlayConfig,
) -> None:
    """Draws the 8 radar sector spokes centered on the robot pose."""
    center_px = world_to_rectified_px(
        pose.x,
        pose.y,
        arena_height_m=config.arena_height_m,
        px_per_m=config.px_per_m,
        margin_m=config.margin_m,
    )
    sector_width_rad = math.tau / RADAR_SECTOR_COUNT
    max_range_m = max(
        config.max_range_m,
        VELOCITY_ZERO_EPSILON,
    )
    for idx, dist_m in enumerate(sectors):
        clamped_m = min(
            max(0.0, dist_m),
            max_range_m,
        )
        spoke_len_px = RADAR_SPOKE_MAX_LEN_PX * (clamped_m / max_range_m)
        if spoke_len_px < 1.0:
            continue
        angle_rad = pose.theta + idx * sector_width_rad
        end_px = (
            int(round(center_px[0] + spoke_len_px * math.cos(angle_rad))),
            int(round(center_px[1] - spoke_len_px * math.sin(angle_rad))),
        )
        cv2.line(
            image_bgr,
            center_px,
            end_px,
            _sector_color(dist_m),
            RADAR_SPOKE_THICKNESS_PX,
            lineType=cv2.LINE_AA,
        )


def _format_safety_label(assessment: CollisionAssessment) -> str:
    """Formats the collision level and nearest obstacle distance in centimeters."""
    if assessment.nearest_dist_m < 0.0:
        return f"{assessment.level.value} --"
    dist_cm = assessment.nearest_dist_m * METERS_TO_CM
    return f"{assessment.level.value} {dist_cm:.0f}cm"


def _draw_robot_ring_and_label(
    image_bgr: np.ndarray,
    *,
    pose: RobotPose,
    assessment: CollisionAssessment,
    config: SafetyOverlayConfig,
) -> None:
    """Draws a robot's collision-level safety ring and nearest-distance label."""
    center_px = world_to_rectified_px(
        pose.x,
        pose.y,
        arena_height_m=config.arena_height_m,
        px_per_m=config.px_per_m,
        margin_m=config.margin_m,
    )
    ring_color = COLLISION_LEVEL_COLOR_BGR[assessment.level]
    ring_radius_px = max(
        1,
        int(round(ROBOT_SAFETY_M * config.px_per_m)),
    )
    cv2.circle(
        image_bgr,
        center_px,
        ring_radius_px,
        ring_color,
        ROBOT_SAFETY_RING_THICKNESS_PX,
        lineType=cv2.LINE_AA,
    )
    label_pos = (
        center_px[0] + LABEL_OFFSET_X_PX,
        center_px[1] + SAFETY_LABEL_OFFSET_Y_PX,
    )
    cv2.putText(
        image_bgr,
        _format_safety_label(assessment),
        label_pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        FONT_SCALE_LABEL,
        ring_color,
        LABEL_OUTLINE_THICKNESS_PX,
        cv2.LINE_AA,
    )


def draw_safety_overlay(
    image_bgr: np.ndarray,
    safety: SafetyResult,
    robot_poses: Sequence[RobotPose],
    *,
    config: SafetyOverlayConfig | None = None,
) -> np.ndarray:
    """Draws safety circles, robot rings, radar spokes, and zone outline on a copy.

    Args:
        image_bgr: Rectified BGR image to annotate without mutating the original.
        safety: Arena-wide obstacle circles and per-robot safety assessments.
        robot_poses: Active robot poses (including held poses) in world meters.
        config: Optional rectified scale and delivery zone geometry configuration.

    Returns:
        New BGR image with safety overlays rendered.
    """
    cfg = config if config is not None else SafetyOverlayConfig()
    annotated = image_bgr.copy()

    _draw_zone_outline(
        annotated,
        config=cfg,
    )
    _draw_obstacle_circles(
        annotated,
        safety.circles,
        config=cfg,
    )

    for pose in robot_poses:
        robot_safety = safety.per_robot.get(pose.name)
        if robot_safety is None:
            continue
        _draw_radar_spokes(
            annotated,
            pose=pose,
            sectors=robot_safety.sectors,
            config=cfg,
        )
        _draw_robot_ring_and_label(
            annotated,
            pose=pose,
            assessment=robot_safety.assessment,
            config=cfg,
        )

    return annotated
