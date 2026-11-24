"""Pure image processing pipeline for the standalone overhead camera web viewer."""

import math
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from overhead_core.aruco_robots import ArucoRobotConfig
from overhead_core.calibration import validate_arena_quad
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    BANNER_HEIGHT_PX,
    BANNER_TEXT_X_PX,
    BANNER_TEXT_Y_PX,
    CAM_HEIGHT_M,
    COLOR_BGR_BANNER_BG,
    COLOR_BGR_BLACK,
    COLOR_BGR_GREEN,
    COLOR_BGR_UNKNOWN,
    COLOR_BGR_WHITE,
    CORNER_LABEL_OFFSET_PX,
    CORNER_RADIUS_PX,
    CUBE_COLOR_BGR,
    CUBE_HEIGHT_M,
    CUBE_MARKER_RADIUS_PX,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_CUBE_HSV_RANGES,
    DEFAULT_DARK_THRESH,
    DEFAULT_DETECTION_CONFIDENCE,
    DEFAULT_MIN_AREA_PX,
    DEFAULT_RECTIFY_PX_PER_M,
    DEFAULT_ROBOT_HSV_RANGES,
    DEFAULT_ROBOT_MIN_AREA_PX,
    DEFAULT_TRIANGLE_HEADING_RULE,
    DEFAULT_ZONE_RECT,
    DRAW_LINE_THICKNESS_PX,
    EMPTY_DARK_PIXEL_MIN_COUNT,
    FONT_SCALE_BANNER,
    FONT_SCALE_HINT,
    FONT_SCALE_LABEL,
    LABEL_OFFSET_X_PX,
    LABEL_OFFSET_Y_PX,
    LABEL_OUTLINE_THICKNESS_PX,
    MASK_BG_DIM_ALPHA,
    MASK_OVERLAY_ALPHA,
    MAX_CUBE_AREA_PX,
    METERS_TO_CM,
    MIN_CORNER_BOUND_MARGIN_PX,
    PLACEHOLDER_HINT_Y_PX,
    PLACEHOLDER_TEXT_X_PX,
    PLACEHOLDER_TEXT_Y_PX,
    QUAD_VERTEX_COUNT,
    RECTIFY_MARGIN_M,
    ROBOT_ARROW_LEN_PX,
    ROBOT_ARROW_TIP_RATIO,
    ROBOT_COLOR_BGR,
    ROBOT_TRIANGLE_HALF_BASE_PX,
    ROBOT_TRIANGLE_LENGTH_PX,
    ROBOT_TRIANGLE_REAR_RATIO,
    VIEWER_CORNER_LABELS,
    VIEWER_TEXT_CALIBRATION_HINT,
    VIEWER_TEXT_NOT_CALIBRATED,
    RobotDetector,
    TriangleHeadingRule,
    ViewerView,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    image_to_world,
    rectify,
    world_to_image,
)
from overhead_core.parallax import correct_parallax
from overhead_core.types import (
    RobotPose,
    ViewerSafetyResult,
)
from overhead_core.viewer_overlay import (
    SafetyOverlayConfig,
    draw_safety_overlay,
    world_to_rectified_px,
)
from overhead_core.viewer_safety import ViewerSafety
from overhead_core.world_frame import (
    WorldFrameConfig,
    detect_filtered_cubes,
    detect_robot_poses,
)

HsvBoundsMap = dict[str, list[tuple[Any, Any]]]


@dataclass(frozen=True)
class ViewerHsvConfig:
    """HSV color thresholds for cube and robot marker detection."""

    cube_ranges: HsvBoundsMap = field(
        default_factory=lambda: dict(DEFAULT_CUBE_HSV_RANGES),
    )
    robot_ranges: HsvBoundsMap = field(
        default_factory=lambda: dict(DEFAULT_ROBOT_HSV_RANGES),
    )


@dataclass(frozen=True)
class ViewerPipelineConfig:
    """Geometry, parallax, and rendering configuration for the viewer pipeline."""

    arena_width_m: float = ARENA_WIDTH_M
    arena_height_m: float = ARENA_HEIGHT_M
    cam_height_m: float = CAM_HEIGHT_M
    camera_center_world: tuple[float, float] = DEFAULT_CAMERA_CENTER_WORLD
    px_per_m: float = DEFAULT_RECTIFY_PX_PER_M
    margin_m: float = RECTIFY_MARGIN_M
    min_area_px: int = DEFAULT_MIN_AREA_PX
    robot_min_area_px: int = DEFAULT_ROBOT_MIN_AREA_PX
    max_area_px: int = MAX_CUBE_AREA_PX
    robot_detector: RobotDetector = RobotDetector.TRIANGLE
    aruco: ArucoRobotConfig | None = None
    triangle_heading_rule: TriangleHeadingRule = DEFAULT_TRIANGLE_HEADING_RULE


@dataclass(frozen=True)
class ViewerCubeResult:
    """Detected cube in world floor coordinates."""

    color: str
    x_m: float
    y_m: float
    area_px: float


@dataclass(frozen=True)
class ViewerRobotResult:
    """Detected robot marker pose in world floor coordinates."""

    name: str
    x_m: float
    y_m: float
    theta_rad: float


@dataclass(frozen=True)
class PipelineOutput:
    """Annotated viewer images and structured detections for a single frame."""

    views: dict[str, np.ndarray]
    cubes: list[ViewerCubeResult]
    robots: list[ViewerRobotResult]
    is_calibrated: bool
    corners_px: np.ndarray | None
    safety: ViewerSafetyResult | None = None


def _normalize_hsv_config(
    hsv_config: ViewerHsvConfig | HsvBoundsMap | None,
) -> ViewerHsvConfig:
    """Converts an optional HSV config or raw cube dict into ViewerHsvConfig."""
    if hsv_config is None:
        return ViewerHsvConfig()
    if isinstance(hsv_config, ViewerHsvConfig):
        return hsv_config
    return ViewerHsvConfig(cube_ranges=hsv_config)


def _are_corners_image_bounds(
    corners: np.ndarray,
    width_px: int,
    height_px: int,
) -> bool:
    """Checks whether all four detected corners lie directly on the image perimeter."""
    on_x_edge = (corners[:, 0] <= MIN_CORNER_BOUND_MARGIN_PX) | (
        corners[:, 0] >= width_px - 1 - MIN_CORNER_BOUND_MARGIN_PX
    )
    on_y_edge = (corners[:, 1] <= MIN_CORNER_BOUND_MARGIN_PX) | (
        corners[:, 1] >= height_px - 1 - MIN_CORNER_BOUND_MARGIN_PX
    )
    return bool(np.all(on_x_edge & on_y_edge))


def calibrate_frame(
    frame_bgr: np.ndarray,
    *,
    arena_width_m: float = ARENA_WIDTH_M,
    arena_height_m: float = ARENA_HEIGHT_M,
    dark_thresh: int = DEFAULT_DARK_THRESH,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Detects arena corners and computes the floor homography if valid.

    Never falls back to image bounds when the black-tape border is absent.

    Args:
        frame_bgr: Input camera image in BGR format.
        arena_width_m: Arena width along world +x in meters.
        arena_height_m: Arena height along world +y in meters.
        dark_thresh: Grayscale intensity threshold separating black tape from floor.

    Returns:
        Tuple of (homography_3x3, corners_4x2_px), or (None, None) on failure.
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return None, None

    gray = cv2.cvtColor(
        frame_bgr,
        cv2.COLOR_BGR2GRAY,
    )
    if int(np.count_nonzero(gray < dark_thresh)) < EMPTY_DARK_PIXEL_MIN_COUNT:
        return None, None

    try:
        corners = detect_arena_corners(
            frame_bgr,
            dark_thresh=dark_thresh,
        )
    except (ValueError, cv2.error):
        return None, None

    height_px, width_px = frame_bgr.shape[:2]
    is_valid_quad = validate_arena_quad(
        corners,
        image_width_px=width_px,
        image_height_px=height_px,
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
    )
    is_image_bounds = _are_corners_image_bounds(
        corners,
        width_px,
        height_px,
    )
    if not is_valid_quad or is_image_bounds:
        return None, None

    homography = compute_homography(
        corners,
        arena_width_m,
        arena_height_m,
    )
    return homography, corners


def _build_hsv_mask(
    hsv_image: np.ndarray,
    ranges: list[tuple[Any, Any]],
) -> np.ndarray:
    """Builds a binary uint8 mask from one or more HSV (lower, upper) bounds."""
    mask = np.zeros(
        hsv_image.shape[:2],
        dtype=np.uint8,
    )
    for lower_b, upper_b in ranges:
        sub_mask = cv2.inRange(
            hsv_image,
            np.asarray(
                lower_b,
                dtype=np.uint8,
            ),
            np.asarray(
                upper_b,
                dtype=np.uint8,
            ),
        )
        mask = cv2.bitwise_or(
            mask,
            sub_mask,
        )
    return mask


def _resolve_corners(
    homography: np.ndarray | None,
    corners_px: np.ndarray | None,
    config: ViewerPipelineConfig,
) -> np.ndarray | None:
    """Returns image corner coordinates for the arena if calibrated."""
    if homography is None:
        return None
    if corners_px is not None:
        return np.asarray(
            corners_px,
            dtype=np.float32,
        ).reshape(QUAD_VERTEX_COUNT, 2)

    world_corners = [
        [0.0, 0.0],
        [config.arena_width_m, 0.0],
        [config.arena_width_m, config.arena_height_m],
        [0.0, config.arena_height_m],
    ]
    return world_to_image(
        homography,
        world_corners,
    ).astype(np.float32)


def _detect_world_cubes(
    frame_bgr: np.ndarray,
    homography: np.ndarray,
    hsv_config: ViewerHsvConfig,
    *,
    pipeline_config: ViewerPipelineConfig,
) -> list[ViewerCubeResult]:
    """Detects cubes outside robot hulls and projects them to parallax-corrected world meters."""
    world_config = _build_world_frame_config(
        hsv_config,
        pipeline_config,
    )
    raw_detections = detect_filtered_cubes(
        frame_bgr,
        config=world_config,
    )
    camera_center_arr = np.asarray(
        pipeline_config.camera_center_world,
        dtype=np.float64,
    )
    results: list[ViewerCubeResult] = []
    for color, u_px, v_px, area_px in raw_detections:
        seen_world = image_to_world(
            homography,
            [[u_px, v_px]],
        )
        floor_xy = correct_parallax(
            seen_world,
            camera_center_arr,
            object_height_m=CUBE_HEIGHT_M,
            cam_height_m=pipeline_config.cam_height_m,
        )
        results.append(
            ViewerCubeResult(
                color=color,
                x_m=float(floor_xy[0, 0]),
                y_m=float(floor_xy[0, 1]),
                area_px=float(area_px),
            )
        )
    return results


def _build_world_frame_config(
    hsv_config: ViewerHsvConfig,
    pipeline_config: ViewerPipelineConfig,
) -> WorldFrameConfig:
    """Builds a WorldFrameConfig matching the node's detection parameters."""
    return WorldFrameConfig(
        hsv_ranges=hsv_config.cube_ranges,
        robot_hsv_ranges=hsv_config.robot_ranges,
        min_area_px=pipeline_config.min_area_px,
        robot_min_area_px=pipeline_config.robot_min_area_px,
        camera_center_world=pipeline_config.camera_center_world,
        cam_height_m=pipeline_config.cam_height_m,
        arena_width_m=pipeline_config.arena_width_m,
        arena_height_m=pipeline_config.arena_height_m,
        robot_names=tuple(hsv_config.robot_ranges.keys()),
        max_area_px=pipeline_config.max_area_px,
        robot_detector=pipeline_config.robot_detector,
        aruco=pipeline_config.aruco,
        triangle_heading_rule=pipeline_config.triangle_heading_rule,
    )


def _detect_world_robots(
    frame_bgr: np.ndarray,
    homography: np.ndarray,
    hsv_config: ViewerHsvConfig,
    *,
    pipeline_config: ViewerPipelineConfig,
) -> list[ViewerRobotResult]:
    """Detects robot markers using the shared world_frame robot detector."""
    world_config = _build_world_frame_config(
        hsv_config,
        pipeline_config,
    )
    poses = detect_robot_poses(
        frame_bgr,
        homography=homography,
        config=world_config,
    )
    return [
        ViewerRobotResult(
            name=pose.name,
            x_m=pose.x,
            y_m=pose.y,
            theta_rad=pose.theta,
        )
        for pose in poses.values()
    ]


def _world_to_rectified_px(
    x_m: float,
    y_m: float,
    config: ViewerPipelineConfig,
) -> tuple[int, int]:
    """Maps world meters (x, y) to rectified image pixel coordinates (u, v)."""
    return world_to_rectified_px(
        x_m,
        y_m,
        arena_height_m=config.arena_height_m,
        px_per_m=config.px_per_m,
        margin_m=config.margin_m,
    )


def _draw_not_calibrated_banner(image: np.ndarray) -> None:
    """Draws a prominent NOT CALIBRATED banner across the top of an image."""
    width_px = image.shape[1]
    cv2.rectangle(
        image,
        (0, 0),
        (width_px, BANNER_HEIGHT_PX),
        COLOR_BGR_BANNER_BG,
        -1,
    )
    cv2.putText(
        image,
        VIEWER_TEXT_NOT_CALIBRATED,
        (BANNER_TEXT_X_PX, BANNER_TEXT_Y_PX),
        cv2.FONT_HERSHEY_SIMPLEX,
        FONT_SCALE_BANNER,
        COLOR_BGR_WHITE,
        DRAW_LINE_THICKNESS_PX,
        cv2.LINE_AA,
    )


def _draw_raw_view(
    frame_bgr: np.ndarray,
    corners_px: np.ndarray | None,
) -> np.ndarray:
    """Draws detected arena corners on the raw camera frame or a NOT CALIBRATED banner."""
    raw_view = frame_bgr.copy()
    if corners_px is None:
        _draw_not_calibrated_banner(raw_view)
        return raw_view

    poly_pts = np.round(corners_px).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(
        raw_view,
        [poly_pts],
        True,
        COLOR_BGR_GREEN,
        DRAW_LINE_THICKNESS_PX,
    )
    for idx, corner in enumerate(corners_px):
        cu = int(round(float(corner[0])))
        cv = int(round(float(corner[1])))
        cv2.circle(
            raw_view,
            (cu, cv),
            CORNER_RADIUS_PX,
            COLOR_BGR_GREEN,
            -1,
        )
        label = VIEWER_CORNER_LABELS[idx] if idx < len(VIEWER_CORNER_LABELS) else str(idx)
        cv2.putText(
            raw_view,
            label,
            (cu + CORNER_LABEL_OFFSET_PX, cv - CORNER_LABEL_OFFSET_PX),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE_LABEL,
            COLOR_BGR_GREEN,
            LABEL_OUTLINE_THICKNESS_PX,
            cv2.LINE_AA,
        )
    return raw_view


def _draw_cube_overlay(
    image: np.ndarray,
    cube: ViewerCubeResult,
    *,
    center_px: tuple[int, int],
) -> None:
    """Draws a cube marker and world cm label on the rectified image."""
    cu, cv = center_px
    bgr = CUBE_COLOR_BGR.get(
        cube.color,
        COLOR_BGR_UNKNOWN,
    )
    cv2.circle(
        image,
        (cu, cv),
        CUBE_MARKER_RADIUS_PX,
        bgr,
        -1,
    )
    cv2.circle(
        image,
        (cu, cv),
        CUBE_MARKER_RADIUS_PX,
        COLOR_BGR_BLACK,
        LABEL_OUTLINE_THICKNESS_PX,
    )
    x_cm = cube.x_m * METERS_TO_CM
    y_cm = cube.y_m * METERS_TO_CM
    cv2.putText(
        image,
        f"{cube.color} ({x_cm:.1f},{y_cm:.1f})cm",
        (cu + LABEL_OFFSET_X_PX, cv + LABEL_OFFSET_Y_PX),
        cv2.FONT_HERSHEY_SIMPLEX,
        FONT_SCALE_LABEL,
        COLOR_BGR_WHITE,
        LABEL_OUTLINE_THICKNESS_PX,
        cv2.LINE_AA,
    )


def _draw_robot_overlay(
    image: np.ndarray,
    robot: ViewerRobotResult,
    *,
    center_px: tuple[int, int],
) -> None:
    """Draws a robot triangle marker, heading arrow, and cm label on the rectified view."""
    ru, rv = center_px
    color = ROBOT_COLOR_BGR.get(
        robot.name,
        COLOR_BGR_GREEN,
    )
    cos_t = math.cos(robot.theta_rad)
    sin_t = math.sin(robot.theta_rad)

    tip = (
        int(round(ru + ROBOT_TRIANGLE_LENGTH_PX * cos_t)),
        int(round(rv - ROBOT_TRIANGLE_LENGTH_PX * sin_t)),
    )
    rear_dist = ROBOT_TRIANGLE_LENGTH_PX * ROBOT_TRIANGLE_REAR_RATIO
    rear_u = ru - rear_dist * cos_t
    rear_v = rv + rear_dist * sin_t
    perp_u = -sin_t * ROBOT_TRIANGLE_HALF_BASE_PX
    perp_v = -cos_t * ROBOT_TRIANGLE_HALF_BASE_PX

    left_corner = (
        int(round(rear_u + perp_u)),
        int(round(rear_v + perp_v)),
    )
    right_corner = (
        int(round(rear_u - perp_u)),
        int(round(rear_v - perp_v)),
    )
    tri_pts = np.array(
        [
            tip,
            left_corner,
            right_corner,
        ],
        dtype=np.int32,
    )
    cv2.polylines(
        image,
        [tri_pts],
        True,
        color,
        DRAW_LINE_THICKNESS_PX,
    )

    arrow_tip = (
        int(round(ru + ROBOT_ARROW_LEN_PX * cos_t)),
        int(round(rv - ROBOT_ARROW_LEN_PX * sin_t)),
    )
    cv2.arrowedLine(
        image,
        (ru, rv),
        arrow_tip,
        color,
        DRAW_LINE_THICKNESS_PX,
        tipLength=ROBOT_ARROW_TIP_RATIO,
    )

    x_cm = robot.x_m * METERS_TO_CM
    y_cm = robot.y_m * METERS_TO_CM
    label = f"{robot.name} ({x_cm:.1f},{y_cm:.1f})cm"
    cv2.putText(
        image,
        label,
        (ru + LABEL_OFFSET_X_PX, rv + LABEL_OFFSET_Y_PX),
        cv2.FONT_HERSHEY_SIMPLEX,
        FONT_SCALE_LABEL,
        color,
        LABEL_OUTLINE_THICKNESS_PX,
        cv2.LINE_AA,
    )


def _draw_rectified_view(
    frame_bgr: np.ndarray,
    homography: np.ndarray | None,
    *,
    cubes: list[ViewerCubeResult],
    robots: list[ViewerRobotResult],
    config: ViewerPipelineConfig,
    safety_result: ViewerSafetyResult | None = None,
) -> np.ndarray:
    """Renders the top-down rectified view with annotated cubes and robots."""
    out_w = max(
        1,
        int(round((config.arena_width_m + 2.0 * config.margin_m) * config.px_per_m)),
    )
    out_h = max(
        1,
        int(round((config.arena_height_m + 2.0 * config.margin_m) * config.px_per_m)),
    )
    if homography is None:
        placeholder = np.zeros(
            (out_h, out_w, 3),
            dtype=np.uint8,
        )
        _draw_not_calibrated_banner(placeholder)
        cv2.putText(
            placeholder,
            VIEWER_TEXT_CALIBRATION_HINT,
            (PLACEHOLDER_TEXT_X_PX, PLACEHOLDER_HINT_Y_PX),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE_HINT,
            COLOR_BGR_WHITE,
            LABEL_OUTLINE_THICKNESS_PX,
            cv2.LINE_AA,
        )
        return placeholder

    rectified = rectify(
        frame_bgr,
        homography,
        config.px_per_m,
        arena_width_m=config.arena_width_m,
        arena_height_m=config.arena_height_m,
        margin_m=config.margin_m,
    )

    for cube in cubes:
        center_px = _world_to_rectified_px(
            cube.x_m,
            cube.y_m,
            config,
        )
        _draw_cube_overlay(
            rectified,
            cube,
            center_px=center_px,
        )

    for robot in robots:
        center_px = _world_to_rectified_px(
            robot.x_m,
            robot.y_m,
            config,
        )
        _draw_robot_overlay(
            rectified,
            robot,
            center_px=center_px,
        )

    if safety_result is not None:
        rectified = draw_safety_overlay(
            rectified,
            safety_result.safety,
            safety_result.robot_poses,
            config=SafetyOverlayConfig(
                arena_height_m=config.arena_height_m,
                px_per_m=config.px_per_m,
                margin_m=config.margin_m,
                zone_rect=DEFAULT_ZONE_RECT,
            ),
        )

    return rectified


def _draw_mask_view(
    frame_bgr: np.ndarray,
    hsv_image: np.ndarray,
    cube_ranges: HsvBoundsMap,
    *,
    is_calibrated: bool,
) -> np.ndarray:
    """Overlays per-cube-color HSV binary masks on a dimmed raw frame."""
    mask_view = cv2.convertScaleAbs(
        frame_bgr,
        alpha=MASK_BG_DIM_ALPHA,
    )
    for color_name, ranges in cube_ranges.items():
        if not ranges:
            continue
        color_mask = _build_hsv_mask(
            hsv_image,
            ranges,
        )
        bgr = CUBE_COLOR_BGR.get(
            color_name,
            COLOR_BGR_UNKNOWN,
        )
        is_hit = color_mask > 0
        if np.any(is_hit):
            overlay = np.empty_like(mask_view[is_hit])
            overlay[:] = bgr
            blended = (
                mask_view[is_hit].astype(np.float32) * (1.0 - MASK_OVERLAY_ALPHA)
                + overlay.astype(np.float32) * MASK_OVERLAY_ALPHA
            )
            mask_view[is_hit] = blended.astype(np.uint8)

        contours, _ = cv2.findContours(
            color_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(
            mask_view,
            contours,
            -1,
            bgr,
            DRAW_LINE_THICKNESS_PX,
        )

    if not is_calibrated:
        _draw_not_calibrated_banner(mask_view)
        cv2.putText(
            mask_view,
            VIEWER_TEXT_CALIBRATION_HINT,
            (PLACEHOLDER_TEXT_X_PX, PLACEHOLDER_TEXT_Y_PX),
            cv2.FONT_HERSHEY_SIMPLEX,
            FONT_SCALE_HINT,
            COLOR_BGR_WHITE,
            LABEL_OUTLINE_THICKNESS_PX,
            cv2.LINE_AA,
        )

    return mask_view


def process_frame(
    frame_bgr: np.ndarray,
    homography: np.ndarray | None,
    hsv_config: ViewerHsvConfig | HsvBoundsMap | None = None,
    *,
    corners_px: np.ndarray | None = None,
    config: ViewerPipelineConfig | None = None,
    safety: ViewerSafety | None = None,
    now_s: float | None = None,
) -> PipelineOutput:
    """Runs detection and renders raw, rectified, and mask views for one frame.

    When homography is None, no world coordinates are computed and all three views
    indicate that the camera is not calibrated.

    Args:
        frame_bgr: Input BGR image frame.
        homography: 3x3 pixel-to-world homography matrix, or None if uncalibrated.
        hsv_config: HSV ranges config or cube HSV dictionary.
        corners_px: Optional 4x2 arena corner pixels for raw view overlay.
        config: Optional geometry, parallax, and rendering configuration.
        safety: Optional stateful safety holder for velocity, hold, and radar checks.
        now_s: Optional monotonic timestamp in seconds passed to safety evaluation.

    Returns:
        PipelineOutput containing annotated view images and detected world objects.
    """
    normalized_hsv = _normalize_hsv_config(hsv_config)
    pipeline_cfg = config if config is not None else ViewerPipelineConfig()
    is_calibrated = homography is not None
    resolved_corners = _resolve_corners(
        homography,
        corners_px,
        pipeline_cfg,
    )
    hsv_image = cv2.cvtColor(
        frame_bgr,
        cv2.COLOR_BGR2HSV,
    )

    cubes: list[ViewerCubeResult] = []
    robots: list[ViewerRobotResult] = []
    if homography is not None:
        cubes = _detect_world_cubes(
            frame_bgr,
            homography,
            normalized_hsv,
            pipeline_config=pipeline_cfg,
        )
        robots = _detect_world_robots(
            frame_bgr,
            homography,
            normalized_hsv,
            pipeline_config=pipeline_cfg,
        )

    safety_result: ViewerSafetyResult | None = None
    if homography is not None and safety is not None:
        robot_poses = [
            RobotPose(
                name=robot.name,
                x=robot.x_m,
                y=robot.y_m,
                theta=robot.theta_rad,
                confidence=DEFAULT_DETECTION_CONFIDENCE,
            )
            for robot in robots
        ]
        cube_tuples = [
            (idx, cube.color, cube.x_m, cube.y_m, DEFAULT_DETECTION_CONFIDENCE)
            for idx, cube in enumerate(cubes)
        ]
        safety_result = safety.update(
            robot_poses,
            cube_tuples,
            now_s=now_s,
        )

    views = {
        ViewerView.RAW.value: _draw_raw_view(
            frame_bgr,
            resolved_corners,
        ),
        ViewerView.RECTIFIED.value: _draw_rectified_view(
            frame_bgr,
            homography,
            cubes=cubes,
            robots=robots,
            config=pipeline_cfg,
            safety_result=safety_result,
        ),
        ViewerView.MASK.value: _draw_mask_view(
            frame_bgr,
            hsv_image,
            normalized_hsv.cube_ranges,
            is_calibrated=is_calibrated,
        ),
    }

    return PipelineOutput(
        views=views,
        cubes=cubes,
        robots=robots,
        is_calibrated=is_calibrated,
        corners_px=resolved_corners,
        safety=safety_result,
    )


def build_views(
    frame_bgr: np.ndarray,
    homography: np.ndarray | None,
    hsv_config: ViewerHsvConfig | HsvBoundsMap | None = None,
) -> dict[str, np.ndarray]:
    """Convenience helper returning only the annotated view dictionary for a frame.

    Args:
        frame_bgr: Input BGR image frame.
        homography: 3x3 pixel-to-world homography matrix, or None if uncalibrated.
        hsv_config: HSV ranges config or cube HSV dictionary.

    Returns:
        Mapping from view name ('raw', 'rectified', 'mask') to BGR image arrays.
    """
    return process_frame(
        frame_bgr,
        homography,
        hsv_config,
    ).views
