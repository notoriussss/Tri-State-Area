"""Pure geometric transforms between image pixels, world meters, and robot frames."""

import math

import cv2
import numpy as np

from overhead_core.constants import (
    APPROX_EPSILON_GROWTH,
    APPROX_EPSILON_RATIO,
    QUAD_VERTEX_COUNT,
    RECTIFY_MARGIN_M,
)


def compute_homography(
    image_corners: np.ndarray | list[list[float]],
    arena_width_m: float,
    arena_height_m: float,
) -> np.ndarray:
    """Computes 3x3 homography mapping image_corners [BL, BR, TR, TL] to world meters.

    Args:
        image_corners: 4x2 array or list of corner pixel coordinates.
        arena_width_m: Arena width in meters along the +x axis.
        arena_height_m: Arena height in meters along the +y axis.

    Returns:
        3x3 perspective homography matrix mapping image pixels to world meters.
    """
    src = np.asarray(
        image_corners,
        dtype=np.float32,
    ).reshape(QUAD_VERTEX_COUNT, 2)
    dst = np.array(
        [
            [0.0, 0.0],
            [arena_width_m, 0.0],
            [arena_width_m, arena_height_m],
            [0.0, arena_height_m],
        ],
        dtype=np.float32,
    )
    return cv2.getPerspectiveTransform(src, dst)


def image_to_world(
    homography: np.ndarray,
    pts_uv: np.ndarray | list[list[float]],
) -> np.ndarray:
    """Transforms Nx2 pixel coordinates (u, v) to Nx2 world coordinates (x, y) in meters.

    Args:
        homography: 3x3 homography matrix mapping pixels to world meters.
        pts_uv: Nx2 array or list of pixel coordinates.

    Returns:
        Nx2 world coordinates in meters.
    """
    pts = np.asarray(
        pts_uv,
        dtype=np.float32,
    ).reshape(-1, 1, 2)
    if pts.size == 0:
        return np.empty(
            (0, 2),
            dtype=np.float64,
        )
    world = cv2.perspectiveTransform(
        pts,
        homography,
    )
    return world.reshape(-1, 2).astype(np.float64)


def world_to_image(
    homography: np.ndarray,
    pts_xy: np.ndarray | list[list[float]],
) -> np.ndarray:
    """Transforms Nx2 world coordinates (x, y) in meters to Nx2 pixel coordinates (u, v).

    Args:
        homography: 3x3 homography matrix mapping pixels to world meters.
        pts_xy: Nx2 array or list of world coordinates in meters.

    Returns:
        Nx2 pixel coordinates.
    """
    pts = np.asarray(
        pts_xy,
        dtype=np.float32,
    ).reshape(-1, 1, 2)
    if pts.size == 0:
        return np.empty(
            (0, 2),
            dtype=np.float64,
        )
    inverse_homography = np.linalg.inv(homography)
    uv = cv2.perspectiveTransform(
        pts,
        inverse_homography,
    )
    return uv.reshape(-1, 2).astype(np.float64)


def wrap_angle_rad(angle_rad: float) -> float:
    """Wraps an angle in radians to the half-open interval (-pi, pi].

    Args:
        angle_rad: Angle in radians.

    Returns:
        Equivalent angle in radians within (-pi, pi].
    """
    normalized = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    if normalized <= -math.pi:
        normalized += 2.0 * math.pi
    return float(normalized)


def world_to_robot(
    robot_pose: tuple[float, float, float],
    point_xy: tuple[float, float],
) -> tuple[float, float, float, float]:
    """Expresses world point (px, py) in REP 103 robot frame (x forward, y left).

    Args:
        robot_pose: Robot pose (x, y, theta) in world meters and radians.
        point_xy: World point (px, py) in meters.

    Returns:
        Tuple of (x_robot, y_robot, distance, bearing) in robot frame.
    """
    robot_x, robot_y, robot_theta = robot_pose
    px, py = point_xy
    dx = px - robot_x
    dy = py - robot_y
    c = math.cos(robot_theta)
    s = math.sin(robot_theta)
    x_robot = c * dx + s * dy
    y_robot = -s * dx + c * dy
    distance = math.hypot(
        x_robot,
        y_robot,
    )
    bearing = math.atan2(
        y_robot,
        x_robot,
    )
    return x_robot, y_robot, distance, bearing


def rectify(
    image: np.ndarray,
    homography: np.ndarray,
    px_per_m: float,
    *,
    arena_width_m: float | None = None,
    arena_height_m: float | None = None,
    margin_m: float = RECTIFY_MARGIN_M,
) -> np.ndarray:
    """Warps camera image to a top-down view of the arena with world +y pointing up.

    margin_m adds a band around the arena so the black tape border (outside the inner
    corners used for homography) stays visible. World (x, y) lands at pixel
    ((x + margin_m) * px_per_m, (arena_height_m + margin_m - y) * px_per_m).

    Args:
        image: Input camera image.
        homography: 3x3 homography matrix mapping pixels to world meters.
        px_per_m: Scale factor in pixels per meter for rectified image.
        arena_width_m: Arena width in meters (derived from bounds if None).
        arena_height_m: Arena height in meters (derived from bounds if None).
        margin_m: Outer margin in meters around the arena perimeter.

    Returns:
        Rectified top-down arena view image.
    """
    if arena_width_m is None or arena_height_m is None:
        h_img, w_img = image.shape[:2]
        img_box = np.array(
            [
                [0.0, float(h_img)],
                [float(w_img), float(h_img)],
                [float(w_img), 0.0],
                [0.0, 0.0],
            ],
            dtype=np.float32,
        )
        world_box = image_to_world(
            homography,
            img_box,
        )
        if arena_width_m is None:
            arena_width_m = float(np.max(world_box[:, 0]))
        if arena_height_m is None:
            arena_height_m = float(np.max(world_box[:, 1]))

    out_w = max(
        1,
        int(round((arena_width_m + 2.0 * margin_m) * px_per_m)),
    )
    out_h = max(
        1,
        int(round((arena_height_m + 2.0 * margin_m) * px_per_m)),
    )
    scale_flip = np.array(
        [
            [px_per_m, 0.0, margin_m * px_per_m],
            [0.0, -px_per_m, (arena_height_m + margin_m) * px_per_m],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return cv2.warpPerspective(
        image,
        scale_flip @ homography,
        (out_w, out_h),
    )


def detect_arena_corners(
    image: np.ndarray,
    dark_thresh: int = 70,
) -> np.ndarray:
    """Finds the 4 inner corners of the black-tape border, ordered [BL, BR, TR, TL] (pixels).

    Takes the non-dark region containing the image center, its convex hull, and simplifies
    it to a quadrilateral. Tolerates the opening of the delivery area on one side.

    Args:
        image: Input arena image (BGR).
        dark_thresh: Grayscale intensity threshold separating black tape from arena floor.

    Returns:
        4x2 array of corner pixel coordinates ordered [BL, BR, TR, TL].

    Raises:
        ValueError: If a quadrilateral border cannot be determined.
    """
    h, w = image.shape[:2]
    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY,
    )
    dark = (gray < dark_thresh).astype(np.uint8) * 255
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_CLOSE,
        np.ones(
            (15, 15),
            np.uint8,
        ),
    )
    _, labels = cv2.connectedComponents(255 - dark)
    inner = (labels == labels[h // 2, w // 2]).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        inner,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    eps = APPROX_EPSILON_RATIO * cv2.arcLength(
        hull,
        True,
    )
    quad = cv2.approxPolyDP(
        hull,
        eps,
        True,
    )
    while len(quad) > QUAD_VERTEX_COUNT:
        eps *= APPROX_EPSILON_GROWTH
        quad = cv2.approxPolyDP(
            hull,
            eps,
            True,
        )
    if len(quad) != QUAD_VERTEX_COUNT:
        raise ValueError("arena border not found")
    q = quad.reshape(
        QUAD_VERTEX_COUNT,
        2,
    ).astype(np.float32)
    s, d = q.sum(1), np.diff(q, axis=1).ravel()
    return np.array(
        [
            q[d.argmax()],
            q[s.argmax()],
            q[d.argmin()],
            q[s.argmin()],
        ],
        dtype=np.float32,
    )
