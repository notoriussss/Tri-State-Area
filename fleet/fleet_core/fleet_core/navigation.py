"""Guided push geometry, red-cube avoidance (D7), two-stage break, and border sweep."""

import math
from dataclasses import dataclass

from fleet_core.constants import (
    ARENA_CENTER_X_M,
    ARENA_CENTER_Y_M,
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    DEFAULT_APPROACH_OFFSET_M,
    DEFAULT_BORDER_SWEEP_INWARD_M,
    DEFAULT_BREAK_FRACTIONS,
    DEFAULT_BREAK_OFFSETS_M,
    DEFAULT_CORRIDOR_HALF_WIDTH_M,
    DEFAULT_TRANSIT_DETOUR_MARGIN_M,
    MAX_TRANSIT_SEGMENTS,
    POINT_EPSILON_M,
    RED_TRANSIT_RADIUS_M,
    VIRTUAL_BORDER_MARGIN_M,
    ZONE_MAX_X_M,
    ZONE_MAX_Y_M,
    ZONE_MIN_Y_M,
    PushMode,
)
from fleet_core.math_utils import wrap_angle
from fleet_core.types import Pose2D

Point2D = tuple[float, float]
CircularObstacle = tuple[float, float, float]


@dataclass(frozen=True)
class NavigationConfig:
    """Geometric clearances and radii for transit and guided cube pushing.

    Setting red_transit_radius_m to 0.12 m enables the D7 two-step variant
    (12 cm transit + clear push corridor); setting it to 0.17 m reproduces the
    full safety radius variant for comparison in simulation.
    """

    border_margin_m: float = VIRTUAL_BORDER_MARGIN_M
    red_transit_radius_m: float = RED_TRANSIT_RADIUS_M
    corridor_half_width_m: float = DEFAULT_CORRIDOR_HALF_WIDTH_M
    approach_offset_m: float = DEFAULT_APPROACH_OFFSET_M
    break_offsets_m: tuple[float, ...] = DEFAULT_BREAK_OFFSETS_M
    break_fractions: tuple[float, ...] = DEFAULT_BREAK_FRACTIONS
    border_sweep_inward_m: float = DEFAULT_BORDER_SWEEP_INWARD_M
    detour_margin_m: float = DEFAULT_TRANSIT_DETOUR_MARGIN_M


@dataclass(frozen=True)
class PushPlan:
    """Result of guided push planning for one cube toward a destination."""

    mode: PushMode
    approach_pose: Pose2D | None
    waypoints: tuple[Point2D, ...]


def is_inside_virtual_margin(
    point_xy: Point2D,
    *,
    margin_m: float = VIRTUAL_BORDER_MARGIN_M,
    allow_zone_entry: bool = False,
) -> bool:
    """Returns True when a point stays safely inside the black border tape margin.

    Args:
        point_xy: Position (x_m, y_m) in the arena frame.
        margin_m: Virtual interior inset from the black border tape in meters.
        allow_zone_entry: Permits crossing into the right-side delivery notch.

    Returns:
        True if the point is within legal drivable bounds.
    """
    x_m, y_m = point_xy
    is_in_main_box = (
        margin_m <= x_m <= ARENA_WIDTH_M - margin_m
        and margin_m <= y_m <= ARENA_HEIGHT_M - margin_m
    )
    if is_in_main_box:
        return True

    if not allow_zone_entry:
        return False

    is_in_delivery_notch = (
        ARENA_WIDTH_M - margin_m <= x_m <= ZONE_MAX_X_M
        and ZONE_MIN_Y_M <= y_m <= ZONE_MAX_Y_M
    )
    return is_in_delivery_notch


def distance_point_to_segment(
    point_xy: Point2D,
    start_xy: Point2D,
    end_xy: Point2D,
) -> float:
    """Computes Euclidean distance in meters from a point to a 2D line segment.

    Args:
        point_xy: Query point (x_m, y_m).
        start_xy: Segment start (x_m, y_m).
        end_xy: Segment end (x_m, y_m).

    Returns:
        Shortest Euclidean distance in meters.
    """
    px, py = point_xy
    sx, sy = start_xy
    ex, ey = end_xy
    dx = ex - sx
    dy = ey - sy
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= POINT_EPSILON_M * POINT_EPSILON_M:
        return math.hypot(
            px - sx,
            py - sy,
        )

    proj = ((px - sx) * dx + (py - sy) * dy) / seg_len_sq
    t = max(
        0.0,
        min(
            1.0,
            proj,
        ),
    )
    closest_x = sx + t * dx
    closest_y = sy + t * dy
    return math.hypot(
        px - closest_x,
        py - closest_y,
    )


def compute_approach_pose(
    cube_xy: Point2D,
    goal_xy: Point2D,
    *,
    offset_m: float = DEFAULT_APPROACH_OFFSET_M,
) -> Pose2D:
    """Computes the approach pose A behind cube C along the line toward goal G.

    Formula: A = C + d * (C - G) / |C - G|, with heading pointing from C to G.

    Args:
        cube_xy: Cube center (x_m, y_m) in arena coordinates.
        goal_xy: Target destination (x_m, y_m) in arena coordinates.
        offset_m: Stand-off distance d behind the cube in meters.

    Returns:
        Approach Pose2D behind the cube facing goal_xy.
    """
    cx, cy = cube_xy
    gx, gy = goal_xy
    dx = gx - cx
    dy = gy - cy
    dist = math.hypot(
        dx,
        dy,
    )
    if dist <= POINT_EPSILON_M:
        return Pose2D(
            x_m=cx - offset_m,
            y_m=cy,
            theta_rad=0.0,
        )

    ux = dx / dist
    uy = dy / dist
    return Pose2D(
        x_m=cx - offset_m * ux,
        y_m=cy - offset_m * uy,
        theta_rad=wrap_angle(math.atan2(dy, dx)),
    )


def is_corridor_clear(
    start_xy: Point2D,
    end_xy: Point2D,
    red_positions: tuple[Point2D, ...],
    *,
    half_width_m: float = DEFAULT_CORRIDOR_HALF_WIDTH_M,
) -> bool:
    """Returns True when no red cube intrudes inside the straight push corridor.

    Args:
        start_xy: Push start point (approach or cube position) in meters.
        end_xy: Push destination point in meters.
        red_positions: Floor centers (x_m, y_m) of all visible red cubes.
        half_width_m: Half-width of the scoop sweep corridor in meters.

    Returns:
        True if every red cube stays at least half_width_m away from the segment.
    """
    for red_xy in red_positions:
        dist_m = distance_point_to_segment(
            red_xy,
            start_xy,
            end_xy,
        )
        if dist_m < half_width_m:
            return False

    return True


def is_point_clear_of_obstacles(
    point_xy: Point2D,
    obstacles: tuple[CircularObstacle, ...],
) -> bool:
    """Returns True when point_xy lies outside every circular obstacle."""
    px, py = point_xy
    for ox, oy, radius_m in obstacles:
        if math.hypot(px - ox, py - oy) < radius_m:
            return False

    return True


def plan_push(
    cube_xy: Point2D,
    goal_xy: Point2D,
    red_positions: tuple[Point2D, ...],
    *,
    config: NavigationConfig | None = None,
) -> PushPlan:
    """Plans a direct push, border sweep, or two-stage break (C -> M -> G).

    Args:
        cube_xy: Target cube center in arena coordinates (m).
        goal_xy: Desired push destination (e.g. pre-zone line) in meters.
        red_positions: Visible red cube centers in arena coordinates (m).
        config: Navigation clearances and radii.

    Returns:
        PushPlan describing the selected mode, approach pose, and push waypoints.
    """
    cfg = config or NavigationConfig()
    red_obstacles = tuple(
        (
            rx,
            ry,
            cfg.red_transit_radius_m,
        )
        for rx, ry in red_positions
    )
    direct_approach = compute_approach_pose(
        cube_xy,
        goal_xy,
        offset_m=cfg.approach_offset_m,
    )
    approach_xy = (
        direct_approach.x_m,
        direct_approach.y_m,
    )

    if not is_inside_virtual_margin(approach_xy, margin_m=cfg.border_margin_m):
        return _plan_border_sweep(
            cube_xy,
            goal_xy,
            red_positions,
            red_obstacles=red_obstacles,
            config=cfg,
        )

    is_direct_valid = is_point_clear_of_obstacles(
        approach_xy,
        red_obstacles,
    ) and is_corridor_clear(
        approach_xy,
        goal_xy,
        red_positions,
        half_width_m=cfg.corridor_half_width_m,
    )
    if is_direct_valid:
        return PushPlan(
            mode=PushMode.DIRECT,
            approach_pose=direct_approach,
            waypoints=(goal_xy,),
        )

    return _plan_two_stage_break(
        cube_xy,
        goal_xy,
        red_positions,
        red_obstacles=red_obstacles,
        config=cfg,
    )


def plan_transit_waypoints(
    start_xy: Point2D,
    goal_xy: Point2D,
    obstacles: tuple[CircularObstacle, ...],
    *,
    config: NavigationConfig | None = None,
) -> tuple[Point2D, ...]:
    """Plans collision-free transit waypoints around circular obstacles.

    Args:
        start_xy: Current robot center in arena coordinates (m).
        goal_xy: Desired transit destination (e.g. approach point A) in meters.
        obstacles: Circular obstacles as (x_m, y_m, radius_m).
        config: Border margin and detour clearance parameters.

    Returns:
        Ordered tuple of waypoints ending at goal_xy, or empty tuple if blocked.
    """
    cfg = config or NavigationConfig()
    waypoints: list[Point2D] = []
    current = start_xy

    for _ in range(MAX_TRANSIT_SEGMENTS):
        blocker = _first_blocking_obstacle(
            current,
            goal_xy,
            obstacles,
        )
        if blocker is None:
            waypoints.append(goal_xy)
            return tuple(waypoints)

        detour = _pick_detour_point(
            current,
            goal_xy,
            blocker,
            obstacles=obstacles,
            config=cfg,
        )
        if detour is None:
            return ()

        waypoints.append(detour)
        current = detour

    is_final_clear = _first_blocking_obstacle(current, goal_xy, obstacles) is None
    if is_final_clear:
        waypoints.append(goal_xy)
        return tuple(waypoints)

    return ()


def _plan_border_sweep(
    cube_xy: Point2D,
    goal_xy: Point2D,
    red_positions: tuple[Point2D, ...],
    *,
    red_obstacles: tuple[CircularObstacle, ...],
    config: NavigationConfig,
) -> PushPlan:
    """Computes an angled inward sweep to unstick a cube near the black tape."""
    cx, cy = cube_xy
    to_center_x = ARENA_CENTER_X_M - cx
    to_center_y = ARENA_CENTER_Y_M - cy
    norm = math.hypot(
        to_center_x,
        to_center_y,
    )
    if norm <= POINT_EPSILON_M:
        return PushPlan(
            mode=PushMode.UNREACHABLE,
            approach_pose=None,
            waypoints=(),
        )

    ux = to_center_x / norm
    uy = to_center_y / norm
    inward_target = (
        cx + ux * config.border_sweep_inward_m,
        cy + uy * config.border_sweep_inward_m,
    )
    # Try sweeping from the side perpendicular to the wall toward the inward point.
    perpendiculars = (
        (-uy, ux),
        (uy, -ux),
    )
    for px, py in perpendiculars:
        sweep_start = (
            cx + px * config.approach_offset_m + ux * (config.border_margin_m / 2.0),
            cy + py * config.approach_offset_m + uy * (config.border_margin_m / 2.0),
        )
        is_valid = (
            is_inside_virtual_margin(sweep_start, margin_m=config.border_margin_m)
            and is_point_clear_of_obstacles(sweep_start, red_obstacles)
            and is_corridor_clear(
                sweep_start,
                inward_target,
                red_positions,
                half_width_m=config.corridor_half_width_m,
            )
        )
        if is_valid:
            heading = wrap_angle(
                math.atan2(
                    inward_target[1] - sweep_start[1],
                    inward_target[0] - sweep_start[0],
                )
            )
            return PushPlan(
                mode=PushMode.BORDER_SWEEP,
                approach_pose=Pose2D(
                    x_m=sweep_start[0],
                    y_m=sweep_start[1],
                    theta_rad=heading,
                ),
                waypoints=(
                    inward_target,
                    goal_xy,
                ),
            )

    return PushPlan(
        mode=PushMode.UNREACHABLE,
        approach_pose=None,
        waypoints=(),
    )


def _plan_two_stage_break(
    cube_xy: Point2D,
    goal_xy: Point2D,
    red_positions: tuple[Point2D, ...],
    *,
    red_obstacles: tuple[CircularObstacle, ...],
    config: NavigationConfig,
) -> PushPlan:
    """Finds the shortest safe intermediate point M for a two-stage push C -> M -> G."""
    cx, cy = cube_xy
    gx, gy = goal_xy
    dx = gx - cx
    dy = gy - cy
    dist = math.hypot(
        dx,
        dy,
    )
    if dist <= POINT_EPSILON_M:
        return PushPlan(
            mode=PushMode.UNREACHABLE,
            approach_pose=None,
            waypoints=(),
        )

    perp_x = -dy / dist
    perp_y = dx / dist
    candidates = [
        (
            cx + frac * dx + sign * offset_m * perp_x,
            cy + frac * dy + sign * offset_m * perp_y,
        )
        for frac in config.break_fractions
        for offset_m in config.break_offsets_m
        for sign in (1.0, -1.0)
    ]

    best_plan: PushPlan | None = None
    best_cost = math.inf

    for candidate in candidates:
        plan, cost = _evaluate_break_candidate(
            cube_xy,
            candidate,
            goal_xy,
            red_positions=red_positions,
            red_obstacles=red_obstacles,
            config=config,
        )
        if plan is not None and cost < best_cost:
            best_plan = plan
            best_cost = cost

    if best_plan is not None:
        return best_plan

    return PushPlan(
        mode=PushMode.UNREACHABLE,
        approach_pose=None,
        waypoints=(),
    )


def _evaluate_break_candidate(
    cube_xy: Point2D,
    mid_xy: Point2D,
    goal_xy: Point2D,
    *,
    red_positions: tuple[Point2D, ...],
    red_obstacles: tuple[CircularObstacle, ...],
    config: NavigationConfig,
) -> tuple[PushPlan | None, float]:
    """Validates both stages (C -> M and M -> G) of a break candidate."""
    if not is_inside_virtual_margin(mid_xy, margin_m=config.border_margin_m):
        return None, math.inf

    if not is_point_clear_of_obstacles(mid_xy, red_obstacles):
        return None, math.inf

    stage1_approach = compute_approach_pose(
        cube_xy,
        mid_xy,
        offset_m=config.approach_offset_m,
    )
    stage2_approach = compute_approach_pose(
        mid_xy,
        goal_xy,
        offset_m=config.approach_offset_m,
    )
    a1_xy = (
        stage1_approach.x_m,
        stage1_approach.y_m,
    )
    a2_xy = (
        stage2_approach.x_m,
        stage2_approach.y_m,
    )

    is_stage1_ok = (
        is_inside_virtual_margin(a1_xy, margin_m=config.border_margin_m)
        and is_point_clear_of_obstacles(a1_xy, red_obstacles)
        and is_corridor_clear(
            a1_xy,
            mid_xy,
            red_positions,
            half_width_m=config.corridor_half_width_m,
        )
    )
    is_stage2_ok = (
        is_inside_virtual_margin(a2_xy, margin_m=config.border_margin_m)
        and is_point_clear_of_obstacles(a2_xy, red_obstacles)
        and is_corridor_clear(
            a2_xy,
            goal_xy,
            red_positions,
            half_width_m=config.corridor_half_width_m,
        )
    )
    if not (is_stage1_ok and is_stage2_ok):
        return None, math.inf

    total_cost = math.hypot(
        mid_xy[0] - cube_xy[0],
        mid_xy[1] - cube_xy[1],
    ) + math.hypot(
        goal_xy[0] - mid_xy[0],
        goal_xy[1] - mid_xy[1],
    )
    plan = PushPlan(
        mode=PushMode.TWO_STAGE,
        approach_pose=stage1_approach,
        waypoints=(
            mid_xy,
            goal_xy,
        ),
    )
    return plan, total_cost


def _first_blocking_obstacle(
    start_xy: Point2D,
    end_xy: Point2D,
    obstacles: tuple[CircularObstacle, ...],
) -> CircularObstacle | None:
    """Returns the blocking obstacle closest to start_xy along the segment."""
    closest_obs: CircularObstacle | None = None
    closest_dist = math.inf

    for obs in obstacles:
        ox, oy, radius_m = obs
        seg_dist = distance_point_to_segment(
            (ox, oy),
            start_xy,
            end_xy,
        )
        if seg_dist < radius_m:
            dist_from_start = math.hypot(
                ox - start_xy[0],
                oy - start_xy[1],
            )
            if dist_from_start < closest_dist:
                closest_dist = dist_from_start
                closest_obs = obs

    return closest_obs


def _pick_detour_point(
    start_xy: Point2D,
    goal_xy: Point2D,
    blocker: CircularObstacle,
    *,
    obstacles: tuple[CircularObstacle, ...],
    config: NavigationConfig,
) -> Point2D | None:
    """Chooses the shorter valid lateral detour point around a blocking obstacle."""
    dx = goal_xy[0] - start_xy[0]
    dy = goal_xy[1] - start_xy[1]
    dist = math.hypot(
        dx,
        dy,
    )
    if dist <= POINT_EPSILON_M:
        return None

    perp_x = -dy / dist
    perp_y = dx / dist
    ox, oy, radius_m = blocker
    clearance_m = radius_m + config.detour_margin_m

    best_point: Point2D | None = None
    best_length = math.inf

    for sign in (1.0, -1.0):
        candidate = (
            ox + sign * clearance_m * perp_x,
            oy + sign * clearance_m * perp_y,
        )
        is_valid = (
            is_inside_virtual_margin(candidate, margin_m=config.border_margin_m)
            and is_point_clear_of_obstacles(candidate, obstacles)
        )
        if not is_valid:
            continue

        total_length = math.hypot(
            candidate[0] - start_xy[0],
            candidate[1] - start_xy[1],
        ) + math.hypot(
            goal_xy[0] - candidate[0],
            goal_xy[1] - candidate[1],
        )
        if total_length < best_length:
            best_length = total_length
            best_point = candidate

    return best_point
