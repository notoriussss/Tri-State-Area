"""Distributed per-robot mission selection with cost penalties, hysteresis, and parking."""

import math
from dataclasses import dataclass, field

from fleet_core.constants import (
    DEFAULT_BORDER_PENALTY_M,
    DEFAULT_CROSS_PENALTY_M,
    DEFAULT_HYSTERESIS_BONUS_M,
    DEFAULT_PARK_LOWER_XY,
    DEFAULT_PARK_UPPER_XY,
    DEFAULT_PEER_CROSS_THRESHOLD_M,
    DEFAULT_PRE_ZONE_LOWER_Y_M,
    DEFAULT_PRE_ZONE_UPPER_Y_M,
    DEFAULT_RED_PENALTY_M,
    PEER_PLAN_TIMEOUT_S,
    PRE_ZONE_X_M,
    ZONE_GREEN_LINE_X_M,
    ZONE_MAX_Y_M,
    ZONE_MIN_Y_M,
    CubeColor,
    PushMode,
    RobotName,
)
from fleet_core.navigation import (
    NavigationConfig,
    Point2D,
    PushPlan,
    distance_point_to_segment,
    plan_push,
)
from fleet_core.types import (
    CubeObservation,
    PlanSnapshot,
    Pose2D,
)


@dataclass(frozen=True)
class MissionConfig:
    """Parameters governing distributed cube selection and end-of-color parking."""

    robot_name: str
    target_color: CubeColor
    pre_zone_goal_xy: Point2D
    park_pose: Pose2D
    cross_penalty_m: float = DEFAULT_CROSS_PENALTY_M
    border_penalty_m: float = DEFAULT_BORDER_PENALTY_M
    red_penalty_m: float = DEFAULT_RED_PENALTY_M
    hysteresis_bonus_m: float = DEFAULT_HYSTERESIS_BONUS_M
    peer_plan_timeout_s: float = PEER_PLAN_TIMEOUT_S
    peer_cross_threshold_m: float = DEFAULT_PEER_CROSS_THRESHOLD_M
    navigation: NavigationConfig = field(default_factory=NavigationConfig)


def default_mission_config(robot_name: str) -> MissionConfig:
    """Builds the default mission configuration for Phineas (blue) or Ferb (yellow).

    Args:
        robot_name: Collector identifier ("phineas" or "ferb").

    Returns:
        MissionConfig with default color, pre-zone lane, and parking corner.
    """
    if robot_name == RobotName.PHINEAS.value:
        return MissionConfig(
            robot_name=robot_name,
            target_color=CubeColor.BLUE,
            pre_zone_goal_xy=(
                PRE_ZONE_X_M,
                DEFAULT_PRE_ZONE_UPPER_Y_M,
            ),
            park_pose=Pose2D(
                x_m=DEFAULT_PARK_UPPER_XY[0],
                y_m=DEFAULT_PARK_UPPER_XY[1],
                theta_rad=0.0,
            ),
        )

    return MissionConfig(
        robot_name=robot_name,
        target_color=CubeColor.YELLOW,
        pre_zone_goal_xy=(
            PRE_ZONE_X_M,
            DEFAULT_PRE_ZONE_LOWER_Y_M,
        ),
        park_pose=Pose2D(
            x_m=DEFAULT_PARK_LOWER_XY[0],
            y_m=DEFAULT_PARK_LOWER_XY[1],
            theta_rad=0.0,
        ),
    )


@dataclass(frozen=True)
class MissionDecision:
    """Outcome of one distributed mission evaluation."""

    target_cube: CubeObservation | None
    push_plan: PushPlan | None
    cost_m: float
    should_park: bool


def is_cube_in_delivery_zone(cube: CubeObservation) -> bool:
    """Returns True when a cube is already past the green line inside the delivery zone.

    Args:
        cube: Tracked cube observation in arena coordinates.

    Returns:
        True if the cube center lies within the delivery zone opening.
    """
    is_past_green = cube.x_m >= ZONE_GREEN_LINE_X_M
    is_within_y = ZONE_MIN_Y_M <= cube.y_m <= ZONE_MAX_Y_M
    return is_past_green and is_within_y


def estimate_peer_cross_penalty(
    own_path: tuple[Point2D, ...],
    peer_plan: PlanSnapshot | None,
    now_s: float,
    *,
    config: MissionConfig,
) -> float:
    """Returns cross_penalty_m if own_path crosses a fresh peer plan, else 0.0.

    Gracefully returns 0.0 when the peer plan is missing or older than
    peer_plan_timeout_s (Pilar 6 degradation rule).

    Args:
        own_path: Planned polyline points for this robot in meters.
        peer_plan: Latest PlanSnapshot received from the peer robot, or None.
        now_s: Current local monotonic timestamp in seconds.
        config: Mission configuration holding timeout and penalty weights.

    Returns:
        Penalty in meters added to the cube selection cost.
    """
    if peer_plan is None or len(own_path) < 2 or len(peer_plan.path) < 2:
        return 0.0

    age_s = now_s - peer_plan.received_at_s
    if age_s < 0.0 or age_s > config.peer_plan_timeout_s:
        return 0.0

    for own_idx in range(len(own_path) - 1):
        a1 = own_path[own_idx]
        a2 = own_path[own_idx + 1]
        for peer_idx in range(len(peer_plan.path) - 1):
            b1 = peer_plan.path[peer_idx]
            b2 = peer_plan.path[peer_idx + 1]
            if _segments_close_or_crossing(
                a1,
                a2,
                b1,
                b2,
                threshold_m=config.peer_cross_threshold_m,
            ):
                return config.cross_penalty_m

    return 0.0


def select_target_cube(
    own_pose: Pose2D,
    cubes: tuple[CubeObservation, ...],
    now_s: float,
    *,
    config: MissionConfig,
    current_target_id: int | None = None,
    peer_plan: PlanSnapshot | None = None,
    excluded_ids: frozenset[int] = frozenset(),
) -> MissionDecision:
    """Selects the lowest-cost reachable cube of own color with sticky hysteresis.

    Implements:
        Cost = d(robot, A) + d(C, G) + Pen_cross + Pen_border + Pen_red - bonus_sticky

    When no reachable cubes of own color remain outside the delivery zone,
    signals should_park=True so the robot moves to a safe left corner.

    Args:
        own_pose: Current robot pose in arena coordinates.
        cubes: All visible high-confidence cubes in the arena.
        now_s: Current local monotonic timestamp in seconds.
        config: Mission parameters and color ownership.
        current_target_id: Cube ID currently pursued (receives hysteresis bonus).
        peer_plan: Latest peer plan snapshot for crossing penalty, or None.
        excluded_ids: Cube IDs temporarily skipped after failed recoveries.

    Returns:
        MissionDecision with chosen cube and push plan, or should_park=True.
    """
    red_positions = tuple(
        (cube.x_m, cube.y_m)
        for cube in cubes
        if cube.color == CubeColor.RED
    )
    candidates = tuple(
        cube
        for cube in cubes
        if cube.color == config.target_color
        and cube.cube_id not in excluded_ids
        and not is_cube_in_delivery_zone(cube)
    )

    best_cube: CubeObservation | None = None
    best_plan: PushPlan | None = None
    best_cost = math.inf

    for cube in candidates:
        plan = plan_push(
            (cube.x_m, cube.y_m),
            config.pre_zone_goal_xy,
            red_positions,
            config=config.navigation,
        )
        if plan.mode == PushMode.UNREACHABLE or plan.approach_pose is None:
            continue

        cost = _score_candidate(
            own_pose,
            cube,
            plan,
            now_s=now_s,
            config=config,
            current_target_id=current_target_id,
            peer_plan=peer_plan,
        )
        if cost < best_cost:
            best_cost = cost
            best_cube = cube
            best_plan = plan

    if best_cube is None or best_plan is None:
        return MissionDecision(
            target_cube=None,
            push_plan=None,
            cost_m=math.inf,
            should_park=True,
        )

    return MissionDecision(
        target_cube=best_cube,
        push_plan=best_plan,
        cost_m=best_cost,
        should_park=False,
    )


def _score_candidate(
    own_pose: Pose2D,
    cube: CubeObservation,
    plan: PushPlan,
    *,
    now_s: float,
    config: MissionConfig,
    current_target_id: int | None,
    peer_plan: PlanSnapshot | None,
) -> float:
    """Computes total mission cost in meters for one reachable candidate cube."""
    assert plan.approach_pose is not None
    approach_xy = (
        plan.approach_pose.x_m,
        plan.approach_pose.y_m,
    )
    cube_xy = (
        cube.x_m,
        cube.y_m,
    )
    dist_to_approach = math.hypot(
        approach_xy[0] - own_pose.x_m,
        approach_xy[1] - own_pose.y_m,
    )
    dist_push = math.hypot(
        config.pre_zone_goal_xy[0] - cube_xy[0],
        config.pre_zone_goal_xy[1] - cube_xy[1],
    )
    own_path = (
        (own_pose.x_m, own_pose.y_m),
        approach_xy,
        *plan.waypoints,
    )
    cross_pen = estimate_peer_cross_penalty(
        own_path,
        peer_plan,
        now_s,
        config=config,
    )
    border_pen = config.border_penalty_m if plan.mode == PushMode.BORDER_SWEEP else 0.0
    red_pen = config.red_penalty_m if plan.mode == PushMode.TWO_STAGE else 0.0
    sticky_bonus = config.hysteresis_bonus_m if cube.cube_id == current_target_id else 0.0

    return dist_to_approach + dist_push + cross_pen + border_pen + red_pen - sticky_bonus


def _segments_close_or_crossing(
    a1: Point2D,
    a2: Point2D,
    b1: Point2D,
    b2: Point2D,
    *,
    threshold_m: float,
) -> bool:
    """Returns True when two 2D segments intersect or come within threshold_m."""
    if _segments_intersect(a1, a2, b1, b2):
        return True

    min_dist = min(
        distance_point_to_segment(a1, b1, b2),
        distance_point_to_segment(a2, b1, b2),
        distance_point_to_segment(b1, a1, a2),
        distance_point_to_segment(b2, a1, a2),
    )
    return min_dist < threshold_m


def _segments_intersect(
    a1: Point2D,
    a2: Point2D,
    b1: Point2D,
    b2: Point2D,
) -> bool:
    """Returns True if closed segments a1-a2 and b1-b2 properly intersect."""
    o1 = _cross_2d(a1, a2, b1)
    o2 = _cross_2d(a1, a2, b2)
    o3 = _cross_2d(b1, b2, a1)
    o4 = _cross_2d(b1, b2, a2)

    return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)


def _cross_2d(
    origin: Point2D,
    p1: Point2D,
    p2: Point2D,
) -> float:
    """Computes the 2D signed cross product (p1 - origin) x (p2 - origin)."""
    return (p1[0] - origin[0]) * (p2[1] - origin[1]) - (p1[1] - origin[1]) * (p2[0] - origin[0])
