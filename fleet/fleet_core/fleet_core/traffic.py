"""Peer-to-peer anti-collision using navigation-light rules and 20 Hz plan validation."""

import math
from dataclasses import dataclass

from fleet_core.constants import (
    DEADLOCK_TIMEOUT_S,
    DEFAULT_DEADLOCK_REVERSE_DISTANCE_M,
    DEFAULT_DEADLOCK_REVERSE_SPEED_MPS,
    DEFAULT_SIDESTEP_DISTANCE_M,
    DEFAULT_SIDESTEP_SPEED_MPS,
    PEER_COLLISION_ALERT_M,
    PEER_PLAN_TIMEOUT_S,
    RED_TRANSIT_RADIUS_M,
    TRAFFIC_CROSSING_MAX_BEARING_RAD,
    TRAFFIC_HEAD_ON_CONE_RAD,
    TRAFFIC_HEAD_ON_HEADING_TOL_RAD,
    TRAFFIC_OVERTAKING_CONE_RAD,
    TRAFFIC_OVERTAKING_HEADING_TOL_RAD,
    TRAFFIC_PROJECTION_HORIZON_S,
    TRAFFIC_URGENT_CLEAR_RATIO,
    VIRTUAL_BORDER_MARGIN_M,
    PlannerState,
    RobotName,
    TrafficAction,
    TrafficSituation,
)
from fleet_core.math_utils import wrap_angle
from fleet_core.navigation import (
    Point2D,
    is_corridor_clear,
    is_inside_virtual_margin,
)
from fleet_core.types import (
    PlanSnapshot,
    Pose2D,
    VelocityCmd,
)

RESTRICTED_STATES: frozenset[PlannerState] = frozenset(
    (
        PlannerState.LEAVE,
        PlannerState.DELIVER,
        PlannerState.PUSH,
    )
)


@dataclass(frozen=True)
class TrafficConfig:
    """Clearances, speeds, and timers for navigation-light P2P anti-collision."""

    robot_name: str
    alert_distance_m: float = PEER_COLLISION_ALERT_M
    projection_horizon_s: float = TRAFFIC_PROJECTION_HORIZON_S
    border_margin_m: float = VIRTUAL_BORDER_MARGIN_M
    red_clearance_m: float = RED_TRANSIT_RADIUS_M
    sidestep_distance_m: float = DEFAULT_SIDESTEP_DISTANCE_M
    sidestep_speed_mps: float = DEFAULT_SIDESTEP_SPEED_MPS
    deadlock_timeout_s: float = DEADLOCK_TIMEOUT_S
    deadlock_reverse_distance_m: float = DEFAULT_DEADLOCK_REVERSE_DISTANCE_M
    deadlock_reverse_speed_mps: float = DEFAULT_DEADLOCK_REVERSE_SPEED_MPS
    peer_plan_timeout_s: float = PEER_PLAN_TIMEOUT_S


@dataclass(frozen=True)
class TrafficDecision:
    """Output of one 20 Hz P2P anti-collision evaluation."""

    situation: TrafficSituation
    action: TrafficAction
    must_wait: bool
    evasion_cmd: VelocityCmd | None


def relative_bearing(
    own_pose: Pose2D,
    peer_pose: Pose2D,
) -> float:
    """Computes bearing of peer seen from own robot in (-pi, pi] (left > 0, right < 0).

    Args:
        own_pose: Own robot pose in arena coordinates.
        peer_pose: Peer robot pose in arena coordinates.

    Returns:
        Relative bearing angle in radians in REP 103 convention.
    """
    world_angle = math.atan2(
        peer_pose.y_m - own_pose.y_m,
        peer_pose.x_m - own_pose.x_m,
    )
    return wrap_angle(world_angle - own_pose.theta_rad)


def classify_encounter(
    own_pose: Pose2D,
    peer_pose: Pose2D,
    *,
    own_state: PlannerState,
    peer_state: PlannerState | None,
) -> TrafficSituation:
    """Classifies right-of-way using restricted-maneuver priority and Red/Green lights.

    Priority rules:
      1. Restricted maneuverability (LEAVE, DELIVER, PUSH) beats an empty robot.
      2. Under equal priority:
         - Head-on -> HEAD_ON (both sidestep starboard; Ferb yields on tie-break).
         - Peer ahead with similar heading -> OVERTAKING (rear yields).
         - Peer entering from right (bearing < 0) -> SEE_RED (yield).
         - Peer entering from left (bearing > 0) or behind -> SEE_GREEN (hold course).

    Args:
        own_pose: Own robot pose in arena coordinates.
        peer_pose: Peer robot pose in arena coordinates.
        own_state: Current PlannerState of own robot.
        peer_state: Fresh PlannerState of peer robot, or None if unavailable.

    Returns:
        Classified TrafficSituation.
    """
    own_restricted = own_state in RESTRICTED_STATES
    peer_restricted = peer_state in RESTRICTED_STATES if peer_state is not None else False

    if own_restricted and not peer_restricted:
        return TrafficSituation.HOLD_PRIORITY
    if peer_restricted and not own_restricted:
        return TrafficSituation.SEE_RED

    bearing = relative_bearing(
        own_pose,
        peer_pose,
    )
    abs_bearing = abs(bearing)
    if abs_bearing > TRAFFIC_CROSSING_MAX_BEARING_RAD:
        return TrafficSituation.SEE_GREEN

    heading_diff = abs(wrap_angle(peer_pose.theta_rad - own_pose.theta_rad))
    opposite_diff = abs(wrap_angle(peer_pose.theta_rad - own_pose.theta_rad - math.pi))

    is_head_on = (
        abs_bearing <= TRAFFIC_HEAD_ON_CONE_RAD
        and opposite_diff <= TRAFFIC_HEAD_ON_HEADING_TOL_RAD
    )
    if is_head_on:
        return TrafficSituation.HEAD_ON

    is_overtaking = (
        abs_bearing <= TRAFFIC_OVERTAKING_CONE_RAD
        and heading_diff <= TRAFFIC_OVERTAKING_HEADING_TOL_RAD
    )
    if is_overtaking:
        return TrafficSituation.OVERTAKING

    if bearing < 0.0:
        return TrafficSituation.SEE_RED

    return TrafficSituation.SEE_GREEN


def is_step_safe(
    own_pose: Pose2D,
    body_offset_xy: Point2D,
    red_positions: tuple[Point2D, ...],
    *,
    border_margin_m: float = VIRTUAL_BORDER_MARGIN_M,
    red_clearance_m: float = RED_TRANSIT_RADIUS_M,
) -> bool:
    """Validates a projected body-frame evasion step before commanding motion.

    Checks that the projected target point stays inside the virtual border margin
    and that the entire swept segment stays >= red_clearance_m from all red cubes.

    Args:
        own_pose: Current robot pose in arena coordinates.
        body_offset_xy: Desired (dx_body_m, dy_body_m) displacement in body frame.
        red_positions: Visible red cube centers in arena coordinates.
        border_margin_m: Virtual interior inset from black border tape in meters.
        red_clearance_m: Minimum allowed clearance from red cubes in meters.

    Returns:
        True when the planned evasion step is safe to execute.
    """
    cos_theta = math.cos(own_pose.theta_rad)
    sin_theta = math.sin(own_pose.theta_rad)
    dx_body, dy_body = body_offset_xy
    target_xy = (
        own_pose.x_m + cos_theta * dx_body - sin_theta * dy_body,
        own_pose.y_m + sin_theta * dx_body + cos_theta * dy_body,
    )
    if not is_inside_virtual_margin(target_xy, margin_m=border_margin_m):
        return False

    start_xy = (
        own_pose.x_m,
        own_pose.y_m,
    )
    return is_corridor_clear(
        start_xy,
        target_xy,
        red_positions,
        half_width_m=red_clearance_m,
    )


class TrafficCoordinator:
    """Evaluates P2P right-of-way and validated evasion maneuvers at 20 Hz."""

    def __init__(self, config: TrafficConfig) -> None:
        """Initializes traffic coordinator with no active deadlock timer."""
        self._config = config
        self._blocked_since_s: float | None = None

    def reset(self) -> None:
        """Clears any active deadlock timer."""
        self._blocked_since_s = None

    def evaluate(
        self,
        own_pose: Pose2D,
        peer_pose: Pose2D | None,
        *,
        own_state: PlannerState,
        red_positions: tuple[Point2D, ...],
        now_s: float,
        own_cmd: VelocityCmd | None = None,
        peer_plan: PlanSnapshot | None = None,
        overhead_alert: bool = False,
    ) -> TrafficDecision:
        """Computes right-of-way and validated starboard/reverse evasion for one cycle.

        Args:
            own_pose: Current own pose in arena coordinates.
            peer_pose: Current peer pose in arena coordinates, or None.
            own_state: Current PlannerState of own robot.
            red_positions: Visible red cube centers in arena coordinates.
            now_s: Current local monotonic timestamp in seconds.
            own_cmd: Current body velocity command for 0.5 s collision projection.
            peer_plan: Optional peer PlanSnapshot for priority comparison.
            overhead_alert: True when /world/sensors reported "near:robot".

        Returns:
            TrafficDecision with must_wait and optional validated evasion_cmd.
        """
        if peer_pose is None:
            self.reset()
            return TrafficDecision(
                situation=TrafficSituation.CLEAR,
                action=TrafficAction.HOLD_COURSE,
                must_wait=False,
                evasion_cmd=None,
            )

        if not self._is_conflict_active(
            own_pose,
            peer_pose,
            own_cmd=own_cmd,
            overhead_alert=overhead_alert,
        ):
            self.reset()
            return TrafficDecision(
                situation=TrafficSituation.CLEAR,
                action=TrafficAction.HOLD_COURSE,
                must_wait=False,
                evasion_cmd=None,
            )

        peer_state = self._fresh_peer_state(
            peer_plan,
            now_s,
        )
        situation = classify_encounter(
            own_pose,
            peer_pose,
            own_state=own_state,
            peer_state=peer_state,
        )
        if situation in (TrafficSituation.HOLD_PRIORITY, TrafficSituation.SEE_GREEN):
            self.reset()
            return TrafficDecision(
                situation=situation,
                action=TrafficAction.HOLD_COURSE,
                must_wait=False,
                evasion_cmd=None,
            )

        return self._resolve_evasion(
            own_pose,
            peer_pose,
            situation,
            red_positions=red_positions,
            now_s=now_s,
        )

    def _is_conflict_active(
        self,
        own_pose: Pose2D,
        peer_pose: Pose2D,
        *,
        own_cmd: VelocityCmd | None,
        overhead_alert: bool,
    ) -> bool:
        """Returns True when separation is below 34 cm now or projected at 0.5 s."""
        if overhead_alert:
            return True

        dist_now = math.hypot(
            peer_pose.x_m - own_pose.x_m,
            peer_pose.y_m - own_pose.y_m,
        )
        if dist_now < self._config.alert_distance_m:
            return True

        if own_cmd is None:
            return False

        cos_theta = math.cos(own_pose.theta_rad)
        sin_theta = math.sin(own_pose.theta_rad)
        dt = self._config.projection_horizon_s
        proj_x = own_pose.x_m + (own_cmd.vx_mps * cos_theta - own_cmd.vy_mps * sin_theta) * dt
        proj_y = own_pose.y_m + (own_cmd.vx_mps * sin_theta + own_cmd.vy_mps * cos_theta) * dt
        dist_proj = math.hypot(
            peer_pose.x_m - proj_x,
            peer_pose.y_m - proj_y,
        )
        return dist_proj < self._config.alert_distance_m

    def _fresh_peer_state(
        self,
        peer_plan: PlanSnapshot | None,
        now_s: float,
    ) -> PlannerState | None:
        """Returns the peer state only if its plan arrived within peer_plan_timeout_s."""
        if peer_plan is None:
            return None

        age_s = now_s - peer_plan.received_at_s
        if age_s < 0.0 or age_s > self._config.peer_plan_timeout_s:
            return None

        return peer_plan.state

    def _resolve_evasion(
        self,
        own_pose: Pose2D,
        peer_pose: Pose2D,
        situation: TrafficSituation,
        *,
        red_positions: tuple[Point2D, ...],
        now_s: float,
    ) -> TrafficDecision:
        """Resolves evasion: starboard sidestep when safe & moving away, else WAIT/reverse."""
        starboard_offset = (
            0.0,
            -self._config.sidestep_distance_m,
        )
        is_starboard_away = _step_increases_peer_distance(
            own_pose,
            peer_pose,
            starboard_offset,
        )
        is_starboard_safe = is_starboard_away and is_step_safe(
            own_pose,
            starboard_offset,
            red_positions,
            border_margin_m=self._config.border_margin_m,
            red_clearance_m=self._config.red_clearance_m,
        )
        if is_starboard_safe:
            self.reset()
            return TrafficDecision(
                situation=situation,
                action=TrafficAction.SIDESTEP_STARBOARD,
                must_wait=False,
                evasion_cmd=VelocityCmd(
                    vx_mps=0.0,
                    vy_mps=-self._config.sidestep_speed_mps,
                    wz_radps=0.0,
                ),
            )

        # In SEE_RED/OVERTAKING, if starboard moves into the peer on our right, step port away.
        dist_now = math.hypot(
            peer_pose.x_m - own_pose.x_m,
            peer_pose.y_m - own_pose.y_m,
        )
        is_urgent_port_escape = (
            situation != TrafficSituation.HEAD_ON
            and dist_now < self._config.alert_distance_m * TRAFFIC_URGENT_CLEAR_RATIO
        )
        if is_urgent_port_escape:
            port_offset = (
                0.0,
                self._config.sidestep_distance_m,
            )
            if _step_increases_peer_distance(
                own_pose,
                peer_pose,
                port_offset,
            ) and is_step_safe(
                own_pose,
                port_offset,
                red_positions,
                border_margin_m=self._config.border_margin_m,
                red_clearance_m=self._config.red_clearance_m,
            ):
                self.reset()
                return TrafficDecision(
                    situation=situation,
                    action=TrafficAction.SIDESTEP_STARBOARD,
                    must_wait=False,
                    evasion_cmd=VelocityCmd(
                        vx_mps=0.0,
                        vy_mps=self._config.sidestep_speed_mps,
                        wz_radps=0.0,
                    ),
                )

        if self._blocked_since_s is None:
            self._blocked_since_s = now_s

        # In a head-on tie-break, Ferb attempts deadlock reverse before Phineas.
        is_phineas_head_on = (
            situation == TrafficSituation.HEAD_ON
            and self._config.robot_name == RobotName.PHINEAS.value
        )
        effective_timeout_s = (
            2.0 * self._config.deadlock_timeout_s
            if is_phineas_head_on
            else self._config.deadlock_timeout_s
        )
        blocked_duration_s = now_s - self._blocked_since_s
        if blocked_duration_s >= effective_timeout_s:
            reverse_offset = (
                -self._config.deadlock_reverse_distance_m,
                0.0,
            )
            if is_step_safe(
                own_pose,
                reverse_offset,
                red_positions,
                border_margin_m=self._config.border_margin_m,
                red_clearance_m=self._config.red_clearance_m,
            ):
                return TrafficDecision(
                    situation=situation,
                    action=TrafficAction.DEADLOCK_REVERSE,
                    must_wait=False,
                    evasion_cmd=VelocityCmd(
                        vx_mps=-self._config.deadlock_reverse_speed_mps,
                        vy_mps=0.0,
                        wz_radps=0.0,
                    ),
                )

        return TrafficDecision(
            situation=situation,
            action=TrafficAction.WAIT,
            must_wait=True,
            evasion_cmd=None,
        )


def _step_increases_peer_distance(
    own_pose: Pose2D,
    peer_pose: Pose2D,
    body_offset_xy: Point2D,
) -> bool:
    """Returns True when the projected body-frame step does not move closer to peer."""
    cos_theta = math.cos(own_pose.theta_rad)
    sin_theta = math.sin(own_pose.theta_rad)
    dx_body, dy_body = body_offset_xy
    target_x = own_pose.x_m + cos_theta * dx_body - sin_theta * dy_body
    target_y = own_pose.y_m + sin_theta * dx_body + cos_theta * dy_body
    dist_now = math.hypot(
        peer_pose.x_m - own_pose.x_m,
        peer_pose.y_m - own_pose.y_m,
    )
    dist_after = math.hypot(
        peer_pose.x_m - target_x,
        peer_pose.y_m - target_y,
    )
    return dist_after > dist_now
