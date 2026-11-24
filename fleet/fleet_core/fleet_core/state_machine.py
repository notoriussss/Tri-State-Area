"""Unified autonomous state machine running onboard Phineas and Ferb at ~20 Hz."""

import math
from dataclasses import dataclass, field

from fleet_core.constants import (
    ARENA_CENTER_X_M,
    ARENA_CENTER_Y_M,
    DEFAULT_DELIVER_SLOT_X_M,
    DEFAULT_DELIVER_SLOT_Y_M,
    DEFAULT_LEAVE_CLEAR_X_M,
    DEFAULT_LEAVE_SPEED_MPS,
    DEFAULT_LINE_ESCAPE_SPEED_MPS,
    DEFAULT_MAX_RECOVER_RETRIES,
    DEFAULT_PRE_ZONE_TOLERANCE_M,
    DEFAULT_RECOVER_BACKUP_M,
    DEFAULT_SCOOP_MAX_AHEAD_M,
    DEFAULT_SCOOP_MAX_LATERAL_M,
    DEFAULT_SCOOP_MIN_AHEAD_M,
    DEFAULT_STAGE_ARRIVAL_M,
    POINT_EPSILON_M,
    PRE_ZONE_X_M,
    ZONE_MAX_X_M,
    CubeColor,
    PlannerState,
    PushMode,
    VirtualSensorEvent,
)
from fleet_core.controller import (
    ControllerConfig,
    PoseController,
    is_at_pose,
)
from fleet_core.math_utils import wrap_angle
from fleet_core.mission import (
    MissionConfig,
    default_mission_config,
    is_cube_in_delivery_zone,
    select_target_cube,
)
from fleet_core.navigation import (
    NavigationConfig,
    Point2D,
    PushPlan,
    compute_approach_pose,
    plan_push,
    plan_transit_waypoints,
)
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    CubeObservation,
    PlanSnapshot,
    Pose2D,
    VelocityCmd,
)
from fleet_core.world_model import WorldModel


@dataclass(frozen=True)
class StateMachineConfig:
    """Configuration for the unified autonomous collector state machine."""

    mission: MissionConfig
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    scoop_max_lateral_m: float = DEFAULT_SCOOP_MAX_LATERAL_M
    scoop_max_ahead_m: float = DEFAULT_SCOOP_MAX_AHEAD_M
    scoop_min_ahead_m: float = DEFAULT_SCOOP_MIN_AHEAD_M
    recover_backup_m: float = DEFAULT_RECOVER_BACKUP_M
    max_recover_retries: int = DEFAULT_MAX_RECOVER_RETRIES
    stage_arrival_m: float = DEFAULT_STAGE_ARRIVAL_M
    pre_zone_tolerance_m: float = DEFAULT_PRE_ZONE_TOLERANCE_M
    line_escape_speed_mps: float = DEFAULT_LINE_ESCAPE_SPEED_MPS
    leave_speed_mps: float = DEFAULT_LEAVE_SPEED_MPS
    leave_clear_x_m: float = DEFAULT_LEAVE_CLEAR_X_M
    default_delivery_slot: Point2D = (
        DEFAULT_DELIVER_SLOT_X_M,
        DEFAULT_DELIVER_SLOT_Y_M,
    )


def default_state_machine_config(robot_name: str) -> StateMachineConfig:
    """Builds the default StateMachineConfig for Phineas or Ferb."""
    return StateMachineConfig(mission=default_mission_config(robot_name))


@dataclass(frozen=True)
class CycleInputs:
    """All external inputs consumed by one 20 Hz state machine tick."""

    now_s: float
    dt_s: float
    line_detected: bool
    peer_plan: PlanSnapshot | None = None
    zone_granted: bool = False
    zone_ready: bool = True
    delivery_slot_xy: Point2D | None = None
    traffic_must_wait: bool = False
    traffic_evasion_cmd: VelocityCmd | None = None


@dataclass(frozen=True)
class CycleOutput:
    """Actuation, Perry zone actions, and plan telemetry produced on each tick."""

    state: PlannerState
    cmd: VelocityCmd
    allow_line_escape: bool
    want_zone: bool
    release_zone: bool
    delivered_color: CubeColor | None
    plan: PlanSnapshot


def is_cube_in_scoop(
    robot_pose: Pose2D,
    cube: CubeObservation,
    *,
    config: StateMachineConfig,
) -> bool:
    """Returns True when the target cube stays aligned ahead of the robot scoop.

    Args:
        robot_pose: Current robot pose in arena coordinates.
        cube: Observed position of the pursued cube.
        config: Scoop tracking envelope thresholds.

    Returns:
        True if the cube is within longitudinal and lateral scoop bounds.
    """
    dx = cube.x_m - robot_pose.x_m
    dy = cube.y_m - robot_pose.y_m
    cos_theta = math.cos(robot_pose.theta_rad)
    sin_theta = math.sin(robot_pose.theta_rad)
    ahead_m = cos_theta * dx + sin_theta * dy
    lateral_m = -sin_theta * dx + cos_theta * dy

    is_ahead_ok = config.scoop_min_ahead_m <= ahead_m <= config.scoop_max_ahead_m
    is_lateral_ok = abs(lateral_m) <= config.scoop_max_lateral_m
    return is_ahead_ok and is_lateral_ok


def compute_line_escape_cmd(
    robot_pose: Pose2D,
    *,
    escape_speed_mps: float = DEFAULT_LINE_ESCAPE_SPEED_MPS,
) -> VelocityCmd:
    """Computes a safe non-forward body velocity toward the arena center.

    Guarantees vx_mps <= 0.0 so the onboard HalGuard permits the escape pulse
    away from the front-mounted TCRT5000 border sensor.

    Args:
        robot_pose: Current robot pose in arena coordinates.
        escape_speed_mps: Desired linear escape speed magnitude in m/s.

    Returns:
        Non-forward VelocityCmd directing the robot inward off the black tape.
    """
    dx = ARENA_CENTER_X_M - robot_pose.x_m
    dy = ARENA_CENTER_Y_M - robot_pose.y_m
    cos_theta = math.cos(robot_pose.theta_rad)
    sin_theta = math.sin(robot_pose.theta_rad)
    vx_body = min(
        0.0,
        cos_theta * dx + sin_theta * dy,
    )
    vy_body = -sin_theta * dx + cos_theta * dy
    norm = math.hypot(
        vx_body,
        vy_body,
    )
    if norm <= POINT_EPSILON_M:
        return VelocityCmd(
            vx_mps=-escape_speed_mps,
            vy_mps=0.0,
            wz_radps=0.0,
        )

    return VelocityCmd(
        vx_mps=escape_speed_mps * (vx_body / norm),
        vy_mps=escape_speed_mps * (vy_body / norm),
        wz_radps=0.0,
    )


class CollectorStateMachine:
    """Deterministic 20 Hz state machine implementing the 6-pillar strategy."""

    def __init__(self, config: StateMachineConfig) -> None:
        """Initializes the collector FSM in IDLE with no assigned cube."""
        self._config = config
        self._controller = PoseController(config.controller)
        self._state: PlannerState = PlannerState.IDLE
        self._resume_state: PlannerState = PlannerState.IDLE
        self._target_cube_id: int | None = None
        self._push_plan: PushPlan | None = None
        self._stage_idx: int = 0
        self._recover_start_pose: Pose2D | None = None
        self._recover_retries: int = 0
        self._excluded_ids: set[int] = set()
        self._delivered_ok: bool = False
        self._last_cmd: VelocityCmd = ZERO_VELOCITY_CMD
        self._last_target_pose: Pose2D = Pose2D(0.0, 0.0, 0.0)
        self._last_path: tuple[Point2D, ...] = ()

    @property
    def state(self) -> PlannerState:
        """Returns the current autonomous PlannerState."""
        return self._state

    @property
    def target_cube_id(self) -> int | None:
        """Returns the currently pursued cube ID, or None."""
        return self._target_cube_id

    def step(
        self,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Advances the state machine by one control cycle.

        Args:
            world_model: Local-clock overhead world model.
            inputs: Cycle timing, sensor reflexes, peer plan, and zone signals.

        Returns:
            CycleOutput with velocity command, zone requests, and plan snapshot.
        """
        own_pose = world_model.own_pose(inputs.now_s)
        if own_pose is None:
            return self._handle_camera_loss(inputs.now_s)

        if inputs.line_detected:
            return self._handle_line_escape(
                own_pose,
                inputs.now_s,
            )

        if self._state == PlannerState.STOPPED:
            self._state = PlannerState.IDLE

        traffic_output = self._handle_traffic_interrupt(
            own_pose,
            inputs,
        )
        if traffic_output is not None:
            return traffic_output

        return self._dispatch_state(
            own_pose,
            world_model,
            inputs,
        )

    def _dispatch_state(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Routes execution to the active state's handler."""
        handlers = {
            PlannerState.IDLE: self._step_idle,
            PlannerState.PARKED: self._step_parked,
            PlannerState.GO_TO_APPROACH: self._step_go_to_approach,
            PlannerState.PUSH: self._step_push,
            PlannerState.REQUEST_ZONE: self._step_request_zone,
            PlannerState.DELIVER: self._step_deliver,
            PlannerState.LEAVE: self._step_leave,
            PlannerState.RECOVER: self._step_recover,
        }
        handler = handlers[self._state]
        return handler(
            own_pose,
            world_model,
            inputs,
        )

    def _handle_camera_loss(self, now_s: float) -> CycleOutput:
        """Stops immediately when overhead pose is missing or stale (> 0.5 s)."""
        self._controller.reset()
        self._state = PlannerState.STOPPED
        return self._make_output(
            cmd=ZERO_VELOCITY_CMD,
            now_s=now_s,
        )

    def _handle_line_escape(
        self,
        own_pose: Pose2D,
        now_s: float,
    ) -> CycleOutput:
        """Commands a safe inward escape pulse off the black border tape."""
        self._controller.reset()
        self._state = PlannerState.STOPPED
        escape_cmd = compute_line_escape_cmd(
            own_pose,
            escape_speed_mps=self._config.line_escape_speed_mps,
        )
        self._last_target_pose = Pose2D(
            x_m=ARENA_CENTER_X_M,
            y_m=ARENA_CENTER_Y_M,
            theta_rad=own_pose.theta_rad,
        )
        self._last_path = (
            (own_pose.x_m, own_pose.y_m),
            (ARENA_CENTER_X_M, ARENA_CENTER_Y_M),
        )
        return self._make_output(
            cmd=escape_cmd,
            now_s=now_s,
            allow_line_escape=True,
        )

    def _handle_traffic_interrupt(
        self,
        own_pose: Pose2D,
        inputs: CycleInputs,
    ) -> CycleOutput | None:
        """Applies WAIT or validated evasion override from the P2P traffic layer."""
        is_zone_critical = self._state in (
            PlannerState.DELIVER,
            PlannerState.LEAVE,
            PlannerState.RECOVER,
        )
        if is_zone_critical:
            return None

        if inputs.traffic_must_wait:
            if self._state != PlannerState.WAIT:
                self._resume_state = self._state
                self._state = PlannerState.WAIT
            self._controller.reset()
            return self._make_output(
                cmd=ZERO_VELOCITY_CMD,
                now_s=inputs.now_s,
            )

        if self._state == PlannerState.WAIT:
            self._state = self._resume_state

        if inputs.traffic_evasion_cmd is not None:
            self._last_target_pose = own_pose
            return self._make_output(
                cmd=inputs.traffic_evasion_cmd,
                now_s=inputs.now_s,
            )

        return None

    def _step_idle(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Selects the lowest-cost cube of own color or transitions to PARKED."""
        cubes = world_model.all_cubes(inputs.now_s)
        decision = select_target_cube(
            own_pose,
            cubes,
            inputs.now_s,
            config=self._config.mission,
            current_target_id=self._target_cube_id,
            peer_plan=inputs.peer_plan,
            excluded_ids=frozenset(self._excluded_ids),
        )
        if decision.should_park or decision.target_cube is None or decision.push_plan is None:
            self._target_cube_id = None
            self._push_plan = None
            self._state = PlannerState.PARKED
            return self._step_parked(
                own_pose,
                world_model,
                inputs,
            )

        self._target_cube_id = decision.target_cube.cube_id
        self._push_plan = decision.push_plan
        self._stage_idx = 0
        self._recover_retries = 0
        self._delivered_ok = False
        self._state = PlannerState.GO_TO_APPROACH
        return self._step_go_to_approach(
            own_pose,
            world_model,
            inputs,
        )

    def _step_parked(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Drives to the safe left parking corner and wakes if a new own cube appears."""
        cubes = world_model.all_cubes(inputs.now_s)
        decision = select_target_cube(
            own_pose,
            cubes,
            inputs.now_s,
            config=self._config.mission,
            peer_plan=inputs.peer_plan,
            excluded_ids=frozenset(self._excluded_ids),
        )
        if not decision.should_park and decision.target_cube is not None:
            self._state = PlannerState.IDLE
            return self._step_idle(
                own_pose,
                world_model,
                inputs,
            )

        park_pose = self._config.mission.park_pose
        cmd = self._drive_transit(
            own_pose,
            park_pose,
            world_model,
            inputs,
        )
        return self._make_output(
            cmd=cmd,
            now_s=inputs.now_s,
        )

    def _step_go_to_approach(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Transits around red cubes to align at approach pose A behind the target cube."""
        cube = self._current_cube(
            world_model,
            inputs.now_s,
        )
        if cube is None or is_cube_in_delivery_zone(cube):
            return self._reset_to_idle(inputs.now_s)

        stage_goal = self._active_stage_goal(
            cube,
            world_model,
            inputs.now_s,
        )
        if stage_goal is None:
            return self._reset_to_idle(inputs.now_s)

        approach_pose = self._active_approach_pose(
            cube,
            stage_goal,
        )
        if is_at_pose(own_pose, approach_pose, self._config.controller):
            self._state = PlannerState.PUSH
            return self._step_push(
                own_pose,
                world_model,
                inputs,
            )

        cmd = self._drive_transit(
            own_pose,
            approach_pose,
            world_model,
            inputs,
        )
        return self._make_output(
            cmd=cmd,
            now_s=inputs.now_s,
        )

    def _step_push(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Pushes the cube straight toward the current stage goal or pre-zone line."""
        cube = self._current_cube(
            world_model,
            inputs.now_s,
        )
        if cube is None:
            return self._enter_recover(
                own_pose,
                inputs.now_s,
            )

        if not is_cube_in_scoop(own_pose, cube, config=self._config):
            return self._enter_recover(
                own_pose,
                inputs.now_s,
            )

        stage_goal = self._active_stage_goal(
            cube,
            world_model,
            inputs.now_s,
        )
        if stage_goal is None:
            return self._reset_to_idle(inputs.now_s)

        is_final_stage = (
            self._push_plan is None or self._stage_idx >= len(self._push_plan.waypoints) - 1
        )
        if is_final_stage and cube.x_m >= PRE_ZONE_X_M - self._config.pre_zone_tolerance_m:
            self._controller.reset()
            self._state = PlannerState.REQUEST_ZONE
            return self._step_request_zone(
                own_pose,
                world_model,
                inputs,
            )

        dist_to_stage = math.hypot(
            stage_goal[0] - cube.x_m,
            stage_goal[1] - cube.y_m,
        )
        if not is_final_stage and dist_to_stage <= self._config.stage_arrival_m:
            self._stage_idx += 1
            self._controller.reset()
            self._state = PlannerState.GO_TO_APPROACH
            return self._make_output(
                cmd=ZERO_VELOCITY_CMD,
                now_s=inputs.now_s,
            )

        heading = wrap_angle(
            math.atan2(
                stage_goal[1] - own_pose.y_m,
                stage_goal[0] - own_pose.x_m,
            )
        )
        target_pose = Pose2D(
            x_m=stage_goal[0],
            y_m=stage_goal[1],
            theta_rad=heading,
        )
        self._last_target_pose = target_pose
        self._last_path = (
            (own_pose.x_m, own_pose.y_m),
            stage_goal,
        )
        cmd = self._controller.step(
            own_pose,
            target_pose,
            inputs.dt_s,
        )
        return self._make_output(
            cmd=cmd,
            now_s=inputs.now_s,
        )

    def _step_request_zone(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Waits cleanly at the pre-zone line until Perry grants the delivery lease."""
        cube = self._current_cube(
            world_model,
            inputs.now_s,
        )
        if cube is None:
            return self._reset_to_idle(inputs.now_s)

        self._last_target_pose = own_pose
        self._last_path = ((own_pose.x_m, own_pose.y_m),)

        if inputs.zone_ready and inputs.zone_granted:
            self._state = PlannerState.DELIVER
            return self._step_deliver(
                own_pose,
                world_model,
                inputs,
            )

        self._controller.reset()
        return self._make_output(
            cmd=ZERO_VELOCITY_CMD,
            now_s=inputs.now_s,
            want_zone=inputs.zone_ready,
        )

    def _step_deliver(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Pushes the cube across the green line into a free slot respecting x_max."""
        cube = self._current_cube(
            world_model,
            inputs.now_s,
        )
        if cube is None:
            self._delivered_ok = False
            self._state = PlannerState.LEAVE
            return self._step_leave(
                own_pose,
                world_model,
                inputs,
            )

        if is_cube_in_delivery_zone(cube):
            self._delivered_ok = True
            self._controller.reset()
            self._state = PlannerState.LEAVE
            return self._step_leave(
                own_pose,
                world_model,
                inputs,
            )

        if not is_cube_in_scoop(own_pose, cube, config=self._config):
            return self._enter_recover(
                own_pose,
                inputs.now_s,
            )

        raw_slot = inputs.delivery_slot_xy or self._config.default_delivery_slot
        slot_x = min(
            raw_slot[0],
            ZONE_MAX_X_M,
        )
        slot_y = raw_slot[1]
        if own_pose.x_m >= ZONE_MAX_X_M:
            self._delivered_ok = False
            self._state = PlannerState.LEAVE
            return self._step_leave(
                own_pose,
                world_model,
                inputs,
            )

        heading = wrap_angle(
            math.atan2(
                slot_y - own_pose.y_m,
                slot_x - own_pose.x_m,
            )
        )
        target_pose = Pose2D(
            x_m=slot_x,
            y_m=slot_y,
            theta_rad=heading,
        )
        self._last_target_pose = target_pose
        self._last_path = (
            (own_pose.x_m, own_pose.y_m),
            (slot_x, slot_y),
        )
        cmd = self._controller.step(
            own_pose,
            target_pose,
            inputs.dt_s,
        )
        return self._make_output(
            cmd=cmd,
            now_s=inputs.now_s,
            want_zone=True,
        )

    def _step_leave(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Backs out straight (-vx, wz=0) until left:zone, then releases Perry's lease."""
        has_left_event = world_model.has_virtual_event(
            VirtualSensorEvent.LEFT_ZONE.value,
            inputs.now_s,
        )
        is_physically_clear = (
            not world_model.is_own_in_zone(inputs.now_s)
            and own_pose.x_m <= self._config.leave_clear_x_m
        )
        if has_left_event or is_physically_clear:
            delivered_color = (
                self._config.mission.target_color if self._delivered_ok else None
            )
            self._target_cube_id = None
            self._push_plan = None
            self._delivered_ok = False
            self._controller.reset()
            self._state = PlannerState.IDLE
            return self._make_output(
                cmd=ZERO_VELOCITY_CMD,
                now_s=inputs.now_s,
                release_zone=True,
                delivered_color=delivered_color,
            )

        reverse_cmd = VelocityCmd(
            vx_mps=-self._config.leave_speed_mps,
            vy_mps=0.0,
            wz_radps=0.0,
        )
        self._last_target_pose = Pose2D(
            x_m=self._config.leave_clear_x_m,
            y_m=own_pose.y_m,
            theta_rad=own_pose.theta_rad,
        )
        self._last_path = (
            (own_pose.x_m, own_pose.y_m),
            (self._config.leave_clear_x_m, own_pose.y_m),
        )
        return self._make_output(
            cmd=reverse_cmd,
            now_s=inputs.now_s,
            want_zone=True,
        )

    def _step_recover(
        self,
        own_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> CycleOutput:
        """Backs up ~12 cm straight to re-acquire a slipped cube (max 2 retries)."""
        if self._recover_start_pose is None:
            self._recover_start_pose = own_pose

        backed_m = math.hypot(
            own_pose.x_m - self._recover_start_pose.x_m,
            own_pose.y_m - self._recover_start_pose.y_m,
        )
        if backed_m >= self._config.recover_backup_m:
            self._recover_start_pose = None
            self._controller.reset()
            if self._recover_retries > self._config.max_recover_retries:
                if self._target_cube_id is not None:
                    self._excluded_ids.add(self._target_cube_id)
                return self._reset_to_idle(inputs.now_s)

            self._state = PlannerState.GO_TO_APPROACH
            return self._make_output(
                cmd=ZERO_VELOCITY_CMD,
                now_s=inputs.now_s,
            )

        reverse_cmd = VelocityCmd(
            vx_mps=-self._config.leave_speed_mps,
            vy_mps=0.0,
            wz_radps=0.0,
        )
        return self._make_output(
            cmd=reverse_cmd,
            now_s=inputs.now_s,
        )

    def _enter_recover(
        self,
        own_pose: Pose2D,
        now_s: float,
    ) -> CycleOutput:
        """Starts a recovery backup after a cube slips off the scoop."""
        self._controller.reset()
        self._recover_start_pose = own_pose
        self._recover_retries += 1
        self._state = PlannerState.RECOVER
        return self._make_output(
            cmd=ZERO_VELOCITY_CMD,
            now_s=now_s,
        )

    def _reset_to_idle(self, now_s: float) -> CycleOutput:
        """Clears the current target cube and returns to IDLE."""
        self._target_cube_id = None
        self._push_plan = None
        self._stage_idx = 0
        self._recover_start_pose = None
        self._controller.reset()
        self._state = PlannerState.IDLE
        return self._make_output(
            cmd=ZERO_VELOCITY_CMD,
            now_s=now_s,
        )

    def _current_cube(
        self,
        world_model: WorldModel,
        now_s: float,
    ) -> CubeObservation | None:
        """Looks up the currently pursued cube ID in WorldModel."""
        if self._target_cube_id is None:
            return None

        return world_model.cube_by_id(
            self._target_cube_id,
            now_s,
        )

    def _active_stage_goal(
        self,
        cube: CubeObservation,
        world_model: WorldModel,
        now_s: float,
    ) -> Point2D | None:
        """Returns the active push waypoint, recomputing PushPlan if needed."""
        if self._push_plan is None:
            red_positions = tuple(
                (red.x_m, red.y_m)
                for red in world_model.cubes_by_color(CubeColor.RED, now_s)
            )
            self._push_plan = plan_push(
                (cube.x_m, cube.y_m),
                self._config.mission.pre_zone_goal_xy,
                red_positions,
                config=self._config.navigation,
            )
            self._stage_idx = 0

        if self._push_plan.mode == PushMode.UNREACHABLE or not self._push_plan.waypoints:
            return None

        clamped_idx = min(
            self._stage_idx,
            len(self._push_plan.waypoints) - 1,
        )
        return self._push_plan.waypoints[clamped_idx]

    def _active_approach_pose(
        self,
        cube: CubeObservation,
        stage_goal: Point2D,
    ) -> Pose2D:
        """Returns the approach pose for the active push stage."""
        is_border_first = (
            self._push_plan is not None
            and self._push_plan.mode == PushMode.BORDER_SWEEP
            and self._stage_idx == 0
            and self._push_plan.approach_pose is not None
        )
        if is_border_first and self._push_plan is not None and self._push_plan.approach_pose:
            return self._push_plan.approach_pose

        return compute_approach_pose(
            (cube.x_m, cube.y_m),
            stage_goal,
            offset_m=self._config.navigation.approach_offset_m,
        )

    def _drive_transit(
        self,
        own_pose: Pose2D,
        goal_pose: Pose2D,
        world_model: WorldModel,
        inputs: CycleInputs,
    ) -> VelocityCmd:
        """Plans red- and peer-free transit waypoints and steps the holonomic controller."""
        obstacles = [
            (
                red.x_m,
                red.y_m,
                self._config.navigation.red_transit_radius_m,
            )
            for red in world_model.cubes_by_color(CubeColor.RED, inputs.now_s)
        ]
        peer_pose = world_model.peer_pose(inputs.now_s)
        if peer_pose is not None:
            obstacles.append(
                (
                    peer_pose.x_m,
                    peer_pose.y_m,
                    self._config.navigation.red_transit_radius_m * 2.0,
                )
            )
        waypoints = plan_transit_waypoints(
            (own_pose.x_m, own_pose.y_m),
            (goal_pose.x_m, goal_pose.y_m),
            tuple(obstacles),
            config=self._config.navigation,
        )
        if not waypoints:
            self._controller.reset()
            self._last_path = ((own_pose.x_m, own_pose.y_m),)
            return ZERO_VELOCITY_CMD

        next_xy = waypoints[0]
        is_last = len(waypoints) == 1
        target_theta = (
            goal_pose.theta_rad
            if is_last
            else wrap_angle(
                math.atan2(
                    next_xy[1] - own_pose.y_m,
                    next_xy[0] - own_pose.x_m,
                )
            )
        )
        target_pose = Pose2D(
            x_m=next_xy[0],
            y_m=next_xy[1],
            theta_rad=target_theta,
        )
        self._last_target_pose = goal_pose
        self._last_path = (
            (own_pose.x_m, own_pose.y_m),
            *waypoints,
        )
        return self._controller.step(
            own_pose,
            target_pose,
            inputs.dt_s,
        )

    def _make_output(
        self,
        *,
        cmd: VelocityCmd,
        now_s: float,
        allow_line_escape: bool = False,
        want_zone: bool = False,
        release_zone: bool = False,
        delivered_color: CubeColor | None = None,
    ) -> CycleOutput:
        """Packages the CycleOutput and current PlanSnapshot."""
        self._last_cmd = cmd
        plan = PlanSnapshot(
            name=self._config.mission.robot_name,
            state=self._state,
            target_cube_id=self._target_cube_id or 0,
            target_pose=self._last_target_pose,
            path=self._last_path,
            received_at_s=now_s,
        )
        return CycleOutput(
            state=self._state,
            cmd=cmd,
            allow_line_escape=allow_line_escape,
            want_zone=want_zone,
            release_zone=release_zone,
            delivered_color=delivered_color,
            plan=plan,
        )
