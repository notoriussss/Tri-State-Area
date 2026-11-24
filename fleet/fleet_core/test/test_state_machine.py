"""Unit tests for the unified autonomous CollectorStateMachine."""

import pytest
from fleet_core.constants import (
    CubeColor,
    PlannerState,
    RobotName,
    VirtualSensorEvent,
)
from fleet_core.state_machine import (
    CollectorStateMachine,
    CycleInputs,
    compute_line_escape_cmd,
    default_state_machine_config,
)
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    CubeObservation,
    Pose2D,
    RobotObservation,
    RobotSensors,
    VirtualSensorsSnapshot,
    WorldSnapshot,
)
from fleet_core.world_model import (
    WorldModel,
    WorldModelConfig,
)


def _make_world_model() -> WorldModel:
    return WorldModel(
        WorldModelConfig(
            robot_name=RobotName.PHINEAS.value,
            peer_name=RobotName.FERB.value,
        )
    )


def _update_scene(
    model: WorldModel,
    *,
    now_s: float,
    robot_pose: Pose2D,
    cubes: tuple[CubeObservation, ...],
    events: tuple[str, ...] = (),
    in_zone: bool = False,
) -> None:
    model.update_world(
        WorldSnapshot(
            received_at_s=now_s,
            cubes=cubes,
            robots=(
                RobotObservation(
                    name=RobotName.PHINEAS.value,
                    x_m=robot_pose.x_m,
                    y_m=robot_pose.y_m,
                    theta_rad=robot_pose.theta_rad,
                    confidence=0.95,
                ),
            ),
        )
    )
    model.update_sensors(
        VirtualSensorsSnapshot(
            received_at_s=now_s,
            robot_distance_m=-1.0,
            robots_in_zone=(RobotName.PHINEAS.value,) if in_zone else (),
            per_robot={
                RobotName.PHINEAS.value: RobotSensors(
                    name=RobotName.PHINEAS.value,
                    nearest_red_id=0,
                    nearest_red_dist_m=-1.0,
                    events=events,
                ),
            },
        )
    )


def test_should_enter_stopped_with_zero_velocity_when_camera_is_stale() -> None:
    model = _make_world_model()
    fsm = CollectorStateMachine(default_state_machine_config(RobotName.PHINEAS.value))
    _update_scene(
        model,
        now_s=1.0,
        robot_pose=Pose2D(0.30, 0.45, 0.0),
        cubes=(CubeObservation(1, CubeColor.BLUE, 0.50, 0.45, 0.95),),
    )

    out = fsm.step(
        model,
        CycleInputs(now_s=1.6, dt_s=0.05, line_detected=False),
    )

    assert out.state == PlannerState.STOPPED
    assert out.cmd == ZERO_VELOCITY_CMD


def test_should_command_inward_non_forward_escape_when_line_is_detected() -> None:
    model = _make_world_model()
    fsm = CollectorStateMachine(default_state_machine_config(RobotName.PHINEAS.value))
    pose_at_left_edge = Pose2D(0.08, 0.35, 3.14159)
    _update_scene(
        model,
        now_s=2.0,
        robot_pose=pose_at_left_edge,
        cubes=(CubeObservation(1, CubeColor.BLUE, 0.50, 0.45, 0.95),),
    )

    escape_out = fsm.step(
        model,
        CycleInputs(now_s=2.0, dt_s=0.05, line_detected=True),
    )
    direct_cmd = compute_line_escape_cmd(Pose2D(0.58, 0.05, 0.0))

    assert escape_out.allow_line_escape is True
    assert escape_out.cmd.vx_mps <= 0.0
    assert direct_cmd.vx_mps <= 0.0
    assert direct_cmd.vy_mps > 0.0


def test_should_complete_full_delivery_cycle_when_nominal() -> None:
    model = _make_world_model()
    fsm = CollectorStateMachine(default_state_machine_config(RobotName.PHINEAS.value))

    # 1) IDLE -> GO_TO_APPROACH
    _update_scene(
        model,
        now_s=1.0,
        robot_pose=Pose2D(0.20, 0.45, 0.0),
        cubes=(CubeObservation(7, CubeColor.BLUE, 0.50, 0.45, 0.95),),
    )
    out_approach = fsm.step(model, CycleInputs(now_s=1.0, dt_s=0.05, line_detected=False))

    # 2) At approach pose A=(0.36, 0.45, 0.0) -> PUSH
    _update_scene(
        model,
        now_s=2.0,
        robot_pose=Pose2D(0.36, 0.45, 0.0),
        cubes=(CubeObservation(7, CubeColor.BLUE, 0.50, 0.45, 0.95),),
    )
    out_push = fsm.step(model, CycleInputs(now_s=2.0, dt_s=0.05, line_detected=False))

    # 3) Cube reaches pre-zone (x >= 0.80) -> REQUEST_ZONE (waiting for Perry)
    _update_scene(
        model,
        now_s=3.0,
        robot_pose=Pose2D(0.73, 0.45, 0.0),
        cubes=(CubeObservation(7, CubeColor.BLUE, 0.85, 0.45, 0.95),),
    )
    out_req = fsm.step(
        model,
        CycleInputs(now_s=3.0, dt_s=0.05, line_detected=False, zone_granted=False),
    )

    # 4) Perry grants turn -> DELIVER
    out_deliver = fsm.step(
        model,
        CycleInputs(now_s=3.1, dt_s=0.05, line_detected=False, zone_granted=True),
    )

    # 5) Cube crosses green line -> LEAVE (straight reverse, no rotation)
    _update_scene(
        model,
        now_s=4.0,
        robot_pose=Pose2D(1.08, 0.355, 0.0),
        cubes=(CubeObservation(7, CubeColor.BLUE, 1.20, 0.355, 0.95),),
        in_zone=True,
    )
    out_leave = fsm.step(
        model,
        CycleInputs(now_s=4.0, dt_s=0.05, line_detected=False, zone_granted=True),
    )

    # 6) Overhead emits left:zone -> release_zone=True & back to IDLE
    _update_scene(
        model,
        now_s=5.0,
        robot_pose=Pose2D(0.88, 0.355, 0.0),
        cubes=(CubeObservation(7, CubeColor.BLUE, 1.20, 0.355, 0.95),),
        events=(VirtualSensorEvent.LEFT_ZONE.value,),
        in_zone=False,
    )
    out_done = fsm.step(
        model,
        CycleInputs(now_s=5.0, dt_s=0.05, line_detected=False, zone_granted=True),
    )

    assert out_approach.state == PlannerState.GO_TO_APPROACH
    assert out_push.state == PlannerState.PUSH
    assert out_req.state == PlannerState.REQUEST_ZONE
    assert out_req.want_zone is True
    assert out_deliver.state == PlannerState.DELIVER
    assert out_leave.state == PlannerState.LEAVE
    assert out_leave.cmd.vx_mps < 0.0
    assert out_leave.cmd.vy_mps == pytest.approx(0.0)
    assert out_leave.cmd.wz_radps == pytest.approx(0.0)
    assert out_done.state == PlannerState.IDLE
    assert out_done.release_zone is True
    assert out_done.delivered_color == CubeColor.BLUE


def test_should_recover_and_retry_when_cube_slips_off_scoop() -> None:
    model = _make_world_model()
    fsm = CollectorStateMachine(default_state_machine_config(RobotName.PHINEAS.value))

    # Enter PUSH at approach pose
    _update_scene(
        model,
        now_s=1.0,
        robot_pose=Pose2D(0.36, 0.45, 0.0),
        cubes=(CubeObservation(3, CubeColor.BLUE, 0.50, 0.45, 0.95),),
    )
    fsm.step(model, CycleInputs(now_s=1.0, dt_s=0.05, line_detected=False))

    # Cube slips 15 cm sideways out of scoop -> RECOVER
    _update_scene(
        model,
        now_s=1.1,
        robot_pose=Pose2D(0.38, 0.45, 0.0),
        cubes=(CubeObservation(3, CubeColor.BLUE, 0.50, 0.60, 0.95),),
    )
    out_slip = fsm.step(model, CycleInputs(now_s=1.1, dt_s=0.05, line_detected=False))
    out_backing = fsm.step(model, CycleInputs(now_s=1.15, dt_s=0.05, line_detected=False))

    # Robot completes 12 cm straight backup -> GO_TO_APPROACH
    _update_scene(
        model,
        now_s=2.0,
        robot_pose=Pose2D(0.25, 0.45, 0.0),
        cubes=(CubeObservation(3, CubeColor.BLUE, 0.50, 0.60, 0.95),),
    )
    out_recovered = fsm.step(model, CycleInputs(now_s=2.0, dt_s=0.05, line_detected=False))

    assert out_slip.state == PlannerState.RECOVER
    assert out_backing.cmd.vx_mps < 0.0
    assert out_recovered.state == PlannerState.GO_TO_APPROACH


def test_should_park_when_no_own_cubes_remain_and_pause_on_traffic_wait() -> None:
    model = _make_world_model()
    fsm = CollectorStateMachine(default_state_machine_config(RobotName.PHINEAS.value))
    _update_scene(
        model,
        now_s=1.0,
        robot_pose=Pose2D(0.50, 0.35, 0.0),
        cubes=(CubeObservation(2, CubeColor.YELLOW, 0.60, 0.25, 0.95),),
    )

    out_parked = fsm.step(model, CycleInputs(now_s=1.0, dt_s=0.05, line_detected=False))
    out_wait = fsm.step(
        model,
        CycleInputs(now_s=1.1, dt_s=0.05, line_detected=False, traffic_must_wait=True),
    )
    out_resume = fsm.step(
        model,
        CycleInputs(now_s=1.2, dt_s=0.05, line_detected=False, traffic_must_wait=False),
    )

    assert out_parked.state == PlannerState.PARKED
    assert out_parked.plan.state == PlannerState.PARKED
    assert out_wait.state == PlannerState.WAIT
    assert out_wait.cmd == ZERO_VELOCITY_CMD
    assert out_resume.state == PlannerState.PARKED
