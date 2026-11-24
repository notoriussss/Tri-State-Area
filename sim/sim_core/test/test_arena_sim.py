"""Closed-loop 2-robot and unit tests for the 2D arena simulator."""

import pytest
from fleet_core.constants import (
    CONTROL_RATE_HZ,
    CubeColor,
    PlannerState,
    RobotName,
    VirtualSensorEvent,
)
from fleet_core.mission import is_cube_in_delivery_zone
from fleet_core.state_machine import (
    CollectorStateMachine,
    CycleInputs,
    default_state_machine_config,
)
from fleet_core.traffic import (
    TrafficConfig,
    TrafficCoordinator,
)
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    PlanSnapshot,
    Pose2D,
    VelocityCmd,
)
from fleet_core.world_model import (
    WorldModel,
    WorldModelConfig,
)
from fleet_core.zone_client import (
    ZoneClient,
    ZoneHeartbeat,
    select_free_delivery_slot,
)
from sim_core.arena_sim import (
    ArenaSimulator,
    MockPerryArbiter,
    SimCube,
)


def _build_agent(
    robot_name: str,
    peer_name: str,
) -> tuple[WorldModel, TrafficCoordinator, ZoneHeartbeat, ZoneClient, CollectorStateMachine]:
    world_model = WorldModel(
        WorldModelConfig(
            robot_name=robot_name,
            peer_name=peer_name,
        )
    )
    traffic = TrafficCoordinator(TrafficConfig(robot_name=robot_name))
    heartbeat = ZoneHeartbeat()
    zone_client = ZoneClient(robot_name)
    fsm = CollectorStateMachine(default_state_machine_config(robot_name))
    return world_model, traffic, heartbeat, zone_client, fsm


def test_should_enforce_exclusive_lease_and_expiry_when_using_mock_perry() -> None:
    arbiter = MockPerryArbiter(lease_duration_s=60.0)

    g1, h1, _ = arbiter.request_zone(RobotName.PHINEAS.value, now_s=1.0)
    g2, h2, _ = arbiter.request_zone(RobotName.FERB.value, now_s=10.0)
    ok_rel, total = arbiter.release_zone(
        RobotName.PHINEAS.value,
        delivered=True,
        color=CubeColor.BLUE.value,
    )
    g3, h3, _ = arbiter.request_zone(RobotName.FERB.value, now_s=15.0)
    expired_holder = arbiter.occupant(now_s=80.0)

    assert g1 is True
    assert h1 == RobotName.PHINEAS.value
    assert g2 is False
    assert h2 == RobotName.PHINEAS.value
    assert ok_rel is True
    assert total == 1
    assert g3 is True
    assert h3 == RobotName.FERB.value
    assert expired_holder == ""


def test_should_detect_black_tape_on_borders_but_not_in_delivery_notch() -> None:
    sim = ArenaSimulator(
        robots={
            RobotName.PHINEAS.value: Pose2D(x_m=1.18, y_m=0.35, theta_rad=0.0),
            RobotName.FERB.value: Pose2D(x_m=0.05, y_m=0.35, theta_rad=3.14159),
        },
        cubes=[],
    )

    phineas_in_notch = sim.is_on_black_line(RobotName.PHINEAS.value)
    ferb_on_left_tape = sim.is_on_black_line(RobotName.FERB.value)

    assert phineas_in_notch is False
    assert ferb_on_left_tape is True


def test_should_deliver_blue_and_yellow_cubes_without_collision_or_red_delivery() -> None:
    sim = ArenaSimulator(
        robots={
            RobotName.PHINEAS.value: Pose2D(x_m=0.22, y_m=0.50, theta_rad=0.0),
            RobotName.FERB.value: Pose2D(x_m=0.22, y_m=0.20, theta_rad=0.0),
        },
        cubes=[
            SimCube(1, CubeColor.BLUE, 0.52, 0.48),
            SimCube(2, CubeColor.YELLOW, 0.52, 0.22),
            SimCube(3, CubeColor.RED, 0.45, 0.35),
        ],
    )
    arbiter = MockPerryArbiter()
    agents = {
        RobotName.PHINEAS.value: _build_agent(RobotName.PHINEAS.value, RobotName.FERB.value),
        RobotName.FERB.value: _build_agent(RobotName.FERB.value, RobotName.PHINEAS.value),
    }
    plans: dict[str, PlanSnapshot | None] = {
        RobotName.PHINEAS.value: None,
        RobotName.FERB.value: None,
    }
    last_cmds: dict[str, VelocityCmd] = {
        RobotName.PHINEAS.value: ZERO_VELOCITY_CMD,
        RobotName.FERB.value: ZERO_VELOCITY_CMD,
    }

    dt_s = 1.0 / CONTROL_RATE_HZ
    max_steps = 900

    for step_idx in range(max_steps):
        now_s = step_idx * dt_s
        world, sensors = sim.snapshot(now_s)
        occupant = arbiter.occupant(now_s)
        next_cmds: dict[str, VelocityCmd] = {}

        for name, (wm, traffic, hb, zc, fsm) in agents.items():
            peer_name = (
                RobotName.FERB.value
                if name == RobotName.PHINEAS.value
                else RobotName.PHINEAS.value
            )
            wm.update_world(world)
            wm.update_sensors(sensors)
            hb.beat(now_s)
            zc.update(occupant=occupant, gate_closed=arbiter.gate_closed)

            own_pose = wm.own_pose(now_s)
            red_positions = tuple(
                (cube.x_m, cube.y_m)
                for cube in wm.cubes_by_color(CubeColor.RED, now_s)
            )
            assert own_pose is not None
            decision = traffic.evaluate(
                own_pose,
                wm.peer_pose(now_s),
                own_state=fsm.state,
                red_positions=red_positions,
                now_s=now_s,
                own_cmd=last_cmds[name],
                peer_plan=plans[peer_name],
                overhead_alert=wm.has_virtual_event(VirtualSensorEvent.NEAR_ROBOT.value, now_s),
            )
            out = fsm.step(
                wm,
                CycleInputs(
                    now_s=now_s,
                    dt_s=dt_s,
                    line_detected=sim.is_on_black_line(name),
                    peer_plan=plans[peer_name],
                    zone_granted=zc.is_granted(hb, now_s),
                    zone_ready=zc.is_ready_to_enter(hb, now_s),
                    delivery_slot_xy=select_free_delivery_slot(wm.all_cubes(now_s)),
                    traffic_must_wait=decision.must_wait,
                    traffic_evasion_cmd=decision.evasion_cmd,
                ),
            )
            plans[name] = out.plan
            next_cmds[name] = out.cmd
            last_cmds[name] = out.cmd

            if out.release_zone:
                zc.request_release(out.delivered_color)
            else:
                zc.update(want=out.want_zone)

            if zc.should_release():
                delivered, color = zc.pop_release()
                arbiter.release_zone(name, delivered=delivered, color=color)
            elif zc.should_request(now_s):
                zc.note_requested(now_s)
                granted, holder, _ = arbiter.request_zone(name, now_s)
                zc.update(occupant=name if granted else holder)

        sim.step(next_cmds, dt_s)

        both_parked = (
            agents[RobotName.PHINEAS.value][4].state == PlannerState.PARKED
            and agents[RobotName.FERB.value][4].state == PlannerState.PARKED
        )
        if both_parked and arbiter.delivered_total >= 2:
            break

    final_world, _ = sim.snapshot(max_steps * dt_s)
    red_cube = next(cube for cube in final_world.cubes if cube.color == CubeColor.RED)

    assert arbiter.delivered_counts[CubeColor.BLUE.value] == 1
    assert arbiter.delivered_counts[CubeColor.YELLOW.value] == 1
    assert arbiter.delivered_counts[CubeColor.RED.value] == 0
    assert is_cube_in_delivery_zone(red_cube) is False
    assert sim.min_robot_distance_m > 0.16
    assert pytest.approx(arbiter.delivered_total) == 2
