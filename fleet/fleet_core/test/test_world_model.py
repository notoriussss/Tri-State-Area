"""Unit tests for WorldModel local-clock freshness and latency prediction."""

import math

import pytest
from fleet_core.constants import (
    CubeColor,
    RobotName,
    VirtualSensorEvent,
)
from fleet_core.types import (
    CubeObservation,
    Pose2D,
    RobotObservation,
    RobotSensors,
    VelocityCmd,
    VirtualSensorsSnapshot,
    WorldSnapshot,
)
from fleet_core.world_model import (
    WorldModel,
    WorldModelConfig,
    predict_pose,
)


def _sample_world(received_at_s: float) -> WorldSnapshot:
    return WorldSnapshot(
        received_at_s=received_at_s,
        cubes=(
            CubeObservation(
                cube_id=1,
                color=CubeColor.BLUE,
                x_m=0.40,
                y_m=0.35,
                confidence=0.95,
            ),
            CubeObservation(
                cube_id=2,
                color=CubeColor.RED,
                x_m=0.60,
                y_m=0.35,
                confidence=0.90,
            ),
            CubeObservation(
                cube_id=3,
                color=CubeColor.BLUE,
                x_m=0.70,
                y_m=0.20,
                confidence=0.20,
            ),
        ),
        robots=(
            RobotObservation(
                name=RobotName.PHINEAS.value,
                x_m=0.20,
                y_m=0.35,
                theta_rad=0.0,
                confidence=0.95,
            ),
            RobotObservation(
                name=RobotName.FERB.value,
                x_m=0.50,
                y_m=0.35,
                theta_rad=math.pi,
                confidence=0.90,
            ),
        ),
    )


def test_should_reject_world_state_when_local_age_exceeds_500ms() -> None:
    model = WorldModel(
        WorldModelConfig(
            robot_name=RobotName.PHINEAS.value,
            peer_name=RobotName.FERB.value,
        )
    )
    model.update_world(_sample_world(received_at_s=10.0))

    fresh = model.is_world_fresh(now_s=10.45)
    stale = model.is_world_fresh(now_s=10.55)
    stale_pose = model.own_pose(now_s=10.55)

    assert fresh is True
    assert stale is False
    assert stale_pose is None


def test_should_filter_cubes_by_color_and_confidence_when_world_is_fresh() -> None:
    model = WorldModel(
        WorldModelConfig(
            robot_name=RobotName.PHINEAS.value,
            peer_name=RobotName.FERB.value,
        )
    )
    model.update_world(_sample_world(received_at_s=5.0))

    blue_cubes = model.cubes_by_color(CubeColor.BLUE, now_s=5.1)
    red_cubes = model.cubes_by_color(CubeColor.RED, now_s=5.1)

    assert len(blue_cubes) == 1
    assert blue_cubes[0].cube_id == 1
    assert len(red_cubes) == 1
    assert red_cubes[0].cube_id == 2


def test_should_predict_pose_only_when_latency_exceeds_threshold() -> None:
    pose = Pose2D(x_m=0.20, y_m=0.30, theta_rad=math.pi / 2.0)
    cmd = VelocityCmd(vx_mps=0.20, vy_mps=0.0, wz_radps=0.5)

    unchanged = predict_pose(pose, cmd, latency_s=0.10)
    predicted = predict_pose(pose, cmd, latency_s=0.20)

    assert unchanged == pose
    assert predicted.x_m == pytest.approx(0.20, abs=1e-6)
    assert predicted.y_m == pytest.approx(0.34, abs=1e-6)
    assert predicted.theta_rad == pytest.approx(math.pi / 2.0 + 0.1, abs=1e-6)


def test_should_fallback_to_world_pose_distance_when_virtual_sensors_missing() -> None:
    model = WorldModel(
        WorldModelConfig(
            robot_name=RobotName.PHINEAS.value,
            peer_name=RobotName.FERB.value,
        )
    )
    model.update_world(_sample_world(received_at_s=1.0))

    dist_m = model.robot_distance_m(now_s=1.1)

    assert dist_m == pytest.approx(0.30)


def test_should_expose_virtual_sensor_events_when_sensors_are_fresh() -> None:
    model = WorldModel(
        WorldModelConfig(
            robot_name=RobotName.PHINEAS.value,
            peer_name=RobotName.FERB.value,
        )
    )
    model.update_sensors(
        VirtualSensorsSnapshot(
            received_at_s=2.0,
            robot_distance_m=0.28,
            robots_in_zone=(RobotName.PHINEAS.value,),
            per_robot={
                RobotName.PHINEAS.value: RobotSensors(
                    name=RobotName.PHINEAS.value,
                    nearest_red_id=2,
                    nearest_red_dist_m=0.15,
                    events=(
                        VirtualSensorEvent.LEFT_ZONE.value,
                        VirtualSensorEvent.NEAR_ROBOT.value,
                    ),
                ),
            },
        )
    )

    has_left = model.has_virtual_event(VirtualSensorEvent.LEFT_ZONE.value, now_s=2.1)
    in_zone = model.is_own_in_zone(now_s=2.1)
    dist_m = model.robot_distance_m(now_s=2.1)

    assert has_left is True
    assert in_zone is True
    assert dist_m == pytest.approx(0.28)
