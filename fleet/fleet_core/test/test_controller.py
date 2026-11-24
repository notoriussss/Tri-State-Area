"""Unit tests for holonomic go-to-pose control and velocity ramping."""

import math

import pytest
from fleet_core.controller import (
    ControllerConfig,
    PoseController,
    compute_go_to_pose,
    is_at_pose,
    ramp_velocity,
)
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    Pose2D,
    VelocityCmd,
)


def test_should_rotate_world_error_into_body_frame_when_heading_is_90deg() -> None:
    current = Pose2D(x_m=0.30, y_m=0.30, theta_rad=math.pi / 2.0)
    target = Pose2D(x_m=0.40, y_m=0.30, theta_rad=math.pi / 2.0)
    cfg = ControllerConfig(kp_linear=1.0, max_linear_mps=0.5)

    cmd = compute_go_to_pose(current, target, cfg)

    assert cmd.vx_mps == pytest.approx(0.0, abs=1e-6)
    assert cmd.vy_mps == pytest.approx(-0.10, abs=1e-6)
    assert cmd.wz_radps == pytest.approx(0.0, abs=1e-6)


def test_should_preserve_heading_direction_when_saturating_linear_velocity() -> None:
    current = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    target = Pose2D(x_m=0.60, y_m=0.80, theta_rad=0.0)
    cfg = ControllerConfig(kp_linear=2.0, max_linear_mps=0.20)

    cmd = compute_go_to_pose(current, target, cfg)

    assert math.hypot(cmd.vx_mps, cmd.vy_mps) == pytest.approx(0.20)
    assert cmd.vx_mps / cmd.vy_mps == pytest.approx(0.60 / 0.80)


def test_should_output_zero_velocity_when_within_position_and_angle_deadbands() -> None:
    current = Pose2D(x_m=0.50, y_m=0.35, theta_rad=0.02)
    target = Pose2D(x_m=0.51, y_m=0.355, theta_rad=0.0)
    cfg = ControllerConfig(
        pos_deadband_m=0.02,
        angle_deadband_rad=math.radians(5.0),
    )

    cmd = compute_go_to_pose(current, target, cfg)
    arrived = is_at_pose(current, target, cfg)

    assert cmd == ZERO_VELOCITY_CMD
    assert arrived is True


def test_should_limit_acceleration_step_when_ramping_velocity() -> None:
    cfg = ControllerConfig(
        max_linear_accel_mps2=0.4,
        max_angular_accel_radps2=2.0,
    )
    target = VelocityCmd(vx_mps=0.20, vy_mps=0.0, wz_radps=1.0)

    ramped = ramp_velocity(
        ZERO_VELOCITY_CMD,
        target,
        dt_s=0.05,
        config=cfg,
    )

    assert ramped.vx_mps == pytest.approx(0.02)
    assert ramped.vy_mps == pytest.approx(0.0)
    assert ramped.wz_radps == pytest.approx(0.10)


def test_should_reset_ramp_state_when_pose_controller_resets() -> None:
    cfg = ControllerConfig(max_linear_accel_mps2=0.5)
    controller = PoseController(cfg)
    current = Pose2D(x_m=0.0, y_m=0.0, theta_rad=0.0)
    target = Pose2D(x_m=0.5, y_m=0.0, theta_rad=0.0)

    first = controller.step(current, target, dt_s=0.1)
    second = controller.step(current, target, dt_s=0.1)
    controller.reset()
    after_reset = controller.step(current, target, dt_s=0.1)

    assert second.vx_mps > first.vx_mps
    assert after_reset.vx_mps == pytest.approx(first.vx_mps)
