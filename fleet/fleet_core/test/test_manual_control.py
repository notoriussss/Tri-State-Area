"""Unit tests for ManualControlState teleop scaling, mode switch, and timeout."""

import pytest
from fleet_core.manual_control import (
    ManualControlConfig,
    ManualControlState,
)
from fleet_core.types import ZERO_VELOCITY_CMD


def test_should_scale_and_clamp_normalized_manual_command() -> None:
    ctrl = ManualControlState(
        ManualControlConfig(
            max_linear_mps=0.20,
            max_angular_radps=1.0,
            cmd_timeout_s=0.30,
        )
    )

    cmd = ctrl.set_normalized_cmd(1.5, -0.5, 0.25, now_s=1.0)

    assert cmd.vx_mps == pytest.approx(0.20)
    assert cmd.vy_mps == pytest.approx(-0.10)
    assert cmd.wz_radps == pytest.approx(0.25)


def test_should_return_zero_when_manual_command_expires() -> None:
    ctrl = ManualControlState(ManualControlConfig(cmd_timeout_s=0.30))
    ctrl.set_normalized_cmd(1.0, 0.0, 0.0, now_s=5.0)

    fresh = ctrl.current_cmd(now_s=5.20)
    stale = ctrl.current_cmd(now_s=5.35)

    assert fresh.vx_mps > 0.0
    assert stale == ZERO_VELOCITY_CMD


def test_should_zero_manual_command_when_toggling_auto_mode() -> None:
    ctrl = ManualControlState(auto_enabled=False)
    ctrl.set_normalized_cmd(1.0, 0.0, 0.0, now_s=2.0)

    ctrl.set_auto(True)

    assert ctrl.auto_enabled is True
    assert ctrl.current_cmd(now_s=2.05) == ZERO_VELOCITY_CMD
