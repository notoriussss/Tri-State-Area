"""Unit tests for HalGuard local black-line and 300 ms watchdog reflexes."""

from fleet_core.constants import (
    HAL_WATCHDOG_TIMEOUT_S,
    MotorDriverType,
    RobotMode,
    WheelName,
)
from fleet_core.hal_guard import (
    HalGuard,
    HalGuardConfig,
)
from fleet_core.hal_line import (
    FakeLineSensor,
    LineSensorConfig,
)
from fleet_core.hal_motors import (
    FakeMotors,
    MotorHardwareConfig,
)
from fleet_core.types import VelocityCmd


def _build_guard(*, confirm_reads: int = 1) -> tuple[HalGuard, FakeMotors, FakeLineSensor]:
    motors = FakeMotors(
        MotorHardwareConfig(
            driver=MotorDriverType.FAKE,
            max_duty_ratio=1.0,
        )
    )
    sensor = FakeLineSensor(
        LineSensorConfig(
            confirm_reads=confirm_reads,
            black_mv=900,
        )
    )
    guard = HalGuard(
        motors,
        sensor,
        HalGuardConfig(watchdog_timeout_s=HAL_WATCHDOG_TIMEOUT_S),
    )
    return guard, motors, sensor


def test_should_drive_motors_and_report_run_when_clear_and_fresh() -> None:
    guard, motors, _ = _build_guard()
    cmd = VelocityCmd(vx_mps=0.12, vy_mps=0.0, wz_radps=0.0)

    status = guard.drive(cmd, now_s=1.0)

    assert status.mode == RobotMode.RUN
    assert status.line_detected is False
    assert status.cmd_age_ms == 0
    assert motors.duties[WheelName.FR] > 0.0


def test_should_stop_immediately_when_black_line_detected() -> None:
    guard, motors, sensor = _build_guard()
    cmd = VelocityCmd(vx_mps=0.12, vy_mps=0.0, wz_radps=0.0)
    guard.drive(cmd, now_s=1.0)
    sensor.set_reading(do_value=1, ao_counts=3000)

    status = guard.drive(cmd, now_s=1.05)

    assert status.mode == RobotMode.STOPPED_LINE
    assert status.line_detected is True
    assert all(duty == 0.0 for duty in motors.duties.values())


def test_should_allow_reverse_escape_but_block_forward_when_on_black_line() -> None:
    guard, motors, sensor = _build_guard()
    sensor.set_reading(do_value=1, ao_counts=3000)
    forward_cmd = VelocityCmd(vx_mps=0.10, vy_mps=0.0, wz_radps=0.0)
    reverse_cmd = VelocityCmd(vx_mps=-0.10, vy_mps=0.0, wz_radps=0.0)

    blocked_status = guard.drive(
        forward_cmd,
        now_s=1.0,
        allow_line_escape=True,
    )
    blocked_duties = dict(motors.duties)
    escape_status = guard.drive(
        reverse_cmd,
        now_s=1.05,
        allow_line_escape=True,
    )

    assert blocked_status.mode == RobotMode.STOPPED_LINE
    assert all(duty == 0.0 for duty in blocked_duties.values())
    assert escape_status.mode == RobotMode.STOPPED_LINE
    assert motors.duties[WheelName.FR] < 0.0


def test_should_stop_on_watchdog_when_command_age_exceeds_300ms() -> None:
    guard, motors, _ = _build_guard()
    cmd = VelocityCmd(vx_mps=0.12, vy_mps=0.0, wz_radps=0.0)
    guard.drive(cmd, now_s=10.0)

    fresh_status = guard.tick(now_s=10.25)
    stale_status = guard.tick(now_s=10.35)

    assert fresh_status.mode == RobotMode.RUN
    assert fresh_status.cmd_age_ms == 250
    assert stale_status.mode == RobotMode.STOPPED_WATCHDOG
    assert stale_status.cmd_age_ms == 350
    assert all(duty == 0.0 for duty in motors.duties.values())


def test_should_close_both_backends_when_guard_closes() -> None:
    guard, motors, sensor = _build_guard()

    guard.close()

    assert motors.closed is True
    assert sensor.closed is True
