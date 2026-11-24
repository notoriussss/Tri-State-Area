"""Unit tests for motor hardware backends and duty-cycle conversion."""

import pytest
from fleet_core.constants import (
    DEFAULT_FL_CHANNELS,
    DEFAULT_FR_CHANNELS,
    DEFAULT_RL_CHANNELS,
    DEFAULT_RR_CHANNELS,
    MotorDriverType,
    WheelName,
)
from fleet_core.hal_motors import (
    FakeMotors,
    MotorHardwareConfig,
    build_motors,
    to_duty,
)


def test_should_cap_and_invert_duty_cycle_when_converting_ratio() -> None:
    normal = to_duty(1.0, inverted=False, max_duty_ratio=0.9)
    inverted = to_duty(0.5, inverted=True, max_duty_ratio=0.9)
    saturated = to_duty(-3.0, inverted=False, max_duty_ratio=0.9)

    assert normal == pytest.approx(0.9)
    assert inverted == pytest.approx(-0.45)
    assert saturated == pytest.approx(-0.9)


def test_should_apply_default_rewired_inversions_when_driving_fake_motors() -> None:
    config = MotorHardwareConfig(
        driver=MotorDriverType.FAKE,
        max_duty_ratio=1.0,
    )
    motors = build_motors(config)

    motors.set(
        {
            WheelName.FL: 0.5,
            WheelName.FR: 0.5,
            WheelName.RL: 0.5,
            WheelName.RR: 0.5,
        }
    )

    assert isinstance(motors, FakeMotors)
    assert motors.duties == {
        WheelName.FL: -0.5,
        WheelName.FR: 0.5,
        WheelName.RL: 0.5,
        WheelName.RR: -0.5,
    }


def test_should_skip_redundant_writes_when_wheel_ratios_unchanged() -> None:
    motors = FakeMotors(
        MotorHardwareConfig(
            driver=MotorDriverType.FAKE,
            max_duty_ratio=1.0,
        )
    )
    ratios = {
        WheelName.FL: 0.4,
        WheelName.FR: 0.4,
        WheelName.RL: 0.4,
        WheelName.RR: 0.4,
    }

    motors.set(ratios)
    first_writes = motors.write_count
    motors.set(ratios)

    assert first_writes == 4
    assert motors.write_count == 4


def test_should_brake_and_mark_closed_when_closing_fake_motors() -> None:
    motors = FakeMotors(
        MotorHardwareConfig(
            driver=MotorDriverType.FAKE,
            max_duty_ratio=1.0,
        )
    )
    motors.set(
        {
            WheelName.FL: 0.5,
            WheelName.FR: 0.5,
            WheelName.RL: 0.5,
            WheelName.RR: 0.5,
        }
    )

    motors.close()

    assert all(duty == 0.0 for duty in motors.duties.values())
    assert motors.closed is True


def test_should_expose_nov2026_fusion_channels_when_using_default_config() -> None:
    config = MotorHardwareConfig()

    fl = config.channels_for(WheelName.FL)
    fr = config.channels_for(WheelName.FR)
    rl = config.channels_for(WheelName.RL)
    rr = config.channels_for(WheelName.RR)

    assert fl == DEFAULT_FL_CHANNELS
    assert fr == DEFAULT_FR_CHANNELS
    assert rl == DEFAULT_RL_CHANNELS
    assert rr == DEFAULT_RR_CHANNELS


def test_should_raise_value_error_when_driver_is_unknown() -> None:
    config = MotorHardwareConfig(driver="invalid_driver")  # type: ignore[arg-type]

    with pytest.raises(ValueError):
        build_motors(config)
