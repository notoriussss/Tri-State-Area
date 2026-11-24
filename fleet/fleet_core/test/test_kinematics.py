"""Unit tests for mecanum inverse kinematics."""

import pytest
from fleet_core.constants import (
    HALF_TRACK_M,
    HALF_WHEELBASE_M,
    WHEEL_RADIUS_M,
    WheelName,
)
from fleet_core.kinematics import (
    max_wheel_rad_s_from_linear,
    scale_to_limit,
    wheel_ratios,
    wheel_speeds,
)
from fleet_core.types import (
    MecanumGeometry,
    VelocityCmd,
)

GEO = MecanumGeometry(
    wheel_radius_m=WHEEL_RADIUS_M,
    half_wheelbase_m=HALF_WHEELBASE_M,
    half_track_m=HALF_TRACK_M,
)


def test_should_spin_all_wheels_equally_when_driving_forward() -> None:
    cmd = VelocityCmd(
        vx_mps=0.1,
        vy_mps=0.0,
        wz_radps=0.0,
    )

    speeds = wheel_speeds(cmd, GEO)

    expected = 0.1 / WHEEL_RADIUS_M
    assert speeds[WheelName.FL] == pytest.approx(expected)
    assert speeds[WheelName.FR] == pytest.approx(expected)
    assert speeds[WheelName.RL] == pytest.approx(expected)
    assert speeds[WheelName.RR] == pytest.approx(expected)


def test_should_spin_diagonal_pairs_opposite_when_strafing_left() -> None:
    cmd = VelocityCmd(
        vx_mps=0.0,
        vy_mps=0.1,
        wz_radps=0.0,
    )

    speeds = wheel_speeds(cmd, GEO)

    assert speeds[WheelName.FL] < 0.0
    assert speeds[WheelName.RR] < 0.0
    assert speeds[WheelName.FR] > 0.0
    assert speeds[WheelName.RL] > 0.0
    assert speeds[WheelName.FL] == pytest.approx(-speeds[WheelName.FR])


def test_should_spin_right_side_forward_when_turning_ccw() -> None:
    cmd = VelocityCmd(
        vx_mps=0.0,
        vy_mps=0.0,
        wz_radps=1.0,
    )

    speeds = wheel_speeds(cmd, GEO)

    expected_right = (HALF_WHEELBASE_M + HALF_TRACK_M) / WHEEL_RADIUS_M
    assert speeds[WheelName.FR] == pytest.approx(expected_right)
    assert speeds[WheelName.RR] == pytest.approx(expected_right)
    assert speeds[WheelName.FL] == pytest.approx(-expected_right)
    assert speeds[WheelName.RL] == pytest.approx(-expected_right)


def test_should_preserve_ratios_when_scaling_above_limit() -> None:
    speeds = {
        WheelName.FL: 10.0,
        WheelName.FR: -5.0,
        WheelName.RL: 2.0,
        WheelName.RR: 0.0,
    }

    scaled = scale_to_limit(speeds, 5.0)

    assert scaled == pytest.approx(
        {
            WheelName.FL: 5.0,
            WheelName.FR: -2.5,
            WheelName.RL: 1.0,
            WheelName.RR: 0.0,
        }
    )


def test_should_keep_speeds_unchanged_when_below_limit() -> None:
    speeds = {
        WheelName.FL: 1.0,
        WheelName.FR: -1.0,
        WheelName.RL: 0.5,
        WheelName.RR: 0.0,
    }

    scaled = scale_to_limit(speeds, 5.0)

    assert scaled == speeds


def test_should_saturate_all_wheels_together_when_command_exceeds_max_speed() -> None:
    cmd = VelocityCmd(
        vx_mps=1.0,
        vy_mps=0.5,
        wz_radps=0.0,
    )

    ratios = wheel_ratios(cmd, GEO, max_wheel_rad_s=1.0)

    assert max(abs(value) for value in ratios.values()) == pytest.approx(1.0)
    assert ratios[WheelName.FL] == pytest.approx(ratios[WheelName.RR])
    assert ratios[WheelName.FR] == pytest.approx(ratios[WheelName.RL])


def test_should_raise_value_error_when_radius_or_max_speed_non_positive() -> None:
    bad_geo = MecanumGeometry(
        wheel_radius_m=0.0,
        half_wheelbase_m=HALF_WHEELBASE_M,
        half_track_m=HALF_TRACK_M,
    )
    cmd = VelocityCmd(vx_mps=0.1, vy_mps=0.0, wz_radps=0.0)

    with pytest.raises(ValueError):
        wheel_speeds(cmd, bad_geo)

    with pytest.raises(ValueError):
        wheel_ratios(cmd, GEO, max_wheel_rad_s=0.0)

    with pytest.raises(ValueError):
        max_wheel_rad_s_from_linear(0.24, -0.01)
