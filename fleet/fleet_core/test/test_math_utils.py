"""Unit tests for math_utils and core enum helpers."""

import math

import pytest
from fleet_core.constants import PlannerState
from fleet_core.math_utils import (
    clamp,
    wrap_angle,
)


def test_should_clamp_value_when_outside_bounds() -> None:
    low = -1.0
    high = 1.0

    below = clamp(-2.5, low, high)
    above = clamp(3.0, low, high)
    inside = clamp(0.25, low, high)

    assert below == pytest.approx(-1.0)
    assert above == pytest.approx(1.0)
    assert inside == pytest.approx(0.25)


def test_should_wrap_angle_into_rep103_interval_when_exceeding_pi() -> None:
    angle_over = 1.5 * math.pi
    angle_under = -1.5 * math.pi

    wrapped_over = wrap_angle(angle_over)
    wrapped_under = wrap_angle(angle_under)

    assert wrapped_over == pytest.approx(-0.5 * math.pi)
    assert wrapped_under == pytest.approx(0.5 * math.pi)


def test_should_return_positive_pi_when_angle_is_negative_pi() -> None:
    angle = -math.pi

    wrapped = wrap_angle(angle)

    assert wrapped == pytest.approx(math.pi)


def test_should_map_parked_to_idle_when_serializing_wire_state() -> None:
    parked = PlannerState.PARKED
    push = PlannerState.PUSH

    parked_wire = parked.to_wire_state()
    push_wire = push.to_wire_state()

    assert parked_wire == PlannerState.IDLE.value
    assert push_wire == PlannerState.PUSH.value
