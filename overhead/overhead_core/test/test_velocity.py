"""Unit tests for robot velocity estimation."""

import math

from overhead_core.constants import (
    MIN_VELOCITY_DT_S,
    VELOCITY_MAX_GAP_S,
    RobotName,
)
from overhead_core.velocity import VelocityEstimator


def test_should_return_zero_velocity_when_robot_first_sighted() -> None:
    estimator = VelocityEstimator()
    positions = {
        RobotName.PHINEAS: (0.4, 0.3),
    }

    velocities = estimator.update(
        positions,
        now_s=0.0,
    )

    assert velocities[RobotName.PHINEAS] == (0.0, 0.0)


def test_should_converge_to_true_speed_when_moving_constantly() -> None:
    estimator = VelocityEstimator()
    period_s = 0.05
    true_vx_mps = 0.4
    true_vy_mps = -0.2

    velocities: dict[str, tuple[float, float]] = {}
    for step in range(25):
        now_s = step * period_s
        velocities = estimator.update(
            {
                RobotName.PHINEAS: (
                    true_vx_mps * now_s,
                    true_vy_mps * now_s,
                ),
            },
            now_s=now_s,
        )

    vx_mps, vy_mps = velocities[RobotName.PHINEAS]
    assert math.isclose(
        vx_mps,
        true_vx_mps,
        abs_tol=1e-4,
    )
    assert math.isclose(
        vy_mps,
        true_vy_mps,
        abs_tol=1e-4,
    )


def test_should_decay_toward_zero_when_robot_stops() -> None:
    estimator = VelocityEstimator()
    period_s = 0.05
    for step in range(10):
        now_s = step * period_s
        estimator.update(
            {
                RobotName.FERB: (0.5 * now_s, 0.0),
            },
            now_s=now_s,
        )

    stopped_x = 0.5 * (9 * period_s)
    stopped_velocities = estimator.update(
        {RobotName.FERB: (stopped_x, 0.0)},
        now_s=10 * period_s,
    )
    prev_speed = abs(stopped_velocities[RobotName.FERB][0])
    for step in range(11, 22):
        velocities = estimator.update(
            {RobotName.FERB: (stopped_x, 0.0)},
            now_s=step * period_s,
        )
        current_speed = abs(velocities[RobotName.FERB][0])
        assert current_speed < prev_speed
        prev_speed = current_speed

    assert prev_speed < 1e-3


def test_should_estimate_sensible_speed_when_one_frame_is_missing() -> None:
    estimator = VelocityEstimator()
    period_s = 0.05
    true_vx_mps = 0.4
    estimator.update(
        {RobotName.PHINEAS: (0.0, 0.0)},
        now_s=0.0,
    )
    estimator.update(
        {RobotName.PHINEAS: (true_vx_mps * period_s, 0.0)},
        now_s=period_s,
    )
    estimator.update(
        {},
        now_s=2 * period_s,
    )

    velocities = estimator.update(
        {RobotName.PHINEAS: (true_vx_mps * 3 * period_s, 0.0)},
        now_s=3 * period_s,
    )

    vx_mps, vy_mps = velocities[RobotName.PHINEAS]
    assert math.isclose(
        vx_mps,
        0.3,
        abs_tol=1e-6,
    )
    assert math.isclose(
        vy_mps,
        0.0,
        abs_tol=1e-6,
    )


def test_should_keep_previous_estimate_when_dt_below_minimum() -> None:
    estimator = VelocityEstimator()
    estimator.update(
        {RobotName.PHINEAS: (0.0, 0.0)},
        now_s=0.0,
    )
    first_v = estimator.update(
        {RobotName.PHINEAS: (0.05, 0.0)},
        now_s=0.05,
    )[RobotName.PHINEAS]

    tiny_dt_v = estimator.update(
        {RobotName.PHINEAS: (0.90, 0.0)},
        now_s=0.05 + (MIN_VELOCITY_DT_S / 2.0),
    )[RobotName.PHINEAS]
    backward_v = estimator.update(
        {RobotName.PHINEAS: (0.90, 0.0)},
        now_s=0.04,
    )[RobotName.PHINEAS]
    next_valid_v = estimator.update(
        {RobotName.PHINEAS: (0.10, 0.0)},
        now_s=0.10,
    )[RobotName.PHINEAS]

    assert tiny_dt_v == first_v
    assert backward_v == first_v
    assert math.isclose(
        next_valid_v[0],
        0.75,
        abs_tol=1e-6,
    )


def test_should_reset_velocity_when_gap_exceeds_maximum() -> None:
    estimator = VelocityEstimator()
    estimator.update(
        {RobotName.FERB: (0.0, 0.0)},
        now_s=0.0,
    )
    estimator.update(
        {RobotName.FERB: (0.05, 0.0)},
        now_s=0.05,
    )

    after_gap = estimator.update(
        {RobotName.FERB: (0.80, 0.20)},
        now_s=0.05 + VELOCITY_MAX_GAP_S + 0.1,
    )
    following = estimator.update(
        {RobotName.FERB: (0.85, 0.20)},
        now_s=0.05 + VELOCITY_MAX_GAP_S + 0.15,
    )

    assert after_gap[RobotName.FERB] == (0.0, 0.0)
    assert math.isclose(
        following[RobotName.FERB][0],
        0.5,
        abs_tol=1e-6,
    )


def test_should_drop_absent_robot_state_when_gap_exceeds_maximum() -> None:
    estimator = VelocityEstimator()
    estimator.update(
        {RobotName.PHINEAS: (0.0, 0.0)},
        now_s=0.0,
    )
    estimator.update(
        {RobotName.PHINEAS: (0.05, 0.0)},
        now_s=0.05,
    )

    absent_result = estimator.update(
        {},
        now_s=0.05 + VELOCITY_MAX_GAP_S + 0.1,
    )
    reappeared = estimator.update(
        {RobotName.PHINEAS: (0.50, 0.0)},
        now_s=0.05 + VELOCITY_MAX_GAP_S + 0.15,
    )

    assert absent_result == {}
    assert reappeared[RobotName.PHINEAS] == (0.0, 0.0)
