#!/usr/bin/env python3
"""Bench test for the 4 mecanum wheels and bottom TCRT5000 line sensor without ROS.

Run with the robot wheels off the ground:
    python3 fleet/scripts/test_hat.py
    python3 fleet/scripts/test_hat.py --fake
"""

import argparse
import logging
import sys
import time
from pathlib import Path

FLEET_CORE_DIR = Path(__file__).resolve().parents[1] / "fleet_core"
if str(FLEET_CORE_DIR) not in sys.path:
    sys.path.insert(0, str(FLEET_CORE_DIR))

from fleet_core.constants import (  # noqa: E402
    WHEEL_ORDER,
    MotorDriverType,
    WheelName,
)
from fleet_core.hal_line import (  # noqa: E402
    FakeLineSensor,
    LineSensorConfig,
    Tcrt5000LineSensor,
    adc_counts_to_mv,
)
from fleet_core.hal_motors import (  # noqa: E402
    MotorHardwareConfig,
    build_motors,
)

LOGGER = logging.getLogger("test_hat")
DEFAULT_RATIO = 0.4
DEFAULT_DURATION_S = 1.0
PAUSE_S = 0.5


def parse_args() -> argparse.Namespace:
    """Parses command-line flags for bench testing."""
    parser = argparse.ArgumentParser(
        description="Test each mecanum wheel and the TCRT5000 line sensor.",
    )
    parser.add_argument(
        "--driver",
        type=str,
        default=MotorDriverType.FUSION_HAT.value,
        choices=[driver.value for driver in MotorDriverType],
    )
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--ratio", type=float, default=DEFAULT_RATIO)
    parser.add_argument("--seconds", type=float, default=DEFAULT_DURATION_S)
    return parser.parse_args()


def spin_wheel(
    motors,
    wheel: WheelName,
    *,
    ratio: float,
    seconds: float,
) -> None:
    """Spins a single wheel at the given ratio for seconds, then brakes."""
    ratios = dict.fromkeys(WHEEL_ORDER, 0.0)
    ratios[wheel] = ratio
    motors.set(ratios)
    time.sleep(seconds)
    motors.stop()
    time.sleep(PAUSE_S)


def main() -> None:
    """Executes the bench check across all four wheels and the line sensor."""
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s %(message)s",
    )
    driver = MotorDriverType.FAKE if args.fake else MotorDriverType(args.driver)
    motors = build_motors(MotorHardwareConfig(driver=driver))
    line_cfg = LineSensorConfig()
    sensor = (
        FakeLineSensor(line_cfg)
        if driver == MotorDriverType.FAKE
        else Tcrt5000LineSensor(line_cfg)
    )

    LOGGER.info("Driver=%s. Keep wheels off the ground! Press Ctrl+C to stop.", driver.value)
    counts = sensor.analog_counts()
    mv = adc_counts_to_mv(counts) if counts is not None else None
    LOGGER.info(
        "Line sensor: DO=%d, AO_counts=%s, AO_mv=%s, is_black=%s",
        sensor.raw_value(),
        counts,
        mv,
        sensor.is_black(),
    )

    try:
        for wheel in WHEEL_ORDER:
            LOGGER.info("Wheel %s: forward", wheel.value)
            spin_wheel(motors, wheel, ratio=args.ratio, seconds=args.seconds)
            LOGGER.info("Wheel %s: backward", wheel.value)
            spin_wheel(motors, wheel, ratio=-args.ratio, seconds=args.seconds)
    except KeyboardInterrupt:
        LOGGER.info("Bench test interrupted by operator.")
    finally:
        motors.close()
        sensor.close()


if __name__ == "__main__":
    main()
