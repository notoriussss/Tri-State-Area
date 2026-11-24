"""Motor hardware backends for collector robots (Fusion HAT+, Cokoino GPIO, Fake)."""

from dataclasses import dataclass
from typing import Any

from fleet_core.constants import (
    BYTE_BITS,
    BYTE_MASK,
    DEFAULT_FL_CHANNELS,
    DEFAULT_FL_INVERTED,
    DEFAULT_FL_PINS,
    DEFAULT_FR_CHANNELS,
    DEFAULT_FR_INVERTED,
    DEFAULT_FR_PINS,
    DEFAULT_MAX_DUTY_RATIO,
    DEFAULT_NSLEEP_PINS,
    DEFAULT_PWM_FREQUENCY_HZ,
    DEFAULT_RL_CHANNELS,
    DEFAULT_RL_INVERTED,
    DEFAULT_RL_PINS,
    DEFAULT_RR_CHANNELS,
    DEFAULT_RR_INVERTED,
    DEFAULT_RR_PINS,
    DUTY_ROUND_DECIMALS,
    FULL_WHEEL_RATIO,
    FUSION_CHANNEL_TO_PORT,
    FUSION_CHANNELS_PER_TIMER,
    FUSION_I2C_ADDRESS,
    FUSION_I2C_BUS,
    FUSION_MAX_POWER_PCT,
    FUSION_PWM_PERIOD,
    FUSION_PWM_PRESCALER,
    FUSION_REG_DUTY,
    FUSION_REG_PERIOD,
    FUSION_REG_PRESCALER,
    WHEEL_ORDER,
    WORD_MASK,
    MotorDriverType,
    WheelName,
)
from fleet_core.math_utils import clamp

ERR_UNKNOWN_MOTOR_DRIVER = "unknown motor driver"
ERR_UNKNOWN_FUSION_CHANNEL = "unknown Fusion HAT+ channel pair"


@dataclass(frozen=True)
class MotorHardwareConfig:
    """Wiring and electrical parameters for all four mecanum wheels."""

    driver: MotorDriverType = MotorDriverType.FUSION_HAT
    fl_channels: tuple[int, int] = DEFAULT_FL_CHANNELS
    fr_channels: tuple[int, int] = DEFAULT_FR_CHANNELS
    rl_channels: tuple[int, int] = DEFAULT_RL_CHANNELS
    rr_channels: tuple[int, int] = DEFAULT_RR_CHANNELS
    fl_pins: tuple[int, int] = DEFAULT_FL_PINS
    fr_pins: tuple[int, int] = DEFAULT_FR_PINS
    rl_pins: tuple[int, int] = DEFAULT_RL_PINS
    rr_pins: tuple[int, int] = DEFAULT_RR_PINS
    nsleep_pins: tuple[int, int] = DEFAULT_NSLEEP_PINS
    fl_inverted: bool = DEFAULT_FL_INVERTED
    fr_inverted: bool = DEFAULT_FR_INVERTED
    rl_inverted: bool = DEFAULT_RL_INVERTED
    rr_inverted: bool = DEFAULT_RR_INVERTED
    pwm_frequency_hz: int = DEFAULT_PWM_FREQUENCY_HZ
    max_duty_ratio: float = DEFAULT_MAX_DUTY_RATIO
    i2c_bus: int = FUSION_I2C_BUS
    i2c_address: int = FUSION_I2C_ADDRESS

    def channels_for(self, wheel: WheelName) -> tuple[int, int]:
        """Returns the Fusion HAT+ PWM channel pair (a, b) for one wheel."""
        mapping = {
            WheelName.FL: self.fl_channels,
            WheelName.FR: self.fr_channels,
            WheelName.RL: self.rl_channels,
            WheelName.RR: self.rr_channels,
        }
        return mapping[wheel]

    def pins_for(self, wheel: WheelName) -> tuple[int, int]:
        """Returns the Cokoino BCM GPIO pin pair (in1, in2) for one wheel."""
        mapping = {
            WheelName.FL: self.fl_pins,
            WheelName.FR: self.fr_pins,
            WheelName.RL: self.rl_pins,
            WheelName.RR: self.rr_pins,
        }
        return mapping[wheel]

    def is_inverted(self, wheel: WheelName) -> bool:
        """Returns whether the given wheel spin direction is inverted in software."""
        mapping = {
            WheelName.FL: self.fl_inverted,
            WheelName.FR: self.fr_inverted,
            WheelName.RL: self.rl_inverted,
            WheelName.RR: self.rr_inverted,
        }
        return mapping[wheel]


def to_duty(
    ratio: float,
    *,
    inverted: bool,
    max_duty_ratio: float,
) -> float:
    """Converts a wheel ratio in [-1, 1] into a capped signed duty cycle.

    Args:
        ratio: Normalized wheel command in [-1.0, 1.0].
        inverted: Flips the sign when the physical motor is wired reversed.
        max_duty_ratio: Duty cycle cap in [0.0, 1.0].

    Returns:
        Rounded signed duty cycle within [-max_duty_ratio, max_duty_ratio].
    """
    signed_ratio = -ratio if inverted else ratio
    clamped = clamp(
        signed_ratio,
        -FULL_WHEEL_RATIO,
        FULL_WHEEL_RATIO,
    )
    return round(
        clamped * max_duty_ratio,
        DUTY_ROUND_DECIMALS,
    )


class MotorBackend:
    """Shared base class for motor drivers: inversion, duty cap, and change filtering."""

    def __init__(self, config: MotorHardwareConfig) -> None:
        """Initializes wheel cache so the first command updates every wheel."""
        self._config = config
        self._last_duty: dict[WheelName, float | None] = dict.fromkeys(WHEEL_ORDER)

    def set(self, ratios: dict[WheelName, float]) -> None:
        """Applies normalized wheel ratios, writing only wheels whose duty changed.

        Args:
            ratios: Normalized wheel commands in [-1.0, 1.0] keyed by WheelName.
        """
        for wheel in WHEEL_ORDER:
            duty = to_duty(
                ratios[wheel],
                inverted=self._config.is_inverted(wheel),
                max_duty_ratio=self._config.max_duty_ratio,
            )
            if duty != self._last_duty[wheel]:
                self._last_duty[wheel] = duty
                self._write(
                    wheel,
                    duty,
                )

    def stop(self) -> None:
        """Brakes all four wheels immediately."""
        self.set(
            dict.fromkeys(
                WHEEL_ORDER,
                0.0,
            )
        )

    def close(self) -> None:
        """Stops all motors and releases underlying hardware resources."""
        self.stop()

    def _write(
        self,
        wheel: WheelName,
        duty: float,
    ) -> None:
        """Writes a signed duty cycle to a single physical or fake wheel."""
        raise NotImplementedError


class FakeMotors(MotorBackend):
    """In-memory motor backend for unit tests and host-side simulation."""

    def __init__(self, config: MotorHardwareConfig | None = None) -> None:
        """Starts with every wheel stopped and zero hardware writes."""
        super().__init__(config or MotorHardwareConfig(driver=MotorDriverType.FAKE))
        self.duties: dict[WheelName, float] = dict.fromkeys(
            WHEEL_ORDER,
            0.0,
        )
        self.write_count: int = 0
        self.closed: bool = False

    def _write(
        self,
        wheel: WheelName,
        duty: float,
    ) -> None:
        """Records the applied duty cycle in memory."""
        self.duties[wheel] = duty
        self.write_count += 1

    def close(self) -> None:
        """Brakes motors and records closure."""
        super().close()
        self.closed = True


class GpioMotors(MotorBackend):
    """Four motors on the Cokoino CKK0018 HAT (2x DRV8833) via gpiozero."""

    def __init__(self, config: MotorHardwareConfig) -> None:
        """Enables both DRV8833 H-bridges and allocates PWM devices."""
        super().__init__(config)
        from gpiozero import (
            Motor,
            OutputDevice,
        )

        self._nsleep = [
            OutputDevice(
                pin,
                active_high=True,
                initial_value=True,
            )
            for pin in config.nsleep_pins
        ]
        self._motors = {}
        for wheel in WHEEL_ORDER:
            in1, in2 = config.pins_for(wheel)
            motor = Motor(
                forward=in1,
                backward=in2,
                pwm=True,
            )
            motor.forward_device.frequency = config.pwm_frequency_hz
            motor.backward_device.frequency = config.pwm_frequency_hz
            self._motors[wheel] = motor

    def _write(
        self,
        wheel: WheelName,
        duty: float,
    ) -> None:
        """Applies a signed duty cycle to the gpiozero motor."""
        self._motors[wheel].value = duty

    def close(self) -> None:
        """Stops and releases every motor and nSLEEP output pin."""
        for motor in self._motors.values():
            motor.stop()
            motor.close()
        for pin in self._nsleep:
            pin.off()
            pin.close()


class FusionHatMotors(MotorBackend):
    """Four motors on the SunFounder Fusion HAT+ via SDK or SMBus fallback."""

    def __init__(self, config: MotorHardwareConfig) -> None:
        """Initializes Fusion HAT+ motors using fusion_hat SDK when available."""
        super().__init__(config)
        self._sdk_motors: dict[WheelName, Any] | None = None
        self._bus: Any = None
        self._address = config.i2c_address
        self._channels = {
            wheel: config.channels_for(wheel)
            for wheel in WHEEL_ORDER
        }
        try:
            self._sdk_motors = self._init_sdk_motors()
        except ImportError:
            self._init_smbus_motors(config.i2c_bus)
        self.stop()

    def _init_sdk_motors(self) -> dict[WheelName, Any]:
        """Opens motor ports M0..M3 using SunFounder's fusion_hat kernel driver SDK."""
        from fusion_hat.motor import Motor

        motors: dict[WheelName, Any] = {}
        for wheel in WHEEL_ORDER:
            channels = self._channels[wheel]
            port = FUSION_CHANNEL_TO_PORT.get(channels)
            if port is None:
                raise ValueError(f"{ERR_UNKNOWN_FUSION_CHANNEL}: {channels!r}")

            motor = Motor(port)
            motor.set_is_reverse(False)
            motors[wheel] = motor

        return motors

    def _init_smbus_motors(self, i2c_bus: int) -> None:
        """Falls back to direct SMBus I2C PWM timer initialization."""
        from smbus2 import SMBus

        self._bus = SMBus(
            i2c_bus,
            force=True,
        )
        timers = {
            channel // FUSION_CHANNELS_PER_TIMER
            for pair in self._channels.values()
            for channel in pair
        }
        for timer in sorted(timers):
            self._write_word(
                FUSION_REG_PERIOD + timer,
                FUSION_PWM_PERIOD,
            )
            self._write_word(
                FUSION_REG_PRESCALER + timer,
                FUSION_PWM_PRESCALER,
            )

    def _write_word(
        self,
        register: int,
        value: int,
    ) -> None:
        """Writes a byte-swapped 16-bit word to a Fusion HAT+ MCU register."""
        masked = value & WORD_MASK
        swapped = ((masked & BYTE_MASK) << BYTE_BITS) | (masked >> BYTE_BITS)
        self._bus.write_word_data(
            self._address,
            register,
            swapped,
        )

    def _write(
        self,
        wheel: WheelName,
        duty: float,
    ) -> None:
        """Writes the signed duty cycle via the fusion_hat SDK or direct I2C PWM."""
        if self._sdk_motors is not None:
            # The fusion_hat kernel SDK drives M0..M3 with opposite polarity to direct SMBus PWM.
            self._sdk_motors[wheel].power(-duty * FUSION_MAX_POWER_PCT)
            return

        channel_a, channel_b = self._channels[wheel]
        counts = int(abs(duty) * FUSION_PWM_PERIOD)
        self._write_word(
            FUSION_REG_DUTY + channel_a,
            counts if duty > 0.0 else 0,
        )
        self._write_word(
            FUSION_REG_DUTY + channel_b,
            counts if duty < 0.0 else 0,
        )

    def close(self) -> None:
        """Brakes all motors and closes the I2C bus when opened."""
        super().close()
        if self._bus is not None:
            self._bus.close()


def build_motors(config: MotorHardwareConfig) -> MotorBackend:
    """Instantiates the motor backend configured in MotorHardwareConfig.

    Args:
        config: Motor driver type, pin/channel mapping, and electrical limits.

    Returns:
        Configured MotorBackend instance.
    """
    driver_map: dict[MotorDriverType, type[MotorBackend]] = {
        MotorDriverType.FAKE: FakeMotors,
        MotorDriverType.GPIO: GpioMotors,
        MotorDriverType.FUSION_HAT: FusionHatMotors,
    }
    driver_cls = driver_map.get(config.driver)
    if driver_cls is None:
        raise ValueError(f"{ERR_UNKNOWN_MOTOR_DRIVER}: {config.driver!r}")

    return driver_cls(config)
