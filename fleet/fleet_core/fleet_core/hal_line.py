"""Bottom TCRT5000 line sensor backend (DO on GPIO 27 + optional Fusion HAT+ ADC)."""

import logging
from dataclasses import dataclass
from typing import Any

from fleet_core.constants import (
    ADC_I2C_ADDRESS,
    ADC_I2C_BUS,
    ADC_MAX_COUNTS,
    ADC_REFERENCE_MV,
    ADC_REG_BASE,
    ADC_REG_STRIDE,
    BYTE_BITS,
    BYTE_MASK,
    DEFAULT_LINE_ADC_CHANNEL,
    DEFAULT_LINE_BLACK_MV,
    DEFAULT_LINE_BLACK_VALUE,
    DEFAULT_LINE_CONFIRM_READS,
    DEFAULT_LINE_SENSOR_PIN,
    LGPIO_CHIP_CANDIDATES,
)

LOGGER = logging.getLogger(__name__)
ERR_GPIO_OPEN_FAILED = "unable to open lgpio chip for line sensor"


@dataclass(frozen=True)
class LineSensorConfig:
    """Wiring and filtering parameters for the bottom TCRT5000 line sensor."""

    pin: int = DEFAULT_LINE_SENSOR_PIN
    black_value: int = DEFAULT_LINE_BLACK_VALUE
    confirm_reads: int = DEFAULT_LINE_CONFIRM_READS
    adc_channel: int | None = DEFAULT_LINE_ADC_CHANNEL
    black_mv: int | None = DEFAULT_LINE_BLACK_MV
    i2c_bus: int = ADC_I2C_BUS
    i2c_address: int = ADC_I2C_ADDRESS


def word_to_adc_counts(smbus_word: int) -> int:
    """Decodes a big-endian 12-bit ADC word read via little-endian SMBus.

    Args:
        smbus_word: Raw 16-bit integer returned by SMBus.read_word_data.

    Returns:
        Decoded integer ADC counts.
    """
    return ((smbus_word & BYTE_MASK) << BYTE_BITS) | (smbus_word >> BYTE_BITS)


def adc_counts_to_mv(counts: int) -> int:
    """Converts 12-bit ADC counts to millivolts using a 3.3 V reference.

    Args:
        counts: Raw 12-bit ADC reading in [0, 4095].

    Returns:
        Equivalent voltage in millivolts.
    """
    return round(counts * ADC_REFERENCE_MV / (ADC_MAX_COUNTS + 1))


def is_raw_black(
    do_value: int,
    *,
    black_value: int,
    ao_mv: int | None,
    black_mv: int | None,
) -> bool:
    """Evaluates whether a single DO/AO sensor sample indicates black border tape.

    Args:
        do_value: Digital output level (0 or 1) from the TCRT5000 DO pin.
        black_value: Digital level corresponding to black tape.
        ao_mv: Optional analog reading in millivolts from the Fusion HAT+ ADC.
        black_mv: Optional analog threshold in millivolts for black tape.

    Returns:
        True when either the digital output or the analog voltage indicates black.
    """
    if do_value == black_value:
        return True

    is_analog_black = (
        ao_mv is not None
        and black_mv is not None
        and ao_mv >= black_mv
    )
    return is_analog_black


class BlackConfirm:
    """Debounces line readings by requiring consecutive black samples."""

    def __init__(self, confirm_reads: int = DEFAULT_LINE_CONFIRM_READS) -> None:
        """Initializes the debounce filter in the white (clear) state."""
        self._confirm_reads = max(
            1,
            int(confirm_reads),
        )
        self.streak: int = 0

    def update(self, raw_black: bool) -> bool:
        """Feeds one sample and returns True once the black streak reaches threshold.

        Args:
            raw_black: Unfiltered boolean sample for the current control cycle.

        Returns:
            True when at least confirm_reads consecutive black samples occurred.
        """
        if raw_black:
            self.streak += 1
        elif self.streak > 0:
            label = "line" if self.streak >= self._confirm_reads else "ignored"
            LOGGER.debug(
                "Line sensor black run ended: %d reads (%s)",
                self.streak,
                label,
            )
            self.streak = 0

        return self.streak >= self._confirm_reads

    def reset(self) -> None:
        """Resets the consecutive black sample counter to zero."""
        self.streak = 0


class LineSensorBackend:
    """Abstract interface for the onboard bottom line sensor."""

    def is_black(self) -> bool:
        """Returns True when black tape is confirmed under the sensor."""
        raise NotImplementedError

    def raw_value(self) -> int:
        """Returns the raw digital output level (0 or 1)."""
        raise NotImplementedError

    def analog_counts(self) -> int | None:
        """Returns raw 12-bit ADC counts, or None when ADC is unavailable."""
        raise NotImplementedError

    def close(self) -> None:
        """Releases GPIO and I2C resources."""


class FakeLineSensor(LineSensorBackend):
    """Deterministic fake line sensor for unit tests and simulation."""

    def __init__(self, config: LineSensorConfig | None = None) -> None:
        """Initializes the fake sensor on white floor with zero ADC counts."""
        self._config = config or LineSensorConfig()
        self._confirm = BlackConfirm(self._config.confirm_reads)
        self._do_value: int = 0 if self._config.black_value == 1 else 1
        self._ao_counts: int | None = 0 if self._config.adc_channel is not None else None
        self.closed: bool = False

    def set_reading(
        self,
        *,
        do_value: int,
        ao_counts: int | None = None,
    ) -> None:
        """Injects synthetic DO and optional ADC counts for testing."""
        self._do_value = do_value
        self._ao_counts = ao_counts

    def is_black(self) -> bool:
        """Evaluates the injected reading through the shared confirmation filter."""
        ao_mv = adc_counts_to_mv(self._ao_counts) if self._ao_counts is not None else None
        raw_black = is_raw_black(
            self._do_value,
            black_value=self._config.black_value,
            ao_mv=ao_mv,
            black_mv=self._config.black_mv,
        )
        return self._confirm.update(raw_black)

    def raw_value(self) -> int:
        """Returns the injected digital value."""
        return self._do_value

    def analog_counts(self) -> int | None:
        """Returns the injected ADC counts."""
        return self._ao_counts

    def close(self) -> None:
        """Marks the fake sensor as closed."""
        self.closed = True


class Tcrt5000LineSensor(LineSensorBackend):
    """Hardware TCRT5000 sensor read via lgpio/gpiozero (DO) and Fusion HAT+ ADC (AO).

    Power the TCRT5000 module from 3.3 V so DO and AO never exceed 3.3 V on the Pi 5.
    """

    def __init__(self, config: LineSensorConfig | None = None) -> None:
        """Opens the GPIO digital input and optional Fusion HAT+ ADC channel."""
        self._config = config or LineSensorConfig()
        self._confirm = BlackConfirm(self._config.confirm_reads)
        self._lgpio_mod: Any = None
        self._lgpio_handle: int | None = None
        self._dev: Any = None
        self._adc: Any = None
        self._bus: Any = None

        self._open_gpio()
        if self._config.adc_channel is not None:
            self._open_adc(int(self._config.adc_channel))

    def _open_gpio(self) -> None:
        """Opens DO input via lgpio (trying chips 0 and 4) or falls back to gpiozero."""
        try:
            import lgpio
        except ImportError:
            from gpiozero import DigitalInputDevice

            self._dev = DigitalInputDevice(
                self._config.pin,
                pull_up=False,
            )
            return

        self._lgpio_mod = lgpio
        for chip in LGPIO_CHIP_CANDIDATES:
            handle: int | None = None
            try:
                handle = lgpio.gpiochip_open(chip)
                lgpio.gpio_claim_input(
                    handle,
                    self._config.pin,
                )
                self._lgpio_handle = handle
                return
            except (lgpio.error, OSError):
                if handle is not None:
                    lgpio.gpiochip_close(handle)

        raise RuntimeError(f"{ERR_GPIO_OPEN_FAILED} (pin={self._config.pin})")

    def _open_adc(self, channel: int) -> None:
        """Opens the analog input via fusion_hat.adc or falls back to SMBus."""
        try:
            from fusion_hat.adc import ADC

            self._adc = ADC(channel)
            return
        except ImportError:
            from smbus2 import SMBus

            self._bus = SMBus(
                self._config.i2c_bus,
                force=True,
            )

    def is_black(self) -> bool:
        """Samples DO and optional AO, updating the consecutive-read debounce filter."""
        ao_mv: int | None = None
        if self._config.black_mv is not None:
            counts = self.analog_counts()
            if counts is not None:
                ao_mv = adc_counts_to_mv(counts)

        raw_black = is_raw_black(
            self.raw_value(),
            black_value=self._config.black_value,
            ao_mv=ao_mv,
            black_mv=self._config.black_mv,
        )
        return self._confirm.update(raw_black)

    def raw_value(self) -> int:
        """Returns the current digital DO level (0 or 1)."""
        if self._lgpio_mod is not None and self._lgpio_handle is not None:
            return int(
                self._lgpio_mod.gpio_read(
                    self._lgpio_handle,
                    self._config.pin,
                )
            )

        return int(self._dev.value)

    def analog_counts(self) -> int | None:
        """Reads 12-bit ADC counts from Fusion HAT+ or returns None on error."""
        if self._config.adc_channel is None:
            return None

        if self._adc is not None:
            try:
                return int(self._adc.read())
            except OSError as exc:
                LOGGER.warning(
                    "Failed to read Fusion HAT+ ADC channel %d: %s",
                    self._config.adc_channel,
                    exc,
                )
                return None

        if self._bus is None:
            return None

        register = ADC_REG_BASE + ADC_REG_STRIDE * int(self._config.adc_channel)
        try:
            word = self._bus.read_word_data(
                self._config.i2c_address,
                register,
            )
        except OSError as exc:
            LOGGER.warning(
                "Failed to read line sensor ADC channel %d: %s",
                self._config.adc_channel,
                exc,
            )
            return None

        return word_to_adc_counts(word)

    def close(self) -> None:
        """Closes GPIO and optional I2C/ADC handles."""
        if self._lgpio_mod is not None and self._lgpio_handle is not None:
            self._lgpio_mod.gpiochip_close(self._lgpio_handle)
            self._lgpio_handle = None
        if self._dev is not None:
            self._dev.close()
        if self._bus is not None:
            self._bus.close()
