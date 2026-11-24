"""Unit tests for the bottom TCRT5000 line sensor logic and fake backend."""

from fleet_core.hal_line import (
    BlackConfirm,
    FakeLineSensor,
    LineSensorConfig,
    adc_counts_to_mv,
    is_raw_black,
    word_to_adc_counts,
)


def test_should_swap_bytes_when_decoding_smbus_adc_word() -> None:
    smbus_word = 0x3412

    counts = word_to_adc_counts(smbus_word)

    assert counts == 0x1234


def test_should_convert_adc_counts_to_millivolts_when_reading_12bit_adc() -> None:
    zero_mv = adc_counts_to_mv(0)
    half_mv = adc_counts_to_mv(2048)
    full_mv = adc_counts_to_mv(4095)

    assert zero_mv == 0
    assert half_mv == 1650
    assert full_mv == 3299


def test_should_detect_black_when_do_or_ao_exceeds_threshold() -> None:
    do_black = is_raw_black(
        1,
        black_value=1,
        ao_mv=150,
        black_mv=900,
    )
    ao_black = is_raw_black(
        0,
        black_value=1,
        ao_mv=1100,
        black_mv=900,
    )
    white = is_raw_black(
        0,
        black_value=1,
        ao_mv=200,
        black_mv=900,
    )

    assert do_black is True
    assert ao_black is True
    assert white is False


def test_should_ignore_single_glitch_when_confirm_reads_is_two() -> None:
    confirm = BlackConfirm(confirm_reads=2)

    first = confirm.update(True)
    reset = confirm.update(False)
    again_first = confirm.update(True)
    second = confirm.update(True)

    assert first is False
    assert reset is False
    assert again_first is False
    assert second is True


def test_should_report_black_through_fake_sensor_when_adc_crosses_threshold() -> None:
    sensor = FakeLineSensor(
        LineSensorConfig(
            confirm_reads=2,
            black_mv=900,
        )
    )
    sensor.set_reading(
        do_value=0,
        ao_counts=2048,
    )

    first_read = sensor.is_black()
    second_read = sensor.is_black()

    assert first_read is False
    assert second_read is True
    assert sensor.analog_counts() == 2048
