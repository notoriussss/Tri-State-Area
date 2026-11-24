"""Unit tests for pure robot web status parsing and link evaluation."""

from typing import Any

import pytest
from overhead_core.constants import (
    ROBOT_LINK_MAX_AGE_S,
    ROBOT_WEB_KEY_AUTO_ENABLED,
    ROBOT_WEB_KEY_CAMERA_AGE_TEXT,
    ROBOT_WEB_KEY_FSM_STATE,
    ROBOT_WEB_KEY_HAL_MODE,
    ROBOT_WEB_KEY_LINE_DETECTED,
    ROBOT_WEB_KEY_POSE_TEXT,
    ROBOT_WEB_KEY_ROBOT_NAME,
    ROBOT_WEB_KEY_TARGET_CUBE_ID,
    ROBOT_WEB_NO_AGE_TEXT,
    ROBOT_WEB_NO_CAMERA_TEXT,
    ROBOT_WEB_REQUIRED_KEYS,
    RobotLinkProblem,
    RobotName,
)
from overhead_core.robot_link import (
    RobotLinkReport,
    RobotWebStatus,
    evaluate_robot_link,
    format_robot_link_report,
    parse_camera_age_s,
    parse_robot_web_status,
)


def _sample_payload(**overrides: Any) -> dict[str, Any]:
    """Builds a valid robot /api/status JSON payload with optional field overrides."""
    payload: dict[str, Any] = {
        ROBOT_WEB_KEY_ROBOT_NAME: RobotName.PHINEAS.value,
        ROBOT_WEB_KEY_AUTO_ENABLED: True,
        ROBOT_WEB_KEY_HAL_MODE: "run",
        ROBOT_WEB_KEY_FSM_STATE: "GO_TO_APPROACH",
        ROBOT_WEB_KEY_LINE_DETECTED: False,
        ROBOT_WEB_KEY_POSE_TEXT: "(0.42, 0.31, 0.15)",
        ROBOT_WEB_KEY_CAMERA_AGE_TEXT: "0.12 s",
        ROBOT_WEB_KEY_TARGET_CUBE_ID: 7,
    }
    payload.update(overrides)
    return payload


def test_should_parse_robot_web_status_when_all_fields_are_valid() -> None:
    payload = _sample_payload()

    status = RobotWebStatus.from_dict(payload)

    assert status == RobotWebStatus(
        robot_name=RobotName.PHINEAS.value,
        auto_enabled=True,
        hal_mode="run",
        fsm_state="GO_TO_APPROACH",
        line_detected=False,
        pose_text="(0.42, 0.31, 0.15)",
        camera_age_text="0.12 s",
        target_cube_id=7,
    )


def test_should_parse_optional_null_target_cube_id_when_none() -> None:
    payload = _sample_payload(**{ROBOT_WEB_KEY_TARGET_CUBE_ID: None})

    status = parse_robot_web_status(payload)

    assert status.target_cube_id is None


@pytest.mark.parametrize(
    "missing_key",
    ROBOT_WEB_REQUIRED_KEYS,
)
def test_should_raise_clear_value_error_when_required_key_is_missing(
    missing_key: str,
) -> None:
    payload = _sample_payload()
    del payload[missing_key]

    with pytest.raises(
        ValueError,
        match=missing_key,
    ):
        RobotWebStatus.from_dict(payload)


def test_should_raise_value_error_when_payload_is_not_a_mapping() -> None:
    with pytest.raises(
        ValueError,
        match="mapping",
    ):
        RobotWebStatus.from_dict(["not", "a", "dict"])  # type: ignore[arg-type]


@pytest.mark.parametrize(
    (
        "field_name",
        "bad_value",
    ),
    [
        (ROBOT_WEB_KEY_ROBOT_NAME, 123),
        (ROBOT_WEB_KEY_AUTO_ENABLED, "true"),
        (ROBOT_WEB_KEY_HAL_MODE, None),
        (ROBOT_WEB_KEY_FSM_STATE, 42),
        (ROBOT_WEB_KEY_LINE_DETECTED, 1),
        (ROBOT_WEB_KEY_POSE_TEXT, 0.5),
        (ROBOT_WEB_KEY_CAMERA_AGE_TEXT, None),
        (ROBOT_WEB_KEY_TARGET_CUBE_ID, True),
        (ROBOT_WEB_KEY_TARGET_CUBE_ID, "7"),
    ],
)
def test_should_raise_value_error_when_field_has_invalid_type(
    field_name: str,
    bad_value: Any,
) -> None:
    payload = _sample_payload(**{field_name: bad_value})

    with pytest.raises(
        ValueError,
        match=field_name,
    ):
        RobotWebStatus.from_dict(payload)


@pytest.mark.parametrize(
    "age_text",
    [
        ROBOT_WEB_NO_AGE_TEXT,
        " none ",
    ],
)
def test_should_return_none_when_camera_age_text_is_none(age_text: str) -> None:
    parsed = parse_camera_age_s(age_text)

    assert parsed is None


@pytest.mark.parametrize(
    (
        "age_text",
        "expected_s",
    ),
    [
        ("0.12 s", 0.12),
        ("1.5s", 1.5),
        ("0.0", 0.0),
    ],
)
def test_should_parse_camera_age_seconds_when_text_is_numeric(
    age_text: str,
    expected_s: float,
) -> None:
    parsed = parse_camera_age_s(age_text)

    assert parsed == pytest.approx(expected_s)


@pytest.mark.parametrize(
    "bad_age_text",
    [
        "",
        "s",
        "abc s",
        "-0.2 s",
        "nan s",
        "inf s",
    ],
)
def test_should_raise_value_error_when_camera_age_text_is_invalid(
    bad_age_text: str,
) -> None:
    with pytest.raises(ValueError):
        parse_camera_age_s(bad_age_text)


def test_should_report_ok_when_all_links_are_healthy() -> None:
    web_status = RobotWebStatus.from_dict(_sample_payload())

    report = evaluate_robot_link(
        web_status,
        0.5,
        ROBOT_LINK_MAX_AGE_S,
        is_robot_located=True,
    )

    assert report == RobotLinkReport(
        problems=(),
        is_ok=True,
    )


def test_should_report_web_unreachable_when_web_status_is_none() -> None:
    report = evaluate_robot_link(
        None,
        0.2,
        0.2,
        is_robot_located=True,
    )

    assert report.is_ok is False
    assert report.problems == (RobotLinkProblem.WEB_UNREACHABLE,)


@pytest.mark.parametrize(
    (
        "plan_age_s",
        "status_age_s",
        "expected_problems",
    ),
    [
        (
            None,
            0.3,
            (RobotLinkProblem.NO_PLAN_TOPIC,),
        ),
        (
            ROBOT_LINK_MAX_AGE_S + 0.01,
            0.3,
            (RobotLinkProblem.NO_PLAN_TOPIC,),
        ),
        (
            0.3,
            None,
            (RobotLinkProblem.NO_STATUS_TOPIC,),
        ),
        (
            0.3,
            ROBOT_LINK_MAX_AGE_S + 0.01,
            (RobotLinkProblem.NO_STATUS_TOPIC,),
        ),
        (
            None,
            None,
            (
                RobotLinkProblem.NO_PLAN_TOPIC,
                RobotLinkProblem.NO_STATUS_TOPIC,
            ),
        ),
    ],
)
def test_should_report_missing_or_stale_topics_when_ages_missing_or_over_limit(
    plan_age_s: float | None,
    status_age_s: float | None,
    expected_problems: tuple[RobotLinkProblem, ...],
) -> None:
    web_status = RobotWebStatus.from_dict(_sample_payload())

    report = evaluate_robot_link(
        web_status,
        plan_age_s,
        status_age_s,
        is_robot_located=True,
    )

    assert report.is_ok is False
    assert report.problems == expected_problems


@pytest.mark.parametrize(
    "overrides",
    [
        {ROBOT_WEB_KEY_POSE_TEXT: ROBOT_WEB_NO_CAMERA_TEXT},
        {ROBOT_WEB_KEY_CAMERA_AGE_TEXT: ROBOT_WEB_NO_AGE_TEXT},
        {
            ROBOT_WEB_KEY_POSE_TEXT: ROBOT_WEB_NO_CAMERA_TEXT,
            ROBOT_WEB_KEY_CAMERA_AGE_TEXT: ROBOT_WEB_NO_AGE_TEXT,
        },
    ],
)
def test_should_report_robot_without_world_when_camera_missing_in_web_status(
    overrides: dict[str, Any],
) -> None:
    web_status = RobotWebStatus.from_dict(_sample_payload(**overrides))

    report = evaluate_robot_link(
        web_status,
        0.1,
        0.1,
        is_robot_located=True,
    )

    assert report.is_ok is False
    assert report.problems == (RobotLinkProblem.ROBOT_WITHOUT_WORLD,)


def test_should_report_robot_not_located_when_not_in_world_state() -> None:
    web_status = RobotWebStatus.from_dict(_sample_payload())

    report = evaluate_robot_link(
        web_status,
        0.1,
        0.1,
        is_robot_located=False,
    )

    assert report.is_ok is False
    assert report.problems == (RobotLinkProblem.ROBOT_NOT_LOCATED,)


def test_should_accumulate_multiple_problems_when_everything_is_down() -> None:
    report = evaluate_robot_link(
        None,
        None,
        ROBOT_LINK_MAX_AGE_S + 1.0,
        is_robot_located=False,
    )

    assert report.is_ok is False
    assert report.problems == (
        RobotLinkProblem.WEB_UNREACHABLE,
        RobotLinkProblem.NO_PLAN_TOPIC,
        RobotLinkProblem.NO_STATUS_TOPIC,
        RobotLinkProblem.ROBOT_NOT_LOCATED,
    )


def test_should_format_ok_and_problem_lines_for_robot_report() -> None:
    ok_line = format_robot_link_report(
        RobotName.PHINEAS.value,
        RobotLinkReport(
            problems=(),
            is_ok=True,
        ),
    )
    bad_line = format_robot_link_report(
        RobotName.FERB.value,
        RobotLinkReport(
            problems=(
                RobotLinkProblem.WEB_UNREACHABLE,
                RobotLinkProblem.ROBOT_NOT_LOCATED,
            ),
            is_ok=False,
        ),
    )

    assert ok_line == "phineas: OK"
    assert bad_line == "ferb: WEB_UNREACHABLE, ROBOT_NOT_LOCATED"
