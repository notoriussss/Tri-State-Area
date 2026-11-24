"""Pure diagnostic helpers for parsing robot web status and evaluating link health."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from overhead_core.constants import (
    ERR_ROBOT_WEB_CAMERA_AGE_FMT,
    ERR_ROBOT_WEB_INVALID_FIELD_FMT,
    ERR_ROBOT_WEB_MISSING_KEY_FMT,
    ERR_ROBOT_WEB_PAYLOAD_TYPE_FMT,
    LOG_ROBOT_LINK_RESULT_FMT,
    ROBOT_LINK_MAX_AGE_S,
    ROBOT_LINK_PROBLEM_SEPARATOR,
    ROBOT_LINK_STATUS_OK,
    ROBOT_WEB_AGE_UNIT_SUFFIX,
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
)


def _require_str(
    payload: Mapping[str, Any],
    key: str,
) -> str:
    """Extracts a string field from a robot web status mapping."""
    value = payload[key]
    if not isinstance(value, str):
        raise ValueError(
            ERR_ROBOT_WEB_INVALID_FIELD_FMT.format(
                key,
                value,
            ),
        )
    return value


def _require_bool(
    payload: Mapping[str, Any],
    key: str,
) -> bool:
    """Extracts a boolean field from a robot web status mapping."""
    value = payload[key]
    if not isinstance(value, bool):
        raise ValueError(
            ERR_ROBOT_WEB_INVALID_FIELD_FMT.format(
                key,
                value,
            ),
        )
    return value


def _require_optional_int(
    payload: Mapping[str, Any],
    key: str,
) -> int | None:
    """Extracts an optional integer field from a robot web status mapping."""
    value = payload[key]
    if value is None:
        return None
    is_valid_int = isinstance(value, int) and not isinstance(value, bool)
    if not is_valid_int:
        raise ValueError(
            ERR_ROBOT_WEB_INVALID_FIELD_FMT.format(
                key,
                value,
            ),
        )
    return int(value)


@dataclass(frozen=True)
class RobotWebStatus:
    """Parsed payload from a robot controller's /api/status HTTP endpoint."""

    robot_name: str
    auto_enabled: bool
    hal_mode: str
    fsm_state: str
    line_detected: bool
    pose_text: str
    camera_age_text: str
    target_cube_id: int | None

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> RobotWebStatus:
        """Parses a JSON object mapping into a validated RobotWebStatus.

        Args:
            payload: Dictionary decoded from the robot controller /api/status JSON.

        Returns:
            Validated immutable RobotWebStatus instance.

        Raises:
            ValueError: If payload is not a mapping, misses a required key, or has invalid types.
        """
        if not isinstance(payload, Mapping):
            raise ValueError(
                ERR_ROBOT_WEB_PAYLOAD_TYPE_FMT.format(type(payload).__name__),
            )

        for key in ROBOT_WEB_REQUIRED_KEYS:
            if key not in payload:
                raise ValueError(
                    ERR_ROBOT_WEB_MISSING_KEY_FMT.format(key),
                )

        return cls(
            robot_name=_require_str(
                payload,
                ROBOT_WEB_KEY_ROBOT_NAME,
            ),
            auto_enabled=_require_bool(
                payload,
                ROBOT_WEB_KEY_AUTO_ENABLED,
            ),
            hal_mode=_require_str(
                payload,
                ROBOT_WEB_KEY_HAL_MODE,
            ),
            fsm_state=_require_str(
                payload,
                ROBOT_WEB_KEY_FSM_STATE,
            ),
            line_detected=_require_bool(
                payload,
                ROBOT_WEB_KEY_LINE_DETECTED,
            ),
            pose_text=_require_str(
                payload,
                ROBOT_WEB_KEY_POSE_TEXT,
            ),
            camera_age_text=_require_str(
                payload,
                ROBOT_WEB_KEY_CAMERA_AGE_TEXT,
            ),
            target_cube_id=_require_optional_int(
                payload,
                ROBOT_WEB_KEY_TARGET_CUBE_ID,
            ),
        )


@dataclass(frozen=True)
class RobotLinkReport:
    """Link health evaluation result for one robot without embedding its name."""

    problems: tuple[RobotLinkProblem, ...]
    is_ok: bool


def parse_robot_web_status(payload: Mapping[str, Any]) -> RobotWebStatus:
    """Parses a JSON object mapping into a validated RobotWebStatus.

    Args:
        payload: Dictionary decoded from the robot controller /api/status JSON.

    Returns:
        Validated immutable RobotWebStatus instance.
    """
    return RobotWebStatus.from_dict(payload)


def parse_camera_age_s(camera_age_text: str) -> float | None:
    """Parses the robot web controller's camera_age_text into seconds.

    Args:
        camera_age_text: Text formatted as "none" or "<seconds> s".

    Returns:
        Non-negative camera age in seconds, or None when camera_age_text is "none".

    Raises:
        ValueError: If camera_age_text is empty, non-numeric, infinite, or negative.
    """
    if not isinstance(camera_age_text, str):
        raise ValueError(
            ERR_ROBOT_WEB_CAMERA_AGE_FMT.format(camera_age_text),
        )

    cleaned = camera_age_text.strip()
    if cleaned.lower() == ROBOT_WEB_NO_AGE_TEXT:
        return None

    if cleaned.lower().endswith(ROBOT_WEB_AGE_UNIT_SUFFIX):
        cleaned = cleaned[:-1].strip()

    if not cleaned:
        raise ValueError(
            ERR_ROBOT_WEB_CAMERA_AGE_FMT.format(camera_age_text),
        )

    try:
        age_s = float(cleaned)
    except ValueError as exc:
        raise ValueError(
            ERR_ROBOT_WEB_CAMERA_AGE_FMT.format(camera_age_text),
        ) from exc

    if not math.isfinite(age_s) or age_s < 0.0:
        raise ValueError(
            ERR_ROBOT_WEB_CAMERA_AGE_FMT.format(camera_age_text),
        )

    return age_s


def _is_topic_missing_or_stale(
    age_s: float | None,
    *,
    max_age_s: float,
) -> bool:
    """Returns True when a topic message is absent or older than max_age_s."""
    if age_s is None:
        return True
    return not math.isfinite(age_s) or age_s < 0.0 or age_s > max_age_s


def _has_world_state_on_robot(web_status: RobotWebStatus) -> bool:
    """Returns True when the robot web status shows active /world/state reception."""
    is_no_camera_pose = web_status.pose_text.strip().lower() == ROBOT_WEB_NO_CAMERA_TEXT
    is_no_camera_age = web_status.camera_age_text.strip().lower() == ROBOT_WEB_NO_AGE_TEXT
    return not (is_no_camera_pose or is_no_camera_age)


def evaluate_robot_link(
    web_status: RobotWebStatus | None,
    plan_age_s: float | None,
    status_age_s: float | None,
    *,
    is_robot_located: bool,
    max_age_s: float = ROBOT_LINK_MAX_AGE_S,
) -> RobotLinkReport:
    """Evaluates end-to-end ROS, web, and vision health for one collector robot.

    Args:
        web_status: Parsed /api/status response, or None when HTTP GET failed.
        plan_age_s: Local monotonic age in seconds of /<robot>/plan, or None if unseen.
        status_age_s: Local monotonic age in seconds of /<robot>/status, or None if unseen.
        is_robot_located: Whether the robot name appears in the latest /world/state.
        max_age_s: Maximum allowed local topic age in seconds before reporting stale.

    Returns:
        RobotLinkReport containing all detected problems and an overall is_ok flag.
    """
    problems: list[RobotLinkProblem] = []
    if web_status is None:
        problems.append(RobotLinkProblem.WEB_UNREACHABLE)

    if _is_topic_missing_or_stale(
        plan_age_s,
        max_age_s=max_age_s,
    ):
        problems.append(RobotLinkProblem.NO_PLAN_TOPIC)

    if _is_topic_missing_or_stale(
        status_age_s,
        max_age_s=max_age_s,
    ):
        problems.append(RobotLinkProblem.NO_STATUS_TOPIC)

    if web_status is not None and not _has_world_state_on_robot(web_status):
        problems.append(RobotLinkProblem.ROBOT_WITHOUT_WORLD)

    if not is_robot_located:
        problems.append(RobotLinkProblem.ROBOT_NOT_LOCATED)

    problems_tuple = tuple(problems)
    return RobotLinkReport(
        problems=problems_tuple,
        is_ok=len(problems_tuple) == 0,
    )


def format_robot_link_report(
    robot_name: str,
    report: RobotLinkReport,
) -> str:
    """Formats a single diagnostic summary line for one robot.

    Args:
        robot_name: Configured robot identifier.
        report: Link evaluation result for that robot.

    Returns:
        Single-line summary containing OK or comma-separated problem codes.
    """
    if report.is_ok:
        return LOG_ROBOT_LINK_RESULT_FMT.format(
            robot_name,
            ROBOT_LINK_STATUS_OK,
        )

    summary = ROBOT_LINK_PROBLEM_SEPARATOR.join(problem.value for problem in report.problems)
    return LOG_ROBOT_LINK_RESULT_FMT.format(
        robot_name,
        summary,
    )
