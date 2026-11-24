"""Unit tests for the pure CameraSession lifecycle and recovery helper."""

import cv2
import numpy as np
from overhead_core.camera_session import CameraSession
from overhead_core.constants import (
    CAMERA_RETRY_S,
    LOG_CALIBRATION_SUCCESS,
    LOG_CAMERA_OPEN_RETRY,
    LOG_CAMERA_OPENED,
    LOG_NO_FRAME_DURING_CALIBRATION,
    NO_FRAME_TIMEOUT_S,
)
from overhead_core.state_machine import OverheadEvent


class _FakeFrameSource:
    def __init__(
        self,
        *,
        opened: bool = True,
        frame: np.ndarray | None = None,
        reopen_result: bool = True,
    ) -> None:
        self.opened = opened
        self.frame = frame
        self.reopen_result = reopen_result
        self.reopen_calls = 0
        self.released = False

    def is_open(self) -> bool:
        return self.opened

    def read(self) -> np.ndarray | None:
        return self.frame

    def reopen(self) -> bool:
        self.reopen_calls += 1
        self.opened = self.reopen_result
        return self.reopen_result

    def release(self) -> None:
        self.released = True


def _make_valid_arena_frame() -> np.ndarray:
    frame = np.full(
        (720, 1280, 3),
        230,
        dtype=np.uint8,
    )
    cv2.rectangle(
        frame,
        (140, 60),
        (1140, 660),
        (0, 0, 0),
        20,
    )
    return frame


def test_should_emit_open_and_calibration_events_when_startup_succeeds() -> None:
    source = _FakeFrameSource(
        opened=True,
        frame=_make_valid_arena_frame(),
    )
    session = CameraSession(source)

    step = session.startup(10.0)

    assert session.homography is not None
    assert session.latest_frame is not None
    assert step.frame is not None
    assert [(o.info_message, o.event) for o in step.outcomes] == [
        (LOG_CAMERA_OPENED, OverheadEvent.CAMERA_OPENED),
        (LOG_CALIBRATION_SUCCESS, OverheadEvent.CORNERS_FOUND),
    ]


def test_should_retry_open_and_enter_calibrating_when_starting_source_recovers() -> None:
    source = _FakeFrameSource(
        opened=False,
        frame=None,
        reopen_result=False,
    )
    session = CameraSession(source)

    startup_step = session.startup(0.0)
    early_step = session.step_starting(0.5)
    source.reopen_result = True
    source.frame = _make_valid_arena_frame()
    recovered_step = session.step_starting(CAMERA_RETRY_S)

    assert startup_step.outcomes[0].warning_message == LOG_CAMERA_OPEN_RETRY
    assert early_step.outcomes == ()
    assert source.reopen_calls == 1
    assert session.homography is not None
    assert [o.event for o in recovered_step.outcomes] == [
        OverheadEvent.CAMERA_OPENED,
        OverheadEvent.CORNERS_FOUND,
    ]


def test_should_respect_calibration_retry_interval_when_calibrating() -> None:
    source = _FakeFrameSource(
        opened=True,
        frame=None,
    )
    session = CameraSession(
        source,
        calibration_retry_s=1.0,
    )

    first = session.startup(1.0)
    early = session.step_calibrating(1.5)
    source.frame = _make_valid_arena_frame()
    retried = session.step_calibrating(2.0)

    assert first.outcomes[1].warning_message == LOG_NO_FRAME_DURING_CALIBRATION
    assert first.outcomes[1].event == OverheadEvent.CORNERS_NOT_FOUND
    assert early.outcomes == ()
    assert retried.outcomes[0].event == OverheadEvent.CORNERS_FOUND
    assert session.homography is not None


def test_should_emit_frame_timeout_then_frame_back_when_frames_drop_and_recover() -> None:
    source = _FakeFrameSource(
        opened=True,
        frame=_make_valid_arena_frame(),
    )
    session = CameraSession(source)
    session.startup(0.0)

    source.frame = None
    before_timeout = session.read_active_frame(NO_FRAME_TIMEOUT_S - 0.1)
    at_timeout = session.read_active_frame(NO_FRAME_TIMEOUT_S)
    source.frame = _make_valid_arena_frame()
    recovered = session.step_no_image(NO_FRAME_TIMEOUT_S + 0.1)

    assert before_timeout.outcomes == ()
    assert [o.event for o in at_timeout.outcomes] == [OverheadEvent.FRAME_TIMEOUT]
    assert recovered.frame is not None
    assert [o.event for o in recovered.outcomes] == [OverheadEvent.FRAME_BACK]


def test_should_clear_homography_and_release_source_when_recalibrated_and_released() -> None:
    source = _FakeFrameSource(
        opened=True,
        frame=_make_valid_arena_frame(),
    )
    session = CameraSession(source)
    session.startup(0.0)

    source.frame = None
    ok, step = session.recalibrate(2.0)
    session.release()

    assert ok is False
    assert session.homography is None
    assert step.outcomes[0].event == OverheadEvent.CORNERS_NOT_FOUND
    assert source.released is True
