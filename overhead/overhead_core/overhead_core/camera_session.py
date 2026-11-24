"""Pure camera frame-source lifecycle, retry, timeout, and calibration session."""

from dataclasses import dataclass

import numpy as np

from overhead_core.arena_calibration import calibrate_from_frame
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CALIBRATION_RETRY_S,
    CAMERA_RETRY_S,
    LOG_CALIBRATION_SUCCESS,
    LOG_CAMERA_OPEN_RETRY,
    LOG_CAMERA_OPENED,
    NO_FRAME_TIMEOUT_S,
)
from overhead_core.frame_source import FrameSource
from overhead_core.state_machine import OverheadEvent


@dataclass(frozen=True)
class CameraOutcome:
    """Single ordered log message and state-machine event emitted by a camera step."""

    info_message: str | None = None
    warning_message: str | None = None
    event: OverheadEvent | None = None


@dataclass(frozen=True)
class CameraStepResult:
    """Result of executing one camera lifecycle or calibration step."""

    frame: np.ndarray | None = None
    outcomes: tuple[CameraOutcome, ...] = ()


class CameraSession:
    """Manages frame-source opening, retry timers, timeouts, and arena calibration."""

    def __init__(
        self,
        frame_source: FrameSource,
        *,
        arena_width_m: float = ARENA_WIDTH_M,
        arena_height_m: float = ARENA_HEIGHT_M,
        calibration_retry_s: float = CALIBRATION_RETRY_S,
    ) -> None:
        """Initializes camera session state around an abstract FrameSource.

        Args:
            frame_source: Live camera or still-image frame provider.
            arena_width_m: Physical arena width along world +x in meters.
            arena_height_m: Physical arena height along world +y in meters.
            calibration_retry_s: Interval in seconds between calibration retries.
        """
        self._frame_source = frame_source
        self._arena_width_m = arena_width_m
        self._arena_height_m = arena_height_m
        self._calibration_retry_s = calibration_retry_s

        self._homography: np.ndarray | None = None
        self._latest_frame: np.ndarray | None = None
        self._last_frame_s: float | None = None
        self._last_camera_retry_s: float = -CAMERA_RETRY_S
        self._last_calibration_try_s: float = -calibration_retry_s

    @property
    def homography(self) -> np.ndarray | None:
        """Returns the latest calibrated 3x3 image-to-world homography, if any."""
        return self._homography

    @property
    def latest_frame(self) -> np.ndarray | None:
        """Returns the most recently captured BGR frame, if any."""
        return self._latest_frame

    def startup(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Checks initial frame-source readiness and attempts initial calibration."""
        if not self._frame_source.is_open():
            self._last_camera_retry_s = now_s
            return CameraStepResult(
                outcomes=(CameraOutcome(warning_message=LOG_CAMERA_OPEN_RETRY),),
            )

        return self._on_opened(now_s)

    def _on_opened(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Records camera open event and immediately runs a calibration attempt."""
        open_outcome = CameraOutcome(
            info_message=LOG_CAMERA_OPENED,
            event=OverheadEvent.CAMERA_OPENED,
        )
        self._last_calibration_try_s = now_s
        _, cal_step = self._attempt_calibration(now_s)
        return CameraStepResult(
            frame=cal_step.frame,
            outcomes=(
                open_outcome,
                *cal_step.outcomes,
            ),
        )

    def _attempt_calibration(
        self,
        now_s: float,
    ) -> tuple[bool, CameraStepResult]:
        """Reads a frame, validates arena corners, and updates floor homography."""
        frame = self._frame_source.read()
        if frame is not None:
            self._latest_frame = frame

        result = calibrate_from_frame(
            frame,
            arena_width_m=self._arena_width_m,
            arena_height_m=self._arena_height_m,
        )
        if not result.is_success:
            return False, CameraStepResult(
                outcomes=(
                    CameraOutcome(
                        warning_message=result.warning_message,
                        event=OverheadEvent.CORNERS_NOT_FOUND,
                    ),
                ),
            )

        self._homography = result.homography
        self._last_frame_s = now_s
        return True, CameraStepResult(
            frame=frame,
            outcomes=(
                CameraOutcome(
                    info_message=LOG_CALIBRATION_SUCCESS,
                    event=OverheadEvent.CORNERS_FOUND,
                ),
            ),
        )

    def recalibrate(
        self,
        now_s: float,
    ) -> tuple[bool, CameraStepResult]:
        """Clears the current homography and immediately attempts recalibration."""
        self._homography = None
        self._last_calibration_try_s = now_s
        return self._attempt_calibration(now_s)

    def step_starting(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Retries opening the frame source every CAMERA_RETRY_S while STARTING."""
        if now_s - self._last_camera_retry_s < CAMERA_RETRY_S:
            return CameraStepResult()

        self._last_camera_retry_s = now_s
        if not self._frame_source.reopen():
            return CameraStepResult(
                outcomes=(CameraOutcome(warning_message=LOG_CAMERA_OPEN_RETRY),),
            )

        return self._on_opened(now_s)

    def step_calibrating(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Retries arena corner calibration every calibration_retry_s while CALIBRATING."""
        if now_s - self._last_calibration_try_s < self._calibration_retry_s:
            return CameraStepResult()

        self._last_calibration_try_s = now_s
        _, cal_step = self._attempt_calibration(now_s)
        return cal_step

    def step_no_image(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Reads a frame or retries reopening every CAMERA_RETRY_S while NO_IMAGE."""
        frame = self._frame_source.read()
        warning_message: str | None = None
        if frame is None and now_s - self._last_camera_retry_s >= CAMERA_RETRY_S:
            self._last_camera_retry_s = now_s
            if self._frame_source.reopen():
                frame = self._frame_source.read()
            else:
                warning_message = LOG_CAMERA_OPEN_RETRY

        if frame is None:
            if warning_message is None:
                return CameraStepResult()
            return CameraStepResult(
                outcomes=(CameraOutcome(warning_message=warning_message),),
            )

        self._last_frame_s = now_s
        self._latest_frame = frame
        return CameraStepResult(
            frame=frame,
            outcomes=(CameraOutcome(event=OverheadEvent.FRAME_BACK),),
        )

    def read_active_frame(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Reads a frame while active and signals timeout after NO_FRAME_TIMEOUT_S."""
        frame = self._frame_source.read()
        if frame is None:
            if self._last_frame_s is None:
                self._last_frame_s = now_s
            if now_s - self._last_frame_s >= NO_FRAME_TIMEOUT_S:
                self._last_camera_retry_s = now_s
                return CameraStepResult(
                    outcomes=(CameraOutcome(event=OverheadEvent.FRAME_TIMEOUT),),
                )
            return CameraStepResult()

        self._last_frame_s = now_s
        self._latest_frame = frame
        return CameraStepResult(frame=frame)

    def release(self) -> None:
        """Frees the camera when the frame source supports it."""
        release_fn = getattr(
            self._frame_source,
            "release",
            None,
        )
        if callable(release_fn):
            release_fn()
