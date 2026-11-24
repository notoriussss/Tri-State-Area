"""Frame source abstraction for live camera capture, HTTP frames, and static test images."""

import http.client
import logging
import urllib.error
import urllib.request
from pathlib import Path
from typing import Protocol, runtime_checkable

import cv2
import numpy as np

from overhead_core.constants import (
    DEFAULT_CAMERA_INDEX,
    DEFAULT_CAPTURE_HEIGHT_PX,
    DEFAULT_CAPTURE_WIDTH_PX,
    HTTP_ERROR_KEY_PREFIX,
    HTTP_FRAME_DECODE_ERROR_KEY,
    HTTP_FRAME_EMPTY_ERROR_KEY,
    HTTP_FRAME_TIMEOUT_S,
    HTTP_URL_PREFIXES,
    IO_ERROR_KEY_PREFIX,
    LOG_HTTP_FRAME_DECODE_FAILED_FMT,
    LOG_HTTP_FRAME_EMPTY_FMT,
    LOG_HTTP_FRAME_FETCH_FAILED_FMT,
    LOG_HTTP_FRAME_RECOVERED_FMT,
    URL_ERROR_KEY_PREFIX,
)

LOGGER = logging.getLogger(__name__)


@runtime_checkable
class FrameSource(Protocol):
    """Protocol for camera, HTTP, or static image frame providers."""

    def is_open(self) -> bool:
        """Returns whether the frame source is currently ready to produce frames."""
        ...

    def read(self) -> np.ndarray | None:
        """Reads the next BGR frame, or returns None when unavailable."""
        ...

    def reopen(self) -> bool:
        """Reinitializes the underlying frame source and reports open status."""
        ...


class CameraFrameSource:
    """Frame source backed by an OpenCV VideoCapture device."""

    def __init__(
        self,
        camera_index: int = DEFAULT_CAMERA_INDEX,
        *,
        width_px: int = DEFAULT_CAPTURE_WIDTH_PX,
        height_px: int = DEFAULT_CAPTURE_HEIGHT_PX,
    ) -> None:
        """Initializes and opens the camera capture device.

        Args:
            camera_index: OS camera index passed to cv2.VideoCapture.
            width_px: Requested capture width in pixels.
            height_px: Requested capture height in pixels.
        """
        self._camera_index = camera_index
        self._width_px = width_px
        self._height_px = height_px
        self._capture: cv2.VideoCapture | None = None
        self._open()

    def _open(self) -> bool:
        """Opens cv2.VideoCapture and requests the configured resolution."""
        capture = cv2.VideoCapture(self._camera_index)
        capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self._width_px,
        )
        capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self._height_px,
        )
        self._capture = capture
        return bool(capture.isOpened())

    def is_open(self) -> bool:
        """Returns True when the underlying VideoCapture device is open."""
        return self._capture is not None and bool(self._capture.isOpened())

    def read(self) -> np.ndarray | None:
        """Reads one BGR frame from the camera, or None if capture fails."""
        if self._capture is None or not self._capture.isOpened():
            return None

        ok, frame = self._capture.read()
        if not ok or frame is None:
            return None

        return frame

    def reopen(self) -> bool:
        """Releases any existing capture handle and attempts to reopen the camera."""
        self.release()
        return self._open()

    def release(self) -> None:
        """Releases the underlying VideoCapture handle if currently allocated."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None


class StillImageFrameSource:
    """Frame source that loads a still image file once and returns copies."""

    def __init__(
        self,
        path: str | Path,
    ) -> None:
        """Loads the image file from disk using Unicode-safe byte reading.

        Args:
            path: Filesystem path to an image file.
        """
        self._path = Path(path)
        self._frame: np.ndarray | None = self._load_image(self._path)

    @staticmethod
    def _load_image(path: Path) -> np.ndarray | None:
        """Reads and decodes a color image file, returning None on failure."""
        try:
            if not path.is_file():
                return None
            raw_bytes = np.fromfile(
                str(path),
                dtype=np.uint8,
            )
        except OSError:
            return None

        if raw_bytes.size == 0:
            return None

        try:
            return cv2.imdecode(
                raw_bytes,
                cv2.IMREAD_COLOR,
            )
        except cv2.error:
            return None

    def is_open(self) -> bool:
        """Returns True when the image file was decoded successfully."""
        return self._frame is not None

    def read(self) -> np.ndarray | None:
        """Returns a fresh copy of the loaded BGR image, or None if invalid."""
        if self._frame is None:
            return None

        return self._frame.copy()

    def reopen(self) -> bool:
        """Reloads the image file from disk and returns whether it decoded."""
        self._frame = self._load_image(self._path)
        return self.is_open()


def _describe_fetch_error(exc: Exception) -> tuple[str, str]:
    """Returns a stable deduplication key and summary text for an HTTP fetch error."""
    if isinstance(exc, urllib.error.HTTPError):
        return (
            f"{HTTP_ERROR_KEY_PREFIX}:{exc.code}",
            f"HTTP {exc.code} {exc.reason}",
        )
    if isinstance(exc, urllib.error.URLError):
        reason_name = type(exc.reason).__name__
        return (
            f"{URL_ERROR_KEY_PREFIX}:{reason_name}:{exc.reason}",
            str(exc.reason),
        )
    return (
        f"{IO_ERROR_KEY_PREFIX}:{type(exc).__name__}:{exc}",
        str(exc),
    )


class HttpFrameSource:
    """Frame source that fetches single JPEG frames from an HTTP(S) endpoint."""

    def __init__(
        self,
        url: str,
        *,
        timeout_s: float = HTTP_FRAME_TIMEOUT_S,
    ) -> None:
        """Initializes the HTTP frame source and probes initial availability.

        Args:
            url: Full HTTP(S) URL serving a JPEG image on GET.
            timeout_s: Request timeout in seconds.
        """
        self._url = url
        self._timeout_s = timeout_s
        self._is_open = False
        self._is_released = False
        self._last_error_key: str | None = None
        self.reopen()

    def _log_error_once(
        self,
        error_key: str,
        *,
        log_fmt: str,
        detail: str | None = None,
    ) -> None:
        """Logs a warning once per distinct error key until a frame succeeds."""
        if self._last_error_key == error_key:
            return
        self._last_error_key = error_key
        if detail is None:
            LOGGER.warning(
                log_fmt,
                self._url,
            )
            return
        LOGGER.warning(
            log_fmt,
            self._url,
            detail,
        )

    def _decode_payload(self, payload: bytes) -> np.ndarray | None:
        """Decodes raw JPEG bytes into a BGR image and logs recovery after failures."""
        raw_bytes = np.frombuffer(
            payload,
            dtype=np.uint8,
        )
        try:
            frame = cv2.imdecode(
                raw_bytes,
                cv2.IMREAD_COLOR,
            )
        except cv2.error:
            frame = None

        if frame is None or frame.size == 0:
            self._log_error_once(
                HTTP_FRAME_DECODE_ERROR_KEY,
                log_fmt=LOG_HTTP_FRAME_DECODE_FAILED_FMT,
            )
            return None

        if self._last_error_key is not None:
            LOGGER.info(
                LOG_HTTP_FRAME_RECOVERED_FMT,
                self._url,
            )
            self._last_error_key = None

        return frame

    def _fetch_frame(self) -> np.ndarray | None:
        """Fetches and decodes one JPEG frame, returning None without raising."""
        try:
            with urllib.request.urlopen(
                self._url,
                timeout=self._timeout_s,
            ) as response:
                payload = response.read()
        except (
            urllib.error.URLError,
            http.client.HTTPException,
            OSError,
            ValueError,
        ) as exc:
            error_key, detail = _describe_fetch_error(exc)
            self._log_error_once(
                error_key,
                log_fmt=LOG_HTTP_FRAME_FETCH_FAILED_FMT,
                detail=detail,
            )
            return None

        if not payload:
            self._log_error_once(
                HTTP_FRAME_EMPTY_ERROR_KEY,
                log_fmt=LOG_HTTP_FRAME_EMPTY_FMT,
            )
            return None

        return self._decode_payload(payload)

    def is_open(self) -> bool:
        """Returns True when the HTTP endpoint yielded a valid frame on the last check."""
        return self._is_open

    def read(self) -> np.ndarray | None:
        """Fetches one BGR frame over HTTP, or returns None on error or release."""
        if self._is_released:
            return None

        frame = self._fetch_frame()
        self._is_open = frame is not None
        return frame

    def reopen(self) -> bool:
        """Clears released state, probes the HTTP endpoint, and reports open status."""
        self._is_released = False
        frame = self._fetch_frame()
        self._is_open = frame is not None
        return self._is_open

    def release(self) -> None:
        """Marks the HTTP frame source closed until reopen is called."""
        self._is_released = True
        self._is_open = False


def make_frame_source(
    video_source: str,
    *,
    camera_index: int = DEFAULT_CAMERA_INDEX,
    width_px: int = DEFAULT_CAPTURE_WIDTH_PX,
    height_px: int = DEFAULT_CAPTURE_HEIGHT_PX,
) -> FrameSource:
    """Constructs a frame source from an HTTP(S) URL, still image path, or live camera.

    Args:
        video_source: HTTP(S) frame URL, still image file path, or empty string for live camera.
        camera_index: OS camera index used when video_source is empty.
        width_px: Requested camera capture width in pixels.
        height_px: Requested camera capture height in pixels.

    Returns:
        Configured FrameSource instance.
    """
    cleaned_source = video_source.strip()
    if cleaned_source.lower().startswith(HTTP_URL_PREFIXES):
        return HttpFrameSource(cleaned_source)
    if cleaned_source:
        return StillImageFrameSource(video_source)

    return CameraFrameSource(
        camera_index,
        width_px=width_px,
        height_px=height_px,
    )
