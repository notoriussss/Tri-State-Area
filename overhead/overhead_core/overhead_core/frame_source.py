"""Frame source abstraction for live camera capture and static test images."""

from pathlib import Path
from typing import Protocol, runtime_checkable

import cv2
import numpy as np

from overhead_core.constants import (
    DEFAULT_CAMERA_INDEX,
    DEFAULT_CAPTURE_HEIGHT_PX,
    DEFAULT_CAPTURE_WIDTH_PX,
)


@runtime_checkable
class FrameSource(Protocol):
    """Protocol for camera or static image frame providers."""

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


def make_frame_source(
    video_source: str,
    *,
    camera_index: int = DEFAULT_CAMERA_INDEX,
    width_px: int = DEFAULT_CAPTURE_WIDTH_PX,
    height_px: int = DEFAULT_CAPTURE_HEIGHT_PX,
) -> FrameSource:
    """Constructs a still-image source when video_source is non-empty, else a camera source.

    Args:
        video_source: Path to a still image file, or empty string for live camera.
        camera_index: OS camera index used when video_source is empty.
        width_px: Requested camera capture width in pixels.
        height_px: Requested camera capture height in pixels.

    Returns:
        Configured FrameSource instance.
    """
    if video_source.strip():
        return StillImageFrameSource(video_source)

    return CameraFrameSource(
        camera_index,
        width_px=width_px,
        height_px=height_px,
    )
