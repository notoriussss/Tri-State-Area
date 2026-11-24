"""CLI application capturing from a camera and serving the overhead web viewer."""

import argparse
import logging
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from overhead_core.constants import (
    CAPTURE_RETRY_SLEEP_S,
    CAPTURE_TIMESTAMP_FORMAT,
    CAPTURES_DIR_NAME,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_CAPTURE_HEIGHT_PX,
    DEFAULT_CAPTURE_WIDTH_PX,
    DEFAULT_WEB_HOST,
    DEFAULT_WEB_PORT,
    FPS_SMOOTHING_ALPHA,
    JPEG_EXTENSION,
    THREAD_JOIN_TIMEOUT_S,
    WINDOWS_PLATFORM,
)
from overhead_core.viewer_pipeline import calibrate_frame, process_frame
from overhead_core.web_viewer import ViewerSnapshot, create_web_server

LOGGER = logging.getLogger(__name__)


def default_captures_dir() -> Path:
    """Returns the default captures directory under overhead/overhead_core/captures."""
    return Path(__file__).resolve().parents[1] / CAPTURES_DIR_NAME


def open_camera(
    camera_index: int,
    *,
    width_px: int = DEFAULT_CAPTURE_WIDTH_PX,
    height_px: int = DEFAULT_CAPTURE_HEIGHT_PX,
) -> cv2.VideoCapture:
    """Opens the camera device, requests target resolution, and logs actual settings.

    On Windows, tries DirectShow (CAP_DSHOW) first before falling back to the default backend.

    Args:
        camera_index: OpenCV camera device index.
        width_px: Requested capture frame width in pixels.
        height_px: Requested capture frame height in pixels.

    Returns:
        Opened cv2.VideoCapture handle.

    Raises:
        RuntimeError: If the camera cannot be opened.
    """
    if sys.platform == WINDOWS_PLATFORM:
        cap = cv2.VideoCapture(
            camera_index,
            cv2.CAP_DSHOW,
        )
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(camera_index)
    else:
        cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Unable to open camera index {camera_index}")

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        width_px,
    )
    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        height_px,
    )

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    LOGGER.info(
        "Opened camera %d: requested %dx%d, actual %dx%d",
        camera_index,
        width_px,
        height_px,
        actual_width,
        actual_height,
    )
    return cap


class CameraViewerState:
    """Thread-safe container holding calibration, latest snapshot, and capture saving."""

    def __init__(
        self,
        captures_dir: Path | None = None,
    ) -> None:
        """Initializes empty camera viewer state.

        Args:
            captures_dir: Directory where raw snapshot JPEGs are written.
        """
        self._lock = threading.Lock()
        self._homography: np.ndarray | None = None
        self._corners_px: np.ndarray | None = None
        self._has_attempted_initial_cal = False
        self._latest_raw_frame: np.ndarray | None = None
        self._latest_snapshot: ViewerSnapshot | None = None
        self._last_frame_s: float | None = None
        self._fps: float = 0.0
        self._captures_dir = captures_dir if captures_dir is not None else default_captures_dir()

    def get_snapshot(self) -> ViewerSnapshot | None:
        """Returns the most recent processed ViewerSnapshot."""
        with self._lock:
            return self._latest_snapshot

    def _ensure_initial_calibration_locked(self, frame_bgr: np.ndarray) -> None:
        """Runs one-shot initial calibration on the first received frame."""
        if self._has_attempted_initial_cal:
            return
        self._has_attempted_initial_cal = True
        self._homography, self._corners_px = calibrate_frame(frame_bgr)
        if self._homography is not None:
            LOGGER.info("Initial arena calibration succeeded.")
        else:
            LOGGER.warning("Initial arena calibration failed; arena corners not found.")

    def _update_fps_locked(self, now_s: float) -> float:
        """Updates exponentially smoothed capture FPS using monotonic timestamps."""
        if self._last_frame_s is None:
            self._last_frame_s = now_s
            return self._fps

        dt_s = now_s - self._last_frame_s
        self._last_frame_s = now_s
        if dt_s <= 0.0:
            return self._fps

        inst_fps = 1.0 / dt_s
        if self._fps == 0.0:
            self._fps = inst_fps
        else:
            self._fps = (1.0 - FPS_SMOOTHING_ALPHA) * self._fps + FPS_SMOOTHING_ALPHA * inst_fps
        return self._fps

    def update_from_frame(self, frame_bgr: np.ndarray) -> None:
        """Processes a captured BGR frame and updates the published viewer snapshot."""
        now_s = time.monotonic()
        with self._lock:
            self._ensure_initial_calibration_locked(frame_bgr)
            homography = self._homography
            corners_px = self._corners_px
            fps = self._update_fps_locked(now_s)

        output = process_frame(
            frame_bgr,
            homography,
            corners_px=corners_px,
        )
        raw_copy = frame_bgr.copy()
        snapshot = ViewerSnapshot(
            views=output.views,
            raw_frame=raw_copy,
            fps=fps,
            cube_count=len(output.cubes),
            robot_count=len(output.robots),
            is_calibrated=output.is_calibrated,
        )
        with self._lock:
            self._latest_raw_frame = raw_copy
            self._latest_snapshot = snapshot

    def recalibrate(self) -> bool:
        """Re-runs corner detection on the latest raw frame and updates homography."""
        with self._lock:
            raw_frame = (
                self._latest_raw_frame.copy() if self._latest_raw_frame is not None else None
            )

        if raw_frame is None:
            LOGGER.warning("Recalibration requested before any camera frame arrived.")
            return False

        homography, corners_px = calibrate_frame(raw_frame)
        with self._lock:
            self._homography = homography
            self._corners_px = corners_px
            self._has_attempted_initial_cal = True

        is_calibrated = homography is not None
        if is_calibrated:
            LOGGER.info("Arena recalibrated successfully.")
        else:
            LOGGER.warning("Arena recalibration failed; corners not found.")
        return is_calibrated

    def save_raw_snapshot(self, snapshot: ViewerSnapshot) -> Path | None:
        """Saves the raw frame from snapshot to captures/<timestamp>.jpg."""
        raw_frame = snapshot.raw_frame
        if raw_frame is None:
            return None

        self._captures_dir.mkdir(
            parents=True,
            exist_ok=True,
        )
        timestamp = datetime.now().strftime(CAPTURE_TIMESTAMP_FORMAT)
        target_path = self._captures_dir / f"{timestamp}{JPEG_EXTENSION}"
        # cv2.imwrite cannot open non-ASCII paths on Windows, so encode in memory instead.
        ok, encoded = cv2.imencode(
            JPEG_EXTENSION,
            raw_frame,
        )
        if ok:
            target_path.write_bytes(encoded.tobytes())
        if not ok:
            LOGGER.warning(
                "Failed to write snapshot to %s",
                target_path,
            )
            return None

        LOGGER.info(
            "Saved raw snapshot to %s",
            target_path,
        )
        return target_path


def _capture_loop(
    cap: cv2.VideoCapture,
    state: CameraViewerState,
    stop_event: threading.Event,
) -> None:
    """Continuously reads frames from the camera and updates viewer state."""
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret or frame is None:
            stop_event.wait(CAPTURE_RETRY_SLEEP_S)
            continue
        state.update_from_frame(frame)


def build_arg_parser() -> argparse.ArgumentParser:
    """Builds the CLI argument parser for the standalone overhead web viewer."""
    parser = argparse.ArgumentParser(
        description="Standalone web viewer for the overhead arena camera.",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=DEFAULT_CAMERA_INDEX,
        help="OpenCV camera device index.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=DEFAULT_CAPTURE_WIDTH_PX,
        help="Requested capture width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=DEFAULT_CAPTURE_HEIGHT_PX,
        help="Requested capture height in pixels.",
    )
    parser.add_argument(
        "--host",
        type=str,
        default=DEFAULT_WEB_HOST,
        help="HTTP server bind address.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_WEB_PORT,
        help="HTTP server TCP port.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point for running the standalone overhead camera web viewer."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    args = build_arg_parser().parse_args(argv)

    cap = open_camera(
        args.camera_index,
        width_px=args.width,
        height_px=args.height,
    )
    state = CameraViewerState()
    stop_event = threading.Event()
    capture_thread = threading.Thread(
        target=_capture_loop,
        args=(cap, state, stop_event),
        daemon=True,
    )
    capture_thread.start()

    server = create_web_server(
        args.host,
        args.port,
        frame_provider=state.get_snapshot,
        on_recalibrate=state.recalibrate,
        on_snapshot=state.save_raw_snapshot,
    )
    LOGGER.info(
        "Overhead web viewer serving at http://%s:%d",
        args.host,
        args.port,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        LOGGER.info("Shutting down overhead web viewer.")
    finally:
        stop_event.set()
        server.stop()
        capture_thread.join(timeout=THREAD_JOIN_TIMEOUT_S)
        cap.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
