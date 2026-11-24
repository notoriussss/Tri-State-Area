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

from overhead_core.aruco_robots import ArucoRobotConfig
from overhead_core.constants import (
    CAPTURE_RETRY_SLEEP_S,
    CAPTURE_TIMESTAMP_FORMAT,
    CAPTURES_DIR_NAME,
    DEFAULT_ARUCO_DICTIONARY,
    DEFAULT_ARUCO_IDS_ARG,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_CAPTURE_HEIGHT_PX,
    DEFAULT_CAPTURE_WIDTH_PX,
    DEFAULT_ROBOT_NAMES,
    DEFAULT_WEB_HOST,
    DEFAULT_WEB_PORT,
    FPS_SMOOTHING_ALPHA,
    JPEG_EXTENSION,
    THREAD_JOIN_TIMEOUT_S,
    WINDOWS_PLATFORM,
    RobotDetector,
)
from overhead_core.viewer_pipeline import (
    ViewerPipelineConfig,
    calibrate_frame,
    process_frame,
)
from overhead_core.viewer_safety import (
    ViewerSafety,
    build_robot_statuses,
)
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
        *,
        pipeline_config: ViewerPipelineConfig | None = None,
    ) -> None:
        """Initializes empty camera viewer state.

        Args:
            captures_dir: Directory where raw snapshot JPEGs are written.
            pipeline_config: Optional vision pipeline configuration for frame processing.
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
        self._pipeline_config = (
            pipeline_config if pipeline_config is not None else ViewerPipelineConfig()
        )
        self._safety = ViewerSafety(
            arena_width_m=self._pipeline_config.arena_width_m,
            arena_height_m=self._pipeline_config.arena_height_m,
        )

    def get_snapshot(self) -> ViewerSnapshot | None:
        """Returns the most recent processed ViewerSnapshot."""
        with self._lock:
            return self._latest_snapshot

    def _ensure_initial_calibration_locked(self, frame_bgr: np.ndarray) -> None:
        """Attempts arena calibration on incoming frames until valid corners are found."""
        if self._homography is not None:
            return

        self._homography, self._corners_px = calibrate_frame(frame_bgr)
        if self._homography is not None:
            self._has_attempted_initial_cal = True
            LOGGER.info("Initial arena calibration succeeded.")
            return

        if not self._has_attempted_initial_cal:
            self._has_attempted_initial_cal = True
            LOGGER.warning("Initial arena calibration failed; retrying.")

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
            config=self._pipeline_config,
            safety=self._safety,
            now_s=now_s,
        )
        raw_copy = frame_bgr.copy()
        robot_statuses = (
            build_robot_statuses(output.safety.safety) if output.safety is not None else ()
        )
        robot_count = (
            len(output.safety.robot_poses) if output.safety is not None else len(output.robots)
        )
        snapshot = ViewerSnapshot(
            views=output.views,
            raw_frame=raw_copy,
            fps=fps,
            cube_count=len(output.cubes),
            robot_count=robot_count,
            is_calibrated=output.is_calibrated,
            robots=robot_statuses,
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
    parser.add_argument(
        "--robot-detector",
        type=str,
        choices=[detector.value for detector in RobotDetector],
        default=RobotDetector.TRIANGLE.value,
        help="Robot marker detection backend.",
    )
    parser.add_argument(
        "--aruco-dictionary",
        type=str,
        default=DEFAULT_ARUCO_DICTIONARY,
        help="OpenCV ArUco dictionary name used with the aruco detector.",
    )
    parser.add_argument(
        "--aruco-ids",
        type=str,
        default=DEFAULT_ARUCO_IDS_ARG,
        help="Comma-separated ArUco marker IDs matching robots in order.",
    )
    return parser


def _parse_aruco_ids(raw_ids: str) -> list[int]:
    """Parses a comma-separated string of ArUco marker IDs into integers."""
    tokens = [token.strip() for token in raw_ids.split(",") if token.strip()]
    return [int(token) for token in tokens]


def build_pipeline_config(args: argparse.Namespace) -> ViewerPipelineConfig:
    """Builds a ViewerPipelineConfig from parsed web viewer CLI arguments."""
    detector = RobotDetector(args.robot_detector)
    aruco_ids = _parse_aruco_ids(args.aruco_ids)
    marker_ids = dict(
        zip(
            DEFAULT_ROBOT_NAMES,
            aruco_ids,
            strict=False,
        )
    )
    aruco_config = ArucoRobotConfig(
        dictionary_name=args.aruco_dictionary,
        marker_ids=marker_ids,
    )
    return ViewerPipelineConfig(
        robot_detector=detector,
        aruco=aruco_config,
    )


def main(argv: list[str] | None = None) -> int:
    """Entry point for running the standalone overhead camera web viewer."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    args = build_arg_parser().parse_args(argv)
    pipeline_config = build_pipeline_config(args)

    cap = open_camera(
        args.camera_index,
        width_px=args.width,
        height_px=args.height,
    )
    state = CameraViewerState(pipeline_config=pipeline_config)
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
