"""Standalone ThreadingHTTPServer serving live overhead camera views."""

import json
import logging
import threading
from collections.abc import Callable
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from overhead_core.constants import (
    CACHE_CONTROL_NO_CACHE,
    CONTENT_TYPE_HTML,
    CONTENT_TYPE_JPEG,
    CONTENT_TYPE_JSON,
    CONTENT_TYPE_MJPEG,
    DEFAULT_WEB_HOST,
    DEFAULT_WEB_PORT,
    HTTP_PATH_INDEX,
    HTTP_PATH_RECALIBRATE,
    HTTP_PATH_SNAPSHOT,
    HTTP_PATH_STATUS,
    HTTP_PATH_STREAM_PREFIX,
    HTTP_PATH_STREAM_SUFFIX,
    HTTP_QUERY_VIEW_KEY,
    JPEG_EXTENSION,
    MJPEG_BOUNDARY,
    NO_OBSTACLE_KIND,
    STATIC_DIR_NAME,
    STATUS_FLOAT_PRECISION,
    VIEWER_HTML_FILENAME,
    VIEWER_JPEG_QUALITY,
    VIEWER_STREAM_FPS_HZ,
    ViewerView,
)
from overhead_core.types import RobotSafetyStatus

LOGGER = logging.getLogger(__name__)


def _load_viewer_html() -> bytes:
    """Loads the static viewer HTML page as UTF-8 bytes."""
    html_path = Path(__file__).parent / STATIC_DIR_NAME / VIEWER_HTML_FILENAME
    return html_path.read_bytes()


@dataclass(frozen=True)
class ViewerSnapshot:
    """Latest rendered views and telemetry passed to the web viewer server."""

    views: dict[str, np.ndarray]
    raw_frame: np.ndarray | None = None
    fps: float = 0.0
    cube_count: int = 0
    robot_count: int = 0
    is_calibrated: bool = False
    robots: tuple[RobotSafetyStatus, ...] = ()


FrameProvider = Callable[[], ViewerSnapshot | dict[str, np.ndarray] | None]


def _normalize_snapshot(
    result: ViewerSnapshot | dict[str, np.ndarray] | None,
) -> ViewerSnapshot | None:
    """Normalizes a frame provider return value into ViewerSnapshot or None."""
    if result is None:
        return None
    if isinstance(result, ViewerSnapshot):
        return result
    return ViewerSnapshot(views=result)


def _encode_jpeg(
    image_bgr: np.ndarray,
    jpeg_quality: int,
) -> bytes | None:
    """Encodes a BGR NumPy image into JPEG bytes."""
    ok, buffer = cv2.imencode(
        JPEG_EXTENSION,
        image_bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
    )
    if not ok:
        return None
    return buffer.tobytes()


def _parse_valid_view(view_name: str) -> str | None:
    """Validates a requested view name against ViewerView members."""
    try:
        return ViewerView(view_name).value
    except ValueError:
        return None


def _extract_stream_view(path: str) -> str | None:
    """Extracts and validates the view name from a stream URL path."""
    has_stream_affixes = path.startswith(HTTP_PATH_STREAM_PREFIX) and path.endswith(
        HTTP_PATH_STREAM_SUFFIX
    )
    if not has_stream_affixes:
        return None
    start = len(HTTP_PATH_STREAM_PREFIX)
    end = len(path) - len(HTTP_PATH_STREAM_SUFFIX)
    return _parse_valid_view(path[start:end])


class OverheadRequestHandler(BaseHTTPRequestHandler):
    """HTTP request handler serving HTML UI, MJPEG streams, snapshots, and recalibration."""

    server: "OverheadWebServer"

    def log_message(
        self,
        fmt: str,
        *args: object,
    ) -> None:
        """Routes BaseHTTPRequestHandler request logs through Python logging."""
        LOGGER.debug(
            fmt,
            *args,
        )

    def do_GET(self) -> None:  # noqa: N802
        """Handles GET requests for index HTML, status JSON, snapshots, and MJPEG streams."""
        parsed = urlparse(self.path)
        path = parsed.path

        if path == HTTP_PATH_INDEX:
            self._handle_index()
            return
        if path == HTTP_PATH_STATUS:
            self._handle_status()
            return
        if path == HTTP_PATH_SNAPSHOT:
            self._handle_snapshot(parsed.query)
            return

        stream_view = _extract_stream_view(path)
        if stream_view is not None:
            self._handle_stream(stream_view)
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        """Handles POST requests for recalibration."""
        parsed = urlparse(self.path)
        if parsed.path == HTTP_PATH_RECALIBRATE:
            self._handle_recalibrate()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def _send_bytes(
        self,
        status: HTTPStatus,
        content_type: str,
        payload: bytes,
    ) -> None:
        """Writes an HTTP response with status, content type, length, and body."""
        self.send_response(status)
        self.send_header(
            "Content-Type",
            content_type,
        )
        self.send_header(
            "Content-Length",
            str(len(payload)),
        )
        self.send_header(
            "Cache-Control",
            CACHE_CONTROL_NO_CACHE,
        )
        self.end_headers()
        self.wfile.write(payload)

    def _handle_index(self) -> None:
        """Serves the single-page HTML viewer UI."""
        self._send_bytes(
            HTTPStatus.OK,
            CONTENT_TYPE_HTML,
            self.server.index_html_bytes,
        )

    def _handle_status(self) -> None:
        """Serves current FPS, cube count, per-robot safety, and calibration state as JSON."""
        snap = _normalize_snapshot(self.server.frame_provider())
        if snap is None:
            payload = {
                "fps": 0.0,
                "cubes": 0,
                "robots": [],
                "calibrated": False,
            }
        else:
            robots_payload = [
                {
                    "name": robot.name,
                    "level": robot.level.value,
                    "ttc_s": round(
                        robot.ttc_s,
                        STATUS_FLOAT_PRECISION,
                    ),
                    "nearest_kind": (
                        robot.nearest_kind.value
                        if robot.nearest_kind is not None
                        else NO_OBSTACLE_KIND
                    ),
                    "nearest_dist_m": round(
                        robot.nearest_dist_m,
                        STATUS_FLOAT_PRECISION,
                    ),
                }
                for robot in snap.robots
            ]
            payload = {
                "fps": round(
                    snap.fps,
                    1,
                ),
                "cubes": snap.cube_count,
                "robots": robots_payload,
                "calibrated": snap.is_calibrated,
            }
        self._send_bytes(
            HTTPStatus.OK,
            CONTENT_TYPE_JSON,
            json.dumps(payload).encode("utf-8"),
        )

    def _handle_snapshot(self, query: str) -> None:
        """Serves a single JPEG frame for the requested view and invokes snapshot hook."""
        params = parse_qs(query)
        raw_view = params.get(
            HTTP_QUERY_VIEW_KEY,
            [ViewerView.RAW.value],
        )[0]
        view_name = _parse_valid_view(raw_view)
        if view_name is None:
            self.send_error(HTTPStatus.BAD_REQUEST)
            return

        snap = _normalize_snapshot(self.server.frame_provider())
        if snap is None or view_name not in snap.views:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE)
            return

        if self.server.on_snapshot is not None:
            try:
                self.server.on_snapshot(snap)
            except OSError as exc:
                LOGGER.warning(
                    "Snapshot callback failed: %s",
                    exc,
                )

        jpeg_bytes = _encode_jpeg(
            snap.views[view_name],
            self.server.jpeg_quality,
        )
        if jpeg_bytes is None:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR)
            return

        self._send_bytes(
            HTTPStatus.OK,
            CONTENT_TYPE_JPEG,
            jpeg_bytes,
        )

    def _handle_recalibrate(self) -> None:
        """Triggers the recalibration callback and returns calibration status as JSON."""
        calibrated = False
        if self.server.on_recalibrate is not None:
            calibrated = bool(self.server.on_recalibrate())

        body = json.dumps(
            {
                "ok": True,
                "calibrated": calibrated,
            }
        ).encode("utf-8")
        self._send_bytes(
            HTTPStatus.OK,
            CONTENT_TYPE_JSON,
            body,
        )

    def _write_mjpeg_frame(self, view_name: str) -> bool:
        """Encodes and writes one multipart MJPEG frame; returns False on disconnect."""
        snap = _normalize_snapshot(self.server.frame_provider())
        if snap is None or view_name not in snap.views:
            return True

        jpeg_bytes = _encode_jpeg(
            snap.views[view_name],
            self.server.jpeg_quality,
        )
        if jpeg_bytes is None:
            return True

        header = (
            f"--{MJPEG_BOUNDARY}\r\n"
            f"Content-Type: {CONTENT_TYPE_JPEG}\r\n"
            f"Content-Length: {len(jpeg_bytes)}\r\n\r\n"
        ).encode("ascii")
        try:
            self.wfile.write(header + jpeg_bytes + b"\r\n")
            self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return False
        return True

    def _handle_stream(self, view_name: str) -> None:
        """Streams multipart MJPEG frames for the selected view until disconnect or stop."""
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            CONTENT_TYPE_MJPEG,
        )
        self.send_header(
            "Cache-Control",
            CACHE_CONTROL_NO_CACHE,
        )
        self.end_headers()

        interval_s = 1.0 / max(
            1.0,
            self.server.stream_fps_hz,
        )
        while not self.server.stop_event.is_set():
            if not self._write_mjpeg_frame(view_name):
                return
            self.server.stop_event.wait(interval_s)


class OverheadWebServer(ThreadingHTTPServer):
    """Multi-threaded HTTP server exposing overhead camera streams and controls."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        host: str = DEFAULT_WEB_HOST,
        port: int = DEFAULT_WEB_PORT,
        *,
        frame_provider: FrameProvider,
        on_recalibrate: Callable[[], bool] | None = None,
        on_snapshot: Callable[[ViewerSnapshot], Path | None] | None = None,
        jpeg_quality: int = VIEWER_JPEG_QUALITY,
        stream_fps_hz: float = VIEWER_STREAM_FPS_HZ,
    ) -> None:
        """Initializes the web viewer server bound to host and port.

        Args:
            host: Network interface address to bind.
            port: TCP port to listen on (0 selects an ephemeral port).
            frame_provider: Callable returning the latest ViewerSnapshot or view dict.
            on_recalibrate: Optional callback invoked on POST /recalibrate.
            on_snapshot: Optional callback invoked when GET /snapshot.jpg is requested.
            jpeg_quality: JPEG encoding quality (0-100).
            stream_fps_hz: Maximum MJPEG stream rate in frames per second.
        """
        self.frame_provider = frame_provider
        self.on_recalibrate = on_recalibrate
        self.on_snapshot = on_snapshot
        self.jpeg_quality = jpeg_quality
        self.stream_fps_hz = stream_fps_hz
        self.index_html_bytes = _load_viewer_html()
        self.stop_event = threading.Event()
        super().__init__(
            (host, port),
            OverheadRequestHandler,
        )

    def stop(self) -> None:
        """Signals active MJPEG stream loops to exit and shuts down the HTTP server."""
        self.stop_event.set()
        self.shutdown()
        self.server_close()


def create_web_server(
    host: str = DEFAULT_WEB_HOST,
    port: int = DEFAULT_WEB_PORT,
    *,
    frame_provider: FrameProvider,
    on_recalibrate: Callable[[], bool] | None = None,
    on_snapshot: Callable[[ViewerSnapshot], Path | None] | None = None,
    jpeg_quality: int = VIEWER_JPEG_QUALITY,
    stream_fps_hz: float = VIEWER_STREAM_FPS_HZ,
) -> OverheadWebServer:
    """Creates an OverheadWebServer instance with the given provider and callbacks.

    Args:
        host: Network interface address to bind.
        port: TCP port to listen on.
        frame_provider: Callable returning the latest ViewerSnapshot or view dict.
        on_recalibrate: Optional callback invoked on POST /recalibrate.
        on_snapshot: Optional callback invoked when GET /snapshot.jpg is requested.
        jpeg_quality: JPEG encoding quality (0-100).
        stream_fps_hz: Maximum MJPEG stream rate in frames per second.

    Returns:
        Configured OverheadWebServer ready to serve requests.
    """
    return OverheadWebServer(
        host,
        port,
        frame_provider=frame_provider,
        on_recalibrate=on_recalibrate,
        on_snapshot=on_snapshot,
        jpeg_quality=jpeg_quality,
        stream_fps_hz=stream_fps_hz,
    )
