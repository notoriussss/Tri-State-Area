"""Unit tests for HttpFrameSource and HTTP(S) dispatch in make_frame_source."""

import logging
import threading
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from overhead_core.camera_session import CameraSession
from overhead_core.constants import (
    CAMERA_RETRY_S,
    CONTENT_TYPE_JPEG,
    LOG_CAMERA_OPEN_RETRY,
)
from overhead_core.frame_source import (
    FrameSource,
    HttpFrameSource,
    StillImageFrameSource,
    make_frame_source,
)
from overhead_core.state_machine import OverheadEvent

FIXTURE_PATH: Path = (
    Path(__file__).resolve().parent / "fixtures" / "arena_two_robots_2026-11-24.jpg"
)
EXPECTED_FRAME_SHAPE: tuple[int, int, int] = (720, 1280, 3)


@dataclass
class _ServerBehavior:
    """Mutable behavior controller for the local HTTP test server."""

    payload: bytes
    mode: str = "ok"
    timeout_delay_s: float = 0.25
    stop_event: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def set_mode(self, mode: str) -> None:
        with self.lock:
            self.mode = mode

    def get_mode(self) -> str:
        with self.lock:
            return self.mode


class _TestFrameServer(ThreadingHTTPServer):
    """ThreadingHTTPServer carrying test frame payload and failure mode."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        behavior: _ServerBehavior,
    ) -> None:
        self.behavior = behavior
        super().__init__(
            ("127.0.0.1", 0),
            _TestFrameHandler,
        )


class _TestFrameHandler(BaseHTTPRequestHandler):
    """Minimal HTTP handler serving JPEG bytes, 503, garbage, or delayed timeout."""

    server: _TestFrameServer

    def log_message(
        self,
        fmt: str,
        *args: object,
    ) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        behavior = self.server.behavior
        mode = behavior.get_mode()
        if mode == "503":
            self.send_error(
                HTTPStatus.SERVICE_UNAVAILABLE,
                "No frame captured yet",
            )
            return

        if mode == "timeout":
            behavior.stop_event.wait(behavior.timeout_delay_s)
            return

        body = behavior.payload if mode == "ok" else b"not-a-valid-jpeg-payload"
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type",
                CONTENT_TYPE_JPEG,
            )
            self.send_header(
                "Content-Length",
                str(len(body)),
            )
            self.end_headers()
            self.wfile.write(body)
        except OSError:
            return


@contextmanager
def _running_frame_server(
    initial_mode: str = "ok",
) -> Iterator[tuple[str, _ServerBehavior]]:
    """Starts a local HTTP server on 127.0.0.1 port 0 serving the arena fixture."""
    behavior = _ServerBehavior(
        payload=FIXTURE_PATH.read_bytes(),
        mode=initial_mode,
    )
    server = _TestFrameServer(behavior)
    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )
    thread.start()
    try:
        port = server.server_address[1]
        yield f"http://127.0.0.1:{port}/frame.jpg", behavior
    finally:
        behavior.stop_event.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_should_decode_fixture_frame_with_expected_shape_when_http_server_serves_jpeg() -> None:
    with _running_frame_server("ok") as (url, _behavior):
        source = HttpFrameSource(url)
        frame = source.read()

    assert isinstance(source, FrameSource)
    assert source.is_open() is True
    assert frame is not None
    assert isinstance(frame, np.ndarray)
    assert frame.shape == EXPECTED_FRAME_SHAPE


def test_should_return_none_and_log_once_per_distinct_error_when_http_fails(
    caplog,
) -> None:
    caplog.set_level(
        logging.WARNING,
        logger="overhead_core.frame_source",
    )

    with _running_frame_server("503") as (url, behavior):
        source = HttpFrameSource(
            url,
            timeout_s=0.05,
        )
        initial_open = source.is_open()
        first_503 = source.read()
        second_503 = source.read()

        behavior.set_mode("garbage")
        first_garbage = source.read()
        second_garbage = source.read()

        behavior.set_mode("timeout")
        timeout_frame = source.read()
        final_open = source.is_open()

    assert initial_open is False
    assert first_503 is None
    assert second_503 is None
    assert first_garbage is None
    assert second_garbage is None
    assert timeout_frame is None
    assert final_open is False
    assert len(caplog.records) == 3


def test_should_recover_when_http_server_starts_answering_again(caplog) -> None:
    caplog.set_level(
        logging.WARNING,
        logger="overhead_core.frame_source",
    )

    with _running_frame_server("503") as (url, behavior):
        source = HttpFrameSource(url)
        assert source.is_open() is False
        assert source.read() is None

        behavior.set_mode("ok")
        recovered_frame = source.read()
        assert source.is_open() is True
        assert recovered_frame is not None
        assert recovered_frame.shape == EXPECTED_FRAME_SHAPE

        behavior.set_mode("503")
        assert source.read() is None
        assert source.is_open() is False

        behavior.set_mode("ok")
        reopened = source.reopen()
        after_reopen_frame = source.read()

    assert reopened is True
    assert source.is_open() is True
    assert after_reopen_frame is not None
    assert after_reopen_frame.shape == EXPECTED_FRAME_SHAPE
    assert len(caplog.records) == 2


def test_should_retry_with_camera_session_when_http_source_unavailable_then_ready() -> None:
    with _running_frame_server("503") as (url, behavior):
        source = HttpFrameSource(url)
        session = CameraSession(source)

        startup_step = session.startup(0.0)
        early_step = session.step_starting(0.5)

        behavior.set_mode("ok")
        ready_step = session.step_starting(CAMERA_RETRY_S)

    assert startup_step.outcomes[0].warning_message == LOG_CAMERA_OPEN_RETRY
    assert early_step.outcomes == ()
    assert [o.event for o in ready_step.outcomes] == [
        OverheadEvent.CAMERA_OPENED,
        OverheadEvent.CORNERS_FOUND,
    ]
    assert ready_step.frame is not None
    assert ready_step.frame.shape == EXPECTED_FRAME_SHAPE
    assert session.homography is not None


def test_should_dispatch_http_https_and_still_sources_when_make_frame_source_called(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        HttpFrameSource,
        "_fetch_frame",
        lambda self: None,
    )

    http_source = make_frame_source("http://localhost:8082/frame.jpg")
    https_source = make_frame_source("https://localhost:8443/frame.jpg")
    still_source = make_frame_source(str(FIXTURE_PATH))

    assert isinstance(http_source, HttpFrameSource)
    assert isinstance(https_source, HttpFrameSource)
    assert isinstance(still_source, StillImageFrameSource)
