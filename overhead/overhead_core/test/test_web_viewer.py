"""Unit tests for the standalone ThreadingHTTPServer web viewer."""

import json
import threading
import urllib.error
import urllib.request
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import pytest
from overhead_core.constants import ViewerView
from overhead_core.web_viewer import (
    FrameProvider,
    OverheadWebServer,
    ViewerSnapshot,
    create_web_server,
)
from overhead_core.web_viewer_app import CameraViewerState


def _make_fake_snapshot() -> ViewerSnapshot:
    """Builds a synthetic ViewerSnapshot for server tests."""
    raw_img = np.full(
        (120, 160, 3),
        50,
        dtype=np.uint8,
    )
    rect_img = np.full(
        (100, 140, 3),
        120,
        dtype=np.uint8,
    )
    mask_img = np.full(
        (120, 160, 3),
        200,
        dtype=np.uint8,
    )
    return ViewerSnapshot(
        views={
            ViewerView.RAW.value: raw_img,
            ViewerView.RECTIFIED.value: rect_img,
            ViewerView.MASK.value: mask_img,
        },
        raw_frame=raw_img,
        fps=18.5,
        cube_count=3,
        robot_count=2,
        is_calibrated=True,
    )


@contextmanager
def _running_server(
    frame_provider: FrameProvider,
    *,
    on_recalibrate: Callable[[], bool] | None = None,
    on_snapshot: Callable[[ViewerSnapshot], Path | None] | None = None,
) -> Iterator[OverheadWebServer]:
    """Starts an OverheadWebServer on localhost port 0 and cleans it up on exit."""
    server = create_web_server(
        "127.0.0.1",
        0,
        frame_provider=frame_provider,
        on_recalibrate=on_recalibrate,
        on_snapshot=on_snapshot,
    )
    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )
    thread.start()
    try:
        yield server
    finally:
        server.stop()
        thread.join(timeout=2.0)


def test_should_serve_html_page_when_get_root():
    with _running_server(_make_fake_snapshot) as server:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/") as resp:
            status = resp.status
            content_type = resp.headers.get("Content-Type", "")
            body = resp.read().decode("utf-8")

    assert status == 200
    assert "text/html" in content_type
    assert "Snapshot" in body
    assert "Recalibrate" in body
    assert ViewerView.RECTIFIED.value in body


def test_should_return_jpeg_with_magic_bytes_when_get_snapshot():
    captured_calls: list[ViewerSnapshot] = []

    def handle_snapshot(snap: ViewerSnapshot) -> None:
        captured_calls.append(snap)

    with _running_server(
        _make_fake_snapshot,
        on_snapshot=handle_snapshot,
    ) as server:
        port = server.server_address[1]
        url = f"http://127.0.0.1:{port}/snapshot.jpg?view={ViewerView.RECTIFIED.value}"
        with urllib.request.urlopen(url) as resp:
            status = resp.status
            content_type = resp.headers.get("Content-Type", "")
            payload = resp.read()

    assert status == 200
    assert "image/jpeg" in content_type
    assert payload[:2] == b"\xff\xd8"
    assert payload[-2:] == b"\xff\xd9"
    assert len(captured_calls) == 1


def test_should_return_bad_request_when_snapshot_view_is_invalid():
    with _running_server(_make_fake_snapshot) as server:
        port = server.server_address[1]
        url = f"http://127.0.0.1:{port}/snapshot.jpg?view=bogus"
        with pytest.raises(urllib.error.HTTPError) as exc_info:
            urllib.request.urlopen(url)

    assert exc_info.value.code == 400


def test_should_trigger_recalibrate_callback_when_post_recalibrate():
    calls = 0

    def handle_recalibrate() -> bool:
        nonlocal calls
        calls += 1
        return True

    with _running_server(
        _make_fake_snapshot,
        on_recalibrate=handle_recalibrate,
    ) as server:
        port = server.server_address[1]
        req = urllib.request.Request(
            f"http://127.0.0.1:{port}/recalibrate",
            data=b"",
            method="POST",
        )
        with urllib.request.urlopen(req) as resp:
            status = resp.status
            data = json.loads(resp.read().decode("utf-8"))

    assert status == 200
    assert calls == 1
    assert data["calibrated"] is True


def test_should_return_live_status_json_when_get_status():
    with _running_server(_make_fake_snapshot) as server:
        port = server.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/status") as resp:
            status = resp.status
            data = json.loads(resp.read().decode("utf-8"))

    assert status == 200
    assert data == {
        "fps": 18.5,
        "cubes": 3,
        "robots": 2,
        "calibrated": True,
    }


def test_should_stream_mjpeg_multipart_when_get_stream():
    with _running_server(_make_fake_snapshot) as server:
        port = server.server_address[1]
        url = f"http://127.0.0.1:{port}/stream/{ViewerView.RAW.value}.mjpg"
        with urllib.request.urlopen(
            url,
            timeout=2.0,
        ) as resp:
            status = resp.status
            content_type = resp.headers.get("Content-Type", "")
            chunk = resp.read(1024)

    assert status == 200
    assert "multipart/x-mixed-replace" in content_type
    assert b"--frame" in chunk
    assert b"\xff\xd8" in chunk


def test_should_save_raw_jpeg_file_when_snapshot_saved(tmp_path: Path):
    state = CameraViewerState(captures_dir=tmp_path)
    snapshot = _make_fake_snapshot()

    saved_path = state.save_raw_snapshot(snapshot)

    assert saved_path is not None
    assert saved_path.exists()
    assert saved_path.suffix == ".jpg"
    raw_bytes = saved_path.read_bytes()
    assert raw_bytes[:2] == b"\xff\xd8"
