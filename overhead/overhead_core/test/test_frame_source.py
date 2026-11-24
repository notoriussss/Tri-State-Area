"""Unit tests for camera and still-image frame sources."""

from pathlib import Path

import cv2
import numpy as np
from overhead_core.frame_source import (
    CameraFrameSource,
    FrameSource,
    StillImageFrameSource,
    make_frame_source,
)


class _FakeVideoCapture:
    """In-memory fake replacing cv2.VideoCapture without touching hardware."""

    created_instances: list["_FakeVideoCapture"] = []

    def __init__(
        self,
        index: int,
        *,
        is_opened: bool = True,
        frame: np.ndarray | None = None,
    ) -> None:
        self.index = index
        self._opened = is_opened
        self._frame = frame
        self.props: dict[int, float] = {}
        self.released = False
        _FakeVideoCapture.created_instances.append(self)

    def isOpened(self) -> bool:  # noqa: N802
        return self._opened

    def set(
        self,
        prop_id: int,
        value: float,
    ) -> bool:
        self.props[prop_id] = value
        return True

    def read(self) -> tuple[bool, np.ndarray | None]:
        if not self._opened or self._frame is None:
            return False, None
        return True, self._frame.copy()

    def release(self) -> None:
        self.released = True
        self._opened = False


def _write_synthetic_png(
    path: Path,
    image: np.ndarray,
) -> None:
    ok, buffer = cv2.imencode(
        ".png",
        image,
    )
    assert ok is True
    path.write_bytes(buffer.tobytes())


def test_should_return_equal_copies_repeatedly_when_still_image_valid(
    tmp_path: Path,
) -> None:
    expected = np.full(
        (24, 32, 3),
        120,
        dtype=np.uint8,
    )
    expected[4:12, 8:20] = (10, 200, 50)
    image_path = tmp_path / "arena_ñ_frame.png"
    _write_synthetic_png(
        image_path,
        expected,
    )
    source = StillImageFrameSource(image_path)

    first_frame = source.read()
    assert first_frame is not None
    first_frame[0, 0] = (0, 0, 0)
    second_frame = source.read()

    assert isinstance(source, FrameSource)
    assert source.is_open() is True
    assert second_frame is not None
    assert np.array_equal(
        second_frame,
        expected,
    )


def test_should_report_not_open_when_still_image_missing(
    tmp_path: Path,
) -> None:
    missing_path = tmp_path / "does_not_exist.png"

    source = StillImageFrameSource(missing_path)

    assert source.is_open() is False
    assert source.read() is None
    assert source.reopen() is False


def test_should_report_not_open_when_still_image_corrupted(
    tmp_path: Path,
) -> None:
    bad_path = tmp_path / "corrupt.jpg"
    bad_path.write_bytes(b"not-a-valid-image")

    source = StillImageFrameSource(bad_path)

    assert source.is_open() is False
    assert source.read() is None


def test_should_pick_still_or_camera_source_when_factory_called(
    tmp_path: Path,
    monkeypatch,
) -> None:
    _FakeVideoCapture.created_instances.clear()
    fake_frame = np.zeros(
        (16, 16, 3),
        dtype=np.uint8,
    )
    monkeypatch.setattr(
        cv2,
        "VideoCapture",
        lambda idx: _FakeVideoCapture(
            idx,
            frame=fake_frame,
        ),
    )
    image_path = tmp_path / "still.png"
    _write_synthetic_png(
        image_path,
        fake_frame,
    )

    still_source = make_frame_source(
        str(image_path),
        camera_index=0,
        width_px=640,
        height_px=480,
    )
    camera_source = make_frame_source(
        "",
        camera_index=2,
        width_px=1280,
        height_px=720,
    )

    assert isinstance(still_source, StillImageFrameSource)
    assert isinstance(camera_source, CameraFrameSource)
    assert camera_source.is_open() is True
    assert len(_FakeVideoCapture.created_instances) == 1
    cap = _FakeVideoCapture.created_instances[0]
    assert cap.index == 2
    assert cap.props[cv2.CAP_PROP_FRAME_WIDTH] == 1280
    assert cap.props[cv2.CAP_PROP_FRAME_HEIGHT] == 720


def test_should_release_and_reopen_capture_when_camera_reopened(
    monkeypatch,
) -> None:
    _FakeVideoCapture.created_instances.clear()
    fake_frame = np.full(
        (10, 10, 3),
        75,
        dtype=np.uint8,
    )
    monkeypatch.setattr(
        cv2,
        "VideoCapture",
        lambda idx: _FakeVideoCapture(
            idx,
            frame=fake_frame,
        ),
    )
    source = CameraFrameSource(
        1,
        width_px=800,
        height_px=600,
    )
    first_cap = _FakeVideoCapture.created_instances[0]

    reopened = source.reopen()
    read_frame = source.read()

    assert reopened is True
    assert first_cap.released is True
    assert len(_FakeVideoCapture.created_instances) == 2
    assert read_frame is not None
    assert np.array_equal(
        read_frame,
        fake_frame,
    )
