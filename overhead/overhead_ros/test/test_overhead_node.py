"""ROS 2 integration tests for overhead_node on real-photo and blank fixtures."""

import math
import threading
import time
from collections import Counter
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
from conftest import RosTestHarness
from overhead_core.constants import CubeColor
from overhead_ros.overhead_node import OverheadNode
from overhead_ros.ros_names import (
    PARAM_VIDEO_SOURCE,
    SENSOR_QOS,
    STATE_QOS,
    WORLD_CORRIDOR_FREE_SERVICE,
    WORLD_SENSORS_TOPIC,
    WORLD_STATE_TOPIC,
)
from rclpy.node import Node
from tsa_interfaces.msg import (
    CubeObservation,
    VirtualSensors,
    WorldState,
)
from tsa_interfaces.srv import CorridorFree

FIXTURE_RELATIVE_PATH: Path = (
    Path("overhead_core") / "test" / "fixtures" / "arena_real_2026-11-24.jpg"
)
REAL_PHOTO_FIXTURE_PATH: Path = Path(__file__).resolve().parents[2] / FIXTURE_RELATIVE_PATH

EXPECTED_TOTAL_CUBES: int = 16
EXPECTED_RED_CUBES: int = 5
EXPECTED_YELLOW_CUBES: int = 7
EXPECTED_BLUE_CUBES: int = 4

KNOWN_RED_CUBE_A_XY_M: tuple[float, float] = (
    0.4378,
    0.1452,
)
KNOWN_RED_CUBE_B_XY_M: tuple[float, float] = (
    0.6040,
    0.1925,
)
POSITION_TOLERANCE_M: float = 0.05

OPEN_CORRIDOR_START_XY_M: tuple[float, float] = (
    0.05,
    0.60,
)
OPEN_CORRIDOR_GOAL_XY_M: tuple[float, float] = (
    0.15,
    0.60,
)
CORRIDOR_WIDTH_M: float = 0.10

FIRST_MSG_TIMEOUT_S: float = 5.0
RATE_WINDOW_S: float = 3.0
MIN_WORLD_STATE_RATE_HZ: float = 15.0
SERVICE_TIMEOUT_S: float = 5.0

BLANK_IMAGE_WIDTH_PX: int = 1280
BLANK_IMAGE_HEIGHT_PX: int = 720
BLANK_IMAGE_GRAY_VALUE: int = 128
BLANK_IMAGE_CHANNELS: int = 3
BLANK_OBSERVATION_S: float = 3.0
BLANK_FILENAME: str = "blank_arena.jpg"

TESTER_NODE_NAME: str = "overhead_tester"


class _MessageBuffer:
    """Thread-safe buffer storing received ROS messages."""

    def __init__(self) -> None:
        """Initializes an empty message list and lock."""
        self._lock = threading.Lock()
        self._messages: list[Any] = []

    def record(self, msg: Any) -> None:
        """Appends a newly received ROS message."""
        with self._lock:
            self._messages.append(msg)

    def count(self) -> int:
        """Returns the number of received messages."""
        with self._lock:
            return len(self._messages)

    def latest(self) -> Any | None:
        """Returns the latest received message, or None when empty."""
        with self._lock:
            if not self._messages:
                return None
            return self._messages[-1]


def _has_red_cube_near(
    cubes: list[CubeObservation],
    *,
    target_xy_m: tuple[float, float],
    tolerance_m: float,
) -> bool:
    """Returns True when any red cube lies within tolerance_m of target_xy_m."""
    target_x_m, target_y_m = target_xy_m
    return any(
        cube.color == CubeColor.RED.value
        and math.hypot(
            cube.x - target_x_m,
            cube.y - target_y_m,
        )
        <= tolerance_m
        for cube in cubes
    )


@pytest.fixture()
def real_photo_path() -> str:
    """Returns the path to the real arena photo fixture or skips if missing."""
    if not REAL_PHOTO_FIXTURE_PATH.is_file():
        pytest.skip(f"Missing real arena photo fixture: {REAL_PHOTO_FIXTURE_PATH}")
    return str(REAL_PHOTO_FIXTURE_PATH)


def test_should_publish_expected_cubes_when_running_on_real_photo(
    real_photo_path: str,
    ros_harness: RosTestHarness,
) -> None:
    overhead = ros_harness.create_node(
        OverheadNode,
        overrides={PARAM_VIDEO_SOURCE: real_photo_path},
    )
    tester = Node(TESTER_NODE_NAME)
    buffer = _MessageBuffer()
    tester.create_subscription(
        WorldState,
        WORLD_STATE_TOPIC,
        buffer.record,
        STATE_QOS,
    )

    with ros_harness.run_nodes(
        overhead,
        tester=tester,
    ):
        has_state = ros_harness.wait_for(
            lambda: buffer.count() > 0,
            timeout_s=FIRST_MSG_TIMEOUT_S,
        )
        latest_state = buffer.latest()

    assert has_state is True
    assert latest_state is not None
    assert len(latest_state.cubes) == EXPECTED_TOTAL_CUBES
    color_counts = Counter(cube.color for cube in latest_state.cubes)
    assert color_counts[CubeColor.RED.value] == EXPECTED_RED_CUBES
    assert color_counts[CubeColor.YELLOW.value] == EXPECTED_YELLOW_CUBES
    assert color_counts[CubeColor.BLUE.value] == EXPECTED_BLUE_CUBES
    assert _has_red_cube_near(
        latest_state.cubes,
        target_xy_m=KNOWN_RED_CUBE_A_XY_M,
        tolerance_m=POSITION_TOLERANCE_M,
    )
    assert _has_red_cube_near(
        latest_state.cubes,
        target_xy_m=KNOWN_RED_CUBE_B_XY_M,
        tolerance_m=POSITION_TOLERANCE_M,
    )


def test_should_publish_world_state_at_least_15_hz_when_running_on_real_photo(
    real_photo_path: str,
    ros_harness: RosTestHarness,
) -> None:
    overhead = ros_harness.create_node(
        OverheadNode,
        overrides={PARAM_VIDEO_SOURCE: real_photo_path},
    )
    tester = Node(TESTER_NODE_NAME)
    buffer = _MessageBuffer()
    tester.create_subscription(
        WorldState,
        WORLD_STATE_TOPIC,
        buffer.record,
        STATE_QOS,
    )

    with ros_harness.run_nodes(
        overhead,
        tester=tester,
    ):
        has_initial_state = ros_harness.wait_for(
            lambda: buffer.count() > 0,
            timeout_s=FIRST_MSG_TIMEOUT_S,
        )
        window_start_s = time.monotonic()
        count_before = buffer.count()
        time.sleep(RATE_WINDOW_S)
        elapsed_s = time.monotonic() - window_start_s
        count_after = buffer.count()

    measured_rate_hz = (count_after - count_before) / elapsed_s
    assert has_initial_state is True
    assert measured_rate_hz >= MIN_WORLD_STATE_RATE_HZ


def test_should_publish_obstacles_without_per_robot_when_no_robots_in_photo(
    real_photo_path: str,
    ros_harness: RosTestHarness,
) -> None:
    overhead = ros_harness.create_node(
        OverheadNode,
        overrides={PARAM_VIDEO_SOURCE: real_photo_path},
    )
    tester = Node(TESTER_NODE_NAME)
    buffer = _MessageBuffer()
    tester.create_subscription(
        VirtualSensors,
        WORLD_SENSORS_TOPIC,
        buffer.record,
        SENSOR_QOS,
    )

    with ros_harness.run_nodes(
        overhead,
        tester=tester,
    ):
        has_sensors = ros_harness.wait_for(
            lambda: buffer.count() > 0,
            timeout_s=FIRST_MSG_TIMEOUT_S,
        )
        latest_sensors = buffer.latest()

    assert has_sensors is True
    assert latest_sensors is not None
    assert len(latest_sensors.obstacles) > 0
    assert len(latest_sensors.per_robot) == 0


def test_should_report_blocked_and_open_corridors_when_real_photo_active(
    real_photo_path: str,
    ros_harness: RosTestHarness,
) -> None:
    overhead = ros_harness.create_node(
        OverheadNode,
        overrides={PARAM_VIDEO_SOURCE: real_photo_path},
    )
    tester = Node(TESTER_NODE_NAME)
    state_buffer = _MessageBuffer()
    tester.create_subscription(
        WorldState,
        WORLD_STATE_TOPIC,
        state_buffer.record,
        STATE_QOS,
    )
    corridor_client = tester.create_client(
        CorridorFree,
        WORLD_CORRIDOR_FREE_SERVICE,
    )
    blocked_request = CorridorFree.Request(
        ax=KNOWN_RED_CUBE_A_XY_M[0],
        ay=KNOWN_RED_CUBE_A_XY_M[1],
        gx=KNOWN_RED_CUBE_B_XY_M[0],
        gy=KNOWN_RED_CUBE_B_XY_M[1],
        width=CORRIDOR_WIDTH_M,
    )
    open_request = CorridorFree.Request(
        ax=OPEN_CORRIDOR_START_XY_M[0],
        ay=OPEN_CORRIDOR_START_XY_M[1],
        gx=OPEN_CORRIDOR_GOAL_XY_M[0],
        gy=OPEN_CORRIDOR_GOAL_XY_M[1],
        width=CORRIDOR_WIDTH_M,
    )

    with ros_harness.run_nodes(
        overhead,
        tester=tester,
    ):
        has_state = ros_harness.wait_for(
            lambda: state_buffer.count() > 0,
            timeout_s=FIRST_MSG_TIMEOUT_S,
        )
        blocked_response = ros_harness.call_service(
            corridor_client,
            blocked_request,
            timeout_s=SERVICE_TIMEOUT_S,
        )
        open_response = ros_harness.call_service(
            corridor_client,
            open_request,
            timeout_s=SERVICE_TIMEOUT_S,
        )

    assert has_state is True
    assert blocked_response.free is False
    assert len(blocked_response.blocking_ids) > 0
    assert open_response.free is True
    assert len(open_response.blocking_ids) == 0


def test_should_never_publish_world_state_when_started_on_blank_photo(
    tmp_path: Path,
    ros_harness: RosTestHarness,
) -> None:
    blank_image = np.full(
        (
            BLANK_IMAGE_HEIGHT_PX,
            BLANK_IMAGE_WIDTH_PX,
            BLANK_IMAGE_CHANNELS,
        ),
        BLANK_IMAGE_GRAY_VALUE,
        dtype=np.uint8,
    )
    blank_path = tmp_path / BLANK_FILENAME
    is_written = cv2.imwrite(
        str(blank_path),
        blank_image,
    )
    assert is_written is True

    overhead = ros_harness.create_node(
        OverheadNode,
        overrides={PARAM_VIDEO_SOURCE: str(blank_path)},
    )
    tester = Node(TESTER_NODE_NAME)
    buffer = _MessageBuffer()
    tester.create_subscription(
        WorldState,
        WORLD_STATE_TOPIC,
        buffer.record,
        STATE_QOS,
    )

    with ros_harness.run_nodes(
        overhead,
        tester=tester,
    ):
        time.sleep(BLANK_OBSERVATION_S)
        state_count = buffer.count()

    assert state_count == 0
