"""ROS 2 integration tests for manipulator_relay_node."""

import threading
import time

import pytest
from conftest import RosTestHarness
from multirobot_interfaces.srv import (
    ReleaseZone,
    RequestZone,
)
from overhead_core.constants import (
    CubeColor,
    RobotName,
)
from overhead_ros.manipulator_relay_node import ManipulatorRelayNode
from overhead_ros.ros_names import (
    PARAM_HEARTBEAT_TIMEOUT_S,
    PARAM_PERRY_CALL_TIMEOUT_S,
    PERRY_ZONE_RELEASE_SERVICE,
    PERRY_ZONE_REQUEST_SERVICE,
    ROBOT_ZONE_OCCUPANT_QOS,
    ZONE_OCCUPANT_QOS,
    ZONE_OCCUPANT_TOPIC,
    zone_occupant_topic,
    zone_release_service,
    zone_request_service,
)
from rclpy.node import Node
from std_msgs.msg import String

EXPECTED_LEASE_SECONDS: float = 60.0
EXPECTED_DELIVERED_TOTAL: int = 7
PERRY_BEAT_PERIOD_S: float = 0.5
SHORT_HEARTBEAT_TIMEOUT_S: float = 1.0
SHORT_CALL_TIMEOUT_S: float = 0.5
SERVICE_TIMEOUT_S: float = 6.0
ABSENT_PERRY_MAX_LATENCY_S: float = 1.0
ABSENT_SERVICE_WAIT_S: float = 0.5
SILENCE_SETTLE_MARGIN_S: float = 0.3
SILENCE_WINDOW_S: float = 1.5
MIN_SILENCE_AFTER_LAST_BEAT_S: float = 1.0
DEFAULT_BEAT_DURATION_S: float = 6.0
UNKNOWN_ROBOT_NAME: str = "candace"
FAKE_PERRY_NODE_NAME: str = "fake_perry"
TESTER_NODE_NAME: str = "relay_tester"
HEARTBEAT_DURATIONS_S: tuple[float, ...] = (
    2.0,
    3.0,
)


class FakePerry(Node):
    """Fake Perry arbiter node serving zone services and /zone/occupant heartbeats."""

    def __init__(
        self,
        *,
        heartbeat_duration_s: float = DEFAULT_BEAT_DURATION_S,
    ) -> None:
        """Initializes Perry's heartbeat timer and zone request/release services.

        Args:
            heartbeat_duration_s: Duration in seconds to beat on /zone/occupant.
        """
        super().__init__(FAKE_PERRY_NODE_NAME)
        self._heartbeat_duration_s = heartbeat_duration_s
        self._start_s = time.monotonic()
        self._lock = threading.Lock()
        self.received_requests: list[str] = []
        self.received_releases: list[tuple[str, bool, str]] = []

        self._pub_occupant = self.create_publisher(
            String,
            ZONE_OCCUPANT_TOPIC,
            ZONE_OCCUPANT_QOS,
        )
        self._beat_timer = self.create_timer(
            PERRY_BEAT_PERIOD_S,
            self._on_beat_timer,
        )
        self._srv_request = self.create_service(
            RequestZone,
            PERRY_ZONE_REQUEST_SERVICE,
            self._on_request,
        )
        self._srv_release = self.create_service(
            ReleaseZone,
            PERRY_ZONE_RELEASE_SERVICE,
            self._on_release,
        )

    def _on_beat_timer(self) -> None:
        """Publishes /zone/occupant while within the configured heartbeat duration."""
        if time.monotonic() - self._start_s < self._heartbeat_duration_s:
            msg = String()
            msg.data = RobotName.PHINEAS.value
            self._pub_occupant.publish(msg)

    def _on_request(
        self,
        request: RequestZone.Request,
        response: RequestZone.Response,
    ) -> RequestZone.Response:
        """Records the requested robot name and returns a granted zone lease."""
        with self._lock:
            self.received_requests.append(str(request.robot))
        response.granted = True
        response.holder = request.robot
        response.lease_seconds = EXPECTED_LEASE_SECONDS
        return response

    def _on_release(
        self,
        request: ReleaseZone.Request,
        response: ReleaseZone.Response,
    ) -> ReleaseZone.Response:
        """Records the release payload and returns the fixed delivered total."""
        with self._lock:
            self.received_releases.append(
                (
                    str(request.robot),
                    bool(request.delivered),
                    str(request.color),
                )
            )
        response.ok = True
        response.delivered_total = EXPECTED_DELIVERED_TOTAL
        return response


class _TimestampRecorder:
    """Thread-safe recorder for local message arrival timestamps."""

    def __init__(self) -> None:
        """Initializes an empty timestamp list and lock."""
        self._lock = threading.Lock()
        self._timestamps_s: list[float] = []

    def record(self, _msg: String) -> None:
        """Appends the current local monotonic timestamp."""
        with self._lock:
            self._timestamps_s.append(time.monotonic())

    def count(self) -> int:
        """Returns the number of recorded timestamps."""
        with self._lock:
            return len(self._timestamps_s)

    def last_s(self) -> float:
        """Returns the most recent timestamp, or 0.0 if none recorded."""
        with self._lock:
            if not self._timestamps_s:
                return 0.0
            return self._timestamps_s[-1]


def _sleep_until(deadline_s: float) -> None:
    """Sleeps until the local monotonic deadline when it lies in the future."""
    remaining_s = deadline_s - time.monotonic()
    if remaining_s > 0.0:
        time.sleep(remaining_s)


def test_should_forward_zone_request_to_perry_and_return_unchanged_response_when_ready(
    ros_harness: RosTestHarness,
) -> None:
    perry = FakePerry()
    relay = ros_harness.create_node(ManipulatorRelayNode)
    tester = Node(TESTER_NODE_NAME)
    client = tester.create_client(
        RequestZone,
        zone_request_service(RobotName.PHINEAS.value),
    )
    request = RequestZone.Request(robot=RobotName.PHINEAS.value)

    with ros_harness.run_nodes(
        [
            perry,
            relay,
        ],
        tester=tester,
    ):
        response = ros_harness.call_service(
            client,
            request,
            timeout_s=SERVICE_TIMEOUT_S,
        )

    assert perry.received_requests == [RobotName.PHINEAS.value]
    assert response.granted is True
    assert response.holder == RobotName.PHINEAS.value
    assert response.lease_seconds == pytest.approx(EXPECTED_LEASE_SECONDS)


def test_should_forward_zone_release_to_perry_and_return_totals_when_ready(
    ros_harness: RosTestHarness,
) -> None:
    perry = FakePerry()
    relay = ros_harness.create_node(ManipulatorRelayNode)
    tester = Node(TESTER_NODE_NAME)
    client = tester.create_client(
        ReleaseZone,
        zone_release_service(RobotName.FERB.value),
    )
    request = ReleaseZone.Request(
        robot=RobotName.FERB.value,
        delivered=True,
        color=CubeColor.BLUE.value,
    )

    with ros_harness.run_nodes(
        [
            perry,
            relay,
        ],
        tester=tester,
    ):
        response = ros_harness.call_service(
            client,
            request,
            timeout_s=SERVICE_TIMEOUT_S,
        )

    assert response.ok is True
    assert response.delivered_total == EXPECTED_DELIVERED_TOTAL
    assert perry.received_releases == [
        (
            RobotName.FERB.value,
            True,
            CubeColor.BLUE.value,
        ),
    ]


@pytest.mark.parametrize(
    "heartbeat_duration_s",
    HEARTBEAT_DURATIONS_S,
)
def test_should_republish_occupant_while_beating_and_stop_after_timeout(
    heartbeat_duration_s: float,
    ros_harness: RosTestHarness,
) -> None:
    perry = FakePerry(heartbeat_duration_s=heartbeat_duration_s)
    relay = ros_harness.create_node(
        ManipulatorRelayNode,
        overrides={PARAM_HEARTBEAT_TIMEOUT_S: SHORT_HEARTBEAT_TIMEOUT_S},
    )
    tester = Node(TESTER_NODE_NAME)
    recorder = _TimestampRecorder()
    tester.create_subscription(
        String,
        zone_occupant_topic(RobotName.FERB.value),
        recorder.record,
        ROBOT_ZONE_OCCUPANT_QOS,
    )
    t_start_s = time.monotonic()

    with ros_harness.run_nodes(
        [
            perry,
            relay,
        ],
        tester=tester,
    ):
        has_heartbeat = ros_harness.wait_for(
            lambda: recorder.count() > 0,
            timeout_s=heartbeat_duration_s + SHORT_HEARTBEAT_TIMEOUT_S,
        )
        stale_deadline_s = (
            t_start_s + heartbeat_duration_s + SHORT_HEARTBEAT_TIMEOUT_S + SILENCE_SETTLE_MARGIN_S
        )
        _sleep_until(stale_deadline_s)
        count_at_stale = recorder.count()
        time.sleep(SILENCE_WINDOW_S)
        count_after_window = recorder.count()
        silence_s = time.monotonic() - recorder.last_s()

    assert has_heartbeat is True
    assert count_at_stale > 0
    assert count_after_window == count_at_stale
    assert silence_s >= MIN_SILENCE_AFTER_LAST_BEAT_S


def test_should_return_failure_quickly_when_perry_is_absent(
    ros_harness: RosTestHarness,
) -> None:
    relay = ros_harness.create_node(
        ManipulatorRelayNode,
        overrides={PARAM_PERRY_CALL_TIMEOUT_S: SHORT_CALL_TIMEOUT_S},
    )
    tester = Node(TESTER_NODE_NAME)
    req_client = tester.create_client(
        RequestZone,
        zone_request_service(RobotName.PHINEAS.value),
    )
    rel_client = tester.create_client(
        ReleaseZone,
        zone_release_service(RobotName.FERB.value),
    )

    with ros_harness.run_nodes(
        relay,
        tester=tester,
    ):
        assert req_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S)
        assert rel_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S)

        req_start_s = time.monotonic()
        req_response = ros_harness.call_service(
            req_client,
            RequestZone.Request(robot=RobotName.PHINEAS.value),
            timeout_s=SERVICE_TIMEOUT_S,
        )
        req_elapsed_s = time.monotonic() - req_start_s

        rel_start_s = time.monotonic()
        rel_response = ros_harness.call_service(
            rel_client,
            ReleaseZone.Request(
                robot=RobotName.FERB.value,
                delivered=True,
                color=CubeColor.BLUE.value,
            ),
            timeout_s=SERVICE_TIMEOUT_S,
        )
        rel_elapsed_s = time.monotonic() - rel_start_s

    assert req_response.granted is False
    assert req_elapsed_s < ABSENT_PERRY_MAX_LATENCY_S
    assert rel_response.ok is False
    assert rel_elapsed_s < ABSENT_PERRY_MAX_LATENCY_S


def test_should_not_expose_zone_services_when_robot_is_outside_configured_robots(
    ros_harness: RosTestHarness,
) -> None:
    relay = ros_harness.create_node(ManipulatorRelayNode)
    tester = Node(TESTER_NODE_NAME)
    known_req_client = tester.create_client(
        RequestZone,
        zone_request_service(RobotName.PHINEAS.value),
    )
    unknown_req_client = tester.create_client(
        RequestZone,
        zone_request_service(UNKNOWN_ROBOT_NAME),
    )
    unknown_rel_client = tester.create_client(
        ReleaseZone,
        zone_release_service(UNKNOWN_ROBOT_NAME),
    )

    with ros_harness.run_nodes(
        relay,
        tester=tester,
    ):
        is_known_ready = known_req_client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S)
        is_unknown_req_ready = unknown_req_client.wait_for_service(
            timeout_sec=ABSENT_SERVICE_WAIT_S,
        )
        is_unknown_rel_ready = unknown_rel_client.wait_for_service(
            timeout_sec=ABSENT_SERVICE_WAIT_S,
        )

    assert is_known_ready is True
    assert is_unknown_req_ready is False
    assert is_unknown_rel_ready is False
