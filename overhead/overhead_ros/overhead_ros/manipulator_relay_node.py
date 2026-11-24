"""Thin ROS 2 relay node forwarding zone messages between robots and Perry."""

import threading
import time
from collections.abc import Callable
from typing import Any

import rclpy
from multirobot_interfaces.srv import (
    ReleaseZone,
    RequestZone,
)
from overhead_core.constants import (
    DEFAULT_DELIVERED_TOTAL,
    DEFAULT_LEASE_SECONDS,
    DEFAULT_RELAY_REPUBLISH_RATE_HZ,
    DEFAULT_ROBOT_NAMES,
    LOG_RELAY_HEARTBEAT_RECOVERED,
    LOG_RELAY_HEARTBEAT_STALE_FMT,
    LOG_RELAY_INITIALIZED_FMT,
    LOG_RELAY_PERRY_CALL_FAILED_FMT,
    LOG_RELAY_PERRY_TIMEOUT_FMT,
    LOG_RELAY_PERRY_UNAVAILABLE_FMT,
    LOG_RELAY_REJECTED_FMT,
    MIN_RELAY_REPUBLISH_RATE_HZ,
    NO_ZONE_OCCUPANT,
    PERRY_CALL_TIMEOUT_S,
    PERRY_HEARTBEAT_TIMEOUT_S,
    RelayDestination,
    RelayMessageKind,
)
from overhead_core.relay import route_message
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String

from overhead_ros.ros_names import (
    MANIPULATOR_RELAY_NODE_NAME,
    PARAM_HEARTBEAT_TIMEOUT_S,
    PARAM_PERRY_CALL_TIMEOUT_S,
    PARAM_REPUBLISH_RATE_HZ,
    PARAM_ROBOTS,
    PERRY_ZONE_RELEASE_SERVICE,
    PERRY_ZONE_REQUEST_SERVICE,
    ROBOT_ZONE_OCCUPANT_QOS,
    ZONE_OCCUPANT_QOS,
    ZONE_OCCUPANT_TOPIC,
    zone_occupant_topic,
    zone_release_service,
    zone_request_service,
)


class ManipulatorRelayNode(Node):
    """Relays zone requests, releases, and occupant heartbeats between robots and Perry."""

    def __init__(self) -> None:
        """Initializes parameters, heartbeat state, and ROS relay interfaces."""
        super().__init__(MANIPULATOR_RELAY_NODE_NAME)

        self._declare_parameters()
        self._load_parameters()

        self._callback_group = ReentrantCallbackGroup()
        # A reentrant group re-adds a service that is still awaiting Perry to the wait set and
        # crashes the executor, so the zone services share a mutually exclusive group. Perry's
        # clients and the timeout timer stay in the reentrant group so the awaited call can finish.
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._heartbeat_lock = threading.Lock()
        self._last_occupant: str | None = None
        self._last_occupant_rx_s: float | None = None
        self._is_heartbeat_stale = False

        self._occupant_pubs: dict[str, Any] = {}
        self._zone_services: list[Any] = []

        self._create_ros_interfaces()
        self.get_logger().info(
            LOG_RELAY_INITIALIZED_FMT.format(list(self._robots)),
        )

    def _declare_parameters(self) -> None:
        """Declares relay ROS parameters using package constants as defaults."""
        self.declare_parameter(
            PARAM_ROBOTS,
            list(DEFAULT_ROBOT_NAMES),
        )
        self.declare_parameter(
            PARAM_PERRY_CALL_TIMEOUT_S,
            PERRY_CALL_TIMEOUT_S,
        )
        self.declare_parameter(
            PARAM_HEARTBEAT_TIMEOUT_S,
            PERRY_HEARTBEAT_TIMEOUT_S,
        )
        self.declare_parameter(
            PARAM_REPUBLISH_RATE_HZ,
            DEFAULT_RELAY_REPUBLISH_RATE_HZ,
        )

    def _load_parameters(self) -> None:
        """Loads parameter values into typed node attributes."""
        raw_robots = self.get_parameter(PARAM_ROBOTS).value
        self._robots: tuple[str, ...] = tuple(str(name) for name in raw_robots)
        self._perry_call_timeout_s = float(
            self.get_parameter(PARAM_PERRY_CALL_TIMEOUT_S).value,
        )
        self._heartbeat_timeout_s = float(
            self.get_parameter(PARAM_HEARTBEAT_TIMEOUT_S).value,
        )
        self._republish_rate_hz = float(
            self.get_parameter(PARAM_REPUBLISH_RATE_HZ).value,
        )

    def _create_ros_interfaces(self) -> None:
        """Creates Perry clients, per-robot publishers and services, and republish timer."""
        self._sub_zone_occupant = self.create_subscription(
            String,
            ZONE_OCCUPANT_TOPIC,
            self._on_zone_occupant,
            ZONE_OCCUPANT_QOS,
            callback_group=self._callback_group,
        )
        self._perry_request_client = self.create_client(
            RequestZone,
            PERRY_ZONE_REQUEST_SERVICE,
            callback_group=self._callback_group,
        )
        self._perry_release_client = self.create_client(
            ReleaseZone,
            PERRY_ZONE_RELEASE_SERVICE,
            callback_group=self._callback_group,
        )

        for robot in self._robots:
            self._occupant_pubs[robot] = self.create_publisher(
                String,
                zone_occupant_topic(robot),
                ROBOT_ZONE_OCCUPANT_QOS,
            )
            self._zone_services.append(
                self.create_service(
                    RequestZone,
                    zone_request_service(robot),
                    self._make_request_handler(robot),
                    callback_group=self._service_group,
                ),
            )
            self._zone_services.append(
                self.create_service(
                    ReleaseZone,
                    zone_release_service(robot),
                    self._make_release_handler(robot),
                    callback_group=self._service_group,
                ),
            )

        republish_period_s = 1.0 / max(
            MIN_RELAY_REPUBLISH_RATE_HZ,
            self._republish_rate_hz,
        )
        self._republish_timer = self.create_timer(
            republish_period_s,
            self._on_republish_timer,
            callback_group=self._callback_group,
        )

    def _on_zone_occupant(self, msg: String) -> None:
        """Records Perry's occupant heartbeat on the local clock and republishes if fresh."""
        rx_s = time.monotonic()
        occupant_value = str(msg.data)

        with self._heartbeat_lock:
            was_stale = self._is_heartbeat_stale
            self._last_occupant = occupant_value
            self._last_occupant_rx_s = rx_s
            self._is_heartbeat_stale = False

        if was_stale:
            self.get_logger().info(LOG_RELAY_HEARTBEAT_RECOVERED)

        age_s = max(
            0.0,
            time.monotonic() - rx_s,
        )
        self._publish_occupant_if_fresh(
            occupant_value=occupant_value,
            age_s=age_s,
        )

    def _on_republish_timer(self) -> None:
        """Republishes the latest Perry heartbeat while fresh and logs staleness once."""
        now_s = time.monotonic()
        with self._heartbeat_lock:
            if self._last_occupant_rx_s is None or self._last_occupant is None:
                return
            occupant_value = self._last_occupant
            last_rx_s = self._last_occupant_rx_s

        age_s = max(
            0.0,
            now_s - last_rx_s,
        )
        if self._publish_occupant_if_fresh(
            occupant_value=occupant_value,
            age_s=age_s,
        ):
            return

        should_log_stale = False
        with self._heartbeat_lock:
            # Verify no newer heartbeat arrived while deciding to avoid stale log races.
            is_still_same_sample = self._last_occupant_rx_s == last_rx_s
            if is_still_same_sample and not self._is_heartbeat_stale:
                self._is_heartbeat_stale = True
                should_log_stale = True

        if should_log_stale:
            self.get_logger().warning(
                LOG_RELAY_HEARTBEAT_STALE_FMT.format(
                    age_s,
                    self._heartbeat_timeout_s,
                ),
            )

    def _publish_occupant_if_fresh(
        self,
        *,
        occupant_value: str,
        age_s: float,
    ) -> bool:
        """Publishes /<robot>/zone/occupant only when route_message marks it fresh."""
        decision = route_message(
            RelayMessageKind.ZONE_OCCUPANT,
            known_robots=self._robots,
            robot_name=None,
            age_s=age_s,
        )
        is_fresh = decision.is_fresh and age_s <= self._heartbeat_timeout_s
        should_publish = (
            decision.is_accepted and is_fresh and RelayDestination.ROBOT in decision.destinations
        )
        if not should_publish:
            return False

        msg = String()
        msg.data = occupant_value
        for robot in decision.target_robots:
            publisher = self._occupant_pubs.get(robot)
            if publisher is not None:
                publisher.publish(msg)

        return True

    def _make_request_handler(
        self,
        robot_name: str,
    ) -> Callable[..., Any]:
        """Creates an async RequestZone service callback bound to one robot namespace."""

        async def _callback(
            request: RequestZone.Request,
            response: RequestZone.Response,
        ) -> RequestZone.Response:
            return await self._handle_zone_request(
                robot_name=robot_name,
                request=request,
                response=response,
            )

        return _callback

    def _make_release_handler(
        self,
        robot_name: str,
    ) -> Callable[..., Any]:
        """Creates an async ReleaseZone service callback bound to one robot namespace."""

        async def _callback(
            request: ReleaseZone.Request,
            response: ReleaseZone.Response,
        ) -> ReleaseZone.Response:
            return await self._handle_zone_release(
                robot_name=robot_name,
                request=request,
                response=response,
            )

        return _callback

    async def _handle_zone_request(
        self,
        *,
        robot_name: str,
        request: RequestZone.Request,
        response: RequestZone.Response,
    ) -> RequestZone.Response:
        """Routes a robot's zone request to Perry and returns Perry's response unchanged."""
        perry_req = RequestZone.Request()
        perry_req.robot = request.robot
        perry_res = await self._forward_zone_call(
            robot_name=robot_name,
            request_kind=RelayMessageKind.ROBOT_ZONE_REQUEST,
            response_kind=RelayMessageKind.ZONE_REQUEST_RESPONSE,
            client=self._perry_request_client,
            service_name=PERRY_ZONE_REQUEST_SERVICE,
            perry_request=perry_req,
        )
        if perry_res is None:
            return self._build_failed_request_response(response)

        response.granted = perry_res.granted
        response.holder = perry_res.holder
        response.lease_seconds = perry_res.lease_seconds
        return response

    async def _handle_zone_release(
        self,
        *,
        robot_name: str,
        request: ReleaseZone.Request,
        response: ReleaseZone.Response,
    ) -> ReleaseZone.Response:
        """Routes a robot's zone release to Perry and returns Perry's response unchanged."""
        perry_req = ReleaseZone.Request()
        perry_req.robot = request.robot
        perry_req.delivered = request.delivered
        perry_req.color = request.color
        perry_res = await self._forward_zone_call(
            robot_name=robot_name,
            request_kind=RelayMessageKind.ROBOT_ZONE_RELEASE,
            response_kind=RelayMessageKind.ZONE_RELEASE_RESPONSE,
            client=self._perry_release_client,
            service_name=PERRY_ZONE_RELEASE_SERVICE,
            perry_request=perry_req,
        )
        if perry_res is None:
            return self._build_failed_release_response(response)

        response.ok = perry_res.ok
        response.delivered_total = perry_res.delivered_total
        return response

    async def _forward_zone_call(
        self,
        *,
        robot_name: str,
        request_kind: RelayMessageKind,
        response_kind: RelayMessageKind,
        client: Any,
        service_name: str,
        perry_request: Any,
    ) -> Any | None:
        """Validates routing, forwards a request to Perry, and validates response routing."""
        if not self._is_route_allowed(
            request_kind,
            robot_name=robot_name,
            expected_dest=RelayDestination.PERRY,
        ):
            return None

        if not client.service_is_ready():
            self.get_logger().warning(
                LOG_RELAY_PERRY_UNAVAILABLE_FMT.format(
                    service_name,
                    robot_name,
                ),
            )
            return None

        perry_res, is_timed_out = await self._call_perry_async(
            client,
            perry_request,
            service_name=service_name,
            robot_name=robot_name,
        )
        if is_timed_out or perry_res is None:
            return None

        if not self._is_route_allowed(
            response_kind,
            robot_name=robot_name,
            expected_dest=RelayDestination.ROBOT,
        ):
            return None

        return perry_res

    def _is_route_allowed(
        self,
        kind: RelayMessageKind,
        *,
        robot_name: str,
        expected_dest: RelayDestination,
    ) -> bool:
        """Checks route_message acceptance and destination for one robot message."""
        decision = route_message(
            kind,
            known_robots=self._robots,
            robot_name=robot_name,
            age_s=0.0,
        )
        is_allowed = decision.is_accepted and expected_dest in decision.destinations
        if not is_allowed:
            self.get_logger().warning(
                LOG_RELAY_REJECTED_FMT.format(
                    kind.value,
                    robot_name,
                ),
            )
        return is_allowed

    async def _call_perry_async(
        self,
        client: Any,
        request: Any,
        *,
        service_name: str,
        robot_name: str,
    ) -> tuple[Any | None, bool]:
        """Calls a Perry service asynchronously with a non-blocking node-clock timeout."""
        if self._perry_call_timeout_s <= 0.0:
            self.get_logger().warning(
                LOG_RELAY_PERRY_TIMEOUT_FMT.format(
                    service_name,
                    0.0,
                    robot_name,
                ),
            )
            return None, True

        start_s = time.monotonic()
        result_future: Future = Future()
        settle_lock = threading.Lock()
        is_settled = False

        def _on_timeout() -> None:
            nonlocal is_settled
            with settle_lock:
                if is_settled:
                    return
                is_settled = True
            result_future.set_result((None, True))

        def _on_client_done(done_future: Future) -> None:
            nonlocal is_settled
            with settle_lock:
                if is_settled:
                    return
                is_settled = True
            result = self._extract_future_result(
                done_future,
                service_name=service_name,
                robot_name=robot_name,
            )
            result_future.set_result((result, False))

        timeout_timer = self.create_timer(
            self._perry_call_timeout_s,
            _on_timeout,
            callback_group=self._callback_group,
        )
        client_future = client.call_async(request)
        client_future.add_done_callback(_on_client_done)

        try:
            call_result, is_timed_out = await result_future
        finally:
            timeout_timer.cancel()
            self.destroy_timer(timeout_timer)

        if is_timed_out:
            client.remove_pending_request(client_future)
            self.get_logger().warning(
                LOG_RELAY_PERRY_TIMEOUT_FMT.format(
                    service_name,
                    time.monotonic() - start_s,
                    robot_name,
                ),
            )

        return call_result, is_timed_out

    def _extract_future_result(
        self,
        done_future: Future,
        *,
        service_name: str,
        robot_name: str,
    ) -> Any | None:
        """Extracts a completed client future's result or logs its failure."""
        if done_future.cancelled():
            return None

        exc = done_future.exception()
        if exc is not None:
            self.get_logger().error(
                LOG_RELAY_PERRY_CALL_FAILED_FMT.format(
                    service_name,
                    robot_name,
                    exc,
                ),
            )
            return None

        return done_future.result()

    def _build_failed_request_response(
        self,
        response: RequestZone.Response,
    ) -> RequestZone.Response:
        """Populates default failure fields for a RequestZone response."""
        response.granted = False
        response.holder = NO_ZONE_OCCUPANT
        response.lease_seconds = DEFAULT_LEASE_SECONDS
        return response

    def _build_failed_release_response(
        self,
        response: ReleaseZone.Response,
    ) -> ReleaseZone.Response:
        """Populates default failure fields for a ReleaseZone response."""
        response.ok = False
        response.delivered_total = DEFAULT_DELIVERED_TOTAL
        return response


def main(args: list[str] | None = None) -> None:
    """Entry point for manipulator_relay_node."""
    rclpy.init(args=args)
    node = ManipulatorRelayNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
