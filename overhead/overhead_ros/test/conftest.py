"""Pytest fixtures and executor helpers for overhead_ros integration tests."""

from __future__ import annotations

import os
import threading
import time
from collections.abc import Callable, Iterator, Sequence
from contextlib import AbstractContextManager, contextmanager, suppress
from dataclasses import dataclass
from typing import Any
from unittest.mock import patch

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import (  # noqa: E402
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

ROS_DOMAIN_ID_ENV: str = "ROS_DOMAIN_ID"
MIN_ISOLATED_DOMAIN_ID: int = 1
ISOLATED_DOMAIN_ID_SPAN: int = 100
EXECUTOR_THREAD_COUNT: int = 4
EXECUTOR_SHUTDOWN_TIMEOUT_S: float = 2.0
THREAD_JOIN_TIMEOUT_S: float = 2.0
POLL_INTERVAL_S: float = 0.02


def _spin_executor(executor: Any) -> None:
    """Spins an executor until shutdown, ignoring external shutdown exceptions."""
    with suppress(ExternalShutdownException):
        executor.spin()


def _stop_executor(
    executor: Any,
    thread: threading.Thread,
    nodes: Sequence[Node],
) -> None:
    """Shuts down an executor, joins its thread, and destroys all attached nodes."""
    executor.shutdown(timeout_sec=EXECUTOR_SHUTDOWN_TIMEOUT_S)
    thread.join(timeout=THREAD_JOIN_TIMEOUT_S)
    for node in nodes:
        node.destroy_node()


@contextmanager
def spin_node_pair(
    under_test: Node | Sequence[Node],
    *,
    tester: Node,
) -> Iterator[None]:
    """Spins test-subject nodes on MultiThreadedExecutor and tester on SingleThreadedExecutor.

    Args:
        under_test: Single node or sequence of nodes executed under MultiThreadedExecutor.
        tester: Client/subscriber test node executed under SingleThreadedExecutor.

    Yields:
        Control to the test body while both executors spin in background threads.
    """
    test_nodes: list[Node] = [under_test] if isinstance(under_test, Node) else list(under_test)
    under_test_executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREAD_COUNT)
    for node in test_nodes:
        under_test_executor.add_node(node)

    tester_executor = SingleThreadedExecutor()
    tester_executor.add_node(tester)

    under_test_thread = threading.Thread(
        target=_spin_executor,
        args=(under_test_executor,),
        daemon=True,
    )
    tester_thread = threading.Thread(
        target=_spin_executor,
        args=(tester_executor,),
        daemon=True,
    )
    under_test_thread.start()
    tester_thread.start()
    try:
        yield
    finally:
        _stop_executor(
            tester_executor,
            tester_thread,
            [tester],
        )
        _stop_executor(
            under_test_executor,
            under_test_thread,
            test_nodes,
        )


def create_node_with_parameters(
    node_cls: type[Any],
    *,
    overrides: dict[str, Any] | None = None,
) -> Any:
    """Instantiates a ROS 2 node subclass with optional initial parameter overrides.

    Args:
        node_cls: Node class whose constructor takes no required arguments.
        overrides: Optional mapping from parameter names to initial override values.

    Returns:
        Constructed node instance with parameter overrides applied at initialization.
    """
    if not overrides:
        return node_cls()

    parameter_overrides = [
        Parameter(
            name,
            value=value,
        )
        for name, value in overrides.items()
    ]
    original_init = Node.__init__

    def _init_with_overrides(
        self: Node,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        kwargs.setdefault(
            "parameter_overrides",
            parameter_overrides,
        )
        original_init(
            self,
            *args,
            **kwargs,
        )

    with patch.object(
        Node,
        "__init__",
        _init_with_overrides,
    ):
        return node_cls()


def wait_until(
    predicate: Callable[[], bool],
    *,
    timeout_s: float,
) -> bool:
    """Polls predicate at fixed intervals until it returns True or timeout_s elapses.

    Args:
        predicate: Zero-argument callable returning True when the awaited condition holds.
        timeout_s: Maximum duration in seconds to wait before giving up.

    Returns:
        True if predicate became True within timeout_s, otherwise its final result.
    """
    deadline_s = time.monotonic() + timeout_s
    while time.monotonic() < deadline_s:
        if predicate():
            return True
        time.sleep(POLL_INTERVAL_S)
    return predicate()


def call_service(
    client: Any,
    request: Any,
    *,
    timeout_s: float,
) -> Any:
    """Waits for a ROS 2 service, performs an async call, and returns its response.

    Args:
        client: Active rclpy service client attached to a spinning node.
        request: Populated ROS 2 service request message.
        timeout_s: Maximum duration in seconds for discovery and response completion.

    Returns:
        Service response message returned by the server.
    """
    service_name = getattr(
        client,
        "srv_name",
        "service",
    )
    is_ready = client.wait_for_service(timeout_sec=timeout_s)
    assert is_ready, f"Service {service_name} did not become ready within {timeout_s}s."

    future = client.call_async(request)
    is_done = wait_until(
        future.done,
        timeout_s=timeout_s,
    )
    assert is_done, f"Service {service_name} did not respond within {timeout_s}s."
    return future.result()


@dataclass(frozen=True)
class RosTestHarness:
    """Bundle of ROS 2 test helpers for node creation, spinning, polling, and service calls."""

    create_node: Callable[..., Any]
    run_nodes: Callable[..., AbstractContextManager[None]]
    wait_for: Callable[..., bool]
    call_service: Callable[..., Any]


@pytest.fixture(
    scope="session",
    autouse=True,
)
def ros_session() -> Iterator[int]:
    """Initializes rclpy once per session with a PID-isolated ROS_DOMAIN_ID.

    Yields:
        Isolated ROS_DOMAIN_ID integer in [1, 100] configured for this session.
    """
    domain_id = (os.getpid() % ISOLATED_DOMAIN_ID_SPAN) + MIN_ISOLATED_DOMAIN_ID
    os.environ[ROS_DOMAIN_ID_ENV] = str(domain_id)
    if not rclpy.ok():
        rclpy.init(args=[])
    try:
        yield domain_id
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture()
def ros_harness() -> RosTestHarness:
    """Provides the ROS 2 integration test helper bundle."""
    return RosTestHarness(
        create_node=create_node_with_parameters,
        run_nodes=spin_node_pair,
        wait_for=wait_until,
        call_service=call_service,
    )
