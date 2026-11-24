"""One-shot ROS 2 CLI node verifying ROS and HTTP connectivity to collector robots."""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from collections.abc import Callable
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from overhead_core.constants import (
    DEFAULT_ROBOT_HOSTS,
    DEFAULT_ROBOT_LINK_LISTEN_S,
    DEFAULT_ROBOT_NAMES,
    DEFAULT_ROBOT_WEB_PORT,
    ERR_ROBOT_LINK_EMPTY_ROBOTS,
    ERR_ROBOT_LINK_HOST_MISMATCH_FMT,
    ERR_ROBOT_LINK_INVALID_LISTEN_S_FMT,
    ERR_ROBOT_LINK_INVALID_PORT_FMT,
    LOG_ROBOT_LINK_START_FMT,
    LOG_ROBOT_LINK_WEB_FETCH_FAILED_FMT,
    MAX_PORT_NUMBER,
    MIN_PORT_NUMBER,
    ROBOT_LINK_POLL_PERIOD_S,
    ROBOT_LINK_THREAD_JOIN_TIMEOUT_S,
    ROBOT_WEB_REQUEST_TIMEOUT_S,
    ROBOT_WEB_STATUS_PATH,
    ROBOT_WEB_URL_FMT,
)
from overhead_core.robot_link import (
    RobotLinkReport,
    RobotWebStatus,
    evaluate_robot_link,
    format_robot_link_report,
    parse_robot_web_status,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tsa_interfaces.msg import (
    RobotPlan,
    RobotStatus,
    WorldState,
)

from overhead_ros.conversions import located_robot_names_from_world_state
from overhead_ros.ros_names import (
    CONFIG_DIR_NAME,
    OVERHEAD_ROS_PACKAGE_NAME,
    PARAM_LISTEN_S,
    PARAM_ROBOT_HOSTS,
    PARAM_ROBOT_NAMES,
    PARAM_ROBOT_WEB_PORT,
    PLAN_QOS,
    ROBOT_LINK_CHECK_NODE_NAME,
    ROBOTS_CONFIG_FILENAME,
    ROS_PARAMETERS_KEY,
    STATE_QOS,
    UTF8_ENCODING,
    WORLD_STATE_TOPIC,
    robot_plan_topic,
    robot_status_topic,
)


def _load_site_yaml_defaults() -> dict[str, Any]:
    """Loads default site network parameters from config/robots.yaml when installed."""
    try:
        share_dir = get_package_share_directory(OVERHEAD_ROS_PACKAGE_NAME)
    except (PackageNotFoundError, OSError):
        return {}

    config_path = Path(share_dir) / CONFIG_DIR_NAME / ROBOTS_CONFIG_FILENAME
    if not config_path.is_file():
        return {}

    try:
        raw = yaml.safe_load(config_path.read_text(encoding=UTF8_ENCODING))
    except (OSError, yaml.YAMLError):
        return {}

    if not isinstance(raw, dict):
        return {}

    node_section = raw.get(ROBOT_LINK_CHECK_NODE_NAME)
    if not isinstance(node_section, dict):
        return {}

    params = node_section.get(ROS_PARAMETERS_KEY)
    return params if isinstance(params, dict) else {}


def _fetch_robot_web_status(
    robot_name: str,
    host: str,
    port: int,
    *,
    logger: Any,
) -> RobotWebStatus | None:
    """Performs a blocking HTTP GET to /api/status inside a worker thread."""
    url = ROBOT_WEB_URL_FMT.format(
        host,
        port,
        ROBOT_WEB_STATUS_PATH,
    )
    try:
        with urllib.request.urlopen(
            url,
            timeout=ROBOT_WEB_REQUEST_TIMEOUT_S,
        ) as response:
            body_text = response.read().decode(UTF8_ENCODING)
        payload = json.loads(body_text)
        return parse_robot_web_status(payload)
    except (
        urllib.error.URLError,
        TimeoutError,
        OSError,
        UnicodeDecodeError,
        json.JSONDecodeError,
        ValueError,
    ) as exc:
        logger.warning(
            LOG_ROBOT_LINK_WEB_FETCH_FAILED_FMT.format(
                robot_name,
                url,
                exc,
            ),
        )
        return None


def _compute_topic_age_s(
    last_rx_s: float | None,
    *,
    now_s: float,
) -> float | None:
    """Computes elapsed seconds since last_rx_s on the local monotonic clock."""
    if last_rx_s is None:
        return None
    return max(
        0.0,
        now_s - last_rx_s,
    )


class RobotLinkCheckNode(Node):
    """One-shot diagnostic node checking ROS discovery, vision, and robot web endpoints."""

    def __init__(self) -> None:
        """Initializes parameters, monotonic timestamp caches, and topic subscriptions."""
        super().__init__(ROBOT_LINK_CHECK_NODE_NAME)

        self._declare_parameters()
        self._load_parameters()

        self._state_lock = threading.Lock()
        self._last_plan_rx_s: dict[str, float] = {}
        self._last_status_rx_s: dict[str, float] = {}
        self._located_robots: frozenset[str] = frozenset()
        self._subscription_handles: list[Any] = []

        self._create_subscriptions()

    def _declare_parameters(self) -> None:
        """Declares parameters using robots.yaml values over package constant fallbacks."""
        site_defaults = _load_site_yaml_defaults()
        parameter_defaults: tuple[tuple[str, Any], ...] = (
            (
                PARAM_ROBOT_NAMES,
                site_defaults.get(
                    PARAM_ROBOT_NAMES,
                    list(DEFAULT_ROBOT_NAMES),
                ),
            ),
            (
                PARAM_ROBOT_HOSTS,
                site_defaults.get(
                    PARAM_ROBOT_HOSTS,
                    list(DEFAULT_ROBOT_HOSTS),
                ),
            ),
            (
                PARAM_ROBOT_WEB_PORT,
                site_defaults.get(
                    PARAM_ROBOT_WEB_PORT,
                    DEFAULT_ROBOT_WEB_PORT,
                ),
            ),
            (
                PARAM_LISTEN_S,
                site_defaults.get(
                    PARAM_LISTEN_S,
                    DEFAULT_ROBOT_LINK_LISTEN_S,
                ),
            ),
        )
        for param_name, default_value in parameter_defaults:
            self.declare_parameter(
                param_name,
                default_value,
            )

    def _load_parameters(self) -> None:
        """Loads and validates configured robot names, hosts, web port, and listen window."""
        self._robot_names = tuple(str(name) for name in self.get_parameter(PARAM_ROBOT_NAMES).value)
        self._robot_hosts = tuple(str(host) for host in self.get_parameter(PARAM_ROBOT_HOSTS).value)
        self._robot_web_port = int(self.get_parameter(PARAM_ROBOT_WEB_PORT).value)
        self._listen_s = float(self.get_parameter(PARAM_LISTEN_S).value)

        if not self._robot_names:
            raise ValueError(ERR_ROBOT_LINK_EMPTY_ROBOTS)

        if len(self._robot_names) != len(self._robot_hosts):
            raise ValueError(
                ERR_ROBOT_LINK_HOST_MISMATCH_FMT.format(
                    len(self._robot_names),
                    len(self._robot_hosts),
                ),
            )

        is_valid_port = MIN_PORT_NUMBER <= self._robot_web_port <= MAX_PORT_NUMBER
        if not is_valid_port:
            raise ValueError(
                ERR_ROBOT_LINK_INVALID_PORT_FMT.format(
                    MIN_PORT_NUMBER,
                    MAX_PORT_NUMBER,
                    self._robot_web_port,
                ),
            )

        if self._listen_s <= 0.0:
            raise ValueError(
                ERR_ROBOT_LINK_INVALID_LISTEN_S_FMT.format(self._listen_s),
            )

    def _create_subscriptions(self) -> None:
        """Subscribes to /world/state and each robot's /plan and /status topics."""
        self._subscription_handles.append(
            self.create_subscription(
                WorldState,
                WORLD_STATE_TOPIC,
                self._on_world_state,
                STATE_QOS,
            ),
        )
        for robot_name in self._robot_names:
            self._subscription_handles.append(
                self.create_subscription(
                    RobotPlan,
                    robot_plan_topic(robot_name),
                    self._make_plan_callback(robot_name),
                    PLAN_QOS,
                ),
            )
            self._subscription_handles.append(
                self.create_subscription(
                    RobotStatus,
                    robot_status_topic(robot_name),
                    self._make_status_callback(robot_name),
                    PLAN_QOS,
                ),
            )

    def _on_world_state(self, msg: WorldState) -> None:
        """Updates the set of robot names currently located in /world/state."""
        located = located_robot_names_from_world_state(msg)
        with self._state_lock:
            self._located_robots = located

    def _make_plan_callback(self, robot_name: str) -> Callable[[RobotPlan], None]:
        """Creates a subscription callback recording /<robot>/plan arrival time."""

        def _on_plan(_msg: RobotPlan) -> None:
            rx_s = time.monotonic()
            with self._state_lock:
                self._last_plan_rx_s[robot_name] = rx_s

        return _on_plan

    def _make_status_callback(self, robot_name: str) -> Callable[[RobotStatus], None]:
        """Creates a subscription callback recording /<robot>/status arrival time."""

        def _on_status(_msg: RobotStatus) -> None:
            rx_s = time.monotonic()
            with self._state_lock:
                self._last_status_rx_s[robot_name] = rx_s

        return _on_status

    def _start_web_workers(
        self,
        web_results: dict[str, RobotWebStatus | None],
        results_lock: threading.Lock,
    ) -> list[threading.Thread]:
        """Spawns one daemon thread per robot to query /api/status without blocking ROS."""
        threads: list[threading.Thread] = []
        logger = self.get_logger()

        for robot_name, host in zip(
            self._robot_names,
            self._robot_hosts,
            strict=True,
        ):

            def _worker(
                target_robot: str = robot_name,
                target_host: str = host,
            ) -> None:
                status = _fetch_robot_web_status(
                    target_robot,
                    target_host,
                    self._robot_web_port,
                    logger=logger,
                )
                with results_lock:
                    web_results[target_robot] = status

            thread = threading.Thread(
                target=_worker,
                daemon=True,
            )
            thread.start()
            threads.append(thread)

        return threads

    def _collect_observations(self) -> dict[str, RobotWebStatus | None]:
        """Spins ROS callbacks for listen_s seconds while querying robot web APIs."""
        executor = SingleThreadedExecutor()
        executor.add_node(self)

        start_s = time.monotonic()
        deadline_s = start_s + self._listen_s
        web_start_s = max(
            start_s,
            deadline_s - ROBOT_WEB_REQUEST_TIMEOUT_S,
        )
        hard_cap_s = deadline_s + ROBOT_WEB_REQUEST_TIMEOUT_S + ROBOT_LINK_THREAD_JOIN_TIMEOUT_S

        web_results: dict[str, RobotWebStatus | None] = {}
        results_lock = threading.Lock()
        web_threads: list[threading.Thread] = []
        has_started_web = False

        try:
            while rclpy.ok():
                now_s = time.monotonic()
                if not has_started_web and now_s >= web_start_s:
                    has_started_web = True
                    web_threads = self._start_web_workers(
                        web_results,
                        results_lock,
                    )

                is_listen_done = now_s >= deadline_s
                is_web_done = has_started_web and not any(
                    thread.is_alive() for thread in web_threads
                )
                is_finished = (is_listen_done and is_web_done) or now_s >= hard_cap_s
                if is_finished:
                    break

                executor.spin_once(timeout_sec=ROBOT_LINK_POLL_PERIOD_S)
        finally:
            executor.shutdown()

        for thread in web_threads:
            thread.join(timeout=ROBOT_LINK_THREAD_JOIN_TIMEOUT_S)

        with results_lock:
            return dict(web_results)

    def _evaluate_reports(
        self,
        web_results: dict[str, RobotWebStatus | None],
        *,
        eval_now_s: float,
    ) -> dict[str, RobotLinkReport]:
        """Evaluates RobotLinkReport for every configured robot."""
        with self._state_lock:
            plan_rx = dict(self._last_plan_rx_s)
            status_rx = dict(self._last_status_rx_s)
            located_robots = self._located_robots

        reports: dict[str, RobotLinkReport] = {}
        for robot_name in self._robot_names:
            plan_age_s = _compute_topic_age_s(
                plan_rx.get(robot_name),
                now_s=eval_now_s,
            )
            status_age_s = _compute_topic_age_s(
                status_rx.get(robot_name),
                now_s=eval_now_s,
            )
            reports[robot_name] = evaluate_robot_link(
                web_results.get(robot_name),
                plan_age_s,
                status_age_s,
                is_robot_located=robot_name in located_robots,
            )
        return reports

    def _log_reports(self, reports: dict[str, RobotLinkReport]) -> int:
        """Logs one summary line per robot and returns 0 only if every robot is OK."""
        for robot_name in self._robot_names:
            report = reports[robot_name]
            line = format_robot_link_report(
                robot_name,
                report,
            )
            if report.is_ok:
                self.get_logger().info(line)
            else:
                self.get_logger().warning(line)

        return 0 if all(report.is_ok for report in reports.values()) else 1

    def run_check(self) -> int:
        """Listens to ROS topics, polls robot web status in worker threads, and logs reports.

        Returns:
            Exit code 0 when every configured robot reports OK, otherwise 1.
        """
        self.get_logger().info(
            LOG_ROBOT_LINK_START_FMT.format(
                list(self._robot_names),
                self._listen_s,
            ),
        )
        web_snapshot = self._collect_observations()
        reports = self._evaluate_reports(
            web_snapshot,
            eval_now_s=time.monotonic(),
        )
        return self._log_reports(reports)


def main(args: list[str] | None = None) -> int:
    """Entry point for the one-shot robot_link_check CLI."""
    rclpy.init(args=args)
    node = RobotLinkCheckNode()
    try:
        return node.run_check()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
