"""ROS 2 node capturing the overhead camera and publishing world state."""

import time
from typing import Any

import cv2
import numpy as np
import rclpy
from multirobot_interfaces.msg import RobotState
from overhead_core import virtual_sensors
from overhead_core.calibration import validate_arena_quad
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CALIBRATION_RETRY_S,
    CAM_HEIGHT_M,
    CAMERA_RETRY_S,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_CAPTURE_HEIGHT_PX,
    DEFAULT_CAPTURE_WIDTH_PX,
    DEFAULT_CUBE_HSV_RANGES,
    DEFAULT_IMAGE_JPEG_QUALITY,
    DEFAULT_IMAGE_RATE_HZ,
    DEFAULT_MIN_AREA_PX,
    DEFAULT_RATE_HZ,
    DEFAULT_RECTIFY_PX_PER_M,
    DEFAULT_ROBOT_HSV_RANGES,
    DEFAULT_ROBOT_NAMES,
    DEFAULT_VIDEO_SOURCE,
    DEFAULT_ZONE_RECT,
    HSV_CHANNEL_COUNT,
    IMAGE_COMPRESSION_FORMAT,
    JPEG_EXTENSION,
    LOG_CALIBRATION_FAILED_FMT,
    LOG_CALIBRATION_INVALID_QUAD,
    LOG_CALIBRATION_SUCCESS,
    LOG_CAMERA_OPEN_RETRY,
    LOG_CAMERA_OPENED,
    LOG_IMAGE_ENCODE_FAILED,
    LOG_NO_FRAME_DURING_CALIBRATION,
    LOG_NODE_INITIALIZED,
    LOG_STATE_TRANSITION_FMT,
    MIN_IMAGE_TIMER_RATE_HZ,
    MIN_MAIN_TIMER_RATE_HZ,
    MSG_RECALIBRATE_FAILED,
    MSG_RECALIBRATE_SUCCESS,
    MSG_RECALIBRATE_UNAVAILABLE,
    NEAR_RED_DISTANCE_M,
    NO_FRAME_TIMEOUT_S,
    RECTIFY_MARGIN_M,
    ROBOT_HOLD_S,
    WARN_THROTTLE_PERIOD_S,
    CubeColor,
    ObstacleKind,
    RobotName,
)
from overhead_core.corridor import corridor_free
from overhead_core.frame_source import (
    FrameSource,
    make_frame_source,
)
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    rectify,
)
from overhead_core.robot_hold import RobotHoldFilter
from overhead_core.state_machine import (
    OverheadEvent,
    OverheadState,
    can_publish,
    next_state,
)
from overhead_core.tracker import CubeTracker
from overhead_core.types import (
    Obstacle,
    RobotPose,
    SafetyResult,
)
from overhead_core.velocity import VelocityEstimator
from overhead_core.world_frame import (
    WorldFrameConfig,
    detect_robot_poses,
    process_cubes,
)
from overhead_core.world_safety import compute_safety
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import (
    Header,
    String,
)
from std_srvs.srv import Trigger
from tsa_interfaces.msg import (
    VirtualSensors,
    WorldState,
)
from tsa_interfaces.srv import (
    CorridorFree,
    GetWorld,
)

from overhead_ros.conversions import (
    to_cube_observation_msg,
    to_obstacle_msg,
    to_robot_observation_msg,
    to_robot_obstacles_msg,
    to_robot_sensors_msg,
    to_virtual_sensors_msg,
    to_world_state_msg,
)
from overhead_ros.ros_names import (
    ARENA_FRAME_ID,
    IMAGE_QOS,
    OVERHEAD_NODE_NAME,
    OVERHEAD_RECALIBRATE_SERVICE,
    PARAM_ARENA_HEIGHT_M,
    PARAM_ARENA_WIDTH_M,
    PARAM_CALIBRATION_RETRY_S,
    PARAM_CAM_HEIGHT_M,
    PARAM_CAMERA_CENTER_WORLD,
    PARAM_CAMERA_INDEX,
    PARAM_HEIGHT,
    PARAM_HSV_BLUE,
    PARAM_HSV_RED_1,
    PARAM_HSV_RED_2,
    PARAM_HSV_ROBOT_FERB,
    PARAM_HSV_ROBOT_PHINEAS,
    PARAM_HSV_YELLOW,
    PARAM_IMAGE_RATE_HZ,
    PARAM_MIN_AREA_PX,
    PARAM_NEAR_RED_M,
    PARAM_RATE_HZ,
    PARAM_ROBOTS,
    PARAM_VIDEO_SOURCE,
    PARAM_WIDTH,
    PARAM_ZONE_RECT,
    PERRY_STATE_QOS,
    PERRY_STATE_TOPIC,
    SENSOR_QOS,
    STATE_QOS,
    WORLD_CORRIDOR_FREE_SERVICE,
    WORLD_GET_SERVICE,
    WORLD_IMAGE_TOPIC,
    WORLD_SENSORS_TOPIC,
    WORLD_STATE_TOPIC,
    ZONE_OCCUPANT_QOS,
    ZONE_OCCUPANT_TOPIC,
)
from overhead_ros.safety_inputs import SafetyInputs


def _hsv_param_default(
    bounds: tuple[tuple[int, int, int], tuple[int, int, int]],
) -> list[int]:
    """Flattens an HSV (lower, upper) pair into [h, s, v, h, s, v].

    ROS 2 parameters cannot hold nested arrays, so the pair is stored flat.
    """
    return [
        *bounds[0],
        *bounds[1],
    ]


class OverheadNode(Node):
    """Thin ROS 2 adapter running the overhead state machine and pure vision core."""

    def __init__(self) -> None:
        """Initializes parameters, frame source, state machine, and ROS interfaces."""
        super().__init__(OVERHEAD_NODE_NAME)

        self._declare_parameters()
        self._load_parameters()

        self._state = OverheadState.STARTING
        self._homography: np.ndarray | None = None
        self._latest_frame: np.ndarray | None = None
        self._latest_state = WorldState()
        self._latest_sensors = VirtualSensors()
        self._prev_in_zone: set[str] = set()
        self._prev_is_all_seen = True

        self._tracker = CubeTracker()
        self._robot_hold = RobotHoldFilter(
            self._robots,
            hold_s=ROBOT_HOLD_S,
        )
        self._velocity_estimator = VelocityEstimator()
        self._safety_inputs = SafetyInputs(
            arena_width_m=self._arena_width_m,
            arena_height_m=self._arena_height_m,
            zone_rect=self._zone_rect,
        )
        self._latest_red_obstacles: list[Obstacle] = []

        self._last_frame_s: float | None = None
        self._last_camera_retry_s: float = -CAMERA_RETRY_S
        self._last_calibration_try_s: float = -self._calibration_retry_s
        self._last_warn_s: float = -WARN_THROTTLE_PERIOD_S

        self._frame_source: FrameSource = make_frame_source(
            self._video_source,
            camera_index=self._camera_index,
            width_px=self._width,
            height_px=self._height,
        )

        self._create_ros_interfaces()
        self._startup_open_and_calibrate()
        self.get_logger().info(LOG_NODE_INITIALIZED)

    def _create_ros_interfaces(self) -> None:
        """Creates publishers, services, and periodic timers."""
        self._pub_state = self.create_publisher(
            WorldState,
            WORLD_STATE_TOPIC,
            STATE_QOS,
        )
        self._pub_sensors = self.create_publisher(
            VirtualSensors,
            WORLD_SENSORS_TOPIC,
            SENSOR_QOS,
        )
        self._pub_image = self.create_publisher(
            CompressedImage,
            WORLD_IMAGE_TOPIC,
            IMAGE_QOS,
        )

        self._srv_recalibrate = self.create_service(
            Trigger,
            OVERHEAD_RECALIBRATE_SERVICE,
            self._on_recalibrate,
        )
        self._srv_get_world = self.create_service(
            GetWorld,
            WORLD_GET_SERVICE,
            self._on_get_world,
        )
        self._srv_corridor_free = self.create_service(
            CorridorFree,
            WORLD_CORRIDOR_FREE_SERVICE,
            self._on_corridor_free,
        )

        self._sub_zone_occupant = self.create_subscription(
            String,
            ZONE_OCCUPANT_TOPIC,
            self._safety_inputs.on_zone_occupant,
            ZONE_OCCUPANT_QOS,
        )
        self._sub_perry_state = self.create_subscription(
            RobotState,
            PERRY_STATE_TOPIC,
            self._safety_inputs.on_perry_state,
            PERRY_STATE_QOS,
        )

        main_timer_period_s = 1.0 / max(
            MIN_MAIN_TIMER_RATE_HZ,
            self._rate_hz,
        )
        self._timer_main = self.create_timer(
            main_timer_period_s,
            self._on_main_timer,
        )

        image_timer_period_s = 1.0 / max(
            MIN_IMAGE_TIMER_RATE_HZ,
            self._image_rate_hz,
        )
        self._timer_image = self.create_timer(
            image_timer_period_s,
            self._on_image_timer,
        )

    def _declare_parameters(self) -> None:
        """Declares all node ROS parameters using package constants as defaults."""
        self.declare_parameter(
            PARAM_CAMERA_INDEX,
            DEFAULT_CAMERA_INDEX,
        )
        self.declare_parameter(
            PARAM_VIDEO_SOURCE,
            DEFAULT_VIDEO_SOURCE,
        )
        self.declare_parameter(
            PARAM_WIDTH,
            DEFAULT_CAPTURE_WIDTH_PX,
        )
        self.declare_parameter(
            PARAM_HEIGHT,
            DEFAULT_CAPTURE_HEIGHT_PX,
        )
        self.declare_parameter(
            PARAM_RATE_HZ,
            DEFAULT_RATE_HZ,
        )
        self.declare_parameter(
            PARAM_CALIBRATION_RETRY_S,
            CALIBRATION_RETRY_S,
        )
        self.declare_parameter(
            PARAM_ARENA_WIDTH_M,
            ARENA_WIDTH_M,
        )
        self.declare_parameter(
            PARAM_ARENA_HEIGHT_M,
            ARENA_HEIGHT_M,
        )
        self.declare_parameter(
            PARAM_CAM_HEIGHT_M,
            CAM_HEIGHT_M,
        )
        self.declare_parameter(
            PARAM_CAMERA_CENTER_WORLD,
            list(DEFAULT_CAMERA_CENTER_WORLD),
        )
        self.declare_parameter(
            PARAM_ZONE_RECT,
            list(DEFAULT_ZONE_RECT),
        )
        self.declare_parameter(
            PARAM_ROBOTS,
            list(DEFAULT_ROBOT_NAMES),
        )
        self.declare_parameter(
            PARAM_IMAGE_RATE_HZ,
            DEFAULT_IMAGE_RATE_HZ,
        )
        self.declare_parameter(
            PARAM_MIN_AREA_PX,
            DEFAULT_MIN_AREA_PX,
        )
        self.declare_parameter(
            PARAM_NEAR_RED_M,
            NEAR_RED_DISTANCE_M,
        )

        self.declare_parameter(
            PARAM_HSV_BLUE,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.BLUE.value][0]),
        )
        self.declare_parameter(
            PARAM_HSV_YELLOW,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.YELLOW.value][0]),
        )
        self.declare_parameter(
            PARAM_HSV_RED_1,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.RED.value][0]),
        )
        self.declare_parameter(
            PARAM_HSV_RED_2,
            _hsv_param_default(DEFAULT_CUBE_HSV_RANGES[CubeColor.RED.value][1]),
        )
        self.declare_parameter(
            PARAM_HSV_ROBOT_PHINEAS,
            _hsv_param_default(DEFAULT_ROBOT_HSV_RANGES[RobotName.PHINEAS.value][0]),
        )
        self.declare_parameter(
            PARAM_HSV_ROBOT_FERB,
            _hsv_param_default(DEFAULT_ROBOT_HSV_RANGES[RobotName.FERB.value][0]),
        )

    def _read_hsv_pair(
        self,
        param_name: str,
    ) -> tuple[tuple[int, ...], tuple[int, ...]]:
        """Reads a flat [h, s, v, h, s, v] parameter as a pair of integer tuples."""
        raw = [int(v) for v in self.get_parameter(param_name).value]
        return (
            tuple(raw[:HSV_CHANNEL_COUNT]),
            tuple(raw[HSV_CHANNEL_COUNT:]),
        )

    def _load_parameters(self) -> None:
        """Loads parameter values and constructs the pure WorldFrameConfig."""
        self._camera_index = int(self.get_parameter(PARAM_CAMERA_INDEX).value)
        self._video_source = str(self.get_parameter(PARAM_VIDEO_SOURCE).value)
        self._width = int(self.get_parameter(PARAM_WIDTH).value)
        self._height = int(self.get_parameter(PARAM_HEIGHT).value)
        self._rate_hz = float(self.get_parameter(PARAM_RATE_HZ).value)
        self._calibration_retry_s = float(self.get_parameter(PARAM_CALIBRATION_RETRY_S).value)
        self._arena_width_m = float(self.get_parameter(PARAM_ARENA_WIDTH_M).value)
        self._arena_height_m = float(self.get_parameter(PARAM_ARENA_HEIGHT_M).value)
        self._cam_height_m = float(self.get_parameter(PARAM_CAM_HEIGHT_M).value)
        center_raw = self.get_parameter(PARAM_CAMERA_CENTER_WORLD).value
        self._camera_center_world = (
            float(center_raw[0]),
            float(center_raw[1]),
        )
        zone_raw = self.get_parameter(PARAM_ZONE_RECT).value
        self._zone_rect = (
            float(zone_raw[0]),
            float(zone_raw[1]),
            float(zone_raw[2]),
            float(zone_raw[3]),
        )
        self._robots = tuple(str(name) for name in self.get_parameter(PARAM_ROBOTS).value)
        self._image_rate_hz = float(self.get_parameter(PARAM_IMAGE_RATE_HZ).value)
        self._min_area_px = int(self.get_parameter(PARAM_MIN_AREA_PX).value)
        self._near_red_m = float(self.get_parameter(PARAM_NEAR_RED_M).value)

        hsv_ranges: dict[str, list[tuple[Any, Any]]] = {
            CubeColor.BLUE.value: [self._read_hsv_pair(PARAM_HSV_BLUE)],
            CubeColor.YELLOW.value: [self._read_hsv_pair(PARAM_HSV_YELLOW)],
            CubeColor.RED.value: [
                self._read_hsv_pair(PARAM_HSV_RED_1),
                self._read_hsv_pair(PARAM_HSV_RED_2),
            ],
        }
        robot_hsv_ranges: dict[str, list[tuple[Any, Any]]] = {
            RobotName.PHINEAS.value: [self._read_hsv_pair(PARAM_HSV_ROBOT_PHINEAS)],
            RobotName.FERB.value: [self._read_hsv_pair(PARAM_HSV_ROBOT_FERB)],
        }

        self._world_config = WorldFrameConfig(
            hsv_ranges=hsv_ranges,
            robot_hsv_ranges=robot_hsv_ranges,
            min_area_px=self._min_area_px,
            camera_center_world=self._camera_center_world,
            cam_height_m=self._cam_height_m,
            arena_width_m=self._arena_width_m,
            arena_height_m=self._arena_height_m,
            robot_names=self._robots,
        )

    def _log_warn_throttled(
        self,
        message: str,
        now_s: float,
    ) -> None:
        """Logs a warning at most once per WARN_THROTTLE_PERIOD_S."""
        if now_s - self._last_warn_s < WARN_THROTTLE_PERIOD_S:
            return
        self._last_warn_s = now_s
        self.get_logger().warn(message)

    def _apply_event(
        self,
        event: OverheadEvent,
    ) -> None:
        """Drives the state machine and logs each actual transition exactly once."""
        prev_state = self._state
        new_state = next_state(
            prev_state,
            event,
        )
        if new_state == prev_state:
            return

        self._state = new_state
        if new_state == OverheadState.RUNNING:
            self._prev_is_all_seen = True

        self.get_logger().info(
            LOG_STATE_TRANSITION_FMT.format(
                prev_state.value,
                new_state.value,
                event.value,
            )
        )

    def _startup_open_and_calibrate(self) -> None:
        """Checks initial frame source availability and runs first calibration."""
        now_s = time.monotonic()
        if not self._frame_source.is_open():
            self._last_camera_retry_s = now_s
            self._log_warn_throttled(
                LOG_CAMERA_OPEN_RETRY,
                now_s,
            )
            return

        self.get_logger().info(LOG_CAMERA_OPENED)
        self._apply_event(OverheadEvent.CAMERA_OPENED)
        self._last_calibration_try_s = now_s
        self._attempt_calibration(now_s)

    def _attempt_calibration(
        self,
        now_s: float,
    ) -> bool:
        """Detects and validates arena corners and computes floor homography."""
        frame = self._frame_source.read()
        if frame is None:
            self._log_warn_throttled(
                LOG_NO_FRAME_DURING_CALIBRATION,
                now_s,
            )
            self._apply_event(OverheadEvent.CORNERS_NOT_FOUND)
            return False

        self._latest_frame = frame
        try:
            corners = detect_arena_corners(frame)
        except (ValueError, cv2.error) as exc:
            self._log_warn_throttled(
                LOG_CALIBRATION_FAILED_FMT.format(exc),
                now_s,
            )
            self._apply_event(OverheadEvent.CORNERS_NOT_FOUND)
            return False

        height_px, width_px = frame.shape[:2]
        is_valid = validate_arena_quad(
            corners,
            image_width_px=width_px,
            image_height_px=height_px,
            arena_width_m=self._arena_width_m,
            arena_height_m=self._arena_height_m,
        )
        if not is_valid:
            self._log_warn_throttled(
                LOG_CALIBRATION_INVALID_QUAD,
                now_s,
            )
            self._apply_event(OverheadEvent.CORNERS_NOT_FOUND)
            return False

        self._homography = compute_homography(
            corners,
            self._arena_width_m,
            self._arena_height_m,
        )
        self._last_frame_s = now_s
        self.get_logger().info(LOG_CALIBRATION_SUCCESS)
        self._apply_event(OverheadEvent.CORNERS_FOUND)
        return True

    def _on_recalibrate(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Service callback triggering recalibration and reporting completion status."""
        now_s = time.monotonic()
        self._apply_event(OverheadEvent.RECALIBRATE_REQUESTED)
        if self._state != OverheadState.CALIBRATING:
            response.success = False
            response.message = MSG_RECALIBRATE_UNAVAILABLE
            return response

        self._homography = None
        self._last_calibration_try_s = now_s
        is_ok = self._attempt_calibration(now_s)
        response.success = is_ok
        response.message = MSG_RECALIBRATE_SUCCESS if is_ok else MSG_RECALIBRATE_FAILED
        return response

    def _on_get_world(
        self,
        _request: GetWorld.Request,
        response: GetWorld.Response,
    ) -> GetWorld.Response:
        """Returns the most recently published world state and virtual sensors."""
        response.world = self._latest_state
        response.sensors = self._latest_sensors
        return response

    def _on_corridor_free(
        self,
        request: CorridorFree.Request,
        response: CorridorFree.Response,
    ) -> CorridorFree.Response:
        """Checks whether a straight corridor is free of the latest red-cube circles."""
        result = corridor_free(
            (request.ax, request.ay),
            (request.gx, request.gy),
            width_m=request.width,
            obstacles=self._latest_red_obstacles,
        )
        response.free = result.is_free
        response.blocking_ids = list(result.blocking_ids)
        return response

    def _handle_starting_state(
        self,
        now_s: float,
    ) -> None:
        """Retries opening the frame source every CAMERA_RETRY_S while STARTING."""
        if now_s - self._last_camera_retry_s < CAMERA_RETRY_S:
            return
        self._last_camera_retry_s = now_s
        if not self._frame_source.reopen():
            self._log_warn_throttled(
                LOG_CAMERA_OPEN_RETRY,
                now_s,
            )
            return

        self.get_logger().info(LOG_CAMERA_OPENED)
        self._apply_event(OverheadEvent.CAMERA_OPENED)
        self._last_calibration_try_s = now_s
        self._attempt_calibration(now_s)

    def _handle_no_image_state(
        self,
        now_s: float,
    ) -> np.ndarray | None:
        """Reads a frame or retries reopening every CAMERA_RETRY_S while NO_IMAGE."""
        frame = self._frame_source.read()
        if frame is None and now_s - self._last_camera_retry_s >= CAMERA_RETRY_S:
            self._last_camera_retry_s = now_s
            if self._frame_source.reopen():
                frame = self._frame_source.read()
            else:
                self._log_warn_throttled(
                    LOG_CAMERA_OPEN_RETRY,
                    now_s,
                )

        if frame is None:
            return None

        self._last_frame_s = now_s
        self._latest_frame = frame
        self._apply_event(OverheadEvent.FRAME_BACK)
        return frame

    def _handle_calibrating_state(
        self,
        now_s: float,
    ) -> None:
        """Retries arena corner calibration every calibration_retry_s while CALIBRATING."""
        if now_s - self._last_calibration_try_s < self._calibration_retry_s:
            return
        self._last_calibration_try_s = now_s
        self._attempt_calibration(now_s)

    def _update_robot_visibility_state(
        self,
        is_all_seen: bool,
    ) -> None:
        """Transitions between RUNNING and DEGRADED when HoldResult.is_all_seen changes."""
        if is_all_seen == self._prev_is_all_seen:
            return
        self._prev_is_all_seen = is_all_seen
        event = OverheadEvent.ALL_ROBOTS_SEEN if is_all_seen else OverheadEvent.ROBOT_LOST
        self._apply_event(event)

    def _publish_state_and_sensors(
        self,
        tracked_cubes: list[tuple[int, str, float, float, float]],
        *,
        robot_poses: list[RobotPose],
        vs_result: dict[str, Any],
        safety_result: SafetyResult,
    ) -> None:
        """Constructs and publishes WorldState and VirtualSensors messages."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = ARENA_FRAME_ID

        cube_msgs = [
            to_cube_observation_msg(
                cid,
                ccolor,
                cx,
                y=cy,
                confidence=cconf,
            )
            for cid, ccolor, cx, cy, cconf in tracked_cubes
        ]
        robot_msgs = [
            to_robot_observation_msg(
                pose.name,
                pose.x,
                pose.y,
                theta=pose.theta,
                confidence=pose.confidence,
            )
            for pose in robot_poses
        ]
        world_state_msg = to_world_state_msg(
            header,
            cube_msgs,
            robot_msgs,
        )
        self._pub_state.publish(world_state_msg)
        self._latest_state = world_state_msg

        per_robot_msgs = [
            to_robot_sensors_msg(
                rname,
                rinfo[0],
                rinfo[1],
                events=rinfo[2],
            )
            for rname, rinfo in vs_result["per_robot"].items()
        ]
        obstacle_msgs = [to_obstacle_msg(circle) for circle in safety_result.circles]
        per_robot_obstacle_msgs = [
            to_robot_obstacles_msg(
                robot_name,
                robot_safety,
            )
            for robot_name, robot_safety in safety_result.per_robot.items()
        ]
        virtual_sensors_msg = to_virtual_sensors_msg(
            header,
            vs_result["robot_distance"],
            vs_result["robots_in_zone"],
            per_robot=per_robot_msgs,
            obstacles=obstacle_msgs,
            per_robot_obstacles=per_robot_obstacle_msgs,
        )
        self._pub_sensors.publish(virtual_sensors_msg)
        self._latest_sensors = virtual_sensors_msg

    def _process_and_publish(
        self,
        frame: np.ndarray,
        now_s: float,
    ) -> None:
        """Runs cube/robot detection, hold filtering, virtual sensors, and publishing."""
        if self._homography is None:
            return

        tracked_cubes = process_cubes(
            frame,
            homography=self._homography,
            tracker=self._tracker,
            now_s=now_s,
            config=self._world_config,
        )
        seen_robots = detect_robot_poses(
            frame,
            homography=self._homography,
            config=self._world_config,
        )
        hold_result = self._robot_hold.update(
            seen_robots,
            now_s,
        )
        self._update_robot_visibility_state(hold_result.is_all_seen)

        if not can_publish(self._state):
            return

        robots_for_sensors = {pose.name: (pose.x, pose.y) for pose in hold_result.poses}
        cubes_for_sensors = [(t[0], t[1], t[2], t[3]) for t in tracked_cubes]
        vs_result = virtual_sensors.compute(
            robots_for_sensors,
            cubes_for_sensors,
            zone_rect=self._zone_rect,
            prev_in_zone=self._prev_in_zone,
            near_red_m=self._near_red_m,
        )
        self._prev_in_zone = vs_result["in_zone"]

        seen_positions = {name: (pose.x, pose.y) for name, pose in seen_robots.items()}
        velocities = self._velocity_estimator.update(
            seen_positions,
            now_s,
        )
        # TODO(overhead): populate carrying_robots once the robots' own state topics are subscribed.
        safety_config = self._safety_inputs.build_config(
            now_s,
            carrying_robots=frozenset(),
        )
        safety_result = compute_safety(
            hold_result.poses,
            tracked_cubes,
            velocities=velocities,
            config=safety_config,
        )
        self._latest_red_obstacles = [
            circle for circle in safety_result.circles if circle.kind == ObstacleKind.RED_CUBE
        ]

        self._publish_state_and_sensors(
            tracked_cubes,
            robot_poses=hold_result.poses,
            vs_result=vs_result,
            safety_result=safety_result,
        )

    def _on_main_timer(self) -> None:
        """Periodic loop driving the overhead state machine and vision publication."""
        now_s = time.monotonic()

        if self._state == OverheadState.STARTING:
            self._handle_starting_state(now_s)
            return

        if self._state == OverheadState.CALIBRATING:
            self._handle_calibrating_state(now_s)
            return

        if self._state == OverheadState.NO_IMAGE:
            frame = self._handle_no_image_state(now_s)
            if frame is not None:
                self._process_and_publish(
                    frame,
                    now_s,
                )
            return

        frame = self._frame_source.read()
        if frame is None:
            if self._last_frame_s is None:
                self._last_frame_s = now_s
            if now_s - self._last_frame_s >= NO_FRAME_TIMEOUT_S:
                self._last_camera_retry_s = now_s
                self._apply_event(OverheadEvent.FRAME_TIMEOUT)
            return

        self._last_frame_s = now_s
        self._latest_frame = frame
        self._process_and_publish(
            frame,
            now_s,
        )

    def _on_image_timer(self) -> None:
        """Generates and publishes the rectified arena view whenever frame and homography exist."""
        if self._latest_frame is None or self._homography is None:
            return

        rectified = rectify(
            self._latest_frame,
            self._homography,
            DEFAULT_RECTIFY_PX_PER_M,
            arena_width_m=self._arena_width_m,
            arena_height_m=self._arena_height_m,
            margin_m=RECTIFY_MARGIN_M,
        )

        encode_success, jpeg_buffer = cv2.imencode(
            JPEG_EXTENSION,
            rectified,
            [int(cv2.IMWRITE_JPEG_QUALITY), DEFAULT_IMAGE_JPEG_QUALITY],
        )
        if not encode_success:
            self._log_warn_throttled(
                LOG_IMAGE_ENCODE_FAILED,
                time.monotonic(),
            )
            return

        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = ARENA_FRAME_ID
        img_msg.format = IMAGE_COMPRESSION_FORMAT
        img_msg.data = jpeg_buffer.tobytes()

        self._pub_image.publish(img_msg)

    def destroy_node(self) -> bool:
        """Releases camera resources before destroying the ROS node."""
        release_fn = getattr(
            self._frame_source,
            "release",
            None,
        )
        if callable(release_fn):
            release_fn()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Entry point for overhead_node."""
    rclpy.init(args=args)
    node = OverheadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
