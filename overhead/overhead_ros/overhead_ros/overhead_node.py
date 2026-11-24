"""ROS 2 node capturing the overhead camera and publishing world state."""

import time

import numpy as np
import rclpy
from multirobot_interfaces.msg import RobotState
from overhead_core import constants as const
from overhead_core.camera_session import (
    CameraSession,
    CameraStepResult,
)
from overhead_core.corridor import corridor_free
from overhead_core.frame_source import make_frame_source
from overhead_core.state_machine import (
    OverheadEvent,
    OverheadState,
    can_publish,
    next_state,
)
from overhead_core.types import Obstacle
from overhead_core.world_pipeline import WorldPipeline
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tsa_interfaces.srv import (
    CorridorFree,
    GetWorld,
)

from overhead_ros import ros_names
from overhead_ros.node_parameters import (
    declare_overhead_parameters,
    load_overhead_parameters,
)
from overhead_ros.safety_inputs import SafetyInputs
from overhead_ros.world_publishers import WorldPublishers


class OverheadNode(Node):
    """Thin ROS 2 adapter running the overhead state machine and pure vision core."""

    def __init__(self) -> None:
        """Initializes parameters, camera session, vision pipeline, and ROS interfaces."""
        super().__init__(ros_names.OVERHEAD_NODE_NAME)

        declare_overhead_parameters(self)
        self._params = load_overhead_parameters(self)

        self._state = OverheadState.STARTING
        self._prev_is_all_seen = True
        self._latest_red_obstacles: list[Obstacle] = []
        self._last_warn_s: float = -const.WARN_THROTTLE_PERIOD_S

        self._pipeline = WorldPipeline(
            self._params.world_config,
            zone_rect=self._params.zone_rect,
            near_red_m=self._params.near_red_m,
            hold_s=const.ROBOT_HOLD_S,
        )
        self._safety_inputs = SafetyInputs(
            arena_width_m=self._params.arena_width_m,
            arena_height_m=self._params.arena_height_m,
            zone_rect=self._params.zone_rect,
        )
        frame_source = make_frame_source(
            self._params.video_source,
            camera_index=self._params.camera_index,
            width_px=self._params.width_px,
            height_px=self._params.height_px,
        )
        self._camera = CameraSession(
            frame_source,
            arena_width_m=self._params.arena_width_m,
            arena_height_m=self._params.arena_height_m,
            calibration_retry_s=self._params.calibration_retry_s,
        )

        self._create_ros_interfaces()
        now_s = time.monotonic()
        self._apply_camera_step(
            self._camera.startup(now_s),
            now_s,
        )
        self.get_logger().info(const.LOG_NODE_INITIALIZED)

    def _create_ros_interfaces(self) -> None:
        """Creates publishers, subscriptions, services, and periodic timers."""
        self._publishers = WorldPublishers(self)
        self._sub_zone_occupant = self.create_subscription(
            String,
            ros_names.ZONE_OCCUPANT_TOPIC,
            self._safety_inputs.on_zone_occupant,
            ros_names.ZONE_OCCUPANT_QOS,
        )
        self._sub_perry_state = self.create_subscription(
            RobotState,
            ros_names.PERRY_STATE_TOPIC,
            self._safety_inputs.on_perry_state,
            ros_names.PERRY_STATE_QOS,
        )
        self._create_services_and_timers()

    def _create_services_and_timers(self) -> None:
        """Creates world query/calibration services and vision/image timers."""
        self._srv_recalibrate = self.create_service(
            Trigger,
            ros_names.OVERHEAD_RECALIBRATE_SERVICE,
            self._on_recalibrate,
        )
        self._srv_get_world = self.create_service(
            GetWorld,
            ros_names.WORLD_GET_SERVICE,
            self._on_get_world,
        )
        self._srv_corridor_free = self.create_service(
            CorridorFree,
            ros_names.WORLD_CORRIDOR_FREE_SERVICE,
            self._on_corridor_free,
        )

        main_timer_period_s = 1.0 / max(
            const.MIN_MAIN_TIMER_RATE_HZ,
            self._params.rate_hz,
        )
        self._timer_main = self.create_timer(
            main_timer_period_s,
            self._on_main_timer,
        )

        image_timer_period_s = 1.0 / max(
            const.MIN_IMAGE_TIMER_RATE_HZ,
            self._params.image_rate_hz,
        )
        self._timer_image = self.create_timer(
            image_timer_period_s,
            self._on_image_timer,
        )

    def _log_warn_throttled(
        self,
        message: str,
        now_s: float,
    ) -> None:
        """Logs a warning at most once per WARN_THROTTLE_PERIOD_S."""
        if now_s - self._last_warn_s < const.WARN_THROTTLE_PERIOD_S:
            return
        self._last_warn_s = now_s
        self.get_logger().warning(message)

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
            const.LOG_STATE_TRANSITION_FMT.format(
                prev_state.value,
                new_state.value,
                event.value,
            )
        )

    def _apply_camera_step(
        self,
        step: CameraStepResult,
        now_s: float,
    ) -> None:
        """Logs camera-step messages and dispatches emitted state-machine events."""
        for outcome in step.outcomes:
            if outcome.info_message is not None:
                self.get_logger().info(outcome.info_message)
            if outcome.warning_message is not None:
                self._log_warn_throttled(
                    outcome.warning_message,
                    now_s,
                )
            if outcome.event is not None:
                self._apply_event(outcome.event)

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
            response.message = const.MSG_RECALIBRATE_UNAVAILABLE
            return response

        is_ok, step = self._camera.recalibrate(now_s)
        self._apply_camera_step(
            step,
            now_s,
        )
        response.success = is_ok
        response.message = const.MSG_RECALIBRATE_SUCCESS if is_ok else const.MSG_RECALIBRATE_FAILED
        return response

    def _on_get_world(
        self,
        _request: GetWorld.Request,
        response: GetWorld.Response,
    ) -> GetWorld.Response:
        """Returns the most recently published world state and virtual sensors."""
        response.world = self._publishers.latest_state
        response.sensors = self._publishers.latest_sensors
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

    def _update_robot_visibility_state(
        self,
        is_all_seen: bool,
    ) -> None:
        """Transitions between RUNNING and DEGRADED when robot visibility changes."""
        if is_all_seen == self._prev_is_all_seen:
            return
        self._prev_is_all_seen = is_all_seen
        event = OverheadEvent.ALL_ROBOTS_SEEN if is_all_seen else OverheadEvent.ROBOT_LOST
        self._apply_event(event)

    def _process_and_publish(
        self,
        frame: np.ndarray,
        now_s: float,
    ) -> None:
        """Runs pure vision/safety pipeline and publishes world state and sensors."""
        homography = self._camera.homography
        if homography is None:
            return

        detection = self._pipeline.detect_and_track(
            frame,
            homography=homography,
            now_s=now_s,
        )
        self._update_robot_visibility_state(detection.is_all_seen)

        if not can_publish(self._state):
            return

        # TODO(overhead): populate carrying_robots once the robots' own state topics are subscribed.
        safety_config = self._safety_inputs.build_config(
            now_s,
            carrying_robots=frozenset(),
        )
        evaluation = self._pipeline.evaluate_sensors_and_safety(
            detection,
            now_s=now_s,
            safety_config=safety_config,
        )
        self._latest_red_obstacles = evaluation.red_obstacles

        self._publishers.publish_world(
            stamp_msg=self.get_clock().now().to_msg(),
            detection=detection,
            evaluation=evaluation,
        )

    def _step_camera_for_state(
        self,
        now_s: float,
    ) -> CameraStepResult:
        """Advances the camera session according to the current state."""
        if self._state == OverheadState.STARTING:
            return self._camera.step_starting(now_s)
        if self._state == OverheadState.CALIBRATING:
            return self._camera.step_calibrating(now_s)
        if self._state == OverheadState.NO_IMAGE:
            return self._camera.step_no_image(now_s)
        return self._camera.read_active_frame(now_s)

    def _on_main_timer(self) -> None:
        """Periodic loop driving the overhead state machine and vision publication."""
        now_s = time.monotonic()
        was_active = self._state not in {
            OverheadState.STARTING,
            OverheadState.CALIBRATING,
        }
        step = self._step_camera_for_state(now_s)
        self._apply_camera_step(
            step,
            now_s,
        )
        if was_active and step.frame is not None:
            self._process_and_publish(
                step.frame,
                now_s,
            )

    def _on_image_timer(self) -> None:
        """Generates and publishes the rectified arena view whenever frame and homography exist."""
        latest_frame = self._camera.latest_frame
        homography = self._camera.homography
        if latest_frame is None or homography is None:
            return

        is_published = self._publishers.publish_image(
            latest_frame,
            homography=homography,
            stamp_msg=self.get_clock().now().to_msg(),
            arena_width_m=self._params.arena_width_m,
            arena_height_m=self._params.arena_height_m,
        )
        if not is_published:
            self._log_warn_throttled(
                const.LOG_IMAGE_ENCODE_FAILED,
                time.monotonic(),
            )

    def destroy_node(self) -> bool:
        """Releases camera resources before destroying the ROS node."""
        self._camera.release()
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
