"""ROS 2 node capturing the arena overhead camera and publishing world state."""

from typing import Any

import cv2
import numpy as np
import rclpy
from overhead_core import virtual_sensors
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    ARENA_WIDTH_M,
    CAM_HEIGHT_M,
    CUBE_HEIGHT_M,
    DEFAULT_CAMERA_CENTER_WORLD,
    DEFAULT_MIN_AREA_PX,
    DEFAULT_ZONE_RECT,
    NEAR_RED_DISTANCE_M,
    RECTIFY_MARGIN_M,
    ROBOT_MARKER_HEIGHT_M,
)
from overhead_core.cubes import detect_cubes
from overhead_core.geometry import (
    compute_homography,
    detect_arena_corners,
    image_to_world,
    rectify,
)
from overhead_core.parallax import correct_parallax
from overhead_core.tracker import CubeTracker
from overhead_core.triangle import heading_to_world, triangle_pose
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tsa_interfaces.msg import (
    RobotObservation,
    VirtualSensors,
    WorldState,
)
from tsa_interfaces.srv import GetWorld

from overhead_ros.conversions import (
    to_cube_observation_msg,
    to_robot_observation_msg,
    to_robot_sensors_msg,
    to_virtual_sensors_msg,
    to_world_state_msg,
)
from overhead_ros.ros_names import (
    ARENA_FRAME_ID,
    IMAGE_QOS,
    OVERHEAD_RECALIBRATE_SERVICE,
    SENSOR_QOS,
    STATE_QOS,
    WORLD_GET_SERVICE,
    WORLD_IMAGE_TOPIC,
    WORLD_SENSORS_TOPIC,
    WORLD_STATE_TOPIC,
)

DEFAULT_RECTIFY_PX_PER_M = 500.0


class OverheadNode(Node):
    """Captures overhead camera frames, runs pure vision tracking, and publishes world state."""

    def __init__(self) -> None:
        """Initializes publishers, services, parameters, and capture pipelines."""
        super().__init__("overhead_node")

        self._declare_parameters()
        self._load_parameters()

        self._cap = cv2.VideoCapture(self._camera_index)
        self._cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self._width,
        )
        self._cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self._height,
        )

        self._homography: np.ndarray | None = None
        self._latest_frame: np.ndarray | None = None
        self._latest_state = WorldState()
        self._latest_sensors = VirtualSensors()
        self._prev_in_zone: set[str] = set()

        self._tracker = CubeTracker()

        self._calibrate()

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

        main_timer_period_s = 1.0 / max(
            1.0,
            self._rate_hz,
        )
        self._timer_main = self.create_timer(
            main_timer_period_s,
            self._on_main_timer,
        )

        image_timer_period_s = 1.0 / max(
            0.5,
            self._image_rate_hz,
        )
        self._timer_image = self.create_timer(
            image_timer_period_s,
            self._on_image_timer,
        )

        self.get_logger().info("Overhead node initialized successfully.")

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "camera_index",
            0,
        )
        self.declare_parameter(
            "width",
            1280,
        )
        self.declare_parameter(
            "height",
            720,
        )
        self.declare_parameter(
            "rate_hz",
            20.0,
        )
        self.declare_parameter(
            "arena_width_m",
            ARENA_WIDTH_M,
        )
        self.declare_parameter(
            "arena_height_m",
            ARENA_HEIGHT_M,
        )
        self.declare_parameter(
            "cam_height_m",
            CAM_HEIGHT_M,
        )
        self.declare_parameter(
            "camera_center_world",
            list(DEFAULT_CAMERA_CENTER_WORLD),
        )
        self.declare_parameter(
            "zone_rect",
            list(DEFAULT_ZONE_RECT),
        )
        self.declare_parameter(
            "robots",
            ["phineas", "ferb"],
        )
        self.declare_parameter(
            "image_rate_hz",
            5.0,
        )
        self.declare_parameter(
            "min_area_px",
            DEFAULT_MIN_AREA_PX,
        )
        self.declare_parameter(
            "near_red_m",
            NEAR_RED_DISTANCE_M,
        )

        # Default HSV ranges
        # Lower hue 110 excludes the blue delivery-zone background (~101); cubes are 118-125
        self.declare_parameter(
            "hsv_blue",
            [[110, 80, 25], [135, 255, 255]],
        )
        self.declare_parameter(
            "hsv_yellow",
            [[20, 100, 55], [35, 255, 255]],
        )
        self.declare_parameter(
            "hsv_red_1",
            [[0, 120, 55], [10, 255, 255]],
        )
        self.declare_parameter(
            "hsv_red_2",
            [[170, 120, 55], [180, 255, 255]],
        )
        self.declare_parameter(
            "hsv_robot_phineas",
            [[40, 100, 100], [80, 255, 255]],
        )
        self.declare_parameter(
            "hsv_robot_ferb",
            [[140, 100, 100], [165, 255, 255]],
        )

    def _load_parameters(self) -> None:
        self._camera_index = int(self.get_parameter("camera_index").value)
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._arena_width_m = float(self.get_parameter("arena_width_m").value)
        self._arena_height_m = float(self.get_parameter("arena_height_m").value)
        self._cam_height_m = float(self.get_parameter("cam_height_m").value)
        self._camera_center_world = tuple(self.get_parameter("camera_center_world").value)
        self._zone_rect = tuple(self.get_parameter("zone_rect").value)
        self._robots = list(self.get_parameter("robots").value)
        self._image_rate_hz = float(self.get_parameter("image_rate_hz").value)
        self._min_area_px = int(self.get_parameter("min_area_px").value)
        self._near_red_m = float(self.get_parameter("near_red_m").value)

        self._hsv_ranges = {
            "blue": [tuple(tuple(v) for v in self.get_parameter("hsv_blue").value)],
            "yellow": [tuple(tuple(v) for v in self.get_parameter("hsv_yellow").value)],
            "red": [
                tuple(tuple(v) for v in self.get_parameter("hsv_red_1").value),
                tuple(tuple(v) for v in self.get_parameter("hsv_red_2").value),
            ],
        }

        self._robot_hsv_ranges: dict[str, list[tuple[Any, Any]]] = {
            "phineas": [tuple(tuple(v) for v in self.get_parameter("hsv_robot_phineas").value)],
            "ferb": [tuple(tuple(v) for v in self.get_parameter("hsv_robot_ferb").value)],
        }

    def _calibrate(self) -> bool:
        """Finds arena corners and computes perspective homography."""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Camera capture unavailable during calibration; using default.")
            corners = np.array(
                [
                    [0.0, float(self._height)],
                    [float(self._width), float(self._height)],
                    [float(self._width), 0.0],
                    [0.0, 0.0],
                ],
                dtype=np.float32,
            )
        else:
            self._latest_frame = frame
            try:
                corners = detect_arena_corners(frame)
            except (ValueError, cv2.error) as exc:
                self.get_logger().warn(
                    f"Corner detection failed ({exc}); falling back to frame bounds."
                )
                corners = np.array(
                    [
                        [0.0, float(self._height)],
                        [float(self._width), float(self._height)],
                        [float(self._width), 0.0],
                        [0.0, 0.0],
                    ],
                    dtype=np.float32,
                )

        self._homography = compute_homography(
            corners,
            self._arena_width_m,
            self._arena_height_m,
        )
        self.get_logger().info("Homography calibration matrix computed successfully.")
        return True

    def _on_recalibrate(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Service callback to recompute the arena homography."""
        success = self._calibrate()
        response.success = success
        response.message = "Overhead camera recalibrated successfully." if success else "Failed."
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

    def _process_cubes(
        self,
        frame: np.ndarray,
        now_s: float,
    ) -> list[tuple[int, str, float, float, float]]:
        """Detects, projects, and tracks arena cubes."""
        cube_raw_dets = detect_cubes(
            frame,
            None,
            self._hsv_ranges,
            min_area_px=self._min_area_px,
        )
        cube_floor_dets: list[tuple[str, float, float, float]] = []
        for color, u_px, v_px, _area in cube_raw_dets:
            seen_world = image_to_world(
                self._homography,
                [[u_px, v_px]],
            )
            corrected_floor = correct_parallax(
                seen_world,
                self._camera_center_world,
                object_height_m=CUBE_HEIGHT_M,
                cam_height_m=self._cam_height_m,
            )
            cube_floor_dets.append(
                (
                    color,
                    float(corrected_floor[0, 0]),
                    float(corrected_floor[0, 1]),
                    1.0,
                )
            )

        return self._tracker.update(
            cube_floor_dets,
            now_s,
        )

    def _process_single_robot(
        self,
        hsv_frame: np.ndarray,
        robot_name: str,
    ) -> tuple[RobotObservation, tuple[float, float]] | None:
        """Detects a single robot marker and computes its world pose."""
        ranges = self._robot_hsv_ranges.get(robot_name)
        if not ranges:
            return None

        robot_mask = np.zeros(
            hsv_frame.shape[:2],
            dtype=np.uint8,
        )
        for lower_b, upper_b in ranges:
            sub_mask = cv2.inRange(
                hsv_frame,
                np.asarray(lower_b, dtype=np.uint8),
                np.asarray(upper_b, dtype=np.uint8),
            )
            robot_mask = cv2.bitwise_or(
                robot_mask,
                sub_mask,
            )

        pose = triangle_pose(robot_mask)
        if pose is None:
            return None

        u_px, v_px, theta_img = pose
        seen_world = image_to_world(
            self._homography,
            [[u_px, v_px]],
        )
        corrected_floor = correct_parallax(
            seen_world,
            self._camera_center_world,
            object_height_m=ROBOT_MARKER_HEIGHT_M,
            cam_height_m=self._cam_height_m,
        )
        theta_world = heading_to_world(
            self._homography,
            u_px,
            v_px,
            theta_img=theta_img,
        )

        rx = float(corrected_floor[0, 0])
        ry = float(corrected_floor[0, 1])
        observation = to_robot_observation_msg(
            robot_name,
            rx,
            ry,
            theta=theta_world,
            confidence=1.0,
        )
        return observation, (rx, ry)

    def _process_robots(
        self,
        frame: np.ndarray,
    ) -> tuple[list[RobotObservation], dict[str, tuple[float, float]]]:
        """Detects all robot markers in the current frame."""
        hsv_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )
        robot_observations: list[RobotObservation] = []
        robots_dict: dict[str, tuple[float, float]] = {}

        for robot_name in self._robots:
            result = self._process_single_robot(
                hsv_frame,
                robot_name,
            )
            if result is not None:
                obs, pos = result
                robot_observations.append(obs)
                robots_dict[robot_name] = pos

        return robot_observations, robots_dict

    def _publish_state_and_sensors(
        self,
        now_stamp: Time,
        tracked_cubes: list[tuple[int, str, float, float, float]],
        *,
        robot_observations: list[RobotObservation],
        vs_result: dict[str, Any],
    ) -> None:
        """Constructs and publishes WorldState and VirtualSensors messages."""
        header = Header()
        header.stamp = now_stamp.to_msg()
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
        world_state_msg = to_world_state_msg(
            header,
            cube_msgs,
            robot_observations,
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
        virtual_sensors_msg = to_virtual_sensors_msg(
            header,
            vs_result["robot_distance"],
            vs_result["robots_in_zone"],
            per_robot=per_robot_msgs,
        )
        self._pub_sensors.publish(virtual_sensors_msg)
        self._latest_sensors = virtual_sensors_msg

    def _on_main_timer(self) -> None:
        """Main periodic vision loop executing cube tracking and robot localization."""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            return

        self._latest_frame = frame
        if self._homography is None:
            return

        now_stamp = self.get_clock().now()
        now_s = now_stamp.nanoseconds * 1e-9

        tracked_cubes = self._process_cubes(
            frame,
            now_s,
        )
        robot_observations, robots_dict = self._process_robots(frame)

        cubes_for_sensors = [(t[0], t[1], t[2], t[3]) for t in tracked_cubes]
        vs_result = virtual_sensors.compute(
            robots_dict,
            cubes_for_sensors,
            zone_rect=self._zone_rect,
            prev_in_zone=self._prev_in_zone,
            near_red_m=self._near_red_m,
        )
        self._prev_in_zone = vs_result["in_zone"]

        self._publish_state_and_sensors(
            now_stamp,
            tracked_cubes,
            robot_observations=robot_observations,
            vs_result=vs_result,
        )

    def _on_image_timer(self) -> None:
        """Generates and publishes the rectified arena view for the viewer."""
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
            ".jpg",
            rectified,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80],
        )
        if not encode_success:
            return

        img_msg = CompressedImage()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = ARENA_FRAME_ID
        img_msg.format = "jpeg"
        img_msg.data = jpeg_buffer.tobytes()

        self._pub_image.publish(img_msg)


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
