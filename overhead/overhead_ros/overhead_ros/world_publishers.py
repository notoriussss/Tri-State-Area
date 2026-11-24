"""Message construction and publishing helpers for overhead_node."""

from typing import Any

import cv2
import numpy as np
from overhead_core.constants import (
    DEFAULT_IMAGE_JPEG_QUALITY,
    DEFAULT_RECTIFY_PX_PER_M,
    IMAGE_COMPRESSION_FORMAT,
    JPEG_EXTENSION,
    RECTIFY_MARGIN_M,
)
from overhead_core.geometry import rectify
from overhead_core.world_pipeline import (
    DetectionStepResult,
    WorldEvaluationResult,
)
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from tsa_interfaces.msg import (
    VirtualSensors,
    WorldState,
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
    SENSOR_QOS,
    STATE_QOS,
    WORLD_IMAGE_TOPIC,
    WORLD_SENSORS_TOPIC,
    WORLD_STATE_TOPIC,
)


def encode_rectified_image_msg(
    frame: np.ndarray,
    homography: np.ndarray,
    *,
    stamp_msg: Any,
    arena_width_m: float,
    arena_height_m: float,
) -> CompressedImage | None:
    """Rectifies the camera frame and JPEG-encodes it into a CompressedImage message.

    Args:
        frame: Latest raw BGR camera frame.
        homography: 3x3 homography mapping image pixels to world meters.
        stamp_msg: ROS Time message assigned to the image header.
        arena_width_m: Physical arena width along world +x in meters.
        arena_height_m: Physical arena height along world +y in meters.

    Returns:
        CompressedImage ready to publish, or None if JPEG encoding fails.
    """
    rectified = rectify(
        frame,
        homography,
        DEFAULT_RECTIFY_PX_PER_M,
        arena_width_m=arena_width_m,
        arena_height_m=arena_height_m,
        margin_m=RECTIFY_MARGIN_M,
    )

    encode_success, jpeg_buffer = cv2.imencode(
        JPEG_EXTENSION,
        rectified,
        [int(cv2.IMWRITE_JPEG_QUALITY), DEFAULT_IMAGE_JPEG_QUALITY],
    )
    if not encode_success:
        return None

    img_msg = CompressedImage()
    img_msg.header.stamp = stamp_msg
    img_msg.header.frame_id = ARENA_FRAME_ID
    img_msg.format = IMAGE_COMPRESSION_FORMAT
    img_msg.data = jpeg_buffer.tobytes()
    return img_msg


class WorldPublishers:
    """Owns overhead ROS 2 publishers and caches the latest published world messages."""

    def __init__(self, node: Node) -> None:
        """Creates /world/state, /world/sensors, and /world/image publishers on node."""
        self._pub_state = node.create_publisher(
            WorldState,
            WORLD_STATE_TOPIC,
            STATE_QOS,
        )
        self._pub_sensors = node.create_publisher(
            VirtualSensors,
            WORLD_SENSORS_TOPIC,
            SENSOR_QOS,
        )
        self._pub_image = node.create_publisher(
            CompressedImage,
            WORLD_IMAGE_TOPIC,
            IMAGE_QOS,
        )
        self._latest_state = WorldState()
        self._latest_sensors = VirtualSensors()

    @property
    def latest_state(self) -> WorldState:
        """Returns the most recently published WorldState message."""
        return self._latest_state

    @property
    def latest_sensors(self) -> VirtualSensors:
        """Returns the most recently published VirtualSensors message."""
        return self._latest_sensors

    def _build_world_state_msg(
        self,
        header: Header,
        detection: DetectionStepResult,
    ) -> WorldState:
        """Builds a WorldState message from tracked cubes and robot poses."""
        cube_msgs = [
            to_cube_observation_msg(
                cid,
                ccolor,
                cx,
                y=cy,
                confidence=cconf,
            )
            for cid, ccolor, cx, cy, cconf in detection.tracked_cubes
        ]
        robot_msgs = [
            to_robot_observation_msg(
                pose.name,
                pose.x,
                pose.y,
                theta=pose.theta,
                confidence=pose.confidence,
            )
            for pose in detection.robot_poses
        ]
        return to_world_state_msg(
            header,
            cube_msgs,
            robot_msgs,
        )

    def _build_virtual_sensors_msg(
        self,
        header: Header,
        evaluation: WorldEvaluationResult,
    ) -> VirtualSensors:
        """Builds a VirtualSensors message from virtual sensors and safety outputs."""
        vs_result = evaluation.vs_result
        safety_result = evaluation.safety_result
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
        return to_virtual_sensors_msg(
            header,
            vs_result["robot_distance"],
            vs_result["robots_in_zone"],
            per_robot=per_robot_msgs,
            obstacles=obstacle_msgs,
            per_robot_obstacles=per_robot_obstacle_msgs,
        )

    def publish_world(
        self,
        *,
        stamp_msg: Any,
        detection: DetectionStepResult,
        evaluation: WorldEvaluationResult,
    ) -> None:
        """Constructs, publishes, and caches WorldState and VirtualSensors messages."""
        header = Header()
        header.stamp = stamp_msg
        header.frame_id = ARENA_FRAME_ID

        world_state_msg = self._build_world_state_msg(
            header,
            detection,
        )
        self._pub_state.publish(world_state_msg)
        self._latest_state = world_state_msg

        virtual_sensors_msg = self._build_virtual_sensors_msg(
            header,
            evaluation,
        )
        self._pub_sensors.publish(virtual_sensors_msg)
        self._latest_sensors = virtual_sensors_msg

    def publish_image(
        self,
        frame: np.ndarray,
        *,
        homography: np.ndarray,
        stamp_msg: Any,
        arena_width_m: float,
        arena_height_m: float,
    ) -> bool:
        """Rectifies, JPEG-encodes, and publishes the arena view if encoding succeeds."""
        img_msg = encode_rectified_image_msg(
            frame,
            homography,
            stamp_msg=stamp_msg,
            arena_width_m=arena_width_m,
            arena_height_m=arena_height_m,
        )
        if img_msg is None:
            return False

        self._pub_image.publish(img_msg)
        return True
