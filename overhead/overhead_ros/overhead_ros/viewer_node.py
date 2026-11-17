"""On-screen virtual projection visualization node using OpenCV."""

import math

import cv2
import numpy as np
import rclpy
from overhead_core.constants import (
    ARENA_HEIGHT_M,
    CubeColor,
)
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from tsa_interfaces.msg import (
    FleetPlan,
    WorldState,
)

from overhead_ros.ros_names import (
    FLEET_PLAN_TOPIC,
    IMAGE_QOS,
    PLAN_QOS,
    STATE_QOS,
    WORLD_IMAGE_TOPIC,
    WORLD_STATE_TOPIC,
)

WINDOW_NAME = "Tri-State-Area Viewer"
DEFAULT_PX_PER_M = 500.0
DEFAULT_MARGIN_M = 0.1

COLOR_BGR_MAP = {
    CubeColor.BLUE.value: (255, 0, 0),
    CubeColor.YELLOW.value: (0, 230, 230),
    CubeColor.RED.value: (0, 0, 255),
}


class ViewerNode(Node):
    """Subscribes to image, world state, and fleet plan to render the virtual projection."""

    def __init__(self) -> None:
        """Initializes subscriptions and window state."""
        super().__init__("viewer_node")

        self.declare_parameter(
            "arena_height_m",
            ARENA_HEIGHT_M,
        )
        self.declare_parameter(
            "px_per_m",
            DEFAULT_PX_PER_M,
        )
        self.declare_parameter(
            "margin_m",
            DEFAULT_MARGIN_M,
        )

        self._arena_height_m = float(self.get_parameter("arena_height_m").value)
        self._px_per_m = float(self.get_parameter("px_per_m").value)
        self._margin_m = float(self.get_parameter("margin_m").value)

        self._latest_state: WorldState | None = None
        self._latest_plan: FleetPlan | None = None

        self._sub_image = self.create_subscription(
            CompressedImage,
            WORLD_IMAGE_TOPIC,
            self._on_image,
            IMAGE_QOS,
        )
        self._sub_state = self.create_subscription(
            WorldState,
            WORLD_STATE_TOPIC,
            self._on_state,
            STATE_QOS,
        )
        self._sub_plan = self.create_subscription(
            FleetPlan,
            FLEET_PLAN_TOPIC,
            self._on_plan,
            PLAN_QOS,
        )

        self.get_logger().info("Viewer node initialized.")

    def _world_to_rect_px(
        self,
        x_m: float,
        y_m: float,
    ) -> tuple[int, int]:
        """Maps world arena coordinates to pixel positions on the rectified view."""
        u_px = int(round((x_m + self._margin_m) * self._px_per_m))
        v_px = int(round((self._arena_height_m + self._margin_m - y_m) * self._px_per_m))
        return u_px, v_px

    def _on_state(self, msg: WorldState) -> None:
        """Caches the latest world state message."""
        self._latest_state = msg

    def _on_plan(self, msg: FleetPlan) -> None:
        """Caches the latest fleet plan message."""
        self._latest_plan = msg

    def _on_image(self, msg: CompressedImage) -> None:
        """Renders world annotations over incoming rectified arena frames."""
        np_arr = np.frombuffer(
            msg.data,
            np.uint8,
        )
        frame = cv2.imdecode(
            np_arr,
            cv2.IMREAD_COLOR,
        )
        if frame is None:
            return

        self._draw_world_state(frame)
        self._draw_fleet_plan(frame)

        cv2.imshow(
            WINDOW_NAME,
            frame,
        )
        cv2.waitKey(1)

    def _draw_world_state(self, frame: np.ndarray) -> None:
        """Draws tracked cubes and detected robots on the display frame."""
        if self._latest_state is None:
            return

        for cube in self._latest_state.cubes:
            cu, cv = self._world_to_rect_px(
                cube.x,
                cube.y,
            )
            color = COLOR_BGR_MAP.get(
                cube.color,
                (200, 200, 200),
            )
            cv2.rectangle(
                frame,
                (cu - 7, cv - 7),
                (cu + 7, cv + 7),
                color,
                -1,
            )
            cv2.rectangle(
                frame,
                (cu - 7, cv - 7),
                (cu + 7, cv + 7),
                (0, 0, 0),
                1,
            )
            cv2.putText(
                frame,
                f"#{cube.id}",
                (cu + 9, cv + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        for robot in self._latest_state.robots:
            ru, rv = self._world_to_rect_px(
                robot.x,
                robot.y,
            )
            arrow_len = 35.0
            tip_u = int(round(ru + arrow_len * math.cos(robot.theta)))
            tip_v = int(round(rv - arrow_len * math.sin(robot.theta)))

            cv2.circle(
                frame,
                (ru, rv),
                12,
                (0, 255, 0),
                2,
            )
            cv2.arrowedLine(
                frame,
                (ru, rv),
                (tip_u, tip_v),
                (0, 255, 0),
                2,
                tipLength=0.3,
            )
            cv2.putText(
                frame,
                robot.name,
                (ru - 20, rv - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    def _draw_fleet_plan(self, frame: np.ndarray) -> None:
        """Draws planned robot trajectories and target positions on the display frame."""
        if self._latest_plan is None:
            return

        for plan in self._latest_plan.robots:
            if len(plan.path) > 1:
                for idx in range(len(plan.path) - 1):
                    p0 = plan.path[idx]
                    p1 = plan.path[idx + 1]
                    u0, v0 = self._world_to_rect_px(
                        p0.x,
                        p0.y,
                    )
                    u1, v1 = self._world_to_rect_px(
                        p1.x,
                        p1.y,
                    )
                    cv2.line(
                        frame,
                        (u0, v0),
                        (u1, v1),
                        (255, 128, 0),
                        2,
                    )

            tu, tv = self._world_to_rect_px(
                plan.target_pose.x,
                plan.target_pose.y,
            )
            cv2.drawMarker(
                frame,
                (tu, tv),
                (0, 165, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=2,
            )


def main(args: list[str] | None = None) -> None:
    """Entry point for viewer_node."""
    rclpy.init(args=args)
    node = ViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
