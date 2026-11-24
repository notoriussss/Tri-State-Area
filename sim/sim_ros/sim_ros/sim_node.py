"""ROS 2 simulator node publishing /world/state, /world/sensors, and mock Perry services."""

import time

import rclpy
from fleet_core.constants import (
    CONTROL_RATE_HZ,
    CubeColor,
    RobotName,
)
from fleet_core.types import (
    Pose2D,
    VirtualSensorsSnapshot,
    WorldSnapshot,
)
from multirobot_interfaces.msg import RobotState
from multirobot_interfaces.srv import (
    ReleaseZone,
    RequestZone,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sim_core.arena_sim import (
    ArenaSimulator,
    MockPerryArbiter,
    SimCube,
)
from std_msgs.msg import (
    Header,
    String,
)
from tsa_interfaces.msg import (
    CubeObservation,
    RobotObservation,
    RobotSensors,
    VirtualSensors,
    WorldState,
)
from tsa_interfaces.srv import GetWorld

NODE_NAME = "sim_node"
ARENA_FRAME_ID = "arena"
OCCUPANT_RATE_HZ = 2.0

WORLD_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
OCCUPANT_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
RELIABLE_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _to_world_msg(
    header: Header,
    snapshot: WorldSnapshot,
) -> WorldState:
    """Converts a simulated WorldSnapshot into a WorldState ROS message."""
    msg = WorldState()
    msg.header = header
    msg.cubes = [
        CubeObservation(
            id=cube.cube_id,
            color=cube.color.value,
            x=cube.x_m,
            y=cube.y_m,
            confidence=cube.confidence,
        )
        for cube in snapshot.cubes
    ]
    msg.robots = [
        RobotObservation(
            name=robot.name,
            x=robot.x_m,
            y=robot.y_m,
            theta=robot.theta_rad,
            confidence=robot.confidence,
        )
        for robot in snapshot.robots
    ]
    return msg


def _to_sensors_msg(
    header: Header,
    snapshot: VirtualSensorsSnapshot,
) -> VirtualSensors:
    """Converts a simulated VirtualSensorsSnapshot into a VirtualSensors ROS message."""
    msg = VirtualSensors()
    msg.header = header
    msg.robot_distance = snapshot.robot_distance_m
    msg.robots_in_zone = list(snapshot.robots_in_zone)
    msg.per_robot = [
        RobotSensors(
            name=entry.name,
            nearest_red_id=entry.nearest_red_id,
            nearest_red_dist=entry.nearest_red_dist_m,
            events=list(entry.events),
        )
        for entry in snapshot.per_robot.values()
    ]
    return msg


class SimNode(Node):
    """Publishes simulated overhead world topics and serves mock Perry zone leases."""

    def __init__(self) -> None:
        """Initializes a default two-robot arena scene and ROS 2 contract endpoints."""
        super().__init__(NODE_NAME)
        self.declare_parameter("rate_hz", CONTROL_RATE_HZ)

        self._sim = ArenaSimulator(
            robots={
                RobotName.PHINEAS.value: Pose2D(x_m=0.22, y_m=0.50, theta_rad=0.0),
                RobotName.FERB.value: Pose2D(x_m=0.22, y_m=0.20, theta_rad=0.0),
            },
            cubes=[
                SimCube(1, CubeColor.BLUE, 0.52, 0.48),
                SimCube(2, CubeColor.YELLOW, 0.52, 0.22),
                SimCube(3, CubeColor.RED, 0.45, 0.35),
            ],
        )
        self._arbiter = MockPerryArbiter()
        self._latest_world = WorldState()
        self._latest_sensors = VirtualSensors()

        self._pub_world = self.create_publisher(WorldState, "/world/state", WORLD_QOS)
        self._pub_sensors = self.create_publisher(VirtualSensors, "/world/sensors", WORLD_QOS)
        self._pub_occupant = self.create_publisher(String, "/zone/occupant", OCCUPANT_QOS)
        self._pub_perry = self.create_publisher(RobotState, "/perry/state", RELIABLE_QOS)

        self.create_service(GetWorld, "/world/get", self._on_get_world)
        self.create_service(RequestZone, "/zone/request", self._on_request_zone)
        self.create_service(ReleaseZone, "/zone/release", self._on_release_zone)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate_hz, self._on_world_tick)
        self.create_timer(1.0 / OCCUPANT_RATE_HZ, self._on_perry_tick)
        self.get_logger().info("SimNode started publishing /world/state and mock Perry services.")

    def _on_world_tick(self) -> None:
        """Publishes /world/state and /world/sensors at the configured rate."""
        now_s = time.monotonic()
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=ARENA_FRAME_ID,
        )
        world_snap, sensors_snap = self._sim.snapshot(now_s)
        self._latest_world = _to_world_msg(header, world_snap)
        self._latest_sensors = _to_sensors_msg(header, sensors_snap)
        self._pub_world.publish(self._latest_world)
        self._pub_sensors.publish(self._latest_sensors)

    def _on_perry_tick(self) -> None:
        """Publishes /zone/occupant and /perry/state heartbeats at 2 Hz."""
        now_s = time.monotonic()
        self._pub_occupant.publish(String(data=self._arbiter.occupant(now_s)))
        state_msg = RobotState()
        state_msg.stamp = self.get_clock().now().to_msg()
        state_msg.robot = "perry"
        state_msg.state = "idle"
        state_msg.carrying = False
        state_msg.delivered = self._arbiter.delivered_total
        state_msg.gate_closed = self._arbiter.gate_closed
        self._pub_perry.publish(state_msg)

    def _on_get_world(self, _request, response):
        """Returns the latest WorldState and VirtualSensors on /world/get."""
        response.state = self._latest_world
        response.sensors = self._latest_sensors
        return response

    def _on_request_zone(self, request, response):
        """Handles /zone/request via MockPerryArbiter."""
        granted, holder, lease_s = self._arbiter.request_zone(
            str(request.robot),
            time.monotonic(),
        )
        response.granted = granted
        response.holder = holder
        response.lease_seconds = float(lease_s)
        return response

    def _on_release_zone(self, request, response):
        """Handles /zone/release via MockPerryArbiter."""
        ok, total = self._arbiter.release_zone(
            str(request.robot),
            delivered=bool(request.delivered),
            color=str(request.color),
        )
        response.ok = ok
        response.delivered_total = total
        return response


def main(args: list[str] | None = None) -> None:
    """Entry point for sim_node."""
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
