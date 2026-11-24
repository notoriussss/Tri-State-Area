"""Pure conversions between tsa_interfaces ROS messages and fleet_core domain types."""

from fleet_core.constants import (
    CubeColor,
    PlannerState,
)
from fleet_core.types import (
    CubeObservation,
    PlanSnapshot,
    Pose2D,
    RobotObservation,
    RobotSensors,
    RobotStatusSnapshot,
    VirtualSensorsSnapshot,
    WorldSnapshot,
)
from geometry_msgs.msg import Point
from tsa_interfaces.msg import (
    Pose2D as Pose2DMsg,
    RobotPlan,
    RobotStatus,
    VirtualSensors,
    WorldState,
)

VALID_COLORS: dict[str, CubeColor] = {
    color.value: color
    for color in CubeColor
}

VALID_STATES: dict[str, PlannerState] = {
    state.value: state
    for state in PlannerState
}


def from_world_state_msg(
    msg: WorldState,
    received_at_s: float,
) -> WorldSnapshot:
    """Converts a WorldState ROS message into a locally timestamped WorldSnapshot.

    Args:
        msg: Incoming /world/state message.
        received_at_s: Local monotonic reception timestamp in seconds.

    Returns:
        Immutable WorldSnapshot for fleet_core.
    """
    cubes = tuple(
        CubeObservation(
            cube_id=int(cube.id),
            color=VALID_COLORS[cube.color],
            x_m=float(cube.x),
            y_m=float(cube.y),
            confidence=float(cube.confidence),
        )
        for cube in msg.cubes
        if cube.color in VALID_COLORS
    )
    robots = tuple(
        RobotObservation(
            name=str(robot.name),
            x_m=float(robot.x),
            y_m=float(robot.y),
            theta_rad=float(robot.theta),
            confidence=float(robot.confidence),
        )
        for robot in msg.robots
    )
    return WorldSnapshot(
        received_at_s=received_at_s,
        cubes=cubes,
        robots=robots,
    )


def from_virtual_sensors_msg(
    msg: VirtualSensors,
    received_at_s: float,
) -> VirtualSensorsSnapshot:
    """Converts a VirtualSensors ROS message into a locally timestamped snapshot.

    Args:
        msg: Incoming /world/sensors message.
        received_at_s: Local monotonic reception timestamp in seconds.

    Returns:
        Immutable VirtualSensorsSnapshot for fleet_core.
    """
    per_robot = {
        str(entry.name): RobotSensors(
            name=str(entry.name),
            nearest_red_id=int(entry.nearest_red_id),
            nearest_red_dist_m=float(entry.nearest_red_dist),
            events=tuple(str(ev) for ev in entry.events),
        )
        for entry in msg.per_robot
    }
    return VirtualSensorsSnapshot(
        received_at_s=received_at_s,
        robot_distance_m=float(msg.robot_distance),
        robots_in_zone=tuple(str(name) for name in msg.robots_in_zone),
        per_robot=per_robot,
    )


def from_robot_plan_msg(
    msg: RobotPlan,
    received_at_s: float,
) -> PlanSnapshot:
    """Converts an incoming peer RobotPlan message into a PlanSnapshot.

    Args:
        msg: Incoming /<peer>/plan message.
        received_at_s: Local monotonic reception timestamp in seconds.

    Returns:
        Immutable PlanSnapshot for mission crossing and traffic priority checks.
    """
    state = VALID_STATES.get(
        str(msg.state),
        PlannerState.IDLE,
    )
    target_pose = Pose2D(
        x_m=float(msg.target_pose.x),
        y_m=float(msg.target_pose.y),
        theta_rad=float(msg.target_pose.theta),
    )
    path = tuple(
        (float(point.x), float(point.y))
        for point in msg.path
    )
    return PlanSnapshot(
        name=str(msg.name),
        state=state,
        target_cube_id=int(msg.target_cube_id),
        target_pose=target_pose,
        path=path,
        received_at_s=received_at_s,
    )


def to_robot_plan_msg(plan: PlanSnapshot) -> RobotPlan:
    """Converts an internal PlanSnapshot into a contract RobotPlan message."""
    msg = RobotPlan()
    msg.name = plan.name
    msg.state = plan.state.to_wire_state()
    msg.target_cube_id = int(plan.target_cube_id)
    msg.target_pose = Pose2DMsg(
        x=float(plan.target_pose.x_m),
        y=float(plan.target_pose.y_m),
        theta=float(plan.target_pose.theta_rad),
    )
    msg.path = [
        Point(
            x=float(px),
            y=float(py),
            z=0.0,
        )
        for px, py in plan.path
    ]
    return msg


def to_robot_status_msg(status: RobotStatusSnapshot) -> RobotStatus:
    """Converts an onboard RobotStatusSnapshot into a contract RobotStatus message."""
    msg = RobotStatus()
    msg.line_detected = bool(status.line_detected)
    msg.battery_v = float(status.battery_v)
    msg.cmd_age_ms = int(status.cmd_age_ms)
    msg.mode = status.mode.value
    return msg
