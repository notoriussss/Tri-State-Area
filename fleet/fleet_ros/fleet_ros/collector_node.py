"""Thin ROS 2 node running onboard each collector Raspberry Pi 5 (Phineas / Ferb)."""

import time

import rclpy
from fleet_core.constants import (
    CONTROL_RATE_HZ,
    DEFAULT_BATTERY_V,
    DEFAULT_FL_CHANNELS,
    DEFAULT_FL_INVERTED,
    DEFAULT_FL_PINS,
    DEFAULT_FR_CHANNELS,
    DEFAULT_FR_INVERTED,
    DEFAULT_FR_PINS,
    DEFAULT_LINE_ADC_CHANNEL,
    DEFAULT_LINE_BLACK_MV,
    DEFAULT_LINE_BLACK_VALUE,
    DEFAULT_LINE_CONFIRM_READS,
    DEFAULT_LINE_SENSOR_PIN,
    DEFAULT_MAX_DUTY_RATIO,
    DEFAULT_MAX_WHEEL_SPEED_MPS,
    DEFAULT_NSLEEP_PINS,
    DEFAULT_PARK_UPPER_XY,
    DEFAULT_PRE_ZONE_UPPER_Y_M,
    DEFAULT_PWM_FREQUENCY_HZ,
    DEFAULT_RL_CHANNELS,
    DEFAULT_RL_INVERTED,
    DEFAULT_RL_PINS,
    DEFAULT_RR_CHANNELS,
    DEFAULT_RR_INVERTED,
    DEFAULT_RR_PINS,
    HAL_WATCHDOG_TIMEOUT_S,
    HALF_TRACK_M,
    HALF_WHEELBASE_M,
    PLAN_RATE_HZ,
    PRE_ZONE_X_M,
    RED_TRANSIT_RADIUS_M,
    STATUS_RATE_HZ,
    WHEEL_RADIUS_M,
    ZONE_CALL_TIMEOUT_S,
    ZONE_HEARTBEAT_TIMEOUT_S,
    ZONE_LEASE_RENEW_S,
    CubeColor,
    MotorDriverType,
    RobotName,
    VirtualSensorEvent,
)
from fleet_core.hal_guard import (
    HalGuard,
    HalGuardConfig,
)
from fleet_core.hal_line import (
    FakeLineSensor,
    LineSensorBackend,
    LineSensorConfig,
    Tcrt5000LineSensor,
)
from fleet_core.hal_motors import (
    MotorHardwareConfig,
    build_motors,
)
from fleet_core.manual_control import (
    ManualControlConfig,
    ManualControlState,
)
from fleet_core.mission import MissionConfig
from fleet_core.navigation import NavigationConfig
from fleet_core.state_machine import (
    CollectorStateMachine,
    CycleInputs,
    StateMachineConfig,
)
from fleet_core.traffic import (
    TrafficConfig,
    TrafficCoordinator,
)
from fleet_core.types import (
    ZERO_VELOCITY_CMD,
    MecanumGeometry,
    PlanSnapshot,
    Pose2D,
    VelocityCmd,
)
from fleet_core.world_model import (
    WorldModel,
    WorldModelConfig,
)
from fleet_core.zone_client import (
    ZoneClient,
    ZoneHeartbeat,
    is_call_expired,
    select_free_delivery_slot,
)
from multirobot_interfaces.msg import RobotState
from multirobot_interfaces.srv import (
    ReleaseZone,
    RequestZone,
)
from rclpy.node import Node
from std_msgs.msg import String
from tsa_interfaces.msg import (
    RobotPlan,
    RobotStatus,
    VirtualSensors,
    WorldState,
)

from fleet_ros.conversions import (
    from_robot_plan_msg,
    from_virtual_sensors_msg,
    from_world_state_msg,
    to_robot_plan_msg,
    to_robot_status_msg,
)
from fleet_ros.ros_names import (
    OCCUPANT_QOS,
    PERRY_STATE_TOPIC,
    PLAN_QOS,
    SENSOR_QOS,
    STATE_QOS,
    STATUS_QOS,
    WORLD_SENSORS_TOPIC,
    WORLD_STATE_TOPIC,
    ZONE_OCCUPANT_TOPIC,
    ZONE_RELEASE_SERVICE,
    ZONE_REQUEST_SERVICE,
    plan_topic_for,
    status_topic_for,
)
from fleet_ros.web_server import (
    DEFAULT_WEB_HOST,
    DEFAULT_WEB_PORT,
    CollectorWebServer,
)

NODE_NAME = "collector_node"
DISABLED_PIN = -1
NO_ADC_CHANNEL = -1


class CollectorNode(Node):
    """Bridges ROS 2 contract topics/services with the pure fleet_core pipeline."""

    def __init__(self) -> None:
        """Declares parameters, initializes fleet_core and HAL, and wires ROS interfaces."""
        super().__init__(NODE_NAME)
        self._declare_all_parameters()

        self._robot_name = str(self.get_parameter("robot_name").value)
        self._peer_name = str(self.get_parameter("peer_name").value)
        self._call_timeout_s = float(self.get_parameter("zone_call_timeout_s").value)

        self._world_model = WorldModel(
            WorldModelConfig(
                robot_name=self._robot_name,
                peer_name=self._peer_name,
            )
        )
        self._traffic = TrafficCoordinator(
            TrafficConfig(robot_name=self._robot_name),
        )
        self._heartbeat = ZoneHeartbeat(
            timeout_s=float(self.get_parameter("zone_heartbeat_timeout_s").value),
        )
        self._zone_client = ZoneClient(
            self._robot_name,
            renew_period_s=float(self.get_parameter("zone_lease_renew_s").value),
        )
        self._fsm = CollectorStateMachine(self._build_fsm_config())
        self._hal = self._build_hal_guard()
        self._manual = ManualControlState(
            ManualControlConfig(
                cmd_timeout_s=float(self.get_parameter("watchdog_timeout_s").value),
            ),
            auto_enabled=bool(self.get_parameter("auto_enabled").value),
        )
        self._web_server = self._build_web_server()

        self._peer_plan: PlanSnapshot | None = None
        self._latest_plan: PlanSnapshot | None = None
        self._last_cmd: VelocityCmd = ZERO_VELOCITY_CMD
        self._last_tick_s: float = time.monotonic()
        self._req_future = None
        self._rel_future = None
        self._req_sent_s: float | None = None
        self._rel_sent_s: float | None = None
        self._rel_sent: tuple[bool, str] = (False, "")

        self._create_ros_interfaces()
        self._create_timers()
        self.get_logger().info(
            f"CollectorNode started for {self._robot_name} (peer={self._peer_name})."
        )

    def _declare_all_parameters(self) -> None:
        """Declares all node parameters with defaults imported from fleet_core.constants."""
        self.declare_parameter("robot_name", RobotName.PHINEAS.value)
        self.declare_parameter("peer_name", RobotName.FERB.value)
        self.declare_parameter("target_color", CubeColor.BLUE.value)
        self.declare_parameter("pre_zone_x_m", PRE_ZONE_X_M)
        self.declare_parameter("pre_zone_y_m", DEFAULT_PRE_ZONE_UPPER_Y_M)
        self.declare_parameter("park_x_m", DEFAULT_PARK_UPPER_XY[0])
        self.declare_parameter("park_y_m", DEFAULT_PARK_UPPER_XY[1])
        self.declare_parameter("red_transit_radius_m", RED_TRANSIT_RADIUS_M)
        self.declare_parameter("control_rate_hz", CONTROL_RATE_HZ)
        self.declare_parameter("plan_rate_hz", PLAN_RATE_HZ)
        self.declare_parameter("status_rate_hz", STATUS_RATE_HZ)
        self.declare_parameter("zone_heartbeat_timeout_s", ZONE_HEARTBEAT_TIMEOUT_S)
        self.declare_parameter("zone_lease_renew_s", ZONE_LEASE_RENEW_S)
        self.declare_parameter("zone_call_timeout_s", ZONE_CALL_TIMEOUT_S)
        self.declare_parameter("watchdog_timeout_s", HAL_WATCHDOG_TIMEOUT_S)
        self.declare_parameter("wheel_radius_m", WHEEL_RADIUS_M)
        self.declare_parameter("half_wheelbase_m", HALF_WHEELBASE_M)
        self.declare_parameter("half_track_m", HALF_TRACK_M)
        self.declare_parameter("max_wheel_speed_mps", DEFAULT_MAX_WHEEL_SPEED_MPS)
        self.declare_parameter("motor_driver", MotorDriverType.FUSION_HAT.value)
        self.declare_parameter("fl_channels", list(DEFAULT_FL_CHANNELS))
        self.declare_parameter("fr_channels", list(DEFAULT_FR_CHANNELS))
        self.declare_parameter("rl_channels", list(DEFAULT_RL_CHANNELS))
        self.declare_parameter("rr_channels", list(DEFAULT_RR_CHANNELS))
        self.declare_parameter("fl_pins", list(DEFAULT_FL_PINS))
        self.declare_parameter("fr_pins", list(DEFAULT_FR_PINS))
        self.declare_parameter("rl_pins", list(DEFAULT_RL_PINS))
        self.declare_parameter("rr_pins", list(DEFAULT_RR_PINS))
        self.declare_parameter("nsleep_pins", list(DEFAULT_NSLEEP_PINS))
        self.declare_parameter("fl_inverted", DEFAULT_FL_INVERTED)
        self.declare_parameter("fr_inverted", DEFAULT_FR_INVERTED)
        self.declare_parameter("rl_inverted", DEFAULT_RL_INVERTED)
        self.declare_parameter("rr_inverted", DEFAULT_RR_INVERTED)
        self.declare_parameter("pwm_frequency_hz", DEFAULT_PWM_FREQUENCY_HZ)
        self.declare_parameter("max_duty_ratio", DEFAULT_MAX_DUTY_RATIO)
        self.declare_parameter("line_pin", DEFAULT_LINE_SENSOR_PIN)
        self.declare_parameter("line_black_value", DEFAULT_LINE_BLACK_VALUE)
        self.declare_parameter("line_confirm_reads", DEFAULT_LINE_CONFIRM_READS)
        self.declare_parameter("line_adc_channel", DEFAULT_LINE_ADC_CHANNEL)
        self.declare_parameter("line_black_mv", DEFAULT_LINE_BLACK_MV)
        self.declare_parameter("battery_v", DEFAULT_BATTERY_V)
        self.declare_parameter("auto_enabled", False)
        self.declare_parameter("web_host", DEFAULT_WEB_HOST)
        self.declare_parameter("web_port", DEFAULT_WEB_PORT)

    def _build_fsm_config(self) -> StateMachineConfig:
        """Builds StateMachineConfig from declared ROS parameters."""
        nav_cfg = NavigationConfig(
            red_transit_radius_m=float(self.get_parameter("red_transit_radius_m").value),
        )
        mission_cfg = MissionConfig(
            robot_name=self._robot_name,
            target_color=CubeColor(str(self.get_parameter("target_color").value)),
            pre_zone_goal_xy=(
                float(self.get_parameter("pre_zone_x_m").value),
                float(self.get_parameter("pre_zone_y_m").value),
            ),
            park_pose=Pose2D(
                x_m=float(self.get_parameter("park_x_m").value),
                y_m=float(self.get_parameter("park_y_m").value),
                theta_rad=0.0,
            ),
            navigation=nav_cfg,
        )
        return StateMachineConfig(
            mission=mission_cfg,
            navigation=nav_cfg,
        )

    def _build_hal_guard(self) -> HalGuard:
        """Instantiates motor and line-sensor backends wrapped by HalGuard."""
        driver = MotorDriverType(str(self.get_parameter("motor_driver").value))
        motor_cfg = MotorHardwareConfig(
            driver=driver,
            fl_channels=self._int_pair("fl_channels"),
            fr_channels=self._int_pair("fr_channels"),
            rl_channels=self._int_pair("rl_channels"),
            rr_channels=self._int_pair("rr_channels"),
            fl_pins=self._int_pair("fl_pins"),
            fr_pins=self._int_pair("fr_pins"),
            rl_pins=self._int_pair("rl_pins"),
            rr_pins=self._int_pair("rr_pins"),
            nsleep_pins=self._int_pair("nsleep_pins"),
            fl_inverted=bool(self.get_parameter("fl_inverted").value),
            fr_inverted=bool(self.get_parameter("fr_inverted").value),
            rl_inverted=bool(self.get_parameter("rl_inverted").value),
            rr_inverted=bool(self.get_parameter("rr_inverted").value),
            pwm_frequency_hz=int(self.get_parameter("pwm_frequency_hz").value),
            max_duty_ratio=float(self.get_parameter("max_duty_ratio").value),
        )
        motors = build_motors(motor_cfg)
        line_sensor = self._build_line_sensor(driver)
        guard_cfg = HalGuardConfig(
            geometry=MecanumGeometry(
                wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
                half_wheelbase_m=float(self.get_parameter("half_wheelbase_m").value),
                half_track_m=float(self.get_parameter("half_track_m").value),
            ),
            max_wheel_speed_mps=float(self.get_parameter("max_wheel_speed_mps").value),
            watchdog_timeout_s=float(self.get_parameter("watchdog_timeout_s").value),
            battery_v=float(self.get_parameter("battery_v").value),
        )
        return HalGuard(
            motors,
            line_sensor,
            guard_cfg,
        )

    def _build_line_sensor(self, driver: MotorDriverType) -> LineSensorBackend:
        """Creates Tcrt5000LineSensor on real hardware or FakeLineSensor in fake mode."""
        pin = int(self.get_parameter("line_pin").value)
        raw_adc = int(self.get_parameter("line_adc_channel").value)
        adc_channel = None if raw_adc == NO_ADC_CHANNEL else raw_adc
        line_cfg = LineSensorConfig(
            pin=pin,
            black_value=int(self.get_parameter("line_black_value").value),
            confirm_reads=int(self.get_parameter("line_confirm_reads").value),
            adc_channel=adc_channel,
            black_mv=int(self.get_parameter("line_black_mv").value),
        )
        if driver == MotorDriverType.FAKE or pin == DISABLED_PIN:
            return FakeLineSensor(line_cfg)

        return Tcrt5000LineSensor(line_cfg)

    def _int_pair(self, name: str) -> tuple[int, int]:
        """Reads a two-element integer ROS parameter as a tuple."""
        values = list(self.get_parameter(name).value)
        return (
            int(values[0]),
            int(values[1]),
        )

    def _create_ros_interfaces(self) -> None:
        """Creates publishers, subscriptions, and Perry zone service clients."""
        self._pub_plan = self.create_publisher(
            RobotPlan,
            plan_topic_for(self._robot_name),
            PLAN_QOS,
        )
        self._pub_status = self.create_publisher(
            RobotStatus,
            status_topic_for(self._robot_name),
            STATUS_QOS,
        )
        self.create_subscription(
            WorldState,
            WORLD_STATE_TOPIC,
            self._on_world_state,
            STATE_QOS,
        )
        self.create_subscription(
            VirtualSensors,
            WORLD_SENSORS_TOPIC,
            self._on_virtual_sensors,
            SENSOR_QOS,
        )
        self.create_subscription(
            RobotPlan,
            plan_topic_for(self._peer_name),
            self._on_peer_plan,
            PLAN_QOS,
        )
        self.create_subscription(
            String,
            ZONE_OCCUPANT_TOPIC,
            self._on_zone_occupant,
            OCCUPANT_QOS,
        )
        self.create_subscription(
            RobotState,
            PERRY_STATE_TOPIC,
            self._on_perry_state,
            PLAN_QOS,
        )
        self._req_client = self.create_client(
            RequestZone,
            ZONE_REQUEST_SERVICE,
        )
        self._rel_client = self.create_client(
            ReleaseZone,
            ZONE_RELEASE_SERVICE,
        )

    def _create_timers(self) -> None:
        """Schedules the 20 Hz control loop, HAL watchdog tick, and telemetry timers."""
        control_hz = float(self.get_parameter("control_rate_hz").value)
        plan_hz = float(self.get_parameter("plan_rate_hz").value)
        status_hz = float(self.get_parameter("status_rate_hz").value)
        self.create_timer(1.0 / control_hz, self._on_control_tick)
        self.create_timer(1.0 / control_hz, self._on_watchdog_tick)
        self.create_timer(1.0 / plan_hz, self._on_publish_plan)
        self.create_timer(1.0 / status_hz, self._on_publish_status)

    def _on_world_state(self, msg: WorldState) -> None:
        """Caches incoming /world/state stamped with local monotonic time."""
        self._world_model.update_world(
            from_world_state_msg(
                msg,
                time.monotonic(),
            )
        )

    def _on_virtual_sensors(self, msg: VirtualSensors) -> None:
        """Caches incoming /world/sensors stamped with local monotonic time."""
        self._world_model.update_sensors(
            from_virtual_sensors_msg(
                msg,
                time.monotonic(),
            )
        )

    def _on_peer_plan(self, msg: RobotPlan) -> None:
        """Caches incoming /<peer>/plan stamped with local monotonic time."""
        self._peer_plan = from_robot_plan_msg(
            msg,
            time.monotonic(),
        )

    def _on_zone_occupant(self, msg: String) -> None:
        """Records Perry's heartbeat and updates the active zone occupant."""
        self._heartbeat.beat(time.monotonic())
        self._zone_client.update(occupant=str(msg.data))

    def _on_perry_state(self, msg: RobotState) -> None:
        """Tracks Perry's gate_closed flag."""
        self._zone_client.update(gate_closed=bool(msg.gate_closed))

    def _on_control_tick(self) -> None:
        """Runs one 20 Hz decision cycle and drives the onboard HAL."""
        now_s = time.monotonic()
        dt_s = max(
            0.0,
            now_s - self._last_tick_s,
        )
        self._last_tick_s = now_s

        if not self._manual.auto_enabled:
            cmd = self._manual.current_cmd(now_s)
            self._last_cmd = cmd
            self._hal.drive(
                cmd,
                now_s,
                allow_line_escape=cmd.vx_mps < 0.0,
            )
            return

        own_pose = self._world_model.own_pose(now_s)
        traffic_must_wait = False
        traffic_evasion_cmd: VelocityCmd | None = None
        if own_pose is not None:
            red_positions = tuple(
                (cube.x_m, cube.y_m)
                for cube in self._world_model.cubes_by_color(CubeColor.RED, now_s)
            )
            decision = self._traffic.evaluate(
                own_pose,
                self._world_model.peer_pose(now_s),
                own_state=self._fsm.state,
                red_positions=red_positions,
                now_s=now_s,
                own_cmd=self._last_cmd,
                peer_plan=self._peer_plan,
                overhead_alert=self._world_model.has_virtual_event(
                    VirtualSensorEvent.NEAR_ROBOT.value,
                    now_s,
                ),
            )
            traffic_must_wait = decision.must_wait
            traffic_evasion_cmd = decision.evasion_cmd

        slot_xy = select_free_delivery_slot(self._world_model.all_cubes(now_s))
        inputs = CycleInputs(
            now_s=now_s,
            dt_s=dt_s,
            line_detected=self._hal.line(),
            peer_plan=self._peer_plan,
            zone_granted=self._zone_client.is_granted(self._heartbeat, now_s),
            zone_ready=self._zone_client.is_ready_to_enter(self._heartbeat, now_s),
            delivery_slot_xy=slot_xy,
            traffic_must_wait=traffic_must_wait,
            traffic_evasion_cmd=traffic_evasion_cmd,
        )
        output = self._fsm.step(
            self._world_model,
            inputs,
        )
        self._latest_plan = output.plan
        self._last_cmd = output.cmd
        self._hal.drive(
            output.cmd,
            now_s,
            allow_line_escape=output.allow_line_escape,
        )
        self._sync_zone_protocol(
            output.want_zone,
            output.release_zone,
            output.delivered_color,
            now_s=now_s,
        )

    def _on_watchdog_tick(self) -> None:
        """Enforces the local 300 ms HAL watchdog and black-line stop."""
        self._hal.tick(time.monotonic())

    def _on_publish_plan(self) -> None:
        """Publishes the latest RobotPlan on /<robot>/plan."""
        if self._latest_plan is None:
            return

        self._pub_plan.publish(to_robot_plan_msg(self._latest_plan))

    def _on_publish_status(self) -> None:
        """Publishes onboard HAL telemetry on /<robot>/status."""
        status = self._hal.status(time.monotonic())
        self._pub_status.publish(to_robot_status_msg(status))

    def _sync_zone_protocol(
        self,
        want_zone: bool,
        release_zone: bool,
        delivered_color: CubeColor | None,
        *,
        now_s: float,
    ) -> None:
        """Updates ZoneClient and dispatches asynchronous Perry service calls."""
        if release_zone:
            self._zone_client.request_release(delivered_color)
        else:
            self._zone_client.update(want=want_zone)

        self._drop_expired_zone_calls(now_s)
        if self._zone_client.should_release() and self._rel_future is None:
            self._send_zone_release(now_s)
        elif self._zone_client.should_request(now_s) and self._req_future is None:
            self._send_zone_request(now_s)

    def _drop_expired_zone_calls(self, now_s: float) -> None:
        """Drops unanswered Perry service calls after zone_call_timeout_s for retry."""
        if is_call_expired(self._req_sent_s, now_s, timeout_s=self._call_timeout_s):
            self._req_client.remove_pending_request(self._req_future)
            self._req_future = None
            self._req_sent_s = None

        if is_call_expired(self._rel_sent_s, now_s, timeout_s=self._call_timeout_s):
            self._rel_client.remove_pending_request(self._rel_future)
            self._rel_future = None
            self._rel_sent_s = None
            self._zone_client.restore_release(
                delivered=self._rel_sent[0],
                color=self._rel_sent[1],
            )

    def _send_zone_request(self, now_s: float) -> None:
        """Dispatches an async /zone/request call to Perry's arbiter."""
        if not self._req_client.service_is_ready():
            return

        self._zone_client.note_requested(now_s)
        req = RequestZone.Request()
        req.robot = self._robot_name
        self._req_sent_s = now_s
        self._req_future = self._req_client.call_async(req)
        self._req_future.add_done_callback(self._on_zone_request_done)

    def _send_zone_release(self, now_s: float) -> None:
        """Dispatches an async /zone/release call to Perry's arbiter."""
        if not self._rel_client.service_is_ready():
            return

        delivered, color = self._zone_client.pop_release()
        req = ReleaseZone.Request()
        req.robot = self._robot_name
        req.delivered = delivered
        req.color = color
        self._rel_sent = (delivered, color)
        self._rel_sent_s = now_s
        self._rel_future = self._rel_client.call_async(req)
        self._rel_future.add_done_callback(self._on_zone_release_done)

    def _on_zone_request_done(self, future) -> None:
        """Handles the async /zone/request response."""
        if future is not self._req_future:
            return

        self._req_future = None
        self._req_sent_s = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Zone request failed: {exc}")
            return

        if response.granted:
            self._zone_client.update(occupant=self._robot_name)
        elif response.holder:
            self._zone_client.update(occupant=str(response.holder))

    def _on_zone_release_done(self, future) -> None:
        """Handles the async /zone/release response."""
        if future is not self._rel_future:
            return

        self._rel_future = None
        self._rel_sent_s = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Zone release failed: {exc}")
            return

        if response.ok and self._zone_client.holds_zone:
            self._zone_client.update(occupant="")

    def _build_web_server(self) -> CollectorWebServer | None:
        """Starts the embedded HTTP controller if web_port > 0."""
        port = int(self.get_parameter("web_port").value)
        if port <= 0:
            return None

        host = str(self.get_parameter("web_host").value)
        server = CollectorWebServer(
            host=host,
            port=port,
            get_status=self._web_status_payload,
            on_cmd=self._on_web_cmd,
            on_mode=self._on_web_mode,
        )
        self.get_logger().info(f"Web controller listening on http://{host}:{port}")
        return server

    def _on_web_cmd(
        self,
        vx_ratio: float,
        vy_ratio: float,
        wz_ratio: float,
    ) -> None:
        """Handles a manual velocity command sent by the browser UI."""
        self._manual.set_normalized_cmd(
            vx_ratio,
            vy_ratio,
            wz_ratio,
            now_s=time.monotonic(),
        )

    def _on_web_mode(self, auto_enabled: bool) -> None:
        """Switches between AUTO and MANUAL modes from the browser UI."""
        self._manual.set_auto(auto_enabled)
        self._hal.stop(time.monotonic())

    def _web_status_payload(self) -> dict[str, object]:
        """Builds JSON status telemetry for the browser controller."""
        now_s = time.monotonic()
        status = self._hal.status(now_s)
        pose = self._world_model.own_pose(now_s)
        age_s = self._world_model.world_age_s(now_s)
        pose_text = (
            f"({pose.x_m:.2f}, {pose.y_m:.2f}, {pose.theta_rad:.2f})"
            if pose is not None
            else "no camera"
        )
        age_text = f"{age_s:.2f} s" if age_s is not None else "none"
        return {
            "robot_name": self._robot_name,
            "auto_enabled": self._manual.auto_enabled,
            "hal_mode": status.mode.value,
            "fsm_state": self._fsm.state.value,
            "line_detected": status.line_detected,
            "pose_text": pose_text,
            "camera_age_text": age_text,
            "target_cube_id": self._fsm.target_cube_id or 0,
        }

    def shutdown_hal(self) -> None:
        """Brakes motors and closes GPIO/I2C and web server handles on teardown."""
        if self._web_server is not None:
            self._web_server.close()
        self._hal.close()


def main(args: list[str] | None = None) -> None:
    """Entry point for collector_node."""
    rclpy.init(args=args)
    node = CollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
