"""Package constants and enums for fleet_core."""

import math
from enum import StrEnum

# Arena geometry constants (meters)
ARENA_WIDTH_M: float = 1.165
ARENA_HEIGHT_M: float = 0.70
ARENA_CENTER_X_M: float = ARENA_WIDTH_M / 2.0
ARENA_CENTER_Y_M: float = ARENA_HEIGHT_M / 2.0
VIRTUAL_BORDER_MARGIN_M: float = 0.11
PRE_ZONE_X_M: float = 0.85
ZONE_GREEN_LINE_X_M: float = 1.165
ZONE_MAX_X_M: float = 1.28
ZONE_MIN_Y_M: float = 0.27
ZONE_MAX_Y_M: float = 0.44
SLOT_MIN_SEPARATION_M: float = 0.015

# Robot chassis and mecanum geometry constants
ROBOT_CHASSIS_LENGTH_M: float = 0.186
ROBOT_CHASSIS_WIDTH_M: float = 0.161
SCOOP_CORRIDOR_WIDTH_M: float = 0.18
WHEEL_RADIUS_M: float = 0.0325
HALF_WHEELBASE_M: float = 0.0605
HALF_TRACK_M: float = 0.0655
DEFAULT_MAX_WHEEL_SPEED_MPS: float = 0.24

# Safety, navigation, and timing constants
ROBOT_SAFETY_RADIUS_M: float = 0.17
PEER_COLLISION_ALERT_M: float = 0.34
RED_TRANSIT_RADIUS_M: float = 0.12
RED_STRICT_RADIUS_M: float = 0.17
HAL_WATCHDOG_TIMEOUT_S: float = 0.30
CAMERA_TIMEOUT_S: float = 0.50
PEER_PLAN_TIMEOUT_S: float = 0.50
ZONE_HEARTBEAT_TIMEOUT_S: float = 2.0
ZONE_LEASE_RENEW_S: float = 20.0
DEADLOCK_TIMEOUT_S: float = 3.0
CONTROL_RATE_HZ: float = 20.0
PLAN_RATE_HZ: float = 5.0
STATUS_RATE_HZ: float = 5.0
MIN_OBSERVATION_CONFIDENCE: float = 0.5

# Numeric and bit-manipulation constants
FULL_TURN_RAD: float = 2.0 * math.pi
FULL_WHEEL_RATIO: float = 1.0
MS_PER_SECOND: float = 1000.0
DUTY_ROUND_DECIMALS: int = 3
BYTE_BITS: int = 8
BYTE_MASK: int = 0xFF
WORD_MASK: int = 0xFFFF

# SunFounder Fusion HAT+ (MCU on I2C bus 1, address 0x17) constants
FUSION_I2C_BUS: int = 1
FUSION_I2C_ADDRESS: int = 0x17
FUSION_PWM_PERIOD: int = 39999
FUSION_PWM_PRESCALER: int = 17
FUSION_REG_PERIOD: int = 0x50
FUSION_REG_PRESCALER: int = 0x40
FUSION_REG_DUTY: int = 0x60
FUSION_CHANNELS_PER_TIMER: int = 4
FUSION_MAX_POWER_PCT: float = 100.0
FUSION_CHANNEL_TO_PORT: dict[tuple[int, int], str] = {
    (11, 10): "M0",
    (9, 8): "M1",
    (6, 7): "M2",
    (4, 5): "M3",
}
LGPIO_CHIP_CANDIDATES: tuple[int, ...] = (
    0,
    4,
)

# Updated Fusion HAT+ PWM channels (M2=FL, M0=FR, M1=RL, M3=RR)
DEFAULT_FL_CHANNELS: tuple[int, int] = (
    6,
    7,
)
DEFAULT_FR_CHANNELS: tuple[int, int] = (
    11,
    10,
)
DEFAULT_RL_CHANNELS: tuple[int, int] = (
    9,
    8,
)
DEFAULT_RR_CHANNELS: tuple[int, int] = (
    4,
    5,
)

# Cokoino CKK0018 HAT (2x DRV8833) GPIO BCM fallback pins
DEFAULT_FL_PINS: tuple[int, int] = (
    24,
    25,
)
DEFAULT_FR_PINS: tuple[int, int] = (
    26,
    16,
)
DEFAULT_RL_PINS: tuple[int, int] = (
    17,
    27,
)
DEFAULT_RR_PINS: tuple[int, int] = (
    22,
    23,
)
DEFAULT_NSLEEP_PINS: tuple[int, int] = (
    12,
    13,
)

# Default wheel spin inversion matching the rewired Fusion HAT+ harness
DEFAULT_FL_INVERTED: bool = True
DEFAULT_FR_INVERTED: bool = False
DEFAULT_RL_INVERTED: bool = False
DEFAULT_RR_INVERTED: bool = True
DEFAULT_PWM_FREQUENCY_HZ: int = 200
DEFAULT_MAX_DUTY_RATIO: float = 0.9

# Bottom TCRT5000 line sensor constants (DO on GPIO 27, AO on Fusion HAT+ ADC A0)
DEFAULT_LINE_SENSOR_PIN: int = 27
DEFAULT_LINE_BLACK_VALUE: int = 1
DEFAULT_LINE_CONFIRM_READS: int = 2
DEFAULT_LINE_ADC_CHANNEL: int = 0
DEFAULT_LINE_BLACK_MV: int = 900
ADC_I2C_BUS: int = 1
ADC_I2C_ADDRESS: int = 0x17
ADC_REG_BASE: int = 0x10
ADC_REG_STRIDE: int = 2
ADC_MAX_COUNTS: int = 4095
ADC_REFERENCE_MV: int = 3300

# Telemetry defaults
DEFAULT_BATTERY_V: float = 0.0

# Holonomic controller and latency compensation constants
LATENCY_PREDICTION_THRESHOLD_S: float = 0.15
DEFAULT_KP_LINEAR: float = 1.2
DEFAULT_KP_ANGULAR: float = 1.8
DEFAULT_MAX_LINEAR_MPS: float = 0.16
DEFAULT_MAX_ANGULAR_RADPS: float = 0.9
DEFAULT_POS_DEADBAND_M: float = 0.02
DEFAULT_ANGLE_DEADBAND_RAD: float = math.radians(5.0)
DEFAULT_LINEAR_ACCEL_MPS2: float = 0.45
DEFAULT_ANGULAR_ACCEL_RADPS2: float = 2.5

# Guided push and obstacle avoidance constants
DEFAULT_APPROACH_OFFSET_M: float = 0.14
DEFAULT_CORRIDOR_HALF_WIDTH_M: float = SCOOP_CORRIDOR_WIDTH_M / 2.0
DEFAULT_BREAK_OFFSETS_M: tuple[float, ...] = (
    0.14,
    0.18,
    0.22,
)
DEFAULT_BREAK_FRACTIONS: tuple[float, ...] = (
    0.25,
    0.5,
    0.75,
)
DEFAULT_BORDER_SWEEP_INWARD_M: float = 0.14
DEFAULT_TRANSIT_DETOUR_MARGIN_M: float = 0.03
MAX_TRANSIT_SEGMENTS: int = 3
POINT_EPSILON_M: float = 1e-6

# Distributed mission cost, lane, and parking constants
DEFAULT_PRE_ZONE_UPPER_Y_M: float = 0.45
DEFAULT_PRE_ZONE_LOWER_Y_M: float = 0.25
DEFAULT_PARK_UPPER_XY: tuple[float, float] = (
    0.18,
    0.52,
)
DEFAULT_PARK_LOWER_XY: tuple[float, float] = (
    0.18,
    0.18,
)
DEFAULT_CROSS_PENALTY_M: float = 0.40
DEFAULT_BORDER_PENALTY_M: float = 0.35
DEFAULT_RED_PENALTY_M: float = 0.30
DEFAULT_HYSTERESIS_BONUS_M: float = 0.15
DEFAULT_PEER_CROSS_THRESHOLD_M: float = 0.22

# Unified state machine, scoop tracking, and recovery constants
DEFAULT_SCOOP_MAX_LATERAL_M: float = 0.08
DEFAULT_SCOOP_MAX_AHEAD_M: float = 0.22
DEFAULT_SCOOP_MIN_AHEAD_M: float = 0.02
DEFAULT_RECOVER_BACKUP_M: float = 0.12
DEFAULT_MAX_RECOVER_RETRIES: int = 2
DEFAULT_STAGE_ARRIVAL_M: float = 0.05
DEFAULT_PRE_ZONE_TOLERANCE_M: float = 0.05
DEFAULT_LINE_ESCAPE_SPEED_MPS: float = 0.10
DEFAULT_LEAVE_SPEED_MPS: float = 0.12
DEFAULT_LEAVE_CLEAR_X_M: float = 0.90
DEFAULT_DELIVER_SLOT_X_M: float = 1.21
DEFAULT_DELIVER_SLOT_Y_M: float = 0.355

# P2P navigation-light traffic constants
TRAFFIC_PROJECTION_HORIZON_S: float = 0.50
TRAFFIC_HEAD_ON_CONE_RAD: float = math.radians(30.0)
TRAFFIC_HEAD_ON_HEADING_TOL_RAD: float = math.radians(45.0)
TRAFFIC_OVERTAKING_CONE_RAD: float = math.radians(45.0)
TRAFFIC_OVERTAKING_HEADING_TOL_RAD: float = math.radians(60.0)
TRAFFIC_CROSSING_MAX_BEARING_RAD: float = math.radians(135.0)
DEFAULT_SIDESTEP_DISTANCE_M: float = 0.18
DEFAULT_SIDESTEP_SPEED_MPS: float = 0.12
TRAFFIC_URGENT_CLEAR_RATIO: float = 0.85
DEFAULT_DEADLOCK_REVERSE_DISTANCE_M: float = 0.15
DEFAULT_DEADLOCK_REVERSE_SPEED_MPS: float = 0.10

# Perry delivery zone protocol and slot-selection constants
ZONE_CALL_TIMEOUT_S: float = 3.0
CUBE_SIZE_M: float = 0.025
ZONE_SLOT_X_CANDIDATES_M: tuple[float, ...] = (
    1.205,
    1.245,
)
ZONE_SLOT_Y_CANDIDATES_M: tuple[float, ...] = (
    0.355,
    0.310,
    0.400,
)


class TrafficSituation(StrEnum):
    """Relative maritime encounter geometry between Phineas and Ferb."""

    CLEAR = "clear"
    HOLD_PRIORITY = "hold_priority"
    SEE_GREEN = "see_green"
    SEE_RED = "see_red"
    HEAD_ON = "head_on"
    OVERTAKING = "overtaking"


class TrafficAction(StrEnum):
    """Action selected by the P2P anti-collision module on each 20 Hz tick."""

    HOLD_COURSE = "hold_course"
    SIDESTEP_STARBOARD = "sidestep_starboard"
    WAIT = "wait"
    DEADLOCK_REVERSE = "deadlock_reverse"


class PushMode(StrEnum):
    """Guided push strategy chosen for a target cube."""

    DIRECT = "direct"
    TWO_STAGE = "two_stage"
    BORDER_SWEEP = "border_sweep"
    UNREACHABLE = "unreachable"


class CubeColor(StrEnum):
    """Cube colors recognized in the arena."""

    BLUE = "blue"
    YELLOW = "yellow"
    RED = "red"


class RobotName(StrEnum):
    """Names of the collector robots."""

    PHINEAS = "phineas"
    FERB = "ferb"


class RobotMode(StrEnum):
    """Operating modes reported by the onboard HAL."""

    RUN = "run"
    STOPPED_WATCHDOG = "stopped_watchdog"
    STOPPED_LINE = "stopped_line"
    MANUAL = "manual"


class PlannerState(StrEnum):
    """Autonomous state machine states executed onboard each robot."""

    IDLE = "IDLE"
    GO_TO_APPROACH = "GO_TO_APPROACH"
    PUSH = "PUSH"
    REQUEST_ZONE = "REQUEST_ZONE"
    DELIVER = "DELIVER"
    LEAVE = "LEAVE"
    PARKED = "PARKED"
    WAIT = "WAIT"
    STOPPED = "STOPPED"
    RECOVER = "RECOVER"

    def to_wire_state(self) -> str:
        """Returns the contract-compatible state string for RobotPlan messages."""
        if self == PlannerState.PARKED:
            return PlannerState.IDLE.value
        return self.value


class MotorDriverType(StrEnum):
    """Supported motor hardware backends."""

    FAKE = "fake"
    GPIO = "gpio"
    FUSION_HAT = "fusion_hat"


class WheelName(StrEnum):
    """Canonical mecanum wheel identifiers."""

    FL = "fl"
    FR = "fr"
    RL = "rl"
    RR = "rr"


WHEEL_ORDER: tuple[WheelName, ...] = (
    WheelName.FL,
    WheelName.FR,
    WheelName.RL,
    WheelName.RR,
)


class VirtualSensorEvent(StrEnum):
    """Event names emitted on /world/sensors."""

    ENTERED_ZONE = "entered:zone"
    LEFT_ZONE = "left:zone"
    NEAR_RED_PREFIX = "near:red"
    NEAR_ROBOT = "near:robot"
