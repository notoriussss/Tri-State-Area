"""Onboard HAL safety supervisor implementing black-line stop and 300 ms watchdog."""

from dataclasses import dataclass

from fleet_core.constants import (
    DEFAULT_BATTERY_V,
    DEFAULT_MAX_WHEEL_SPEED_MPS,
    HAL_WATCHDOG_TIMEOUT_S,
    HALF_TRACK_M,
    HALF_WHEELBASE_M,
    MS_PER_SECOND,
    WHEEL_RADIUS_M,
    RobotMode,
)
from fleet_core.hal_line import LineSensorBackend
from fleet_core.hal_motors import MotorBackend
from fleet_core.kinematics import (
    max_wheel_rad_s_from_linear,
    wheel_ratios,
)
from fleet_core.types import (
    MecanumGeometry,
    RobotStatusSnapshot,
    VelocityCmd,
)

DEFAULT_MECANUM_GEOMETRY: MecanumGeometry = MecanumGeometry(
    wheel_radius_m=WHEEL_RADIUS_M,
    half_wheelbase_m=HALF_WHEELBASE_M,
    half_track_m=HALF_TRACK_M,
)


@dataclass(frozen=True)
class HalGuardConfig:
    """Safety and kinematic parameters for the onboard HAL supervisor."""

    geometry: MecanumGeometry = DEFAULT_MECANUM_GEOMETRY
    max_wheel_speed_mps: float = DEFAULT_MAX_WHEEL_SPEED_MPS
    watchdog_timeout_s: float = HAL_WATCHDOG_TIMEOUT_S
    battery_v: float = DEFAULT_BATTERY_V


class HalGuard:
    """Wraps motors and line sensor with local black-line and 300 ms watchdog reflexes.

    All timestamps must come from the local monotonic clock on the same Pi.
    """

    def __init__(
        self,
        motors: MotorBackend,
        line_sensor: LineSensorBackend,
        config: HalGuardConfig | None = None,
    ) -> None:
        """Initializes the HAL supervisor with all motors stopped."""
        self._motors = motors
        self._line_sensor = line_sensor
        self._config = config or HalGuardConfig()
        self._max_wheel_rad_s = max_wheel_rad_s_from_linear(
            self._config.max_wheel_speed_mps,
            self._config.geometry.wheel_radius_m,
        )
        self._last_cmd_s: float | None = None
        self._line_detected: bool = False
        self._escaping_line: bool = False
        self._mode: RobotMode = RobotMode.STOPPED_WATCHDOG
        self._motors.stop()

    def drive(
        self,
        cmd: VelocityCmd,
        now_s: float,
        *,
        allow_line_escape: bool = False,
    ) -> RobotStatusSnapshot:
        """Applies a local velocity command subject to black-line safety reflexes.

        When the bottom sensor detects black tape, normal motion is stopped
        immediately. If allow_line_escape is True and the command does not drive
        forward (vx_mps <= 0.0), the inward escape pulse is permitted while
        still reporting STOPPED_LINE until the tape clears.

        Args:
            cmd: Body-frame velocity command in REP 103 units.
            now_s: Current local monotonic time in seconds.
            allow_line_escape: Permits non-forward escape motion off the border tape.

        Returns:
            Updated RobotStatusSnapshot reflecting the applied safety state.
        """
        self._last_cmd_s = now_s
        self._line_detected = self._line_sensor.is_black()

        if self._line_detected:
            is_valid_escape = allow_line_escape and cmd.vx_mps <= 0.0
            self._mode = RobotMode.STOPPED_LINE
            self._escaping_line = is_valid_escape
            if not is_valid_escape:
                self._motors.stop()
                return self.status(now_s)

            self._apply_velocity(cmd)
            return self.status(now_s)

        self._escaping_line = False
        self._mode = RobotMode.RUN
        self._apply_velocity(cmd)
        return self.status(now_s)

    def tick(self, now_s: float) -> RobotStatusSnapshot:
        """Enforces the 300 ms local command watchdog and black-line stop reflex.

        Args:
            now_s: Current local monotonic time in seconds.

        Returns:
            Current RobotStatusSnapshot after safety enforcement.
        """
        self._line_detected = self._line_sensor.is_black()
        is_stale = self._is_watchdog_expired(now_s)

        if is_stale:
            self._escaping_line = False
            self._mode = RobotMode.STOPPED_WATCHDOG
            self._motors.stop()
            return self.status(now_s)

        if self._line_detected and not self._escaping_line:
            self._mode = RobotMode.STOPPED_LINE
            self._motors.stop()

        return self.status(now_s)

    def stop(self, now_s: float | None = None) -> RobotStatusSnapshot:
        """Brakes all motors immediately and clears any active escape flag.

        Args:
            now_s: Optional local monotonic timestamp for computing cmd_age_ms.

        Returns:
            RobotStatusSnapshot after stopping.
        """
        self._escaping_line = False
        self._motors.stop()
        if self._line_detected:
            self._mode = RobotMode.STOPPED_LINE
        elif now_s is not None and self._is_watchdog_expired(now_s):
            self._mode = RobotMode.STOPPED_WATCHDOG

        return self.status(now_s)

    def line(self) -> bool:
        """Reads and returns whether the bottom sensor currently detects black tape."""
        self._line_detected = self._line_sensor.is_black()
        return self._line_detected

    def status(self, now_s: float | None = None) -> RobotStatusSnapshot:
        """Builds a RobotStatusSnapshot without changing motor outputs.

        Args:
            now_s: Optional local monotonic timestamp in seconds.

        Returns:
            Immutable status telemetry snapshot.
        """
        cmd_age_ms = self._compute_cmd_age_ms(now_s)
        return RobotStatusSnapshot(
            line_detected=self._line_detected,
            battery_v=self._config.battery_v,
            cmd_age_ms=cmd_age_ms,
            mode=self._mode,
        )

    def close(self) -> None:
        """Brakes motors and closes both hardware backends."""
        self._escaping_line = False
        self._motors.close()
        self._line_sensor.close()

    def _apply_velocity(self, cmd: VelocityCmd) -> None:
        """Converts body velocity to wheel ratios and writes them to the motors."""
        ratios = wheel_ratios(
            cmd,
            self._config.geometry,
            self._max_wheel_rad_s,
        )
        self._motors.set(ratios)

    def _is_watchdog_expired(self, now_s: float) -> bool:
        """Returns True when no command was received within watchdog_timeout_s."""
        if self._last_cmd_s is None:
            return True

        elapsed_s = now_s - self._last_cmd_s
        return elapsed_s > self._config.watchdog_timeout_s

    def _compute_cmd_age_ms(self, now_s: float | None) -> int:
        """Returns milliseconds since the last local command, or 0 when unknown."""
        if self._last_cmd_s is None or now_s is None:
            return 0

        age_s = max(
            0.0,
            now_s - self._last_cmd_s,
        )
        return round(age_s * MS_PER_SECOND)
