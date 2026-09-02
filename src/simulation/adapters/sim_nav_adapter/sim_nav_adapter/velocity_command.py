"""Frame conversion and source arbitration for simulation velocity commands."""

from dataclasses import dataclass
from enum import Enum
import math


@dataclass(frozen=True)
class PlanarCommand:
    vx: float
    vy: float
    wz: float

    @classmethod
    def zero(cls):
        return cls(0.0, 0.0, 0.0)

    def is_finite(self):
        return all(math.isfinite(value) for value in (self.vx, self.vy, self.wz))

    def is_zero(self, epsilon=1.0e-6):
        return (
            abs(self.vx) <= epsilon
            and abs(self.vy) <= epsilon
            and abs(self.wz) <= epsilon
        )


def global_to_body(command, yaw):
    """Rotate a base-origin planar velocity from the global frame to the body."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return PlanarCommand(
        cos_yaw * command.vx + sin_yaw * command.vy,
        -sin_yaw * command.vx + cos_yaw * command.vy,
        command.wz,
    )


class CommandMode(Enum):
    STOPPED = "stopped"
    CONTROLLER = "controller"
    BEHAVIOR = "behavior"


class VelocityCommandGate:
    """Arbitrate semantic command streams and make all stale states fail closed."""

    def __init__(self, source_timeout_sec, odom_timeout_sec=None):
        if source_timeout_sec <= 0.0:
            raise ValueError("source_timeout_sec must be positive")
        if odom_timeout_sec is None:
            odom_timeout_sec = source_timeout_sec
        if odom_timeout_sec <= 0.0:
            raise ValueError("odom_timeout_sec must be positive")

        self.source_timeout_sec = source_timeout_sec
        self.odom_timeout_sec = odom_timeout_sec
        self.mode = CommandMode.STOPPED
        self.controller_started_sec = None
        self.last_controller_raw_sec = None
        self.last_controller_smoothed_sec = None
        self.last_behavior_sec = None

    def _stop(self):
        self.mode = CommandMode.STOPPED
        self.controller_started_sec = None
        self.last_controller_raw_sec = None
        self.last_controller_smoothed_sec = None
        self.last_behavior_sec = None
        return PlanarCommand.zero()

    def _odom_is_fresh(self, odom_age_sec):
        return (
            odom_age_sec is not None
            and math.isfinite(odom_age_sec)
            and 0.0 <= odom_age_sec <= self.odom_timeout_sec
        )

    def accept_controller_raw(self, command, now_sec):
        if not command.is_finite() or command.is_zero():
            if self.mode == CommandMode.BEHAVIOR:
                return None
            return self._stop()

        # Behavior commands have priority until their source explicitly stops
        # or times out. This prevents delayed controller samples from stealing
        # the chassis while a recovery action is running.
        if self.mode == CommandMode.BEHAVIOR:
            return None

        if self.mode != CommandMode.CONTROLLER:
            self.controller_started_sec = now_sec
            self.last_controller_smoothed_sec = None
        self.mode = CommandMode.CONTROLLER
        self.last_controller_raw_sec = now_sec
        self.last_behavior_sec = None
        return None

    def accept_controller_smoothed(self, command, now_sec, yaw, odom_age_sec):
        if self.mode != CommandMode.CONTROLLER:
            return None
        if (
            not command.is_finite()
            or not math.isfinite(yaw)
            or not self._odom_is_fresh(odom_age_sec)
            or self.last_controller_raw_sec is None
            or now_sec - self.last_controller_raw_sec > self.source_timeout_sec
        ):
            return self._stop()

        self.last_controller_smoothed_sec = now_sec
        return global_to_body(command, yaw)

    def accept_behavior(self, command, now_sec, odom_age_sec):
        if not command.is_finite() or command.is_zero():
            if self.mode == CommandMode.CONTROLLER:
                return None
            return self._stop()
        if not self._odom_is_fresh(odom_age_sec):
            return self._stop()

        self.mode = CommandMode.BEHAVIOR
        self.last_behavior_sec = now_sec
        self.controller_started_sec = None
        self.last_controller_raw_sec = None
        self.last_controller_smoothed_sec = None
        return command

    def poll(self, now_sec, odom_age_sec):
        if self.mode == CommandMode.STOPPED:
            return None
        if not self._odom_is_fresh(odom_age_sec):
            return self._stop()

        if self.mode == CommandMode.CONTROLLER:
            raw_stale = (
                self.last_controller_raw_sec is None
                or now_sec - self.last_controller_raw_sec > self.source_timeout_sec
            )
            if self.last_controller_smoothed_sec is None:
                smoothed_reference = self.controller_started_sec
            else:
                smoothed_reference = self.last_controller_smoothed_sec
            smoothed_stale = (
                smoothed_reference is None
                or now_sec - smoothed_reference > self.source_timeout_sec
            )
            if raw_stale or smoothed_stale:
                return self._stop()
            return None

        if (
            self.last_behavior_sec is None
            or now_sec - self.last_behavior_sec > self.source_timeout_sec
        ):
            return self._stop()
        return None
