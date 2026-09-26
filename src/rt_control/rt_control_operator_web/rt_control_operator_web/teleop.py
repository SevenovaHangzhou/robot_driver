"""Chassis drive arbitration and command smoothing without ROS or web dependencies.

The arbiter owns the single control lease, the operator's stick/yaw target, the
selected gear, the server-side input watchdog and the jerk-limited (S-curve)
velocity profile that is actually published. Every public method is
thread-safe because the web event loop and the ROS publish timer call it from
different threads.

Why the profile lives here: ``swerve_driver`` stops the drives in the same
cycle when it receives a zero Twist, so a released stick must be ramped down
by the command producer to keep the handling smooth.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Callable, Mapping, Optional, Sequence

# ELECTRI-109 agreed interaction timing (software values, not hardware facts).
CLIENT_MIN_SEND_PERIOD_S = 0.033
CLIENT_HEARTBEAT_PERIOD_S = 0.05
COMMAND_TIMEOUT_S = 0.2
PUBLISH_RATE_HZ = 50.0
MAX_TICK_DT_S = 0.1
# Number of explicit zero commands emitted after motion and lease have ended.
TRAILING_ZERO_COUNT = 3

# Stick shaping (operator feel; agreed in ELECTRI-109).
STICK_DEADZONE = 0.10
STICK_EXPO = 2.0
STICK_SNAP_DEG = 8.0
# Largest accepted raw stick magnitude (client rounding slack above the unit disc).
STICK_MAX_INPUT = 1.01

_EPSILON = 1e-9
_SNAP_ERROR = 1e-4

ZERO = (0.0, 0.0, 0.0)


class TeleopError(Exception):
    """A rejected operator request with a stable machine-readable code."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Gear:
    linear: float
    angular: float


@dataclass(frozen=True)
class AxisLimits:
    accel: float
    decel: float
    stop_decel: float
    jerk: float


@dataclass(frozen=True)
class MotionLimits:
    linear: AxisLimits
    angular: AxisLimits


def _positive(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value) and value > 0.0


def validate_gears(gears: Mapping[str, Gear]) -> dict[str, Gear]:
    if not gears:
        raise ValueError("at least one gear is required")
    checked: dict[str, Gear] = {}
    for name, gear in gears.items():
        if not name or not name.isidentifier():
            raise ValueError(f"invalid gear name: {name!r}")
        for label, value in (("linear", gear.linear), ("angular", gear.angular)):
            if not _positive(value):
                raise ValueError(f"gear {name}.{label} must be finite and positive")
        checked[name] = Gear(float(gear.linear), float(gear.angular))
    return checked


def validate_limits(limits: MotionLimits) -> MotionLimits:
    for axis_name, axis in (("linear", limits.linear), ("angular", limits.angular)):
        for label in ("accel", "decel", "stop_decel", "jerk"):
            if not _positive(getattr(axis, label)):
                raise ValueError(f"{axis_name}_{label} must be finite and positive")
        if axis.stop_decel < axis.decel:
            raise ValueError(f"{axis_name}_stop_decel must not be below {axis_name}_decel")
    return limits


def shape_stick(x: float, y: float) -> tuple[float, float]:
    """Deadzone, quadratic magnitude curve and axis snapping on a unit-disc stick.

    ``x`` is forward (+base_link X) and ``y`` is left (+base_link Y).
    """
    magnitude = math.hypot(x, y)
    if magnitude <= STICK_DEADZONE:
        return 0.0, 0.0
    magnitude = min(magnitude, 1.0)
    angle = math.atan2(y, x)
    snap = math.radians(STICK_SNAP_DEG)
    for axis in (0.0, math.pi / 2, math.pi, -math.pi / 2, -math.pi):
        if abs(angle - axis) <= snap:
            angle = axis
            break
    scaled = ((magnitude - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)) ** STICK_EXPO
    out_x = scaled * math.cos(angle)
    out_y = scaled * math.sin(angle)
    # Remove floating residue from snapped axes so a pure crab is exactly pure.
    return (0.0 if abs(out_x) < _EPSILON else out_x, 0.0 if abs(out_y) < _EPSILON else out_y)


class JerkLimitedFollower:
    """Vector S-curve follower: bounded acceleration and jerk toward a target.

    The acceleration magnitude tapers as sqrt(2 * jerk * error) (with one step
    of discrete look-ahead) so the profile
    reaches the target with acceleration approaching zero instead of a step.
    Limiting the vector (not each axis) keeps the travel direction while
    slowing down.
    """

    def __init__(self, dimensions: int) -> None:
        self.value = [0.0] * dimensions
        self.accel = [0.0] * dimensions

    def reset(self) -> None:
        self.value = [0.0] * len(self.value)
        self.accel = [0.0] * len(self.accel)

    def is_zero(self) -> bool:
        return all(v == 0.0 for v in self.value)

    def step(self, target: Sequence[float], dt: float, limits: AxisLimits, stopping: bool) -> None:
        error = [t - v for t, v in zip(target, self.value)]
        error_norm = math.sqrt(sum(e * e for e in error))
        accel_norm = math.sqrt(sum(a * a for a in self.accel))
        if error_norm < _SNAP_ERROR:
            self.value = list(target)
            self.accel = [0.0] * len(self.accel)
            return
        target_norm = math.sqrt(sum(t * t for t in target))
        value_norm = math.sqrt(sum(v * v for v in self.value))
        jerk = limits.jerk
        if stopping:
            peak = limits.stop_decel
            # The fast stop scales jerk with the deceleration ratio; otherwise
            # the jerk bound, not the deceleration, would dominate the stop.
            jerk = limits.jerk * limits.stop_decel / limits.decel
        elif target_norm > value_norm + _EPSILON:
            peak = limits.accel
        else:
            peak = limits.decel
        # Discrete braking look-ahead: reserve one step of travel so the
        # acceleration can unwind within the jerk bound before the target.
        braking_error = max(error_norm - accel_norm * dt, 0.0)
        desired_norm = min(peak, math.sqrt(2.0 * jerk * braking_error))
        desired = [e / error_norm * desired_norm for e in error]
        delta = [d - a for d, a in zip(desired, self.accel)]
        delta_norm = math.sqrt(sum(d * d for d in delta))
        max_delta = jerk * dt
        if delta_norm > max_delta:
            delta = [d * max_delta / delta_norm for d in delta]
        self.accel = [a + d for a, d in zip(self.accel, delta)]
        accel_norm = math.sqrt(sum(a * a for a in self.accel))
        if accel_norm > peak:
            self.accel = [a * peak / accel_norm for a in self.accel]
        new_value = [v + a * dt for v, a in zip(self.value, self.accel)]
        remaining = [t - n for t, n in zip(target, new_value)]
        if sum(r * e for r, e in zip(remaining, error)) <= 0.0:
            new_value = list(target)
            self.accel = [0.0] * len(self.accel)
        self.value = new_value


@dataclass(frozen=True)
class DriveSnapshot:
    lease_holder: Optional[str]
    gear: str
    target: tuple[float, float, float]
    command: tuple[float, float, float]
    moving: bool
    stopping_fast: bool
    last_stop_reason: str


class DriveArbiter:
    def __init__(
        self,
        gears: Mapping[str, Gear],
        limits: MotionLimits,
        clock: Callable[[], float],
        command_timeout: float = COMMAND_TIMEOUT_S,
    ) -> None:
        if not math.isfinite(command_timeout) or command_timeout <= 0.0:
            raise ValueError("command_timeout must be finite and positive")
        self._gears = validate_gears(gears)
        self._limits = validate_limits(limits)
        self._clock = clock
        self._timeout = command_timeout
        self._lock = threading.Lock()
        self._holder: Optional[str] = None
        self._gear = next(iter(self._gears))
        self._target = ZERO
        self._last_input = -math.inf
        self._linear = JerkLimitedFollower(2)
        self._angular = JerkLimitedFollower(1)
        self._stopping_fast = False
        self._last_tick: Optional[float] = None
        self._trailing_zeros = 0
        self._stop_reason = "idle"

    @property
    def gears(self) -> dict[str, Gear]:
        return dict(self._gears)

    # --- lease -----------------------------------------------------------
    def acquire(self, session: str) -> None:
        with self._lock:
            if self._holder is not None and self._holder != session:
                raise TeleopError("lease_busy", "另一个终端正在控制")
            if self._holder is None:
                self._holder = session
                self._target = ZERO
                self._stop_reason = "lease_acquired"

    def release(self, session: str) -> None:
        with self._lock:
            if self._holder == session:
                self._end_lease_locked("lease_released", fast=False)

    def disconnect(self, session: str) -> None:
        with self._lock:
            if self._holder == session:
                self._end_lease_locked("client_disconnected", fast=False)

    def soft_stop(self, reason: str = "soft_stop") -> None:
        """Fast ramp to zero and revoke any lease; allowed for every authenticated client."""
        with self._lock:
            self._end_lease_locked(reason, fast=True)
            self._trailing_zeros = TRAILING_ZERO_COUNT

    # --- operator input --------------------------------------------------
    def drive(self, session: str, x: float, y: float, yaw: int) -> None:
        with self._lock:
            if self._holder != session:
                raise TeleopError("lease_required", "未持有控制权")
            # Allow client rounding slack; shape_stick clamps the magnitude to 1.
            if not (math.isfinite(x) and math.isfinite(y)) or math.hypot(x, y) > STICK_MAX_INPUT:
                self._zero_target_locked("invalid_input")
                raise TeleopError("invalid_input", "摇杆输入超出范围")
            if yaw not in (-1, 0, 1):
                self._zero_target_locked("invalid_input")
                raise TeleopError("invalid_input", "转向输入无效")
            sx, sy = shape_stick(x, y)
            if (sx != 0.0 or sy != 0.0) and yaw != 0:
                self._zero_target_locked("mixed_input_rejected")
                raise TeleopError("mixed_input_rejected", "平移与原地转向不能同时进行")
            gear = self._gears[self._gear]
            self._target = (sx * gear.linear, sy * gear.linear, yaw * gear.angular)
            self._last_input = self._clock()
            self._stopping_fast = False
            if self._target != ZERO:
                self._stop_reason = ""
            elif not self._linear.is_zero() or not self._angular.is_zero():
                self._stop_reason = "stick_released"

    def halt(self, session: str) -> None:
        with self._lock:
            if self._holder == session:
                self._zero_target_locked("stick_released")

    def set_gear(self, session: str, gear: str) -> None:
        with self._lock:
            if self._holder != session:
                raise TeleopError("lease_required", "未持有控制权")
            if gear not in self._gears:
                raise TeleopError("unknown_gear", f"未知档位: {gear}")
            if self._moving_locked():
                raise TeleopError("gear_change_while_moving", "停车后才能换档")
            self._gear = gear

    # --- publish side ----------------------------------------------------
    def tick(self) -> Optional[tuple[float, float, float]]:
        """Advance the profile and return the command to publish, or None."""
        with self._lock:
            now = self._clock()
            dt = 0.0 if self._last_tick is None else now - self._last_tick
            self._last_tick = now
            dt = min(max(dt, 0.0), MAX_TICK_DT_S)
            if (
                self._holder is not None
                and self._target != ZERO
                and now - self._last_input > self._timeout
            ):
                self._end_lease_locked("command_timeout", fast=False)
            if dt > 0.0:
                limits = self._limits
                self._linear.step(self._target[:2], dt, limits.linear, self._stopping_fast)
                self._angular.step(self._target[2:], dt, limits.angular, self._stopping_fast)
            command = self._command_locked()
            if command == ZERO:
                self._stopping_fast = False
            if self._holder is not None or command != ZERO:
                if self._holder is None:
                    self._trailing_zeros = TRAILING_ZERO_COUNT
                return command
            if self._trailing_zeros > 0:
                self._trailing_zeros -= 1
                return ZERO
            return None

    def snapshot(self) -> DriveSnapshot:
        with self._lock:
            return DriveSnapshot(
                lease_holder=self._holder,
                gear=self._gear,
                target=self._target,
                command=self._command_locked(),
                moving=self._moving_locked(),
                stopping_fast=self._stopping_fast,
                last_stop_reason=self._stop_reason,
            )

    # --- internals (lock held) --------------------------------------------
    def _command_locked(self) -> tuple[float, float, float]:
        vx, vy = self._linear.value
        (wz,) = self._angular.value
        return (vx, vy, wz)

    def _moving_locked(self) -> bool:
        return self._target != ZERO or self._command_locked() != ZERO

    def _zero_target_locked(self, reason: str) -> None:
        self._target = ZERO
        self._stop_reason = reason

    def _end_lease_locked(self, reason: str, fast: bool) -> None:
        had_lease = self._holder is not None
        self._holder = None
        self._zero_target_locked(reason)
        if fast:
            self._stopping_fast = True
        if had_lease or fast:
            self._trailing_zeros = TRAILING_ZERO_COUNT
