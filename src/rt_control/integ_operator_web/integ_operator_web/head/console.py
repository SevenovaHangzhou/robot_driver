"""Head page logic, no ROS: lease, readiness checks and absolute position targets.

The running stack declares the head action (capability ``head_move``, R-IN-16 /rt/head/move);
joint order comes from that controller's ``joints`` parameter and limits from the Robot Model.
A move sends one absolute target; the head motor hardware limits the speed.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Mapping, Optional


class HeadError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


class HeadConsole:
    def __init__(
        self,
        config: Mapping,
        publish: Callable[[list[float]], None],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._margin = math.radians(float(config["limit_margin_deg"]))
        self._max_feedback_age = float(config["max_feedback_age_s"])
        self._state_stale = float(config["state_stale_s"])
        self._publish = publish
        self._clock = clock
        self._lock = threading.RLock()
        self.joints: tuple[str, ...] = ()
        self.limits: dict[str, tuple[float, float]] = {}
        self.model_error = "等待头部控制器与机器人模型"
        self.capability: Optional[dict] = None
        self.positions: dict[str, float] = {}
        self.feedback_at = -math.inf
        self.enable_state = "UNKNOWN"
        self.motion_allowed = False
        self.device_fault = False
        self.enable_at = -math.inf
        self.target: Optional[dict[str, float]] = None
        self._holder: Optional[str] = None

    # ------------------------------------------------------------------ inputs
    def set_model(self, joints, limits: Mapping[str, tuple[float, float]]) -> None:
        with self._lock:
            missing = [joint for joint in joints if joint not in limits]
            if missing:
                self.model_error = f"机器人模型缺少头部关节限位：{', '.join(missing)}"
                return
            self.joints = tuple(joints)
            self.limits = {joint: limits[joint] for joint in joints}
            self.model_error = ""

    def update(self, **values) -> None:
        with self._lock:
            for key, value in values.items():
                setattr(self, key, value)

    # ------------------------------------------------------------------ lease
    @property
    def lease_holder(self) -> Optional[str]:
        return self._holder

    def acquire(self, connection: str) -> None:
        with self._lock:
            if self._holder not in (None, connection):
                raise HeadError("lease_busy", "另一个终端持有头部控制权")
            self._holder = connection

    def release(self, connection: str) -> None:
        with self._lock:
            if self._holder == connection:
                self._holder = None

    def disconnect(self, connection: str) -> None:
        """Closing the holding connection holds the head where it is, then frees the lease."""
        with self._lock:
            if self._holder != connection:
                return
            self._holder = None
            if self._fresh_positions() is not None and self.capability is not None:
                self.hold()

    # ------------------------------------------------------------------ commands
    def _fresh_positions(self) -> Optional[list[float]]:
        if self._clock() - self.feedback_at > self._max_feedback_age:
            return None
        if not self.joints or any(joint not in self.positions for joint in self.joints):
            return None
        return [self.positions[joint] for joint in self.joints]

    def _ready(self) -> None:
        if self.capability is None:
            raise HeadError("capability_missing", "运行栈未声明头部位置能力")
        if not self.joints:
            raise HeadError("model_unavailable", self.model_error)
        fresh = self._clock() - self.enable_at <= self._state_stale
        if not fresh or self.enable_state != "ENABLED":
            raise HeadError("not_enabled", "头部未使能")
        if self.device_fault:
            raise HeadError("device_fault", "头部设备故障锁存，请先复位")
        if not self.motion_allowed:
            raise HeadError("motion_not_allowed", "头部尚未允许运动")

    def move(self, connection: str, target: Mapping[str, float]) -> dict:
        with self._lock:
            if self._holder != connection:
                raise HeadError("lease_required", "未持有头部控制权")
            self._ready()
            if self._fresh_positions() is None:
                raise HeadError("feedback_stale", "头部关节反馈超时")
            if set(target) != set(self.joints):
                raise HeadError("bad_target", "目标必须包含全部头部关节")
            values = []
            for joint in self.joints:
                value = target[joint]
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise HeadError("bad_target", f"{joint} 目标不是数字")
                lower, upper = self.limits[joint]
                lower, upper = lower + self._margin, upper - self._margin
                if not math.isfinite(value) or not lower <= value <= upper:
                    raise HeadError(
                        "target_out_of_range",
                        f"{joint} 目标 {math.degrees(value):.1f}° 超出允许范围 "
                        f"[{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]",
                    )
                values.append(float(value))
            self._publish(values)
            self.target = dict(zip(self.joints, values))
            return {"target": self.target}

    def hold(self) -> dict:
        """Command the measured position; any logged-in client may stop the head."""
        with self._lock:
            if self.capability is None:
                raise HeadError("capability_missing", "运行栈未声明头部位置能力")
            positions = self._fresh_positions()
            if positions is None:
                raise HeadError("feedback_stale", "头部关节反馈超时，无法原地保持")
            self._publish(positions)
            self.target = dict(zip(self.joints, positions))
            return {"target": self.target}

    def status(self, connection: str) -> dict:
        with self._lock:
            now = self._clock()
            holder = self._holder
            return {
                "lease": "none" if holder is None else "you" if holder == connection else "other",
                "capability": self.capability,
                "joints": [
                    {
                        "name": joint,
                        "position": self.positions.get(joint),
                        "lower": self.limits[joint][0] + self._margin,
                        "upper": self.limits[joint][1] - self._margin,
                    }
                    for joint in self.joints
                ],
                "model_error": self.model_error,
                "feedback_fresh": self._fresh_positions() is not None,
                "enable": {
                    "state": self.enable_state
                    if now - self.enable_at <= self._state_stale
                    else "UNKNOWN",
                    "motion_allowed": self.motion_allowed,
                    "device_fault": self.device_fault,
                },
                "target": self.target,
            }
