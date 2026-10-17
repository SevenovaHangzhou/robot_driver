"""Arm console core: admission checks, planning, execution state. No ROS imports.

The console is a FollowJointTrajectory client. It never writes drive words, never
switches controllers and never enables on its own. Every check runs on the server;
the browser only proposes targets.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Callable, Mapping, Optional, Protocol, Sequence

from .model import ArmModel
from .poses import PoseError, PoseStore
from .trajectory import Limits, Plan, TrajectoryPoint, plan_move, plan_stop

LEASE_TIMEOUT_S = 3.0
STATE_STALE_S = 2.5


class ConsoleError(Exception):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Gear:
    name: str
    label: str
    limits: Limits  # rad units


@dataclass(frozen=True)
class ConsoleConfig:
    gears: tuple[Gear, ...]
    limit_margin: float  # rad kept inside each Robot Model position limit
    max_feedback_age: float  # s
    start_tolerance: float  # rad; must stay below the JTC start-consistency tolerance
    sample_period: float  # s between trajectory points
    stop_lead: float  # s; the stop trajectory joins the running one this far ahead
    start_delay: float  # s; trajectory t = 0 is this far after the send time

    def gear(self, name: str) -> Gear:
        for gear in self.gears:
            if gear.name == name:
                return gear
        raise ConsoleError("bad_gear", f"未知档位：{name}")


def config_from_mapping(document: Mapping) -> ConsoleConfig:
    radians = math.radians
    gears = tuple(
        Gear(
            name,
            str(item["label"]),
            Limits(
                radians(float(item["velocity_deg_s"])),
                radians(float(item["acceleration_deg_s2"])),
                radians(float(item["jerk_deg_s3"])),
            ),
        )
        for name, item in document["gears"].items()
    )
    if not gears:
        raise ValueError("at least one gear is required")
    config = ConsoleConfig(
        gears=gears,
        limit_margin=radians(float(document["limit_margin_deg"])),
        max_feedback_age=float(document["max_feedback_age_s"]),
        start_tolerance=radians(float(document["start_tolerance_deg"])),
        sample_period=float(document["sample_period_s"]),
        stop_lead=float(document["stop_lead_s"]),
        start_delay=float(document["start_delay_s"]),
    )
    for name in ("sample_period", "stop_lead", "start_delay"):
        if not 0.0 < getattr(config, name) <= 0.1:
            raise ValueError(f"{name}_s must be in (0, 0.1]")
    if config.limit_margin < 0.0 or config.max_feedback_age <= 0.0:
        raise ValueError("limit_margin_deg must be >= 0 and max_feedback_age_s > 0")
    return config


class Driver(Protocol):
    """Implemented by the ROS bridge.

    ``start`` is the monotonic time of trajectory t = 0. The driver stamps the
    trajectory with it, so a stop can continue the running motion on the same
    timeline instead of the controller's receive time.
    """

    def send(
        self,
        joints: Sequence[str],
        points: Sequence[TrajectoryPoint],
        done: Callable[[bool, str], None],
        start: float,
    ) -> None: ...

    def cancel(self) -> None: ...


@dataclass
class _Execution:
    kind: str  # "move" | "stop"
    plan: Plan
    gear: Gear
    started: float  # monotonic time of trajectory t = 0 (the stamp)
    time_offset: float = 0.0  # trajectory time of ``plan`` t = 0

    def plan_time(self, now: float) -> float:
        return max(now - self.started - self.time_offset, 0.0)

    label: str = ""


@dataclass
class Robot:
    """Latest robot-side facts, written by the ROS bridge."""

    positions: dict[str, float] = field(default_factory=dict)
    feedback_at: float = -math.inf
    enable_state: str = "DISCONNECTED"
    enable_stage: str = ""
    enable_at: float = -math.inf
    controller_active: bool = False
    controller_at: float = -math.inf
    mock: Optional[bool] = None
    capability: Optional[dict] = None


class ArmConsole:
    def __init__(
        self,
        config: ConsoleConfig,
        controller: str,
        driver: Driver,
        pose_path,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.controller = controller
        self._driver = driver
        self._pose_path = pose_path
        self._clock = clock
        self._lock = threading.RLock()
        self.robot = Robot()
        self.model: Optional[ArmModel] = None
        self.model_error = "等待机器人模型"
        self.poses: Optional[PoseStore] = None
        self._execution: Optional[_Execution] = None
        self._last_result = ""
        self._lease: Optional[str] = None
        self._lease_seen = -math.inf
        self._enable_busy = False

    # ------------------------------------------------------------------ inputs
    def set_model(self, model: ArmModel) -> None:
        with self._lock:
            if self.model is not None and self.model.joints != model.joints:
                raise ConsoleError("model_changed", "机器人模型关节已变化，请重启操作台")
            self.model = model
            self.model_error = ""
            if self.poses is None:
                self.poses = PoseStore(self._pose_path, self.joint_names)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(limit.name for limit in self.model.joints) if self.model else ()

    def update(self, **values) -> None:
        with self._lock:
            for key, value in values.items():
                setattr(self.robot, key, value)

    # ------------------------------------------------------------------ lease
    def touch(self, session: str) -> None:
        with self._lock:
            if self._lease == session:
                self._lease_seen = self._clock()

    def acquire(self, session: str) -> None:
        with self._lock:
            self._expire_lease()
            if self._lease not in (None, session):
                raise ConsoleError("lease_busy", "另一个终端持有控制权")
            self._lease, self._lease_seen = session, self._clock()

    def release(self, session: str) -> None:
        with self._lock:
            if self._lease == session:
                self._lease = None

    def _expire_lease(self) -> None:
        if self._lease is not None and self._clock() - self._lease_seen > LEASE_TIMEOUT_S:
            self._lease = None

    def _require_lease(self, session: str) -> None:
        self._expire_lease()
        if self._lease != session:
            raise ConsoleError("lease_required", "未持有控制权")

    # ------------------------------------------------------------------ checks
    def _fresh(self, at: float, limit: float) -> bool:
        return self._clock() - at <= limit

    def _measured(self) -> tuple[float, ...]:
        if self.model is None:
            raise ConsoleError("model_unavailable", self.model_error)
        if not self._fresh(self.robot.feedback_at, self.config.max_feedback_age):
            raise ConsoleError("feedback_stale", "关节反馈超时")
        try:
            return tuple(self.robot.positions[name] for name in self.joint_names)
        except KeyError as exc:
            raise ConsoleError("feedback_incomplete", f"缺少关节反馈：{exc.args[0]}") from exc

    def _motion_ready(self) -> Optional[str]:
        robot = self.robot
        if robot.capability is None:
            return "运行栈未声明手臂轨迹能力"
        if not self._fresh(robot.enable_at, STATE_STALE_S) or robot.enable_state != "ENABLED":
            return "手臂未使能"
        if not self._fresh(robot.controller_at, STATE_STALE_S) or not robot.controller_active:
            return f"轨迹控制器 {self.controller} 未激活"
        if self._enable_busy:
            return "正在切换使能"
        return None

    def bounds(self) -> list[tuple[float, float]]:
        margin = self.config.limit_margin
        return [(j.lower + margin, j.upper - margin) for j in self.model.joints]

    def _check_target(self, target: Mapping[str, float]) -> tuple[float, ...]:
        if self.model is None:
            raise ConsoleError("model_unavailable", self.model_error)
        if set(target) != set(self.joint_names):
            raise ConsoleError("bad_target", "目标必须包含全部受控关节")
        values = []
        for (lower, upper), name in zip(self.bounds(), self.joint_names):
            value = target[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConsoleError("bad_target", f"{name} 目标不是数字")
            value = float(value)
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ConsoleError(
                    "target_out_of_range",
                    f"{name} 目标 {math.degrees(value):.1f}° 超出允许范围 "
                    f"[{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]",
                )
            values.append(value)
        return tuple(values)

    def _limits(self, gear: Gear, start, target) -> Limits:
        """Gear limits, capped by the Robot Model velocity of every moving joint."""
        velocity = gear.limits.velocity
        for joint, origin, goal in zip(self.model.joints, start, target):
            if abs(goal - origin) > 1e-9:
                velocity = min(velocity, joint.velocity)
        return Limits(velocity, gear.limits.acceleration, gear.limits.jerk)

    # ------------------------------------------------------------------ commands
    def preview(self, target: Mapping[str, float], gear_name: str) -> dict:
        with self._lock:
            gear = self.config.gear(gear_name)
            goal = self._check_target(target)
            start = self._measured()
            plan = plan_move(start, goal, self._limits(gear, start, goal))
            return {
                "duration_s": plan.duration,
                "max_delta_deg": max(math.degrees(abs(g - s)) for s, g in zip(start, goal)),
                "gear": gear.name,
            }

    def execute(
        self, session: str, target: Mapping[str, float], gear_name: str, label: str = ""
    ) -> dict:
        with self._lock:
            self._require_lease(session)
            if self._execution is not None:
                raise ConsoleError("busy", "上一段运动尚未结束")
            reason = self._motion_ready()
            if reason:
                raise ConsoleError("not_ready", reason)
            gear = self.config.gear(gear_name)
            goal = self._check_target(target)
            start = self._measured()
            plan = plan_move(start, goal, self._limits(gear, start, goal))
            if plan.duration <= 0.0:
                raise ConsoleError("no_motion", "目标与当前位置相同")
            started = self._clock() + self.config.start_delay
            execution = _Execution("move", plan, gear, started, 0.0, label)
            self._execution = execution
            self._last_result = ""
        self._send(execution, plan.points(self.config.sample_period))
        return {"duration_s": plan.duration}

    def goto_pose(self, session: str, name: str, gear_name: str) -> dict:
        try:
            values = self._pose_store().get(name)
        except PoseError as exc:
            raise ConsoleError("bad_pose", str(exc)) from exc
        return self.execute(session, dict(zip(self.joint_names, values)), gear_name, name)

    def stop(self) -> dict:
        """Smooth stop on the current line; any authenticated client may request it."""
        with self._lock:
            execution = self._execution
            if execution is None or execution.kind == "stop":
                return {"stopping": execution is not None}
            # Same timeline (stamp) as the move. The stop trajectory starts one sample
            # in the past with the move's own points, so the controller continues inside
            # it and never interpolates from its receive-time state. Braking starts at
            # ``cut``, ``stop_lead`` ahead of now.
            period = self.config.sample_period
            now = execution.plan_time(self._clock())
            cut = now + self.config.stop_lead
            stop = plan_stop(execution.plan, cut, execution.gear.limits)
            stopping = _Execution(
                "stop", stop, execution.gear, execution.started, cut, execution.label
            )
            self._execution = stopping
            count = max(int(math.ceil((cut - max(now - period, 0.0)) / period)), 1)
            prefix = [execution.plan.state(cut - (count - i) * period) for i in range(count)]
        points = [point for point in prefix if point.time >= 0.0] + [
            TrajectoryPoint(
                point.time + cut,
                point.positions,
                point.velocities,
                point.accelerations,
            )
            for point in stop.points(period)
        ]
        self._send(stopping, points)
        return {"stopping": True, "duration_s": stop.duration}

    def _send(self, execution: _Execution, points) -> None:
        def done(success: bool, message: str) -> None:
            with self._lock:
                if self._execution is not execution:
                    return  # superseded (e.g. by a stop)
                self._execution = None
                if execution.kind == "stop":
                    self._last_result = "已停止" if success else f"停止失败：{message}"
                else:
                    self._last_result = "已到达" if success else f"运动未完成：{message}"
            if not success and execution.kind == "stop":
                self._driver.cancel()  # fall back to the controller's hold

        try:
            self._driver.send(self.joint_names, points, done, execution.started)
        except Exception as exc:  # noqa: BLE001 - report, never leave a phantom execution
            done(False, str(exc))
            raise ConsoleError("send_failed", f"轨迹发送失败：{exc}") from exc

    def save_pose(self, session: str, name: str, overwrite: bool) -> None:
        with self._lock:
            self._require_lease(session)
            if self._execution is not None:
                raise ConsoleError("busy", "运动中不能保存位姿")
            positions = self._measured()
        try:
            self._pose_store().save(name, positions, overwrite=overwrite)
        except PoseError as exc:
            raise ConsoleError("bad_pose", str(exc)) from exc

    def delete_pose(self, session: str, name: str) -> None:
        with self._lock:
            self._require_lease(session)
        try:
            self._pose_store().delete(name)
        except PoseError as exc:
            raise ConsoleError("bad_pose", str(exc)) from exc

    def _pose_store(self) -> PoseStore:
        if self.poses is None:
            raise ConsoleError("model_unavailable", self.model_error)
        return self.poses

    def begin_enable(self, session: str) -> None:
        with self._lock:
            self._require_lease(session)
            if self._execution is not None:
                raise ConsoleError("busy", "运动中不能切换使能")
            if self._enable_busy:
                raise ConsoleError("busy", "正在切换使能")
            self._enable_busy = True

    def end_enable(self) -> None:
        with self._lock:
            self._enable_busy = False

    # ------------------------------------------------------------------ status
    def status(self, session: str) -> dict:
        with self._lock:
            self._expire_lease()
            robot, now = self.robot, self._clock()
            feedback_fresh = self._fresh(robot.feedback_at, self.config.max_feedback_age)
            execution = self._execution
            moving = None
            if execution is not None:
                elapsed = execution.plan_time(now)
                moving = {
                    "kind": execution.kind,
                    "label": execution.label,
                    "elapsed_s": elapsed,
                    "duration_s": execution.plan.duration,
                    "target": dict(zip(self.joint_names, execution.plan.target)),
                    "commanded": dict(
                        zip(self.joint_names, execution.plan.state(elapsed).positions)
                    ),
                }
            ready = None if self.model is None else self._motion_ready()
            return {
                "lease": "none"
                if self._lease is None
                else "you"
                if self._lease == session
                else "other",
                "model_error": self.model_error,
                "feedback_fresh": feedback_fresh,
                "positions": {n: robot.positions.get(n) for n in self.joint_names},
                "enable": {
                    "state": robot.enable_state
                    if self._fresh(robot.enable_at, STATE_STALE_S)
                    else "DISCONNECTED",
                    "stage": robot.enable_stage,
                    "busy": self._enable_busy,
                },
                "controller": {
                    "name": self.controller,
                    "active": robot.controller_active
                    and self._fresh(robot.controller_at, STATE_STALE_S),
                },
                "mock": robot.mock,
                "motion_ready": ready is None and feedback_fresh,
                "not_ready_reason": ready or ("" if feedback_fresh else "关节反馈超时"),
                "moving": moving,
                "last_result": self._last_result,
            }

    def static_info(self) -> dict:
        with self._lock:
            return {
                "controller": self.controller,
                "joints": [
                    {"name": j.name, "lower": lo, "upper": hi, "model_velocity": j.velocity}
                    for j, (lo, hi) in zip(self.model.joints, self.bounds())
                ]
                if self.model
                else [],
                "gears": [
                    {
                        "name": g.name,
                        "label": g.label,
                        "velocity_deg_s": math.degrees(g.limits.velocity),
                        "acceleration_deg_s2": math.degrees(g.limits.acceleration),
                        "jerk_deg_s3": math.degrees(g.limits.jerk),
                    }
                    for g in self.config.gears
                ],
                "poses": self.poses.list() if self.poses else [],
            }
