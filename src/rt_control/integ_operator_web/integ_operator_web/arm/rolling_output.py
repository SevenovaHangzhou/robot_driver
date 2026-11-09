"""Rolling publication behind the same arm authority as joint commands.

Mode selection uses the shared arm reservation; hardware enable remains explicit. On disconnect
it stops accepting windows; the controller owns timeout braking.
"""
import math

from .errors import ConsoleError
from .console_types import STATE_STALE_S
from .rolling_hold import holding_identity


def time_ns(value):
    if not isinstance(value, dict) or set(value) != {"sec", "nanosec"}:
        raise ValueError("invalid ROS time")
    sec, nano = value["sec"], value["nanosec"]
    if type(sec) is not int or type(nano) is not int or not 0 <= sec < 2**31 or not 0 <= nano < 10**9:
        raise ValueError("invalid ROS time")
    return sec * 10**9 + nano


class RollingOutput:
    def __init__(self, console, bridge, sender):
        self.console, self.bridge, self.sender = console, bridge, sender
        self.started = False
        self.terminal_seen = False
        self.hold_reference = None

    def snapshot(self):
        result = self.sender.status()
        evidence = self.bridge._readback.evidence
        with evidence.lock:
            state = evidence.rolling
            result["state_age_s"] = (evidence.clock() - evidence.rolling_at
                                     if math.isfinite(evidence.rolling_at) else None)
            result["controller"] = None if state is None else {
                key: getattr(state, key) for key in
                ("epoch", "state", "last_sequence", "last_result", "stop_reason")}
        rejections = getattr(self.bridge, 'rolling_rejections', None)
        result['rejection_detail'] = (rejections.matching(result['controller'], self.console.joint_names)
                                      if rejections else None)
        return result

    def ready(self, *, allow_mode_switch=True):
        c = self.console
        evidence = self.bridge._readback.evidence
        return bool(self._feedback_ready()
                    and ((evidence.rolling_active and not evidence.jtc_active)
                         or (allow_mode_switch and c.modes is not None and c.modes.can_select("rolling")))
                    and not c._mode_busy and c.idle_probe(-math.inf))

    def begin(self, connection, body):
        c = self.console
        with c._lock:
            owner = c._permit_for(connection)
            if owner is None or owner.entry != "jog":
                raise ConsoleError("lease_required", "请先获取双臂控制权")
            c.authority.require(owner)
            identity = (c.model.source_sha256 if c.model else None, list(c.joint_names))
            if (body.get("model_sha256"), body.get("joint_names")) != identity:
                raise ConsoleError("jog_model_mismatch", "模型或双臂轴序与主操作台不一致")
        with c._select_mode(connection, "rolling"):
            with c._lock:
                owner = c._permit_for(connection)
                c.authority.require(owner)
                # The mode reservation is ours here; only controller readiness is checked.
                if not self._controller_ready() or not self.sender.reset():
                    raise ConsoleError("jog_not_ready", "需要已使能、Rolling HOLDING 和完整停稳反馈")
                before = holding_identity(self.snapshot())
            # Keep the mode reservation, but let disconnect/stop invalidate its
            # permit while the read-only service is in flight.
            if self.bridge.rolling_hold is None:
                raise ConsoleError("jog_hold_unavailable", "Rolling 指令保持值查询尚未发现")
            positions = self.bridge.rolling_hold.query(c.joint_names)
            with c._lock:
                c.authority.require(owner)
                if not self._controller_ready() or holding_identity(self.snapshot()) != before:
                    raise ConsoleError("jog_hold_changed", "Rolling 保持状态已变化，未启动运动")
                c._reserve_motion(connection)
                self.hold_reference = {"positions": positions, "identity": before}
                self.started = self.terminal_seen = False
                c._jog_busy = True
                return {"positions": dict(c.robot.positions), "parameters": self.bridge.rolling_parameters,
                        "rolling": self.snapshot(), "hold_reference": self.hold_reference}

    def _feedback_ready(self):
        c, b = self.console, self.bridge
        now = c._clock()
        return (c.model is not None and c.robot.enable_state == "ENABLED" and now - c.robot.enable_at <= STATE_STALE_S
                and now - c.robot.feedback_at <= c.config.max_feedback_age
                and b.rolling_parameters is not None and now - b.rolling_parameters_at <= STATE_STALE_S)

    def _controller_ready(self):
        e = self.bridge._readback.evidence
        return (self._feedback_ready() and e.rolling_active and not e.jtc_active
                and self.console.idle_probe(-math.inf))

    def publish(self, connection, body):
        c = self.console
        with c._lock:
            owner = c._permit_for(connection)
            if owner is None or owner.entry != "jog" or not c._jog_busy:
                raise ConsoleError("lease_required", "末端运动未持有双臂执行资源")
            c.authority.require(owner)
            if c.authority.poll(owner)["phase"] != "moving":
                raise ConsoleError("jog_not_started", "末端运动尚未启动")
            window = body["window"]
            self._validate(window)
            result = self.sender.send(window)
            if result == 0:
                self.started = True
            return {"send_result": result, "rolling": self.snapshot()}

    def _validate(self, window):
        parameters = self.bridge.rolling_parameters
        points = window["points"]
        if type(window["final"]) is not bool or not 2 <= len(points) <= min(256, parameters["buffer_capacity"]):
            raise ValueError("invalid Rolling window")
        start, previous = time_ns(window["replace_from"]), -1
        for point in points:
            at = time_ns(point["time_from_start"])
            if at <= previous or (previous < 0 and at != 0):
                raise ValueError("Rolling knots must increase from zero")
            previous = at
            for key in ("positions", "velocities"):
                values = point[key]
                if len(values) != len(parameters["joints"]) or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in values):
                    raise ValueError("invalid Rolling joint values")
        now = self.sender.status()["ros_time_ns"]
        if start < now + parameters["replace_lead_ms"] * 10**6:
            raise ConsoleError("jog_late", "未来轨迹接缝已过期")
        if start + previous > now + parameters["max_horizon_ms"] * 10**6:
            raise ValueError("Rolling horizon exceeded")
        if window["final"] and any(v != 0 for v in points[-1]["velocities"]):
            raise ValueError("final window must end at rest")
        if not self.started:
            self._validate_start(points)

    def _validate_start(self, points):
        reference = self.hold_reference
        if reference is None or holding_identity(self.snapshot()) != reference["identity"]:
            raise ConsoleError("jog_hold_changed", "首窗发布前 Rolling 保持状态已变化，请重新按键")
        first, second = points[:2]
        dt = time_ns(second["time_from_start"]) / 1e9
        for axis, name in enumerate(self.bridge.rolling_parameters["joints"]):
            if first["positions"][axis] != reference["positions"][name] or first["velocities"][axis] != 0.:
                raise ConsoleError("jog_start_mismatch", f"{name} 首点未对齐控制器保持指令，未发布")
            # Acceleration is linear on the controller's cubic Hermite segment;
            # its exact maximum is at one of the two endpoints.
            dq = second["positions"][axis] - first["positions"][axis]
            velocity = second["velocities"][axis]
            c2, c3 = 3 * dq - dt * velocity, dt * velocity - 2 * dq
            peak = max(abs(2 * c2 / dt**2), abs((2 * c2 + 6 * c3) / dt**2))
            limit = self.bridge.rolling_parameters["joint_acceleration_limits"][axis]
            if not math.isfinite(peak) or peak > limit:
                raise ConsoleError("jog_start_acceleration", f"{name} 首段加速度 {peak:.17g} 超过 {limit:.17g} rad/s²（超出 {peak - limit:.6g}），未发布")

    def stop(self, connection, *, release=False):
        c = self.console
        with c._lock:
            owner = c._permit_for(connection)
            if owner is not None and owner.entry == "jog":
                phase = c.authority.poll(owner)["phase"]
                if phase in ("moving", "stopping"):
                    c.authority.request_stop(owner, release=release)
                elif release:
                    c.authority.release(owner)
        return {"stopping": c._jog_busy}

    def poll(self):
        c = self.console
        if not c._jog_busy:
            return
        phase = self.sender.status()["phase"]
        if phase in (6, 7):
            c._authority_unknown = True
        ended = phase in (3, 4, 5)
        never_sent = not self.started and c.authority.poll()["phase"] == "stopping"
        if (ended or never_sent) and not self.terminal_seen:
            self.terminal_seen = True
            c._jog_busy = False
            c._authority_outcome_at = c._clock()

    def close(self):
        self.sender.close()
