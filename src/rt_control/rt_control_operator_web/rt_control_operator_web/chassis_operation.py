"""One confirmed relative action at a time; browser input never owns motor targets."""

import asyncio
import math
import time

from .teleop import TeleopError

LIMIT_FIELDS = (
    "max_translation_velocity",
    "max_translation_acceleration",
    "max_yaw_velocity",
    "max_yaw_acceleration",
    "max_wheel_velocity",
    "max_wheel_acceleration",
    "max_duration",
)
PARAMETERS = [
    "relative." + ("max_goal_duration" if key == "max_duration" else key) for key in LIMIT_FIELDS
]


async def ros_future(future, timeout):
    """Bridge ROS executor callbacks without cancelling an outstanding ROS transaction."""
    loop = asyncio.get_running_loop()
    completion = loop.create_future()

    def done(source):
        def resolve():
            if not completion.done():
                try:
                    completion.set_result(source.result())
                except Exception as exc:
                    completion.set_exception(exc)

        if not loop.is_closed():
            loop.call_soon_threadsafe(resolve)

    future.add_done_callback(done)
    return await asyncio.wait_for(completion, timeout)


def validate_motion(values, limits):
    required = {"dx", "dy", "dyaw_deg", *LIMIT_FIELDS}
    if set(values) != required:
        raise TeleopError("bad_motion", "作业参数不完整")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values.values()):
        raise TeleopError("bad_motion", "作业参数必须是有限数字")
    if math.hypot(values["dx"], values["dy"]) > 0.700 or abs(values["dyaw_deg"]) > 15:
        raise TeleopError("bad_motion", "单次平移合量不超过0.7米，旋转不超过15度")
    bounded = dict(values)
    for field in LIMIT_FIELDS:
        if values[field] <= 0 or (
            values[field] > limits[field]
            and not math.isclose(values[field], limits[field], rel_tol=1e-12)
        ):
            raise TeleopError("bad_motion", f"{field}须大于0且不超过控制器上限{limits[field]:g}")
        bounded[field] = min(values[field], limits[field])
    return {**bounded, "dyaw": math.radians(values["dyaw_deg"])}


class ChassisOperation:
    def __init__(self, client, make_goal, admission, lock, clock=time.monotonic):
        self.client, self.make_goal, self.admission = client, make_goal, admission
        self.lock, self.clock = lock, clock
        self.limits, self.limits_at = {}, -math.inf
        self.running = self.unknown = self.cancel_requested = self.cancel_sent = False
        self.cancel_ok = False
        self.handle = None
        self.sequence = 0
        self.feedback = {}
        self.message = "先切换到作业模式，再配置相对位移"

    def configure(self, values):
        with self.lock:
            self.limits = {}
            if set(values) != set(LIMIT_FIELDS):
                return
            limits = {}
            for key, value in values.items():
                if isinstance(value, list):
                    if len(value) != 4:
                        return
                    if any(
                        type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in value
                    ):
                        return
                    value = min(value)
                if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                    return
                limits[key] = value
            limits["max_translation_velocity"] = min(0.2, limits["max_translation_velocity"])
            self.limits, self.limits_at = limits, self.clock()

    def status(self, mode, connected, enabled, busy):
        with self.lock:
            ready = (
                connected
                and enabled
                and not busy
                and mode.get("confirmed")
                and mode.get("mode") == "operation"
                and mode.get("phase") == 5
                and mode.get("ready")
                and mode.get("stationary")
                and mode.get("imu_valid")
                and not mode.get("fault_code")
                and not mode.get("goal_active")
            )
            limits_fresh = bool(self.limits) and self.clock() - self.limits_at < 2.5
            return {
                "can_execute": bool(
                    ready
                    and limits_fresh
                    and self.client.server_is_ready()
                    and not self.running
                    and not self.unknown
                ),
                "active": self.running,
                "unknown": self.unknown,
                "limits": dict(self.limits) if limits_fresh else {},
                "feedback": dict(self.feedback),
                "message": self.message,
            }

    def receive_state(self, state):
        with self.lock:

            def finite(value):
                return value if math.isfinite(value) else None

            self.feedback = {
                "phase": state.phase,
                "progress": finite(state.progress),
                "actual_dx": finite(state.actual_dx),
                "actual_dy": finite(state.actual_dy),
                "actual_dyaw_deg": finite(math.degrees(state.actual_dyaw)),
                "estimate_valid": bool(
                    state.estimate_valid
                    and all(
                        math.isfinite(v)
                        for v in (state.actual_dx, state.actual_dy, state.actual_dyaw)
                    )
                ),
                "fault_code": state.fault_code,
            }

    async def cancel(self):
        with self.lock:
            if not self.running:
                return {"ok": not self.unknown, "message": self.message}
            self.cancel_requested = True
            self.message = "已请求取消，等待执行器停车及任务结果"
            handle = self.handle
            if handle is None:
                return {"ok": True, "message": self.message}
            if self.cancel_sent:
                return {"ok": self.cancel_ok, "message": self.message}
            self.cancel_sent = True
        try:
            response = await ros_future(handle.cancel_goal_async(), 3)
            if not response.goals_canceling:
                self.message = "取消未获接受，等待任务实际结果"
                return {"ok": False, "message": self.message}
            self.cancel_ok = True
        except Exception:
            self.message = "取消响应未确认，请检查底盘实际状态"
            return {"ok": False, "message": self.message}
        return {"ok": True, "message": self.message}

    async def execute(self, values):
        with self.lock:
            if not self.admission():
                raise TeleopError(
                    "operation_not_ready", "需已使能、确认作业模式、静止及有效IMU，且无其他任务"
                )
            values = validate_motion(values, self.limits)
            self.running = True
            self.cancel_requested = self.cancel_sent = False
            self.cancel_ok = False
            self.handle, self.feedback = None, {}
            self.message = "正在提交作业任务"
            self.sequence += 1
            sequence = self.sequence
        confirmed = False
        pending = None
        try:

            def feedback(message):
                with self.lock:
                    if self.running and self.sequence == sequence:
                        self.receive_state(message.feedback.state)

            pending = self.client.send_goal_async(
                self.make_goal(values), feedback_callback=feedback
            )
            handle = await ros_future(pending, 5)
            if not handle.accepted:
                confirmed = True
                self.message = "控制器拒绝任务，请检查作业准入与运动参数"
                return {"ok": False, "message": self.message}
            with self.lock:
                self.handle = handle
                cancel = self.cancel_requested
                self.message = "作业执行中"
            if cancel:
                await self.cancel()
            result = await ros_future(handle.get_result_async(), values["max_duration"] + 10)
            if result.status not in (4, 5, 6):
                raise RuntimeError("Action result is not terminal")
            ok = result.status == 4 and result.result.code == 0
            self.message = (
                "作业运动完成"
                if ok
                else "作业已取消并结束"
                if result.status == 5 and result.result.code == 1
                else f"作业失败：结果码{result.result.code}，故障码{result.result.state.fault_code}"
            )
            self.receive_state(result.result.state)
            confirmed = True
            return {"ok": ok, "message": self.message}
        except (Exception, asyncio.CancelledError):
            await self.cancel()
            # A late goal acceptance must also be cancelled, even after the web task exits.
            if pending is not None and self.handle is None:

                def cancel_late(future):
                    try:
                        late = future.result()
                        if late.accepted:
                            late.cancel_goal_async()
                    except Exception:
                        pass  # No cancellation confirmation is claimed.

                pending.add_done_callback(cancel_late)
            self.message = "作业结果未确认，已请求取消；请核对控制器状态后重新连接服务"
            raise
        finally:
            with self.lock:
                self.running = False
                self.unknown = not confirmed
                self.handle = None
