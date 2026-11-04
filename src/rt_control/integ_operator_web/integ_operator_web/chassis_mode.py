"""Readback and admission rules for the in-domain chassis mode endpoint."""

from __future__ import annotations

import time

STALE_S = 2.5
MODES = {1: "navigation", 2: "operation"}


class ChassisModeView:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.configured = None
        self.configured_at = -float("inf")
        self.feedback = None
        self.feedback_at = -float("inf")

    def configure(self, enabled):
        self.configured = enabled
        self.configured_at = self.clock()

    def receive(self, message, age):
        if not 0 <= age < STALE_S:
            self.invalidate()
            return
        sample_at = self.clock() - age
        if sample_at < self.feedback_at:
            return  # A delayed service response must not overwrite newer topic feedback.
        self.feedback = {
            "mode": MODES.get(message.mode, "unknown"),
            "phase": message.phase,
            "ready": message.ready,
            "stationary": message.stationary,
            "imu_valid": message.imu_valid,
            "goal_active": message.goal_active,
            "fault_code": message.fault_code,
        }
        # Replaying an old sample must not extend its validity.
        self.feedback_at = sample_at

    def invalidate(self):
        self.feedback_at = -float("inf")

    def status(self, *, connected, enabled, busy, service_ready):
        now = self.clock()
        config_fresh = now - self.configured_at < STALE_S
        fresh = self.feedback is not None and now - self.feedback_at < STALE_S
        feedback = dict(self.feedback) if fresh else {}
        mode = feedback.get("mode", "unknown")
        # Fixed CSV deployments have no ChassisState endpoint. Parameter readback
        # identifies their velocity entry without claiming a confirmed motor mode.
        fixed = config_fresh and self.configured is False and not fresh
        if fixed:
            mode = "navigation"
        if not config_fresh or self.configured is None:
            reason = "等待控制器模式配置"
        elif not self.configured:
            reason = "当前部署未启用作业模式，需完成标定及切换配置"
        elif not connected or not enabled:
            reason = "请先连接并使能底盘"
        elif not service_ready:
            reason = "作业模式服务不可用"
        elif not fresh:
            reason = "模式反馈过期或尚未收到，等待实际状态"
        elif feedback["fault_code"]:
            reason = f"模式故障：{feedback['fault_code']}"
        elif busy or feedback["goal_active"] or feedback["phase"] not in (0, 5):
            reason = "切换或作业执行中，请等待完成"
        elif not feedback["ready"]:
            reason = "控制器尚未就绪"
        elif mode == "navigation" and not feedback["imu_valid"]:
            reason = "IMU反馈无效或过期，暂不能进入作业模式"
        elif mode == "operation" and not feedback["stationary"]:
            reason = "等待作业执行器停止并确认静止"
        elif mode not in ("navigation", "operation"):
            reason = "实际模式待确认"
        else:
            reason = "导航按速度控制；作业按相对位移任务执行"
        can_switch = (
            config_fresh
            and self.configured is True
            and connected
            and enabled
            and service_ready
            and fresh
            and not busy
            and not feedback.get("fault_code")
            and not feedback.get("goal_active")
            and feedback.get("ready", False)
            and feedback.get("phase") in (0, 5)
            and mode in ("navigation", "operation")
            and (mode != "navigation" or feedback.get("imu_valid", False))
            and (mode != "operation" or feedback.get("stationary", False))
        )
        velocity_allowed = fixed or (
            config_fresh
            and self.configured is True
            and fresh
            and mode == "navigation"
            and feedback.get("ready", False)
            and feedback.get("phase") == 0
            and not feedback.get("fault_code")
            and not feedback.get("goal_active")
        )
        return {
            **feedback,
            "mode": mode,
            "confirmed": fresh,
            "can_switch": bool(can_switch),
            "velocity_allowed": bool(velocity_allowed),
            "message": reason,
        }
