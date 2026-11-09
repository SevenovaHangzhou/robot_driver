"""Mock-only execution, tab ownership, command epochs and independent input TTL."""
import secrets
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .errors import JogError
from .model import robot_direction_map

SPEED_OPTIONS_MM_S = (.5, 1, 2, 5, 10, 20, 30, 50)
SPEED_ERROR = "速度必须为 " + "、".join(f"{speed:g}" for speed in SPEED_OPTIONS_MM_S) + " mm/s"


class MockJog:
    mode = "mock"

    def __init__(self, planner, initial_q, heartbeat_timeout=.4, lease_timeout=3., clock=time.monotonic,
                 arms=None, selected_arm='single', execution=None, direction_frame=None):
        if not .1 <= heartbeat_timeout <= 1 or lease_timeout < heartbeat_timeout:
            raise JogError("config_invalid", "保活/控制权超时配置无效")
        self.planner, self.q = planner, planner.chain.validate(initial_q).copy()
        self.arms = {name: (p, p.chain.validate(q).copy()) for name, (p, q) in
                     (arms or {selected_arm: (planner, initial_q)}).items()}
        if selected_arm not in self.arms or any(p.chain.model_hash != planner.chain.model_hash for p, _ in self.arms.values()):
            raise JogError('config_invalid', '左右臂必须来自同一模型，且默认臂必须存在')
        self.selected_arm = selected_arm
        self.direction_frame = direction_frame
        self.direction_maps = {name: robot_direction_map(p.chain.source_xml,
                                direction_frame or p.chain.base, p.chain.base)
                               for name, (p, _) in self.arms.items()}
        self.planner, self.q = self.arms[selected_arm]
        self.q = self.q.copy()
        self.command_velocity = np.zeros_like(self.q)
        self.clock, self.heartbeat_timeout, self.lease_timeout = clock, heartbeat_timeout, lease_timeout
        self.lock = threading.RLock()
        self.owner, self.lease_at, self.input_at = None, 0., 0.
        self.epoch, self.seq, self.armed = "", -1, False
        self.heartbeat_seq = -1
        self.active, self.started, self.planning = None, 0., False
        self.generation = 0
        self.result, self.last_plan = "等待获取控制权", None
        self.external_busy, self.feedback_fresh = False, True
        self.stopping = False
        self.orientation = None
        self.execution = execution

    def _advance(self):
        if self.active is not None:
            elapsed = max(0., self.clock() - self.started)
            self.q, self.command_velocity = self.active.state(elapsed)[:2]
            if getattr(self.active, 'fault', ''):
                self.armed, self.epoch = False, ''
                self.result = self.active.fault
            self.stopping = self.stopping or getattr(self.active, 'stopping', False)
            if getattr(self.active, 'complete', elapsed >= self.active.duration):
                fault = getattr(self.active, 'fault', '')
                self.active = None
                if not fault:
                    self.result = "已停止" if self.stopping else "单步完成"
                self.stopping = False
                self.command_velocity = np.zeros_like(self.q)

    def _stop(self, reason, revoke=True):
        self._advance()
        self.generation += 1
        self.planning = False
        if revoke:
            self.armed = False
            self.epoch = ""
        if self.active is not None and not self.stopping:
            brake = self.active.brake(self.clock() - self.started)
            self.active, self.started = brake, self.clock()
            self.stopping = brake is not None
        self.result = reason

    def tick(self):
        with self.lock:
            self._advance()
            now = self.clock()
            if self.owner is not None and now - self.lease_at > self.lease_timeout:
                self._stop("控制权过期，已退出接管")
                self.owner = None
            elif self.armed and now - self.input_at > self.heartbeat_timeout:
                self._stop("输入保活超时，已退出接管")
            elif self.armed and (not self.feedback_fresh or self.external_busy):
                self._stop("反馈失效或执行资源被占用，已退出接管")

    def state(self, session, client):
        self.tick()
        with self.lock:
            if self.owner == (session, client):
                self.lease_at = self.clock()
            pose, jacobian = self.planner.chain.fk_jacobian(self.q)
            model_joints, model_q = [], []
            for name, (planner, saved_q) in self.arms.items():
                model_joints.extend(planner.chain.names)
                model_q.extend((self.q if name == self.selected_arm else saved_q).tolist())
            return {"mode": "mock", "hardware_connected": False,
                    "direction_frame": self.direction_frame or self.planner.chain.base,
                    "direction_mapping": self.direction_maps[self.selected_arm],
                    "publication": self.execution.publication() if self.execution is not None else {"mode": "step"},
                    "selected_arm": self.selected_arm, "arms": list(self.arms),
                    "model_joint_names": model_joints, "model_q_rad": model_q,
                    "lease": "none" if self.owner is None else "you" if self.owner == (session, client) else "other",
                    "armed": self.armed and self.owner == (session, client),
                    "epoch": self.epoch if self.owner == (session, client) else "",
                    "busy": self.active is not None or self.planning or self.external_busy,
                    "stopping": self.stopping, "feedback_fresh": self.feedback_fresh,
                    "frame": self.planner.chain.base, "tip": self.planner.chain.tip,
                    "model_hash": self.planner.chain.model_hash,
                    "model_identity": self.planner.chain.identity,
                    "tcp_xyz_m": self.planner.chain.tcp[:3, 3].tolist(),
                    "joint_names": list(self.planner.chain.names), "q_rad": self.q.tolist(),
                    "tcp_mm": (pose[:3, 3] * 1000).tolist(),
                    "tcp_command_speed_mm_s": float(np.linalg.norm(jacobian[:3] @ self.command_velocity) * 1000),
                    "rpy_deg": Rotation.from_matrix(pose[:3, :3]).as_euler("xyz", degrees=True).tolist(),
                    "result": self.result, "last_plan": self.last_plan,
                    "step_options_mm": [.5, 1, 2, 5, 10],
                    "speed_options_mm_s": list(SPEED_OPTIONS_MM_S),
                    "heartbeat_timeout_s": self.heartbeat_timeout,
                    "blockers": ["TCP 与水平参考系未实机核验", "未接入全模型碰撞检查", "未接入实机执行仲裁与故障停车契约"],
                    "can_arm": self.feedback_fresh and not self.external_busy}

    def _owner(self, session, body):
        client = body.get("client")
        if not isinstance(client, str) or not 8 <= len(client) <= 128:
            raise JogError("bad_client", "页面身份无效")
        if self.owner != (session, client):
            raise JogError("lease_required", "当前页面未持有控制权")
        self.lease_at = self.clock()

    def _command(self, body, heartbeat=False):
        if not self.armed or not secrets.compare_digest(str(body.get("epoch", "")), self.epoch):
            raise JogError("epoch_expired", "接管代次失效，请重新接管")
        seq = body.get("seq")
        if type(seq) is not int or seq <= (self.heartbeat_seq if heartbeat else self.seq) or seq > 2**53 - 1:
            raise JogError("stale_command", "指令重复或顺序已失效")
        if heartbeat:
            self.heartbeat_seq = seq
        else:
            self.seq = seq

    def dispatch(self, action, session, body):
        self.tick()
        with self.lock:
            if action == "stop":
                # Like the console stop: any authenticated operator can stop.
                self._stop("已请求停止并退出接管")
                return {"stopping": self.stopping}
            if action == "acquire":
                client = body.get("client")
                if not isinstance(client, str) or not 8 <= len(client) <= 128:
                    raise JogError("bad_client", "页面身份无效")
                if self.owner not in (None, (session, client)):
                    raise JogError("lease_busy", "另一个页面持有控制权")
                self.owner, self.lease_at = (session, client), self.clock()
                return {}
            self._owner(session, body)
            if action == "release":
                self._stop("已释放控制权")
                self.owner = None
                return {}
            if action == 'select_arm':
                name = body.get('arm')
                if not isinstance(name, str) or name not in self.arms:
                    raise JogError('bad_arm', '所选机械臂未配置')
                if self.armed or self.active is not None or self.planning or self.external_busy:
                    raise JogError('not_ready', '请先停止并退出键盘接管，再切换机械臂')
                self.arms[self.selected_arm] = (self.planner, self.q.copy())
                self.planner, q = self.arms[name]
                self.q, self.selected_arm = q.copy(), name
                self.generation += 1
                self.epoch, self.seq, self.heartbeat_seq = '', -1, -1
                self.orientation, self.last_plan = None, None
                self.result = '已切换机械臂，请重新开启键盘接管'
                return {'selected_arm': name}
            if action == "arm":
                if self.external_busy or self.active is not None or self.planning or not self.feedback_fresh:
                    raise JogError("not_ready", "运动中、反馈失效或执行资源被占用")
                self.epoch, self.seq, self.armed = secrets.token_urlsafe(18), -1, True
                self.heartbeat_seq = -1
                self.input_at = self.clock()
                self.orientation = self.planner.chain.fk(self.q)[:3, :3].copy()
                return {"epoch": self.epoch}
            self._command(body, heartbeat=action == "heartbeat")
            if action == "halt":
                self._stop("松键减速停止", revoke=False)
                self.epoch, self.seq = secrets.token_urlsafe(18), -1
                self.heartbeat_seq = -1
                self.input_at = self.clock()
                return {"epoch": self.epoch, "stopping": self.stopping}
            self.input_at = self.clock()
            if action == "heartbeat":
                return {}
            if action != "step":
                raise JogError("not_found", "未知点动操作")
            if self.active is not None or self.planning or self.external_busy:
                raise JogError("busy", "上一段尚未结束，不接受排队动作")
            if not self.feedback_fresh:
                raise JogError("feedback_stale", "关节反馈超时")
            if 'robot_direction' in body:
                requested = body['robot_direction']
                if 'direction' in body or not isinstance(requested, str) or requested not in self.direction_maps[self.selected_arm]:
                    raise JogError('bad_direction', '机器人方向无效或与规划轴方向参数冲突')
                direction = self.direction_maps[self.selected_arm][requested]
            else:
                direction = body.get('direction')
            if "speed_mm_s" in body:
                speed_mm_s = body["speed_mm_s"]
                if type(speed_mm_s) not in (float, int) or speed_mm_s not in SPEED_OPTIONS_MM_S:
                    raise JogError("bad_speed", SPEED_ERROR)
                if "step_mm" in body:
                    raise JogError("bad_step", "速度模式不接受步长参数")
                # Distance is an internal planning horizon, independent of speed.
                step_mm = 5.
            else:
                # Keep existing offline step clients compatible.
                step_mm = body.get("step_mm")
                if type(step_mm) not in (float, int) or step_mm not in (.5, 1, 2, 5, 10):
                    raise JogError("bad_step", "步长必须为 0.5、1、2、5 或 10 mm")
                speed_mm_s = self.planner.limits.tcp_speed * 1000
            self.generation += 1
            generation, start = self.generation, self.q.copy()
            self.planning = True
        try:
            plan = self.planner.step(start, direction, step_mm / 1000,
                                     require_collision=False, orientation=self.orientation,
                                     tcp_speed=speed_mm_s / 1000, speed_profile="speed_mm_s" in body)
        except Exception:
            with self.lock:
                if generation == self.generation:
                    self.planning = False
            raise
        with self.lock:
            # Planning must not resurrect a stop, expired input, release, or restart.
            self.tick()
            if generation != self.generation or not self.armed:
                raise JogError("cancelled", "规划期间已停止或接管失效")
            if not np.allclose(self.q, start, atol=1e-10, rtol=0):
                self.planning = False
                raise JogError("start_changed", "规划起点已改变")
            self.planning = False
            if self.execution is not None:
                positions = {joint: float(value) for name, (planner, saved) in self.arms.items()
                             for joint, value in zip(planner.chain.names, self.q if name == self.selected_arm else saved)}
                self.active = self.execution.start(plan, positions)
            else:
                self.active = plan
            self.started = self.clock()
            self.last_plan = {**plan.document(), 'direction': direction}
            self.result = (f"正在通过 Rolling 发布 Mock 轨迹 · {speed_mm_s:g} mm/s" if self.execution is not None
                           else f"正在执行 Mock 轨迹 · {speed_mm_s:g} mm/s")
            return self.last_plan
