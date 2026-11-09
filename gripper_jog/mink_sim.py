"""Continuous keyboard targets for the local real2sim Mink/MuJoCo follower.

The simulation worker owns physics. HTTP handlers only change input state under
the same lock. This module creates no ROS nodes or controller clients.
"""
import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

from .control import MockJog, SPEED_OPTIONS_MM_S, SPEED_ERROR
from .errors import JogError
from .model import Chain
from .model_assets import ModelAssets
from .planner import DIRECTIONS


def check_viewer_display():
    """Reject a desktop with no outputs before GLFW waits for window visibility."""
    message = ('MuJoCo 原生窗口没有可用的显示输出；请连接显示器或启用远程桌面的虚拟显示器，'
               '也可追加 --headless 使用网页三维视图')
    # Xwayland can leave GLFW a synthetic primary monitor after all physical
    # outputs disappear. glfwGetMonitors then succeeds but window creation hangs.
    if os.environ.get('DISPLAY') and shutil.which('xrandr'):
        try:
            outputs = subprocess.run(['xrandr', '--listactivemonitors'], capture_output=True,
                                     text=True, timeout=3, env={**os.environ, 'LC_ALL': 'C'})
        except subprocess.TimeoutExpired as exc:
            raise JogError('viewer_display_unavailable', '显示服务无响应，请追加 --headless 使用网页三维视图') from exc
        if outputs.returncode == 0 and outputs.stdout.splitlines()[:1] == ['Monitors: 0']:
            raise JogError('viewer_display_unavailable', message)
    import glfw
    try:
        if not glfw.init() or not glfw.get_monitors():
            raise JogError('viewer_display_unavailable', message)
    finally:
        glfw.terminate()


def build_mink(root, *, clock=time.monotonic, autostart=True, rolling=False,
               gravity_mode='off', gravity_source=None, gravity_cache=None):
    root = Path(root).resolve()
    if not (root / 'real2sim/mink_demo.py').is_file():
        raise JogError('mink_missing', f'未找到 Mink 示例，请用 --real2sim 指定 real2sim_v3：{root}')
    sys.path.insert(0, str(root))
    try:
        demo = importlib.import_module('real2sim.mink_demo')
    except ImportError as exc:
        raise JogError('mink_missing', '缺少 Mink 仿真依赖，请用项目 .venv 执行 pip install -e .') from exc
    finally:
        sys.path.remove(str(root))
    if Path(demo.__file__).resolve() != root / 'real2sim/mink_demo.py':
        raise JogError('model_mismatch', '已加载的 real2sim 来自另一个目录，请重启并检查 --real2sim')
    urdf = root / 'urdf/robot.urdf'
    assets = ModelAssets(urdf, root, allow_relative=True)
    from .gravity import GravityBackend, GravityFollowerMixin, load_static_gravity

    class JogFollower(GravityFollowerMixin, demo.MinkFollower):
        pass

    follower = JogFollower()
    calculator = None
    if gravity_mode == 'static':
        source = gravity_source or root.parent / 'Dynamic_Parameter_Identification/static_gravity'
        kwargs = {} if gravity_cache is None else {'cache': gravity_cache}
        calculator = load_static_gravity(source, urdf, follower.backend.names, **kwargs)
    follower.backend = GravityBackend(follower.backend, gravity_mode, calculator)
    jog_type = MinkJog
    if rolling:
        from .mink_rolling import MinkRollingJog
        jog_type = MinkRollingJog
    jog = jog_type(follower, assets.xml, clock=clock, autostart=autostart)
    jog.managed_viewer = demo.managed_viewer
    return jog, assets


class MinkJog(MockJog):
    acceleration_m_s2 = .05
    max_target_lag_m = .003

    def __init__(self, follower, xml, *, clock=time.monotonic, autostart=True):
        self.follower = follower
        self.backend = follower.backend
        b = self.backend
        arms = {}
        self.indices = {}
        for side in ('left', 'right'):
            chain = Chain(xml, 'arm_carriage', side + '_tool0', margin=0)
            indices = np.array([b.names.index(name) for name in chain.names])
            self.indices[side] = indices
            arms[side] = (SimpleNamespace(chain=chain), b.q[indices])
            if not np.allclose(chain.lower, b.lower[indices], atol=1e-10) or not np.allclose(chain.upper, b.upper[indices], atol=1e-10):
                raise JogError('model_mismatch', 'Mink 场景与显示 URDF 关节限位不同')
            frame = b.data.body(chain.base)
            pose = chain.fk(b.q[indices])
            world_rotation = frame.xmat.reshape(3, 3)
            site = b.data.site(side + '_ee')
            if (not np.allclose(world_rotation @ pose[:3, 3] + frame.xpos, site.xpos, atol=1e-7)
                    or not np.allclose(world_rotation @ pose[:3, :3], site.xmat.reshape(3, 3), atol=1e-7)):
                raise JogError('model_mismatch', 'Mink 场景与显示 URDF 的 TCP 不一致')
        super().__init__(*arms['left'], arms=arms, selected_arm='left', clock=clock,
                         direction_frame='base_link')
        self.direction = None
        self.target_speed = 0.
        self.cart_velocity = np.zeros(3)
        self.realign = False
        self.targets = b.data.mocap_pos.copy()
        self.target_rotations = b.data.mocap_quat.copy()
        self.stopped_event = threading.Event()
        self.worker = None
        self.fault = ''
        self.iterations = 0
        self.last_cycle_s = 0.
        self.motion_start_q = self.q.copy()
        self.motion_origin = self.planner.chain.fk(self.q)[:3, 3].copy()
        self.motion_direction = None
        self.motion_elapsed = 0.
        self.metrics = {}
        self.result = 'Mink 连续控制已就绪，等待获取控制权'
        if autostart:
            self.start()

    def start(self):
        """Start physics after an optional viewer has finished reading the model."""
        with self.lock:
            if self.worker is not None or self.stopped_event.is_set():
                return
            self.worker = threading.Thread(target=self._run, name='mink-physics', daemon=True)
            self.worker.start()

    def _advance(self):
        # Physics never advances on an HTTP request or state read.
        pass

    def _stop(self, reason, revoke=True):
        if self.direction is not None:
            self.realign = True  # Discard target lag; never chase an old target after stop.
        self.direction = None
        self.target_speed = 0.
        self.generation += 1
        self.planning = False
        self.stopping = self.active is not None
        if revoke:
            self.armed, self.epoch = False, ''
        self.result = reason

    def dispatch(self, action, session, body):
        if action != 'velocity':
            if action == 'step':
                raise JogError('continuous_only', 'Mink 使用 velocity 连续控制，不接受分段轨迹')
            with self.lock:
                result = super().dispatch(action, session, body)
                if action == 'arm':
                    self.fault = ''
                elif action == 'select_arm':
                    self.metrics = {}
                    self.motion_elapsed = 0.
                return result
        self.tick()
        with self.lock:
            self._owner(session, body)
            self._command(body)
            if self.active is not None or not self.feedback_fresh:
                raise JogError('not_ready', '请等待停止或反馈恢复')
            requested = body.get('robot_direction')
            mapping = self.direction_maps[self.selected_arm]
            if not isinstance(requested, str) or requested not in mapping or 'direction' in body:
                raise JogError('bad_direction', '请提供机器人本体方向 robot_direction')
            speed = body.get('speed_mm_s')
            if type(speed) not in (int, float) or speed not in SPEED_OPTIONS_MM_S or 'step_mm' in body:
                raise JogError('bad_speed', SPEED_ERROR + '，不接受步长')
            self.input_at = self.clock()
            self.direction = mapping[requested]
            self.motion_direction = self.direction
            self.target_speed = speed / 1000
            self.active, self.stopping, self.realign = True, False, True
            self.motion_start_q = self.q.copy()
            self.motion_origin = self.planner.chain.fk(self.q)[:3, 3].copy()
            self.motion_elapsed = 0.
            self.metrics = {}
            self.fault = ''
            self.result = f'Mink 连续移动 · {speed:g} mm/s'
            return {'continuous': True, 'direction': self.direction, 'speed_mm_s': speed}

    def _sync_feedback(self):
        b = self.backend
        for side, (planner, _) in self.arms.items():
            self.arms[side] = (planner, b.q[self.indices[side]])
        self.planner, q = self.arms[self.selected_arm]
        self.q = q.copy()
        self.command_velocity = self.follower.last_velocity[self.indices[self.selected_arm]].copy()

    def _cycle(self):
        """Advance one fixed simulation interval, also used by deterministic tests."""
        f, b = self.follower, self.backend
        self.tick()  # Expire input before integrating another target increment.
        if self.realign:
            f.align_targets()
            self.targets = b.data.mocap_pos.copy()
            self.target_rotations = b.data.mocap_quat.copy()
            # Freeze the redundant posture at the present state on each press/stop.
            f.configuration.update(b.data.qpos)
            f.posture.set_target_from_configuration(f.configuration)
            self.realign = False
        desired = np.zeros(3)
        if self.direction is not None:
            desired = np.array(DIRECTIONS[self.direction]) * self.target_speed
        change = desired - self.cart_velocity
        size = np.linalg.norm(change)
        if size:
            self.cart_velocity += change * min(1., self.acceleration_m_s2 * f.dt / size)
        mid = f.mocap_ids[self.selected_arm]
        rotation = b.data.body(self.planner.chain.base).xmat.reshape(3, 3)
        self.targets[mid] += rotation @ self.cart_velocity * f.dt
        # Viewer interaction cannot replace keyboard targets or target orientation.
        b.data.mocap_pos[:] = self.targets
        b.data.mocap_quat[:] = self.target_rotations
        f.step()
        self.iterations += 1
        self._sync_feedback()
        error = f.errors()[self.selected_arm]
        if f.last_error or f.collision_blocked or error['position_m'] > self.max_target_lag_m:
            reason = f.last_error or ('避碰约束阻挡，已停止' if f.collision_blocked else '目标跟随误差过大，已停止')
            self.fault = reason
            self._stop(reason)
            self.cart_velocity[:] = 0
            self.realign = True
        if self.active is not None:
            self.motion_elapsed += f.dt
            pose = self.planner.chain.fk(self.q)
            axis = np.array(DIRECTIONS[self.motion_direction])
            offset = pose[:3, 3] - self.motion_origin
            cross = offset - axis * np.dot(axis, offset)
            self.metrics = {
                'max_path_error_mm': max(self.metrics.get('max_path_error_mm', 0.), float(np.linalg.norm(cross) * 1000)),
                'max_orientation_error_deg': max(self.metrics.get('max_orientation_error_deg', 0.), float(np.rad2deg(error['orientation_rad']))),
                'max_joint_delta_deg': float(np.rad2deg(np.max(np.abs(self.q - self.motion_start_q)))),
                'tracking_error_mm': error['position_m'] * 1000,
            }
        actual_speed = np.max(np.abs(b.v))
        if self.stopping and np.linalg.norm(self.cart_velocity) < 1e-10 and actual_speed < 1e-4:
            self.active, self.stopping = None, False

    def _run(self):
        deadline = time.monotonic()
        while not self.stopped_event.is_set():
            start = time.monotonic()
            try:
                with self.lock:
                    self._cycle()
                    self.last_cycle_s = time.monotonic() - start
            except Exception as exc:
                with self.lock:
                    self.fault = f'Mink 控制循环失败：{exc}'
                    self._stop(self.fault)
                    self.feedback_fresh = False
                return
            deadline += self.follower.dt
            now = time.monotonic()
            # Never catch up a long wall-clock stall with a burst of old commands.
            if now - deadline > .1:
                with self.lock:
                    self.fault = 'Mink 控制循环超时，已退出接管'
                    self._stop(self.fault)
                deadline = now
            elif now > deadline:
                deadline = now
            self.stopped_event.wait(max(0., deadline - time.monotonic()))

    def state(self, session, client):
        with self.lock:
            state = super().state(session, client)
            _, jacobian = self.planner.chain.fk_jacobian(self.q)
            state.update(
                publication={'mode': 'mink', 'backend': 'mujoco_mink', 'solver': self.follower.options['solver'],
                             'control_dt_s': self.follower.dt, 'feedback_source': 'mujoco_actual'},
                input_mode='continuous', fault=self.fault,
                tcp_actual_speed_mm_s=float(np.linalg.norm(jacobian[:3] @ self.backend.v[self.indices[self.selected_arm]]) * 1000),
                simulation_time_s=float(self.backend.data.time),
                servo={'iterations': self.iterations, 'last_cycle_s': self.last_cycle_s,
                       'collision_pairs': len(self.follower.collision_limit.geom_id_pairs),
                       'tracking_error_mm': self.metrics.get('tracking_error_mm', 0.)},
                gravity_compensation=self.backend.gravity_state(),
                last_plan={'duration_s': self.motion_elapsed, 'metrics': self.metrics} if self.metrics else None,
                blockers=['TCP 与实际工具尚未标定', '避碰仅覆盖当前仿真几何', '实机执行接口尚未接入',
                          '实机重力前馈的力矩换算与驱动接口尚未核验'])
            return state

    def close(self):
        self.stopped_event.set()
        if self.worker:
            self.worker.join(timeout=5)


def run_viewer(server):
    """Serve the webpage while a passive local MuJoCo window is open."""
    jog = server.jog
    worker = None
    try:
        with jog.managed_viewer(jog.backend.model, jog.backend.data) as viewer:
            jog.start()
            worker = threading.Thread(target=server.serve_forever, daemon=True)
            worker.start()
            while viewer.is_running():
                with jog.lock:
                    viewer.sync()
                if jog.stopped_event.wait(1 / 30):
                    break
    finally:
        if worker is not None:
            server.shutdown()
            worker.join(timeout=3)
