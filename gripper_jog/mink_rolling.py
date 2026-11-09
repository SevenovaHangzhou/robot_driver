"""Mink kinematic prediction -> continuous Rolling windows -> local physics.

MinkRollingSource and RollingStream also accept RosRollingTransport from a
Motion owner. This module never creates ROS nodes, changes mode or enables arms.
The web builder below uses only the local protocol simulator.
"""
import math
import time

import mink
from mink.limits import Constraint
import mujoco
import numpy as np

from .control import SPEED_ERROR, SPEED_OPTIONS_MM_S
from .errors import JogError
from .mink_sim import MinkJog
from .planner import DIRECTIONS, Limits
from .rolling import JOINT_NAMES
from .rolling_mock import MockRollingTransport
from .rolling_stream import Knot, RollingStream, validate_window
from scipy.spatial.transform import Rotation


class _DynamicsLimit:
    """Bounds on endpoint acceleration of a constant-jerk cubic interval."""
    def __init__(self, backend, knot, speed, acceleration, jerk, selected):
        self.backend, self.knot = backend, knot
        self.speed, self.acceleration, self.jerk = speed, acceleration, jerk
        self.selected = selected

    def bounds(self, dt):
        v, a, j = self.knot.v, self.knot.a, self.jerk
        # Leave enough velocity headroom to bring a1 to zero at bounded jerk.
        # This avoids infeasible next intervals close to the velocity limit.
        upper = -j * dt + np.sqrt(np.maximum(0., (j * dt)**2 + 2 * j * (self.speed - v - .5 * a * dt)))
        lower = j * dt - np.sqrt(np.maximum(0., (j * dt)**2 + 2 * j * (self.speed + v + .5 * a * dt)))
        # Keep the next velocity quadratic's middle Bernstein coefficient in
        # bounds too; endpoint limits alone can miss an interior speed peak.
        lo = np.maximum.reduce([a - j * dt, np.full(14, -self.acceleration), lower,
                                (-self.speed - v - .5 * a * dt) / dt])
        hi = np.minimum.reduce([a + j * dt, np.full(14, self.acceleration), upper,
                                (self.speed - v - .5 * a * dt) / dt])
        other = np.ones(14, dtype=bool)
        other[self.selected] = False
        lo[other] = hi[other] = 0.
        if np.any(lo > hi + 1e-10):
            raise JogError('rolling_dynamics', '当前状态无法满足下一段动态限制')
        return lo, hi

    def compute_qp_inequalities(self, configuration, dt):
        lo, hi = self.bounds(dt)
        v, a = self.knot.v, self.knot.a
        # Mink's optimization variable is displacement, not endpoint velocity.
        base = v * dt + a * dt**2 / 3
        projection = np.eye(self.backend.model.nv)[self.backend.vadr]
        return Constraint(G=np.vstack([projection, -projection]),
                          h=np.r_[base + hi * dt**2 / 6, -base - lo * dt**2 / 6])


class MinkRollingSource:
    duration = math.inf

    def __init__(self, follower, chain, side, positions, limits=None):
        self.follower, self.chain, self.side = follower, chain, side
        self.backend = b = follower.backend
        if tuple(b.names) != JOINT_NAMES:
            raise JogError('rolling_axes', 'Mink 必须映射完整 V3 双臂 14 轴')
        self.indices = np.array([JOINT_NAMES.index(n) for n in chain.names])
        self.initial = np.asarray(positions, dtype=float).copy()
        if self.initial.shape != (14,) or not np.isfinite(self.initial).all():
            raise JogError('rolling_anchor', '缺少有效的 Rolling 双臂命令锚点')
        # These are existing offline client limits, not measured hardware limits.
        self.limits = limits or Limits(tcp_speed=.05)
        self.speed = np.minimum(b.max_velocity, self.limits.joint_speed)
        self.configuration = mink.Configuration(b.model)
        self.configuration.update(b.data.qpos)
        q = self.configuration.q.copy()
        q[b.qadr] = self.initial
        self.configuration.update(q)
        frame = self.configuration.data.body(chain.base)
        self.world_rotation = frame.xmat.reshape(3, 3).copy()
        site = self.configuration.data.site(side + '_ee')
        self.rotation = site.xmat.reshape(3, 3).copy()
        self.task = mink.FrameTask(frame_name=side + '_ee', frame_type='site',
                                   position_cost=follower.options['position_cost'],
                                   orientation_cost=follower.options['orientation_cost'],
                                   gain=follower.options['task_gain'], lm_damping=follower.options['lm_damping'])
        self.posture = mink.PostureTask(b.model, cost=follower.options['posture_cost'])
        self.posture.set_target_from_configuration(self.configuration)
        self.stop_knots = None
        self.stop_until_ns = None
        self.path_velocity = None
        self.path_origin = None

    def state(self, _at):
        return self.initial[self.indices].copy(), np.zeros(7), np.zeros(7), np.zeros(7)

    def rollout(self, anchor, times, velocity):
        b, f, configuration = self.backend, self.follower, self.configuration
        knot, result = anchor, []
        if self.path_velocity is None or not np.array_equal(velocity, self.path_velocity):
            self.path_origin = self.chain.fk(anchor.q[self.indices])[:3, 3].copy()
            self.path_velocity = velocity.copy()
        for time_ns in times:
            dt = (time_ns - knot.time_ns) / 1e9
            q = configuration.q.copy()
            q[b.qadr] = knot.q
            configuration.update(q)
            site = configuration.data.site(self.side + '_ee')
            # Velocity feedforward as a moving task target, without integrating
            # a distant target that can run away when joint limits bind.
            speed = np.linalg.norm(velocity)
            commanded = velocity.copy()
            if speed:
                axis = velocity / speed
                offset = self.chain.fk(knot.q[self.indices])[:3, 3] - self.path_origin
                commanded -= 2 * (offset - axis * np.dot(axis, offset))
                commanded *= min(1., speed / np.linalg.norm(commanded))
            target = site.xpos + self.world_rotation @ commanded * (.98 * dt / self.task.gain)
            self.task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(self.rotation), target))
            dynamic = _DynamicsLimit(b, knot, self.speed, self.limits.joint_acceleration,
                                     self.limits.joint_jerk, self.indices)
            # Solve in endpoint acceleration coordinates. A displacement-only
            # QP has O(dt^3) jerk bounds, smaller than normal solver tolerances.
            problem = mink.build_ik(configuration, [self.task, self.posture], dt,
                                    damping=f.options['solver_damping'],
                                    limits=[mink.ConfigurationLimit(b.model), f.collision_limit])
            base = np.zeros(b.model.nv)
            base[b.vadr] = knot.v * dt + knot.a * dt**2 / 3
            scale = dt**2 / 6
            lo, hi = dynamic.bounds(dt)
            from qpsolvers import solve_qp
            projection = np.eye(b.model.nv)[b.vadr]
            G = np.vstack([problem.G, projection, -projection])
            h = np.r_[(problem.h - problem.G @ base) / scale, hi, -lo]
            # A velocity target at just dt would repeatedly saturate jerk and
            # overshoot the requested speed. Predict over a short response
            # interval and damp the existing acceleration in the objective.
            response = .2
            objective_base = np.zeros(b.model.nv)
            objective_base[b.vadr] = (knot.v + .5 * response * knot.a) * dt
            objective_scale = response * dt
            endpoint_a = solve_qp(problem.P, (problem.P @ objective_base + problem.q) / objective_scale,
                                  G, h, solver=f.options['solver'], primal_tol=1e-10, dual_tol=1e-10)
            if endpoint_a is None or not np.isfinite(endpoint_a).all():
                raise JogError('mink_infeasible', 'Mink 连续轨迹求解无可行解')
            a1 = endpoint_a[b.vadr]
            if np.any(a1 < lo - 1e-7) or np.any(a1 > hi + 1e-7):
                raise JogError('rolling_dynamics', 'Mink 求解结果超过动态限制')
            a1 = np.clip(a1, lo, hi)  # Only numerical tolerance, not a motion filter.
            q1 = knot.q + knot.v * dt + (knot.a / 3 + a1 / 6) * dt**2
            v1 = knot.v + .5 * (knot.a + a1) * dt
            other = np.ones(14, dtype=bool)
            other[self.indices] = False
            q1[other], v1[other], a1[other] = self.initial[other], 0., 0.
            knot = Knot(time_ns, q1, v1, a1)
            result.append(knot)
        return result

    def validate(self, knots):
        b = self.backend
        curve = validate_window(knots, b.lower, b.upper, self.speed,
                                self.limits.joint_acceleration, self.limits.joint_jerk)
        # Joint extrema above are analytic. TCP and geometry checks are sampled
        # and remain simulation checks, not a full swept-volume proof.
        last_checked = None
        for begin, end in zip(knots, knots[1:]):
            for at in np.linspace(begin.time_ns / 1e9, end.time_ns / 1e9, 3):
                q, v = curve(at), curve(at, 1)
                if last_checked is not None and np.array_equal(q, last_checked) and not np.any(v):
                    continue
                last_checked = q.copy()
                pose, jacobian = self.chain.fk_jacobian(q[self.indices])
                if np.linalg.norm(jacobian[:3] @ v[self.indices]) > self.limits.tcp_speed + 1e-7:
                    raise JogError('rolling_tcp_speed', 'Rolling 插值超过 TCP 速度上限')
                full = self.configuration.q.copy()
                full[b.qadr] = q
                self.configuration.update(full)
                if self.follower.collision_clearance(self.configuration.data, self.follower.guard_distance)['distance_m'] < self.follower.guard_distance:
                    raise JogError('rolling_collision', '未来轨迹几何检查失败')

    def stop_rollout(self, anchor, times):
        """A finite, bounded-jerk stop; cache at most 256 future knots."""
        if self.stop_knots is None:
            from qpsolvers import solve_qp
            dt = (times[0] - anchor.time_ns) / 1e9
            duration = (.16 + float(np.max(np.abs(anchor.v))) / self.limits.joint_acceleration
                        + 2 * float(np.max(np.abs(anchor.a))) / self.limits.joint_jerk
                        + self.limits.joint_acceleration / self.limits.joint_jerk)
            count = max(8, math.ceil(duration / dt))
            if count > 255:
                raise JogError('rolling_stop', '减速规划超过本地容量预算')
            diff = np.eye(count) - np.eye(count, k=-1)
            velocity_matrix = dt * (np.tril(np.ones((count, count)), -1) + .5 * np.eye(count))
            terminal = np.zeros(count)
            terminal[-1] = 1.
            A = np.array([velocity_matrix[-1], terminal])
            middle_velocity = np.vstack([np.zeros(count), velocity_matrix[:-1] + .5 * dt * np.eye(count)[:-1]])
            G = np.vstack([diff, -diff, velocity_matrix, -velocity_matrix, middle_velocity, -middle_velocity])
            P = diff.T @ diff + 1e-6 * np.eye(count)
            accelerations = np.zeros((count + 1, 14))
            accelerations[0] = anchor.a
            for axis in self.indices:
                a0, v0 = anchor.a[axis], anchor.v[axis]
                first = np.zeros(count)
                first[0] = a0
                base_v = v0 + .5 * dt * a0
                h = np.r_[self.limits.joint_jerk * dt + first,
                          self.limits.joint_jerk * dt - first,
                          np.full(count, self.speed[axis] - base_v),
                          np.full(count, self.speed[axis] + base_v),
                          np.full(count, self.speed[axis] - base_v),
                          np.full(count, self.speed[axis] + base_v)]
                result = solve_qp(P, -diff.T @ first, G, h, A, np.array([-base_v, 0.]),
                                  lb=np.full(count, -self.limits.joint_acceleration),
                                  ub=np.full(count, self.limits.joint_acceleration), solver=self.follower.options['solver'],
                                  primal_tol=1e-10, dual_tol=1e-10)
                if result is None or not np.isfinite(result).all():
                    raise JogError('rolling_stop', '无法生成连续减速尾段')
                accelerations[1:, axis] = result
            knots = [anchor]
            for i in range(count):
                previous, a1 = knots[-1], accelerations[i + 1].copy()
                q1 = previous.q + previous.v * dt + (previous.a / 3 + a1 / 6) * dt**2
                v1 = previous.v + .5 * (previous.a + a1) * dt
                knots.append(Knot(anchor.time_ns + (i + 1) * (times[0] - anchor.time_ns), q1, v1, a1))
            # Equality constraints should already give these values. Refuse a
            # numerical failure before replacing the future, then remove noise.
            if np.max(np.abs(knots[-1].v)) > 1e-9 or np.max(np.abs(knots[-1].a)) > 1e-9:
                raise JogError('rolling_stop', '减速末端未达到静止状态')
            knots[-1].v[:] = knots[-1].a[:] = 0.
            # Cache the bounded mathematical stop, then validate TCP/geometry
            # only for each window immediately before publication. Checking
            # the whole stop here duplicates work and stalls the servo loop.
            validate_window(knots, self.backend.lower, self.backend.upper, self.speed,
                            self.limits.joint_acceleration, self.limits.joint_jerk)
            self.stop_knots = {k.time_ns: k for k in knots}
            self.stop_until_ns = knots[-1].time_ns
        last = self.stop_knots[self.stop_until_ns]
        return [self.stop_knots[t] if t <= self.stop_until_ns else
                Knot(t, last.q.copy(), np.zeros(14), np.zeros(14)) for t in times]


class MinkRollingJog(MinkJog):
    """Same keyboard contract and actual MuJoCo view, with a Rolling executor."""
    def __init__(self, follower, xml, *, clock, autostart=True, transport=None):
        super().__init__(follower, xml, clock=clock, autostart=False)
        self.transport = transport or MockRollingTransport(self.backend.q, Limits().joint_acceleration, clock)
        self.requested_velocity = np.zeros(3)
        self.stop_requested = False
        self.halt_requested = False
        self.last_motion = None
        self.last_command_q, self.last_command_v = self.backend.q.copy(), np.zeros(14)
        self.result = 'Mink → Rolling 连续控制已就绪'
        if autostart:
            self.start()

    def _stop(self, reason, revoke=True):
        if revoke:
            self.stop_requested = True
        else:
            self.halt_requested = True
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
            return super().dispatch(action, session, body)
        self.tick()
        with self.lock:
            self._owner(session, body)
            self._command(body)
            if self.stopping or not self.feedback_fresh:
                raise JogError('not_ready', '请等待 Rolling 停稳并完成 FINALIZE')
            requested = body.get('robot_direction')
            if not isinstance(requested, str) or requested not in self.direction_maps[self.selected_arm] or 'direction' in body:
                raise JogError('bad_direction', '请提供机器人本体方向 robot_direction')
            speed = body.get('speed_mm_s')
            if type(speed) not in (int, float) or speed not in SPEED_OPTIONS_MM_S or 'step_mm' in body:
                raise JogError('bad_speed', SPEED_ERROR + '，不接受步长')
            self.input_at = self.clock()
            self.direction = self.direction_maps[self.selected_arm][requested]
            self.motion_direction = self.direction
            self.target_speed = speed / 1000
            self.requested_velocity = np.array(DIRECTIONS[self.direction]) * self.target_speed
            if self.active is None:
                self.active, self.stop_requested, self.halt_requested = True, False, False
                self.motion_start_q = self.q.copy()
                self.motion_origin = self.planner.chain.fk(self.q)[:3, 3].copy()
                self.motion_rotation = self.planner.chain.fk(self.q)[:3, :3].copy()
                self.motion_elapsed, self.metrics, self.fault = 0., {}, ''
            self.result = f'Mink → Rolling 连续移动 · {speed:g} mm/s'
            return {'continuous': True, 'direction': self.direction, 'speed_mm_s': speed}

    def _cycle(self):
        try:
            self._stream_cycle()
        except Exception as exc:
            self.fault = f'Mink → Rolling 控制失败：{exc}'
            self._stop(self.fault)
            self.feedback_fresh = False
            if self.active is True:
                self.active, self.stopping = None, False
            elif isinstance(self.active, RollingStream):
                self.active.brake(0)
                self.active.state(0)
                if self.active.complete:
                    self.active, self.stopping = None, False
            # Keep the worker alive to resolve uncertain stop/finalize results.
            try:
                self._drive(self.last_command_q, np.zeros(14))
            except Exception:
                # A broken physics engine must not stop protocol cleanup.
                pass

    def _stream_cycle(self):
        self.tick()
        if self.active is True:
            if self.stop_requested or self.halt_requested:
                self.active, self.stopping = None, False
            else:
                snapshot = self.transport.state()
                source = MinkRollingSource(self.follower, self.planner.chain, self.selected_arm,
                                           snapshot['desired_positions'])
                self.active = self.last_motion = RollingStream(source, self.transport, self.requested_velocity, self.clock)
        motion = self.active if isinstance(self.active, RollingStream) else None
        if motion:
            if self.stop_requested:
                motion.brake(0)
            elif self.halt_requested:
                motion.request_halt()
            else:
                motion.set_velocity(self.requested_velocity)
            motion.state(0)
            if motion.fault:
                self.fault = motion.fault
                self._stop(self.fault)
            if motion.complete:
                self.active, self.stopping = None, False
        snapshot = self.transport.state()
        command_q, command_v = (np.asarray(snapshot[key]) for key in ('desired_positions', 'desired_velocities'))
        if not np.isfinite([command_q, command_v]).all():
            raise JogError('rolling_feedback', 'Rolling 命令反馈无效')
        self.feedback_fresh = True
        self.last_command_q, self.last_command_v = command_q.copy(), command_v.copy()
        self._drive(command_q, command_v)
        if motion:
            self.motion_elapsed += self.follower.dt
            pose = self.planner.chain.fk(self.q)
            axis = np.array(DIRECTIONS[self.motion_direction])
            offset = pose[:3, 3] - self.motion_origin
            cross = offset - axis * np.dot(axis, offset)
            command_pose = self.planner.chain.fk(command_q[self.indices[self.selected_arm]])
            tracking = float(np.linalg.norm(pose[:3, 3] - command_pose[:3, 3]))
            self.metrics = {
                'max_path_error_mm': max(self.metrics.get('max_path_error_mm', 0.), float(np.linalg.norm(cross) * 1000)),
                'max_orientation_error_deg': max(self.metrics.get('max_orientation_error_deg', 0.),
                                                float(np.rad2deg(Rotation.from_matrix(pose[:3, :3] @ self.motion_rotation.T).magnitude()))),
                'max_joint_delta_deg': float(np.rad2deg(np.max(np.abs(self.q - self.motion_start_q)))),
                'tracking_error_mm': tracking * 1000}
            if not self.stop_requested and (tracking > self.max_target_lag_m or
                    self.follower.collision_clearance(cap=self.follower.guard_distance)['distance_m'] < self.follower.guard_distance):
                self.fault = '实际跟随误差过大或仿真避碰保护触发，已请求停止'
                self._stop(self.fault)

    def _drive(self, command_q, command_v):
        for _ in range(self.follower.substeps):
            self.backend.step(command_q, command_v)
        mujoco.mj_forward(self.backend.model, self.backend.data)
        self.follower.last_velocity = command_v.copy()
        self._sync_feedback()
        self.iterations += 1

    def state(self, session, client):
        with self.lock:
            state = super().state(session, client)
            publication = self.last_motion.publication() if self.last_motion else {'mode': 'rolling', 'sequence': 0}
            state['publication'] = {**publication, 'backend': 'mujoco_mink_rolling', 'solver': self.follower.options['solver'],
                                    'control_dt_s': self.follower.dt, 'feedback_source': 'mujoco_actual'}
            state['blockers'] = ['TCP 与实际工具尚未标定', '未来轨迹避碰为仿真采样检查', '实机执行仲裁尚未接入',
                                 '实机重力前馈的力矩换算与驱动接口尚未核验']
            return state

    def close(self):
        with self.lock:
            self._stop('正在关闭 Rolling 会话')
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            with self.lock:
                if self.active is True:
                    self.active, self.stopping = None, False
                if isinstance(self.active, RollingStream):
                    self.active.brake(0)
                    self.active.state(0)
                    if self.active.complete:
                        self.active, self.stopping = None, False
                if self.active is None:
                    break
            time.sleep(.01)
        if self.active is not None:
            # Keep the existing cleanup worker/owner; never pretend an unknown
            # service outcome is a closed session, or start another session.
            raise JogError('rolling_cleanup_pending', 'Rolling 关闭尚未确认，保留清理责任，请继续等待或重试关闭')
        super().close()
