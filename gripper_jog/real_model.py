"""Mink kinematics from the actual artifact, without physics or command output.

Optional arm collision constraints use the same artifact's collision geometry.
An offline solution is not permission to execute it on hardware.
"""
from numbers import Real

import mink
import mujoco
import numpy as np
from scipy.interpolate import CubicSpline

from .errors import JogError
from .collision_model import artifact_model
from .model import Chain, robot_direction_map
from .real_collision import ArmCollision
from .rolling_stream import Knot, validate_window


class ArtifactMink:
    def __init__(self, xml, parameters, *, model_share=None, collision_policy=None):
        self.xml, self.parameters = xml, parameters
        self.names = tuple(parameters['joints'])
        self.chains = {side: Chain(xml, 'arm_carriage', side + '_tool0', margin=0)
                       for side in ('left', 'right')}
        expected = {n for c in self.chains.values() for n in c.names}
        if len(self.names) != 14 or set(self.names) != expected:
            raise JogError('real_axes', 'Rolling 轴序必须精确覆盖模型中的双臂关节')
        self.bounds = np.array([parameters[k] for k in
                                ('joint_position_lower', 'joint_position_upper',
                                 'joint_velocity_limits', 'joint_acceleration_limits')], dtype=float)
        margin = float(parameters['position_margin'])
        if self.bounds.shape != (4, 14) or not np.isfinite(self.bounds).all() or not np.isfinite(margin) or margin < 0:
            raise JogError('real_limits', 'Rolling 模型限值缺失或无效')
        if np.any(self.bounds[2:] <= 0) or np.any(self.bounds[0] + 2 * margin >= self.bounds[1]):
            raise JogError('real_limits', 'Rolling 模型限值范围无效')
        for chain in self.chains.values():
            indices = [self.names.index(n) for n in chain.names]
            for actual, required in ((self.bounds[0, indices], chain.lower),
                                     (self.bounds[1, indices], chain.upper),
                                     (self.bounds[2, indices], chain.velocity)):
                if not np.allclose(actual, required, atol=1e-10, rtol=0):
                    raise JogError('real_limits', 'Rolling 限值与运行 URDF 不一致')
        self.bounds[0] += margin
        self.bounds[1] -= margin
        self.model, groups, sources = artifact_model(xml, self.names, model_share,
                                                    collisions=collision_policy is not None)
        self.joint_ids = {self.model.joint(i).name: i for i in range(self.model.njnt)}
        self.qpos = {n: int(self.model.jnt_qposadr[j]) for n, j in self.joint_ids.items()}
        self.dofs = {n: int(self.model.jnt_dofadr[j]) for n, j in self.joint_ids.items()}
        if self.model.nq != self.model.nv or self.model.nq != self.model.njnt:
            raise JogError('real_model', '仅支持标量 URDF 关节')
        for i, name in enumerate(self.names):
            self.model.jnt_range[self.joint_ids[name]] = self.bounds[:2, i]
        self.direction_maps = {s: robot_direction_map(xml, 'base_link', c.base)
                               for s, c in self.chains.items()}
        self.collision = (ArmCollision(self.model, self.names, groups, collision_policy, sources)
                          if collision_policy is not None else None)

    def configuration(self, positions):
        if any(n not in positions for n in self.names):
            raise JogError('feedback_incomplete', '影子求解需要完整双臂实测关节值')
        q = np.zeros(self.model.nq)
        for name, value in positions.items():
            if name not in self.qpos:
                continue
            if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value):
                raise JogError('feedback_invalid', '关节反馈包含非有限数值')
            q[self.qpos[name]] = value
        arm_q = np.array([q[self.qpos[n]] for n in self.names])
        if np.any(arm_q < self.bounds[0]) or np.any(arm_q > self.bounds[1]):
            raise JogError('joint_limit', '起始状态超出 Rolling 的位置余量范围')
        return mink.Configuration(self.model, q=q)

    def verify_fk(self, positions):
        config = self.configuration(positions)
        base = config.data.body('arm_carriage')
        rotation = base.xmat.reshape(3, 3)
        results = {}
        for side, chain in self.chains.items():
            q = [positions[n] for n in chain.names]
            expected, jac = chain.fk_jacobian(q)
            body = config.data.body(chain.tip)
            expected_position = base.xpos + rotation @ expected[:3, 3]
            jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, config.data, jp, jr, body.id)
            indices = [self.dofs[n] for n in chain.names]
            actual_jac = np.vstack((rotation.T @ jp[:, indices], rotation.T @ jr[:, indices]))
            errors = {'position_m': float(np.linalg.norm(body.xpos - expected_position)),
                      'rotation_matrix': float(np.max(np.abs(body.xmat.reshape(3, 3) - rotation @ expected[:3, :3]))),
                      'jacobian': float(np.max(np.abs(jac - actual_jac)))}
            if max(errors.values()) > 1e-9:
                raise JogError('model_mismatch', '真机 URDF 的 Mink FK/Jacobian 对账失败')
            results[side] = errors
        return results

    def shadow_path(self, positions, side, direction, distance_mm=1., speed_mm_s=.5, dt=.02):
        if side not in self.chains or direction not in self.direction_maps[side]:
            raise JogError('bad_direction', '机械臂或方向无效')
        if not np.isfinite([distance_mm, speed_mm_s, dt]).all() or min(distance_mm, speed_mm_s, dt) <= 0:
            raise JogError('bad_plan', '距离、速度和求解周期必须为有限正数')
        config = self.configuration(positions)
        if self.collision:
            self.collision.check_configuration(config)
        chain = self.chains[side]
        target = config.get_transform_frame_to_world(chain.tip, 'body')
        initial = target.translation().copy()
        cardinal = self.direction_maps[side][direction]
        base_direction = np.eye(3)['xyz'.index(cardinal[0])] * (1 if cardinal[1] == '+' else -1)
        world_direction = config.data.body(chain.base).xmat.reshape(3, 3) @ base_direction
        task = mink.FrameTask(chain.tip, 'body', position_cost=1., orientation_cost=1.)
        posture = mink.PostureTask(self.model, cost=1e-5)
        posture.set_target_from_configuration(config)
        frozen = [d for n, d in self.dofs.items() if n not in chain.names]
        freeze = mink.DofFreezingTask(self.model, frozen)
        limits = [mink.ConfigurationLimit(self.model),
                  mink.VelocityLimit(self.model, dict(zip(self.names, self.bounds[2])))]
        if self.collision:
            limits.append(self.collision.limit)
        # Quintic Cartesian profile: its peak speed is 1.875 * distance / duration.
        duration = 1.875 * distance_mm / speed_mm_s
        time_ns = np.rint(np.linspace(0., duration * 1e9, int(np.ceil(duration / dt)) + 1)).astype(np.int64)
        times = time_ns / 1e9
        rows = [[config.q[self.qpos[n]] for n in self.names]]
        max_error, max_rotation = 0., 0.
        for previous, now in zip(times, times[1:]):
            s = now / duration
            desired = initial + world_direction * distance_mm / 1000 * (10*s**3 - 15*s**4 + 6*s**5)
            task.set_target(mink.SE3.from_rotation_and_translation(target.rotation(), desired))
            velocity = mink.solve_ik(config, [task, posture], now - previous, solver='daqp',
                                     limits=limits, constraints=[freeze])
            config.integrate_inplace(velocity, now - previous)
            actual = config.get_transform_frame_to_world(chain.tip, 'body')
            max_error = max(max_error, float(np.linalg.norm(actual.translation() - desired)))
            max_rotation = max(max_rotation, float(np.linalg.norm((actual.rotation().inverse() @ target.rotation()).log())))
            rows.append([config.q[self.qpos[n]] for n in self.names])
        curve = CubicSpline(times, rows, bc_type=((1, np.zeros(14)), (1, np.zeros(14))), axis=0)
        knots = [Knot(round(t * 1e9), np.array(q), curve(t, 1), curve(t, 2))
                 for t, q in zip(times, rows)]
        validate_window(knots, self.bounds[0], self.bounds[1], self.bounds[2],
                        float(np.min(self.bounds[3])), np.inf)
        collision_result = self.collision.check_curve(curve, config) if self.collision else None
        # Numerical tolerances describe solver accuracy, not hardware safety.
        return curve, {'arm': side, 'robot_direction': direction,
                       'duration_s': duration, 'distance_mm': distance_mm, 'peak_tcp_target_mm_s': speed_mm_s,
                       'max_knot_path_error_mm': max_error * 1000,
                       'max_knot_rotation_error_rad': max_rotation,
                       'kinematic_path_ok': max_error <= 1e-5 and max_rotation <= 1e-4,
                       'cubic_limits_ok': True, 'collision_checked': self.collision is not None,
                       'collision': collision_result,
                       'hardware_motion_ready': False}
