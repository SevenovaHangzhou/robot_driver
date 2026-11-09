"""Continuous C2 Mink knots using the collected real model and its limits."""
import math

import mink
import numpy as np
from qpsolvers import solve_qp

from .errors import JogError
from .rolling_stream import Knot, validate_window


# Planning reserve inside the artifact envelope; hardware limits are unchanged.
PLANNING_SCALE = .98


def acceleration_bounds(anchor, dt, speed, acceleration):
    # Quadratic velocity has Bezier controls v0, v0+a0*dt/2, v1.
    # Bound v1 and its next control v1+a1*dt/2: each new anchor remains
    # viable for the following segment, including a first ramp to zero a.
    delta_low, delta_high = -speed - anchor.v, speed - anchor.v
    lower = np.maximum.reduce((-acceleration, 2 * delta_low / dt - anchor.a,
                               delta_low / dt - .5 * anchor.a))
    upper = np.minimum.reduce((acceleration, 2 * delta_high / dt - anchor.a,
                               delta_high / dt - .5 * anchor.a))
    return lower, upper


class RealRollingSource:
    def __init__(self, model, positions, side, *, response_s=.2, held_positions=None):
        if model.collision is None:
            raise JogError('collision_required', '实机连续求解必须配置双臂与自碰撞约束')
        self.model, self.side, self.response = model, side, response_s
        self.speed_limits = model.bounds[2] * PLANNING_SCALE
        self.acceleration_limits = model.bounds[3] * PLANNING_SCALE
        self.config = model.configuration(positions)
        if held_positions is not None:
            # Keep the actual-pose collision check as well as the command-path check.
            model.collision.check_configuration(self.config)
            self.config = model.configuration({**positions, **held_positions})
        self.indices = [model.names.index(n) for n in model.chains[side].names]
        self.qadr = [model.qpos[n] for n in model.names]
        self.vadr = [model.dofs[n] for n in model.names]
        self.initial = self.config.q[self.qadr].copy()
        self.anchor = Knot(0, self.initial.copy(), np.zeros(14), np.zeros(14))
        self.frame = self.config.get_transform_frame_to_world(model.chains[side].tip, 'body')
        self.rotation = self.frame.rotation()
        self.position = self.frame.translation().copy()
        self.j7_rotation = self.config.get_transform_frame_to_world(
            model.chains[side].rotation_frame, 'body').rotation()
        self.j7_to_tcp = self.j7_rotation.inverse() @ self.rotation
        self.world_rotation = self.config.data.body(model.chains[side].base).xmat.reshape(3, 3).copy()
        self.task = mink.FrameTask(model.chains[side].tip, 'body', position_cost=1., orientation_cost=1.)
        self.posture = mink.PostureTask(model.model, cost=1e-5)
        self.posture.set_target_from_configuration(self.config)
        frozen = [d for n, d in model.dofs.items() if n not in model.chains[side].names]
        self.freeze = mink.DofFreezingTask(model.model, frozen)
        self.limits = [mink.ConfigurationLimit(model.model),
                       mink.VelocityLimit(model.model, dict(zip(model.names, self.speed_limits))),
                       model.collision.limit]
        model.collision.check_configuration(self.config)
        self.stop_knots = None
        self.stop_until_ns = None

    def rollout(self, anchor, times, velocity):
        model, config = self.model, self.config
        knot, result = anchor, []
        velocity = np.asarray(velocity, dtype=float)
        if velocity.shape == (3,):
            velocity = np.r_[velocity, np.zeros(3)]
        if velocity.shape != (6,) or not np.isfinite(velocity).all():
            raise JogError('bad_velocity', '笛卡尔速度必须是有限的六维线速度和角速度')
        world_velocity = self.world_rotation @ velocity[:3]
        projection = np.eye(model.model.nv)[self.vadr]
        for at in times:
            dt = (at - knot.time_ns) / 1e9
            q = config.q.copy()
            q[self.qadr] = knot.q
            config.update(q)
            current = config.get_transform_frame_to_world(model.chains[self.side].tip, 'body')
            j7 = config.get_transform_frame_to_world(model.chains[self.side].rotation_frame, 'body')
            rotation = self.rotation
            if np.any(velocity[3:]):
                angular_speed = np.linalg.norm(velocity[3:])
                axis = velocity[3:] / angular_speed
                # Keep the original single-axis path while advancing from the
                # committed knot. Integrating from the solved orientation would
                # accumulate transverse IK error across suffix replacements.
                progress = float((self.j7_rotation.inverse() @ j7.rotation()).log() @ axis)
                rotation = (self.j7_rotation @ mink.SO3.exp(axis * (progress + angular_speed * dt))
                            @ self.j7_to_tcp)
            position = (current.translation() + world_velocity * dt
                        if np.any(world_velocity) else self.position)
            self.task.set_target(mink.SE3.from_rotation_and_translation(
                rotation, position))
            problem = mink.build_ik(config, [self.task, self.posture], dt, limits=self.limits,
                                    constraints=[self.freeze])
            # The IK variable is displacement. Solve for endpoint acceleration
            # so q/v/a at the committed splice remain exactly continuous.
            base = np.zeros(model.model.nv)
            base[self.vadr] = knot.v * dt + knot.a * dt**2 / 3
            scale = dt**2 / 6
            lo, hi = acceleration_bounds(knot, dt, self.speed_limits, self.acceleration_limits)
            G = np.vstack((problem.G, projection, -projection))
            h = np.r_[(problem.h - problem.G @ base) / scale, hi, -lo]
            A = problem.A
            b = None if A is None else (problem.b - A @ base) / scale
            objective_base = np.zeros(model.model.nv)
            objective_base[self.vadr] = (knot.v + .5 * self.response * knot.a) * dt
            solution = solve_qp(problem.P, (problem.P @ objective_base + problem.q) / (self.response * dt),
                                G, h, A, b, solver='daqp', primal_tol=1e-10, dual_tol=1e-10)
            if solution is None or not np.isfinite(solution).all():
                raise JogError('mink_infeasible', '连续 Mink 求解没有可行解')
            a1 = solution[self.vadr]
            q1 = knot.q + knot.v * dt + (knot.a / 3 + a1 / 6) * dt**2
            v1 = knot.v + .5 * (knot.a + a1) * dt
            other = np.ones(14, dtype=bool)
            other[self.indices] = False
            q1[other], v1[other], a1[other] = self.initial[other], 0., 0.
            knot = Knot(at, q1, v1, a1)
            result.append(knot)
        return result

    def stop_rollout(self, anchor, times):
        if self.stop_knots is None:
            dt_ns = times[0] - anchor.time_ns
            dt = dt_ns / 1e9
            limits = self.acceleration_limits
            knots = [anchor]
            # Remove residual acceleration first. The rolling QP reserves this
            # step inside the velocity envelope at every possible stop anchor.
            if np.any(anchor.a != 0.):
                knots.append(Knot(anchor.time_ns + dt_ns,
                    anchor.q + anchor.v * dt + anchor.a * dt**2 / 3,
                    anchor.v + .5 * anchor.a * dt, np.zeros(14)))
            start = knots[-1]
            duration = max(2 * dt, float(np.max(4 * np.abs(start.v) / limits)) + 2 * dt)
            count = 2 * math.ceil(duration / dt / 2)
            if count + len(knots) - 1 > 256:
                raise JogError('rolling_stop', '连续减速尾段超过客户端容量')
            duration = count * dt
            middle = -2 * start.v / duration
            accelerations = np.vstack((np.linspace(np.zeros(14), middle, count // 2 + 1),
                                       np.linspace(middle, np.zeros(14), count // 2 + 1)[1:]))
            for i, a1 in enumerate(accelerations[1:]):
                k = knots[-1]
                knots.append(Knot(start.time_ns + (i + 1) * dt_ns,
                    k.q + k.v * dt + (k.a / 3 + a1 / 6) * dt**2,
                    k.v + .5 * (k.a + a1) * dt, a1.copy()))
            if np.max(np.abs(knots[-1].v)) > 1e-9:
                raise JogError('rolling_stop', '减速末端速度未收敛')
            knots[-1].v[:] = knots[-1].a[:] = 0.
            self.stop_knots = {k.time_ns: k for k in knots}
            self.stop_until_ns = knots[-1].time_ns
        last = self.stop_knots[self.stop_until_ns]
        return [self.stop_knots[t] if t <= self.stop_until_ns else
                Knot(t, last.q.copy(), np.zeros(14), np.zeros(14)) for t in times]

    def validate(self, knots):
        bounds = self.model.bounds
        # The artifact specifies q/v/a, with no hardware jerk envelope.
        curve = validate_window(knots, bounds[0], bounds[1], bounds[2], bounds[3], np.inf, joint_names=self.model.names)
        self.model.collision.check_curve(curve, self.config)
        return curve
