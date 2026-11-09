"""Arm collision constraints and conservative checks of each controller cubic.

MuJoCo treats each mesh as a convex hull. Pair exclusions are only fixed welds
and directly adjacent welds, as in the pinned Mink implementation. No pair is
removed because it happens to overlap in a reference pose.
"""
from dataclasses import asdict, dataclass
from numbers import Real

import mink
from mink.limits import Constraint
import mujoco
import numpy as np

from .errors import JogError


@dataclass(frozen=True)
class CollisionPolicy:
    minimum_distance_m: float
    detection_distance_m: float
    gain: float
    max_subdivision_depth: int = 12
    max_interval_checks: int = 10000
    numerical_tolerance_m: float = 1e-9

    def __post_init__(self):
        values = [self.minimum_distance_m, self.detection_distance_m, self.gain, self.numerical_tolerance_m]
        if (any(isinstance(v, bool) or not isinstance(v, Real) for v in values)
                or not np.isfinite(values).all()
                or not 0 < self.minimum_distance_m < self.detection_distance_m
                or not 0 < self.gain <= 1 or not 0 < self.numerical_tolerance_m < self.minimum_distance_m
                or type(self.max_subdivision_depth) is not int or not 0 <= self.max_subdivision_depth <= 20
                or type(self.max_interval_checks) is not int or not 1 <= self.max_interval_checks <= 100000):
            raise JogError('collision_policy', '碰撞距离、增益或区间检查深度无效')


class DisplacementCollisionLimit(mink.CollisionAvoidanceLimit):
    def compute_qp_inequalities(self, configuration, dt):
        # Mink 1.3.0 build_ik optimizes delta_q. Its collision limit returns a
        # velocity bound, so convert h to displacement, as in demo_mink.sh.
        constraint = super().compute_qp_inequalities(configuration, dt)
        return Constraint(G=constraint.G, h=constraint.h * dt)


class ArmCollision:
    def __init__(self, model, names, groups, policy, sources=()):
        self.model, self.names, self.policy = model, tuple(names), policy
        self.sources = list(sources)
        geoms = {s: [g for g in range(model.ngeom)
                     if model.body(int(model.geom_bodyid[g])).name in links] for s, links in groups.items()}
        if any(not geoms.get(s) for s in ('left', 'right')):
            raise JogError('collision_model', '双臂碰撞几何不能为空')
        self.limit = DisplacementCollisionLimit(
            model, [(geoms['left'], geoms['right']), (geoms['left'], geoms['left']),
                    (geoms['right'], geoms['right'])], gain=policy.gain,
            minimum_distance_from_collisions=policy.minimum_distance_m,
            collision_detection_distance=policy.detection_distance_m)
        self.pairs = sorted(self.limit.geom_id_pairs)
        self.counts = {s: 0 for s in ('between_arms', 'left_self', 'right_self')}
        for a, b in self.pairs:
            left_a, left_b = a in geoms['left'], b in geoms['left']
            self.counts['left_self' if left_a and left_b else
                        'right_self' if not left_a and not left_b else 'between_arms'] += 1
        if not all(self.counts.values()):
            raise JogError('collision_model', '双臂互碰或单臂自碰撞检查对缺失')
        count = len(geoms['left']) + len(geoms['right'])
        total = count * (count - 1) // 2
        self.excluded_count = total - len(self.pairs)
        self.qadr = [int(model.jnt_qposadr[model.joint(n).id]) for n in self.names]
        self.motion_weights = self._motion_weights()

    def info(self):
        return {'enabled': True, 'scope': ['between_arms', 'left_self', 'right_self'],
                'uncovered': ['arms_vs_torso_head_chassis', 'scene_obstacles', 'unmodeled_tools_cables'],
                'pair_counts': self.counts, 'excluded_pair_count': self.excluded_count,
                'exclusions': 'same_weld_or_directly_adjacent_welds',
                'mesh_representation': 'convex_hull', 'policy': asdict(self.policy),
                'mesh_sources': self.sources, 'hardware_validated': False}

    def _motion_weights(self):
        # Radius about an ancestor joint <= geom radius + all intervening link
        # translations. Hinge rotations preserve lengths. Prismatic travel is
        # included before walking to the next ancestor. This holds for all q.
        m = self.model
        weights = np.zeros((m.ngeom, len(self.names)))
        for g in range(m.ngeom):
            reach = float(np.linalg.norm(m.geom_pos[g]) + m.geom_rbound[g])
            body = int(m.geom_bodyid[g])
            while body:
                travel = 0.
                for j in range(m.body_jntadr[body], m.body_jntadr[body] + m.body_jntnum[body]):
                    name = m.joint(j).name
                    if name in self.names:
                        weights[g, self.names.index(name)] = (
                            1. if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE
                            else reach + np.linalg.norm(m.jnt_pos[j]))
                    if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE:
                        if not m.jnt_limited[j]:
                            raise JogError('collision_model', '碰撞运动界需要有限的移动关节行程')
                        travel += float(np.max(np.abs(m.jnt_range[j])))
                reach += float(np.linalg.norm(m.body_pos[body])) + travel
                body = int(m.body_parentid[body])
        return weights

    def _distance(self, data, a, b, cap):
        distance = float(mujoco.mj_geomDistance(self.model, data, a, b, cap, None))
        if not np.isfinite(distance):
            raise JogError('collision_distance', '碰撞距离计算失败')
        return distance

    def _reject(self, a, b, distance):
        names = [self.model.body(int(self.model.geom_bodyid[g])).name for g in (a, b)]
        raise JogError('real_collision', f'{names[0]} / {names[1]} 距离 {distance * 1000:.3f} mm，'
                       f'小于碰撞余量 {self.policy.minimum_distance_m * 1000:g} mm')

    def check_configuration(self, configuration):
        minimum = self.policy.detection_distance_m
        for a, b in self.pairs:
            distance = self._distance(configuration.data, a, b, self.policy.detection_distance_m)
            minimum = min(minimum, distance)
            if distance < self.policy.minimum_distance_m:
                self._reject(a, b, distance)
        return minimum

    def check_curve(self, curve, configuration):
        """Certify the whole cubic using distance minus a joint-motion bound.

        Check the midpoint and bound each shape's motion over the interval by
        sum(radius_j * maximum_joint_excursion_j). If the pair cannot be proven
        separated, subdivide. An unresolved interval is rejected, never accepted
        based on sparse samples. This is only for the imported convex geometry.
        """
        if curve.c.shape[0] != 4 or curve.c.shape[2:] != (len(self.names),):
            raise JogError('collision_curve', '碰撞复核需要完整 14 轴三次轨迹')
        p = self.policy
        qbase = configuration.q.copy()
        check = mink.Configuration(self.model, q=qbase)
        summary = {'method': 'cubic_interval_motion_bounds', 'interval_checks': 0,
                   'scope': ['between_arms', 'left_self', 'right_self'],
                   'distance_queries': 0, 'minimum_sampled_distance_m': p.detection_distance_m,
                   'distance_cap_m': p.detection_distance_m}
        for index, (begin, end) in enumerate(zip(curve.x, curve.x[1:])):
            c = curve.c[:, index]
            critical = [[] for _ in self.names]
            for axis in range(len(self.names)):
                for root in np.roots([3*c[0, axis], 2*c[1, axis], c[2, axis]]):
                    if abs(root.imag) < 1e-12 and 0 < root.real < end - begin:
                        critical[axis].append(float(begin + root.real))
            pending = [(float(begin), float(end), self.pairs, 0)]
            while pending:
                low, high, pairs, depth = pending.pop()
                middle = (low + high) / 2
                center = curve(middle)
                if not np.isfinite(center).all():
                    raise JogError('collision_curve', '碰撞复核收到非有限轨迹')
                excursion = np.maximum(np.abs(curve(low) - center), np.abs(curve(high) - center))
                for axis, roots in enumerate(critical):
                    for at in roots:
                        if low < at < high:
                            excursion[axis] = max(excursion[axis], abs(curve(at)[axis] - center[axis]))
                motion = self.motion_weights @ excursion
                q = qbase.copy()
                q[self.qadr] = center
                check.update(q)
                summary['interval_checks'] += 1
                if summary['interval_checks'] > p.max_interval_checks:
                    raise JogError('collision_uncertain', '碰撞区间检查预算耗尽，拒绝轨迹')
                unresolved = []
                for a, b in pairs:
                    bound = p.minimum_distance_m + motion[a] + motion[b] + p.numerical_tolerance_m
                    # A bounding sphere is sufficient to prove distant pairs.
                    sphere_gap = (np.linalg.norm(check.data.geom_xpos[a] - check.data.geom_xpos[b])
                                  - self.model.geom_rbound[a] - self.model.geom_rbound[b])
                    if sphere_gap >= max(bound, p.detection_distance_m):
                        continue
                    cap = max(bound, p.detection_distance_m)
                    distance = self._distance(check.data, a, b, cap)
                    summary['distance_queries'] += 1
                    summary['minimum_sampled_distance_m'] = min(summary['minimum_sampled_distance_m'], distance)
                    if distance < p.minimum_distance_m:
                        self._reject(a, b, distance)
                    if distance < bound:
                        unresolved.append((a, b))
                if unresolved:
                    if depth >= p.max_subdivision_depth:
                        raise JogError('collision_uncertain', '区间细分后仍无法证明碰撞余量，拒绝轨迹')
                    pending.extend([(low, middle, unresolved, depth + 1),
                                    (middle, high, unresolved, depth + 1)])
        return summary
