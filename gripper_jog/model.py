"""URDF serial-chain FK/Jacobian and bounded, local damped IK. SI units."""
from dataclasses import dataclass
from hashlib import sha256
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .errors import JogError


def vector(text, default):
    result = np.array([float(x) for x in (text or default).split()])
    if result.shape != (3,) or not np.isfinite(result).all():
        raise JogError("model_invalid", "URDF 三维向量无效")
    return result


def transform(xyz, rpy):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    out[:3, 3] = xyz
    return out


def robot_direction_map(xml, frame, base):
    """Map robot-body XYZ commands to the cardinal axes of the arm planner."""
    root = ET.fromstring(xml)
    links = {link.get('name') for link in root.findall('link')}
    if frame not in links or base not in links:
        raise JogError('frame_missing', '机器人方向参考系不在模型中')
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    rotations, seen, child = [], set(), base
    while child != frame:
        if child in seen or child not in parents:
            raise JogError('frame_disconnected', '规划参考系不在机器人方向参考系下游')
        seen.add(child)
        joint = parents[child]
        # A prismatic lift changes position, but never changes body directions.
        if joint.get('type') not in ('fixed', 'prismatic'):
            raise JogError('direction_frame_invalid', '方向参考系之间存在可旋转关节，无法固定方向映射')
        origin = joint.find('origin')
        rpy = vector(origin.get('rpy') if origin is not None else None, '0 0 0')
        rotations.append(Rotation.from_euler('xyz', rpy).as_matrix())
        child = joint.find('parent').get('link')
    rotation = np.eye(3)
    for r in reversed(rotations):
        rotation = rotation @ r
    mapping = {}
    for index, axis in enumerate('xyz'):
        vector_in_base = rotation.T[:, index]
        target = int(np.argmax(np.abs(vector_in_base)))
        sign = 1 if vector_in_base[target] > 0 else -1
        expected = np.eye(3)[target] * sign
        if not np.allclose(vector_in_base, expected, atol=1e-6, rtol=0):
            raise JogError('direction_frame_invalid', '方向参考系与规划参考系的轴未对齐')
        mapping[axis + '+'] = 'xyz'[target] + ('+' if sign > 0 else '-')
        mapping[axis + '-'] = 'xyz'[target] + ('-' if sign > 0 else '+')
    return mapping


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = 0.0
    upper: float = 0.0
    velocity: float = 0.0


class Chain:
    def __init__(self, xml, base, tip, tcp_xyz=(0, 0, 0), tcp_rpy=(0, 0, 0), margin=0.035):
        if not np.isfinite(margin) or margin < 0:
            raise JogError("model_invalid", "限位余量无效")
        self.source_xml = xml
        root = ET.fromstring(xml)
        links = {link.get("name") for link in root.findall("link")}
        if base not in links or tip not in links:
            raise JogError("frame_missing", "参考系或工具 frame 不在模型中")
        parents = {}
        for j in root.findall("joint"):
            child = j.find("child").get("link")
            if child in parents:
                raise JogError("model_invalid", "模型存在多个父关节")
            parents[child] = j
        path, seen, child = [], set(), tip
        while child != base:
            if child in seen or child not in parents:
                raise JogError("frame_disconnected", "工具 frame 不在参考系下游")
            seen.add(child)
            j = parents[child]
            kind = j.get("type")
            if kind not in ("fixed", "revolute", "prismatic") or j.find("mimic") is not None:
                raise JogError("joint_unsupported", "链中包含未支持的关节类型或 mimic")
            o = j.find("origin")
            xyz = vector(o.get("xyz") if o is not None else None, "0 0 0")
            rpy = vector(o.get("rpy") if o is not None else None, "0 0 0")
            axis, low, high, vel = np.zeros(3), 0., 0., 0.
            if kind != "fixed":
                a, limit = j.find("axis"), j.find("limit")
                axis = vector(a.get("xyz") if a is not None else None, "1 0 0")
                norm = np.linalg.norm(axis)
                if norm < 1e-12 or limit is None:
                    raise JogError("model_invalid", "关节轴或限位缺失")
                axis /= norm
                low, high, vel = (float(limit.get(k)) for k in ("lower", "upper", "velocity"))
                if not np.isfinite([low, high, vel]).all() or low + 2 * margin >= high or vel <= 0:
                    raise JogError("model_invalid", "关节限位无效")
                low, high = low + margin, high - margin
            path.append(Joint(j.get("name"), kind, transform(xyz, rpy), axis, low, high, vel))
            child = j.find("parent").get("link")
        self.path = tuple(reversed(path))
        self.joints = tuple(j for j in self.path if j.kind != "fixed")
        if len(self.joints) < 6:
            raise JogError("model_invalid", "保持姿态的平移要求至少六个受控自由度")
        self.base, self.tip = base, tip
        self.names = tuple(j.name for j in self.joints)
        # The child of the final actuated joint is the moving J7 frame.
        self.rotation_frame = root.find(f"joint[@name='{self.names[-1]}']/child").get('link')
        last = max(i for i, j in enumerate(self.path) if j.kind != 'fixed')
        self.end_to_tcp = np.eye(4)
        for joint in self.path[last + 1:]:
            self.end_to_tcp = self.end_to_tcp @ joint.origin
        self.lower = np.array([j.lower for j in self.joints])
        self.upper = np.array([j.upper for j in self.joints])
        self.velocity = np.array([j.velocity for j in self.joints])
        if not np.isfinite([*tcp_xyz, *tcp_rpy]).all():
            raise JogError("model_invalid", "TCP 变换包含无效数值")
        self.tcp = transform(tcp_xyz, tcp_rpy)
        self.end_to_tcp = self.end_to_tcp @ self.tcp
        self.model_hash = sha256(xml.encode()).hexdigest()
        self.identity = sha256((self.model_hash + base + tip + repr(self.tcp.tolist())).encode()).hexdigest()

    def validate(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != self.lower.shape or not np.isfinite(q).all():
            raise JogError("feedback_invalid", "关节反馈数量或数值无效")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise JogError("joint_limit", "关节状态超出软限位")
        return q

    def fk_jacobian(self, q):
        q = self.validate(q)
        pose, axes, origins, kinds, n = np.eye(4), [], [], [], 0
        for joint in self.path:
            pose = pose @ joint.origin
            if joint.kind == "fixed":
                continue
            axes.append(pose[:3, :3] @ joint.axis)
            origins.append(pose[:3, 3].copy())
            kinds.append(joint.kind)
            motion = np.eye(4)
            if joint.kind == "revolute":
                motion[:3, :3] = Rotation.from_rotvec(joint.axis * q[n]).as_matrix()
            else:
                motion[:3, 3] = joint.axis * q[n]
            pose = pose @ motion
            n += 1
        pose = pose @ self.tcp
        jac = np.zeros((6, n))
        for i, (axis, origin, kind) in enumerate(zip(axes, origins, kinds)):
            if kind == "revolute":
                jac[:3, i] = np.cross(axis, pose[:3, 3] - origin)
                jac[3:, i] = axis
            else:
                jac[:3, i] = axis
        return pose, jac

    def fk(self, q):
        return self.fk_jacobian(q)[0]

    def ik(self, target, seed, min_sigma=0.015, max_jump=0.08):
        origin = self.validate(seed).copy()
        q = origin.copy()
        # Scale angular rows by 0.25 m so the singularity threshold has a defined scale.
        scale = np.array([1., 1., 1., .25, .25, .25])
        for _ in range(100):
            pose, jac = self.fk_jacobian(q)
            error = np.r_[target[:3, 3] - pose[:3, 3],
                          Rotation.from_matrix(target[:3, :3] @ pose[:3, :3].T).as_rotvec()]
            weighted = jac * scale[:, None]
            sigma = np.linalg.svd(weighted, compute_uv=False)[-1]
            if sigma < min_sigma:
                raise JogError("singularity", "当前位置接近奇异位形，请调整初始关节姿态")
            if np.linalg.norm(error[:3]) < 2e-7 and np.linalg.norm(error[3:]) < 2e-6:
                if np.max(np.abs(q - origin)) > max_jump:
                    raise JogError("joint_jump", "逆解关节变化超过连续性限制")
                return q
            dq = weighted.T @ np.linalg.solve(weighted @ weighted.T + np.eye(6) * 1e-8, error * scale)
            dq *= min(1., .025 / max(np.max(np.abs(dq)), 1e-12))
            # A local solve must fail instead of clipping a requested Cartesian path.
            candidate = q + dq
            if np.any(candidate <= self.lower) or np.any(candidate >= self.upper):
                raise JogError("joint_limit", "直线路径将触及关节软限位")
            q = candidate
        raise JogError("ik_failed", "无法得到连续且保持姿态的局部逆解")
