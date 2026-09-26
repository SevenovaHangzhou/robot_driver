"""Top-view chassis geometry derived from the shared Robot Model (URDF + meshes).

The operator console never stores its own copy of the chassis geometry. This
module takes the URDF published on ``/robot_description``, walks the kinematic
tree at zero joint positions, projects every visual mesh of the chassis onto
the base_link XY plane and extracts vector silhouettes (outer contours and
holes) for the web top view.

Pure Python + numpy; no ROS imports so it can be unit tested in isolation.
"""

from __future__ import annotations

import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional, Sequence

import numpy as np

GRID_M = 0.004  # silhouette raster resolution
SIMPLIFY_M = 0.003  # Douglas-Peucker tolerance
MIN_LOOP_AREA_M2 = 2e-4  # drop specks and tiny holes (< ~14 mm square)
MAX_MESH_TRIANGLES = 200_000

MeshResolver = Callable[[str], Optional[Path]]


class ModelError(Exception):
    pass


# --------------------------------------------------------------------------- math
def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _origin(element: Optional[ET.Element]) -> np.ndarray:
    matrix = np.eye(4)
    if element is None:
        return matrix
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    if len(xyz) != 3 or len(rpy) != 3 or not all(map(math.isfinite, xyz + rpy)):
        raise ModelError("invalid origin element")
    matrix[:3, :3] = _rpy_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


# --------------------------------------------------------------------------- meshes
def load_stl(path: Path) -> np.ndarray:
    """Return triangles as an (n, 3, 3) array in the file's units."""
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]
        if 0 < count <= MAX_MESH_TRIANGLES and len(data) == 84 + count * 50:
            record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
            return np.frombuffer(data[84:], dtype=record, count=count)["v"].astype(float)
    text = data.decode("ascii", errors="ignore")
    if not text.lstrip().startswith("solid"):
        raise ModelError(f"unsupported mesh format: {path.name}")
    vertices = [
        [float(v) for v in line.split()[1:4]]
        for line in text.splitlines()
        if line.strip().startswith("vertex")
    ]
    if not vertices or len(vertices) % 3 or len(vertices) > 3 * MAX_MESH_TRIANGLES:
        raise ModelError(f"invalid ASCII STL: {path.name}")
    return np.asarray(vertices, dtype=float).reshape(-1, 3, 3)


# --------------------------------------------------------------------------- silhouette
def _rasterize(triangles_2d: np.ndarray, origin: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Fill grid cells whose centres lie inside any projected triangle."""
    grid = np.zeros(shape, dtype=bool)
    for tri in triangles_2d:
        (x0, y0), (x1, y1), (x2, y2) = tri
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if abs(area) < 1e-12:
            continue
        lo = np.floor((tri.min(0) - origin) / GRID_M).astype(int)
        hi = np.ceil((tri.max(0) - origin) / GRID_M).astype(int)
        lo = np.clip(lo, 0, [shape[1] - 1, shape[0] - 1])
        hi = np.clip(hi, 0, [shape[1] - 1, shape[0] - 1])
        xs = origin[0] + (np.arange(lo[0], hi[0] + 1) + 0.5) * GRID_M
        ys = origin[1] + (np.arange(lo[1], hi[1] + 1) + 0.5) * GRID_M
        px, py = np.meshgrid(xs, ys)
        w0 = (x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)
        w1 = (x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)
        w2 = (x0 - px) * (y1 - py) - (x1 - px) * (y0 - py)
        if area > 0:
            inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        else:
            inside = (w0 <= 0) & (w1 <= 0) & (w2 <= 0)
        grid[lo[1]:hi[1] + 1, lo[0]:hi[0] + 1] |= inside
    return grid


# Marching-squares edge table: corner bits tl=8 tr=4 br=2 bl=1; edges t r b l.
_SEGMENTS = {
    1: [("l", "b")], 2: [("b", "r")], 3: [("l", "r")], 4: [("r", "t")],
    5: [("l", "t"), ("r", "b")], 6: [("b", "t")], 7: [("l", "t")], 8: [("t", "l")],
    9: [("t", "b")], 10: [("t", "r"), ("b", "l")], 11: [("t", "r")], 12: [("r", "l")],
    13: [("r", "b")], 14: [("b", "l")],
}


def _trace(grid: np.ndarray) -> list[list[tuple[float, float]]]:
    """Closed boundary loops of a boolean grid, in grid-corner coordinates."""
    padded = np.pad(grid, 1)
    h, w = padded.shape
    edge_point = {
        "t": lambda r, c: (c + 0.5, r),
        "b": lambda r, c: (c + 0.5, r + 1),
        "l": lambda r, c: (c, r + 0.5),
        "r": lambda r, c: (c + 1, r + 0.5),
    }
    nxt: dict[tuple[float, float], tuple[float, float]] = {}
    tl = padded[:-1, :-1].astype(int)
    tr = padded[:-1, 1:].astype(int)
    br = padded[1:, 1:].astype(int)
    bl = padded[1:, :-1].astype(int)
    code = tl * 8 + tr * 4 + br * 2 + bl
    for r, c in zip(*np.nonzero((code > 0) & (code < 15))):
        for a, b in _SEGMENTS[int(code[r, c])]:
            nxt[edge_point[a](r, c)] = edge_point[b](r, c)
    loops = []
    while nxt:
        start, point = next(iter(nxt.items()))
        loop = [start]
        del nxt[start]
        while point != start and point in nxt:
            loop.append(point)
            point = nxt.pop(point)
        if len(loop) >= 3:
            # Grid corners were padded by one cell; convert to unpadded cell units.
            loops.append([(x - 1.0, y - 1.0) for x, y in loop])
    return loops


def _simplify(points: np.ndarray, tolerance: float) -> np.ndarray:
    """Douglas-Peucker on an open polyline."""
    if len(points) < 3:
        return points
    start, end = points[0], points[-1]
    seg = end - start
    length = math.hypot(*seg)
    if length < 1e-12:
        dist = np.hypot(*(points - start).T)
    else:
        cross = seg[0] * (points[:, 1] - start[1]) - seg[1] * (points[:, 0] - start[0])
        dist = np.abs(cross) / length
    index = int(np.argmax(dist))
    if dist[index] <= tolerance:
        return np.array([start, end])
    left = _simplify(points[: index + 1], tolerance)
    right = _simplify(points[index:], tolerance)
    return np.vstack([left[:-1], right])


def _loop_area(points: np.ndarray) -> float:
    x, y = points[:, 0], points[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def silhouette(triangles_2d: np.ndarray) -> list[list[list[float]]]:
    """Vector outline loops (outer contours and holes) of projected triangles."""
    if len(triangles_2d) == 0:
        return []
    lo = triangles_2d.reshape(-1, 2).min(0) - 2 * GRID_M
    hi = triangles_2d.reshape(-1, 2).max(0) + 2 * GRID_M
    shape = (int(math.ceil((hi[1] - lo[1]) / GRID_M)), int(math.ceil((hi[0] - lo[0]) / GRID_M)))
    grid = _rasterize(triangles_2d, lo, shape)
    loops = []
    for loop in _trace(grid):
        pts = np.asarray(loop) * GRID_M + lo
        if abs(_loop_area(pts)) < MIN_LOOP_AREA_M2:
            continue
        # Split the closed loop at its farthest point so both halves simplify well.
        far = int(np.argmax(np.hypot(*(pts - pts[0]).T)))
        a = _simplify(pts[: far + 1], SIMPLIFY_M)
        b = _simplify(np.vstack([pts[far:], pts[:1]]), SIMPLIFY_M)
        simplified = np.vstack([a[:-1], b[:-1]])
        if len(simplified) >= 3:
            loops.append([[round(float(x), 4), round(float(y), 4)] for x, y in simplified])
    return loops


# --------------------------------------------------------------------------- URDF
@dataclass
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray


@dataclass
class TopViewModel:
    version: str
    parts: list[dict] = field(default_factory=list)
    modules: list[dict] = field(default_factory=list)
    bounds: list[float] = field(default_factory=list)
    missing_meshes: list[str] = field(default_factory=list)

    def to_message(self) -> dict:
        return {
            "version": self.version,
            "frame": "base_link",
            "parts": self.parts,
            "modules": self.modules,
            "bounds": self.bounds,
            "missing_meshes": self.missing_meshes,
        }


class UrdfTree:
    def __init__(self, urdf_xml: str) -> None:
        try:
            self.root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            raise ModelError(f"invalid URDF: {exc}") from exc
        self.links = {link.get("name"): link for link in self.root.findall("link")}
        self.joints: dict[str, _Joint] = {}
        self.parent_joint: dict[str, _Joint] = {}
        self.children: dict[str, list[_Joint]] = {}
        for element in self.root.findall("joint"):
            joint = _Joint(
                name=element.get("name", ""),
                kind=element.get("type", ""),
                parent=element.find("parent").get("link"),
                child=element.find("child").get("link"),
                origin=_origin(element.find("origin")),
            )
            self.joints[joint.name] = joint
            self.parent_joint[joint.child] = joint
            self.children.setdefault(joint.parent, []).append(joint)

    def transform(self, link: str, frame: str = "base_link") -> np.ndarray:
        """Pose of ``link`` in ``frame`` with every joint at zero."""
        matrix = np.eye(4)
        current = link
        while current != frame:
            joint = self.parent_joint.get(current)
            if joint is None:
                raise ModelError(f"link {link} is not below {frame}")
            matrix = joint.origin @ matrix
            current = joint.parent
        return matrix

    def subtree_links(self, link: str) -> list[str]:
        result = [link]
        for joint in self.children.get(link, []):
            result.extend(self.subtree_links(joint.child))
        return result

    def visual_meshes(self, link: str) -> Iterable[tuple[str, np.ndarray, np.ndarray]]:
        element = self.links.get(link)
        if element is None:
            return
        for visual in element.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
            yield mesh.get("filename", ""), _origin(visual.find("origin")), np.asarray(scale)


def _projected(tree: UrdfTree, link: str, base_from: np.ndarray, resolver: MeshResolver,
               missing: list[str]) -> list[tuple[str, np.ndarray]]:
    """(mesh uri, triangles projected to base_link XY) for every visual of ``link``."""
    result = []
    for uri, visual_origin, scale in tree.visual_meshes(link):
        path = resolver(uri)
        if path is None or not path.is_file():
            missing.append(uri)
            continue
        tris = load_stl(path) * scale
        matrix = base_from @ tree.transform(link) @ visual_origin
        flat = tris.reshape(-1, 3) @ matrix[:3, :3].T + matrix[:3, 3]
        result.append((uri, flat.reshape(-1, 3, 3)[:, :, :2]))
    return result


def build_top_view(
    urdf_xml: str,
    resolver: MeshResolver,
    chassis_root: str,
    steering_joints: Sequence[str],
    base_frame: str = "base_link",
) -> TopViewModel:
    """Top-view silhouettes of the chassis in ``base_frame``.

    Steering subtrees (steering joint child and everything below it) are
    returned per module, relative to the steering axis, so the client can
    rotate them with the measured steering angle.
    """
    tree = UrdfTree(urdf_xml)
    if chassis_root not in tree.links:
        raise ModelError(f"chassis root link '{chassis_root}' not found in URDF")
    missing: list[str] = []
    identity = np.eye(4)

    module_links: set[str] = set()
    modules = []
    for name in steering_joints:
        joint = tree.joints.get(name)
        if joint is None or joint.kind not in ("revolute", "continuous"):
            raise ModelError(f"steering joint '{name}' not found in URDF")
        axis = tree.transform(joint.child, base_frame)[:3, 3]
        links = tree.subtree_links(joint.child)
        module_links.update(links)
        parts = []
        for link in links:
            role = "steer" if link == joint.child else "wheel"
            for _uri, tris in _projected(tree, link, identity, resolver, missing):
                loops = silhouette(tris - axis[:2])
                if loops:
                    parts.append({"link": link, "role": role, "loops": loops})
        modules.append(
            {"joint": name, "x": round(float(axis[0]), 4), "y": round(float(axis[1]), 4),
             "parts": parts}
        )

    body_parts = []
    for link in tree.subtree_links(chassis_root):
        if link in module_links:
            continue
        hidden = link != chassis_root  # e.g. suspension carriage below the plate
        for uri, tris in _projected(tree, link, identity, resolver, missing):
            loops = silhouette(tris)
            if loops:
                body_parts.append(
                    {"link": link, "mesh": Path(uri).name, "hidden": hidden, "loops": loops}
                )
    if not body_parts:
        raise ModelError("no chassis meshes could be projected")

    points = [p for part in body_parts for loop in part["loops"] for p in loop]
    for module in modules:
        for part in module["parts"]:
            for loop in part["loops"]:
                points.extend([[p[0] + module["x"], p[1] + module["y"]] for p in loop])
    array = np.asarray(points)
    bounds = [round(float(v), 4) for v in (*array.min(0), *array.max(0))]
    digest = hashlib.sha256(urdf_xml.encode("utf-8")).hexdigest()[:16]
    return TopViewModel(version=digest, parts=body_parts, modules=modules, bounds=bounds,
                        missing_meshes=sorted(set(missing)))


def package_resolver(share_lookup: Callable[[str], str]) -> MeshResolver:
    """Resolve ``package://`` and ``file://`` mesh URIs."""

    def resolve(uri: str) -> Optional[Path]:
        if uri.startswith("package://"):
            package, _, rel = uri[len("package://"):].partition("/")
            try:
                return Path(share_lookup(package)) / rel
            except Exception:  # noqa: BLE001 - unknown package is reported as missing
                return None
        if uri.startswith("file://"):
            return Path(uri[len("file://"):])
        return None

    return resolve
