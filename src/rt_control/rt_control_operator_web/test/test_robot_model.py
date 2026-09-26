import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rt_control_operator_web.robot_model import (  # noqa: E402
    ModelError,
    UrdfTree,
    build_top_view,
    load_stl,
    package_resolver,
    silhouette,
)


def box_triangles(x0, y0, z0, x1, y1, z1):
    c = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5), (3, 0, 4), (3, 4, 7)]
    return np.array([[c[i] for i in f] for f in faces], dtype=float)


def write_binary_stl(path: Path, tris: np.ndarray) -> None:
    with path.open("wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for tri in tris:
            f.write(struct.pack("<3f", 0, 0, 0))
            for v in tri:
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")


def ring_triangles(r_out, r_in, n=48):
    tris = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p = lambda r, a: (r * math.cos(a), r * math.sin(a), 0.0)  # noqa: E731
        tris.append([p(r_in, a0), p(r_out, a0), p(r_out, a1)])
        tris.append([p(r_in, a0), p(r_out, a1), p(r_in, a1)])
    return np.array(tris)


URDF = """<robot name="t">
  <link name="base_footprint"/>
  <joint name="fp" type="fixed"><parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0.2 0 0.4"/></joint>
  <link name="base_link"/>
  <joint name="to_chassis" type="fixed"><parent link="base_link"/><child link="chassis"/>
    <origin xyz="-0.2 0 0"/></joint>
  <link name="chassis"><visual><origin xyz="0 0 0"/>
    <geometry>
      <mesh filename="package://pkg/plate.stl" scale="0.001 0.001 0.001"/>
    </geometry></visual></link>
  <joint name="steer_fl" type="revolute"><parent link="chassis"/><child link="caster"/>
    <origin xyz="0.3 0.3 0" rpy="3.14159265 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="caster"><visual>
    <geometry><mesh filename="package://pkg/disk.stl"/></geometry></visual></link>
  <joint name="drive_fl" type="revolute"><parent link="caster"/><child link="wheel"/>
    <origin xyz="0 0.03 0.2" rpy="-1.5707963 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <link name="wheel"><visual>
    <geometry><mesh filename="package://pkg/wheel.stl"/></geometry></visual></link>
  <joint name="susp" type="prismatic"><parent link="chassis"/><child link="carriage"/>
    <origin xyz="-0.3 0 -0.2"/><axis xyz="0 0 1"/>
    <limit lower="-0.1" upper="0" effort="1" velocity="1"/></joint>
  <link name="carriage"><visual>
    <geometry><mesh filename="package://pkg/bar.stl"/></geometry></visual></link>
</robot>"""


@pytest.fixture
def meshes(tmp_path: Path) -> Path:
    # 1.0 x 1.0 m plate (mm units) with a 0.2 x 0.2 m square opening.
    plate = np.vstack([
        box_triangles(-500, -500, 0, 500, -100, 10),
        box_triangles(-500, 100, 0, 500, 500, 10),
        box_triangles(-500, -100, 0, -100, 100, 10),
        box_triangles(100, -100, 0, 500, 100, 10),
    ])
    write_binary_stl(tmp_path / "plate.stl", plate)
    write_binary_stl(tmp_path / "disk.stl", ring_triangles(0.18, 0.03))
    write_binary_stl(tmp_path / "wheel.stl", box_triangles(-0.1, -0.1, -0.06, 0.1, 0.1, 0.0))
    write_binary_stl(tmp_path / "bar.stl", box_triangles(-0.05, -0.3, 0, 0.05, 0.3, 0.02))
    return tmp_path


def resolver_for(root: Path):
    def share(package: str) -> str:
        if package != "pkg":
            raise KeyError(package)
        return str(root)

    return package_resolver(share)


def test_load_binary_and_ascii_stl(tmp_path: Path) -> None:
    tris = box_triangles(0, 0, 0, 1, 1, 1)
    write_binary_stl(tmp_path / "b.stl", tris)
    assert load_stl(tmp_path / "b.stl").shape == (12, 3, 3)
    ascii_text = "solid t\n" + "".join(
        "facet normal 0 0 0\nouter loop\n" + "".join(f"vertex {x} {y} {z}\n" for x, y, z in tri)
        + "endloop\nendfacet\n" for tri in tris) + "endsolid t\n"
    (tmp_path / "a.stl").write_text(ascii_text)
    assert np.allclose(load_stl(tmp_path / "a.stl"), tris)
    (tmp_path / "x.stl").write_bytes(b"garbage")
    with pytest.raises(ModelError):
        load_stl(tmp_path / "x.stl")


def test_silhouette_keeps_holes() -> None:
    loops = silhouette(ring_triangles(0.18, 0.05)[:, :, :2])
    assert len(loops) == 2
    radii = sorted(max(math.hypot(x, y) for x, y in loop) for loop in loops)
    assert radii[0] == pytest.approx(0.05, abs=0.01)
    assert radii[1] == pytest.approx(0.18, abs=0.01)


def test_zero_pose_transform() -> None:
    tree = UrdfTree(URDF)
    wheel = tree.transform("wheel")
    assert wheel[:3, 3] == pytest.approx([0.1, 0.27, -0.2], abs=1e-6)


def test_top_view(meshes: Path) -> None:
    model = build_top_view(URDF, resolver_for(meshes), "chassis", ["steer_fl"]).to_message()
    assert model["frame"] == "base_link"
    assert model["missing_meshes"] == []
    plate = next(p for p in model["parts"] if p["link"] == "chassis")
    assert plate["hidden"] is False and len(plate["loops"]) == 2  # outline + opening
    carriage = next(p for p in model["parts"] if p["link"] == "carriage")
    assert carriage["hidden"] is True
    (module,) = model["modules"]
    assert (module["x"], module["y"]) == pytest.approx((0.1, 0.3), abs=1e-6)
    roles = {p["role"]: p for p in module["parts"]}
    assert set(roles) == {"steer", "wheel"}
    wheel = np.array(roles["wheel"]["loops"][0])
    # Joint offset and mesh thickness cancel: the tyre sits under the steering
    # axis, 0.2 m long along X (rolling direction) and 0.06 m wide along Y.
    assert wheel[:, 1].mean() == pytest.approx(0.0, abs=0.01)
    assert np.ptp(wheel[:, 0]) == pytest.approx(0.2, abs=0.01)
    assert np.ptp(wheel[:, 1]) == pytest.approx(0.06, abs=0.01)
    assert model["bounds"][0] == pytest.approx(-0.7, abs=0.01)
    assert model["bounds"][2] == pytest.approx(0.3, abs=0.02)
    assert len(model["version"]) == 16


def test_missing_meshes_are_reported(meshes: Path) -> None:
    (meshes / "bar.stl").unlink()
    model = build_top_view(URDF, resolver_for(meshes), "chassis", ["steer_fl"]).to_message()
    assert model["missing_meshes"] == ["package://pkg/bar.stl"]


@pytest.mark.parametrize(
    "root, joints",
    [("nope", ["steer_fl"]), ("chassis", ["drive_missing"]), ("chassis", ["susp"])],
)
def test_invalid_inputs(meshes: Path, root, joints) -> None:
    with pytest.raises(ModelError):
        build_top_view(URDF, resolver_for(meshes), root, joints)


def test_invalid_urdf() -> None:
    with pytest.raises(ModelError):
        UrdfTree("<robot")


def test_resolver_schemes(tmp_path: Path) -> None:
    resolve = resolver_for(tmp_path)
    assert resolve("package://pkg/a/b.stl") == tmp_path / "a" / "b.stl"
    assert resolve("package://other/x.stl") is None
    assert resolve("file:///tmp/x.stl") == Path("/tmp/x.stl")
    assert resolve("http://x") is None
