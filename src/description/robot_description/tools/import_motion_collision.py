"""Import source arm collision meshes into the existing link frames at model q=0.
Only collision STL contents and provenance/asset hashes are changed; no kinematic edits.
"""

import argparse, hashlib, json, math, struct
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET

DT = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def rotate(axis, a):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def origin(e):
    T = np.eye(4)
    if e is not None:
        T[:3, 3] = np.fromstring(e.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(e.get("rpy", "0 0 0"), sep=" ")
        T[:3, :3] = rotate([0, 0, 1], y) @ rotate([0, 1, 0], p) @ rotate([1, 0, 0], r)
    return T


class Model:
    def __init__(self, p):
        self.root = ET.parse(p).getroot()
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.parents = {j.find("child").get("link"): j for j in self.joints.values()}

    def frames(self, q):
        frames = {"arm_carriage": np.eye(4)}

        def fk(link):
            if link not in frames:
                j = self.parents[link]
                m = origin(j.find("origin"))
                turn = np.eye(4)
                if j.get("type") in ("revolute", "continuous"):
                    turn[:3, :3] = rotate(
                        np.fromstring(j.find("axis").get("xyz"), sep=" "),
                        q.get(j.get("name"), 0),
                    )
                frames[link] = fk(j.find("parent").get("link")) @ m @ turn
            return frames[link]

        for side in ("left", "right"):
            for i in range(1, 8):
                fk(f"{side}_link{i}")
        return frames


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source", type=Path)
    p.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    a = p.parse_args()
    src = Model(a.source / "robot.urdf")
    dst = Model(a.root / "urdf/robot_alfav3_mobile.urdf")
    S, D = src.frames({}), dst.frames({})
    source_manifest = json.loads((a.source / "manifest.json").read_text())
    for name, v in source_manifest["mesh_files"].items():
        assert sha(a.source / name) == v["sha256"], name
    info = {
        "source_bundle": "robot_urdf_stl_20261027",
        "coordinate_revision": source_manifest["coordinate_revision"],
        "source_urdf_sha256": sha(a.source / "robot.urdf"),
        "target_urdf_sha256": sha(a.root / "urdf/robot_alfav3_mobile.urdf"),
        "alignment": "Fixed per-link rigid transform at q=0, anchored to shared arm_carriage. Kinematics unchanged; not globally equivalent at all q.",
        "scope": "left/right link1..7 collision only; existing tool collision and TCP retained",
        "links": {},
    }
    for side in ("left", "right"):
        for i in range(1, 8):
            name = f"{side}_link{i}"
            sc = src.links[name].findall("collision")
            dc = dst.links[name].findall("collision")
            assert len(sc) == len(dc) == 1
            sm = sc[0].find("geometry/mesh")
            dm = dc[0].find("geometry/mesh")
            assert sm is not None and dm is not None
            source = a.source / sm.get("filename")
            target = a.root / dm.get("filename").removeprefix(
                "package://robot_description/"
            )
            C = np.linalg.inv(D[name]) @ S[name]
            G = (
                np.linalg.inv(origin(dc[0].find("origin")))
                @ C
                @ origin(sc[0].find("origin"))
            )
            raw = source.read_bytes()
            n = struct.unpack_from("<I", raw, 80)[0]
            assert len(raw) == 84 + 50 * n
            data = np.frombuffer(raw, DT, offset=84).copy()
            v = data["vertices"].astype(float) * np.fromstring(
                sm.get("scale", "1 1 1"), sep=" "
            )
            transformed = (v @ G[:3, :3].T + G[:3, 3]) / np.fromstring(
                dm.get("scale", "1 1 1"), sep=" "
            )
            data["vertices"] = transformed
            tri = data["vertices"].astype(float)
            norm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            size = np.linalg.norm(norm, axis=1)
            norm[size > 0] /= size[size > 0, None]
            data["normal"] = norm
            target.write_bytes(
                b"Motion collision transformed into current link frame; q0 alignment".ljust(
                    80, b" "
                )
                + struct.pack("<I", n)
                + data.tobytes()
            )
            actual = data["vertices"].astype(float) @ D[name][:3, :3].T + D[name][:3, 3]
            reference = (
                v @ (S[name] @ origin(sc[0].find("origin")))[:3, :3].T
                + (S[name] @ origin(sc[0].find("origin")))[:3, 3]
            )
            err = float(np.max(np.linalg.norm(actual - reference, axis=2)))
            assert err < 1e-6, (name, err)
            info["links"][name] = {
                "source_mesh": sm.get("filename"),
                "source_sha256": sha(source),
                "target_mesh": str(target.relative_to(a.root)),
                "target_sha256": sha(target),
                "triangles": n,
                "target_link_from_source_link_q0": C.tolist(),
                "target_mesh_from_source_mesh": G.tolist(),
                "zero_pose_max_vertex_error_m": err,
            }
    # Bound observations across named test poses; not a certified collision clearance.
    poses = {
        "zero": {},
        "both_j2_plus90": {"left_joint2": math.pi / 2, "right_joint2": math.pi / 2},
        "both_j2_plus122": {
            "left_joint2": math.radians(122),
            "right_joint2": math.radians(122),
        },
        "both_j2_minus92": {
            "left_joint2": math.radians(-92),
            "right_joint2": math.radians(-92),
        },
    }
    checks = {}
    for pose, q in poses.items():
        sp, dp = src.frames(q), dst.frames(q)
        errs = []
        for name, item in info["links"].items():
            sc = src.links[name].find("collision")
            sm = sc.find("geometry/mesh")
            v = np.frombuffer(
                (a.source / sm.get("filename")).read_bytes(), DT, offset=84
            )["vertices"].astype(float) * np.fromstring(
                sm.get("scale", "1 1 1"), sep=" "
            )
            C = np.array(item["target_link_from_source_link_q0"])
            A = dp[name] @ C @ origin(sc.find("origin"))
            B = sp[name] @ origin(sc.find("origin"))
            errs.append(
                float(
                    np.max(
                        np.linalg.norm(
                            (v @ A[:3, :3].T + A[:3, 3]) - (v @ B[:3, :3].T + B[:3, 3]),
                            axis=2,
                        )
                    )
                )
            )
        checks[pose] = {"q_rad": q, "maximum_collision_vertex_separation_m": max(errs)}
    info["pose_comparison"] = checks
    (a.root / "config/arm_collision_import.json").write_text(
        json.dumps(info, ensure_ascii=False, indent=2) + "\n"
    )
    assets = a.root / "config/mesh_assets.json"
    d = json.loads(assets.read_text())
    for item in info["links"].values():
        d["files"][item["target_mesh"]] = item["target_sha256"]
    assets.write_text(json.dumps(d, ensure_ascii=False, indent=2) + "\n")
    print(
        json.dumps(
            {
                "links": len(info["links"]),
                "max_q0_vertex_error_m": max(
                    x["zero_pose_max_vertex_error_m"] for x in info["links"].values()
                ),
                "poses": checks,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
