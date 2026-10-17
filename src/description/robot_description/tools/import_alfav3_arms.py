#!/usr/bin/env python3
"""Import the SolidWorks Alfav3 dual-arm export into model_sources/alfav3_arms_1011_4.

Usage (developer tool; needs trimesh, scipy and coacd, not a runtime dependency):
    uv run --isolated --python 3.10 --with coacd==1.0.14 --with trimesh==5.1.1 --with scipy \
        python tools/import_alfav3_arms.py /path/to/export_package_dir

The export keeps its geometry, inertials, joint origins and axes. The import only:
- renames links/joints to the robot convention (L1_joint -> left_joint1, Ltool_link -> left_tool_link);
- renames the robot/package to Alfav3 and mesh paths to model_sources-relative files;
- copies the original visual meshes unchanged;
- generates conservative collision meshes (CoACD convex decomposition or a single convex hull).
Joint limits stay as exported (all zero); the deployed entry sets reviewed limits.
"""

import hashlib
import json
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "model_sources/alfav3_arms_1011_4"
COACD = dict(threshold=0.03, max_convex_hull=20, preprocess_mode="auto", seed=0)
# CoACD works on a voxel-repaired copy, so pieces can miss the visual surface by ~0.3 mm; push each
# convex piece out by 1 mm so the envelope contains every visual vertex (verified below).
INFLATE_M = 0.001
DECOMPOSE = {"base_link", *(f"{side}_link{i}" for side in ("left", "right") for i in range(1, 6))}


def link_name(name):
    match = re.fullmatch(r"([LR])(\d|tool)_link", name)
    if not match:
        return name
    side = "left" if match[1] == "L" else "right"
    return f"{side}_tool_link" if match[2] == "tool" else f"{side}_link{match[2]}"


def joint_name(name):
    match = re.fullmatch(r"([LR])(\d|tool)_joint", name)
    side = "left" if match[1] == "L" else "right"
    return f"{side}_tool_fixed" if match[2] == "tool" else f"{side}_joint{match[2]}"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collision_mesh(visual, name):
    import coacd
    import trimesh
    from scipy.spatial import cKDTree

    mesh = trimesh.load_mesh(visual)
    if name in DECOMPOSE:
        coacd.set_log_level("error")
        parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **COACD)
        hulls = []
        for v, f in parts:
            hull = trimesh.Trimesh(v, f).convex_hull
            hulls.append(trimesh.Trimesh(hull.vertices + INFLATE_M * hull.vertex_normals, hull.faces).convex_hull)
        method = "coacd"
    else:
        hulls = [mesh.convex_hull]
        method = "convex_hull"
    merged = trimesh.util.concatenate(hulls)
    dense, _ = trimesh.sample.sample_surface(mesh, 400000, seed=1)
    probe, _ = trimesh.sample.sample_surface(merged, 20000, seed=2)
    gap = cKDTree(dense).query(probe)[0] * 1000.0
    # Exact containment for convex pieces: a vertex is inside a piece if it is behind every face plane.
    depth = np.full(len(mesh.vertices), np.inf)
    for hull in hulls:
        normals = hull.face_normals
        offsets = np.einsum("ij,ij->i", normals, hull.triangles[:, 0])
        depth = np.minimum(depth, (mesh.vertices @ normals.T - offsets).max(axis=1))
    stats = dict(
        method=method,
        pieces=len(hulls),
        triangles=int(len(merged.faces)),
        surface_gap_mm=dict(
            p50=round(float(np.percentile(gap, 50)), 1),
            p95=round(float(np.percentile(gap, 95)), 1),
            max=round(float(gap.max()), 1),
        ),
        visual_vertices_outside=int((depth > 1e-6).sum()),
        max_penetration_mm=round(max(0.0, float(depth.max())) * 1000.0, 3),
    )
    return merged, stats


def main(export_dir):
    export_dir = Path(export_dir)
    urdf = next((export_dir / "urdf").glob("*.urdf"))
    original_package = urdf.stem
    if TARGET.exists():
        shutil.rmtree(TARGET)
    (TARGET / "meshes/visual").mkdir(parents=True)
    (TARGET / "meshes/collision").mkdir(parents=True)

    robot = ET.parse(urdf).getroot()
    link_map = {link.get("name"): link_name(link.get("name")) for link in robot.findall("link")}
    joint_map = {joint.get("name"): joint_name(joint.get("name")) for joint in robot.findall("joint")}
    robot.set("name", "Alfav3")
    for link in robot.findall("link"):
        old = link.get("name")
        link.set("name", link_map[old])
        for kind in ("visual", "collision"):
            for mesh in link.findall(f"{kind}/geometry/mesh"):
                mesh.set("filename", f"meshes/{kind}/{link_map[old]}.STL")
    for joint in robot.findall("joint"):
        joint.set("name", joint_map[joint.get("name")])
        for tag in ("parent", "child"):
            element = joint.find(tag)
            element.set("link", link_map[element.get("link")])
    ET.indent(robot)
    ET.ElementTree(robot).write(TARGET / "robot.urdf", encoding="utf-8", xml_declaration=True)

    collision = {}
    originals = {}
    for old, new in link_map.items():
        source_mesh = export_dir / "meshes" / f"{old}.STL"
        originals[f"meshes/{old}.STL"] = sha256(source_mesh)
        shutil.copy2(source_mesh, TARGET / "meshes/visual" / f"{new}.STL")
        mesh, stats = collision_mesh(source_mesh, new)
        mesh.export(TARGET / "meshes/collision" / f"{new}.STL")
        collision[new] = stats
        print(new, stats, flush=True)

    originals[f"urdf/{urdf.name}"] = sha256(urdf)
    for extra in [*(export_dir / "urdf").glob("*.csv"), export_dir / "export.log"]:
        if extra.exists():
            originals[str(extra.relative_to(export_dir))] = sha256(extra)
    header = urdf.read_text(encoding="utf-8").splitlines()[1]
    import coacd
    import trimesh

    report = {
        "source": "SolidWorks to URDF Exporter dual-arm export, 2026-10-16",
        "exporter_header": header.strip(),
        "original_package_name": original_package,
        "original_files_sha256": originals,
        "renamed_package": "Alfav3",
        "link_rename": link_map,
        "joint_rename": joint_map,
        "kept_from_export": ["inertials", "joint origins", "joint axes", "visual meshes (bytes unchanged)"],
        "export_joint_limits": "all revolute limits are 0 in the export; deployed entry sets reviewed limits",
        "collision": {
            "generator": {"coacd": coacd.__version__ if hasattr(coacd, "__version__") else "1.0.14",
                          "trimesh": trimesh.__version__, "coacd_settings": COACD, "inflate_m": INFLATE_M,
                          "decomposed_links": sorted(DECOMPOSE)},
            "metric": "surface_gap_mm: distance from collision-surface samples to the visual surface; "
                      "large values on base_link include interior fill of the hollow shell",
            "links": collision,
        },
        "limitations": [
            "Inertials come from the SolidWorks export and are not hardware validated; "
            "masses differ from the previous V3.2.2 arm links.",
            "Collision meshes are generated envelopes; they are conservative and not visual geometry.",
        ],
    }
    (TARGET / "export_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    files = sorted(p for p in TARGET.rglob("*") if p.is_file() and p.name != "SHA256SUMS")
    (TARGET / "SHA256SUMS").write_text(
        "".join(f"{sha256(p)}  {p.relative_to(TARGET).as_posix()}\n" for p in files)
    )


if __name__ == "__main__":
    main(sys.argv[1])
