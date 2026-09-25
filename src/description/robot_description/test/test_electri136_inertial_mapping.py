import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from test_v3_description_semantics import PACKAGE_ROOT, render_urdf
from test_v3_1_1_integration import fk, transform as origin_transform


SOURCE = PACKAGE_ROOT / "model_sources/arm_inertial_20260924"
MAPPING = json.loads((SOURCE / "mapping_result.json").read_text())
EXPECTED = {row["target_link"]: row for row in MAPPING["mappings"]}


def inertia(link):
    values = link.find("inertial/inertia").attrib
    return np.array([
        [float(values["ixx"]), float(values["ixy"]), float(values["ixz"])],
        [float(values["ixy"]), float(values["iyy"]), float(values["iyz"])],
        [float(values["ixz"]), float(values["iyz"]), float(values["izz"])],
    ])


def assert_mapping(link, expected):
    assert float(link.find("inertial/mass").attrib["value"]) == pytest.approx(
        expected["mass_kg"], abs=1e-10
    )
    com = [float(value) for value in link.find("inertial/origin").attrib["xyz"].split()]
    assert com == pytest.approx(expected["target_com_m"], abs=1e-10)
    tensor = inertia(link)
    assert tensor == pytest.approx(np.array(expected["target_inertia_kg_m2"]), abs=1e-10)
    eigenvalues = np.linalg.eigvalsh(tensor)
    assert eigenvalues[0] > 0.0
    assert eigenvalues[0] + eigenvalues[1] >= eigenvalues[2] - 1e-10


def test_mapping_identity_and_unverified_status_are_explicit():
    manifest = json.loads((SOURCE / "manifest.json").read_text())
    candidates = json.loads((SOURCE / "inertial_candidates.json").read_text())
    assert MAPPING["source_urdf_sha256"] == manifest["source_files_sha256"]["robot.urdf"]
    assert MAPPING["target_revision_before_mapping"] == (
        "ec69ca04297896c1296720324d23cdb8f80d9e63"
    )
    assert MAPPING["status"] == "temporary_cad_prior_requires_bench_calibration"
    assert manifest["hardware_verified"] is False
    assert manifest["runtime_model_modified"] is True
    assert candidates["runtime_ready"] is False
    assert len(MAPPING["mappings"]) == 16
    assert all(row["hardware_verified"] is False for row in MAPPING["mappings"])
    for row in MAPPING["mappings"]:
        transform = np.array(row["target_from_source"])
        rotation = transform[:3, :3]
        assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-9)
        assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-9)
        if row["source_link"].endswith("link1"):
            assert transform == pytest.approx(np.eye(4), abs=1e-9)
    assert hashlib.sha256((SOURCE / "mapping_result.json").read_bytes()).hexdigest() == (
        manifest["mapping_result_sha256"]
    )
    for relative_path, expected_hash in manifest["source_files_sha256"].items():
        assert hashlib.sha256((SOURCE / relative_path).read_bytes()).hexdigest() == expected_hash


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_mapped_arm_link1_through_link6_are_shared_by_both_variants(variant):
    robot = ET.fromstring(render_urdf(variant))
    for side in ("left", "right"):
        for index in range(1, 7):
            name = f"{side}_link{index}"
            assert_mapping(robot.find(f"link[@name='{name}']"), EXPECTED[name])


def test_gripper_fixed_and_moving_bodies_use_side_specific_mapping():
    robot = ET.fromstring(render_urdf("gripper"))
    for side in ("left", "right"):
        assert_mapping(robot.find(f"link[@name='{side}_link7']"), EXPECTED[f"{side}_link7"])
        assert_mapping(
            robot.find(f"link[@name='{side}_moving_jaw']"),
            EXPECTED[f"{side}_moving_jaw"],
        )
    arm_mass = {
        side: sum(float(robot.find(f"link[@name='{side}_link{i}']/inertial/mass").attrib["value"])
                  for i in range(1, 8))
        for side in ("left", "right")
    }
    assert arm_mass == pytest.approx({"left": 43.146, "right": 43.146}, abs=1e-10)


def test_mapped_properties_preserve_source_mass_distribution_under_anchor_alignment():
    source_robot = ET.parse(SOURCE / "robot.urdf").getroot()
    target_robot = ET.fromstring(render_urdf("gripper"))
    source_frames = fk(source_robot, {})
    target_frames = fk(target_robot, {})
    for row in MAPPING["mappings"]:
        source_name, target_name = row["source_link"], row["target_link"]
        side = target_name.split("_")[0]
        alignment = np.array(target_frames[f"{side}_link1"]) @ np.linalg.inv(
            np.array(source_frames[f"{side}_link1"])
        )
        source_link = source_robot.find(f"link[@name='{source_name}']")
        target_link = target_robot.find(f"link[@name='{target_name}']")
        source_com_frame = alignment @ np.array(source_frames[source_name]) @ np.array(
            origin_transform(source_link.find("inertial/origin"))
        )
        target_com_frame = np.array(target_frames[target_name]) @ np.array(
            origin_transform(target_link.find("inertial/origin"))
        )
        assert target_com_frame[:3, 3] == pytest.approx(source_com_frame[:3, 3], abs=1e-10)
        source_rotation = source_com_frame[:3, :3]
        target_rotation = target_com_frame[:3, :3]
        source_world_tensor = source_rotation @ inertia(source_link) @ source_rotation.T
        target_world_tensor = target_rotation @ inertia(target_link) @ target_rotation.T
        assert target_world_tensor == pytest.approx(source_world_tensor, abs=1e-10)
        expected_mapping = np.linalg.inv(np.array(target_frames[target_name])) @ (
            alignment @ np.array(source_frames[source_name])
        )
        assert np.array(row["target_from_source"]) == pytest.approx(expected_mapping, abs=1e-10)


def test_gripper_mapping_does_not_overwrite_suction_payload():
    robot = ET.fromstring(render_urdf("suction"))
    for side in ("left", "right"):
        link = robot.find(f"link[@name='{side}_link7']")
        assert float(link.find("inertial/mass").attrib["value"]) == 2.618
        assert link.find("inertial/origin").attrib["xyz"] == "-0.000330 -0.000125 0.070303"
        assert robot.find(f"link[@name='{side}_moving_jaw']") is None
