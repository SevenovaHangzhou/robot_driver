import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "model_sources" / "robot_v3_2_2"
RUNTIME_URDF = PACKAGE_ROOT / "urdf" / "robot_v3_2_2.urdf"


def test_v3_2_2_source_snapshots_are_immutable():
    manifest = json.loads((PACKAGE_ROOT / "config" / "v3_2_2_source_manifest.json").read_text())
    for relative_path, expected_hash in manifest["source_files"].items():
        payload = (PACKAGE_ROOT / relative_path).read_bytes()
        assert hashlib.sha256(payload).hexdigest() == expected_hash


def test_v3_2_2_runtime_entry_has_ground_footprint_root():
    robot = ET.parse(RUNTIME_URDF).getroot()
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = robot.findall("joint")
    children = {joint.find("child").attrib["link"] for joint in joints}
    assert links - children == {"base_footprint"}
    assert len(links) == 19
    assert len(joints) == 18
    assert sum(joint.attrib["type"] != "fixed" for joint in joints) == 17

    footprint = robot.find("joint[@name='base_footprint_to_base_link']")
    assert footprint is not None
    assert footprint.attrib["type"] == "fixed"
    assert footprint.find("parent").attrib["link"] == "base_footprint"
    assert footprint.find("child").attrib["link"] == "base_link"
    assert footprint.find("origin").attrib == {"xyz": "0 0 0.335", "rpy": "0 0 0"}


def test_v3_2_2_runtime_meshes_are_installable_and_complete():
    robot = ET.parse(RUNTIME_URDF).getroot()
    prefix = "package://robot_description/model_sources/robot_v3_2_2/"
    mesh_uris = {
        mesh.attrib["filename"]
        for mesh in robot.findall(".//mesh")
    }
    assert len(mesh_uris) == 49
    for uri in mesh_uris:
        assert uri.startswith(prefix)
        assert (PACKAGE_ROOT / uri.removeprefix("package://robot_description/")).is_file()

    report = json.loads((SOURCE_ROOT / "export_report.json").read_text())
    assert report["status"] == "ok"
    assert len(report["warnings"]) == 1
    assert "16 unassigned CAD parts" in report["warnings"][0]


def test_v3_2_2_runtime_entry_passes_urdfdom():
    result = subprocess.run(
        ["check_urdf", str(RUNTIME_URDF)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "root Link: base_footprint" in result.stdout


def test_default_profile_uses_v3_2_2_joint_inventory():
    robot = ET.parse(PACKAGE_ROOT / "urdf" / "robot.urdf").getroot()
    joints = robot.findall("joint")
    children = {joint.find("child").attrib["link"] for joint in joints}
    links = {link.attrib["name"] for link in robot.findall("link")}
    assert links - children == {"base_footprint"}
    assert sum(joint.attrib["type"] != "fixed" for joint in joints) == 17
    assert {"left_tool0", "right_tool0"} <= links
    assert not {"left_suction", "right_suction"} & links
    for side in ("left", "right"):
        tool_joint = robot.find(f"joint[@name='{side}_tool0_fixed']")
        assert tool_joint.find("parent").attrib["link"] == f"{side}_link7"
        assert tool_joint.find("origin").attrib == {"xyz": "0 0 0.151", "rpy": "0 0 0"}
