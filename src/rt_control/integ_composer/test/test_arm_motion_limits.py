"""Compose final arm limits from the model URDF and its companion acceleration file."""

import json
import math
import shutil
import xml.etree.ElementTree as ET

import pytest
import yaml

from integ_composer.compose import compose
from test_bench_left_arm import UNIT, shares as shares


@pytest.mark.parametrize("side", ["left", "right", "dual"])
def test_arm_limits_follow_joint_names_and_changed_urdf(shares, tmp_path, side):
    model = tmp_path / "model"
    shutil.copytree(shares["robot_description"], model)
    shares["robot_description"] = model
    urdf = model / "urdf/robot_alfav3_mobile.urdf"
    robot = ET.parse(urdf)
    path = model / "config/joint_limits_alfav3_mobile.yaml"
    limits = yaml.safe_load(path.read_text())
    accelerations = {}
    velocities = {}
    for joint in robot.getroot().findall("joint"):
        name = joint.attrib["name"]
        if name.startswith(("left_joint", "right_joint")):
            accelerations[name] = 0.5 + 0.05 * len(accelerations)
            limits["joints"][name]["max_acceleration_rad_s2"] = accelerations[name]
            velocities[name] = 0.1 + 0.01 * len(velocities)
            joint.find("limit").set("velocity", str(velocities[name]))
    robot.write(urdf)
    path.write_text(yaml.safe_dump(limits))

    policy = shares["hwcfg_machines"] / (
        f"machines/Alfa_V3/policies/Alfa_V3_bench_{side}_arm_motion.yaml"
    )
    motion = yaml.safe_load(policy.read_text())
    motion["joints"].reverse()
    policy.write_text(yaml.safe_dump(motion))
    result = compose(
        robot_variant="Alfa_V3",
        machine_unit=UNIT,
        physical_profile=f"bench_{side}_arm",
        control_scope=f"bench_{side}_arm",
        package_share=lambda name: str(shares[name]),
        runtime_dir=tmp_path / "runtime",
        use_mock_hardware=True,
    )
    controllers = [f"{side}_arm_jtc"]
    if side == "dual":
        controllers.append("dual_arm_rolling")
    for name in controllers:
        parameters = result.controllers[name]["ros__parameters"]
        assert parameters["joints"] == motion["joints"]
        assert parameters["joint_velocity_limits"] == [
            velocities[joint] for joint in parameters["joints"]
        ]
        assert parameters["joint_acceleration_limits"] == pytest.approx(
            [accelerations[joint] for joint in parameters["joints"]]
        )


def test_missing_acceleration_rejects_composition(shares, tmp_path):
    model = tmp_path / "model"
    shutil.copytree(shares["robot_description"], model)
    shares["robot_description"] = model
    path = model / "config/joint_limits_alfav3_mobile.yaml"
    limits = yaml.safe_load(path.read_text())
    del limits["joints"]["right_joint4"]["max_acceleration_rad_s2"]
    path.write_text(yaml.safe_dump(limits))
    with pytest.raises(ValueError, match=r"no acceleration limit for \['right_joint4'\]"):
        compose(
            robot_variant="Alfa_V3",
            machine_unit=UNIT,
            physical_profile="bench_dual_arm",
            control_scope="bench_dual_arm",
            package_share=lambda name: str(shares[name]),
            runtime_dir=tmp_path / "runtime",
            use_mock_hardware=True,
        )


@pytest.mark.parametrize("side", ["left", "right", "dual"])
def test_production_arm_speeds_use_manual_output_rpm(shares, tmp_path, side):
    specs = json.loads(
        (shares["robot_description"] / "config/alfav3_arm_module_specs.json").read_text()
    )
    result = compose(
        robot_variant="Alfa_V3",
        machine_unit=UNIT,
        physical_profile=f"bench_{side}_arm",
        control_scope=f"bench_{side}_arm",
        package_share=lambda name: str(shares[name]),
        runtime_dir=tmp_path / "runtime",
        use_mock_hardware=True,
    )
    limits = yaml.safe_load(
        (shares["robot_description"] / "config/joint_limits_alfav3_mobile.yaml").read_text()
    )["joints"]
    checked = 0
    for controller in result.controllers.values():
        parameters = controller["ros__parameters"]
        if "joint_velocity_limits" not in parameters:
            continue
        expected = [
            specs["joints"][joint.split("_", 1)[1]]["output_peak_rpm"] * math.tau / 60
            for joint in parameters["joints"]
        ]
        assert parameters["joint_velocity_limits"] == pytest.approx(expected)
        assert parameters["joint_acceleration_limits"] == pytest.approx(
            [limits[joint]["max_acceleration_rad_s2"] for joint in parameters["joints"]]
        )
        checked += 1
    assert checked == (2 if side == "dual" else 1)


@pytest.mark.parametrize("value", [0, -1, True, "2", float("nan"), float("inf")])
def test_invalid_model_acceleration_rejected_before_artifact(shares, tmp_path, value):
    model = tmp_path / "model"
    shutil.copytree(shares["robot_description"], model)
    shares["robot_description"] = model
    path = model / "config/joint_limits_alfav3_mobile.yaml"
    limits = yaml.safe_load(path.read_text())
    limits["joints"]["left_joint1"]["max_acceleration_rad_s2"] = value
    path.write_text(yaml.safe_dump(limits))
    from jsonschema import ValidationError

    with pytest.raises((ValueError, ValidationError)):
        compose(
            robot_variant="Alfa_V3",
            machine_unit=UNIT,
            physical_profile="bench_left_arm",
            control_scope="bench_left_arm",
            package_share=lambda name: str(shares[name]),
            runtime_dir=tmp_path / "runtime",
            use_mock_hardware=True,
        )
