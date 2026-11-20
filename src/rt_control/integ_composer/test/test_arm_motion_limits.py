"""Compose final arm limits from the URDF and the shared RT acceleration policy."""

import shutil
import xml.etree.ElementTree as ET

import pytest
import yaml

from integ_composer.compose import compose
from test_bench_left_arm import UNIT, shares as shares
from test_compose import ARM_ACCELERATION_LIMITS


@pytest.mark.parametrize("side", ["left", "right", "dual"])
def test_arm_limits_follow_joint_names_and_changed_urdf(shares, tmp_path, side):
    model = tmp_path / "model"
    shutil.copytree(shares["robot_description"], model)
    shares["robot_description"] = model
    urdf = model / "urdf/robot_alfav3_mobile.urdf"
    robot = ET.parse(urdf)
    velocities = {}
    for joint in robot.getroot().findall("joint"):
        name = joint.attrib["name"]
        if name in ARM_ACCELERATION_LIMITS:
            velocities[name] = 0.1 + 0.01 * len(velocities)
            joint.find("limit").set("velocity", str(velocities[name]))
    robot.write(urdf)

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
            [ARM_ACCELERATION_LIMITS[joint] for joint in parameters["joints"]]
        )


def test_missing_acceleration_rejects_composition(shares, tmp_path):
    path = shares["hwcfg_machines"] / "machines/Alfa_V3/policies/arm_acceleration_limits.yaml"
    limits = yaml.safe_load(path.read_text())
    del limits["acceleration_rad_s2"]["right_joint4"]
    path.write_text(yaml.safe_dump(limits))
    with pytest.raises(ValueError, match=r"no value for \['right_joint4'\]"):
        compose(
            robot_variant="Alfa_V3",
            machine_unit=UNIT,
            physical_profile="bench_dual_arm",
            control_scope="bench_dual_arm",
            package_share=lambda name: str(shares[name]),
            runtime_dir=tmp_path / "runtime",
            use_mock_hardware=True,
        )
