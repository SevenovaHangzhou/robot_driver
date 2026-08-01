from copy import deepcopy
import math
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PROFILE_DIR = PACKAGE_DIR / "config" / "slaves"
XACRO_PATH = PACKAGE_DIR / "urdf" / "ecat.ros2_control.xacro"
POSITION_POLARITY_SDO = {
    "index": 0x607E,
    "sub_index": 0,
    "type": "uint8",
    "value": 0x80,
}


def load_profile(name):
    return yaml.safe_load((PROFILE_DIR / f"{name}.yaml").read_text(encoding="utf-8"))


def without_position_polarity(profile):
    result = deepcopy(profile)
    result["sdo"] = [entry for entry in result["sdo"] if entry["index"] != 0x607E]
    return result


def test_only_authorized_axes_use_dedicated_ti5_profiles():
    xacro = XACRO_PATH.read_text(encoding="utf-8")

    assert '<xacro:ti5_axis joint_name="right_joint2" ring_position="2" profile="ti5_right_joint2"/>' in xacro
    assert '<xacro:ti5_axis joint_name="left_joint3" ring_position="9" profile="ti5_left_joint3"/>' in xacro
    assert '<xacro:ti5_axis joint_name="left_joint2" ring_position="8" profile="ti5_j2"/>' in xacro
    assert '<xacro:ti5_axis joint_name="right_joint3" ring_position="3" profile="ti5_j3"/>' in xacro


def test_position_polarity_sdo_is_limited_to_right_joint2_and_left_joint3():
    for dedicated_name, shared_name in (
        ("ti5_right_joint2", "ti5_j2"),
        ("ti5_left_joint3", "ti5_j3"),
    ):
        dedicated = load_profile(dedicated_name)
        shared = load_profile(shared_name)

        assert [entry for entry in dedicated["sdo"] if entry["index"] == 0x607E] == [
            POSITION_POLARITY_SDO
        ]
        assert all(entry["index"] != 0x607E for entry in shared["sdo"])
        assert without_position_polarity(dedicated) == shared


def test_zeroerr_position_transform_is_limited_to_right_joint4():
    xacro = XACRO_PATH.read_text(encoding="utf-8")

    assert '<xacro:zeroerr_axis joint_name="right_joint4" ring_position="4" profile="zeroerr_right_joint4"/>' in xacro
    assert '<xacro:zeroerr_axis joint_name="left_joint5" ring_position="11" profile="zeroerr_left_joint5"/>' in xacro
    assert '<xacro:zeroerr_axis joint_name="left_joint4" ring_position="10" profile="zeroerr_j4"/>' in xacro
    assert '<xacro:zeroerr_axis joint_name="right_joint5" ring_position="5" profile="zeroerr_j5"/>' in xacro

    dedicated = load_profile("zeroerr_right_joint4")
    expected = deepcopy(load_profile("zeroerr_j4"))
    expected_command = next(
        channel for channel in expected["rpdo"][0]["channels"] if channel["index"] == 0x607A
    )
    expected_state = next(
        channel for channel in expected["tpdo"][0]["channels"] if channel["index"] == 0x6064
    )
    expected_command["factor"] *= -1
    expected_state["factor"] *= -1
    expected_state["offset"] *= -1

    assert dedicated == expected
    assert all(entry["index"] != 0x607E for entry in dedicated["sdo"])
    assert all(
        channel["index"] != 0x606C
        for channel in dedicated["rpdo"][0]["channels"] + dedicated["tpdo"][0]["channels"]
    )


def test_left_joint5_keeps_shared_zeroerr_direction():
    assert load_profile("zeroerr_left_joint5") == load_profile("zeroerr_j5")


def test_zeroerr_position_transform_negates_physical_position_but_preserves_ros_feedback():
    profile = load_profile("zeroerr_right_joint4")
    command = next(channel for channel in profile["rpdo"][0]["channels"] if channel["index"] == 0x607A)
    state = next(channel for channel in profile["tpdo"][0]["channels"] if channel["index"] == 0x6064)

    for command_degrees, expected_raw_target in ((90.0, 131072), (-90.0, 393216), (0.0, 262144)):
        command_rad = math.radians(command_degrees)
        raw_target = round(command["factor"] * command_rad + command["offset"])
        ros_feedback = state["factor"] * raw_target + state["offset"]

        assert raw_target == expected_raw_target
        assert math.isclose(ros_feedback, command_rad, abs_tol=2e-8)
