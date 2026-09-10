import math
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PROFILE_DIR = PACKAGE_DIR / "config" / "slaves"
VARIANT_PATH = PACKAGE_DIR / "variants" / "alfa_v1.yaml"


def load_profile(name):
    return yaml.safe_load((PROFILE_DIR / f"{name}.yaml").read_text(encoding="utf-8"))


def channel(profile, pdo_name, object_index):
    for pdo in profile[pdo_name]:
        for candidate in pdo["channels"]:
            if candidate["index"] == object_index:
                return candidate
    raise AssertionError(f"missing object 0x{object_index:04X} in {pdo_name}")


def axis_binding(family, joint_name):
    descriptor = yaml.safe_load(VARIANT_PATH.read_text(encoding="utf-8"))
    matches = [
        axis
        for axis in descriptor["axes"]
        if axis["family"] == family and axis["joint_name"] == joint_name
    ]
    assert len(matches) == 1
    return matches[0]


def test_only_authorized_axes_use_dedicated_ti5_profiles():
    assert axis_binding("ti5", "right_joint2")["profile"] == "ti5_right_joint2"
    assert axis_binding("ti5", "left_joint3")["profile"] == "ti5_left_joint3"
    assert axis_binding("ti5", "left_joint2")["profile"] == "ti5_j2"
    assert axis_binding("ti5", "right_joint3")["profile"] == "ti5_j3"


def test_ti5_fixed_position_pdos_are_preserved_and_registered_in_order():
    for name in ("ti5_j2", "ti5_j3", "ti5_left_joint3", "ti5_right_joint2"):
        profile = load_profile(name)
        assert profile["use_slave_pdo_defaults"] is True
        assert profile["auto_fault_reset"] is False
        assert profile["auto_state_transitions"] is False
        for direction, pdo_index, expected in (
            ("rpdo", 0x1601, [(0x6040, 0, "uint16"), (0x607A, 0, "int32")]),
            ("tpdo", 0x1A01, [(0x6041, 0, "uint16"), (0x6064, 0, "int32")]),
        ):
            assert len(profile[direction]) == 1
            pdo = profile[direction][0]
            assert pdo["index"] == pdo_index
            assert [(item["index"], item["sub_index"], item["type"])
                    for item in pdo["channels"]] == expected


def test_ti5_position_polarity_is_reversed_by_master_mapping_only():
    for dedicated_name, shared_name in (
        ("ti5_right_joint2", "ti5_j2"),
        ("ti5_left_joint3", "ti5_j3"),
    ):
        dedicated = load_profile(dedicated_name)
        shared = load_profile(shared_name)

        assert all(entry["index"] != 0x607E for entry in dedicated["sdo"])
        assert all(entry["index"] != 0x607E for entry in shared["sdo"])

        dedicated_command = channel(dedicated, "rpdo", 0x607A)
        dedicated_state = channel(dedicated, "tpdo", 0x6064)
        shared_command = channel(shared, "rpdo", 0x607A)
        shared_state = channel(shared, "tpdo", 0x6064)

        assert dedicated_command["factor"] == -shared_command["factor"]
        assert dedicated_state["factor"] == -shared_state["factor"]


def test_zeroerr_position_transform_is_limited_to_authorized_axes():
    assert axis_binding("zeroerr", "right_joint4")["profile"] == "zeroerr_right_joint4"
    assert axis_binding("zeroerr", "right_joint5")["profile"] == "zeroerr_right_joint5"
    assert axis_binding("zeroerr", "left_joint5")["profile"] == "zeroerr_left_joint5"
    assert axis_binding("zeroerr", "left_joint4")["profile"] == "zeroerr_j4"

    for dedicated_name, shared_name in (
        ("zeroerr_right_joint4", "zeroerr_j4"),
        ("zeroerr_right_joint5", "zeroerr_j5"),
        ("zeroerr_left_joint5", "zeroerr_j5"),
    ):
        dedicated = load_profile(dedicated_name)
        expected = yaml.safe_load(yaml.safe_dump(load_profile(shared_name)))
        expected_command = channel(expected, "rpdo", 0x607A)
        expected_state = channel(expected, "tpdo", 0x6064)
        expected_command["factor"] *= -1
        expected_state["factor"] *= -1
        expected_state["offset"] *= -1

        assert dedicated == expected
        assert all(entry["index"] != 0x607E for entry in dedicated["sdo"])
        assert all(
            channel["index"] != 0x606C
            for channel in dedicated["rpdo"][0]["channels"] + dedicated["tpdo"][0]["channels"]
        )


def test_shared_joint5_profile_remains_unreversed():
    shared = load_profile("zeroerr_j5")
    command = channel(shared, "rpdo", 0x607A)
    state = channel(shared, "tpdo", 0x6064)

    assert command["factor"] > 0
    assert state["factor"] > 0


def test_zeroerr_position_transform_negates_physical_position_but_preserves_ros_feedback():
    profile = load_profile("zeroerr_right_joint4")
    command = channel(profile, "rpdo", 0x607A)
    state = channel(profile, "tpdo", 0x6064)

    for command_degrees, expected_raw_target in ((90.0, 131072), (-90.0, 393216), (0.0, 262144)):
        command_rad = math.radians(command_degrees)
        raw_target = round(command["factor"] * command_rad + command["offset"])
        ros_feedback = state["factor"] * raw_target + state["offset"]

        assert raw_target == expected_raw_target
        assert math.isclose(ros_feedback, command_rad, abs_tol=2e-8)
