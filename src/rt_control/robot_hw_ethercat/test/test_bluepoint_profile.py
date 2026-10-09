from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PROFILES = {
    side: PACKAGE / f"config/slaves/bluepoint_p140000100_{side}.yaml"
    for side in ("left", "right")
}
ESI = PACKAGE / "config/esi/P140000107-1.0.1.1-ECXML.xml"
FAMILIES = PACKAGE / "config/families.yaml"
ALFA_V1 = PACKAGE / "variants/alfa_v1.yaml"
BROADCASTER_DRAFT = (
    PACKAGE.parent / "rt_control_bringup/config/bluepoint_force_sensors.draft.yaml"
)


def test_bluepoint_esi_identity_and_pdo_layout_are_archived_exactly():
    root = ET.parse(ESI).getroot()
    vendor = root.find("Vendor")
    device = root.find("Descriptions/Devices/Device")

    assert vendor.findtext("Id") == "#x0000A1"
    assert vendor.findtext("Name") == "ACTI"
    assert device.find("Type").attrib == {
        "ProductCode": "#x00008081",
        "RevisionNo": "#x2",
    }
    assert device.findtext("Type") == "P140000107"
    assert [entry.findtext("Index") for entry in device.findall("RxPdo/Entry")] == [
        f"#x{index:04X}" for index in range(0x2000, 0x2008)
    ]
    assert [entry.findtext("Index") for entry in device.findall("TxPdo/Entry")] == [
        f"#x{index:04X}" for index in range(0x4000, 0x4009)
    ]


@pytest.mark.parametrize("side,position", [("left", 2), ("right", 4)])
def test_bluepoint_profile_is_state_only_and_matches_the_user_selected_docx_layout(side, position):
    profile = yaml.safe_load(PROFILES[side].read_text(encoding="utf-8"))
    assert profile["metadata"]["side"] == side
    assert profile["metadata"]["ring_positions"] == [position]
    assert profile["metadata"]["observed_ring_position"] == position
    assert profile["metadata"]["verified"] is False

    assert profile["vendor_id"] == 0xA1
    assert profile["product_id"] == 0x8081
    assert profile["assign_activate"] == 0x0300
    assert profile["use_slave_pdo_defaults"] is True
    assert profile["metadata"]["revision_id"] == 1
    assert profile["metadata"]["reference_esi_original_sha256"] == (
        "8e654bdf540ebac4522f68f403b6a4568677c46ecec14db8451559fa028742ad"
    )
    assert profile["metadata"]["reference_esi_sha256"] == (
        "f92f783bfe91152163e4812b10314399a2f82ffa8135ea76295ee0f953b6e6f8"
    )
    assert profile["metadata"]["reference_manual_sha256"] == (
        "9c275c21bb171a99cd814c2a041b8bc9b03774ef1034cac523104c1e63ab8cdf"
    )
    assert profile["metadata"]["force_raw_per_newton"] == 100000
    assert profile["metadata"]["torque_raw_per_newton_metre"] == 100000
    assert profile["metadata"]["status_code_semantics"] == "reserved"
    assert profile["metadata"]["temperature_raw_per_degree_celsius"] == "TBD"
    rpdo = profile["rpdo"][0]
    assert rpdo["index"] == 0x1600
    assert [channel["index"] for channel in rpdo["channels"]] == [0x7000] * 8
    assert [channel["sub_index"] for channel in rpdo["channels"]] == list(range(1, 9))
    assert all(channel["default"] == 0 for channel in rpdo["channels"])
    assert all("command_interface" not in channel for channel in rpdo["channels"])
    tpdo = profile["tpdo"][0]
    assert tpdo["index"] == 0x1A00
    assert [channel["index"] for channel in tpdo["channels"]] == [0x6000] * 9
    assert [channel["sub_index"] for channel in tpdo["channels"]] == list(range(1, 10))
    assert [channel.get("state_interface") for channel in tpdo["channels"]] == [
        *(f"channel_{index}_raw" for index in range(1, 7)),
        "status_code_raw",
        "sample_counter_raw",
        "temperature_raw",
    ]


def test_bluepoint_family_contract_is_separate_from_x503():
    registry = yaml.safe_load(FAMILIES.read_text(encoding="utf-8"))
    family = registry["families"]["bluepoint_p140000100"]
    contract = registry["interface_contracts"][family["interface_contract"]]

    assert family == {
        "identity_profile": "bluepoint_p140000100_left",
        "interface_contract": "bluepoint_wrench_input",
        "certified_modes": [],
    }
    assert contract["required_command_interfaces"] == []
    assert [item["name"] for item in contract["required_state_interfaces"]] == [
        *(f"channel_{index}_raw" for index in range(1, 7)),
        "status_code_raw",
        "sample_counter_raw",
        "temperature_raw",
    ]


def test_bluepoint_broadcaster_draft_reuses_the_shared_cpp_plugin_fail_closed():
    document = yaml.safe_load(BROADCASTER_DRAFT.read_text(encoding="utf-8"))

    assert document["verified"] is False
    assert document["plugin"] == (
        "rt_force_torque_broadcaster/ForceTorqueBroadcaster"
    )
    assert [sensor["side"] for sensor in document["sensors"]] == ["left", "right"]
    assert [sensor["wrench_topic"] for sensor in document["sensors"]] == [
        "/rt_control/left_wrist/wrench",
        "/rt_control/right_wrist/wrench",
    ]
    for sensor in document["sensors"]:
        assert sensor["profile"] == f"bluepoint_p140000100_{sensor['side']}"
        assert sensor["ring_position"] == (2 if sensor["side"] == "left" else 4)
        assert sensor["frame_id"] == "TBD"
        assert sensor["calibration_valid"] is False
        assert sensor["snapshot_source"] == "fixed_protocol"
        assert sensor["scale_factors"] == [0.00001] * 6
        assert sensor["decimals"] == [5] * 6
        assert sensor["unit_codes"] == [5, 5, 5, 7, 7, 7]
        assert sensor["validity_policy"] == "none"
        assert sensor["diagnostic_name"].startswith(
            "/robot/rt_control/bluepoint/"
        )
    assert yaml.safe_load(ALFA_V1.read_text(encoding="utf-8"))["sensors"] == [
        {
            "sensor_name": "right_force_sensor",
            "family": "x503",
            "ring_position": 14,
            "profile": "x503_right",
            "wrench_topic": "/rt_control/right_x503b/wrench",
            "raw_topic": "/rt_control/right_x503b/raw",
            "frame_id": "right_ft_sensor_link",
        },
        {
            "sensor_name": "left_force_sensor",
            "family": "x503",
            "ring_position": 15,
            "profile": "x503_left",
            "wrench_topic": "/rt_control/left_x503b/wrench",
            "raw_topic": "/rt_control/left_x503b/raw",
            "frame_id": "left_ft_sensor_link",
        },
    ]
