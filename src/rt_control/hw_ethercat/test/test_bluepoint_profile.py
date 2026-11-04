from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
PROFILE = PACKAGE / "config/profiles/bluepoint_p140000100.yaml"
MACHINE = PACKAGE.parent / "hwcfg_machines/machines/Alfa_V3/wrist_force_sensors.yaml"
ESI = PACKAGE / "config/esi/P140000107-1.0.1.1-ECXML.xml"


def test_bluepoint_esi_identity_and_pdo_layout_are_archived_exactly():
    root = ET.parse(ESI).getroot()
    vendor = root.find("Vendor")
    device = root.find("Descriptions/Devices/Device")
    assert vendor.findtext("Id") == "#x0000A1"
    assert vendor.findtext("Name") == "ACTI"
    assert device.find("Type").attrib == {"ProductCode": "#x00008081", "RevisionNo": "#x2"}
    assert device.findtext("Type") == "P140000107"
    assert [entry.findtext("Index") for entry in device.findall("RxPdo/Entry")] == [
        f"#x{index:04X}" for index in range(0x2000, 0x2008)
    ]
    assert [entry.findtext("Index") for entry in device.findall("TxPdo/Entry")] == [
        f"#x{index:04X}" for index in range(0x4000, 0x4009)
    ]


@pytest.mark.parametrize("side,position", [("left", 8), ("right", 16)])
def test_bluepoint_profile_is_state_only_and_matches_the_user_selected_docx_layout(side, position):
    record = yaml.safe_load(PROFILE.read_text())
    profile = record["driver_config"]
    machine = yaml.safe_load(MACHINE.read_text())
    sensor = next(item for item in machine["spec"]["sensors"] if item["side"] == side)
    assert sensor["ring_position"] == position
    assert machine["spec"]["full_ring_positions"] == "TBD"
    assert profile["metadata"]["verified"] is False
    assert not {"side", "ring_positions", "observed_ring_position"}.intersection(
        profile["metadata"]
    )
    assert profile["vendor_id"] == 0xA1 and profile["product_id"] == 0x8081
    assert profile["assign_activate"] == 0x0300 and profile["use_slave_pdo_defaults"] is True
    assert profile["metadata"]["revision_id"] == 1
    assert (
        profile["metadata"]["reference_esi_original_sha256"]
        == "8e654bdf540ebac4522f68f403b6a4568677c46ecec14db8451559fa028742ad"
    )
    assert (
        profile["metadata"]["reference_esi_sha256"]
        == "f92f783bfe91152163e4812b10314399a2f82ffa8135ea76295ee0f953b6e6f8"
    )
    assert (
        profile["metadata"]["reference_manual_sha256"]
        == "9c275c21bb171a99cd814c2a041b8bc9b03774ef1034cac523104c1e63ab8cdf"
    )
    assert record["force_raw_per_newton"] == record["torque_raw_per_newton_metre"] == 100000
    assert profile["metadata"]["status_code_semantics"] == "reserved"
    assert profile["metadata"]["temperature_raw_per_degree_celsius"] == "TBD"
    rpdo = profile["rpdo"][0]
    assert rpdo["index"] == 0x1600
    assert [channel["index"] for channel in rpdo["channels"]] == [0x7000] * 8
    assert [channel["sub_index"] for channel in rpdo["channels"]] == list(range(1, 9))
    assert all(
        channel["default"] == 0 and "command_interface" not in channel
        for channel in rpdo["channels"]
    )
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
    family = yaml.safe_load((PACKAGE / "config/profiles/families.yaml").read_text())["families"][
        "bluepoint"
    ]
    registry = yaml.safe_load((PACKAGE / "config/families.yaml").read_text())
    contract = registry["interface_contracts"][family["interface_contract"]]
    assert family["certified_modes"] == []
    assert contract["required_command_interfaces"] == []
    assert [item["name"] for item in contract["required_state_interfaces"]] == [
        *(f"channel_{index}_raw" for index in range(1, 7)),
        "status_code_raw",
        "sample_counter_raw",
        "temperature_raw",
    ]


def test_bluepoint_broadcaster_draft_reuses_the_shared_cpp_plugin_fail_closed():
    record = yaml.safe_load(PROFILE.read_text())
    assert record["unit_codes"] == [5, 5, 5, 7, 7, 7]
    assert record["decimals"] == [5] * 6
    assert record["broadcaster"]["validity_policy"] == "none"
    assert record["broadcaster"]["snapshot_source"] == "fixed_protocol"
