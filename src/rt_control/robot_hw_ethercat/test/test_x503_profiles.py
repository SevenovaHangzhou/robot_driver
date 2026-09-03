from pathlib import Path

import yaml


PROFILE_DIR = Path(__file__).resolve().parents[1] / "config" / "slaves"
EXPORTED_INTERFACES = {
    *(f"channel_{index}_raw" for index in range(1, 7)),
    *(f"sample_code_{index}_raw" for index in range(1, 7)),
}


def load_profile(side: str) -> dict:
    return yaml.safe_load(
        (PROFILE_DIR / f"x503_{side}.yaml").read_text(encoding="utf-8")
    )


def test_x503_profiles_have_confirmed_identity_and_fixed_pdo_layout():
    for side in ("right", "left"):
        profile = load_profile(side)
        assert profile["vendor_id"] == 0x00000503
        assert profile["product_id"] == 0x26483052
        assert profile["use_slave_pdo_defaults"] is True
        assert profile["assign_activate"] == 0x0300
        assert "sdo" not in profile

        assert len(profile["rpdo"]) == 1
        assert profile["rpdo"][0]["index"] == 0x1601
        assert len(profile["rpdo"][0]["channels"]) == 9
        assert [channel["type"] for channel in profile["rpdo"][0]["channels"]] == [
            *(["bool"] * 8),
            "bit8",
        ]

        assert len(profile["tpdo"]) == 1
        assert profile["tpdo"][0]["index"] == 0x1A00
        channels = profile["tpdo"][0]["channels"]
        assert len(channels) == 25
        assert all(channel["type"] == "int32" for channel in channels)
        assert {
            channel["state_interface"]
            for channel in channels
            if channel.get("state_interface") is not None
        } == EXPORTED_INTERFACES


def test_x503_profiles_encode_the_confirmed_daisy_chain_order():
    right = load_profile("right")["metadata"]
    left = load_profile("left")["metadata"]

    assert right["side"] == "right"
    assert right["ring_position"] == 14
    assert right["chain_index"] == 1
    assert right["upstream"] == "hub_position_13_out8"
    assert right["downstream"] == "left_force_sensor"

    assert left["side"] == "left"
    assert left["ring_position"] == 15
    assert left["chain_index"] == 2
    assert left["upstream"] == "right_force_sensor"
    assert left["downstream"] is None

    for metadata in (right, left):
        assert metadata["topology"] == "daisy_chain_from_hub_position_13_out8"
        assert metadata["channel_semantics"] == "unresolved"
        assert metadata["engineering_units"] == "unresolved"
        assert metadata["online_txpdo_size_bytes"] == 100
        assert metadata["esi_sm3_default_size_bytes"] == 6
