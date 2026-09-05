from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src/x503_sdo_snapshot_node.cpp"


def test_only_unit_and_decimal_sdos_are_read():
    text = SOURCE.read_text(encoding="utf-8")

    assert '"readback_config"' in text
    config = (
        SOURCE.parents[2] / "robot_hw_ethercat/config/x503b_readback.yaml"
    ).read_text(encoding="utf-8")
    assert "index: 0x8005" in config
    assert "validity_policy: unresolved" in config
    assert "valid_sample_codes: []" in config
    assert "decimal_subindices: [6, 7, 8, 9, 10, 11]" in config
    assert "unit_subindices: [12, 13, 14, 15, 16, 17]" in config
    assert "expected_unit_codes: [5, 5, 5, 7, 7, 7]" in config
    assert "sdo_download" not in text
    assert "set_sdo" not in text
    assert "0x8001" not in text
    assert "0x8002" not in text
    assert "0x8007" not in text


def test_unit_contract_is_fail_closed():
    text = SOURCE.read_text(encoding="utf-8")
    assert "expected_unit_codes_" in text
    assert "decimals[channel] > 9U" in text
    assert "snapshot_valid" in text
