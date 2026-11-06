"""Owner findings -> located self-check entries (stage A)."""

import yaml

from integ_composer.self_check import entries

HEAD = {
    "source": "head_gimbal/H2_joint/head_motor",
    "address": {"can_id": 2},
    "fault_table": "damiao_dm",
}
TABLES = {
    "damiao_dm": {
        "vendor": "DaMiao",
        "faults": [{"code": "0x0009", "message": "欠压", "dree": "RT_DEVICE_FAULT", "page": 12}],
    }
}


def finding(**values):
    return {"owner": "hw_can_damiao", "blocking": True, "detail": "d", **values}


def test_head_motor_findings_are_located_by_can_id_and_looked_up():
    diagnostics = yaml.safe_dump({"devices": [HEAD], "fault_tables": TABLES}, allow_unicode=True)
    [entry] = entries(
        diagnostics, [finding(address={"can_id": 2}, problem="fault_code", fault_code=9)]
    )
    assert entry["source"] == "head_gimbal/H2_joint/head_motor"
    assert entry["dree"] == "RT_DEVICE_FAULT" and "欠压" in entry["detail"]


def test_unlisted_can_node_gets_a_fallback_source():
    [entry] = entries(None, [finding(address={"can_id": 3}, problem="offline")])
    assert (entry["source"], entry["dree"]) == ("can/node3", "RT_DEVICE_OFFLINE")
