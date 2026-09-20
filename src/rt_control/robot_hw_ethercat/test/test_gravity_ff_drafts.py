from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

PACKAGE = Path(__file__).parents[1]
ESI = PACKAGE / "config/esi/ZeroErr_Driver_V3_2_0.xml"
MACHINES = PACKAGE / "config/machines"


def _load(name):
    return yaml.safe_load((MACHINES / name).read_text())


def _bytes(profile, direction):
    sizes = {"int8": 1, "uint8": 1, "int16": 2, "uint16": 2,
             "int32": 4, "uint32": 4, "int64": 8, "uint64": 8}
    return sum(sizes[channel["type"]] for pdo in profile[direction] for channel in pdo["channels"])


def test_esi_identity_capabilities_and_fixed_pdos_match_supplied_source():
    assert sha256(ESI.read_bytes()).hexdigest() == "b0c1108829f725f29f5386b3be6a808355ef3369f96d50c776539f0dc9e27e15"
    root = ET.parse(ESI).getroot()
    assert int(root.findtext("./Vendor/Id").replace("#x", "0x"), 0) == 0x5A65726F
    device = next(d for d in root.findall(".//Device") if int(d.find("Type").attrib["ProductCode"].replace("#x", "0x"), 0) == 0x29252)
    assert device.find(".//CoE").attrib == {"SdoInfo": "true", "SegmentedSdo": "true", "PdoConfig": "true", "CompleteAccess": "true", "PdoAssign": "true"}
    expected = {0x1600: (False, [0x607A, 0x60FE, 0x6040]), 0x1618: (True, [0x60B2]),
                0x1A00: (False, [0x6064, 0x60FD, 0x6041]), 0x1A11: (True, [0x606C]),
                0x1A12: (True, [0x6074]), 0x1A13: (True, [0x6077])}
    actual = {}
    for tag in ("RxPdo", "TxPdo"):
        for pdo in device.findall(tag):
            index = int(pdo.findtext("Index").replace("#x", "0x"), 0)
            if index in expected:
                actual[index] = (pdo.attrib.get("Fixed") == "1", [int(e.findtext("Index").replace("#x", "0x"), 0) for e in pdo.findall("Entry")])
    assert actual == expected


def test_candidates_are_draft_even_sized_and_not_runtime_registered():
    design = _load("zeroerr_csp_ff.draft.yaml")
    assert design["verified"] is False
    assert design["alternatives"]["assignment_only_a"]["loadable_with_current_stack"] is False
    for alternative in design["alternatives"].values():
        assert all(value % 2 == 0 for value in alternative["bytes"].values())
    assert all("zeroerr_csp_ff" not in path.read_text() for path in (PACKAGE / "variants").glob("*.yaml"))


def test_remap_candidate_has_fail_closed_downlink_and_raw_uplink():
    profile = _load("zeroerr_csp_ff_remap_b.draft.yaml")
    assert profile["verified"] is False
    assert _bytes(profile, "rpdo") == 12
    assert _bytes(profile, "tpdo") == 16
    torque = next(c for p in profile["rpdo"] for c in p["channels"] if c["index"] == 0x60B2)
    assert torque == {"index": 0x60B2, "sub_index": 0, "type": "int16", "command_interface": "effort", "factor": "TBD", "offset": 0, "default": 0}
    raw = {c["index"]: c for p in profile["tpdo"] for c in p["channels"] if c["index"] in (0x606C, 0x6077)}
    assert raw[0x606C]["state_interface"] == "velocity_actual_raw"
    assert raw[0x6077]["state_interface"] == "torque_actual_permille"
    assert all(c["factor"] == 1 for c in raw.values())


def test_uplink_only_candidate_has_no_effort_command_or_calibration_dependency():
    profile = _load("zeroerr_csp_ff_uplink_only.draft.yaml")
    assert profile["scope"] == "commissioning_raw_feedback_only"
    assert _bytes(profile, "rpdo") == 10
    assert _bytes(profile, "tpdo") == 16
    assert all(c.get("command_interface") != "effort" for p in profile["rpdo"] for c in p["channels"])
    assert "effort_calibration" not in profile
    raw = {c["state_interface"]: c for p in profile["tpdo"] for c in p["channels"] if "state_interface" in c}
    assert raw["velocity_actual_raw"]["factor"] == 1
    assert raw["torque_actual_permille"]["factor"] == 1
