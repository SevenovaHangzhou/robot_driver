"""Kinco's exact Shutdown/Ready pair is opt-in; admission remains read-only."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_hw_ethercat.topology_preflight import preflight


def topology(allow=True):
    return {
        "master_id": 0,
        "preflight_required": True,
        "responders": [
            {
                "ring_position": 0,
                "vendor_id": 0x681168,
                "product_code": 0x464445,
                "revision_number": 505,
            }
        ],
        "axes": [
            {
                "ring_position": 0,
                "allowed_errors": [],
                **({"allow_shutdown_ready": allow} if allow is not None else {}),
            }
        ],
        "branches": [],
        "internal_ports": [],
        "pdo_checks": [{"ring_position": 0, "objects": {0x1600: [0x60400010]}}],
        "position_windows": [{"ring_position": 0, "raw_min": -100, "raw_max": 100}],
    }


def reader(
    cw=6,
    sw=0x0231,
    error=0,
    *,
    active=False,
    phase="Idle",
    al="PREOP",
    second_sw=None,
    second_error=None,
    pdo=0x60400010,
    position=0,
):
    calls = []
    reads = {}

    def run(argv, **_):
        calls.append(argv)
        assert argv[1] in ("master", "slaves", "upload"), "preflight must never write"
        if argv[1] == "master":
            return f"Phase: {phase}\nActive: {'yes' if active else 'no'}\nSlaves: 1\n"
        if argv[1] == "slaves":
            return (
                f"=== Master 0, Slave 0 ===\nState: {al}\nVendor Id: 0x00681168\n"
                "Product code: 0x00464445\nRevision number: 0x000001f9\n"
            )
        index, sub = int(argv[-2], 16), int(argv[-1])
        reads[index] = reads.get(index, 0) + 1
        value = {0x6040: cw, 0x6041: sw, 0x603F: error, 0x6064: position}.get(
            index, 1 if sub == 0 else pdo
        )
        if reads[index] > 1:
            if index == 0x6041 and second_sw is not None:
                value = second_sw
            if index == 0x603F and second_error is not None:
                value = second_error
        return f"0x{value:x} {value}\n"

    return run, calls


@pytest.mark.parametrize("status", [0x0021, 0x0231, 0x0031, 0x4031, 0x4231])
def test_shutdown_ready_admission_preserves_other_readback_bits(status):
    run, calls = reader(sw=status)
    preflight(topology(), run=run)
    assert sum(c[1] == "upload" and c[-2] == "0x6041" for c in calls) == 2
    assert any(c[1] == "upload" and c[-2] == "0x6064" for c in calls)


@pytest.mark.parametrize("allow", [None, False, "true", 1])
def test_other_families_and_non_boolean_opt_in_keep_zero_only(allow):
    with pytest.raises(RuntimeError, match="unsupported startup state"):
        preflight(topology(allow), run=reader()[0])


@pytest.mark.parametrize(
    "kwargs",
    [
        {"cw": 7},
        {"cw": 15},
        {"cw": 0x86},
        {"cw": 2},
        {"sw": 0x27},
        {"sw": 0x23},
        {"sw": 0x40},
        {"sw": 0x07},
        {"sw": 0x08},
        {"sw": 0x0F},
        {"error": 0x730F},
        {"error": 0x1234},
        {"active": True},
        {"phase": "Operation"},
        {"al": "OP"},
        {"al": "PREOP + ERROR"},
        {"second_sw": 0x27},
        {"second_error": 0x1234},
        {"pdo": 0x60600008},
        {"position": 101},
    ],
)
def test_unsafe_or_inconsistent_conditions_are_still_rejected(kwargs):
    with pytest.raises(RuntimeError):
        preflight(topology(), run=reader(**kwargs)[0])


def test_zero_controlword_legacy_path_still_works():
    preflight(topology(None), run=reader(cw=0, sw=0x40)[0])
    with pytest.raises(RuntimeError):
        preflight(topology(None), run=reader(cw=0, sw=0x27)[0])
