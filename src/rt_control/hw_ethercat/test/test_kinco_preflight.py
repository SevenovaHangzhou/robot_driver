"""Kinco startup admission: a held Shutdown controlword is accepted only when opted in."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "hw_semantic_components"))
from hw_ethercat.topology_preflight import preflight  # noqa: E402

IDENTITY = {"vendor_id": 0x6811E8, "product_code": 0x464445, "revision_number": 0x1F9}


def topology(allow):
    return {
        "master_id": 0,
        "responders": [{"ring_position": 0, **IDENTITY}],
        "axes": [{"ring_position": 0, "allowed_errors": [], "allow_shutdown_ready": allow}],
        "branches": [],
        "internal_ports": [],
        "preflight_required": True,
    }


def bus(control_word, status_word, phase="Idle"):
    def run(command, **_):
        if command[1] == "master":
            return f"Master0\n  Phase: {phase}\n  Active: no\n  Slaves: 1\n"
        if command[1] == "slaves":
            return (
                "=== Master 0, Slave 0 ===\nState: PREOP\n"
                f"Vendor Id:       0x{IDENTITY['vendor_id']:08x}\n"
                f"Product code:    0x{IDENTITY['product_code']:08x}\n"
                f"Revision number: 0x{IDENTITY['revision_number']:08x}\n"
            )
        index = int(command[-2], 16)
        value = {0x6040: control_word, 0x6041: status_word, 0x603F: 0}[index]
        return f"0x{value:04x} {value}\n"

    return run


@pytest.mark.parametrize("status_word", [0x0231, 0x4031, 0x0031])
def test_held_shutdown_is_admitted_for_opted_in_kinco_axes(status_word):
    assert preflight(topology(True), run=bus(0x0006, status_word)) == []


@pytest.mark.parametrize(
    "allow, control_word, status_word, phase",
    [
        (False, 0x0006, 0x0231, "Idle"),  # not opted in
        (True, 0x000F, 0x0237, "Idle"),  # operation enabled
        (True, 0x0007, 0x0233, "Idle"),  # switched on, not Shutdown
        (True, 0x0006, 0x0240, "Idle"),  # switch on disabled with a held Shutdown
        (True, 0x0006, 0x0231, "Operation"),  # master still running
    ],
)
def test_other_held_controlwords_stay_rejected(allow, control_word, status_word, phase):
    findings = preflight(topology(allow), run=bus(control_word, status_word, phase))
    assert [finding["problem"] for finding in findings] == ["already_controlled"]
