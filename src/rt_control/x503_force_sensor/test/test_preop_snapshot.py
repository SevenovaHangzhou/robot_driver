"""PREOP readback must finish before a controller can activate the master."""
import copy
from pathlib import Path

import pytest
import yaml

from x503_force_sensor.preop import (
    PreopRequired,
    SensorReadSpec,
    SnapshotError,
    read_preop_snapshot,
)


CONFIG = Path(__file__).resolve().parents[2] / "robot_hw_ethercat/config/x503b_readback.yaml"
SPECS = (
    SensorReadSpec("right_force_sensor", 14, 0x503, 0x26483052, 0x20111),
    SensorReadSpec("left_force_sensor", 15, 0x503, 0x26483052, 0x20111),
)


class FakeBus:
    def __init__(self):
        self.calls = []
        self.active = False
        self.states = {14: "PREOP", 15: "PREOP"}
        self.fail = None
        self.bad_unit = False
        self.flip_after = None
        self.vendor = 0x503

    @property
    def uploads(self):
        return [c for c in self.calls if "upload" in c]

    def __call__(self, argv):
        self.calls.append(list(argv))
        if "master" in argv:
            return ("Master0\n  Phase: " + ("Operation" if self.active else "Idle")
                    + "\n  Active: " + ("yes" if self.active else "no")
                    + "\n  Slaves: 18\n  Link: UP\n")
        pos = int(argv[argv.index("-p") + 1])
        if "slaves" in argv:
            return (f"State: {self.states[pos]}\nFlag: +\n"
                    f"Vendor Id: 0x{self.vendor:08x}\nProduct code: 0x26483052\n"
                    "Revision number: 0x00020111\nSerial number: 0x00000000\n")
        assert "upload" in argv, f"Unexpected device operation: {argv}"
        assert not self.active and self.states[pos] == "PREOP", "SDO after OP"
        sub = int(argv[-1])
        if self.fail == (pos, sub):
            raise SnapshotError("0-byte response")
        value = {6: 1, 7: 1, 8: 1, 9: 3, 10: 3, 11: 3,
                 12: 5, 13: 5, 14: 5, 15: 7, 16: 7, 17: 7}[sub]
        if self.bad_unit and pos == 14 and sub == 12:
            value = 6
        if self.flip_after == len(self.uploads):
            self.active = True
        return f"0x{value:08x} {value}\n"


def read(bus, **kwargs):
    return read_preop_snapshot(
        SPECS, yaml.safe_load(CONFIG.read_text()), startup_id="new-start",
        expected_responders=18, run=bus, **kwargs,
    )


def test_reads_each_whitelisted_parameter_once_and_keeps_actual_values():
    bus = FakeBus()
    snapshot = read(bus)
    assert snapshot["startup_id"] == "new-start"
    assert len(bus.uploads) == 24
    for pos in [14, 15]:
        calls = [c for c in bus.uploads if c[c.index("-p") + 1] == str(pos)]
        assert sorted(int(c[-1]) for c in calls) == list(range(6, 18))
        assert all(c[-2] == "0x8005" and "uint32" in c for c in calls)
    assert [s["values"]["snapshot_valid"] for s in snapshot["sensors"]] == ["true", "true"]
    assert snapshot["sensors"][0]["values"]["decimal_4"] == "3"
    assert snapshot["sensors"][1]["values"]["unit_6"] == "7"


@pytest.mark.parametrize("state", ["OP", "SAFEOP", "INIT", "PREOP + ERROR"])
def test_non_preop_device_is_rejected_before_any_sdo(state):
    bus = FakeBus()
    bus.states[14] = state
    with pytest.raises(PreopRequired):
        read(bus)
    assert bus.uploads == []


def test_active_master_is_rejected_before_any_sdo():
    bus = FakeBus()
    bus.active = True
    with pytest.raises(PreopRequired):
        read(bus)
    assert bus.uploads == []


def test_state_change_during_snapshot_stops_further_reads():
    bus = FakeBus()
    bus.flip_after = 3
    with pytest.raises(PreopRequired):
        read(bus)
    assert len(bus.uploads) == 3


def test_identity_mismatch_never_reads_parameters():
    bus = FakeBus()
    bus.vendor = 0x999
    with pytest.raises(PreopRequired):
        read(bus)
    assert bus.uploads == []


def test_incomplete_sensor_stays_invalid_without_hiding_other_sensor():
    bus = FakeBus()
    bus.fail = (14, 12)
    snapshot = read(bus)
    right, left = snapshot["sensors"]
    assert right["values"]["snapshot_valid"] == "false"
    assert "decimal_1" not in right["values"]
    assert left["values"]["snapshot_valid"] == "true"
    assert len([c for c in bus.uploads if c[c.index("-p") + 1] == "14"]) == 2


def test_unexpected_unit_does_not_become_valid_calibration():
    bus = FakeBus()
    bus.bad_unit = True
    snapshot = read(bus)
    assert snapshot["sensors"][0]["values"]["snapshot_valid"] == "false"


def test_config_cannot_expand_device_access_beyond_whitelist():
    bus = FakeBus()
    config = copy.deepcopy(yaml.safe_load(CONFIG.read_text()))
    config["read_only_sdo"]["decimal_subindices"][0] = 1
    with pytest.raises(SnapshotError):
        read_preop_snapshot(SPECS, config, startup_id="new-start", expected_responders=18, run=bus)
    assert bus.calls == []


def test_mock_snapshot_never_touches_the_master():
    bus = FakeBus()
    snapshot = read(bus, mock=True)
    assert bus.calls == []
    assert all(s["values"]["snapshot_valid"] == "false" for s in snapshot["sensors"])
