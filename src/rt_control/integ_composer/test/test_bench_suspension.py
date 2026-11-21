"""Active-suspension bench scope: the Tongyi PP drive alone on the bench master (ELECTRI-224)."""

from pathlib import Path
import shutil
import sys

import pytest
import yaml

from integ_composer.compose import compose, preflight
from integ_composer.schema.primitives import load_yaml

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
sys.path.insert(0, str(RT / "hw_ethercat"))
from hw_ethercat import rt_control_renderer as OWNER

UNIT = "Alfa_V3-bench-01"
SELECTION = dict(
    robot_variant="Alfa_V3",
    machine_unit=UNIT,
    physical_profile="bench_suspension",
    control_scope="bench_suspension",
)
ZERO = 1_000_000


@pytest.fixture
def shares(tmp_path, monkeypatch):
    data = tmp_path / "machine_config"
    shutil.copytree(RT / "hwcfg_machines", data)
    monkeypatch.setattr(OWNER, "_share", lambda: RT / "hw_ethercat")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["hwcfg_machines"] = data
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def confirm(shares):
    """Test-only: the bench facts a user confirms before the first real start."""
    root = shares["hwcfg_machines"]
    for relative, update in (
        ("machines/Alfa_V3/bench_suspension.yaml", lambda doc: doc.update(verified=True)),
        (
            f"units/{UNIT}/bench_suspension_zero.yaml",
            lambda doc: (
                doc.update(verified=True),
                doc["spec"].update(confirmed_by="user"),
                doc["spec"]["axes"][0].update(zero_counts=ZERO),
            ),
        ),
    ):
        path = root / relative
        document = load_yaml(path)
        update(document)
        path.write_text(yaml.safe_dump(document, sort_keys=False))


def arguments(shares):
    return dict(**SELECTION, package_share=lambda name: str(shares[name]))


def scan(slaves=19, position=ZERO, saved_mode_entry=0x60600008):
    """Bench enumeration of 2026-11-17: arms, sensors and couplers passive; Tongyi at ring 18."""
    identities = {0: (0x00100000, 0x10F40931, 0x10000), 17: (0x00100000, 0x10F40932, 0x10000)}
    identities.update({ring: (0xA1, 0x8081, 1) for ring in (8, 16)})
    identities.update({ring: (0x5A65726F, 0x00029252, 1) for ring in [*range(1, 8), *range(9, 16)]})
    identities[18] = (0x34E, 0x445566, 0)
    saved = {
        0x1C12: [0x1600],
        0x1C13: [0x1A00],
        0x1600: [
            0x60400010,
            0x60710010,
            0x60FF0020,
            0x607A0020,
            0x60810020,
            saved_mode_entry,
            0x607F0020,
        ],
        0x1A00: [
            0x60410010,
            0x603F0010,
            0x60780010,
            0x606C0020,
            0x60640020,
            0x60610008,
            0x60000008,
            0x23000010,
            0x23010010,
            0x60FD0020,
            0x60770010,
        ],
    }

    def run(command, **_):
        assert command[0] == "ethercat" and command[1] in {"master", "slaves", "upload"}
        if command[1] == "master":
            return f"Master0\n  Phase: Idle\n  Active: no\n  Slaves: {slaves}\n"
        if command[1] == "slaves":
            return "".join(
                f"=== Master 0, Slave {ring} ===\nState: PREOP\n"
                f"Vendor Id:       0x{vendor:08x}\nProduct code:    0x{product:08x}\n"
                f"Revision number: 0x{revision:08x}\n"
                for ring, (vendor, product, revision) in sorted(identities.items())
            )
        assert int(command[command.index("-p") + 1]) == 18
        index, sub = int(command[-2], 16), int(command[-1])
        if index in saved:
            value = len(saved[index]) if sub == 0 else saved[index][sub - 1]
        elif index == 0x6041:
            value = 0x0250
        elif index == 0x6064:
            value = position
        else:
            value = 0
        return f"0x{value:x} {value}\n"

    return run


def test_mock_scope_enables_one_pp_axis_and_its_command_controller(shares, tmp_path):
    result = compose(**arguments(shares), runtime_dir=tmp_path / "mock")
    system = result.systems[0]["result"]
    assert system["managed_joints"] == ["suspension"]
    assert result.lifecycle["hardware_activation_order"] == ["ecat_suspension"]
    assert result.lifecycle["bus_locks"] == ["ethercat-master-0"]
    enable = result.controllers["enable_manager"]["ros__parameters"]
    assert enable["enable_batch_joint_names"] == ["suspension"]
    assert enable["motion_groups"] == ["suspension"]
    assert enable["suspension"] == {"motion_controller": "suspension_command_controller"}
    assert enable["al_state_interfaces"] == ["ethercat_slave_18/al_state"]
    command = result.controllers["suspension_command_controller"]["ros__parameters"]
    assert command == {
        "interface_names": ["position", "profile_velocity", "pp_sequence", "pp_halt"],
        "joint": "suspension",
    }
    responders = system["expected_topology"]["responders"]
    assert [item["ring_position"] for item in responders] == list(range(19))


def test_real_start_is_refused_until_the_bench_facts_are_confirmed(shares, tmp_path):
    with pytest.raises(ValueError, match="machine.verified, unit.verified, suspension.zero_counts"):
        compose(**arguments(shares), runtime_dir=tmp_path / "real", use_mock_hardware=False)


def test_real_drive_config_carries_the_confirmed_scale_speed_and_ramps(shares, tmp_path):
    confirm(shares)
    result = compose(**arguments(shares), runtime_dir=tmp_path / "real", use_mock_hardware=False)
    system = result.systems[0]["result"]
    config = yaml.safe_load(system["runtime_files"]["suspension.yaml"])
    pp = config["pp"]
    # 65535 counts per motor turn over 20 mm (40 mm lead, 2:1); the motor's positive is down.
    assert pp["counts_per_metre"] == pytest.approx(-65535 / 0.02)
    assert (pp["zero_counts"], pp["min_position"], pp["max_position"]) == (ZERO, 0.0, 0.18)
    # 0x6081/0x607F in 0.1 r/min: 1 m/s = 50 r/s = 30000; 30 mm/s = 900.
    assert pp["profile_velocity_per_metre_per_second"] == pytest.approx(30000.0)
    assert (pp["max_velocity"], pp["max_profile_velocity"]) == (0.03, 900)
    assert config["sdo"] == [
        {"index": 0x6083, "sub_index": 0, "type": "uint32", "value": 5556},
        {"index": 0x6084, "sub_index": 0, "type": "uint32", "value": 5556},
    ]
    assert config["use_slave_pdo_defaults"] is True
    assert (config["vendor_id"], config["product_id"], config["assign_activate"]) == (
        0x34E,
        0x445566,
        0x300,
    )


def test_preflight_checks_identity_saved_pdos_and_the_start_position(shares, tmp_path):
    confirm(shares)
    result = compose(**arguments(shares), runtime_dir=tmp_path / "real", use_mock_hardware=False)
    preflight(result, run=scan())
    # Inside the 2.5 mm hard-stop margin below zero (the motor counts up when moving down).
    preflight(result, run=scan(position=ZERO + int(0.002 * 3276750)))
    with pytest.raises(RuntimeError, match="responder count"):
        preflight(result, run=scan(slaves=18))
    with pytest.raises(RuntimeError, match="saved PDO 0x1600"):
        preflight(result, run=scan(saved_mode_entry=0x60600010))
    with pytest.raises(RuntimeError, match="position range"):
        preflight(result, run=scan(position=ZERO + int(0.003 * 3276750)))
    with pytest.raises(RuntimeError, match="position range"):
        preflight(result, run=scan(position=ZERO - int(0.183 * 3276750)))
