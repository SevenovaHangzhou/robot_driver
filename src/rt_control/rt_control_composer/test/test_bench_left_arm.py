"""Single-arm bench scope: data-only selection on the shared owners; no hardware is accessed."""

from pathlib import Path
import shutil
import sys

import pytest
import yaml

from rt_control_composer.compose import compose, preflight
from rt_control_composer.schema.primitives import load_yaml

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
sys.path.insert(0, str(RT / "robot_hw_ethercat"))
from robot_hw_ethercat import rt_control_renderer as OWNER

UNIT = "Alfa_V3-bench-01"
ZERO = f"units/{UNIT}/bench_left_arm_zero.yaml"
JOINTS = [f"left_j{index}" for index in range(1, 8)]
SELECTION = dict(
    robot_variant="Alfa_V3",
    machine_unit=UNIT,
    physical_profile="bench_left_arm",
    control_scope="bench_left_arm_stationary_enable",
)


@pytest.fixture
def shares(tmp_path, monkeypatch):
    data = tmp_path / "machine_config"
    shutil.copytree(RT / "rt_control_machine_config", data)
    monkeypatch.setattr(OWNER, "_share", lambda: RT / "robot_hw_ethercat")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["rt_control_machine_config"] = data
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def arguments(shares, *, confirmed=True):
    if not confirmed:
        # Test-only regression: an unconfirmed capture must keep the owner gate closed.
        path = shares["rt_control_machine_config"] / ZERO
        zero = load_yaml(path)
        zero["spec"]["confirmed_by"] = "pending"
        path.write_text(yaml.safe_dump(zero, sort_keys=False))
    return dict(**SELECTION, package_share=lambda name: str(shares[name]))


def scan(topology_ports=((2, "EBUS", 8), (3, "MII", 1)), slaves=9):
    """Text in the IgH CLI format recorded by the 2026-10-17 bench scan."""
    identities = {0: (0x00100000, 0x10F40931), 8: (0x00100000, 0x10F40932)}
    identities.update({ring: (0x5A65726F, 0x00029252) for ring in range(1, 8)})

    def run(command, **_):
        assert command[0] == "ethercat" and command[1] in {"master", "slaves", "upload"}
        if command[1] == "master":
            return f"Master0\n  Phase: Idle\n  Active: no\n  Slaves: {slaves}\n"
        if command[1] == "slaves":
            blocks = []
            for ring, (vendor, product) in sorted(identities.items()):
                revision = 0x00010000 if ring in (0, 8) else 0x00000001
                block = (
                    f"=== Master 0, Slave {ring} ===\nState: PREOP\n"
                    f"Vendor Id:       0x{vendor:08x}\nProduct code:    0x{product:08x}\n"
                    f"Revision number: 0x{revision:08x}\n"
                )
                if ring == 0:
                    block += "".join(
                        f"   {port}  {kind}  up    open    yes             {downstream}\n"
                        for port, kind, downstream in topology_ports
                    )
                blocks.append(block)
            return "".join(blocks)
        return "0x0000 0\n"

    return run


def test_pending_zero_capture_rejects_real_and_mock(shares, tmp_path):
    for mock in (False, True):
        with pytest.raises(ValueError, match="confirmed output-encoder zero capture"):
            compose(
                **arguments(shares, confirmed=False),
                runtime_dir=tmp_path / str(mock),
                use_mock_hardware=mock,
            )
    assert not list(tmp_path.glob("True")) and not list(tmp_path.glob("False"))


def test_mock_scope_is_enable_only_with_staged_hardware(shares, tmp_path):
    result = compose(**arguments(shares, confirmed=True), runtime_dir=tmp_path / "mock")
    managed = result.systems[0]["result"]["managed_joints"]
    assert managed == JOINTS
    assert result.lifecycle["hardware_activation_order"] == ["ecat_arms"]
    assert result.lifecycle["motion_controllers"] == []
    assert result.lifecycle["bus_locks"] == ["ethercat-master-0"]
    assert result.staged_hardware_activation and "lifecycle.yaml" in result.files
    manager = result.controllers["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 1000
    assert manager["hardware_components_initial_state"] == {"unconfigured": ["ecat_arms"]}
    enable = result.controllers["enable_manager"]["ros__parameters"]
    assert enable["managed_joints"] == JOINTS
    assert enable["enable_batch_joint_names"] == JOINTS
    assert enable["enable_batch_sizes"] == [1] * 7
    assert enable["enable_only"] is True and enable["jtc_name"] == ""
    assert "expected_modes" not in enable


def test_real_topology_matches_the_bench_scan(shares, tmp_path):
    result = compose(
        **arguments(shares, confirmed=True),
        runtime_dir=tmp_path / "real",
        use_mock_hardware=False,
    )
    topology = result.systems[0]["result"]["expected_topology"]
    assert [item["ring_position"] for item in topology["responders"]] == list(range(9))
    assert topology["branches"] == [{"main_esc_port": 3, "downstream": 1}]
    assert topology["internal_ports"] == [{"main_esc_port": 2, "downstream": 8}]
    assert topology["preflight_required"] is True
    preflight(result, run=scan())
    with pytest.raises(RuntimeError, match="responder count"):
        preflight(result, run=scan(slaves=18))
    with pytest.raises(RuntimeError, match="Branch wiring"):
        preflight(result, run=scan(topology_ports=((2, "EBUS", 8), (1, "MII", 1))))


def test_bench_calibration_stays_in_the_bench_unit():
    root = RT / "rt_control_machine_config"
    assert not (root / "units/Alfa_V3-01/bench_left_arm_zero.yaml").exists()
    zero = load_yaml(root / ZERO)
    assert zero["spec"]["confirmed_by"] == "user"
    assert [axis["ring_position"] for axis in zero["spec"]["axes"]] == list(range(1, 8))


MODEL_JOINTS = [f"left_joint{index}" for index in range(1, 8)]


def test_motion_scope_exposes_the_left_arm_trajectory_capability(shares, tmp_path):
    selection = {**arguments(shares, confirmed=True), "control_scope": "bench_left_arm"}
    result = compose(**selection, runtime_dir=tmp_path / "mock")
    assert result.systems[0]["result"]["managed_joints"] == MODEL_JOINTS
    assert result.lifecycle["motion_controllers"] == ["left_arm_jtc"]
    jtc = result.controllers["left_arm_jtc"]["ros__parameters"]
    assert jtc["joints"] == MODEL_JOINTS and jtc["command_interfaces"] == ["position"]
    assert jtc["trajectory_start_consistency_check"]["enabled"] is True
    enable = result.controllers["enable_manager"]["ros__parameters"]
    assert enable["enable_only"] is False and enable["jtc_name"] == "left_arm_jtc"
    assert enable["motion_mode_switching"] is False
    assert {
        "name": "arm_joint_trajectory",
        "topic": "/left_arm_jtc/follow_joint_trajectory",
        "type": "control_msgs/action/FollowJointTrajectory",
        "controller": "left_arm_jtc",
    } in result.lifecycle["capabilities"]


def test_motion_scope_real_mode_stays_closed_until_directions_are_verified(shares, tmp_path):
    selection = {**arguments(shares, confirmed=True), "control_scope": "bench_left_arm"}
    with pytest.raises(ValueError, match="calibration is not verified"):
        compose(**selection, runtime_dir=tmp_path / "real", use_mock_hardware=False)


def test_bench_directions_follow_the_alfav3_model():
    module = load_yaml(RT / "rt_control_machine_config/machines/Alfa_V3/bench_left_arm.yaml")
    directions = [axis["direction"] for axis in module["spec"]["branches"]["left"]["axes"]]
    assert directions == [1, -1, -1, 1, 1, 1, 1]
