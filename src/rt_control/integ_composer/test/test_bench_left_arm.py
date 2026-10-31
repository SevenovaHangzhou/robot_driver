"""Single-arm bench scope: data-only selection on the shared owners; no hardware is accessed."""

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
    shutil.copytree(RT / "hwcfg_machines", data)
    monkeypatch.setattr(OWNER, "_share", lambda: RT / "hw_ethercat")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["hwcfg_machines"] = data
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def arguments(shares, *, confirmed=True):
    if not confirmed:
        # Test-only regression: an unconfirmed capture must keep the owner gate closed.
        path = shares["hwcfg_machines"] / ZERO
        zero = load_yaml(path)
        zero["spec"]["confirmed_by"] = "pending"
        path.write_text(yaml.safe_dump(zero, sort_keys=False))
    return dict(**SELECTION, package_share=lambda name: str(shares[name]))


def scan(topology_ports=((1, "MII", 9), (2, "EBUS", 17), (3, "MII", 1)), slaves=18, errors=None, faulted=()):
    """Connected dual arms plus two passive BluePoint tails, cached SII scan 2026-10-24."""
    identities = {0: (0x00100000, 0x10F40931), 17: (0x00100000, 0x10F40932),
                  8: (0xA1, 0x8081), 16: (0xA1, 0x8081)}
    identities.update({ring: (0x5A65726F, 0x00029252) for ring in [*range(1, 8), *range(9, 16)]})

    def run(command, **_):
        assert command[0] == "ethercat" and command[1] in {"master", "slaves", "upload"}
        if command[1] == "master":
            return f"Master0\n  Phase: Idle\n  Active: no\n  Slaves: {slaves}\n"
        if command[1] == "slaves":
            blocks = []
            for ring, (vendor, product) in sorted(identities.items()):
                revision = 0x00010000 if ring in (0, 17) else 0x00000001
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
        index, ring = int(command[-2], 16), int(command[command.index("-p") + 1])
        value = (errors or {}).get(ring, 0) if index == 0x603F else 0
        if index == 0x6041:
            value = 0x0208 if ring in faulted else 0x0250
        return f"0x{value:04x} {value}\n"

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
    assert [item["ring_position"] for item in topology["responders"]] == list(range(18))
    assert topology["branches"] == [{"main_esc_port": 3, "downstream": 1}]
    assert topology["internal_ports"] == [{"main_esc_port": 2, "downstream": 17}]
    assert topology["preflight_required"] is True
    preflight(result, run=scan())
    with pytest.raises(RuntimeError, match="responder count"):
        preflight(result, run=scan(slaves=17))
    with pytest.raises(RuntimeError, match="Branch wiring"):
        preflight(result, run=scan(topology_ports=((2, "EBUS", 8), (1, "MII", 1))))


def test_bench_calibration_stays_in_the_bench_unit():
    root = RT / "hwcfg_machines"
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


def test_motion_scope_real_mode_uses_confirmed_directions_and_stop_parameters(shares, tmp_path):
    selection = {**arguments(shares, confirmed=True), "control_scope": "bench_left_arm"}
    result = compose(**selection, runtime_dir=tmp_path / "real", use_mock_hardware=False)
    signs, sdos = [], []
    for joint in MODEL_JOINTS:
        profile = yaml.safe_load(result.files[f"{joint}.yaml"])
        signs.append(1 if profile["tpdo"][0]["channels"][0]["factor"] > 0 else -1)
        sdos.append({entry["index"]: entry["value"] for entry in profile["sdo"]})
    assert signs == [1] * 7
    # CSP, fault reaction = quick-stop ramp then disable, quick-stop deceleration 40 deg/s^2.
    assert sdos == [{0x6060: 8, 0x605E: 2, 0x6085: 58254}] * 7


@pytest.mark.parametrize("mock", [False, True])
def test_motion_scope_maps_diagnostic_feedback(shares, tmp_path, mock):
    import math
    import xml.etree.ElementTree as ET

    selection = {**arguments(shares, confirmed=True), "control_scope": "bench_left_arm"}
    result = compose(**selection, runtime_dir=tmp_path / "diag", use_mock_hardware=mock)
    robot = ET.fromstring(result.robot_description)
    for joint in MODEL_JOINTS:
        profile = yaml.safe_load(result.files[f"{joint}.yaml"])
        channels = profile["tpdo"][0]["channels"]
        sensor = joint in {f"left_joint{i}" for i in range(3, 7)}
        encoder = joint == "left_joint6"
        assert [(c["index"], c["type"], c["state_interface"]) for c in channels] == [
            (0x6064, "int32", "position"),
            (0x60FD, "uint32", "digital_inputs"),
            (0x6041, "uint16", "status_word"),
            (0x606C, "int32", "velocity"),
            (0x6077, "int16", "torque_permille"),
            (0x603F, "uint16", "error_code"),
        ] + ([(0x3B69, "int32", "sensor_torque_mnm")] if sensor else []) + ([(0x2241, "int32", "dual_encoder_difference_counts")] if encoder else [])
        assert all("scale" not in channel for channel in channels)
        # 18/22/26 bytes of TxPDO, inside the 76-byte 0x1A00 limit.
        sizes = {"int32": 4, "uint32": 4, "int16": 2, "uint16": 2}
        assert sum(sizes[c["type"]] for c in channels) == (26 if encoder else 22 if sensor else 18)
        if sensor:
            assert "factor" not in channels[-1] and "offset" not in channels[-1]
        position, velocity, torque = channels[0], channels[3], channels[4]
        assert math.isclose(velocity["factor"], position["factor"]) and "offset" not in velocity
        assert torque["factor"] == 1.0 and "factor" not in channels[5]
        element = robot.find(f"ros2_control/joint[@name='{joint}']")
        states = [state.attrib["name"] for state in element.findall("state_interface")]
        assert states == [
            "position",
            "digital_inputs",
            "status_word",
            "target_position",
            "control_word_sent",
            "velocity",
            "torque_permille",
            "error_code",
        ] + (["sensor_torque_mnm"] if sensor else []) + (["dual_encoder_difference_counts"] if encoder else [])
        recorder = result.controllers["runtime_log_recorder"]["ros__parameters"]["interfaces"]
        assert (f"{joint}/sensor_torque_mnm" in recorder) is sensor
        assert (f"{joint}/dual_encoder_difference_counts" in recorder) is encoder
        mirrors = [c.get("mirror_state_interface") for c in profile["rpdo"][0]["channels"]]
        assert mirrors == ["target_position", "control_word_sent"]


def test_bench_directions_follow_the_alfav3_model():
    module = load_yaml(RT / "hwcfg_machines/machines/Alfa_V3/bench_left_arm.yaml")
    directions = [axis["direction"] for axis in module["spec"]["branches"]["left"]["axes"]]
    # Joint positive = encoder count increasing on every axis; the model axis carries any flip.
    assert directions == [1] * 7


def test_startup_accepts_the_master_left_op_code_and_only_j3_battery_code(shares, tmp_path):
    result = compose(
        **arguments(shares, confirmed=True),
        runtime_dir=tmp_path / "real",
        use_mock_hardware=False,
    )
    axes = result.systems[0]["result"]["expected_topology"]["axes"]
    assert [axis["allowed_errors"] for axis in axes] == [
        [0x730F, 0xA000] if axis["ring_position"] == 3 else [0xA000] for axis in axes
    ]
    declared = preflight(result, run=scan(errors={1: 0xA000, 3: 0x730F, 7: 0xA000}))
    assert [(e["dree"], e["severity"], e["blocking"]) for e in declared] == [
        ("RT_DEVICE_FAULT_RESETTABLE", "WARN", False),
        ("RT_DEVICE_FAULT", "WARN", False),
        ("RT_DEVICE_FAULT_RESETTABLE", "WARN", False),
    ]
    with pytest.raises(RuntimeError, match="Unqualified error 0x730f") as failure:
        preflight(result, run=scan(errors={2: 0x730F}))
    [entry] = failure.value.entries
    # Located through diagnostics.yaml and explained by the ZeroErr fault table.
    assert entry["source"].endswith("/drive") and entry["dree"] == "RT_DEVICE_FAULT"
    assert "0x730F 电池电压低" in entry["detail"] and entry["severity"] == "FAULT"
    with pytest.raises(RuntimeError, match="Unqualified error 0x8500"):
        preflight(result, run=scan(errors={5: 0x8500}))


def test_faulted_axis_passes_only_with_a_declared_startup_code(shares, tmp_path):
    result = compose(
        **arguments(shares, confirmed=True),
        runtime_dir=tmp_path / "real",
        use_mock_hardware=False,
    )
    # Recorded 2026-10-17: after an orderly stop J1 reads 0x6041=0x0208, 0x603F=0xA000.
    preflight(result, run=scan(errors={1: 0xA000}, faulted={1}))
    with pytest.raises(RuntimeError, match="faulted with unqualified error 0x0000"):
        preflight(result, run=scan(faulted={4}))
    with pytest.raises(RuntimeError, match="faulted with unqualified error 0x8500"):
        preflight(result, run=scan(errors={4: 0x8500}, faulted={4}))


def test_real_artifact_carries_the_bench_realtime_cpu_layout(shares, tmp_path):
    # Without `host` the scope runtime skipped the pre-activation pinning of the update thread
    # (2026-10-22 dual-arm handoff burst); the bench now declares its isolated CPU and housekeeping set.
    selection = {**arguments(shares, confirmed=True), "control_scope": "bench_left_arm"}
    result = compose(**selection, runtime_dir=tmp_path / "real", use_mock_hardware=False)
    host = result.lifecycle["host"]
    assert host["rt_cpu"] == 14 and host["hostname"] == "ar-Default-string"
    assert 14 not in host["housekeeping_cpus"]
    assert sorted(host["housekeeping_cpus"]) == [0, 2, 4, 6, 8, 10, 12, *range(16, 28)]
