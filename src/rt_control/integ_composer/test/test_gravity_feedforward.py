"""Opt-in current feedforward must not change default artifacts or position ownership."""

import xml.etree.ElementTree as ET

import pytest
import yaml

from integ_composer.compose import compose, preflight
from test_bench_left_arm import shares, UNIT, scan  # noqa: F401


def arguments(shares, tmp_path, **options):
    return dict(
        robot_variant="Alfa_V3",
        machine_unit=UNIT,
        physical_profile="bench_dual_arm",
        control_scope="bench_dual_arm_head",
        package_share=lambda name: str(shares[name]),
        runtime_dir=tmp_path / "runtime",
        **options,
    )


def commissioning(shares):
    path = shares["hwcfg_machines"] / "machines/Alfa_V3/policies/gravity_feedforward.yaml"
    # Synthetic limits are test input, never written to the installed machine policy.
    data = yaml.safe_load(path.read_text())
    for values in data.values():
        values.update(scale=0.25, max_effort_nm=[10.0] * 7, max_effort_rate_nm_s=[100.0] * 7)
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize("jtc_only", [False, True])
@pytest.mark.parametrize("mock", [False, True])
def test_optional_gravity_has_independent_ownership(shares, tmp_path, jtc_only, mock):
    commissioning(shares)
    common = arguments(shares, tmp_path, jtc_only=jtc_only, use_mock_hardware=mock)
    before = compose(**common)
    disabled = compose(**common, gravity_feedforward=False)
    assert before.files == disabled.files
    active = compose(**common, gravity_feedforward=True)
    companions = active.controllers["enable_manager"]["ros__parameters"]["companion_controllers"]
    assert companions == [
        "head_position_controller",
        "left_gravity_feedforward",
        "right_gravity_feedforward",
    ]
    assert ("dual_arm_rolling" in active.controller_order["inactive"]) == (not jtc_only)
    robot = ET.fromstring(active.robot_description)
    for side in ("left", "right"):
        name = side + "_gravity_feedforward"
        assert name in active.lifecycle["motion_controllers"]
        params = active.controllers[name]["ros__parameters"]
        assert len(params["joints"]) == 7 and len(params["health_interfaces"]) == 14
        assert params["health_values"] == [8.0] * 14
        assert params["feedback_age_interfaces"] == ["ethercat_domain/process_data_age_ms"]
        assert params["gravity_in_root"] is True
        assert "<robot" in params["robot_description"]
        for joint in params["joints"]:
            fragment = robot.find(f"ros2_control/joint[@name='{joint}']")
            assert {c.attrib["name"] for c in fragment.findall("command_interface")} == {
                "position",
                "control_word",
                "effort",
            }
            profile = yaml.safe_load(active.files[joint + ".yaml"])
            offset = [c for p in profile["rpdo"] for c in p["channels"] if c["index"] == 0x60B2]
            assert len(offset) == 1 and offset[0]["default"] == 0.0
            assert offset[0]["factor"] > 0 and offset[0]["type"] == "int16"
            assert offset[0]["mirror_state_interface"] == "gravity_effort_sent"
            if not mock:
                assert (
                    fragment.find("param[@name='ec_module.plugin']").text
                    == "hw_ethercat/CspTorqueOffsetSlave"
                )


def test_unqualified_limits_block_only_enabled_view(shares, tmp_path):
    path = shares["hwcfg_machines"] / "machines/Alfa_V3/policies/gravity_feedforward.yaml"
    data = yaml.safe_load(path.read_text())
    data["left"]["scale"] = "TBD"
    path.write_text(yaml.safe_dump(data))
    args = arguments(shares, tmp_path)
    compose(**args)
    with pytest.raises(ValueError, match="commissioning scale"):
        compose(**args, gravity_feedforward=True)


def test_missing_gain_and_wrong_policy_length_rejected(shares, tmp_path):
    policy = commissioning(shares)
    data = yaml.safe_load(policy.read_text())
    data["left"]["max_effort_nm"].pop()
    policy.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="per-joint max_effort_nm"):
        compose(**arguments(shares, tmp_path), gravity_feedforward=True)
    commissioning(shares)
    zero = shares["hwcfg_machines"] / f"units/{UNIT}/bench_dual_arm_zero.yaml"
    data = yaml.safe_load(zero.read_text())
    del data["spec"]["axes"][0]["gravity_feedforward"]
    zero.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="explicit positive gravity gain"):
        compose(**arguments(shares, tmp_path), gravity_feedforward=True)


def test_unsupported_scope_and_nonboolean_switch_rejected(shares, tmp_path):
    args = arguments(shares, tmp_path)
    with pytest.raises(ValueError, match="must be a boolean"):
        compose(**args, gravity_feedforward="false")
    args["control_scope"] = "bench_dual_arm_stationary_enable"
    with pytest.raises(ValueError, match="no gravity_feedforward view"):
        compose(**args, gravity_feedforward=True)


def test_current_normalization_is_checked_before_runtime(shares, tmp_path):
    commissioning(shares)
    args = arguments(shares, tmp_path, use_mock_hardware=False)
    args["control_scope"] = "bench_dual_arm"
    active = compose(**args, gravity_feedforward=True)
    expected = active.systems[0]["result"]["expected_topology"]
    currents = {c["ring_position"]: c["rated_current_ma"] for c in expected["rated_current_checks"]}
    assert len(currents) == 14 and currents[7] == currents[15] == 5400
    base_scan = scan()

    def read(command, **options):
        assert command[1] in {"master", "slaves", "upload"}
        if command[1] == "upload" and command[-2] == "0x6075":
            ring = int(command[command.index("-p") + 1])
            assert command[command.index("-t") + 1] == "uint32"
            return str(currents[ring])
        return base_scan(command, **options)

    preflight(active, run=read)
    currents[7] = 2550
    with pytest.raises(RuntimeError, match="0x6075"):
        preflight(active, run=read)
