"""Head/arm software-stop scope, unit timing and launch handoff; no physical IO."""

import pytest
import yaml
from jsonschema import ValidationError

from integ_composer.compose import compose
from test_bench_left_arm import UNIT, shares as shares
from emergency_fixture import EMERGENCY_INPUT


def arguments(shares, tmp_path, profile="bench_left_arm", scope=None, mock=False):
    return dict(
        robot_variant="Alfa_V3",
        machine_unit=UNIT,
        physical_profile=profile,
        control_scope=scope or profile,
        package_share=lambda name: str(shares[name]),
        runtime_dir=tmp_path / "runtime",
        use_mock_hardware=mock,
    )


def change_settings(shares, key, value):
    path = shares["hwcfg_machines"] / f"units/{UNIT}/deployment.yaml"
    data = yaml.safe_load(path.read_text())
    owner = data["emergency_stop"]
    if key != "receiver_timeout_ms":
        owner = owner["input"]
    if value is None:
        del owner[key]
    else:
        owner[key] = value
    path.write_text(yaml.safe_dump(data, sort_keys=False))


@pytest.mark.parametrize(
    "profile,scope",
    [
        (f"bench_{side}_arm", f"bench_{side}_arm{suffix}")
        for side in ("left", "right", "dual")
        for suffix in ("", "_stationary_enable")
    ]
    + [("head_only", "head_only"), ("bench_dual_arm", "bench_dual_arm_head")],
)
def test_unit_timing_and_publisher_reach_only_head_and_arm_scopes(shares, tmp_path, profile, scope):
    result = compose(**arguments(shares, tmp_path, profile, scope))
    receiver = result.controllers["enable_manager"]["ros__parameters"]
    assert receiver["emergency_stop_response"] is True
    assert receiver["emergency_stop_timeout_ms"] == EMERGENCY_INPUT["receiver_timeout_ms"]
    [node] = result.lifecycle["input_peripherals"]
    assert (node["package"], node["executable"]) == ("periph_emergency_stop", "emergency_stop_node")
    config = yaml.safe_load(result.files[node["config_file"]])
    assert config[node["name"]]["ros__parameters"] == EMERGENCY_INPUT["input"]
    mock = compose(**arguments(shares, tmp_path, profile, scope, mock=True))
    assert "input_peripherals" not in mock.lifecycle
    assert node["config_file"] not in mock.files


@pytest.mark.parametrize(
    "key", ["receiver_timeout_ms", "poll_period_ms", "request_timeout_ms", "stale_timeout_ms"]
)
@pytest.mark.parametrize("value", ["TBD", None, 0, -1, True])
@pytest.mark.parametrize("profile", ["bench_left_arm", "head_only"])
def test_missing_or_invalid_unit_timing_blocks_real_artifact(shares, tmp_path, key, value, profile):
    change_settings(shares, key, value)
    with pytest.raises((ValueError, ValidationError)):
        compose(**arguments(shares, tmp_path, profile))


def test_draft_mock_does_not_guess_timeout_or_connect_hardware(shares, tmp_path):
    change_settings(shares, "receiver_timeout_ms", "TBD")
    result = compose(**arguments(shares, tmp_path, mock=True))
    assert result.controllers["enable_manager"]["ros__parameters"]["emergency_stop_response"]
    assert (
        "emergency_stop_timeout_ms" not in result.controllers["enable_manager"]["ros__parameters"]
    )
    assert "input_peripherals" not in result.lifecycle


@pytest.mark.parametrize("profile", ["chassis_only", "bench_suspension", "force_bench"])
def test_other_scopes_do_not_consume_or_launch_soft_stop(shares, tmp_path, profile):
    unit = "Alfa_V3-Advantech-01" if profile == "chassis_only" else UNIT
    path = shares["hwcfg_machines"] / f"units/{unit}/deployment.yaml"
    data = yaml.safe_load(path.read_text())
    del data["emergency_stop"]
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    selection = arguments(shares, tmp_path, profile, mock=True)
    selection["machine_unit"] = unit
    result = compose(**selection)
    for controller in result.controllers.values():
        assert not controller["ros__parameters"].get("emergency_stop_response", False)
    assert "input_peripherals" not in result.lifecycle
    assert "peripherals/emergency_stop.yaml" not in result.files
