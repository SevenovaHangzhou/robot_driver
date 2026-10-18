"""Optional Runtime-LOG recorder: deployment-gated, outside controller_order, all state interfaces."""

from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from rt_control_composer.compose import RECORDER_NAME, RECORDER_PLUGIN, compose
from rt_control_composer.schema.primitives import load_yaml

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
sys.path.insert(0, str(RT / "robot_hw_ethercat"))
from robot_hw_ethercat import rt_control_renderer as OWNER  # noqa: E402

UNIT = "Alfa_V3-bench-01"
DEPLOYMENT = f"units/{UNIT}/deployment.yaml"
SELECTION = dict(
    robot_variant="Alfa_V3",
    machine_unit=UNIT,
    physical_profile="bench_left_arm",
    control_scope="bench_left_arm",
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


def set_recorder(shares, value):
    path = shares["rt_control_machine_config"] / DEPLOYMENT
    deployment = load_yaml(path)
    if value is None:
        deployment.pop("runtime_log_recorder", None)
    else:
        deployment["runtime_log_recorder"] = value
    path.write_text(yaml.safe_dump(deployment, sort_keys=False))


def build(shares, tmp_path, mock=True):
    return compose(
        **SELECTION,
        runtime_dir=tmp_path / "runtime",
        use_mock_hardware=mock,
        package_share=lambda name: str(shares[name]),
    )


def state_interfaces(description):
    robot = ET.fromstring(description)
    return [
        f"{element.attrib['name']}/{state.attrib['name']}"
        for system in robot.findall("ros2_control")
        for element in system
        if element.tag in ("joint", "sensor", "gpio")
        for state in element.findall("state_interface")
    ]


def test_disabled_or_absent_recorder_changes_nothing(shares, tmp_path):
    set_recorder(shares, None)
    absent = build(shares, tmp_path / "a")
    set_recorder(shares, {"enabled": False})
    disabled = build(shares, tmp_path / "b")
    assert absent.files == disabled.files
    assert RECORDER_NAME not in absent.files["controllers.yaml"]
    assert "runtime_log_trace" not in absent.files["controllers.yaml"]
    assert "recorder_controllers" not in absent.lifecycle


@pytest.mark.parametrize("mock", [True, False])
def test_enabled_recorder_records_every_state_interface_outside_controller_order(
    shares, tmp_path, mock
):
    set_recorder(shares, None)
    # Same runtime_dir for both: real mode embeds it in the description; compose writes nothing.
    baseline = build(shares, tmp_path, mock)
    set_recorder(shares, {"enabled": True, "slots": 3000})
    result = build(shares, tmp_path, mock)
    controllers = yaml.safe_load(result.files["controllers.yaml"])
    assert controllers["controller_manager"]["ros__parameters"][RECORDER_NAME] == {
        "type": RECORDER_PLUGIN
    }
    parameters = controllers[RECORDER_NAME]["ros__parameters"]
    assert parameters["slots"] == 3000
    assert parameters["interfaces"] == state_interfaces(result.robot_description)
    assert "left_joint3/status_word" in parameters["interfaces"]
    if not mock:
        assert "ethercat_master/wc_error_count" in parameters["interfaces"]
        assert "ethercat_slave_7/al_state" in parameters["interfaces"]
    assert result.lifecycle["recorder_controllers"] == [RECORDER_NAME]
    # Everything else is identical to the scope without the recorder.
    assert result.controller_order == baseline.controller_order
    assert result.files["controller_order.yaml"] == baseline.files["controller_order.yaml"]
    assert result.files["robot_description.xml"] == baseline.files["robot_description.xml"]
    assert controllers["enable_manager"]["ros__parameters"].pop("runtime_log_trace") is True
    del controllers[RECORDER_NAME]
    del controllers["controller_manager"]["ros__parameters"][RECORDER_NAME]
    assert controllers == yaml.safe_load(baseline.files["controllers.yaml"])
    lifecycle = dict(result.lifecycle)
    lifecycle.pop("recorder_controllers")
    assert lifecycle == baseline.lifecycle


def test_recorder_settings_are_schema_checked(shares, tmp_path):
    set_recorder(shares, {"enabled": True, "slots": 10})
    with pytest.raises(Exception):
        build(shares, tmp_path)
