import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from launch.actions import EmitEvent
from launch.events import Shutdown
from launch.utilities import perform_substitutions
from launch import LaunchContext
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "rt_control.launch.py"
CONTROLLERS_PATH = PACKAGE_ROOT / "config" / "controllers.yaml"
PACKAGE_XML_PATH = PACKAGE_ROOT / "package.xml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("rt_control_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _spawner_arguments(spawner):
    command = spawner.process_description.cmd
    return [perform_substitutions(LaunchContext(), token) for token in command][1:-1]


def test_rolling_plugin_is_registered_and_declared_as_runtime_dependency():
    controllers = yaml.safe_load(CONTROLLERS_PATH.read_text(encoding="utf-8"))
    manager_parameters = controllers["controller_manager"]["ros__parameters"]
    assert manager_parameters["rolling_trajectory_controller"]["type"] == (
        "rolling_trajectory_controller/RollingTrajectoryController"
    )

    package_root = ET.parse(PACKAGE_XML_PATH).getroot()
    runtime_dependencies = {element.text for element in package_root.findall("exec_depend")}
    assert "rolling_trajectory_controller" in runtime_dependencies


def test_mandatory_inactive_spawner_and_fail_fast_transition():
    launch_module = _load_launch_module()
    next_action = object()
    spawner, transition = launch_module.make_mandatory_inactive_spawner(
        "rolling_trajectory_controller",
        next_action,
        parameter_files=["/tmp/controllers.yaml", "/tmp/rolling_override.yaml"],
    )

    assert _spawner_arguments(spawner) == [
        "rolling_trajectory_controller",
        "--inactive",
        "--controller-manager",
        "/controller_manager",
        "--param-file",
        "/tmp/controllers.yaml",
        "--param-file",
        "/tmp/rolling_override.yaml",
    ]
    assert transition(SimpleNamespace(returncode=0), None) == [next_action]

    failure_actions = transition(SimpleNamespace(returncode=1), None)
    shutdown_actions = [action for action in failure_actions if isinstance(action, EmitEvent)]
    assert len(shutdown_actions) == 1
    assert isinstance(shutdown_actions[0].event, Shutdown)
    assert "rolling_trajectory_controller" in shutdown_actions[0].event.reason


def test_generated_override_uses_existing_absolute_envelope_path(tmp_path):
    launch_module = _load_launch_module()
    envelope_path = PACKAGE_ROOT / "config" / "rolling_envelope_provisional.yaml"
    override_path = launch_module.write_rolling_parameter_override(
        envelope_path, directory=tmp_path
    )
    try:
        assert override_path.is_absolute()
        override = yaml.safe_load(override_path.read_text(encoding="utf-8"))
        configured_path = override["rolling_trajectory_controller"]["ros__parameters"][
            "envelope_file"
        ]
        assert Path(configured_path).is_absolute()
        assert Path(configured_path).samefile(envelope_path)
    finally:
        override_path.unlink(missing_ok=True)


def test_launch_source_chains_jtc_then_rolling_then_enable_manager():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    rolling_definition = source.index(
        'make_mandatory_inactive_spawner(\n        "rolling_trajectory_controller",\n'
        "        enable_spawner,"
    )
    jtc_definition = source.index(
        'make_mandatory_inactive_spawner(\n        "whole_body_jtc", rolling_spawner'
    )
    assert rolling_definition < jtc_definition
    assert "jtc_transition_handler" in source
    assert "rolling_transition_handler" in source
