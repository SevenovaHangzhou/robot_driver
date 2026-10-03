import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


LAUNCH = Path(__file__).resolve().parents[1] / "launch/rt_control_interface_runtime.launch.py"


def _module():
    spec = importlib.util.spec_from_file_location("rt_control_interface_runtime", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_interface_runtime_is_validation_only_and_mock_by_default() -> None:
    description = _module().generate_launch_description()
    defaults = {
        action.name: action.default_value[0].text
        for action in description.entities
        if hasattr(action, "name") and hasattr(action, "default_value")
    }

    assert defaults["validation_only"] == "true"
    assert defaults["use_mock_hardware"] == "true"
    assert defaults["start_ultrasonic"] == "false"


def test_interface_runtime_lists_only_preview_adapters() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    for executable in (
        "vacuum_adapter",
        "module_state_adapter",
        "position_action_adapter",
        "rt_status_adapter",
    ):
        assert f'executable="{executable}"' in source
    for executable in ("plc_io_modbus", "bms_node", "led_strip_node", "ultrasonic_node"):
        assert f'executable="{executable}"' not in source


@pytest.mark.parametrize("use_mock,ultrasonic", [("false", "false"), ("true", "true")])
def test_draft_rejects_hardware_before_resolving_packages(monkeypatch, use_mock, ultrasonic):
    module = _module()
    context = LaunchContext()
    context.launch_configurations.update(
        validation_only="false", use_mock_hardware=use_mock, start_ultrasonic=ultrasonic
    )

    def unexpected_package_lookup(_):
        pytest.fail("hardware gate must run before constructing nodes")

    monkeypatch.setattr(module, "get_package_share_directory", unexpected_package_lookup)
    with pytest.raises(ValueError, match="Draft"):
        module._setup(context)
