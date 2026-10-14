"""Default launch entries must run composition/preflight before constructing Nodes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

BRINGUP = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("entry", ["arm_runtime", "enable_only", "head_runtime", "force_sensors"])
def test_default_entry_uses_only_composer_and_preflight_precedes_nodes(monkeypatch, entry):
    path = BRINGUP / "launch" / f"rt_control_{entry}.launch.py"
    spec = importlib.util.spec_from_file_location("runtime_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description().entities
    declarations = {item.name for item in actions if isinstance(item, DeclareLaunchArgument)}
    assert not {"use_composer", "calibration_file", "head_can_config"}.intersection(declarations)
    calls = []

    def compose(**kwargs):
        calls.append("compose")
        return SimpleNamespace(
            controllers={},
            robot_description="<robot name='synthetic'/>",
            controller_order={"active": [], "inactive": []},
        )

    def preflight(composed):
        calls.append("preflight")
        raise ValueError("synthetic preflight rejection")

    def node(**kwargs):
        pytest.fail("Node constructed before rejected preflight")

    monkeypatch.setattr(module, "compose", compose)
    monkeypatch.setattr(module, "preflight", preflight)
    monkeypatch.setattr(module, "Node", node)
    context = LaunchContext()
    context.launch_configurations.update(
        use_mock_hardware="false", jtc_only="false", publish_robot_state="false", sync_mode="dc"
    )
    with pytest.raises(ValueError, match="synthetic preflight"):
        module.setup(context)
    assert calls == ["compose", "preflight"]
