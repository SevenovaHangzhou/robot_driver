"""Default launch entries read a verified artifact, then preflight, before any Node."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

BRINGUP = Path(__file__).resolve().parents[1]
SCOPES = {
    "arm_runtime": ("arms_only", "arms_only"),
    "enable_only": ("arms_only", "arms_stationary_enable"),
    "force_sensors": ("force_bench", "force_bench"),
}


def load(entry):
    path = BRINGUP / "launch" / f"rt_control_{entry}.launch.py"
    spec = importlib.util.spec_from_file_location("runtime_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("entry", sorted(SCOPES))
def test_entry_reads_artifact_and_preflight_precedes_nodes(monkeypatch, entry):
    module = load(entry)
    actions = module.generate_launch_description().entities
    declarations = {item.name for item in actions if isinstance(item, DeclareLaunchArgument)}
    assert "generated_dir" in declarations
    assert not {"use_composer", "calibration_file", "head_can_config"}.intersection(declarations)
    assert not hasattr(module, "compose"), "Launch must not compose or write configuration"
    assert not hasattr(module, "tempfile")
    calls = []

    def load_artifact(directory, **selection):
        calls.append("load_artifact")
        assert directory == "/synthetic/artifact"
        assert (selection["physical_profile"], selection["control_scope"]) == SCOPES[entry]
        assert selection["use_mock_hardware"] is False
        return SimpleNamespace(
            controllers={},
            robot_description="<robot name='synthetic'/>",
            controller_order={"active": [], "inactive": []},
            runtime_dir=Path(directory),
            runtime_parameters={},
        )

    def preflight(composed):
        calls.append("preflight")
        raise ValueError("synthetic preflight rejection")

    def node(**kwargs):
        pytest.fail("Node constructed before rejected preflight")

    monkeypatch.setattr(module, "load_artifact", load_artifact)
    monkeypatch.setattr(module, "preflight", preflight)
    monkeypatch.setattr(module, "Node", node)
    context = LaunchContext()
    context.launch_configurations.update(
        use_mock_hardware="false",
        jtc_only="false",
        publish_robot_state="false",
        sync_mode="dc",
        generated_dir="/synthetic/artifact",
    )
    with pytest.raises(ValueError, match="synthetic preflight"):
        module.setup(context)
    assert calls == ["load_artifact", "preflight"]


def test_rejected_artifact_stops_before_preflight(monkeypatch):
    module = load("arm_runtime")

    def load_artifact(directory, **selection):
        raise ValueError("Artifact is stale against installed configuration")

    monkeypatch.setattr(module, "load_artifact", load_artifact)
    monkeypatch.setattr(module, "preflight", lambda composed: pytest.fail("preflight ran"))
    monkeypatch.setattr(module, "Node", lambda **kwargs: pytest.fail("Node constructed"))
    context = LaunchContext()
    context.launch_configurations.update(
        use_mock_hardware="true", jtc_only="false", generated_dir="/synthetic/artifact"
    )
    with pytest.raises(ValueError, match="stale"):
        module.setup(context)
