"""The snapshot must precede process creation, not run beside the controller."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node
import pytest


ROOT = Path(__file__).resolve().parents[4]
LAUNCH = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control.launch.py"


def module():
    spec = importlib.util.spec_from_file_location("preop_launch_test", LAUNCH)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def _setup_launch(monkeypatch):
    loaded = module()
    events = []
    monkeypatch.setattr(loaded, "get_package_share_directory", lambda name: str(ROOT / "src/rt_control" / name))

    class RecordingNode(Node):
        def __init__(self, **kwargs):
            events.append(("node", kwargs))
            super().__init__(**kwargs)

    monkeypatch.setattr(loaded, "Node", RecordingNode)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_mock_hardware": "false", "use_sim_time": "false", "ethercat_variant": "alfa_v1",
        "canopen_variant": "alfa_v1", "start_plc": "false", "start_bms": "false",
        "start_x503_force_sensor": "true", "start_x503_sdo_snapshot": "false",
    })
    return loaded, context, events


def test_preop_snapshot_finishes_before_any_node_is_created(monkeypatch):
    loaded, context, events = _setup_launch(monkeypatch)

    def snapshot(*args, **kwargs):
        events.append(("snapshot", kwargs))
        return {"schema_version": 1, "startup_id": kwargs["startup_id"], "source": "preop_sdo", "sensors": []}

    monkeypatch.setattr(loaded, "read_preop_snapshot", snapshot, raising=False)
    loaded._launch_setup(context)
    assert events[0][0] == "snapshot"
    nodes = [value for kind, value in events if kind == "node"]
    assert not any(n["executable"] == "x503_sdo_snapshot" for n in nodes)
    bridge = next(n for n in nodes if n["executable"] == "x503_wrench_bridge")
    assert "preop_snapshot_json" in bridge["parameters"][0]


def test_failed_preop_guard_creates_no_control_process(monkeypatch):
    from x503_force_sensor.preop import PreopRequired
    loaded, context, events = _setup_launch(monkeypatch)

    def reject(*args, **kwargs):
        raise PreopRequired("master already OP")

    monkeypatch.setattr(loaded, "read_preop_snapshot", reject, raising=False)
    with pytest.raises(PreopRequired):
        loaded._launch_setup(context)
    assert events == []


def test_legacy_runtime_sdo_flag_is_rejected(monkeypatch):
    loaded, context, events = _setup_launch(monkeypatch)
    context.launch_configurations["start_x503_sdo_snapshot"] = "true"
    with pytest.raises(ValueError, match="PREOP"):
        loaded._launch_setup(context)
    assert events == []
