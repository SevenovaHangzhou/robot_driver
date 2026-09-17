"""The snapshot must precede process creation, not run beside the controller."""
import importlib.util
from pathlib import Path
import stat

from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[4]
LAUNCH = ROOT / "src/rt_control/rt_control_bringup/launch/rt_control.launch.py"


def module():
    spec = importlib.util.spec_from_file_location("preop_launch_test", LAUNCH)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


@pytest.mark.parametrize("mock, expected", [("true", "False"), ("false", "True")])
def test_led_default_does_not_access_gateway_in_mock(mock, expected):
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    loaded = module()
    context = LaunchContext()
    context.launch_configurations["use_mock_hardware"] = mock
    argument = next(
        entity for entity in loaded.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument) and entity.name == "start_led"
    )
    assert perform_substitutions(context, argument.default_value) == expected


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


def _valid_snapshot(startup_id, specs):
    values = {
        "snapshot_valid": "true",
        "engineering_unit_contract": "force_N_torque_Nm",
        "validity_policy": "sample_codes_in_range",
        "sample_code_min": "-999999",
        "sample_code_max": "999999",
        **{f"decimal_{index}": str(1 if index <= 3 else 3) for index in range(1, 7)},
        **{f"unit_{index}": str(5 if index <= 3 else 7) for index in range(1, 7)},
    }
    return {
        "schema_version": 1,
        "startup_id": startup_id,
        "source": "preop_sdo",
        "sensors": [
            {
                "sensor_name": spec.sensor_name,
                "slave_position": spec.slave_position,
                "values": {**values, "slave_position": str(spec.slave_position)},
                "error": "",
            }
            for spec in specs
        ],
    }


def test_preop_snapshot_finishes_before_any_node_and_spawns_cpp_controllers(monkeypatch):
    loaded, context, events = _setup_launch(monkeypatch)
    documents = []

    def snapshot(specs, *args, **kwargs):
        events.append(("snapshot", kwargs))
        return _valid_snapshot(kwargs["startup_id"], specs)

    monkeypatch.setattr(loaded, "read_preop_snapshot", snapshot, raising=False)
    monkeypatch.setattr(
        loaded,
        "_write_controller_parameter_file",
        lambda document: documents.append(document) or Path("/tmp/x503-controllers.yaml"),
        raising=False,
    )
    loaded._launch_setup(context)
    assert events[0][0] == "snapshot"
    nodes = [value for kind, value in events if kind == "node"]
    assert not any(n["executable"] == "x503_sdo_snapshot" for n in nodes)
    assert not any(n["executable"] == "x503_wrench_bridge" for n in nodes)
    spawners = [
        node for node in nodes
        if node["executable"] == "spawner"
        and node["arguments"][0] in {
            "right_force_sensor_broadcaster", "left_force_sensor_broadcaster"
        }
    ]
    assert len(spawners) == 2
    assert all(
        node["arguments"][-2:] == ["--param-file", "/tmp/x503-controllers.yaml"]
        for node in spawners
    )
    assert len(documents) == 1
    document = documents[0]
    assert set(document) == {
        "right_force_sensor_broadcaster", "left_force_sensor_broadcaster"
    }
    right = document["right_force_sensor_broadcaster"]["ros__parameters"]
    assert right["startup_id"]
    assert right["calibration_valid"] is True
    assert right["scale_factors"] == [0.1, 0.1, 0.1, 0.001, 0.001, 0.001]
    assert right["wrench_topic"] == "/rt_control/right_x503b/wrench"


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


def test_generated_controller_parameters_are_private_and_removed_idempotently():
    loaded = module()
    document = {
        "right_force_sensor_broadcaster": {
            "ros__parameters": {"startup_id": "run-1"}
        }
    }

    path = loaded._write_controller_parameter_file(document)
    try:
        assert stat.S_IMODE(path.stat().st_mode) == 0o600
        assert yaml.safe_load(path.read_text(encoding="utf-8")) == document
    finally:
        loaded._remove_controller_parameter_file(None, path=str(path))
        loaded._remove_controller_parameter_file(None, path=str(path))
    assert not path.exists()
