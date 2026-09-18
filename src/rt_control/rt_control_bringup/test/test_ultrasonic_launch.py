"""V3 ultrasonic startup is independent of motor and legacy runtime launches."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest


BRINGUP = Path(__file__).resolve().parents[1]


def load_launch():
    path = BRINGUP / "launch/rt_control_ultrasonic.launch.py"
    spec = importlib.util.spec_from_file_location("ultrasonic_launch_test", path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def test_v3_installs_the_independent_ultrasonic_entry():
    assert "launch/rt_control_ultrasonic.launch.py" in (BRINGUP / "CMakeLists.txt").read_text()
    assert "<exec_depend>modbus_tcp_rtu485</exec_depend>" in (BRINGUP / "package.xml").read_text()


@pytest.mark.parametrize("mock, expected", [("true", False), ("false", True)])
def test_mock_never_starts_the_hardware_node(mock, expected):
    context = LaunchContext()
    context.launch_configurations["use_mock_hardware"] = mock
    description = load_launch().generate_launch_description()
    node = next(entity for entity in description.entities if isinstance(entity, Node))
    assert node.condition.evaluate(context) is expected


def test_launch_contains_only_the_ultrasonic_node(monkeypatch):
    loaded = load_launch()
    nodes = []
    original = loaded.Node

    def record_node(**kwargs):
        nodes.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(loaded, "Node", record_node)
    description = loaded.generate_launch_description()
    assert len(nodes) == 1
    assert nodes[0]["package"] == "modbus_tcp_rtu485"
    assert nodes[0]["executable"] == "ultrasonic_node"
    assert nodes[0]["name"] == "ultrasonic_node"
    context = LaunchContext()
    context.launch_configurations["ultrasonic_config"] = "/tmp/test-ultrasonic.yaml"
    assert perform_substitutions(context, [nodes[0]["parameters"][0]]) == "/tmp/test-ultrasonic.yaml"
    arguments = {
        entity.name for entity in description.entities if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {"use_mock_hardware", "ultrasonic_config"}
