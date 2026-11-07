"""The started selection is never guessed: both entries require the variant and the unit."""

import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument

BRINGUP = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    "launch_file", ["rt_control_scope_runtime.launch.py", "rt_control_module.launch.py"]
)
def test_variant_and_unit_have_no_default(launch_file):
    spec = importlib.util.spec_from_file_location(launch_file, BRINGUP / "launch" / launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    arguments = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    for name in ("robot_variant", "machine_unit"):
        assert arguments[name].default_value is None, name
