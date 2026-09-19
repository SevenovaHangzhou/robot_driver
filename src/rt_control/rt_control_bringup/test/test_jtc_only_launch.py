"""V3 JTC-only entry must not depend on rolling or change the default startup."""

import importlib.util

import pytest

from test_arm_motion_runtime import BRINGUP, HARDWARE, DESCRIPTION


def _launch_module():
    path = BRINGUP / "launch/rt_control_arm_runtime.launch.py"
    spec = importlib.util.spec_from_file_location("v3_arm_runtime_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("mock", [True, False])
def test_jtc_only_entry_is_explicit_and_uses_verified_real_calibration(tmp_path, mock):
    from rt_control_bringup import arm_motion_runtime

    launch_path = BRINGUP / "launch/rt_control_arm_runtime.launch.py"
    launch_source = launch_path.read_text()
    start_source = (BRINGUP / "scripts/rt_control_start").read_text()
    assert "--arm-jtc" in start_source
    assert "jtc_only:=true" in start_source
    assert "jtc_only" in launch_source
    assert "calibration_file" in launch_source
    if mock:
        runtime = arm_motion_runtime.build_arm_motion_runtime(
            HARDWARE, DESCRIPTION, tmp_path, use_mock_hardware=True, jtc_only=True
        )
        assert "rolling_trajectory_controller" not in runtime.controllers
    else:
        with pytest.raises(ValueError, match="calibration is not verified"):
            arm_motion_runtime.build_arm_motion_runtime(
                HARDWARE, DESCRIPTION, tmp_path, use_mock_hardware=False, jtc_only=True
            )


def test_jtc_only_launch_provides_public_enable_adapter(monkeypatch, tmp_path):
    from launch import LaunchContext
    from launch_ros.actions import Node

    module = _launch_module()
    shares = {
        "rt_control_bringup": BRINGUP,
        "robot_hw_ethercat": HARDWARE,
        "robot_description": DESCRIPTION,
    }
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: str(shares[package]))
    monkeypatch.setattr(module.tempfile, "mkdtemp", lambda **kwargs: str(tmp_path))
    node_arguments = []
    original_node = module.Node

    def capture_node(**kwargs):
        node_arguments.append(kwargs)
        return original_node(**kwargs)

    monkeypatch.setattr(module, "Node", capture_node)
    context = LaunchContext()
    context.launch_configurations.update({
        "use_mock_hardware": "true", "jtc_only": "true", "calibration_file": "",
    })
    actions = module.setup(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert nodes
    assert any("whole_body_jtc" in node.get("arguments", []) for node in node_arguments)
    assert not any("rolling_trajectory_controller" in node.get("arguments", []) for node in node_arguments)
    assert any(
        node.get("package") == "control_api_adapter"
        and node.get("executable") == "control_enable_adapter"
        for node in node_arguments
    )
