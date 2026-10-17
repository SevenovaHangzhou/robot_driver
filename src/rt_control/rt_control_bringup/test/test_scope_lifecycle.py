"""Generic scope lifecycle: order, teardown and bus ownership; no ROS graph or bus."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext

from rt_control_bringup import scope_lifecycle as lifecycle

BRINGUP = Path(__file__).resolve().parents[1]
PLAN = {
    "hardware_activation_order": ["canopen_encoders", "ecat_master"],
    "motion_controllers": ["swerve_controller"],
    "lifecycle_controllers": ["joint_state_broadcaster", "enable_manager"],
}


class Backend:
    def __init__(self, fail=()):
        self.calls, self.fail = [], set(fail)

    def hardware_state(self, name, state):
        self.calls.append(("hardware", name, state))
        return ("hardware", name, state) not in self.fail

    def quiesce(self, names):
        self.calls.append(("quiesce", tuple(names)))
        return ("quiesce", tuple(names)) not in self.fail

    def disable(self):
        self.calls.append(("disable",))
        return ("disable",) not in self.fail


def test_state_sources_activate_first_and_failure_stops_before_actuators():
    backend = Backend(fail={("hardware", "canopen_encoders", lifecycle.ACTIVE)})
    with pytest.raises(RuntimeError, match="canopen_encoders"):
        lifecycle.activate_hardware(backend, PLAN)
    assert backend.calls == [("hardware", "canopen_encoders", lifecycle.ACTIVE)]


def test_shutdown_disables_before_lifecycle_and_cleans_hardware_in_reverse():
    backend = Backend()
    assert lifecycle.shutdown_sequence(backend, PLAN, log=lambda _: None)
    assert backend.calls == [
        ("quiesce", ("swerve_controller",)),
        ("disable",),
        ("quiesce", ("joint_state_broadcaster", "enable_manager")),
        ("hardware", "ecat_master", lifecycle.INACTIVE),
        ("hardware", "ecat_master", lifecycle.UNCONFIGURED),
        ("hardware", "canopen_encoders", lifecycle.INACTIVE),
        ("hardware", "canopen_encoders", lifecycle.UNCONFIGURED),
    ]


def test_failed_disable_is_reported_and_claimed_hardware_is_not_deactivated():
    backend = Backend(
        fail={("disable",), ("quiesce", ("joint_state_broadcaster", "enable_manager"))}
    )
    assert not lifecycle.shutdown_sequence(backend, PLAN, log=lambda _: None)
    assert not [call for call in backend.calls if call[0] == "hardware"]


def test_bus_lock_is_exclusive_until_released(tmp_path):
    held = lifecycle.acquire_bus_locks(["ethercat-master-0", "can-can0"], tmp_path)
    with pytest.raises(RuntimeError, match="ethercat-master-0"):
        lifecycle.acquire_bus_locks(["ethercat-master-0"], tmp_path)
    for stream in held:
        stream.close()
    again = lifecycle.acquire_bus_locks(["ethercat-master-0"], tmp_path)
    for stream in again:
        stream.close()


def test_scope_launch_locks_then_preflights_before_any_node(monkeypatch, tmp_path):
    path = BRINGUP / "launch/rt_control_scope_runtime.launch.py"
    spec = importlib.util.spec_from_file_location("scope_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.generate_launch_description()
    calls = []

    def compose(**kwargs):
        calls.append(("compose", kwargs["control_scope"]))
        return SimpleNamespace(
            staged_hardware_activation=True,
            lifecycle={"bus_locks": ["ethercat-master-0"]},
            controller_order={"inactive": [], "active": []},
            robot_description="<robot name='synthetic'/>",
        )

    monkeypatch.setattr(module, "compose", compose)
    monkeypatch.setattr(
        module, "acquire_bus_locks", lambda names: calls.append(("lock", tuple(names))) or []
    )

    def preflight(_):
        calls.append(("preflight",))
        raise RuntimeError("synthetic preflight rejection")

    monkeypatch.setattr(module, "preflight", preflight)
    monkeypatch.setattr(module, "Node", lambda **_: pytest.fail("Node before preflight"))
    context = LaunchContext()
    context.launch_configurations.update(
        use_mock_hardware="false",
        control_scope="chassis_only",
        physical_profile="",
        robot_variant="Alfa_V3",
        machine_unit="Alfa_V3-01",
        runtime_dir=str(tmp_path),
    )
    with pytest.raises(RuntimeError, match="synthetic preflight"):
        module.setup(context)
    assert calls == [("compose", "chassis_only"), ("lock", ("ethercat-master-0",)), ("preflight",)]
