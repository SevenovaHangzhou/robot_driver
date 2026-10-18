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


def test_scope_launch_reads_artifact_locks_then_preflights_before_any_node(monkeypatch):
    path = BRINGUP / "launch/rt_control_scope_runtime.launch.py"
    spec = importlib.util.spec_from_file_location("scope_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.generate_launch_description()
    assert not hasattr(module, "compose"), "Launch must not compose or write configuration"
    calls = []

    def load_artifact(directory, **selection):
        calls.append(("artifact", directory, selection["control_scope"]))
        return SimpleNamespace(
            staged_hardware_activation=True,
            lifecycle={"bus_locks": ["ethercat-master-0"]},
            controller_order={"inactive": [], "active": []},
            robot_description="<robot name='synthetic'/>",
            runtime_dir=Path(directory),
            runtime_parameters={},
        )

    monkeypatch.setattr(module, "load_artifact", load_artifact)
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
        generated_dir="/synthetic/artifact",
    )
    with pytest.raises(RuntimeError, match="synthetic preflight"):
        module.setup(context)
    assert calls == [
        ("artifact", "/synthetic/artifact", "chassis_only"),
        ("lock", ("ethercat-master-0",)),
        ("preflight",),
    ]


@pytest.mark.parametrize(
    "inactive, expected",
    # Field tuning (ELECTRI-127) runs only when motion controllers exist to be tuned.
    [([], ["lifecycle"]), (["swerve_controller"], ["motion", "tuning", "lifecycle"])],
)
def test_scope_launch_spawns_motion_only_when_the_scope_declares_it(
    monkeypatch, inactive, expected
):
    path = BRINGUP / "launch/rt_control_scope_runtime.launch.py"
    spec = importlib.util.spec_from_file_location("scope_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module,
        "load_artifact",
        lambda directory, **_: SimpleNamespace(
            staged_hardware_activation=True,
            lifecycle={"bus_locks": []},
            controller_order={"inactive": inactive, "active": ["enable_manager"]},
            robot_description="<robot name='synthetic'/>",
            runtime_dir=Path(directory),
            runtime_parameters={},
        ),
    )
    monkeypatch.setattr(module, "preflight", lambda _: None)
    spawners = []
    real_node = module.Node

    def node(**kwargs):
        if kwargs["executable"] == "spawner":
            spawners.append("motion" if "--inactive" in kwargs["arguments"] else "lifecycle")
            assert kwargs["arguments"][0] in [*inactive, "enable_manager"]
        if kwargs["executable"] == "scope_operator_tuning":
            spawners.append("tuning")
        return real_node(**kwargs)

    monkeypatch.setattr(module, "Node", node)
    context = LaunchContext()
    context.launch_configurations.update(
        use_mock_hardware="true",
        control_scope="synthetic",
        physical_profile="",
        robot_variant="Alfa_V3",
        machine_unit="Alfa_V3-01",
        generated_dir="/synthetic/artifact",
    )
    module.setup(context)
    assert sorted(spawners) == sorted(expected)


def test_capability_document_is_derived_from_the_lifecycle_plan():
    document = lifecycle.capability_document(
        {
            "control_scope": "chassis_only",
            "use_mock_hardware": False,
            "capabilities": [{"name": "chassis_velocity", "topic": "/cmd_vel"}],
            "hardware": [{"name": "ecat", "plugin": "ethercat_driver/EthercatDriver"}],
        }
    )
    assert document["schema_version"] == 1
    assert document["control_scope"] == "chassis_only"
    assert document["capabilities"][0]["topic"] == "/cmd_vel"


def test_realtime_qualification_is_skipped_for_mock_and_rejects_shared_rt_cpu(tmp_path):
    plan = {"use_mock_hardware": True, "host": {"rt_cpu": 7, "housekeeping_cpus": [7]}}
    assert lifecycle.qualify_realtime(tmp_path, plan, activated=True) is None
    plan["use_mock_hardware"] = False
    with pytest.raises(RuntimeError, match="housekeeping"):
        lifecycle.qualify_realtime(tmp_path, plan, activated=True)
