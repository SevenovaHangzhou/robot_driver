"""The real node end to end: self-check report, controller state in, /rt/readiness out."""

import json
from pathlib import Path
from types import SimpleNamespace
import time

from control_msgs.msg import DynamicJointState, InterfaceValue
from controller_manager_msgs.msg import ControllerState, HardwareComponentState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import robot_interfaces_qos
from robot_system_interfaces.msg import DomainReadiness, ErrorInfo
from std_srvs.srv import Trigger
import yaml

from integ_diagnostics.node import (
    ENABLE_DIAGNOSTIC,
    READINESS_TOPIC,
    SELF_CHECK_SERVICE,
    STATE_TOPIC,
    DiagnosticsNode,
    dree_codes,
)
from integ_diagnostics.report import write_self_check

RT = Path(__file__).resolve().parents[2]
TABLE = yaml.safe_load(
    (RT / "hwcfg_machines/devices/faults/zeroerr_erob.yaml").read_text(encoding="utf-8")
)
CODES = SimpleNamespace(
    RT_DEVICE_OFFLINE=1001,
    RT_DEVICE_IDENTITY_MISMATCH=1010,
    RT_ENABLE_FAILED=1021,
    RT_RESTART_REQUIRED=1027,
    RT_DEVICE_FAULT=1028,
    RT_STO_ACTIVE=1070,
    RT_ENABLE_MANAGER_NOT_READY=1101,
    RT_DEVICE_FAULT_RESETTABLE=1128,
)


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    generated = tmp_path / "artifact"
    generated.mkdir()
    diagnostics = {
        "schema_version": 1,
        "devices": [
            {
                "source": "arms/left_joint3/drive",
                "role": "drive",
                "address": {"master_id": 0, "ring_position": 3},
                "state": {
                    "al_state": "ethercat_slave_3/al_state",
                    "error_code": "left_joint3/error_code",
                },
                "allowed_startup_codes": ["0xA000"],
                "fault_table": "zeroerr_erob",
            }
        ],
        "fault_tables": {"zeroerr_erob": TABLE},
    }
    (generated / "diagnostics.yaml").write_text(yaml.safe_dump(diagnostics, allow_unicode=True))
    lifecycle = {
        "motion_controllers": ["arm_jtc"],
        "hardware": [{"name": "ecat_arms", "plugin": "ethercat_driver/EthercatDriver"}],
    }
    (generated / "lifecycle.yaml").write_text(yaml.safe_dump(lifecycle))
    monkeypatch.setenv("RT_CONTROL_STATE_DIR", str(tmp_path / "state"))
    warning = {
        "source": "arms/left_joint3/drive",
        "owner": "hw_ethercat",
        "dree": "RT_DEVICE_FAULT_RESETTABLE",
        "severity": "WARN",
        "blocking": False,
        "detail": "Declared startup code 0xa000",
    }
    write_self_check("arms_only", str(generated), [warning], directory=tmp_path / "state")
    rclpy.init()
    yield generated, tmp_path / "state"
    rclpy.shutdown()


def device_state(error_code):
    message = DynamicJointState()
    message.joint_names = ["ethercat_slave_3", "left_joint3"]
    message.interface_values = [
        InterfaceValue(interface_names=["al_state"], values=[8.0]),
        InterfaceValue(interface_names=["error_code"], values=[float(error_code)]),
    ]
    return message


def enable_status(state, **values):
    status = DiagnosticStatus(name=ENABLE_DIAGNOSTIC)
    values = {"state": state, "stage": "success", "failed_joint": "", **values}
    status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    return DiagnosticArray(status=[status])


def test_missing_contract_codes_refuse_to_start():
    with pytest.raises(RuntimeError, match="update the pinned contract"):
        dree_codes({"zeroerr_erob": TABLE}, SimpleNamespace(RT_DEVICE_FAULT=1028))


def test_readiness_follows_self_check_and_controller_state(runtime):
    generated, state_dir = runtime
    node = DiagnosticsNode(
        codes=CODES,
        parameter_overrides=[
            Parameter("generated_dir", value=str(generated)),
            Parameter("control_scope", value="arms_only"),
        ],
    )
    peer = rclpy.create_node("diagnostics_test_peer")
    controller = {"state": "inactive"}

    def list_controllers(_request, response):
        response.controller = [ControllerState(name="arm_jtc", state=controller["state"])]
        return response

    def list_hardware(_request, response):
        item = HardwareComponentState(name="ecat_arms")
        item.state.label = "active"
        response.component = [item]
        return response

    peer.create_service(ListControllers, "/controller_manager/list_controllers", list_controllers)
    peer.create_service(
        ListHardwareComponents, "/controller_manager/list_hardware_components", list_hardware
    )
    received = []
    peer.create_subscription(
        DomainReadiness, READINESS_TOPIC, received.append, robot_interfaces_qos.latched()
    )
    states = peer.create_publisher(DynamicJointState, STATE_TOPIC, 10)
    diagnostics = peer.create_publisher(
        DiagnosticArray, "/diagnostics", robot_interfaces_qos.diagnostic()
    )
    self_check = peer.create_client(Trigger, SELF_CHECK_SERVICE)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)

    def until(predicate, *messages):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            for publisher, message in messages:
                publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
            if received and predicate(received[-1]):
                return received[-1]
        pytest.fail(f"no matching readiness; last: {received[-1] if received else None}")

    pending = until(lambda m: m.operational_state == "self_check_pending")
    assert not pending.ready and "self_check_pending" in pending.blockers
    assert [(e.code, e.severity) for e in pending.errors] == [(1128, ErrorInfo.WARN)]

    # Stage B on the first complete sample; enable_manager idle: not ready, warnings kept.
    idle = until(
        lambda m: m.operational_state == "idle",
        (states, device_state(0xA000)),
        (diagnostics, enable_status("IDLE")),
    )
    assert list(idle.blockers) == ["not_enabled"] and idle.status == "UNAVAILABLE"
    report = json.loads((state_dir / "self_check/arms_only.json").read_text())
    assert report["stage"] == "after_activation" and report["passed"]

    # Enabled, but the trajectory controller is not active yet.
    until(
        lambda m: list(m.blockers) == ["controller_inactive"],
        (diagnostics, enable_status("ENABLED")),
    )
    controller["state"] = "active"
    healthy = until(lambda m: m.ready, (diagnostics, enable_status("ENABLED")))
    assert healthy.status == "HEALTHY" and not healthy.errors

    # enable_manager latched the drive going offline while enabled.
    offline = enable_status(
        "FAILED",
        stage="device_offline",
        failed_joint="left_joint3",
        offline_feedback="ethercat_domain/process_data_age_ms",
    )
    failed = until(lambda m: not m.ready, (diagnostics, offline))
    assert list(failed.blockers) == ["device_offline"]
    [error] = [e for e in failed.errors if e.severity == ErrorInfo.FAULT]
    assert (error.code, error.severity, error.source) == (
        1001,
        ErrorInfo.FAULT,
        "arms/left_joint3/drive",
    )

    # On-demand self-check on the latest sample.
    future = self_check.call_async(Trigger.Request())
    deadline = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert future.result().success and "0 blocking" in future.result().message
    report = json.loads((state_dir / "self_check/arms_only.json").read_text())
    assert report["stage"] == "on_demand"
    assert len({m.producer_instance_id for m in received}) == 1
    executor.shutdown()
    node.destroy_node()
    peer.destroy_node()
