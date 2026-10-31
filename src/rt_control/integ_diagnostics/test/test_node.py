"""The real node end to end: stage A report in, state samples in, /rt_control/readiness out."""

import json
from pathlib import Path
from types import SimpleNamespace
import time

from control_msgs.msg import DynamicJointState, InterfaceValue
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import robot_interfaces_qos
from robot_system_interfaces.msg import DomainReadiness, ErrorInfo
import yaml

from integ_diagnostics.node import READINESS_TOPIC, STATE_TOPIC, DiagnosticsNode, dree_codes
from integ_diagnostics.report import write_self_check

RT = Path(__file__).resolve().parents[2]
TABLE = yaml.safe_load(
    (RT / "hwcfg_machines/devices/faults/zeroerr_erob.yaml").read_text(encoding="utf-8")
)
# The pinned robot_interfaces predates the device codes (robot_interfaces#12); same values.
CODES = SimpleNamespace(
    RT_DEVICE_OFFLINE=1001,
    RT_DEVICE_IDENTITY_MISMATCH=1010,
    RT_DEVICE_FAULT=1028,
    RT_STO_ACTIVE=1070,
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


def state(al_state, error_code):
    message = DynamicJointState()
    message.joint_names = ["ethercat_slave_3", "left_joint3"]
    message.interface_values = [
        InterfaceValue(interface_names=["al_state"], values=[float(al_state)]),
        InterfaceValue(interface_names=["error_code"], values=[float(error_code)]),
    ]
    return message


def test_missing_contract_codes_refuse_to_start():
    with pytest.raises(RuntimeError, match="update the pinned contract"):
        dree_codes({"zeroerr_erob": TABLE}, SimpleNamespace(RT_DEVICE_FAULT=1028))


def test_node_reports_stage_a_then_stage_b_then_passive_faults(runtime):
    generated, state_dir = runtime
    node = DiagnosticsNode(
        codes=CODES,
        parameter_overrides=[
            Parameter("generated_dir", value=str(generated)),
            Parameter("control_scope", value="arms_only"),
        ],
    )
    peer = rclpy.create_node("diagnostics_test_peer")
    received = []
    peer.create_subscription(
        DomainReadiness, READINESS_TOPIC, received.append, robot_interfaces_qos.latched()
    )
    publisher = peer.create_publisher(DynamicJointState, STATE_TOPIC, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)

    def until(predicate, publish=None):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if publish is not None:
                publisher.publish(publish)
            executor.spin_once(timeout_sec=0.05)
            if received and predicate(received[-1]):
                return received[-1]
        pytest.fail(f"no matching readiness; last: {received[-1] if received else None}")

    pending = until(lambda m: m.operational_state == "self_check_pending")
    assert not pending.ready and list(pending.blockers) == ["self_check_pending"]
    assert [(e.code, e.severity) for e in pending.errors] == [(1128, ErrorInfo.WARN)]
    assert pending.domain == "rt_control" and pending.readiness_name == "arms_only"

    # Stage B: the drive still holds 0xA000 after the master left OP; declared, so ready.
    done = until(lambda m: m.operational_state == "self_check_done", state(8, 0xA000))
    assert done.ready and done.status == "DEGRADED" and done.errors[0].retryable
    report = json.loads((state_dir / "self_check/arms_only.json").read_text())
    assert report["stage"] == "after_activation" and report["passed"]

    # Passive: the drive reports over-current while running.
    fault = until(lambda m: not m.ready, state(8, 0x2214))
    [error] = fault.errors
    assert (error.code, error.severity, error.retryable) == (1028, ErrorInfo.FAULT, False)
    assert error.source == "arms/left_joint3/drive" and "电机电流过流" in error.detail
    assert list(fault.blockers) == ["device_fault"] and fault.status == "UNAVAILABLE"

    cleared = until(lambda m: m.ready, state(8, 0))
    assert cleared.status == "HEALTHY" and not cleared.errors
    assert len({m.producer_instance_id for m in received}) == 1
    executor.shutdown()
    node.destroy_node()
    peer.destroy_node()
