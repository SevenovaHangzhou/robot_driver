"""integ_diagnostics: self-check stage B, passive fault reporting, R-OUT-09 /rt_control/readiness.

Reads only the artifact's diagnostics.yaml, the stage-A report and the internal state
broadcast. It never commands a device and never takes part in enable admission.
"""

import json
from pathlib import Path
import uuid

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import robot_interfaces_qos
from robot_system_interfaces.msg import DomainReadiness, ErrorCode, ErrorInfo
import yaml

from integ_composer.self_check import PROBLEM_DREE

from .evaluate import complete, evaluate, readiness
from .report import state_directory, write_self_check

STATE_TOPIC = "/rt_internal_state_broadcaster/dynamic_joint_states"
READINESS_TOPIC = "/rt_control/readiness"
SEVERITY = {"WARN": ErrorInfo.WARN, "FAULT": ErrorInfo.FAULT}


def dree_codes(tables, codes):
    """Numeric DREE value of every name this node may publish, from the pinned contract."""
    names = set(PROBLEM_DREE.values()) | {
        fault["dree"] for table in tables.values() for fault in table["faults"]
    }
    missing = sorted(name for name in names if not hasattr(codes, name))
    if missing:
        raise RuntimeError(
            f"robot_interfaces ErrorCode lacks {missing}; update the pinned contract"
        )
    return {name: getattr(codes, name) for name in names}


class DiagnosticsNode(Node):
    def __init__(self, codes=ErrorCode, **options):
        super().__init__("rt_diagnostics", **options)
        self._generated_dir = self.declare_parameter("generated_dir", "").value
        self._scope = self.declare_parameter("control_scope", "").value
        diagnostics = yaml.safe_load(
            (Path(self._generated_dir) / "diagnostics.yaml").read_text(encoding="utf-8")
        )
        self._devices = diagnostics["devices"]
        self._tables = diagnostics["fault_tables"]
        self._codes = dree_codes(self._tables, codes)
        report = state_directory() / "self_check" / f"{self._scope}.json"
        stage_a = json.loads(report.read_text(encoding="utf-8")) if report.is_file() else {}
        self._before_bus = (
            stage_a["entries"] if stage_a.get("generated_dir") == self._generated_dir else []
        )
        self._pending = True
        self._current = []
        self._instance = uuid.uuid4().hex
        self._publisher = self.create_publisher(
            DomainReadiness, READINESS_TOPIC, robot_interfaces_qos.latched()
        )
        self.create_subscription(DynamicJointState, STATE_TOPIC, self._on_state, 10)
        self.create_timer(1.0, self._publish)
        self._publish()

    def _on_state(self, message):
        values = {
            f"{joint}/{name}": value
            for joint, interfaces in zip(message.joint_names, message.interface_values)
            for name, value in zip(interfaces.interface_names, interfaces.values)
        }
        if self._pending:
            if not complete(self._devices, values):
                return
            # Stage B runs once, on the first complete sample after hardware activation.
            self._pending = False
            self._current = evaluate(self._devices, self._tables, values)
            write_self_check(
                self._scope,
                self._generated_dir,
                [entry for entry in self._before_bus if not entry["blocking"]] + self._current,
                stage="after_activation",
            )
            self._publish()
            return
        current = evaluate(self._devices, self._tables, values)
        if current != self._current:
            self._current = current
            self._publish()

    def _error(self, entry):
        code = self._codes[entry["dree"]]
        error = ErrorInfo()
        error.code = code
        error.message = entry["dree"]
        error.retryable = (code // 100) % 10 == 1
        error.severity = SEVERITY[entry["severity"]]
        error.source = entry["source"]
        error.detail = entry["detail"]
        return error

    def _publish(self):
        fields = readiness(self._pending, self._before_bus, self._current)
        message = DomainReadiness()
        message.header.stamp = self.get_clock().now().to_msg()
        message.domain = "rt_control"
        message.readiness_name = self._scope
        message.ready = fields["ready"]
        message.status = fields["status"]
        message.operational_state = fields["operational_state"]
        message.producer_instance_id = self._instance
        message.blockers = fields["blockers"]
        message.errors = [self._error(entry) for entry in fields["errors"]]
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
