"""integ_diagnostics: self-check (startup stage B and on demand) and R-OUT-09 /rt_control/readiness.

Readiness comes from the controllers' own state (enable_manager, motion controllers, hardware
components); the real-time controllers detect and handle faults, Runtime-LOG keeps the evidence.
This node never commands a device and never takes part in enable admission.
"""

import json
from pathlib import Path
import time
import uuid

from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import robot_interfaces_qos
from robot_system_interfaces.msg import DomainReadiness, ErrorCode, ErrorInfo
from std_srvs.srv import Trigger
import yaml

from integ_composer.self_check import PROBLEM_DREE

from .domain import DREE_NAMES, domain_readiness, failure_entry, readiness_line
from .evaluate import complete, evaluate
from .report import state_directory, write_self_check

STATE_TOPIC = "/rt_internal_state_broadcaster/dynamic_joint_states"
READINESS_TOPIC = "/rt_control/readiness"
SELF_CHECK_SERVICE = "/rt_control/self_check"
ENABLE_DIAGNOSTIC = "/robot/rt_control/enable_manager"
# enable_manager publishes its diagnostics at 1 Hz; older values count as unavailable.
ENABLE_STALE_S = 2.5
SEVERITY = {"WARN": ErrorInfo.WARN, "FAULT": ErrorInfo.FAULT}


def dree_codes(tables, codes):
    """Numeric DREE value of every name this node may publish, from the pinned contract."""
    names = (
        set(PROBLEM_DREE.values())
        | DREE_NAMES
        | {fault["dree"] for table in tables.values() for fault in table["faults"]}
    )
    missing = sorted(name for name in names if not hasattr(codes, name))
    if missing:
        raise RuntimeError(
            f"robot_interfaces ErrorCode lacks {missing}; update the pinned contract"
        )
    return {name: getattr(codes, name) for name in names}


class DiagnosticsNode(Node):
    def __init__(self, codes=ErrorCode, monotonic=time.monotonic, **options):
        super().__init__("rt_diagnostics", **options)
        self._generated_dir = self.declare_parameter("generated_dir", "").value
        self._scope = self.declare_parameter("control_scope", "").value
        generated = Path(self._generated_dir)
        diagnostics = yaml.safe_load((generated / "diagnostics.yaml").read_text(encoding="utf-8"))
        lifecycle = yaml.safe_load((generated / "lifecycle.yaml").read_text(encoding="utf-8"))
        self._devices = diagnostics["devices"]
        self._tables = diagnostics["fault_tables"]
        self._codes = dree_codes(self._tables, codes)
        self._motion_controllers = lifecycle["motion_controllers"]
        self._hardware_names = [item["name"] for item in lifecycle["hardware"]]
        report = state_directory() / "self_check" / f"{self._scope}.json"
        stage_a = json.loads(report.read_text(encoding="utf-8")) if report.is_file() else {}
        self._before_bus = (
            stage_a["entries"] if stage_a.get("generated_dir") == self._generated_dir else []
        )
        self._monotonic = monotonic
        self._self_check = {
            "pending": True,
            "entries": [entry for entry in self._before_bus if not entry["blocking"]],
        }
        self._values = {}
        self._enable, self._enable_at = None, -float("inf")
        self._controllers = self._hardware = None
        self._last = None
        self._line = None
        self._instance = uuid.uuid4().hex
        self._publisher = self.create_publisher(
            DomainReadiness, READINESS_TOPIC, robot_interfaces_qos.latched()
        )
        self.create_subscription(DynamicJointState, STATE_TOPIC, self._on_state, 10)
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, robot_interfaces_qos.diagnostic()
        )
        self._list_controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._list_hardware = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.create_service(Trigger, SELF_CHECK_SERVICE, self._on_self_check)
        self.create_timer(1.0, self._tick)
        self._publish(force=True)

    def _on_state(self, message):
        self._values = {
            f"{joint}/{name}": value
            for joint, interfaces in zip(message.joint_names, message.interface_values)
            for name, value in zip(interfaces.interface_names, interfaces.values)
        }
        if self._self_check["pending"] and complete(self._devices, self._values):
            # Stage B runs once, on the first complete sample after hardware activation.
            self._run_self_check("after_activation")

    def _run_self_check(self, stage):
        current = evaluate(self._devices, self._tables, self._values)
        entries = [entry for entry in self._before_bus if not entry["blocking"]] + current
        self._self_check = {"pending": False, "entries": entries}
        path = write_self_check(self._scope, self._generated_dir, entries, stage=stage)
        self._publish()
        return entries, path

    def _on_self_check(self, _request, response):
        if not complete(self._devices, self._values):
            response.success = False
            response.message = "no complete device state sample yet"
            return response
        entries, path = self._run_self_check("on_demand")
        blocking = [entry for entry in entries if entry["blocking"]]
        response.success = not blocking
        response.message = (
            f"{len(blocking)} blocking, {len(entries) - len(blocking)} warning; report {path}"
        )
        return response

    def _on_diagnostics(self, message):
        for status in message.status:
            if status.name == ENABLE_DIAGNOSTIC:
                self._enable = {item.key: item.value for item in status.values}
                self._enable_at = self._monotonic()
                self._publish()

    def _tick(self):
        for client, request, store in (
            (self._list_controllers, ListControllers.Request(), self._store_controllers),
            (self._list_hardware, ListHardwareComponents.Request(), self._store_hardware),
        ):
            if client.service_is_ready():
                client.call_async(request).add_done_callback(store)
        self._publish(force=True)

    def _store_controllers(self, future):
        states = {item.name: item.state for item in future.result().controller}
        self._controllers = {
            name: states.get(name, "unloaded") for name in self._motion_controllers
        }
        self._publish()

    def _store_hardware(self, future):
        states = {item.name: item.state.label for item in future.result().component}
        self._hardware = {name: states.get(name, "missing") for name in self._hardware_names}
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

    def _publish(self, force=False):
        enable = self._enable if self._monotonic() - self._enable_at <= ENABLE_STALE_S else None
        failure = None
        if enable is not None and enable.get("state") == "FAILED":
            failure = failure_entry(enable, self._devices, self._tables, self._values)
        fields = domain_readiness(
            self._self_check, enable, self._controllers, self._hardware, failure
        )
        if fields == self._last and not force:
            return
        line = readiness_line(fields)
        if line != self._line:
            # Runtime-LOG follows this line in launch.log as a timeline event.
            self.get_logger().info(line)
            self._line = line
        self._last = fields
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
