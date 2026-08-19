#!/usr/bin/env python3
"""Public-IDL DDS harness for the ELECTRI-102 Motion producer.

This is an isolated protocol peer, not a robot or controller simulator.  It
checks the producer's complete public service/topic workflow over DDS while
the controller packages independently verify trajectory semantics and the
fake 250 Hz RT loop.  No hardware, controller manager, bus, or enable endpoint
is opened by this process.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Sequence


PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 0
AXIS_SET_HASH = bytes.fromhex(
    "25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526"
)


def _copy_uuid(destination, source) -> None:
    destination.uuid = [int(value) for value in source.uuid]


def _set_uuid(destination, value: uuid.UUID) -> None:
    destination.uuid = list(value.bytes)


def _uuid_bytes(value) -> bytes:
    return bytes(value.uuid)


@contextmanager
def _isolated_ros_daemon():
    """Ensure discovery is warm without disturbing an existing daemon."""

    try:
        domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError as error:
        raise RuntimeError("ROS_DOMAIN_ID must be an integer") from error
    if not 1 <= domain_id <= 232:
        raise RuntimeError(
            "public mock harness requires an isolated ROS_DOMAIN_ID in [1, 232]"
        )
    status = subprocess.run(
        ["ros2", "daemon", "status"], capture_output=True, text=True, check=False
    )
    was_running = "The daemon is running" in status.stdout
    if not was_running:
        started = subprocess.run(
            ["ros2", "daemon", "start"], capture_output=True, text=True, check=False
        )
        if started.returncode != 0:
            raise RuntimeError(f"could not start ROS discovery daemon: {started.stderr}")
    try:
        yield not was_running
    finally:
        if not was_running:
            subprocess.run(
                ["ros2", "daemon", "stop"],
                capture_output=True,
                text=True,
                check=False,
            )


def _serve_peer(ready_file: Path, timeout_seconds: float) -> dict:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    from robot_interfaces_qos import rolling_command, rolling_state
    from robot_motion_interfaces.msg import RollingJointTargetBatch
    from robot_rt_control_interfaces.msg import (
        JointControlMode,
        RollingJointControlState,
        RollingLimitsSource,
        RollingRejectCode,
        RollingServiceResult,
        RollingSessionState,
        RollingStopReason,
    )
    from robot_rt_control_interfaces.srv import (
        CloseRollingJointSession,
        OpenRollingJointSession,
        SetJointControlMode,
    )

    class ContractPeer(Node):
        def __init__(self) -> None:
            super().__init__(f"electri_102_public_peer_{os.getpid()}")
            self.boot_id = uuid.uuid4()
            self.session_id = uuid.uuid4()
            self.client_id: bytes | None = None
            self.mode = JointControlMode.FJT_READY
            self.session_state = RollingSessionState.NONE
            self.has_session = False
            self.last_sequence = 0
            self.accepted_batches = 0
            self.failures: list[str] = []
            self.mode_transitions: list[tuple[int, int]] = []
            self.state_publisher = self.create_publisher(
                RollingJointControlState,
                "/rt/rolling_joint_control/state",
                rolling_state(),
            )
            self.update_subscription = self.create_subscription(
                RollingJointTargetBatch,
                "/rt/rolling_joint_control/update",
                self._on_update,
                rolling_command(),
            )
            self.mode_service = self.create_service(
                SetJointControlMode,
                "/rt/joint_control/set_mode",
                self._on_mode,
            )
            self.open_service = self.create_service(
                OpenRollingJointSession,
                "/rt/rolling_joint_control/open",
                self._on_open,
            )
            self.close_service = self.create_service(
                CloseRollingJointSession,
                "/rt/rolling_joint_control/close",
                self._on_close,
            )
            self.timer = self.create_timer(0.02, self._publish_state)

        @staticmethod
        def _accept(response) -> None:
            response.accepted = True
            response.result.value = RollingServiceResult.NONE
            response.error.code = 0
            response.error.message = ""
            response.error.retryable = False
            response.error.severity = response.error.OK
            response.error.source = "electri_102_public_mock_harness"
            response.error.detail = ""

        def _on_mode(self, request, response):
            if (
                request.protocol_major != PROTOCOL_MAJOR
                or request.protocol_minor != PROTOCOL_MINOR
                or request.expected_mode.value != self.mode
            ):
                self.failures.append("invalid compare-and-set mode request")
                response.accepted = False
                response.result.value = RollingServiceResult.WRONG_MODE
                return response
            target = request.target_mode.value
            allowed = {
                (JointControlMode.FJT_READY, JointControlMode.ROLLING_READY),
                (JointControlMode.ROLLING_READY, JointControlMode.FJT_READY),
            }
            if (self.mode, target) not in allowed:
                self.failures.append("invalid mode transition")
                response.accepted = False
                response.result.value = RollingServiceResult.WRONG_MODE
                return response
            source = self.mode
            self.mode = target
            self.mode_transitions.append((source, target))
            self._accept(response)
            response.mode.value = target
            _set_uuid(response.controller_boot_id, self.boot_id)
            _copy_uuid(response.request_id, request.request_id)
            response.source_controller_deactivated = True
            response.target_controller_activated = True
            response.restart_required = False
            return response

        def _on_open(self, request, response):
            valid = (
                self.mode == JointControlMode.ROLLING_READY
                and not self.has_session
                and request.protocol_major == PROTOCOL_MAJOR
                and request.protocol_minor == PROTOCOL_MINOR
                and _uuid_bytes(request.expected_controller_boot_id)
                == self.boot_id.bytes
                and bytes(request.axis_set_hash) == AXIS_SET_HASH
            )
            if not valid:
                self.failures.append("invalid open request")
                response.accepted = False
                response.result.value = RollingServiceResult.WRONG_REQUEST
                return response
            self.client_id = _uuid_bytes(request.client_instance_id)
            self.has_session = True
            self.session_state = RollingSessionState.PRIMING
            self._accept(response)
            _copy_uuid(response.request_id, request.request_id)
            response.protocol_major = PROTOCOL_MAJOR
            response.protocol_minor = PROTOCOL_MINOR
            _set_uuid(response.controller_boot_id, self.boot_id)
            _set_uuid(response.session_id, self.session_id)
            _copy_uuid(response.client_instance_id, request.client_instance_id)
            response.session_state.value = RollingSessionState.PRIMING
            response.axis_set_hash = list(AXIS_SET_HASH)
            response.limits_version = [0x42] * 32
            response.limits_source.value = RollingLimitsSource.PROVISIONAL
            response.test_only_limits = False
            response.capability_bits = (
                response.CAPABILITY_CUBIC_HERMITE
                | response.CAPABILITY_EXPLICIT_SPLICE_BOUNDARY
                | response.CAPABILITY_DIRECTIONAL_LIMITS
                | response.CAPABILITY_SYNCHRONOUS_C1_STOP
                | response.CAPABILITY_LATEST_VALID_PENDING
            )
            response.transport_max_points = 256
            response.buffer_capacity = 64
            response.required_initial_horizon_ns = 500_000_000
            response.max_horizon_ns = 600_000_000
            response.replace_lead_ns = 16_000_000
            response.update_timeout_ns = 200_000_000
            response.nominal_controller_period_ns = 4_000_000
            response.initial_replaceable_from_ns = 16_000_000
            response.hold_positions = [0.0] * 14
            response.hold_velocities = [0.0] * 14
            return response

        def _on_update(self, batch) -> None:
            findings: list[str] = []
            if batch.protocol_major != PROTOCOL_MAJOR or batch.protocol_minor != PROTOCOL_MINOR:
                findings.append("wrong protocol")
            if _uuid_bytes(batch.controller_boot_id) != self.boot_id.bytes:
                findings.append("wrong boot id")
            if _uuid_bytes(batch.session_id) != self.session_id.bytes:
                findings.append("wrong session id")
            if self.client_id is None or _uuid_bytes(batch.client_instance_id) != self.client_id:
                findings.append("wrong client id")
            if batch.sequence != self.last_sequence + 1:
                findings.append("non-consecutive sequence")
            if len(batch.points) != 6:
                findings.append("point count is not six")
            else:
                for index, point in enumerate(batch.points):
                    expected_time = batch.replace_from_ns + index * 100_000_000
                    if point.time_from_session_start_ns != expected_time:
                        findings.append("knots are not exactly 100 ms apart")
                        break
                    values = list(point.positions) + list(point.velocities)
                    if len(point.positions) != 14 or len(point.velocities) != 14:
                        findings.append("point is not fixed 14-axis shape")
                        break
                    if not all(math.isfinite(value) for value in values):
                        findings.append("point contains non-finite value")
                        break
                    if any(point.velocities):
                        findings.append("hold example contains non-zero velocity")
                        break
            if findings:
                self.failures.extend(findings)
                return
            self.last_sequence = batch.sequence
            self.accepted_batches += 1
            self.session_state = RollingSessionState.RUNNING

        def _on_close(self, request, response):
            identity_ok = (
                self.has_session
                and _uuid_bytes(request.controller_boot_id) == self.boot_id.bytes
                and _uuid_bytes(request.session_id) == self.session_id.bytes
                and self.client_id is not None
                and _uuid_bytes(request.client_instance_id) == self.client_id
            )
            if not identity_ok:
                self.failures.append("invalid close identity")
                response.accepted = False
                response.result.value = RollingServiceResult.WRONG_SESSION
                return response
            self._accept(response)
            _copy_uuid(response.request_id, request.request_id)
            if request.operation == request.REQUEST_STOP:
                self.session_state = RollingSessionState.HOLDING
                response.completed = False
                response.session_state.value = RollingSessionState.HOLDING
                response.stop_reason.value = RollingStopReason.GRACEFUL_CLOSE
            elif request.operation == request.FINALIZE and self.session_state == RollingSessionState.HOLDING:
                self.has_session = False
                self.session_state = RollingSessionState.NONE
                response.completed = True
                response.session_state.value = RollingSessionState.NONE
                response.stop_reason.value = RollingStopReason.GRACEFUL_CLOSE
            else:
                self.failures.append("invalid close phase")
                response.accepted = False
                response.result.value = RollingServiceResult.WRONG_REQUEST
            return response

        def _publish_state(self) -> None:
            message = RollingJointControlState()
            message.protocol_major = PROTOCOL_MAJOR
            message.protocol_minor = PROTOCOL_MINOR
            _set_uuid(message.controller_boot_id, self.boot_id)
            if self.has_session:
                _set_uuid(message.session_id, self.session_id)
                if self.client_id is not None:
                    message.client_instance_id.uuid = list(self.client_id)
            message.control_mode.value = self.mode
            message.session_state.value = self.session_state
            message.has_session = self.has_session
            message.has_accepted_update = self.accepted_batches > 0
            message.last_seen_sequence = self.last_sequence
            message.last_accepted_sequence = self.last_sequence
            message.replaceable_from_ns = 16_000_000
            message.buffered_until_ns = 516_000_000
            message.available_horizon_ns = 500_000_000
            message.buffer_point_count = 6 if self.has_session else 0
            message.buffer_capacity = 64
            message.last_reject.value = RollingRejectCode.NONE
            message.stop_reason.value = (
                RollingStopReason.GRACEFUL_CLOSE
                if self.session_state == RollingSessionState.HOLDING
                else RollingStopReason.NONE
            )
            message.limits_source.value = RollingLimitsSource.PROVISIONAL
            message.axis_set_hash = list(AXIS_SET_HASH)
            message.limits_version = [0x42] * 32
            message.accepted_count = self.accepted_batches
            message.state_sequence = self.accepted_batches + 1
            self.state_publisher.publish(message)

    rclpy.init(args=None)
    peer = ContractPeer()
    executor = SingleThreadedExecutor()
    executor.add_node(peer)
    started = time.monotonic()
    ready_file.write_text("ready\n")
    try:
        deadline = started + timeout_seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if peer.failures:
                break
            if (
                peer.mode_transitions
                == [
                    (JointControlMode.FJT_READY, JointControlMode.ROLLING_READY),
                    (JointControlMode.ROLLING_READY, JointControlMode.FJT_READY),
                ]
                and peer.accepted_batches >= 2
                and not peer.has_session
            ):
                break
    finally:
        executor.remove_node(peer)
        peer.destroy_node()
        rclpy.shutdown()
    expected_transitions = [
        (JointControlMode.FJT_READY, JointControlMode.ROLLING_READY),
        (JointControlMode.ROLLING_READY, JointControlMode.FJT_READY),
    ]
    findings = list(peer.failures)
    if peer.mode_transitions != expected_transitions:
        findings.append(f"mode transitions={peer.mode_transitions}")
    if peer.accepted_batches < 2:
        findings.append(f"accepted_batches={peer.accepted_batches}")
    if peer.has_session:
        findings.append("session still open")
    if findings:
        raise RuntimeError("; ".join(findings))
    return {
        "requirement": "ELECTRI-102",
        "result": "PASS",
        "transport": "public ROS 2 IDL over DDS",
        "hardware_started": False,
        "controller_manager_started": False,
        "accepted_batches": peer.accepted_batches,
        "mode_transitions": ["FJT_READY->ROLLING_READY", "ROLLING_READY->FJT_READY"],
        "duration_ms": round((time.monotonic() - started) * 1000),
    }


def run_harness(producer: Path, run_seconds: float, timeout_seconds: float) -> dict:
    """Run the peer and producer as clean sibling processes.

    Keeping the orchestrator free of an initialized RMW context also makes the
    DDS-process boundary match the real Motion/RT-Control deployment.
    """

    with _isolated_ros_daemon() as daemon_started, tempfile.TemporaryDirectory(
        prefix="electri-102-public-dds-"
    ) as temporary:
        directory = Path(temporary)
        ready_file = directory / "peer.ready"
        report_file = directory / "peer.json"
        peer_command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--peer-only",
            "--peer-ready-file",
            str(ready_file),
            "--peer-report-file",
            str(report_file),
            "--timeout-seconds",
            str(run_seconds + 3.0 * timeout_seconds),
        ]
        peer_process = subprocess.Popen(
            peer_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        started = time.monotonic()
        try:
            ready_deadline = time.monotonic() + timeout_seconds
            while not ready_file.is_file() and time.monotonic() < ready_deadline:
                if peer_process.poll() is not None:
                    break
                time.sleep(0.02)
            if not ready_file.is_file():
                stdout, stderr = peer_process.communicate(timeout=2.0)
                raise RuntimeError(
                    f"public peer did not become ready: {stderr.strip() or stdout.strip()}"
                )

            # Allow the peer participant to announce before the independent
            # producer process joins the isolated ROS domain.
            time.sleep(0.5)
            producer_command = [
                sys.executable,
                str(producer),
                "--allow-command-publication",
                "--run-seconds",
                str(run_seconds),
                "--timeout-seconds",
                str(timeout_seconds),
            ]
            completed = subprocess.run(
                producer_command,
                capture_output=True,
                text=True,
                timeout=run_seconds + 3.0 * timeout_seconds,
                check=False,
            )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"producer exit={completed.returncode}: {completed.stderr.strip()}"
                )
            producer_result = json.loads(completed.stdout)
            if producer_result.get("result") != "PASS":
                raise RuntimeError("producer did not report PASS")

            stdout, stderr = peer_process.communicate(timeout=timeout_seconds)
            if peer_process.returncode != 0 or not report_file.is_file():
                raise RuntimeError(
                    "public peer failed: "
                    + (stderr.strip() or stdout.strip() or "missing peer report")
                )
            result = json.loads(report_file.read_text())
            result["producer"] = producer_result
            result["discovery_daemon_started_by_harness"] = daemon_started
            result["duration_ms"] = round((time.monotonic() - started) * 1000)
            return result
        finally:
            if peer_process.poll() is None:
                peer_process.terminate()
                try:
                    peer_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    peer_process.kill()
                    peer_process.wait(timeout=2.0)


def parse_arguments(arguments: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--producer",
        type=Path,
        default=Path(__file__).with_name("electri_102_motion_mock_producer.py"),
    )
    parser.add_argument("--run-seconds", type=float, default=0.2)
    parser.add_argument("--timeout-seconds", type=float, default=5.0)
    parser.add_argument("--peer-only", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--peer-ready-file", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--peer-report-file", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] | None = None) -> int:
    options = parse_arguments(arguments)
    if options.run_seconds <= 0.0 or options.timeout_seconds <= 0.0:
        print("ERROR: durations must be positive", file=sys.stderr)
        return 2
    try:
        if options.peer_only:
            if options.peer_ready_file is None or options.peer_report_file is None:
                raise RuntimeError("peer-only mode requires ready and report files")
            result = _serve_peer(
                options.peer_ready_file.resolve(), options.timeout_seconds
            )
            options.peer_report_file.resolve().write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n"
            )
        else:
            result = run_harness(
                options.producer.resolve(strict=True),
                options.run_seconds,
                options.timeout_seconds,
            )
    except (ImportError, OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
