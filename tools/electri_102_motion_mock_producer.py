#!/usr/bin/env python3
"""Minimal public-IDL-only ELECTRI-102 producer for mock integration.

With no arguments this program only prints the frozen 10 Hz/30 Hz message
plan and does not import or initialize ROS.  Passing
``--allow-command-publication`` explicitly runs the complete hold-only mock
workflow: FJT_READY -> ROLLING_READY, open, prime, rolling updates, graceful
stop, finalize, and return to FJT_READY.

The example never enables hardware and never cancels an FJT goal that it does
not own.  The caller must cancel its own FJT goal and wait for the Action result
before invoking the live mock path.  Use only with ``use_mock_hardware:=true``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import uuid
from dataclasses import dataclass
from typing import Callable, Sequence


AXIS_NAMES = (
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "turn",
    "updown",
)
AXIS_SET_HASH = bytes.fromhex(
    "25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526"
)
KNOT_INTERVAL_NS = 100_000_000
BATCH_RATE_HZ = 30
DEFAULT_POINT_COUNT = 6
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 0


@dataclass(frozen=True)
class Knot:
    time_ns: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


def build_hold_suffix(
    replace_from_ns: int,
    positions: Sequence[float],
    *,
    point_count: int = DEFAULT_POINT_COUNT,
    knot_interval_ns: int = KNOT_INTERVAL_NS,
) -> list[Knot]:
    """Build a self-contained, exact-splice hold suffix.

    A hold is intentionally used here: every future axis value is sampled from
    the same already-accepted constant trajectory.  A visual-servo producer
    must replace this with its shaped 14-axis q/qdot suffix and carry forward
    non-servo axes from the previously accepted trajectory at each knot time.
    """

    values = tuple(float(value) for value in positions)
    if len(values) != len(AXIS_NAMES) or not all(math.isfinite(value) for value in values):
        raise ValueError("positions must contain exactly 14 finite values")
    if replace_from_ns < 0:
        raise ValueError("replace_from_ns must be non-negative")
    if not 2 <= point_count <= 64:
        raise ValueError("point_count must be in [2, 64]")
    if knot_interval_ns <= 0:
        raise ValueError("knot_interval_ns must be positive")
    zero_velocities = (0.0,) * len(AXIS_NAMES)
    return [
        Knot(
            time_ns=replace_from_ns + index * knot_interval_ns,
            positions=values,
            velocities=zero_velocities,
        )
        for index in range(point_count)
    ]


def point_count_for_open_response(
    required_initial_horizon_ns: int,
    max_horizon_ns: int,
    buffer_capacity: int,
) -> int:
    point_count = math.ceil(required_initial_horizon_ns / KNOT_INTERVAL_NS) + 1
    horizon_ns = (point_count - 1) * KNOT_INTERVAL_NS
    if point_count > buffer_capacity or point_count > 64:
        raise RuntimeError(
            f"100 ms example needs {point_count} points but capacity={buffer_capacity}"
        )
    if horizon_ns > max_horizon_ns:
        raise RuntimeError(
            f"100 ms example horizon={horizon_ns} exceeds max_horizon={max_horizon_ns}"
        )
    return point_count


def dry_run_payload() -> dict:
    knots = build_hold_suffix(0, (0.0,) * len(AXIS_NAMES))
    return {
        "requirement": "ELECTRI-102",
        "command_publication_enabled": False,
        "axis_order": list(AXIS_NAMES),
        "axis_set_hash": AXIS_SET_HASH.hex(),
        "batch_rate_hz": BATCH_RATE_HZ,
        "knot_interval_ms": KNOT_INTERVAL_NS // 1_000_000,
        "point_count": len(knots),
        "initial_horizon_ms": (knots[-1].time_ns - knots[0].time_ns) // 1_000_000,
        "workflow": [
            "caller_cancel_owned_fjt_and_wait_terminal",
            "set_mode_fjt_to_rolling",
            "open",
            "prime_immediately",
            "publish_complete_suffix_at_30hz",
            "request_stop",
            "wait_holding",
            "finalize",
            "set_mode_rolling_to_fjt",
        ],
        "safety": "dry-run only; use --allow-command-publication only with mock hardware",
    }


def _set_uuid(message, value: uuid.UUID) -> None:
    message.uuid = list(value.bytes)


def _uuid_is_zero(message) -> bool:
    return not any(message.uuid)


def _same_uuid(left, right) -> bool:
    return bytes(left.uuid) == bytes(right.uuid)


def run_public_mock_workflow(run_seconds: float, timeout_seconds: float) -> dict:
    """Execute the example using only public robot_interfaces packages."""

    import rclpy
    from rclpy.node import Node

    from robot_interfaces_qos import rolling_command, rolling_state
    from robot_motion_interfaces.msg import RollingJointPoint, RollingJointTargetBatch
    from robot_rt_control_interfaces.msg import (
        JointControlMode,
        RollingJointControlState,
        RollingRejectCode,
        RollingSessionState,
        RollingStopReason,
    )
    from robot_rt_control_interfaces.srv import (
        CloseRollingJointSession,
        OpenRollingJointSession,
        SetJointControlMode,
    )

    class PublicProducer(Node):
        def __init__(self) -> None:
            super().__init__(f"electri_102_motion_mock_{os.getpid()}")
            self.latest_state = None
            self.mode_client = self.create_client(
                SetJointControlMode, "/rt/joint_control/set_mode"
            )
            self.open_client = self.create_client(
                OpenRollingJointSession, "/rt/rolling_joint_control/open"
            )
            self.close_client = self.create_client(
                CloseRollingJointSession, "/rt/rolling_joint_control/close"
            )
            self.publisher = self.create_publisher(
                RollingJointTargetBatch,
                "/rt/rolling_joint_control/update",
                rolling_command(),
            )
            self.subscription = self.create_subscription(
                RollingJointControlState,
                "/rt/rolling_joint_control/state",
                self._handle_state,
                rolling_state(),
            )

        def _handle_state(self, message) -> None:
            self.latest_state = message

        def call(self, client, request, label: str):
            deadline = time.monotonic() + timeout_seconds
            while not client.service_is_ready() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
            if not client.service_is_ready():
                raise RuntimeError(f"{label} service unavailable")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_seconds)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"{label} service timed out")
            return future.result()

        def wait_state(self, predicate: Callable[[object], bool], label: str):
            deadline = time.monotonic() + timeout_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
                state = self.latest_state
                if state is not None and predicate(state):
                    return state
            raise RuntimeError(f"state timeout while waiting for {label}")

        def wait_for_update_subscription(self) -> None:
            deadline = time.monotonic() + timeout_seconds
            subscription_count = 0
            while time.monotonic() < deadline:
                subscription_count = self.publisher.get_subscription_count()
                if subscription_count == 1:
                    return
                rclpy.spin_once(self, timeout_sec=0.02)
            raise RuntimeError(
                "rolling update subscription unavailable or duplicated: "
                f"count={subscription_count}"
            )

        @staticmethod
        def make_batch(open_response, sequence: int, replace_from_ns: int, point_count: int):
            batch = RollingJointTargetBatch()
            batch.protocol_major = PROTOCOL_MAJOR
            batch.protocol_minor = PROTOCOL_MINOR
            batch.controller_boot_id = open_response.controller_boot_id
            batch.session_id = open_response.session_id
            batch.client_instance_id = open_response.client_instance_id
            batch.sequence = sequence
            batch.replace_from_ns = replace_from_ns
            knots = build_hold_suffix(
                replace_from_ns,
                open_response.hold_positions,
                point_count=point_count,
            )
            for knot in knots:
                point = RollingJointPoint()
                point.time_from_session_start_ns = knot.time_ns
                point.positions = list(knot.positions)
                point.velocities = list(knot.velocities)
                batch.points.append(point)
            return batch

    rclpy.init(args=None)
    node = PublicProducer()
    client_id = uuid.uuid4()
    mode_is_rolling = False
    open_response = None
    finalized = False
    sequence = 0

    def mode_request(expected: int, target: int):
        request = SetJointControlMode.Request()
        request.protocol_major = PROTOCOL_MAJOR
        request.protocol_minor = PROTOCOL_MINOR
        _set_uuid(request.client_instance_id, client_id)
        _set_uuid(request.request_id, uuid.uuid4())
        request.expected_mode.value = expected
        request.target_mode.value = target
        return node.call(node.mode_client, request, "set-mode")

    def require_service_success(response, label: str) -> None:
        if not response.accepted:
            raise RuntimeError(
                f"{label} rejected: result={response.result.value} "
                f"error={response.error.code} retryable={response.error.retryable} "
                f"detail={response.error.detail!r}"
            )

    def close_request(operation: int):
        request = CloseRollingJointSession.Request()
        request.protocol_major = PROTOCOL_MAJOR
        request.protocol_minor = PROTOCOL_MINOR
        request.controller_boot_id = open_response.controller_boot_id
        request.session_id = open_response.session_id
        request.client_instance_id = open_response.client_instance_id
        _set_uuid(request.request_id, uuid.uuid4())
        request.operation = operation
        return node.call(node.close_client, request, "close")

    try:
        mode_response = mode_request(
            JointControlMode.FJT_READY, JointControlMode.ROLLING_READY
        )
        require_service_success(mode_response, "FJT->rolling")
        if (
            mode_response.mode.value != JointControlMode.ROLLING_READY
            or not mode_response.source_controller_deactivated
            or not mode_response.target_controller_activated
            or mode_response.restart_required
            or _uuid_is_zero(mode_response.controller_boot_id)
        ):
            raise RuntimeError("FJT->rolling response lacks verified switch evidence")
        mode_is_rolling = True
        # The verified mode service round-trip also gives DDS discovery time.
        # Check the single command consumer before opening the 100 ms prime
        # window; never spend discovery time after a session is open.
        node.wait_for_update_subscription()

        open_request = OpenRollingJointSession.Request()
        open_request.protocol_major = PROTOCOL_MAJOR
        open_request.protocol_minor = PROTOCOL_MINOR
        _set_uuid(open_request.client_instance_id, client_id)
        _set_uuid(open_request.request_id, uuid.uuid4())
        open_request.expected_controller_boot_id = mode_response.controller_boot_id
        open_request.axis_set_hash = list(AXIS_SET_HASH)
        open_response = node.call(node.open_client, open_request, "open")
        require_service_success(open_response, "open")
        if (
            open_response.protocol_major != PROTOCOL_MAJOR
            or open_response.protocol_minor != PROTOCOL_MINOR
            or open_response.session_state.value != RollingSessionState.PRIMING
            or not _same_uuid(open_response.client_instance_id, open_request.client_instance_id)
            or bytes(open_response.axis_set_hash) != AXIS_SET_HASH
        ):
            raise RuntimeError("open response violates protocol/session/axis identity")
        point_count = point_count_for_open_response(
            open_response.required_initial_horizon_ns,
            open_response.max_horizon_ns,
            open_response.buffer_capacity,
        )

        sequence = 1
        prime = node.make_batch(
            open_response,
            sequence,
            open_response.initial_replaceable_from_ns,
            point_count,
        )
        node.publisher.publish(prime)
        state = node.wait_state(
            lambda value: (
                value.session_state.value == RollingSessionState.RUNNING
                and value.last_accepted_sequence >= sequence
                and _same_uuid(value.session_id, open_response.session_id)
            ),
            "prime acceptance",
        )

        period_seconds = 1.0 / BATCH_RATE_HZ
        deadline = time.monotonic() + run_seconds
        next_publication = time.monotonic()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
            state = node.latest_state or state
            if state.stop_reason.value != RollingStopReason.NONE:
                raise RuntimeError(
                    f"rolling stopped unexpectedly: reason={state.stop_reason.value}"
                )
            if (
                state.last_rejected_sequence != 0
                and state.last_rejected_sequence >= sequence
                and state.last_reject.value != RollingRejectCode.NONE
            ):
                raise RuntimeError(
                    f"batch rejected: sequence={state.last_rejected_sequence} "
                    f"code={state.last_reject.value}"
                )
            now = time.monotonic()
            if now < next_publication:
                rclpy.spin_once(node, timeout_sec=min(0.01, next_publication - now))
                continue
            sequence += 1
            batch = node.make_batch(
                open_response,
                sequence,
                state.replaceable_from_ns,
                point_count,
            )
            node.publisher.publish(batch)
            next_publication += period_seconds

        stop_response = close_request(CloseRollingJointSession.Request.REQUEST_STOP)
        require_service_success(stop_response, "request-stop")
        node.wait_state(
            lambda value: (
                value.session_state.value == RollingSessionState.HOLDING
                and _same_uuid(value.session_id, open_response.session_id)
            ),
            "HOLDING",
        )
        finalize_response = close_request(CloseRollingJointSession.Request.FINALIZE)
        require_service_success(finalize_response, "finalize")
        if not finalize_response.completed:
            raise RuntimeError("finalize was accepted but not completed")
        finalized = True
        node.wait_state(lambda value: not value.has_session, "session removal")

        return_response = mode_request(
            JointControlMode.ROLLING_READY, JointControlMode.FJT_READY
        )
        require_service_success(return_response, "rolling->FJT")
        if (
            return_response.mode.value != JointControlMode.FJT_READY
            or not return_response.source_controller_deactivated
            or not return_response.target_controller_activated
            or return_response.restart_required
        ):
            raise RuntimeError("rolling->FJT response lacks verified switch evidence")
        mode_is_rolling = False
        return {
            "requirement": "ELECTRI-102",
            "result": "PASS",
            "published_sequences": sequence,
            "batch_rate_hz": BATCH_RATE_HZ,
            "knot_interval_ms": KNOT_INTERVAL_NS // 1_000_000,
            "point_count": point_count,
            "limits_source": open_response.limits_source.value,
            "test_only_limits": open_response.test_only_limits,
            "final_mode": "FJT_READY",
        }
    finally:
        if open_response is not None and not finalized:
            try:
                close_request(CloseRollingJointSession.Request.REQUEST_STOP)
                node.wait_state(
                    lambda value: value.session_state.value
                    == RollingSessionState.HOLDING,
                    "cleanup HOLDING",
                )
                close_request(CloseRollingJointSession.Request.FINALIZE)
            except Exception as error:  # Best-effort cleanup is reported, never hidden.
                node.get_logger().error(f"best-effort rolling cleanup failed: {error}")
        if mode_is_rolling:
            try:
                mode_request(JointControlMode.ROLLING_READY, JointControlMode.FJT_READY)
            except Exception as error:
                node.get_logger().error(f"best-effort return-to-FJT failed: {error}")
        node.destroy_node()
        rclpy.shutdown()


def parse_arguments(arguments: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--allow-command-publication",
        action="store_true",
        help=(
            "publish rolling commands; use only in a verified mock launch after the "
            "caller has cancelled and awaited its own FJT goal"
        ),
    )
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=5.0,
        help="duration of 30 Hz hold suffix updates (default: 5)",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=5.0,
        help="service/state timeout (default: 5)",
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] | None = None) -> int:
    options = parse_arguments(arguments)
    if not options.allow_command_publication:
        print(json.dumps(dry_run_payload(), indent=2, sort_keys=True))
        return 0
    if options.run_seconds <= 0.0 or options.timeout_seconds <= 0.0:
        print("ERROR: durations must be positive", file=sys.stderr)
        return 2
    try:
        result = run_public_mock_workflow(
            options.run_seconds, options.timeout_seconds
        )
    except (ImportError, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
