import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.command_gateway import CommandGateway  # noqa: E402


def test_gateway_dispatches_once_and_completes_only_matching_result() -> None:
    dispatched = []
    results = []
    gateway = CommandGateway(
        dispatch=lambda command, operation_id: dispatched.append((command, operation_id)),
        on_result=lambda success, text: results.append((success, text)),
    )

    assert gateway.request("enable") is True
    assert gateway.request("enable") is False
    assert dispatched == [("enable", 1)]
    assert gateway.complete(2, True, "stale") is False
    assert results == []
    assert gateway.complete(1, True, "使能成功") is True
    assert results == [(True, "使能成功")]


def test_gateway_does_not_retry_a_failed_operation() -> None:
    dispatched = []
    results = []
    gateway = CommandGateway(
        dispatch=lambda command, operation_id: dispatched.append((command, operation_id)),
        on_result=lambda success, text: results.append((success, text)),
    )

    gateway.request("reset_fault")
    gateway.complete(1, False, "复位超时")

    assert dispatched == [("reset_fault", 1)]
    assert results == [(False, "复位超时")]
