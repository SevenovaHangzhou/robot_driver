import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from rt_control_operator_ui.app import spin_ros_until_stopped  # noqa: E402


class ExpectedShutdown(Exception):
    pass


class FakeExecutor:
    def spin(self) -> None:
        raise ExpectedShutdown()


def test_ros_shutdown_notifies_qt_event_loop_exactly_once() -> None:
    notifications = []

    spin_ros_until_stopped(
        FakeExecutor(), ExpectedShutdown, lambda: notifications.append("stopped")
    )

    assert notifications == ["stopped"]
