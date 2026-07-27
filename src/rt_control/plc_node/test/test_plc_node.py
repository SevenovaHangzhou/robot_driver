from builtin_interfaces.msg import Time
from rclpy.executors import ExternalShutdownException

import plc_node.plc_node as plc_node_module
from plc_node.plc_logic import PlcIoSnapshot
from plc_node.plc_node import OUTPUT_SERVICE_BINDINGS, plc_state_message, state_is_fresh


def test_node_exposes_exactly_three_unambiguous_write_services() -> None:
    assert tuple(name for name, _bit in OUTPUT_SERVICE_BINDINGS) == (
        "/plc/left_solenoid",
        "/plc/right_solenoid",
        "/plc/vacuum_pump",
    )


def test_plc_state_message_projects_confirmed_semantics() -> None:
    message = plc_state_message(
        PlcIoSnapshot(
            left_vacuum_established=True,
            right_vacuum_established=False,
            left_solenoid_on=False,
            right_solenoid_on=True,
            vacuum_pump_on=True,
            io_alarm=3,
        ),
        connected=True,
        data_fresh=True,
        error="",
        stamp=Time(sec=10),
    )

    assert message.connected
    assert message.data_fresh
    assert message.left_vacuum_established
    assert not message.right_vacuum_established
    assert not message.left_solenoid_on
    assert message.right_solenoid_on
    assert message.vacuum_pump_on
    assert message.io_alarm == 3
    assert message.header.frame_id == "plc"


def test_state_freshness_rejects_missing_stale_and_future_data() -> None:
    assert state_is_fresh(9.0, now_s=10.0, timeout_s=1.5)
    assert not state_is_fresh(None, now_s=10.0, timeout_s=1.5)
    assert not state_is_fresh(8.0, now_s=10.0, timeout_s=1.5)
    assert not state_is_fresh(11.0, now_s=10.0, timeout_s=1.5)


def test_main_treats_context_shutdown_as_clean_exit(monkeypatch) -> None:
    class FakeNode:
        destroyed = False

        def destroy_node(self):
            self.destroyed = True

    node = FakeNode()
    monkeypatch.setattr(plc_node_module.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(plc_node_module, "PlcNode", lambda: node)
    monkeypatch.setattr(
        plc_node_module.rclpy,
        "spin",
        lambda _node: (_ for _ in ()).throw(ExternalShutdownException()),
    )
    monkeypatch.setattr(plc_node_module.rclpy, "ok", lambda: False)

    plc_node_module.main()

    assert node.destroyed
