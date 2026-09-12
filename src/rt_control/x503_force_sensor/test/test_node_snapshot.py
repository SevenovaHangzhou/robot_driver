"""Runtime uses immutable startup metadata and never starts an SDO reader."""
import json
import subprocess

from control_msgs.msg import DynamicJointState, InterfaceValue
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
import pytest

from x503_force_sensor.node import X503WrenchBridge


def _snapshot():
    values = {"snapshot_valid": "true", "slave_position": "14",
              "engineering_unit_contract": "force_N_torque_Nm",
              "validity_policy": "sample_codes_in_range",
              "sample_code_min": "-999999", "sample_code_max": "999999",
              **{f"decimal_{i}": str(1 if i < 4 else 3) for i in range(1, 7)},
              **{f"unit_{i}": str(5 if i < 4 else 7) for i in range(1, 7)}}
    return json.dumps({"schema_version": 1, "startup_id": "run-1", "source": "preop_sdo",
                       "sensors": [{"sensor_name": "right_force_sensor", "slave_position": 14,
                                    "values": values, "error": ""}]})


@pytest.fixture
def bridge(monkeypatch):
    context = Context()
    context.init(args=[], domain_id=142)
    published = {}

    class Recorder:
        def __init__(self, topic): self.topic = topic
        def publish(self, message): published.setdefault(self.topic, []).append(message)

    monkeypatch.setattr(Node, "create_publisher", lambda self, kind, topic, qos: Recorder(topic))
    parameters = {
        "sensor_names": ["right_force_sensor"], "slave_positions": [14],
        "frame_ids": ["right_ft_sensor_link"], "raw_topics": ["/test/raw"],
        "wrench_topics": ["/test/wrench"], "startup_id": "run-1", "preop_snapshot_json": _snapshot(),
    }
    node = X503WrenchBridge(context=context, parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()])
    yield node, published
    node.destroy_node()
    context.shutdown()


def _frame(al_state=8, link=1):
    message = DynamicJointState()
    message.header.stamp.sec = 1
    message.joint_names = ["right_force_sensor", "ethercat_master", "ethercat_slave_14"]
    message.interface_values = [
        InterfaceValue(interface_names=[*(f"channel_{i}_raw" for i in range(1, 7)),
                                         *(f"sample_code_{i}_raw" for i in range(1, 7))],
                       values=[100., 200., 300., 1000., 2000., 3000., 0., 0., 0., 0., 0., 0.]),
        InterfaceValue(interface_names=["link_up"], values=[float(link)]),
        InterfaceValue(interface_names=["al_state"], values=[float(al_state)]),
    ]
    return message


def test_pdo_conversion_uses_startup_snapshot_without_any_device_command(bridge, monkeypatch):
    node, published = bridge
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: pytest.fail("Runtime device command"))
    node._on_dynamic_state(_frame())
    wrench = published["/test/wrench"][-1].wrench
    assert (wrench.force.x, wrench.force.y, wrench.force.z) == (10., 20., 30.)
    assert (wrench.torque.x, wrench.torque.y, wrench.torque.z) == (1., 2., 3.)
    assert not any(s.topic_name == "/rt_control/x503b/calibration" for s in node.subscriptions)
    status = published["/rt_control/x503b/calibration"][0].status[0]
    assert dict((v.key, v.value) for v in status.values)["startup_id"] == "run-1"


def test_recovered_link_does_not_reuse_the_invalidated_snapshot(bridge):
    node, published = bridge
    node._on_dynamic_state(_frame())
    node._on_dynamic_state(_frame(al_state=2))
    node._on_dynamic_state(_frame())
    assert len(published["/test/raw"]) == 3
    assert len(published["/test/wrench"]) == 1
    status = published["/rt_control/x503b/calibration"][-1].status[0]
    assert dict((v.key, v.value) for v in status.values)["snapshot_valid"] == "false"
