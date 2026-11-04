"""The real action server against a simulated head controller and readiness publisher."""

import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
import robot_interfaces_qos
from robot_rt_control_interfaces.action import MoveHead
from robot_system_interfaces.msg import DomainReadiness, ErrorCode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from integ_head_move.node import HeadMoveServer


class SimulatedHead:
    """Follows the last command at `speed` rad/s and publishes /joint_states at 50 Hz."""

    def __init__(self, node, speed):
        self.speed = speed
        self.positions = [0.0, 0.0]
        self.commands = []
        self._target = list(self.positions)
        self._states = node.create_publisher(JointState, "/joint_states", 10)
        node.create_subscription(
            Float64MultiArray, "/head_position_controller/commands", self._on_command, 10
        )
        node.create_timer(0.02, self._step)
        self._node = node

    def _on_command(self, message):
        self.commands.append(list(message.data))
        self._target = list(message.data)

    def _step(self):
        step = self.speed * 0.02
        self.positions = [
            now + max(-step, min(step, goal - now))
            for now, goal in zip(self.positions, self._target)
        ]
        message = JointState(name=["H1_joint", "H2_joint"], position=self.positions)
        message.header.stamp = self._node.get_clock().now().to_msg()
        self._states.publish(message)


@pytest.fixture
def rig(generated_dir):
    rclpy.init()
    server = HeadMoveServer(
        parameter_overrides=[Parameter("generated_dir", value=str(generated_dir))]
    )
    peer = rclpy.create_node("head_move_test_peer")
    head = SimulatedHead(peer, speed=2.0)
    readiness = peer.create_publisher(
        DomainReadiness, "/rt/readiness", robot_interfaces_qos.latched()
    )
    client = ActionClient(peer, MoveHead, "/rt/head/move")
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.add_node(peer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def set_ready(ready):
        readiness.publish(DomainReadiness(ready=ready, blockers=[] if ready else ["not_enabled"]))

    set_ready(True)
    assert client.wait_for_server(timeout_sec=5.0)
    time.sleep(0.3)
    yield head, client, set_ready
    executor.shutdown()
    server.destroy_node()
    peer.destroy_node()
    rclpy.shutdown()


def wait(future, timeout=10.0):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert future.done()
    return future.result()


def goal(h1, h2):
    return MoveHead.Goal(head_joint_position_rad=h1, head_pitch_joint_position_rad=h2)


def test_goal_moves_the_head_and_succeeds_on_arrival(rig):
    head, client, _ = rig
    handle = wait(client.send_goal_async(goal(0.3, -0.2)))
    assert handle.accepted
    result = wait(handle.get_result_async()).result
    assert result.error.code == ErrorCode.SUCCESS
    assert head.commands[0] == pytest.approx([0.3, -0.2])
    assert list(result.final_state.actual_positions) == pytest.approx([0.3, -0.2], abs=0.01)


def test_busy_head_rejects_a_second_goal_and_cancel_holds_the_measured_position(rig):
    head, client, _ = rig
    head.speed = 0.1
    first = wait(client.send_goal_async(goal(0.5, 0.0)))
    time.sleep(0.3)
    second = wait(client.send_goal_async(goal(-0.5, 0.0)))
    assert first.accepted and not second.accepted
    wait(first.cancel_goal_async())
    result = wait(first.get_result_async()).result
    assert result.error.code == ErrorCode.CANCELED
    hold = head.commands[-1]
    assert 0.0 < hold[0] < 0.5
    assert hold == pytest.approx(list(result.final_state.actual_positions), abs=0.01)


def test_out_of_range_and_not_ready_goals_abort_with_their_codes(rig):
    head, client, set_ready = rig
    handle = wait(client.send_goal_async(goal(2.0, 0.0)))
    assert wait(handle.get_result_async()).result.error.code == ErrorCode.INVALID_GOAL
    set_ready(False)
    time.sleep(0.3)
    handle = wait(client.send_goal_async(goal(0.1, 0.0)))
    result = wait(handle.get_result_async()).result
    assert result.error.code == ErrorCode.NOT_READY
    assert "not_enabled" in result.error.message
    assert head.commands == []
