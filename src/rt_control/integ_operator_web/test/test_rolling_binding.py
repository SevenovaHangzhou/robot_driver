"""Thin binding transport test on an isolated local ROS domain; no hardware."""

import importlib.util
import os
import time
import threading


def test_binding_uses_public_sender_for_epoch_sequence_ack_and_final():
    path = os.environ.get("JOG_SENDER_EXTENSION")
    assert path, "Run the standard package CTest target to build and load RollingSender"
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1"
    assert os.environ.get("ROS_DOMAIN_ID") == "219"
    spec = importlib.util.spec_from_file_location("_rolling_sender", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import rclpy
    from robot_motion_interfaces.msg import RollingJointTargetBatch
    from robot_rt_control_interfaces.msg import RollingJointControlState
    import robot_interfaces_qos

    assert module.Sender.MAX_POINTS == RollingJointTargetBatch.MAX_POINTS
    assert module.Sender.HOLDING == RollingJointControlState.HOLDING
    assert module.Sender.STATE_MAX_AGE_S > 0
    rclpy.init()
    node = rclpy.create_node("rolling_binding_transport_peer")
    state = RollingJointControlState(epoch=17, state=0)
    publisher = node.create_publisher(
        RollingJointControlState, "/rt/arms/rolling/state", robot_interfaces_qos.rolling_state()
    )
    received = []

    def receive(batch):
        received.append(batch)
        state.last_sequence, state.last_result, state.state = batch.sequence, 0, 1

    node.create_subscription(
        RollingJointTargetBatch,
        "/rt/arms/rolling/target_batch",
        receive,
        robot_interfaces_qos.rolling_command(),
    )
    node.create_timer(0.01, lambda: publisher.publish(state))
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    sender = module.Sender(200)
    window = {
        "replace_from": {"sec": 10, "nanosec": 20},
        "final": False,
        "points": [
            {
                "time_from_start": {"sec": 0, "nanosec": t},
                "positions": [0.0] * 14,
                "velocities": [0.0] * 14,
            }
            for t in (0, 100_000_000)
        ],
    }

    def until(predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise AssertionError("local ROS transport did not converge")

    try:
        until(lambda: sender.send(window) == int(module.SendResult.SENT))
        until(lambda: sender.status()["phase"] == int(module.Phase.STREAMING))
        assert received[0].epoch == 17 and received[0].sequence > 0
        assert received[0].replace_from.sec == 10 and received[0].replace_from.nanosec == 20
        assert list(received[0].points[0].positions) == [0.0] * 14
        window["final"] = True
        assert sender.send(window) == int(module.SendResult.SENT)
        until(lambda: len(received) == 2)
        assert received[-1].final and received[-1].sequence == received[0].sequence + 1
        state.state, state.stop_reason = 0, 6
        until(lambda: sender.status()["phase"] == int(module.Phase.COMPLETED))
        assert sender.reset()
    finally:
        sender.close()
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
