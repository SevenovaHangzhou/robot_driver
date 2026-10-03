"""Cross-process preview only: no device nodes, no motion targets admitted."""

import os
import signal
import subprocess
import sys
import time


def test_preview_discovery_rejects_unconfigured_commands_and_exits(tmp_path, monkeypatch):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from robot_interfaces_qos import latched
    from robot_rt_control_interfaces.action import MoveHead
    from robot_rt_control_interfaces.msg import ModuleStateArray, OutputCommandResult
    from robot_rt_control_interfaces.srv import SetPumpEnabled

    # Domain isolation is for the test graph, not a hardware authorization mechanism.
    domain = 180 + os.getpid() % 40
    # Parent and children must share transport policy as well as domain ID.
    # CI may set ROS_LOCALHOST_ONLY=0; patch this process before initializing RMW.
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    env = os.environ.copy()
    processes = []
    logs = []
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("electri174_preview_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        ModuleStateArray, "/rt_control/modules/state", received.append, latched()
    )
    pump = node.create_client(SetPumpEnabled, "/vacuum/pump/set_enabled")
    head = ActionClient(node, MoveHead, "/head/move")

    def await_condition(predicate, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not predicate():
            for child in processes:
                assert child.poll() is None, diagnostic_logs()
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), diagnostic_logs()

    def diagnostic_logs():
        return "\n".join(path.read_text(errors="replace") for path in logs)

    try:
        for module in ("vacuum_adapter", "module_state_adapter", "position_action_adapter"):
            path = tmp_path / (module + ".log")
            logs.append(path)
            with path.open("w") as output:
                processes.append(subprocess.Popen(
                    [sys.executable, "-c", f"from control_api_adapter.{module} import main; main()"],
                    env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
                ))
        await_condition(lambda: bool(received) and pump.service_is_ready() and head.server_is_ready())
        modules = {item.module_id: item for item in received[-1].modules}
        assert modules["edge_protection"].installation_state == 1
        assert not modules["edge_protection"].enabled
        assert modules["force_control"].implementation_state == 1
        assert modules["battery_exchange"].implementation_state == 1
        assert not modules["vacuum_io"].configuration_valid

        # No physical IO server exists. Pump rejection must happen before any output call.
        request = SetPumpEnabled.Request()
        request.enabled = True
        future = pump.call_async(request)
        await_condition(future.done, seconds=5)
        response = future.result()
        assert not response.accepted
        assert response.result.outcome == OutputCommandResult.OUTCOME_NOT_EXECUTED

        goal_future = head.send_goal_async(MoveHead.Goal())
        await_condition(goal_future.done, seconds=5)
        assert not goal_future.result().accepted
        topics = dict(node.get_topic_names_and_types())
        assert "/head_position_controller/commands" not in topics
    finally:
        # Only processes created by this test are signalled; no global pkill or hardware stop.
        for child in processes:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        results = []
        for child in processes:
            try:
                results.append(child.wait(timeout=5))
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=3)
                results.append(-signal.SIGKILL)
        executor.shutdown()
        head.destroy()
        node.destroy_subscription(subscription)
        node.destroy_node()
        if context.ok():
            context.shutdown()
        assert all(code == 0 for code in results), diagnostic_logs()
