#!/usr/bin/env python3
"""T1: actual controller_manager + GenericSystem, disjoint position/effort claims and clearing.

Run in an isolated ROS domain with ROS_LOCALHOST_ONLY=1 and a writable ROS_LOG_DIR.
This fixture contains no physical hardware plugin, bus or robot-specific joint.
"""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ConfigureController, LoadController, SwitchController
import yaml

ROBOT = """<robot name="gravity_mock">
  <link name="base"/><link name="body"><inertial><mass value="1"/>
    <origin xyz="1 0 0"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
  </inertial></link>
  <joint name="hinge" type="continuous"><parent link="base"/><child link="body"/>
    <axis xyz="0 1 0"/></joint>
  <ros2_control name="mock" type="system"><hardware>
    <plugin>mock_components/GenericSystem</plugin><param name="calculate_dynamics">false</param>
  </hardware><joint name="hinge">
    <command_interface name="position"><param name="initial_value">0</param></command_interface>
    <command_interface name="effort"><param name="initial_value">0</param></command_interface>
    <state_interface name="position"><param name="initial_value">0</param></state_interface>
    <state_interface name="effort"><param name="initial_value">0</param></state_interface>
    <state_interface name="status_word"><param name="initial_value">39</param></state_interface>
    <state_interface name="gravity_command_epoch"><param name="initial_value">0</param></state_interface>
  </joint><sensor name="bus">
    <state_interface name="age_ms"><param name="initial_value">0</param></state_interface>
    <state_interface name="ready"><param name="initial_value">1</param></state_interface>
  </sensor></ros2_control></robot>"""


def main():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        raise RuntimeError("Use an explicitly isolated localhost ROS domain")
    params = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 1000,
                "robot_description": ROBOT,
                "position_hold": {"type": "forward_command_controller/ForwardCommandController"},
                "gravity": {"type": "ctrl_gravity_feedforward/GravityController"},
                "state": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            }
        },
        "position_hold": {"ros__parameters": {"joints": ["hinge"], "interface_name": "position"}},
        "gravity": {
            "ros__parameters": {
                "robot_description": ROBOT,
                "base_link": "base",
                "tip_link": "body",
                "joints": ["hinge"],
                "gravity_in_base": [0.0, 0.0, -9.81],
                "gravity_in_root": True,
                "scale": 0.5,
                "max_effort_nm": [10.0],
                "max_effort_rate_nm_s": [10.0],
                "max_feedback_age_ms": 5.0,
                "feedback_age_interfaces": ["bus/age_ms"],
                "health_interfaces": ["bus/ready"],
                "health_values": [1.0],
            }
        },
    }
    with tempfile.TemporaryDirectory(prefix="gravity-mock-") as folder:
        config = Path(folder) / "controllers.yaml"
        config.write_text(yaml.safe_dump(params))
        with (Path(folder) / "manager.log").open("w+") as log:
            process = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "controller_manager",
                    "ros2_control_node",
                    "--ros-args",
                    "--params-file",
                    str(config),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init()
            node = rclpy.create_node("gravity_mock_verifier")
            observed = []

            def state(message):
                if "hinge" in message.joint_names:
                    value = message.interface_values[message.joint_names.index("hinge")]
                    if "effort" in value.interface_names:
                        observed.append(value.values[value.interface_names.index("effort")])

            subscription = node.create_subscription(
                DynamicJointState, "/dynamic_joint_states", state, 10
            )

            def call(kind, service, request):
                client = node.create_client(kind, "/controller_manager/" + service)
                if not client.wait_for_service(timeout_sec=10):
                    raise RuntimeError("Missing mock service " + service)
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10)
                reply = future.result()
                node.destroy_client(client)
                if reply is None or not reply.ok:
                    raise RuntimeError("Mock service rejected " + service)

            def wait_effort(target):
                observed.clear()
                end = time.monotonic() + 5
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if len(observed) >= 5 and all(abs(v - target) < 1e-6 for v in observed[-5:]):
                        return
                raise AssertionError(f"Expected effort {target}, last values {observed[-5:]}")

            try:
                for name in ("state", "position_hold", "gravity"):
                    call(LoadController, "load_controller", LoadController.Request(name=name))
                    call(
                        ConfigureController,
                        "configure_controller",
                        ConfigureController.Request(name=name),
                    )
                call(
                    SwitchController,
                    "switch_controller",
                    SwitchController.Request(
                        activate_controllers=["state", "position_hold", "gravity"], strictness=2
                    ),
                )
                wait_effort(-4.905)
                call(
                    SwitchController,
                    "switch_controller",
                    SwitchController.Request(deactivate_controllers=["gravity"], strictness=2),
                )
                wait_effort(0.0)
                call(
                    SwitchController,
                    "switch_controller",
                    SwitchController.Request(activate_controllers=["gravity"], strictness=2),
                )
                wait_effort(-4.905)
                print(
                    "PASS: T1 GenericSystem position + gravity activation, deactivation to zero, reactivation"
                )
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read()[-10000:])
                raise
            finally:
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


if __name__ == "__main__":
    main()
