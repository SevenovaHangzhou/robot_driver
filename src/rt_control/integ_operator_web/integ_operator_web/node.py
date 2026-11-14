"""ROS connection of the chassis operator console.

Public interfaces only: the scope runtime's ``/rt/capabilities`` declaration,
``/rt/enable``, ``/rt/disable``, ``/rt/reset_fault`` (FAILED only), controller-manager and
controller parameter services, ``/diagnostics`` and the declared velocity topic. The
console never starts hardware, writes drive words or enables on its own.
"""

from __future__ import annotations

from pathlib import Path
import signal
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import robot_interfaces_qos

from .chassis_connection import ChassisConnection
from .config import build_config, load_token
from .teleop import PUBLISH_RATE_HZ, DriveArbiter
from .tuning import FieldTuning


class OperatorWebNode(Node):
    def __init__(self, *, software_test_only=False):
        super().__init__("integ_operator_web")
        defaults = {
            "bind_host": "127.0.0.1",
            "port": 18120,
            "command_topic": "/rt/base/cmd_vel",
            "controller_node": "/swerve_controller",
            "gear_names": ["leisure", "sport"],
            "gear_linear_speeds": [0.05, 0.15],
            "gear_angular_speeds": [0.1, 0.3],
            "linear_limits": [0.3, 0.5, 1.0, 2.0],
            "angular_limits": [0.6, 1.0, 2.0, 4.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("static_dir", "")
        self.declare_parameter("jog_motion_enabled", False)
        # Deployment artifact of the running scope: steering defaults and the unit name.
        self.declare_parameter("generated_dir", "")
        # Status page: read-only MJPEG preview base URL of the NX camera service ("" hides cameras).
        self.declare_parameter("status_camera_base", "")
        # Status page: label shown next to "机器人状态"; the machine configuration supplies it.
        self.declare_parameter("status_robot_label", "")
        # Status page: log-station base URL for cycle statistics and waveform links ("" disables).
        self.declare_parameter("status_log_url", "")
        # Status page: joint names for the pose readouts, as deployed (empty means not shown).
        for role in (
            "status_joint_updown",
            "status_joint_head_yaw",
            "status_joint_head_pitch",
            "status_joint_steer1",
            "status_joint_steer2",
            "status_joint_steer3",
            "status_joint_steer4",
        ):
            self.declare_parameter(role, "")
        self.config = build_config(**{k: self.get_parameter(k).value for k in defaults})
        generated_dir = self.get_parameter("generated_dir").value
        tuning = None
        if generated_dir:
            tuning = FieldTuning(Path(generated_dir), self.config.controller_node.lstrip("/"))
        self.arbiter = DriveArbiter(self.config.gears, self.config.limits, time.monotonic)
        self.publisher = self.create_publisher(
            Twist, self.config.command_topic, robot_interfaces_qos.control()
        )
        self.backend = ChassisConnection(self, software_test_only, tuning)
        self.create_timer(1 / PUBLISH_RATE_HZ, self.publish_command)

    def publish_zero(self):
        self.publisher.publish(Twist())

    def publish_command(self):
        if not self.backend.allowed():
            self.arbiter.reset_motion("chassis_not_enabled")
            return
        command = self.arbiter.tick()
        if command is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = command
            self.publisher.publish(msg)

    def robot_status(self):
        return {
            "chassis": self.backend.status(),
            "drive_available": self.backend.allowed(),
            "command_subscribers": self.publisher.get_subscription_count(),
        }


def arm_page(arm, sessions, operators, audit, static_dir, *, jog_motion_enabled=False):
    """The arm page and its ROS bridge (own executor thread), sharing login, lease rule and audit."""
    from ament_index_python.packages import get_package_share_directory
    import yaml

    from .arm.console import ArmConsole
    from .arm.console_types import config_from_mapping
    from .arm.page import ArmPage
    from .arm.jog import JogPage
    from .arm.ros_bridge import RosBridge, spin_in_background
    from .arm.vacuum_control import VacuumControl

    config = config_from_mapping(yaml.safe_load(arm["config"].read_text(encoding="utf-8")))
    bridge = RosBridge(arm["joint_state_topic"])
    console = ArmConsole(config, "", bridge, arm["poses"])
    bridge.attach_console(console)
    if jog_motion_enabled:
        from ._rolling_sender import Sender
        from .arm.rolling_output import RollingOutput
        console.jog_output = RollingOutput(console, bridge, Sender(100))
    spin_in_background(bridge)
    page = ArmPage(
        console,
        sessions,
        operators,
        static_dir / "arm",
        get_package_share_directory,
        bridge.set_enabled,
        bridge.reset_fault,
        audit,
        view_model_share=arm["view_model_share"],
        view_model_file=arm["view_model_file"],
        vacuum=VacuumControl(arm["vacuum_script"]) if arm["vacuum_script"].is_file() else None,
    )
    return page, JogPage(page), bridge


def head_page(head, sessions, operators, audit, static_dir, arm_control=None):
    """The head page and its ROS bridge (own executor thread)."""
    import yaml

    from .arm.ros_bridge import spin_in_background
    from .head.console import HeadConsole
    from .head.page import HeadPage
    from .head.ros_bridge import HeadBridge

    bridge = HeadBridge(head["joint_state_topic"])
    console = HeadConsole(
        yaml.safe_load(head["config"].read_text(encoding="utf-8")), bridge.publish
    )
    bridge.console = console
    spin_in_background(bridge)
    page = HeadPage(
        console,
        sessions,
        operators,
        static_dir / "head",
        bridge.set_enabled,
        bridge.reset_fault,
        audit,
        arm_control=arm_control,
    )
    return page, bridge


def status_page(
    node,
    sessions,
    static_dir,
    *,
    camera_base="",
    robot_label="",
    roles=None,
    log_url="",
    model_file=None,
    mesh_lookup=None,
):
    """The read-only status page and its own subscription thread."""
    from .status.feed import Feed, FeedNode
    from .status.logstation import LogStation
    from .status.page import StatusPage

    feed = Feed()
    feed_node = FeedNode(feed)
    executor = SingleThreadedExecutor()
    executor.add_node(feed_node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin, name="operator-status", daemon=True).start()
    page = StatusPage(
        feed,
        sessions,
        static_dir / "status",
        camera_base=camera_base,
        robot_label=robot_label,
        roles=roles,
        log_station=LogStation(log_url),
        model_file=model_file,
        mesh_lookup=mesh_lookup,
    )
    # Polling starts in app start-up: at node start-up there is no running event loop yet.
    return page, feed_node


def main(argv=None, *, software_test_only=False, audit_path=None, arm=None, head=None):
    from aiohttp import web

    from .audit import AuditLog
    from .operators import Operators
    from .server import OperatorServer
    from .session import Sessions

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = OperatorWebNode(software_test_only=software_test_only)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    sessions, operators, audit = Sessions(load_token()), Operators(), AuditLog(audit_path)
    static_dir = Path(node.get_parameter("static_dir").value)
    # The status page reuses the arm page's installed model share for the whole-vehicle view.
    model_file = mesh_lookup = None
    if arm is not None:
        from .arm.model import resolve_mesh
        from ament_index_python.packages import get_package_share_directory

        model_file = arm.get("view_model_file")
        view_share = arm["view_model_share"]
        view_share = view_share.resolve() if view_share is not None else None

        def mesh_lookup(path, share=view_share):
            # The browser URL omits "package://", so the resolver needs the package name only.
            relative = path.lstrip("/")
            found = resolve_mesh(
                relative,
                lambda package: str(share) if share else get_package_share_directory(package),
            )
            if found is None and share is not None:
                candidate = Path(share) / relative
                found = str(candidate) if candidate.is_file() else None
            return found

    status, status_node = status_page(
        node,
        sessions,
        static_dir,
        camera_base=node.get_parameter("status_camera_base").value,
        robot_label=node.get_parameter("status_robot_label").value,
        log_url=node.get_parameter("status_log_url").value,
        roles={
            key: node.get_parameter(f"status_joint_{key}").value
            for key in ("updown", "head_yaw", "head_pitch", "steer1", "steer2", "steer3", "steer4")
        },
        model_file=model_file,
        mesh_lookup=mesh_lookup,
    )
    pages, bridge = (status,), None
    arm_control = None
    if arm is not None:
        page, jog, bridge = arm_page(arm, sessions, operators, audit, static_dir,
            jog_motion_enabled=node.get_parameter("jog_motion_enabled").value)
        arm_control = page.console
        pages = (status, page, jog)
    head_bridge = None
    if head is not None:
        page, head_bridge = head_page(head, sessions, operators, audit, static_dir, arm_control)
        pages = (*pages, page)
    # The status page pulls the read-only camera preview straight from the NX.
    camera_origin = (node.get_parameter("status_camera_base").value or "").rstrip("/")
    server = OperatorServer(
        node.arbiter,
        sessions,
        node.robot_status,
        static_dir,
        module_operation=node.backend.operate,
        drive_allowed=node.backend.allowed,
        steering_operation=node.backend.set_steering,
        steering_defaults=node.backend.restore_steering_defaults,
        audit=audit,
        operators=operators,
        pages=pages,
        arm_control=arm_control,
        sensor_sources=(camera_origin,),
    )

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, name="operator-ros", daemon=True)
    thread.start()
    try:
        web.run_app(
            server.build_app(),
            host=node.config.bind_host,
            port=node.config.port,
            access_log=None,
            print=None,
        )
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.arbiter.reset_motion("process_shutdown")
        for _ in range(3):
            node.publish_zero()
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        if bridge is not None:
            if arm_control.jog_output is not None:
                arm_control.jog_output.close()
            bridge.destroy_node()
        status_node.destroy_node()
        if head_bridge is not None:
            head_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
