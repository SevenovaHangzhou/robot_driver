"""ROS 2 process: publishes arbitrated drive commands and serves the web UI."""

from __future__ import annotations

import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState, JointState, Range
from std_msgs.msg import String

import robot_interfaces_qos

from .config import ConfigError, OperatorWebConfig, build_config, load_token
from .robot_model import ModelError, build_top_view, package_resolver
from .teleop import PUBLISH_RATE_HZ, DriveArbiter

REQUIRED_PARAMETERS = {
    "bind_host": Parameter.Type.STRING,
    "port": Parameter.Type.INTEGER,
    "command_topic": Parameter.Type.STRING,
    "controller_node": Parameter.Type.STRING,
    "joint_states_topic": Parameter.Type.STRING,
    "battery_topic": Parameter.Type.STRING,
    "gear_names": Parameter.Type.STRING_ARRAY,
    "gear_linear_speeds": Parameter.Type.DOUBLE_ARRAY,
    "gear_angular_speeds": Parameter.Type.DOUBLE_ARRAY,
    "linear_limits": Parameter.Type.DOUBLE_ARRAY,
    "angular_limits": Parameter.Type.DOUBLE_ARRAY,
    "ultrasonic_topics": Parameter.Type.STRING_ARRAY,
    "ultrasonic_layout": Parameter.Type.STRING_ARRAY,
    "ultrasonic_layout_confirmed": Parameter.Type.BOOL,
    "robot_description_topic": Parameter.Type.STRING,
    "chassis_root_link": Parameter.Type.STRING,
}
# robot_state_publisher latches the URDF with these settings.
ROBOT_DESCRIPTION_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# Controller module positions further than this from the URDF steering axes are flagged.
MODEL_MISMATCH_WARN_M = 0.01
# Display-only parking-radar bands in metres (ELECTRI-109 UI decision).
RADAR_BANDS_M = (0.3, 0.6, 1.0, 1.5, 2.5)
GEOMETRY_PARAMETERS = ("steering_joints", "drive_joints", "module_x", "module_y", "wheel_radius")
GEOMETRY_RETRY_S = 2.0
SHUTDOWN_ZERO_COUNT = 3


def _finite_or_none(value: float) -> Optional[float]:
    return float(value) if math.isfinite(value) else None


def _age(now: float, stamp: float) -> Optional[float]:
    return None if not math.isfinite(stamp) else round(now - stamp, 2)


class OperatorWebNode(Node):
    def __init__(self) -> None:
        super().__init__("rt_control_operator_web")
        for name, kind in REQUIRED_PARAMETERS.items():
            self.declare_parameter(name, kind)
        self.declare_parameter("static_dir", "")
        self.config = self._read_config()
        self.arbiter = DriveArbiter(self.config.gears, self.config.limits, time.monotonic)
        self._lock = threading.Lock()
        self._battery: Optional[BatteryState] = None
        self._battery_received = -math.inf
        self._joints: dict[str, tuple[float, float]] = {}
        self._joints_received = -math.inf
        self._diagnostic: Optional[tuple[int, str]] = None
        self._diagnostic_received = -math.inf
        self._ranges: list[Optional[tuple[float, float, float]]] = [None] * len(
            self.config.ultrasonic
        )
        self._geometry: Optional[dict[str, list]] = None
        self._geometry_pending = False
        self._urdf: Optional[str] = None
        self._model: Optional[dict[str, Any]] = None
        self._model_error = "waiting for robot_description"
        self._model_key: Optional[tuple] = None
        self._model_thread: Optional[threading.Thread] = None

        controller = self.config.controller_node.rstrip("/")
        self._publisher = self.create_publisher(
            Twist, self.config.command_topic, robot_interfaces_qos.control()
        )
        self.create_subscription(
            BatteryState, self.config.battery_topic, self._on_battery, robot_interfaces_qos.state()
        )
        self.create_subscription(
            JointState,
            self.config.joint_states_topic,
            self._on_joints,
            robot_interfaces_qos.fast_state(),
        )
        self.create_subscription(
            DiagnosticArray,
            f"{controller}/diagnostics",
            self._on_diagnostics,
            robot_interfaces_qos.diagnostic(),
        )
        for index, channel in enumerate(self.config.ultrasonic):
            self.create_subscription(
                Range,
                channel.topic,
                lambda message, i=index: self._on_range(i, message),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            String, self.config.robot_description_topic, self._on_description, ROBOT_DESCRIPTION_QOS
        )
        self._parameter_client = self.create_client(GetParameters, f"{controller}/get_parameters")
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)
        self.create_timer(GEOMETRY_RETRY_S, self._request_geometry)
        self.get_logger().info(
            f"operator web publishing drive commands to {self.config.command_topic} "
            "(ELECTRI-109 V1: Mock-only target)"
        )

    # --- configuration -----------------------------------------------------
    def _read_config(self) -> OperatorWebConfig:
        values: dict[str, Any] = {}
        for name in REQUIRED_PARAMETERS:
            try:
                values[name] = self.get_parameter(name).value
            except ParameterUninitializedException as exc:
                raise ConfigError(f"required parameter '{name}' is not set") from exc
        return build_config(
            bind_host=values["bind_host"],
            port=values["port"],
            command_topic=values["command_topic"],
            controller_node=values["controller_node"],
            joint_states_topic=values["joint_states_topic"],
            battery_topic=values["battery_topic"],
            gear_names=list(values["gear_names"]),
            gear_linear_speeds=list(values["gear_linear_speeds"]),
            gear_angular_speeds=list(values["gear_angular_speeds"]),
            linear_limits=list(values["linear_limits"]),
            angular_limits=list(values["angular_limits"]),
            ultrasonic_topics=list(values["ultrasonic_topics"]),
            ultrasonic_layout=list(values["ultrasonic_layout"]),
            ultrasonic_layout_confirmed=values["ultrasonic_layout_confirmed"],
            robot_description_topic=values["robot_description_topic"],
            chassis_root_link=values["chassis_root_link"],
        )

    def static_dir(self) -> Path:
        override = self.get_parameter("static_dir").value
        if override:
            return Path(override)
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("rt_control_operator_web")) / "web"

    def static_info(self) -> dict[str, Any]:
        return {
            "ultrasonic": {
                "channels": [
                    {"corner": c.corner, "facing": c.facing} for c in self.config.ultrasonic
                ],
                "layout_confirmed": self.config.ultrasonic_layout_confirmed,
                "bands_m": list(RADAR_BANDS_M),
            },
        }

    # --- publishing ----------------------------------------------------------
    def _publish(self, command: tuple[float, float, float]) -> None:
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        self._publisher.publish(message)

    def _on_timer(self) -> None:
        command = self.arbiter.tick()
        if command is not None:
            self._publish(command)

    def publish_shutdown_zeros(self) -> None:
        # The process is exiting: a ramp cannot be completed, so the final
        # zeros hand over to the controller's own stop behaviour.
        self.arbiter.soft_stop("process_shutdown")
        for _ in range(SHUTDOWN_ZERO_COUNT):
            self._publish((0.0, 0.0, 0.0))

    # --- controller geometry (read, never duplicated) ------------------------
    def _request_geometry(self) -> None:
        with self._lock:
            if self._geometry is not None or self._geometry_pending:
                return
            if not self._parameter_client.service_is_ready():
                return
            self._geometry_pending = True
        request = GetParameters.Request(names=list(GEOMETRY_PARAMETERS))
        self._parameter_client.call_async(request).add_done_callback(self._on_geometry)

    def _on_geometry(self, future) -> None:
        geometry: Optional[dict[str, list]] = None
        try:
            values = future.result().values
            if len(values) == len(GEOMETRY_PARAMETERS):
                parsed = {
                    name: list(value.string_array_value or value.double_array_value)
                    for name, value in zip(GEOMETRY_PARAMETERS, values)
                }
                sizes = {len(v) for v in parsed.values()}
                numbers = parsed["module_x"] + parsed["module_y"] + parsed["wheel_radius"]
                if sizes == {4} and all(math.isfinite(v) for v in numbers):
                    geometry = parsed
        except Exception as exc:  # noqa: BLE001 - reported, then retried
            self.get_logger().warning(f"controller geometry unavailable: {exc}")
        with self._lock:
            self._geometry = geometry
            self._geometry_pending = False
        self._maybe_build_model()

    # --- shared Robot Model top view ---------------------------------------------
    def _on_description(self, message: String) -> None:
        with self._lock:
            self._urdf = message.data
        self._maybe_build_model()

    def _maybe_build_model(self) -> None:
        """Rebuild the top view off the executor thread when its inputs change.

        Mesh projection takes a noticeable fraction of a second; running it in
        a ROS callback would stall the 50 Hz command publisher.
        """
        with self._lock:
            urdf, geometry = self._urdf, self._geometry
            if urdf is None or geometry is None:
                return
            key = (hash(urdf), tuple(geometry["steering_joints"]))
            busy = self._model_thread is not None and self._model_thread.is_alive()
            if key == self._model_key or busy:
                return
            self._model_key = key
            steering = list(geometry["steering_joints"])
        thread = threading.Thread(
            target=self._build_model, args=(urdf, steering), name="top-view-model", daemon=True
        )
        with self._lock:
            self._model_thread = thread
        thread.start()

    def _build_model(self, urdf: str, steering: list[str]) -> None:
        from ament_index_python.packages import get_package_share_directory

        model: Optional[dict[str, Any]] = None
        error = ""
        try:
            model = build_top_view(
                urdf,
                package_resolver(get_package_share_directory),
                self.config.chassis_root_link,
                steering,
            ).to_message()
            if model["missing_meshes"]:
                self.get_logger().warning(f"top view: missing meshes {model['missing_meshes']}")
        except (ModelError, OSError, ValueError) as exc:
            error = str(exc)
            self.get_logger().warning(f"top view unavailable: {error}")
        with self._lock:
            self._model = model
            self._model_error = error

    def model_message(self) -> Optional[dict[str, Any]]:
        with self._lock:
            return None if self._model is None else {"type": "model", **self._model}

    # --- subscriptions --------------------------------------------------------
    def _on_battery(self, message: BatteryState) -> None:
        with self._lock:
            self._battery = message
            self._battery_received = time.monotonic()

    def _on_joints(self, message: JointState) -> None:
        now = time.monotonic()
        with self._lock:
            for i, name in enumerate(message.name):
                position = message.position[i] if i < len(message.position) else math.nan
                velocity = message.velocity[i] if i < len(message.velocity) else math.nan
                self._joints[name] = (position, velocity)
            self._joints_received = now

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        if not message.status:
            return
        status = message.status[0]
        level = status.level[0] if isinstance(status.level, (bytes, bytearray)) else status.level
        with self._lock:
            self._diagnostic = (int(level), status.message)
            self._diagnostic_received = time.monotonic()

    def _on_range(self, index: int, message: Range) -> None:
        with self._lock:
            self._ranges[index] = (
                float(message.range),
                float(message.max_range),
                time.monotonic(),
            )

    # --- status for the web ---------------------------------------------------
    def robot_status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            battery = self._battery
            battery_received = self._battery_received
            joints = dict(self._joints)
            joints_received = self._joints_received
            diagnostic = self._diagnostic
            diagnostic_received = self._diagnostic_received
            ranges = list(self._ranges)
            geometry = self._geometry
            model = self._model
            model_error = self._model_error
        status: dict[str, Any] = {
            "controller_connected": self._publisher.get_subscription_count() > 0,
            "battery": None,
            "controller": None,
            "geometry": None,
            "wheels": None,
            "ultrasonic": [],
        }
        if battery is not None:
            status["battery"] = {
                "percentage": _finite_or_none(battery.percentage),
                "voltage": _finite_or_none(battery.voltage),
                "present": bool(battery.present),
                "age_s": _age(now, battery_received),
            }
        if diagnostic is not None:
            status["controller"] = {
                "level": diagnostic[0],
                "state": diagnostic[1],
                "age_s": _age(now, diagnostic_received),
            }
        if geometry is not None:
            status["geometry"] = {
                "module_x": geometry["module_x"],
                "module_y": geometry["module_y"],
                "wheel_radius": geometry["wheel_radius"],
                "steering_joints": geometry["steering_joints"],
            }
            wheels = []
            for steer, drive in zip(geometry["steering_joints"], geometry["drive_joints"]):
                angle = joints.get(steer, (math.nan, math.nan))[0]
                speed = joints.get(drive, (math.nan, math.nan))[1]
                wheels.append({"angle": _finite_or_none(angle), "speed": _finite_or_none(speed)})
            status["wheels"] = {"modules": wheels, "age_s": _age(now, joints_received)}
        status["model"] = {"version": model["version"] if model else None, "error": model_error}
        if model and geometry is not None:
            deviation = max(
                math.hypot(m["x"] - x, m["y"] - y)
                for m, x, y in zip(model["modules"], geometry["module_x"], geometry["module_y"])
            )
            status["model"]["controller_mismatch_m"] = round(deviation, 4)
            status["model"]["controller_mismatch"] = deviation > MODEL_MISMATCH_WARN_M
        for entry in ranges:
            if entry is None:
                status["ultrasonic"].append(None)
            else:
                distance, max_range, received = entry
                status["ultrasonic"].append(
                    {
                        "range": _finite_or_none(distance),
                        "max_range": _finite_or_none(max_range),
                        "age_s": _age(now, received),
                    }
                )
        return status


def main(argv: Optional[list[str]] = None) -> int:
    from aiohttp import web

    from .server import OperatorServer

    # aiohttp owns SIGINT/SIGTERM so the ROS context stays valid long enough to
    # publish the final zero commands during shutdown.
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[OperatorWebNode] = None
    executor = SingleThreadedExecutor()
    spin_thread: Optional[threading.Thread] = None
    try:
        try:
            node = OperatorWebNode()
            token = load_token()
            server = OperatorServer(
                node.arbiter,
                token,
                node.robot_status,
                node.static_dir(),
                node.static_info(),
                model_provider=node.model_message,
            )
        except (ConfigError, ValueError) as exc:
            print(f"rt_control_operator_web: configuration error: {exc}", file=sys.stderr)
            return 2
        executor.add_node(node)

        def spin() -> None:
            try:
                executor.spin()
            except ExternalShutdownException:
                pass

        spin_thread = threading.Thread(target=spin, name="ros-executor", daemon=True)
        spin_thread.start()
        web.run_app(
            server.build_app(),
            host=node.config.bind_host,
            port=node.config.port,
            print=None,
            access_log=None,
        )
        return 0
    finally:
        # A second Ctrl-C (terminal plus launch forwarding) must not abort cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.publish_shutdown_zeros()
        executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
