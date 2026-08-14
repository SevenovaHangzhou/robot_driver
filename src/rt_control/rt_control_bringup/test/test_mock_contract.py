# Copyright 2026
#
# ELECTRI-77: mock-mode cross-domain contract launch_testing suite for the
# RT-Control domain (TC-RO-01 positive / TC-RO-02 negative assertions).
#
# AUTHORITATIVE SPEC NOTE
# -----------------------
# docs/cross-domain-interfaces.md is the authoritative implementation view of
# the cross-domain contract (which itself mirrors robot_interfaces
# contract/endpoints.yaml, see doc section 1). The tables below are a MIRROR of
# that document, kept here only so the test is self-contained. If the doc and
# this file disagree, the doc wins and this mirror must be updated:
#   - Section 2  "RT-Control 接收的公共命令"  -> doc lines 24-28 (R-IN-01..05)
#   - Section 3  "RT-Control 发布的公共状态"  -> doc lines 37-45 (R-OUT-01..10)
#   - Section 6  "已删除接口"                 -> doc lines 123-127
#
# Mock degradations (BQ-132): use_mock_hardware:=true has no CANopen bus and
# the bringup defaults start_plc:=false / start_bms:=false, so:
#   - R-OUT-02 /wheel/odom  : CANopen-backed in production; asserted for
#     existence + type only (mock diff_drive still publishes, but rate/content
#     are not contract evidence here).
#   - R-OUT-04 /battery_state : bms_node not started; the name/type is still
#     visible in the graph through the rt_status_adapter subscription, so it is
#     asserted for existence + type only.

import collections
import os
import time
import unittest

# Isolate from the live robot graph (ROS_DOMAIN_ID 0) on the industrial PC.
# Must run before rclpy.init() and before the launch service forks children.
DEFAULT_TEST_DOMAIN_ID = "142"
os.environ.setdefault("ROS_DOMAIN_ID", DEFAULT_TEST_DOMAIN_ID)

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from diagnostic_msgs.msg import DiagnosticArray  # noqa: F401 (documents type)
from robot_rt_control_interfaces.msg import SafetyState, VacuumState
from robot_system_interfaces.msg import DomainReadiness
from sensor_msgs.msg import JointState

# How long we allow the bringup (controller_manager + spawner chain +
# adapters) to converge before the positive assertions run. The chain needs
# roughly 20 s on the target IPC; 120 s keeps CI margins generous.
GRAPH_CONVERGE_TIMEOUT_SEC = 120.0
GRAPH_POLL_PERIOD_SEC = 1.0

# Frequency spot checks: generous tolerance, lower bound only (>= 70 % of the
# contract-nominal rate). Known mock artefact: joint_state_broadcaster asks
# for 100 Hz but the 250 Hz controller_manager rounds it to the nearest
# divisor, 125 Hz, so /joint_states measures ~125 Hz in mock mode.
FREQUENCY_TOLERANCE_RATIO = 0.7
MEASUREMENT_WARMUP_SEC = 2.0
MEASUREMENT_WINDOW_SEC = 10.0

PublicEndpoint = collections.namedtuple(
    "PublicEndpoint",
    ["contract_id", "kind", "name", "ros_type", "doc_line", "degraded_note"],
)

# Section 2 -- public commands RT-Control receives (doc lines 24-28).
PUBLIC_COMMAND_ENDPOINTS = (
    PublicEndpoint("R-IN-01", "topic", "/cmd_vel_safe",
                   "geometry_msgs/msg/Twist", 24, None),
    PublicEndpoint("R-IN-02", "action", "/whole_body_jtc/follow_joint_trajectory",
                   "control_msgs/action/FollowJointTrajectory", 25, None),
    PublicEndpoint("R-IN-03", "service", "/control/set_enabled",
                   "robot_rt_control_interfaces/srv/SetControlEnabled", 26, None),
    PublicEndpoint("R-IN-04", "service", "/vacuum/pump/set_enabled",
                   "robot_rt_control_interfaces/srv/SetPumpEnabled", 27, None),
    PublicEndpoint("R-IN-05", "action", "/vacuum/grip",
                   "robot_rt_control_interfaces/action/VacuumGrip", 28, None),
)

# Section 3 -- public states RT-Control publishes (doc lines 37-45).
PUBLIC_STATE_ENDPOINTS = (
    PublicEndpoint("R-OUT-01", "topic", "/tf",
                   "tf2_msgs/msg/TFMessage", 37, None),
    PublicEndpoint("R-OUT-01S", "topic", "/tf_static",
                   "tf2_msgs/msg/TFMessage", 38, None),
    PublicEndpoint("R-OUT-02", "topic", "/wheel/odom",
                   "nav_msgs/msg/Odometry", 39,
                   "CANopen-backed; existence+type only in mock (BQ-132)"),
    PublicEndpoint("R-OUT-03", "topic", "/joint_states",
                   "sensor_msgs/msg/JointState", 40, None),
    PublicEndpoint("R-OUT-04", "topic", "/battery_state",
                   "sensor_msgs/msg/BatteryState", 41,
                   "start_bms:=false in mock bringup; name/type visible via "
                   "rt_status_adapter subscription only"),
    PublicEndpoint("R-OUT-05", "topic", "/vacuum/state",
                   "robot_rt_control_interfaces/msg/VacuumState", 42, None),
    PublicEndpoint("R-OUT-06", "topic", "/control/safety_state",
                   "robot_rt_control_interfaces/msg/SafetyState", 43, None),
    PublicEndpoint("R-OUT-09", "topic", "/rt_control/readiness",
                   "robot_system_interfaces/msg/DomainReadiness", 44, None),
    PublicEndpoint("R-OUT-10", "topic", "/diagnostics",
                   "diagnostic_msgs/msg/DiagnosticArray", 45, None),
)

# Section 6 -- removed endpoints; their presence in the graph is a contract
# violation (doc lines 123-127).
REMOVED_ENDPOINT_NAMES = (
    "/robot_model/info",           # doc line 123
    "/calibration/info",           # doc line 124
    "/navigation/set_base_motion_inhibit",   # doc line 125
    "/navigation/base_motion_gate_state",    # doc line 126
    "/motion/base_travel_readiness",         # doc line 127
)

# Frequency spot checks actually MEASURED in mock mode (topic -> msg type,
# contract-nominal Hz, doc line). Not measured: /wheel/odom and
# /battery_state (degraded, see header), /tf and /tf_static (event-driven /
# latched), /diagnostics (no rate in section 3).
FREQUENCY_CHECKS = {
    "/joint_states": (JointState, 100.0, 40),
    "/vacuum/state": (VacuumState, 20.0, 42),
    "/control/safety_state": (SafetyState, 10.0, 43),
    "/rt_control/readiness": (DomainReadiness, 1.0, 44),
}

# Best-effort/volatile subscriber matches every publisher QoS in the bringup
# (reliable publishers and the transient-local readiness publisher included).
MEASUREMENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@launch_testing.markers.keep_alive
def generate_test_description():
    bringup_launch = PathJoinSubstitution(
        [FindPackageShare("rt_control_bringup"), "launch", "rt_control.launch.py"]
    )
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    "use_mock_hardware": "true",
                    # start_plc / start_bms stay at their default "false":
                    # no PLC TCP target and no can1 in the mock environment.
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMockContract(unittest.TestCase):
    """TC-RO-01 / TC-RO-02 graph assertions against the mock bringup."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("electri77_contract_probe")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls._wait_for_graph_convergence()

    @classmethod
    def tearDownClass(cls):
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- graph helpers -----------------------------------------------------

    @classmethod
    def _topic_types(cls):
        return {name: set(types)
                for name, types in cls.node.get_topic_names_and_types()}

    @classmethod
    def _service_types(cls):
        return {name: set(types)
                for name, types in cls.node.get_service_names_and_types()}

    @classmethod
    def _action_types(cls):
        return {name: set(types)
                for name, types in get_action_names_and_types(cls.node)}

    @classmethod
    def _graph_lookup(cls, kind):
        return {
            "topic": cls._topic_types,
            "service": cls._service_types,
            "action": cls._action_types,
        }[kind]()

    @classmethod
    def _missing_public_endpoints(cls):
        graphs = {kind: cls._graph_lookup(kind)
                  for kind in ("topic", "service", "action")}
        missing = []
        for endpoint in PUBLIC_COMMAND_ENDPOINTS + PUBLIC_STATE_ENDPOINTS:
            types = graphs[endpoint.kind].get(endpoint.name)
            if types is None or endpoint.ros_type not in types:
                missing.append(endpoint)
        return missing

    @classmethod
    def _wait_for_graph_convergence(cls):
        deadline = time.monotonic() + GRAPH_CONVERGE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if not cls._missing_public_endpoints():
                return
            cls.executor.spin_once(timeout_sec=GRAPH_POLL_PERIOD_SEC)
        # Do not raise here: the individual tests produce the precise report.

    def _assert_endpoints_present(self, endpoints):
        for endpoint in endpoints:
            with self.subTest(contract_id=endpoint.contract_id,
                              name=endpoint.name):
                graph = self._graph_lookup(endpoint.kind)
                self.assertIn(
                    endpoint.name, graph,
                    msg=(
                        f"{endpoint.contract_id}: {endpoint.kind} "
                        f"'{endpoint.name}' missing from graph "
                        f"(cross-domain-interfaces.md line {endpoint.doc_line})"
                    ),
                )
                self.assertEqual(
                    graph[endpoint.name], {endpoint.ros_type},
                    msg=(
                        f"{endpoint.contract_id}: {endpoint.kind} "
                        f"'{endpoint.name}' has types {sorted(graph[endpoint.name])}, "
                        f"contract requires exactly '{endpoint.ros_type}' "
                        f"(cross-domain-interfaces.md line {endpoint.doc_line})"
                    ),
                )

    # -- TC-RO-01: positive contract surface -------------------------------

    def test_tc_ro_01_public_command_endpoints_present_with_types(self):
        """Section 2: all five R-IN endpoints exist with the contract types."""
        self._assert_endpoints_present(PUBLIC_COMMAND_ENDPOINTS)

    def test_tc_ro_01_public_state_endpoints_present_with_types(self):
        """Section 3: all R-OUT endpoints exist with the contract types.

        R-OUT-02 and R-OUT-04 are existence+type only (mock degradations,
        see module header / BQ-132).
        """
        self._assert_endpoints_present(PUBLIC_STATE_ENDPOINTS)

    # -- TC-RO-02: removed endpoints must not exist ------------------------

    def test_tc_ro_02_removed_endpoints_absent(self):
        """Section 6: removed endpoints appearing in the graph is a breach."""
        graph_names = (
            set(self._topic_types())
            | set(self._service_types())
            | set(self._action_types())
        )
        for removed in REMOVED_ENDPOINT_NAMES:
            with self.subTest(removed=removed):
                self.assertNotIn(
                    removed, graph_names,
                    msg=(
                        f"Removed endpoint '{removed}' reappeared in the graph "
                        "(cross-domain-interfaces.md section 6)"
                    ),
                )
                offenders = [n for n in graph_names
                             if n.startswith(removed + "/")]
                self.assertEqual(
                    offenders, [],
                    msg=(
                        f"Removed endpoint '{removed}' has sub-names "
                        f"{offenders} in the graph "
                        "(cross-domain-interfaces.md section 6)"
                    ),
                )

    # -- Frequency spot checks ---------------------------------------------

    def test_publish_frequencies_meet_contract_lower_bound(self):
        """Measured: /joint_states, /vacuum/state, /control/safety_state,
        /rt_control/readiness. Lower bound only: >= 70 % of nominal."""
        counters = {name: 0 for name in FREQUENCY_CHECKS}

        def make_callback(topic_name):
            def callback(_msg):
                counters[topic_name] += 1
            return callback

        subscriptions = [
            self.node.create_subscription(
                msg_type, name, make_callback(name), MEASUREMENT_QOS
            )
            for name, (msg_type, _nominal, _line) in FREQUENCY_CHECKS.items()
        ]
        try:
            self._spin_for(MEASUREMENT_WARMUP_SEC)
            baseline = dict(counters)
            window_start = time.monotonic()
            self._spin_for(MEASUREMENT_WINDOW_SEC)
            elapsed = time.monotonic() - window_start
            for name, (_msg_type, nominal_hz, doc_line) in FREQUENCY_CHECKS.items():
                measured_hz = (counters[name] - baseline[name]) / elapsed
                with self.subTest(topic=name):
                    self.assertGreaterEqual(
                        measured_hz, FREQUENCY_TOLERANCE_RATIO * nominal_hz,
                        msg=(
                            f"'{name}' measured {measured_hz:.2f} Hz over "
                            f"{elapsed:.1f} s; contract nominal {nominal_hz} Hz "
                            f"(>= {FREQUENCY_TOLERANCE_RATIO * nominal_hz:.1f} Hz "
                            f"required, cross-domain-interfaces.md line {doc_line})"
                        ),
                    )
        finally:
            for subscription in subscriptions:
                self.node.destroy_subscription(subscription)

    def _spin_for(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
