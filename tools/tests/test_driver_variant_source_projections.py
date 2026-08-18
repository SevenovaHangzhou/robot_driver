#!/usr/bin/env python3

import ast
import copy
import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from tools import driver_variant
from tools import driver_variant_source_projections as source_projections


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "src"
    / "rt_control"
    / "rt_control_bringup"
    / "config"
    / "driver_variant.yaml"
)


class DriverVariantSourceProjectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = driver_variant.load_manifest(MANIFEST, repository_root=ROOT)
        cls.sources = source_projections.load_source_projection_sources(ROOT)

    def assert_invalid(self, path, text, message):
        sources = dict(self.sources)
        sources[path] = text
        with self.assertRaisesRegex(driver_variant.DriverVariantError, message):
            source_projections.validate_source_projections(self.document, sources)

    def test_current_repository_matches_source_projections(self):
        source_projections.validate_source_projections(self.document, self.sources)

    def test_repository_projection_entrypoint_checks_all_projection_layers(self):
        document = source_projections.validate_repository_projections(ROOT)
        self.assertEqual(document["variant_id"], self.document["variant_id"])

    def test_repository_projection_entrypoint_aggregates_both_projection_layers(self):
        with (
            mock.patch.object(
                source_projections.driver_variant_projections,
                "validate_structured_projections",
                side_effect=driver_variant.DriverVariantError("structured drift"),
            ),
            mock.patch.object(
                source_projections,
                "validate_source_projections",
                side_effect=driver_variant.DriverVariantError("source drift"),
            ),
        ):
            with self.assertRaises(driver_variant.DriverVariantError) as raised:
                source_projections.validate_repository_projections(ROOT)
        self.assertIn("structured drift", raised.exception.findings)
        self.assertIn("source drift", raised.exception.findings)

    def test_cli_reports_success_and_missing_manifest_failure(self):
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            self.assertEqual(
                source_projections.main(["--repository-root", str(ROOT)]), 0
            )
        self.assertIn("PASS: driver variant projections", stdout.getvalue())

        with tempfile.TemporaryDirectory() as temporary_directory:
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(
                    source_projections.main(
                        ["--repository-root", temporary_directory]
                    ),
                    1,
                )
            self.assertIn("cannot read manifest", stderr.getvalue())

    def test_missing_source_projection_is_rejected(self):
        sources = dict(self.sources)
        del sources[source_projections.RT_DIAGNOSTICS]
        with self.assertRaisesRegex(
            driver_variant.DriverVariantError, "rt_diagnostics_node.cpp.*missing"
        ):
            source_projections.validate_source_projections(self.document, sources)

    def test_status_adapter_requires_literal_ordered_unique_ids(self):
        source = self.sources[source_projections.STATUS_ADAPTER]
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            source.replace(
                "ETHERCAT_DRIVE_POSITIONS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15)",
                "ETHERCAT_DRIVE_POSITIONS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 14)",
                1,
            ),
            "ETHERCAT_DRIVE_POSITIONS",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            source.replace(
                "CANOPEN_NODE_IDS = (2, 3)",
                "CANOPEN_NODE_IDS = tuple(range(2, 4))",
                1,
            ),
            "CANOPEN_NODE_IDS.*literal",
        )

    def test_recovery_axis_checker_tracks_joint_and_disable_terminal_policy(self):
        source = self.sources[source_projections.AXIS_STATE_CHECKER]
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            source.replace(
                '"right_joint1",\n    "right_joint2",',
                '"right_joint2",\n    "right_joint1",',
                1,
            ),
            "AXES.*manifest",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            source.replace(
                "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES = (\n"
                '    "right_joint2", "right_joint3", "left_joint2", "left_joint3"\n'
                ")",
                "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES = (\n"
                '    "right_joint1", "right_joint3", "left_joint2", "left_joint3"\n'
                ")",
                1,
            ),
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*manifest",
        )

    def test_python_constants_must_be_top_level_single_assignments(self):
        source = self.sources[source_projections.STATUS_ADAPTER]
        tree = ast.parse(source)
        assignment = next(
            node
            for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "CANOPEN_NODE_IDS"
                for target in node.targets
            )
        )
        segment = ast.get_source_segment(source, assignment)
        assert segment is not None
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            source + "\n" + segment + "\n",
            "CANOPEN_NODE_IDS.*exactly once",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            source
            + "\nif True:\n"
            + '    CANOPEN_NODE_NAMES = ("/robot/rt_control/canopen/node_99",)\n',
            "CANOPEN_NODE_IDS.*runtime name projection",
        )
        for target, projected_from, replacement in (
            (
                "ETHERCAT_SLAVE_NAMES",
                "ETHERCAT_DRIVE_POSITIONS",
                '    ETHERCAT_SLAVE_NAMES = ("/robot/rt_control/ethercat/slave_1",)',
            ),
            (
                "CANOPEN_NODE_NAMES",
                "CANOPEN_NODE_IDS",
                '    CANOPEN_NODE_NAMES = ("/robot/rt_control/canopen/node_2",)',
            ),
        ):
            with self.subTest(target=target):
                self.assert_invalid(
                    source_projections.STATUS_ADAPTER,
                    source
                    + f"\ndef override_{target.lower()}():\n"
                    + f"    global {target}\n"
                    + replacement
                    + f"\noverride_{target.lower()}()\n",
                    f"{projected_from}.*runtime name projection",
                )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            source.replace(
                "from rclpy.callback_groups import MutuallyExclusiveCallbackGroup",
                "from rclpy.callback_groups import "
                "ReentrantCallbackGroup as MutuallyExclusiveCallbackGroup",
                1,
            ),
            "MutuallyExclusiveCallbackGroup.*without alias",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            source.replace(
                "from rclpy.callback_groups import MutuallyExclusiveCallbackGroup",
                "from rclpy.callback_groups import MutuallyExclusiveCallbackGroup\n"
                "    MutuallyExclusiveCallbackGroup = ReentrantCallbackGroup",
                1,
            ),
            "MutuallyExclusiveCallbackGroup.*must not be rebound",
        )

    def test_status_and_recovery_constants_must_feed_runtime_consumers(self):
        status = self.sources[source_projections.STATUS_ADAPTER]
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "for node_id in CANOPEN_NODE_IDS",
                "for node_id in (2, 3)",
                1,
            ),
            "CANOPEN_NODE_IDS.*runtime name projection",
        )

        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "self._diagnostic_state = update_diagnostic_state(",
                "discarded_state = update_diagnostic_state(",
                1,
            ),
            "runtime diagnostic callback",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "                self._diagnostic_state, message.status, time.monotonic()\n"
                "            )",
                "                self._diagnostic_state, message.status, time.monotonic()\n"
                "            )\n"
                "            self._diagnostic_state = DiagnosticState(\n"
                "                DiagnosticTopology(\n"
                "                    ETHERCAT_SLAVE_NAMES, CANOPEN_NODE_NAMES\n"
                "                ),\n"
                "                self._diagnostic_state.components,\n"
                "            )",
                1,
            ),
            "runtime diagnostic callback",
        )
        for managed_names, static_names in (
            (
                "diagnostic_state.topology.ethercat_slaves",
                "ETHERCAT_SLAVE_NAMES",
            ),
            ("diagnostic_state.topology.canopen_nodes", "CANOPEN_NODE_NAMES"),
        ):
            self.assert_invalid(
                source_projections.STATUS_ADAPTER,
                status.replace(managed_names, static_names, 1),
                "runtime safety summary",
            )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "diagnostic_state = self._diagnostic_state",
                "diagnostic_state = DiagnosticState(DiagnosticTopology(), {})",
                1,
            ),
            "runtime safety summary",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "        def _build_summary(self) -> SafetySummary:\n",
                "        def _build_summary(self) -> SafetySummary:\n"
                "            def build_safety_summary(**_kwargs):\n"
                "                return None\n",
                1,
            ),
            "runtime safety summary",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "diagnostic_state = self._diagnostic_state",
                "diagnostic_state = self._diagnostic_state\n"
                "            if True:\n"
                "                diagnostic_state = DiagnosticState(\n"
                "                    DiagnosticTopology(\n"
                "                        ETHERCAT_SLAVE_NAMES, CANOPEN_NODE_NAMES\n"
                "                    ),\n"
                "                    diagnostic_state.components,\n"
                "                )",
                1,
            ),
            "runtime safety summary",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "ENABLE_MANAGER_NAME, diagnostic_state.components",
                "ENABLE_MANAGER_NAME, self._diagnostic_state.components",
                1,
            ),
            "runtime safety summary",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "MutuallyExclusiveCallbackGroup()", "ReentrantCallbackGroup()", 1
            ),
            "serialize diagnostic snapshots",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "self._callback_group = MutuallyExclusiveCallbackGroup()",
                "self._callback_group = MutuallyExclusiveCallbackGroup()\n"
                '            setattr(self, "_callback_group", ReentrantCallbackGroup())',
                1,
            ),
            "serialize diagnostic snapshots",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status
            + "\ndef update_diagnostic_state(current, statuses, received):\n"
            + "    return current\n",
            "update_diagnostic_state.*top level exactly once",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status
            + "\nupdate_diagnostic_state = "
            + "lambda current, statuses, received: current\n",
            "update_diagnostic_state.*must not be rebound",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status + "\nbuild_safety_summary = lambda **kwargs: None\n",
            "build_safety_summary.*must not be rebound",
        )
        self.assert_invalid(
            source_projections.STATUS_ADAPTER,
            status.replace(
                "self._callback_group = MutuallyExclusiveCallbackGroup()",
                "self._callback_group = MutuallyExclusiveCallbackGroup()\n"
                "            if True:\n"
                "                self._callback_group = ReentrantCallbackGroup()",
                1,
            ),
            "serialize diagnostic snapshots",
        )

        checker = self.sources[source_projections.AXIS_STATE_CHECKER]
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "joint in READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES",
                'joint in ("right_joint2", "right_joint3", "left_joint2", "left_joint3")',
            ),
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*runtime policy",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "for joint in AXES:",
                "for joint in (\n"
                '        "right_joint1", "right_joint2", "right_joint3",\n'
                '        "right_joint4", "right_joint5", "right_joint6",\n'
                '        "left_joint1", "left_joint2", "left_joint3",\n'
                '        "left_joint4", "left_joint5", "left_joint6",\n'
                '        "turn", "updown",\n'
                "    ):",
                1,
            ),
            "AXES.*runtime extraction",
        )

    def test_status_name_projection_rejects_discarded_source_references(self):
        status = self.sources[source_projections.STATUS_ADAPTER]
        mutations = (
            (
                status.replace(
                    "for position in ETHERCAT_DRIVE_POSITIONS",
                    "for position in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15)",
                    1,
                )
                .replace(
                    ")\nCANOPEN_NODE_NAMES = tuple(",
                    ") if ETHERCAT_DRIVE_POSITIONS else ()\n"
                    "CANOPEN_NODE_NAMES = tuple(",
                    1,
                ),
                "ETHERCAT_DRIVE_POSITIONS.*directly drive",
            ),
            (
                status.replace(
                    "for node_id in CANOPEN_NODE_IDS",
                    "for node_id in (2, 3)",
                    1,
                ).replace(
                    ")\nENABLE_MANAGER_NAME =",
                    ") if CANOPEN_NODE_IDS else ()\nENABLE_MANAGER_NAME =",
                    1,
                ),
                "CANOPEN_NODE_IDS.*directly drive",
            ),
        )
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(
                    source_projections.STATUS_ADAPTER, mutated, message
                )

    def test_axis_checker_rejects_dead_projection_references(self):
        checker = self.sources[source_projections.AXIS_STATE_CHECKER]
        hardcoded_axes = (
            "(\n"
            '        "right_joint1", "right_joint2", "right_joint3",\n'
            '        "right_joint4", "right_joint5", "right_joint6",\n'
            '        "left_joint1", "left_joint2", "left_joint3",\n'
            '        "left_joint4", "left_joint5", "left_joint6",\n'
            '        "turn", "updown",\n'
            "    )"
        )
        extraction = checker.replace(
            "def _extract_status_words(document: object) -> dict[str, int]:\n",
            "def _extract_status_words(document: object) -> dict[str, int]:\n"
            "    projection_reference_only = AXES\n",
            1,
        ).replace("for joint in AXES:", f"for joint in {hardcoded_axes}:", 1)
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            extraction,
            "AXES.*directly drive.*runtime extraction",
        )

        post_extraction_filter = checker.replace(
            "        observed[joint] = status_word\n    return observed",
            "        observed[joint] = status_word\n"
            "    observed = {\n"
            "        joint: observed[joint]\n"
            "        for joint in (\n"
            '            "right_joint1", "right_joint2", "right_joint3",\n'
            '            "right_joint4", "right_joint5", "right_joint6",\n'
            '            "left_joint1", "left_joint2", "left_joint3",\n'
            '            "left_joint4", "left_joint5", "left_joint6",\n'
            '            "turn", "updown",\n'
            "        )\n"
            "    }\n"
            "    return observed",
            1,
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            post_extraction_filter,
            "AXES.*directly drive.*runtime extraction",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "    for joint in AXES:",
                "    return observed\n    for joint in AXES:",
                1,
            ),
            "AXES.*directly drive.*runtime extraction",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "    for joint in AXES:\n",
                "    for joint in AXES:\n        continue\n",
                1,
            ),
            "AXES.*directly drive.*runtime extraction",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "        observed[joint] = status_word",
                "        observed[joint] = status_word\n"
                "        observed[joint] = 0x0040",
                1,
            ),
            "AXES.*directly drive.*runtime extraction",
        )

        policy_axes = (
            '("right_joint2", "right_joint3", "left_joint2", "left_joint3")'
        )
        policy = checker.replace(
            "def check_axis_states(document: object, expected: str) -> dict[str, int]:\n",
            "def check_axis_states(document: object, expected: str) -> dict[str, int]:\n"
            "    projection_reference_only = READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES\n",
            1,
        ).replace(
            "joint in READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES",
            f"joint in {policy_axes}",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            policy,
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*directly drive.*runtime policy",
        )

        masked_policy = checker.replace(
            "            joint in READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES\n"
            "            and status_word & 0x006F == 0x0021",
            "            ((joint in READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES and False)\n"
            "            or joint in (\n"
            '                "right_joint2", "right_joint3",\n'
            '                "left_joint2", "left_joint3",\n'
            "            ))\n"
            "            and status_word & 0x006F == 0x0021",
            1,
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            masked_policy,
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*directly drive.*runtime policy",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "        switch_on_disabled =",
                "        continue\n        switch_on_disabled =",
                1,
            ),
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*directly drive.*runtime policy",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace('if expected == "enabled":', "if True:", 1),
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*directly drive.*runtime policy",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                "switch_on_disabled = status_word & 0x004F == 0x0040",
                "switch_on_disabled = True",
                1,
            ),
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*directly drive.*runtime policy",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker.replace(
                ")\n\n\nclass AxisStateError",
                ")\nglobals()[\"READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES\"] = AXES\n"
                "\n\nclass AxisStateError",
                1,
            ),
            "READY_TO_SWITCH_ON_DISABLE_TERMINAL_AXES.*top level exactly once",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker
            + "\ndef override_axes():\n"
            + "    global AXES\n"
            + '    AXES = ("right_joint1",)\n'
            + "override_axes()\n",
            "AXES.*top level exactly once",
        )
        self.assert_invalid(
            source_projections.AXIS_STATE_CHECKER,
            checker + '\nvars()["AXES"] = ("right_joint1",)\n',
            "AXES.*top level exactly once",
        )

    def test_enable_manager_marker_block_tracks_joint_and_batch_order(self):
        source = self.sources[source_projections.ENABLE_MANAGER_HEADER]
        self.assert_invalid(
            source_projections.ENABLE_MANAGER_HEADER,
            source.replace('"right_joint1"', '"wrong_joint"', 1),
            "enable_manager_topology.*drift",
        )
        self.assert_invalid(
            source_projections.ENABLE_MANAGER_HEADER,
            source.replace("{{{0, 1, 2}", "{{{0, 2, 1}", 1),
            "enable_manager_topology.*drift",
        )

    def test_cpp_marker_is_unique_and_exact(self):
        source = self.sources[source_projections.RT_DIAGNOSTICS]
        begin = "// DRIVER_VARIANT_PROJECTION_BEGIN:rt_diagnostics_topology"
        self.assert_invalid(
            source_projections.RT_DIAGNOSTICS,
            source.replace(begin, "", 1),
            "rt_diagnostics_topology.*marker",
        )
        self.assert_invalid(
            source_projections.RT_DIAGNOSTICS,
            source + "\n" + begin + "\n",
            "rt_diagnostics_topology.*marker",
        )

    def test_rt_diagnostics_node_and_responder_drift_is_rejected(self):
        source = self.sources[source_projections.RT_DIAGNOSTICS]
        self.assert_invalid(
            source_projections.RT_DIAGNOSTICS,
            source.replace("{2U, 3U}", "{2U, 4U}", 1),
            "rt_diagnostics_topology.*drift",
        )
        self.assert_invalid(
            source_projections.RT_DIAGNOSTICS,
            source.replace(
                "kExpectedEthercatResponders = 16U",
                "kExpectedEthercatResponders = 15U",
                1,
            ),
            "rt_diagnostics_topology.*drift",
        )

    def test_rt_diagnostics_rejects_legacy_contiguous_node_consumers(self):
        source = self.sources[source_projections.RT_DIAGNOSTICS]
        self.assert_invalid(
            source_projections.RT_DIAGNOSTICS,
            source.replace(
                "const std::uint8_t node_id = kCanopenNodeIds[node];",
                "const std::uint8_t node_id = static_cast<std::uint8_t>(node + 2U);",
                1,
            ),
            "legacy hard-coded consumer",
        )

    def test_rt_diagnostics_comments_do_not_count_as_topology_consumers(self):
        source = self.sources[source_projections.RT_DIAGNOSTICS]
        marker_end = (
            "// DRIVER_VARIANT_PROJECTION_END:rt_diagnostics_topology"
        )
        marker, consumers = source.split(marker_end, 1)
        for symbol, replacement in (
            ("kRingPositions", "legacyRingPositions"),
            ("kJointNames", "legacyJointNames"),
            ("kCanopenNodeIds", "legacyCanopenNodeIds"),
            ("kExpectedEthercatResponders", "16U"),
        ):
            for dead_reference in (
                f"// dead projection reference: {symbol}",
                f"/* dead projection reference: {symbol} */",
            ):
                with self.subTest(symbol=symbol, comment=dead_reference[:2]):
                    mutated_consumers = consumers.replace(symbol, replacement)
                    mutated = (
                        marker
                        + marker_end
                        + mutated_consumers
                        + f"\n{dead_reference}\n"
                    )
                    self.assert_invalid(
                        source_projections.RT_DIAGNOSTICS,
                        mutated,
                        f"{symbol}.*runtime consumer",
                    )

    def test_enable_manager_rejects_historical_ti5_index_consumer(self):
        source = self.sources[source_projections.ENABLE_MANAGER_CPP]
        mutated = source.replace(
            "return axis < kAxisCount && ready_to_switch_on_disable_terminal_[axis] &&",
            "const bool is_ti5_axis = axis == 1U || axis == 2U || axis == 7U || axis == 8U;\n"
            "  return is_ti5_axis &&",
            1,
        )
        self.assert_invalid(
            source_projections.ENABLE_MANAGER_CPP,
            mutated,
            "legacy hard-coded disable-terminal consumer",
        )

    def test_enable_manager_topology_tables_must_feed_cpp_runtime_paths(self):
        source = self.sources[source_projections.ENABLE_MANAGER_CPP]
        for symbol, replacement in (
            ("kJointNames", "legacyJointNames"),
            ("kEnableBatches", "legacyEnableBatches"),
            ("kBatchSizes", "legacyBatchSizes"),
        ):
            with self.subTest(symbol=symbol):
                mutated = (
                    source.replace(symbol, replacement)
                    + f"\n// dead projection reference: {symbol}\n"
                )
                self.assert_invalid(
                    source_projections.ENABLE_MANAGER_CPP,
                    mutated,
                    f"{symbol}.*runtime consumer",
                )

    def test_ros2_canopen_safety_node_table_is_anchored_in_added_lines(self):
        source = self.sources[source_projections.ROS2_CANOPEN_PATCH]
        self.assert_invalid(
            source_projections.ROS2_CANOPEN_PATCH,
            source.replace("{2U, 3U}", "{2U, 4U}", 1),
            "ros2_canopen_track_nodes.*drift",
        )
        begin = "+// DRIVER_VARIANT_PROJECTION_BEGIN:ros2_canopen_track_nodes"
        self.assert_invalid(
            source_projections.ROS2_CANOPEN_PATCH,
            source.replace(begin, begin[1:], 1),
            "ros2_canopen_track_nodes.*added patch lines",
        )

    def test_ros2_canopen_safety_table_is_canonical_by_node_id(self):
        reversed_document = copy.deepcopy(self.document)
        reversed_document["buses"]["canopen"]["nodes"].reverse()

        self.assertEqual(
            source_projections.render_ros2_canopen_track_nodes(reversed_document),
            source_projections.render_ros2_canopen_track_nodes(self.document),
        )

    def test_ros2_canopen_table_covers_node_mode_uniqueness_and_all_consumers(self):
        source = self.sources[source_projections.ROS2_CANOPEN_PATCH]
        mutations = [
            (
                source.replace("{3U, 3U}", "{4U, 3U}", 1),
                "ros2_canopen_track_nodes.*drift",
            ),
            (
                source.replace("{3U, 3U}", "{2U, 3U}", 1),
                "ros2_canopen_track_nodes.*drift",
            ),
            (
                source.replace("{2U, 3U}", "{2U, 1U}", 1),
                "ros2_canopen_track_nodes.*drift",
            ),
            (
                source.replace(
                    "!isConfiguredTrackNode(id)",
                    "(id != 2U && id != 3U)",
                    1,
                ),
                "legacy hard-coded consumer",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(
                    source_projections.ROS2_CANOPEN_PATCH, mutated, message
                )

    def test_ros2_canopen_table_directly_controls_mode_and_emcy_safety_paths(self):
        source = self.sources[source_projections.ROS2_CANOPEN_PATCH]
        mutations = (
            source.replace(
                "static_cast<uint32_t>(configuration.operation_mode)", "1U", 1
            ).replace(
                "driver->set_operation_mode(configuration.operation_mode)",
                "driver->set_operation_mode(1U)",
                1,
            ),
            source.replace(
                "emcy.eec == 0U || !isConfiguredTrackNode(id)",
                "emcy.eec == 0U || (true || !isConfiguredTrackNode(id))",
                1,
            ),
            source.replace(
                "return std::any_of(", "return false && std::any_of(", 1
            ),
        )
        for mutated in mutations:
            with self.subTest():
                self.assert_invalid(
                    source_projections.ROS2_CANOPEN_PATCH,
                    mutated,
                    "track-node table must directly control runtime mode and EMCY safety",
                )


if __name__ == "__main__":
    unittest.main()
