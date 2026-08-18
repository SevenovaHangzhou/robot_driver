#!/usr/bin/env python3

import copy
import unittest
from pathlib import Path

import yaml

from tools import driver_variant
from tools import driver_variant_projections as projections


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "src"
    / "rt_control"
    / "rt_control_bringup"
    / "config"
    / "driver_variant.yaml"
)
ROBOT_DESCRIPTION = "src/description/robot_description/urdf/robot.urdf.xacro"


class DriverVariantStructuredProjectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = driver_variant.load_manifest(MANIFEST, repository_root=ROOT)
        cls.sources = projections.load_structured_sources(cls.document, ROOT)

    def assert_invalid(self, source_path, mutated_text, message):
        sources = dict(self.sources)
        sources[source_path] = mutated_text
        with self.assertRaisesRegex(driver_variant.DriverVariantError, message):
            projections.validate_structured_projections(self.document, sources)

    def test_current_repository_matches_the_manifest(self):
        projections.validate_structured_projections(self.document, self.sources)

    def test_robot_model_active_joint_set_must_match_whole_body_topology(self):
        source = (ROOT / ROBOT_DESCRIPTION).read_text(encoding="utf-8")
        sources = dict(self.sources)
        sources[ROBOT_DESCRIPTION] = source.replace(
            '<joint name="right_joint1" type="revolute">',
            '<joint name="renamed_right_joint1" type="revolute">',
            1,
        )
        with self.assertRaisesRegex(
            driver_variant.DriverVariantError,
            "robot_description.*active joints.*whole-body manifest",
        ):
            projections.validate_structured_projections(self.document, sources)

    def test_robot_model_joint_types_bind_manifest_si_units(self):
        document = copy.deepcopy(self.document)
        transmission = next(
            item
            for item in document["transmissions"]
            if item["id"] == "zeroerr_rad_pos"
        )
        transmission["si_unit"] = "m"
        with self.assertRaisesRegex(
            driver_variant.DriverVariantError,
            "right_joint1.*revolute.*SI unit 'rad'.*got 'm'",
        ):
            projections.validate_structured_projections(document, self.sources)

    def test_robot_model_must_remain_statically_enumerable(self):
        source = self.sources[projections.ROBOT_DESCRIPTION]
        dynamic = source.replace(
            '<robot xmlns:xacro="http://wiki.ros.org/xacro"',
            '<robot xmlns:xacro="http://wiki.ros.org/xacro" '
            'xmlns:xc="http://www.ros.org/wiki/xacro"',
            1,
        ).replace(
            "</robot>",
            '<xc:macro name="injected" params="">'
            '<joint name="injected_joint" type="revolute"/>'
            "</xc:macro><xc:injected/></robot>",
            1,
        )
        self.assert_invalid(
            projections.ROBOT_DESCRIPTION,
            dynamic,
            "robot_description.*must remain statically enumerable.*xacro",
        )
        nested = source.replace(
            "</robot>",
            '<group><joint name="injected_joint" type="revolute"/></group></robot>',
            1,
        )
        self.assert_invalid(
            projections.ROBOT_DESCRIPTION,
            nested,
            "robot_description.*active joints.*direct robot children",
        )
        for attribute, replacement in (
            (
                '<joint name="right_joint1"',
                '<joint name="${\'right_joint1\'}"',
            ),
            ('type="revolute"', 'type="${\'revolute\'}"'),
        ):
            with self.subTest(attribute=attribute):
                self.assert_invalid(
                    projections.ROBOT_DESCRIPTION,
                    source.replace(attribute, replacement, 1),
                    "robot_description.*active joint.*name/type.*literal",
                )

    def test_missing_projection_file_is_rejected(self):
        sources = dict(self.sources)
        del sources[projections.ECAT_XACRO]

        with self.assertRaisesRegex(
            driver_variant.DriverVariantError, "ecat.ros2_control.xacro.*missing"
        ):
            projections.validate_structured_projections(self.document, sources)

    def test_ethercat_xacro_joint_ring_profile_and_sensor_drift_is_rejected(self):
        source = self.sources[projections.ECAT_XACRO]
        mutations = [
            (
                source.replace('ring_position="1"', 'ring_position="16"', 1),
                "right_joint1.*ring position",
            ),
            (
                source.replace('profile="zeroerr_j1"', 'profile="ti5_j2"', 1),
                "right_joint1.*profile",
            ),
            (
                source.replace(
                    '<sensor name="ethercat_slave_1">',
                    '<sensor name="ethercat_slave_16">',
                    1,
                ),
                "EtherCAT sensor positions",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(projections.ECAT_XACRO, mutated, message)

    def test_controllers_joint_order_tolerances_and_track_lists_are_rejected(self):
        source = yaml.safe_load(self.sources[projections.CONTROLLERS])

        reordered = copy.deepcopy(source)
        joints = reordered["whole_body_jtc"]["ros__parameters"]["joints"]
        joints[0], joints[1] = joints[1], joints[0]
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(reordered, sort_keys=False),
            "whole_body_jtc.*joint order",
        )

        tolerance = copy.deepcopy(source)
        tolerance["whole_body_jtc"]["ros__parameters"][
            "trajectory_start_consistency_check"
        ]["position_tolerances"][0] = 0.5
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(tolerance, sort_keys=False),
            "position_tolerances",
        )

        track = copy.deepcopy(source)
        track["diff_drive_controller"]["ros__parameters"]["left_wheel_names"] = [
            "right_track_joint"
        ]
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(track, sort_keys=False),
            "left_wheel_names",
        )

    def test_enable_manager_disable_terminal_policy_must_match_manifest(self):
        source = yaml.safe_load(self.sources[projections.CONTROLLERS])
        missing = copy.deepcopy(source)
        del missing["enable_manager"]["ros__parameters"][
            "ready_to_switch_on_disable_terminal_joints"
        ]
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(missing, sort_keys=False),
            "ready_to_switch_on_disable_terminal_joints.*missing",
        )

        wrong = copy.deepcopy(source)
        wrong["enable_manager"]["ros__parameters"][
            "ready_to_switch_on_disable_terminal_joints"
        ] = ["right_joint1", "right_joint2"]
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(wrong, sort_keys=False),
            "ready_to_switch_on_disable_terminal_joints.*manifest",
        )

    def test_canopen_bus_mapping_mode_master_and_scales_are_rejected(self):
        source = yaml.safe_load(self.sources[projections.CANOPEN_BUS])
        mutations = []

        master = copy.deepcopy(source)
        master["master"]["node_id"] = 101
        mutations.append((master, "CANopen master node"))

        node = copy.deepcopy(source)
        node["nodes"]["left_track_joint"]["node_id"] = 4
        mutations.append((node, "left_track_joint.*node_id"))

        mode = copy.deepcopy(source)
        mode["nodes"]["left_track_joint"]["operation_mode"] = 1
        mutations.append((mode, "left_track_joint.*operation_mode"))

        factor = copy.deepcopy(source)
        factor["nodes"]["left_track_joint"]["scale_vel_to_dev"] *= -1
        mutations.append((factor, "left_track_joint.*velocity.*to_device"))

        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(
                    projections.CANOPEN_BUS,
                    yaml.safe_dump(mutated, sort_keys=False),
                    message,
                )

    def test_canopen_xacro_mapping_and_mode_are_rejected(self):
        source = self.sources[projections.CANOPEN_XACRO]
        self.assert_invalid(
            projections.CANOPEN_XACRO,
            source.replace("<param name=\"node_id\">2</param>", "<param name=\"node_id\">4</param>", 1),
            "left_track_joint.*node_id",
        )
        self.assert_invalid(
            projections.CANOPEN_XACRO,
            source.replace(
                "<param name=\"operation_mode\">3</param>",
                "<param name=\"operation_mode\">1</param>",
                1,
            ),
            "left_track_joint.*operation_mode",
        )

    def test_mock_joint_sensor_and_responder_drift_is_rejected(self):
        source = self.sources[projections.MOCK_XACRO]
        mutations = [
            (
                source.replace('joint_name="right_joint1"', 'joint_name="wrong_joint"', 1),
                "mock EtherCAT joint order",
            ),
            (
                source.replace(
                    '<joint name="left_track_joint">',
                    '<joint name="wrong_track_joint">',
                    1,
                ),
                "mock CANopen joints",
            ),
            (
                source.replace('initial_value">16.0', 'initial_value">15.0', 1),
                "slaves_responding",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(projections.MOCK_XACRO, mutated, message)

    def test_profile_factor_and_duplicate_interface_channels_are_rejected(self):
        profile_path = (
            "src/rt_control/robot_hw_ethercat/config/slaves/ti5_right_joint2.yaml"
        )
        source = yaml.safe_load(self.sources[profile_path])

        factor = copy.deepcopy(source)
        position_command = next(
            channel
            for pdo in factor["rpdo"]
            for channel in pdo["channels"]
            if channel.get("command_interface") == "position"
        )
        position_command["factor"] *= -1
        self.assert_invalid(
            profile_path,
            yaml.safe_dump(factor, sort_keys=False),
            "ti5_right_joint2.*position.*to_device",
        )

        duplicate = copy.deepcopy(source)
        duplicate["rpdo"][0]["channels"].append(
            copy.deepcopy(duplicate["rpdo"][0]["channels"][-1])
        )
        self.assert_invalid(
            profile_path,
            yaml.safe_dump(duplicate, sort_keys=False),
            "ti5_right_joint2.*position.*exactly one command channel",
        )

    def test_profile_declared_interface_multisets_are_exact(self):
        profile_path = (
            "src/rt_control/robot_hw_ethercat/config/slaves/zeroerr_j1.yaml"
        )
        source = yaml.safe_load(self.sources[profile_path])
        mutations = []
        for pdo_name, interface_key, interface in (
            ("rpdo", "command_interface", "control_word"),
            ("rpdo", "command_interface", "digital_outputs"),
            ("tpdo", "state_interface", "status_word"),
            ("tpdo", "state_interface", "digital_inputs"),
        ):
            missing = copy.deepcopy(source)
            missing[pdo_name][0]["channels"] = [
                channel
                for channel in missing[pdo_name][0]["channels"]
                if channel.get(interface_key) != interface
            ]
            mutations.append((missing, pdo_name))

        duplicate = copy.deepcopy(source)
        digital_output = next(
            channel
            for channel in duplicate["rpdo"][0]["channels"]
            if channel.get("command_interface") == "digital_outputs"
        )
        duplicate["rpdo"][0]["channels"].append(copy.deepcopy(digital_output))
        mutations.append((duplicate, "rpdo"))

        for mutated, pdo_name in mutations:
            with self.subTest(pdo_name=pdo_name):
                direction = "command" if pdo_name == "rpdo" else "state"
                self.assert_invalid(
                    profile_path,
                    yaml.safe_dump(mutated, sort_keys=False),
                    rf"zeroerr_j1.*{direction} interfaces.*manifest",
                )

    def test_yaml_key_reordering_does_not_create_false_drift(self):
        sources = dict(self.sources)
        for path in (projections.CONTROLLERS, projections.CANOPEN_BUS):
            sources[path] = yaml.safe_dump(
                yaml.safe_load(sources[path]), sort_keys=True
            )

        projections.validate_structured_projections(self.document, sources)


if __name__ == "__main__":
    unittest.main()
