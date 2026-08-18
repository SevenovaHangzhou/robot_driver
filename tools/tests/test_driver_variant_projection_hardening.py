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


class DriverVariantProjectionHardeningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = driver_variant.load_manifest(MANIFEST, repository_root=ROOT)
        cls.sources = projections.load_structured_sources(cls.document, ROOT)

    def assert_invalid(self, path, text, message, document=None):
        sources = dict(self.sources)
        sources[path] = text
        with self.assertRaisesRegex(driver_variant.DriverVariantError, message):
            projections.validate_structured_projections(
                self.document if document is None else document, sources
            )

    def test_ethercat_invocation_must_use_profile_family_macro(self):
        source = self.sources[projections.ECAT_XACRO]
        self.assert_invalid(
            projections.ECAT_XACRO,
            source.replace(
                '<xacro:ti5_axis joint_name="right_joint2"',
                '<xacro:zeroerr_axis joint_name="right_joint2"',
                1,
            ),
            "right_joint2.*macro family",
        )

    def test_ethercat_macro_must_forward_ring_profile_and_joint_parameters(self):
        source = self.sources[projections.ECAT_XACRO]
        mutations = [
            (
                source.replace(
                    '<joint name="${joint_name}">', '<joint name="fixed_joint">', 1
                ),
                "zeroerr_axis.*joint_name.*forward",
            ),
            (
                source.replace(
                    '<param name="position">${ring_position}</param>',
                    '<param name="position">999</param>',
                    1,
                ),
                "zeroerr_axis.*ring_position.*forward",
            ),
            (
                source.replace(
                    "$(arg config_dir)/slaves/${profile}.yaml",
                    "$(arg config_dir)/slaves/wrong.yaml",
                    1,
                ),
                "zeroerr_axis.*profile.*forward",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(projections.ECAT_XACRO, mutated, message)

    def test_xacro_config_directory_arguments_are_bound_to_runtime_assets(self):
        ethercat = self.sources[projections.ECAT_XACRO]
        self.assert_invalid(
            projections.ECAT_XACRO,
            ethercat.replace(
                'name="config_dir" default="$(find robot_hw_ethercat)/config"',
                'name="config_dir" default="$(find wrong_package)/config"',
                1,
            ),
            "config_dir.*default",
        )

        canopen = self.sources[projections.CANOPEN_XACRO]
        self.assert_invalid(
            projections.CANOPEN_XACRO,
            canopen.replace(
                'name="canopen_config_dir" default="$(find robot_hw_canopen)/config"',
                'name="canopen_config_dir" default="$(find wrong_package)/config"',
                1,
            ),
            "canopen_config_dir.*default",
        )

    def test_ethercat_macro_is_unique_and_manifest_config_matches_runtime_path(self):
        source = self.sources[projections.ECAT_XACRO]
        duplicate = source.replace(
            "</robot>",
            '<xacro:macro name="zeroerr_axis" params="joint_name ring_position profile"/>'
            "\n</robot>",
            1,
        )
        self.assert_invalid(
            projections.ECAT_XACRO, duplicate, "zeroerr_axis.*exactly once"
        )

        document = copy.deepcopy(self.document)
        profile = next(
            item for item in document["drive_profiles"] if item["id"] == "zeroerr_j1"
        )
        profile["config"] = (
            "src/rt_control/robot_hw_ethercat/config/slaves/left_j6.yaml"
        )
        with self.assertRaisesRegex(
            driver_variant.DriverVariantError, "zeroerr_j1.*runtime profile path"
        ):
            projections.validate_structured_projections(document, self.sources)

    def test_xacro_namespace_and_direct_tree_ownership_are_strict(self):
        source = self.sources[projections.ECAT_XACRO]
        self.assert_invalid(
            projections.ECAT_XACRO,
            source.replace(
                "http://www.ros.org/wiki/xacro", "urn:not-xacro", 1
            ),
            "canonical xacro namespace",
        )
        nested = source.replace(
            '<ros2_control name="ecat_arms"',
            '<xacro:group><ros2_control name="ecat_arms"',
            1,
        ).replace("</ros2_control>\n</robot>", "</ros2_control></xacro:group>\n</robot>", 1)
        self.assert_invalid(
            projections.ECAT_XACRO, nested, "direct child.*ecat_arms"
        )

        alternate_prefix = source.replace("xmlns:xacro=", "xmlns:xc=").replace(
            "xacro:", "xc:"
        )
        sources = dict(self.sources)
        sources[projections.ECAT_XACRO] = alternate_prefix
        projections.validate_structured_projections(self.document, sources)

    def test_ethercat_readiness_interfaces_and_declared_resources_are_exact(self):
        source = self.sources[projections.ECAT_XACRO]
        mutations = [
            (
                source.replace(
                    '<state_interface name="al_state"/>',
                    '<state_interface name="wrong_state"/>',
                    1,
                ),
                "ethercat_slave_1.*al_state",
            ),
            (
                source.replace(
                    '<state_interface name="link_up"/>',
                    '<state_interface name="wrong_link"/>',
                    1,
                ),
                "ethercat_master.*interfaces",
            ),
            (
                source.replace(
                    "</ros2_control>\n</robot>",
                    '<joint name="rogue_joint"/>\n  </ros2_control>\n</robot>',
                    1,
                ),
                "undeclared raw joint",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(projections.ECAT_XACRO, mutated, message)

    def test_controller_safety_switches_and_interfaces_are_checked(self):
        source = yaml.safe_load(self.sources[projections.CONTROLLERS])
        disabled = copy.deepcopy(source)
        disabled["whole_body_jtc"]["ros__parameters"][
            "trajectory_start_consistency_check"
        ]["enabled"] = False
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(disabled, sort_keys=False),
            "trajectory_start_consistency_check.enabled",
        )

        interface = copy.deepcopy(source)
        interface["whole_body_jtc"]["ros__parameters"]["command_interfaces"] = [
            "velocity"
        ]
        self.assert_invalid(
            projections.CONTROLLERS,
            yaml.safe_dump(interface, sort_keys=False),
            "whole_body_jtc.*command_interfaces",
        )

    def test_canopen_xacro_hardware_wiring_is_checked(self):
        source = self.sources[projections.CANOPEN_XACRO]
        mutations = [
            (
                source.replace(
                    "$(arg canopen_config_dir)/bus.yml",
                    "$(arg canopen_config_dir)/wrong.yml",
                    1,
                ),
                "bus_config.*bus.yml",
            ),
            (
                source.replace(
                    "$(arg can_interface_name)", "can9", 1
                ),
                "can_interface_name.*argument",
            ),
            (
                source.replace(
                    "<plugin>canopen_ros2_control/Cia402System</plugin>",
                    "<plugin>wrong/Plugin</plugin>",
                    1,
                ),
                "Cia402System",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(projections.CANOPEN_XACRO, mutated, message)

    def test_canopen_effective_dcf_honors_node_override(self):
        source = yaml.safe_load(self.sources[projections.CANOPEN_BUS])
        source["nodes"]["left_track_joint"]["dcf"] = "eds/wrong.eds"
        self.assert_invalid(
            projections.CANOPEN_BUS,
            yaml.safe_dump(source, sort_keys=False),
            "left_track_joint.*effective dcf",
        )

    def test_canopen_keyed_node_reordering_is_not_drift(self):
        source = yaml.safe_load(self.sources[projections.CANOPEN_BUS])
        source["nodes"] = dict(reversed(list(source["nodes"].items())))
        sources = dict(self.sources)
        sources[projections.CANOPEN_BUS] = yaml.safe_dump(source, sort_keys=False)

        projections.validate_structured_projections(self.document, sources)

    def test_mock_rejects_undeclared_raw_joint_and_missing_readiness_interface(self):
        source = self.sources[projections.MOCK_XACRO]
        mutations = [
            (
                source.replace(
                    "</ros2_control>\n\n  <ros2_control name=\"canopen_mobile_axes\"",
                    '<joint name="rogue_joint"/>\n  </ros2_control>\n\n'
                    '  <ros2_control name="canopen_mobile_axes"',
                    1,
                ),
                "mock.*undeclared raw joint",
            ),
            (
                source.replace(
                    '<state_interface name="al_state">',
                    '<state_interface name="wrong_state">',
                    1,
                ),
                "ethercat_slave_1.*al_state",
            ),
        ]
        for mutated, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(projections.MOCK_XACRO, mutated, message)

    def test_mock_axis_macro_must_forward_joint_name(self):
        source = self.sources[projections.MOCK_XACRO]
        self.assert_invalid(
            projections.MOCK_XACRO,
            source.replace(
                '<joint name="${joint_name}">', '<joint name="fixed_joint">', 1
            ),
            "mock_ecat_axis.*joint_name.*forward",
        )

    def test_malformed_xml_and_duplicate_yaml_are_rejected(self):
        self.assert_invalid(
            projections.ECAT_XACRO,
            self.sources[projections.ECAT_XACRO].replace("</robot>", "", 1),
            "invalid XML",
        )
        duplicate = self.sources[projections.CANOPEN_BUS].replace(
            "master:\n", "master:\nmaster:\n", 1
        )
        self.assert_invalid(
            projections.CANOPEN_BUS, duplicate, "duplicate YAML key.*master"
        )


if __name__ == "__main__":
    unittest.main()
