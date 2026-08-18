#!/usr/bin/env python3

import copy
import tempfile
import unittest
from pathlib import Path

from tools import driver_variant


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "src"
    / "rt_control"
    / "rt_control_bringup"
    / "config"
    / "driver_variant.yaml"
)


def valid_manifest():
    return {
        "schema_version": 1,
        "variant_id": "test-variant-v1",
        "buses": {
            "ethercat": {
                "master_id": 0,
                "ring": [
                    {"position": 0, "role": "hub"},
                    {"position": 1, "role": "drive", "actuator": "arm_motor"},
                ],
            },
            "canopen": {
                "interface": "can0",
                "master_node_id": 100,
                "nodes": [
                    {
                        "node_id": 2,
                        "actuator": "track_motor",
                        "operation_mode": 3,
                    },
                    {
                        "node_id": 3,
                        "actuator": "right_track_motor",
                        "operation_mode": 3,
                    },
                ],
            },
        },
        "drive_profiles": [
            {
                "id": "ti5_j2",
                "family": "ti5",
                "bus": "ethercat",
                "config": "src/rt_control/robot_hw_ethercat/config/slaves/ti5_j2.yaml",
                "command_interfaces": ["position", "control_word"],
                "state_interfaces": ["position", "status_word"],
            },
            {
                "id": "track_profile",
                "family": "leadshine_ld2",
                "bus": "canopen",
                "config": "src/rt_control/robot_hw_canopen/config/eds/ld2_drive.eds",
                "command_interfaces": ["velocity"],
                "state_interfaces": ["position", "velocity"],
            },
        ],
        "transmissions": [
            {
                "id": "arm_transmission",
                "si_unit": "rad",
                "effective_conversion": {
                    "position": {"to_device": 2.0, "from_device": 0.5}
                },
                "reciprocity_rel_tol": 1.0e-9,
            },
            {
                "id": "track_transmission",
                "si_unit": "m",
                "effective_conversion": {
                    "position": {"to_device": -4.0, "from_device": -0.25},
                    "velocity": {"to_device": -4.0, "from_device": -0.25},
                },
                "reciprocity_rel_tol": 1.0e-9,
            },
        ],
        "actuators": [
            {
                "id": "arm_motor",
                "drive_profile": "ti5_j2",
                "transmission": "arm_transmission",
            },
            {
                "id": "track_motor",
                "drive_profile": "track_profile",
                "transmission": "track_transmission",
            },
            {
                "id": "right_track_motor",
                "drive_profile": "track_profile",
                "transmission": "track_transmission",
            },
        ],
        "joints": [
            {
                "name": "arm_joint",
                "actuator": "arm_motor",
                "controller_role": "whole_body_trajectory",
                "position_tolerance": 0.01,
                "lifecycle_policy": "cia402_default",
            },
            {
                "name": "left_track_joint",
                "actuator": "track_motor",
                "controller_role": "left_track",
            },
            {
                "name": "right_track_joint",
                "actuator": "right_track_motor",
                "controller_role": "right_track",
            },
        ],
        "enable": {
            "batches": [{"id": "arm", "joints": ["arm_joint"]}],
            "lifecycle_policies": [
                {
                    "id": "cia402_default",
                    "disable_terminals": ["switch_on_disabled"],
                    "fault_reset": "explicit_only",
                    "source": "CiA402",
                }
            ],
        },
    }


class DriverVariantManifestTest(unittest.TestCase):
    def assert_invalid(self, document, message):
        with self.assertRaisesRegex(driver_variant.DriverVariantError, message):
            driver_variant.validate_manifest(document)

    def test_valid_generic_variant_is_accepted(self):
        driver_variant.validate_manifest(valid_manifest())

    def test_unknown_fields_are_rejected_fail_closed(self):
        document = valid_manifest()
        document["vendor_id"] = "must-stay-in-drive-profile"

        self.assert_invalid(document, "unexpected field.*vendor_id")

        nested = valid_manifest()
        nested["buses"]["ethercat"]["hot_reload"] = True
        self.assert_invalid(nested, "buses.ethercat.*unexpected field.*hot_reload")

    def test_required_fields_types_and_schema_version_are_rejected(self):
        missing = valid_manifest()
        missing.pop("enable")
        self.assert_invalid(missing, "required field.*enable.*missing")

        wrong_type = valid_manifest()
        wrong_type["joints"] = "right_joint1"
        self.assert_invalid(wrong_type, "joints.*must be a list")

        unsupported = valid_manifest()
        unsupported["schema_version"] = 2
        self.assert_invalid(unsupported, "schema_version.*only version 1")

        non_string_key = valid_manifest()
        non_string_key[1] = "invalid"
        self.assert_invalid(non_string_key, "field names must be strings")

        invalid_master = valid_manifest()
        invalid_master["buses"]["canopen"]["master_node_id"] = 2
        self.assert_invalid(invalid_master, "master_node_id.*must not reuse.*node_id")

        for invalid_mode in (-1, 0, 2, 6, 32, 33, 65535, 65536):
            with self.subTest(invalid_mode=invalid_mode):
                document = valid_manifest()
                document["buses"]["canopen"]["nodes"][0][
                    "operation_mode"
                ] = invalid_mode
                self.assert_invalid(
                    document, "operation_mode.*must be 3.*Profiled Velocity"
                )

    def test_duplicate_identifiers_and_bus_addresses_are_rejected(self):
        mutations = []

        duplicate_joint = valid_manifest()
        duplicate_joint["joints"].append(copy.deepcopy(duplicate_joint["joints"][0]))
        mutations.append((duplicate_joint, "duplicate joint.*arm_joint"))

        duplicate_profile = valid_manifest()
        duplicate_profile["drive_profiles"].append(
            copy.deepcopy(duplicate_profile["drive_profiles"][0])
        )
        mutations.append((duplicate_profile, "duplicate drive profile.*ti5_j2"))

        duplicate_ring = valid_manifest()
        duplicate_ring["buses"]["ethercat"]["ring"].append(
            {"position": 1, "role": "hub"}
        )
        mutations.append((duplicate_ring, "duplicate EtherCAT ring position.*1"))

        duplicate_node = valid_manifest()
        duplicate_node["buses"]["canopen"]["nodes"].append(
            {
                "node_id": 2,
                "actuator": "track_motor",
                "operation_mode": 3,
            }
        )
        mutations.append((duplicate_node, "duplicate CANopen node_id.*2"))

        for document, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(document, message)

    def test_unknown_references_are_rejected(self):
        mutations = []

        unknown_profile = valid_manifest()
        unknown_profile["actuators"][0]["drive_profile"] = "missing_profile"
        mutations.append((unknown_profile, "unknown drive profile.*missing_profile"))

        unknown_transmission = valid_manifest()
        unknown_transmission["actuators"][0]["transmission"] = "missing_transmission"
        mutations.append(
            (unknown_transmission, "unknown transmission.*missing_transmission")
        )

        unknown_actuator = valid_manifest()
        unknown_actuator["joints"][0]["actuator"] = "missing_actuator"
        mutations.append((unknown_actuator, "unknown actuator.*missing_actuator"))

        unknown_policy = valid_manifest()
        unknown_policy["joints"][0]["lifecycle_policy"] = "missing_policy"
        mutations.append((unknown_policy, "unknown lifecycle policy.*missing_policy"))

        unknown_batch_joint = valid_manifest()
        unknown_batch_joint["enable"]["batches"][0]["joints"] = ["missing_joint"]
        mutations.append((unknown_batch_joint, "unknown joint.*missing_joint"))

        for document, message in mutations:
            with self.subTest(message=message):
                self.assert_invalid(document, message)

    def test_profile_interfaces_and_conversion_channels_are_consistent(self):
        unknown_interface = valid_manifest()
        unknown_interface["drive_profiles"][0]["command_interfaces"].append(
            "unbounded_vendor_interface"
        )
        self.assert_invalid(unknown_interface, "unexpected interface")

        duplicate_interface = valid_manifest()
        duplicate_interface["drive_profiles"][0]["state_interfaces"].append("position")
        self.assert_invalid(duplicate_interface, "duplicate item.*position")

        missing_conversion = valid_manifest()
        del missing_conversion["transmissions"][1]["effective_conversion"]["velocity"]
        self.assert_invalid(
            missing_conversion,
            "track_motor.*transmission.*missing conversion.*velocity",
        )

        for wrong_command in ("status_word", "digital_inputs"):
            with self.subTest(wrong_command=wrong_command):
                reversed_direction = valid_manifest()
                reversed_direction["drive_profiles"][0][
                    "command_interfaces"
                ].append(wrong_command)
                self.assert_invalid(
                    reversed_direction,
                    rf"command_interfaces.*{wrong_command}.*not a command interface",
                )

        for wrong_state in ("control_word", "digital_outputs"):
            with self.subTest(wrong_state=wrong_state):
                reversed_direction = valid_manifest()
                reversed_direction["drive_profiles"][0]["state_interfaces"].append(
                    wrong_state
                )
                self.assert_invalid(
                    reversed_direction,
                    rf"state_interfaces.*{wrong_state}.*not a state interface",
                )

    def test_ethercat_profile_config_is_bound_to_the_runtime_profile_path(self):
        decoy = valid_manifest()
        decoy["drive_profiles"][0]["config"] = (
            "src/rt_control/robot_hw_ethercat/config/decoy/ti5_j2.yaml"
        )
        self.assert_invalid(
            decoy,
            "config.*must equal.*config/slaves/ti5_j2.yaml",
        )

    def test_controller_roles_bind_bus_and_directional_motion_interfaces(self):
        crossed_buses = valid_manifest()
        crossed_buses["joints"][0]["actuator"] = "track_motor"
        crossed_buses["joints"][1]["actuator"] = "arm_motor"
        self.assert_invalid(
            crossed_buses,
            "whole_body_trajectory.*arm_joint.*requires.*ethercat",
        )
        self.assert_invalid(
            crossed_buses,
            "left_track.*left_track_joint.*requires.*canopen",
        )

        wrong_whole_body_direction = valid_manifest()
        wrong_whole_body_direction["drive_profiles"][0]["command_interfaces"] = [
            "control_word"
        ]
        self.assert_invalid(
            wrong_whole_body_direction,
            "whole_body_trajectory.*command motion interfaces.*position",
        )

        wrong_track_direction = valid_manifest()
        wrong_track_direction["drive_profiles"][1]["command_interfaces"] = [
            "position"
        ]
        wrong_track_direction["drive_profiles"][1]["state_interfaces"] = [
            "velocity"
        ]
        self.assert_invalid(
            wrong_track_direction,
            "left_track.*command motion interfaces.*velocity",
        )
        self.assert_invalid(
            wrong_track_direction,
            "left_track.*state motion interfaces.*position.*velocity",
        )

        missing_control_word = valid_manifest()
        missing_control_word["drive_profiles"][0]["command_interfaces"] = [
            "position"
        ]
        self.assert_invalid(
            missing_control_word,
            "whole_body_trajectory.*requires command interface.*control_word",
        )

        missing_status_word = valid_manifest()
        missing_status_word["drive_profiles"][0]["state_interfaces"] = ["position"]
        self.assert_invalid(
            missing_status_word,
            "whole_body_trajectory.*requires state interface.*status_word",
        )

        non_linear_track = valid_manifest()
        non_linear_track["transmissions"][1]["si_unit"] = "rad"
        self.assert_invalid(
            non_linear_track,
            "left_track.*transmission.*linear SI unit 'm'",
        )

        track_with_unmanaged_interfaces = valid_manifest()
        track_with_unmanaged_interfaces["drive_profiles"][1][
            "command_interfaces"
        ].append("control_word")
        track_with_unmanaged_interfaces["drive_profiles"][1][
            "state_interfaces"
        ].append("status_word")
        self.assert_invalid(
            track_with_unmanaged_interfaces,
            "left_track.*command interfaces must be exactly.*velocity",
        )
        self.assert_invalid(
            track_with_unmanaged_interfaces,
            "left_track.*state interfaces must be exactly.*position.*velocity",
        )

    def test_track_roles_are_balanced_and_nonempty(self):
        missing_right = valid_manifest()
        missing_right["buses"]["canopen"]["nodes"].pop()
        missing_right["actuators"].pop()
        missing_right["joints"].pop()
        self.assert_invalid(
            missing_right,
            "track roles.*left_track.*right_track.*equal non-zero counts",
        )

    def test_enable_topology_respects_compile_time_index_limits(self):
        no_whole_body = valid_manifest()
        no_whole_body["buses"]["ethercat"]["ring"] = [
            {"position": 0, "role": "hub"}
        ]
        no_whole_body["drive_profiles"] = no_whole_body["drive_profiles"][1:]
        no_whole_body["transmissions"] = no_whole_body["transmissions"][1:]
        no_whole_body["actuators"] = no_whole_body["actuators"][1:]
        no_whole_body["joints"] = no_whole_body["joints"][1:]
        no_whole_body["enable"] = {"batches": [], "lifecycle_policies": []}
        with self.assertRaises(driver_variant.DriverVariantError) as context:
            driver_variant.validate_manifest(no_whole_body)
        self.assertIn("at least one whole_body_trajectory joint", str(context.exception))
        self.assertIn("at least one enable batch", str(context.exception))

        too_many_axes = valid_manifest()
        template_joint = too_many_axes["joints"][0]
        too_many_axes["joints"] = [
            {**copy.deepcopy(template_joint), "name": f"axis_{index}"}
            for index in range(129)
        ] + too_many_axes["joints"][1:]
        self.assert_invalid(too_many_axes, "at most 128 whole_body_trajectory joints")

        too_many_batches = valid_manifest()
        too_many_batches["enable"]["batches"] = [
            {"id": f"batch_{index}", "joints": ["arm_joint"]}
            for index in range(129)
        ]
        self.assert_invalid(too_many_batches, "at most 128 enable batches")

        oversized_batch = valid_manifest()
        oversized_batch["enable"]["batches"][0]["joints"] = [
            "arm_joint",
            "unknown_1",
            "unknown_2",
            "unknown_3",
        ]
        self.assert_invalid(oversized_batch, "at most 3 joints")

    def test_actuators_have_one_bus_address_and_one_logical_joint(self):
        unmapped = valid_manifest()
        unmapped["buses"]["ethercat"]["ring"][1].pop("actuator")
        self.assert_invalid(unmapped, "actuator.*arm_motor.*bus address")

        duplicate_bus_mapping = valid_manifest()
        duplicate_bus_mapping["buses"]["ethercat"]["ring"].append(
            {"position": 2, "role": "drive", "actuator": "arm_motor"}
        )
        self.assert_invalid(duplicate_bus_mapping, "actuator.*arm_motor.*multiple bus")

        duplicate_joint_mapping = valid_manifest()
        duplicate_joint_mapping["joints"].append(
            {
                "name": "second_arm_joint",
                "actuator": "arm_motor",
                "controller_role": "whole_body_trajectory",
                "position_tolerance": 0.01,
                "lifecycle_policy": "cia402_default",
            }
        )
        self.assert_invalid(
            duplicate_joint_mapping, "actuator.*arm_motor.*multiple logical joints"
        )

    def test_enable_batches_cover_managed_joints_once(self):
        duplicate = valid_manifest()
        duplicate["enable"]["batches"].append(
            {"id": "again", "joints": ["arm_joint"]}
        )
        self.assert_invalid(duplicate, "joint.*arm_joint.*multiple enable batches")

        missing = valid_manifest()
        missing["enable"]["batches"] = []
        self.assert_invalid(missing, "managed joint.*arm_joint.*enable batch")

        missing_policy = valid_manifest()
        missing_policy["joints"][0].pop("lifecycle_policy")
        missing_policy["enable"]["batches"] = []
        self.assert_invalid(
            missing_policy, "whole_body_trajectory joint.*arm_joint.*lifecycle policy"
        )

        canopen_joint = valid_manifest()
        canopen_joint["enable"]["batches"][0]["joints"].append("left_track_joint")
        self.assert_invalid(
            canopen_joint,
            "joint.*left_track_joint.*without lifecycle policy.*enable batch",
        )

    def test_effective_conversion_must_be_reciprocal_with_explicit_tolerance(self):
        non_reciprocal = valid_manifest()
        non_reciprocal["transmissions"][0]["effective_conversion"]["position"][
            "from_device"
        ] = 0.4
        self.assert_invalid(non_reciprocal, "arm_transmission.*position.*not reciprocal")

        missing_tolerance = valid_manifest()
        missing_tolerance["transmissions"][0].pop("reciprocity_rel_tol")
        self.assert_invalid(missing_tolerance, "reciprocity_rel_tol.*required")

        current_rounding = valid_manifest()
        conversion = current_rounding["transmissions"][0]["effective_conversion"][
            "position"
        ]
        conversion["to_device"] = 41721.513402
        conversion["from_device"] = 0.0000239684498
        driver_variant.validate_manifest(current_rounding)

    def test_multiple_findings_are_reported_together(self):
        document = valid_manifest()
        document["schema_version"] = 2
        document["vendor_id"] = 1

        with self.assertRaises(driver_variant.DriverVariantError) as context:
            driver_variant.validate_manifest(document)

        self.assertGreaterEqual(len(context.exception.findings), 2)
        self.assertIn("schema_version", str(context.exception))
        self.assertIn("vendor_id", str(context.exception))

    def test_load_manifest_rejects_io_yaml_and_path_boundary_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            with self.assertRaisesRegex(
                driver_variant.DriverVariantError, "cannot read manifest"
            ):
                driver_variant.load_manifest(root / "missing.yaml")

            malformed = root / "malformed.yaml"
            malformed.write_text("joints: [\n", encoding="utf-8")
            with self.assertRaisesRegex(driver_variant.DriverVariantError, "invalid YAML"):
                driver_variant.load_manifest(malformed)

            invalid_utf8 = root / "invalid-utf8.yaml"
            invalid_utf8.write_bytes(b"\xff\xfe")
            with self.assertRaisesRegex(
                driver_variant.DriverVariantError, "cannot read manifest"
            ):
                driver_variant.load_manifest(invalid_utf8)

            duplicate_key = root / "duplicate-key.yaml"
            duplicate_key.write_text(
                "schema_version: 1\nschema_version: 2\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(
                driver_variant.DriverVariantError, "duplicate YAML key.*schema_version"
            ):
                driver_variant.load_manifest(duplicate_key)

            not_mapping = root / "not-mapping.yaml"
            not_mapping.write_text("- item\n", encoding="utf-8")
            with self.assertRaisesRegex(
                driver_variant.DriverVariantError, "manifest.*must be a mapping"
            ):
                driver_variant.load_manifest(not_mapping)

        missing_profile = valid_manifest()
        missing_profile["drive_profiles"][0]["config"] = "missing/profile.yaml"
        with self.assertRaisesRegex(driver_variant.DriverVariantError, "file does not exist"):
            driver_variant.validate_manifest(missing_profile, repository_root=ROOT)

        traversal = valid_manifest()
        traversal["drive_profiles"][0]["config"] = "../outside.yaml"
        self.assert_invalid(traversal, "config.*repository-relative path")

        with tempfile.TemporaryDirectory() as repository_directory:
            with tempfile.TemporaryDirectory() as outside_directory:
                repository_root = Path(repository_directory)
                outside = Path(outside_directory) / "profile.yaml"
                outside.write_text("profile: outside\n", encoding="utf-8")
                (repository_root / "escaped.yaml").symlink_to(outside)
                symlink_escape = valid_manifest()
                symlink_escape["drive_profiles"][0]["config"] = "escaped.yaml"
                for profile in symlink_escape["drive_profiles"][1:]:
                    profile_path = repository_root / profile["config"]
                    profile_path.parent.mkdir(parents=True, exist_ok=True)
                    profile_path.write_text("profile: local\n", encoding="utf-8")
                with self.assertRaisesRegex(
                    driver_variant.DriverVariantError,
                    "config.*resolves outside repository root",
                ):
                    driver_variant.validate_manifest(
                        symlink_escape, repository_root=repository_root
                    )

    def test_current_manifest_describes_the_complete_fixed_machine(self):
        document = driver_variant.load_manifest(MANIFEST, repository_root=ROOT)

        self.assertEqual(document["schema_version"], 1)
        self.assertEqual(len(document["buses"]["ethercat"]["ring"]), 16)
        self.assertEqual(len(document["buses"]["canopen"]["nodes"]), 2)
        self.assertEqual(len(document["actuators"]), 16)
        self.assertEqual(len(document["joints"]), 16)
        self.assertEqual(
            [joint["name"] for joint in document["joints"]],
            [
                "right_joint1",
                "right_joint2",
                "right_joint3",
                "right_joint4",
                "right_joint5",
                "right_joint6",
                "left_joint1",
                "left_joint2",
                "left_joint3",
                "left_joint4",
                "left_joint5",
                "left_joint6",
                "turn",
                "updown",
                "left_track_joint",
                "right_track_joint",
            ],
        )

        self.assertEqual(
            [entry["position"] for entry in document["buses"]["ethercat"]["ring"]],
            list(range(16)),
        )
        self.assertEqual(
            [entry["node_id"] for entry in document["buses"]["canopen"]["nodes"]],
            [2, 3],
        )

        self.assertEqual(
            [batch["joints"] for batch in document["enable"]["batches"]],
            [
                ["right_joint1", "right_joint2", "right_joint3"],
                ["left_joint1", "left_joint2", "left_joint3"],
                ["right_joint4", "right_joint5", "right_joint6"],
                ["left_joint4", "left_joint5", "left_joint6"],
                ["turn", "updown"],
            ],
        )

        policies = {policy["id"]: policy for policy in document["enable"]["lifecycle_policies"]}
        self.assertEqual(
            policies["ti5_bq115"]["disable_terminals"],
            ["switch_on_disabled", "ready_to_switch_on"],
        )
        self.assertEqual(policies["ti5_bq115"]["source"], "BQ-115")
        self.assertEqual(
            [
                joint["name"]
                for joint in document["joints"]
                if joint.get("lifecycle_policy") == "ti5_bq115"
            ],
            ["right_joint2", "right_joint3", "left_joint2", "left_joint3"],
        )


if __name__ == "__main__":
    unittest.main()
