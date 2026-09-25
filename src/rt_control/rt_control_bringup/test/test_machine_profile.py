"""ELECTRI-118 machine-profile and control-scope contract tests."""

from __future__ import annotations

import copy
from pathlib import Path
import sys

import pytest
import yaml


BRINGUP_DIR = Path(__file__).resolve().parents[1]
MACHINE_PATH = BRINGUP_DIR / "config/machines/alfa_v3.yaml"
SWERVE_MECHANICS_PATH = (
    BRINGUP_DIR / "config/machines/alfa_v3_swerve_mechanics.draft.yaml"
)
ARM_LAYOUT_PATH = (
    BRINGUP_DIR.parent / "robot_hw_ethercat/config/machines/alfa_v3_arms_only.draft.yaml"
)

if str(BRINGUP_DIR) not in sys.path:
    sys.path.insert(0, str(BRINGUP_DIR))


def _module():
    from rt_control_bringup import machine_profile

    return machine_profile


def _load_document() -> dict:
    return yaml.safe_load(MACHINE_PATH.read_text(encoding="utf-8"))


def _write_document(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / "machine.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def test_alfa_v3_manifest_exposes_the_five_actuator_modules_and_five_physical_profiles():
    module = _module()

    manifest = module.load_machine_manifest(MACHINE_PATH)

    assert manifest.variant == "alfa_v3"
    assert manifest.robot_model.package == "robot_description"
    assert manifest.robot_model.xacro_file == "urdf/robot_dual_gripper.urdf.xacro"
    assert manifest.robot_model.srdf_file == "srdf/robot.srdf"
    assert manifest.robot_model.joint_limits_file == "config/joint_limits_gripper.yaml"
    assert manifest.robot_model.initial_positions_file == (
        "config/initial_positions_gripper.yaml"
    )
    assert manifest.robot_model.end_effector == "gripper"
    assert manifest.robot_model.source_revision == (
        "ec69ca04297896c1296720324d23cdb8f80d9e63"
    )
    assert manifest.functional_modules == (
        "arms",
        "updown",
        "swerve_chassis",
        "active_suspension",
        "head_gimbal",
    )
    assert set(manifest.modules) == {
        "arms",
        "wrist_force_sensors",
        "updown",
        "swerve_chassis",
        "swerve_encoders",
        "active_suspension",
        "head_gimbal",
    }
    assert set(manifest.profiles) == {
        "arms_only",
        "arms_updown",
        "chassis_only",
        "full_robot",
        "head_only",
    }
    assert set(manifest.scopes) >= {
        "arms_only",
        "arms_updown",
        "chassis_only",
        "full",
        "head_only",
    }
    assert manifest.modules["swerve_encoders"].role == "state_sensor_group"
    force_option = manifest.hardware_options["force_sensors"]
    assert force_option.default_selection == "none"
    assert set(force_option.selections) == {"none", "bluepoint_dual"}
    assert force_option.selections["bluepoint_dual"].modules == (
        "wrist_force_sensors",
    )
    assert manifest.modules["swerve_encoders"].transport == "canopen"
    assert manifest.modules["swerve_encoders"].profile_ref == {
        "package": "robot_hw_canopen",
        "file": "config/machines/alfa_v3_swerve_encoders.draft.yaml",
        "verified": False,
    }
    assert manifest.modules["head_gimbal"].transport == "damiao_can"
    assert manifest.modules["active_suspension"].transport == "ethercat"
    assert manifest.modules["active_suspension"].mode_groups[0].name == "csp"
    assert manifest.modules["arms"].mechanical_parameters == "TBD"
    assert manifest.modules["swerve_chassis"].mechanical_parameters == {
        "package": "rt_control_bringup",
        "file": "config/machines/alfa_v3_swerve_mechanics.draft.yaml",
        "verified": False,
    }
    assert manifest.modules["swerve_chassis"].profile_ref == {
        "package": "robot_hw_ethercat",
        "file": "config/machines/alfa_v3_swerve_chassis.draft.yaml",
        "verified": False,
    }
    assert manifest.modules["arms"].instance_count == 2
    assert manifest.modules["arms"].per_instance_mode_groups == {
        "csp": 7,
        "gripper_pp": 1,
    }
    assert manifest.profiles["full_robot"].allowed_scopes == ("full",)
    assert {
        name: profile.allowed_scopes
        for name, profile in manifest.profiles.items()
    } == {
        "arms_only": ("arms_only",),
        "arms_updown": ("arms_updown",),
        "chassis_only": ("chassis_only",),
        "full_robot": ("full",),
        "head_only": ("head_only",),
    }
    assert [(item.source_module, item.affected_modules, item.reaction)
            for item in manifest.fault_dependencies] == [
        ("active_suspension", ("swerve_chassis",), "stop_and_inhibit"),
        ("swerve_chassis", ("arms",), "stop"),
    ]


def test_swerve_vendor_drawing_facts_do_not_replace_vehicle_calibration():
    mechanics = yaml.safe_load(SWERVE_MECHANICS_PATH.read_text(encoding="utf-8"))

    assert mechanics["verified"] is False
    assert mechanics["source"]["sha256"] == (
        "8feb1c75ce4258e551ba6fddc9b517d55e842fc32d7e7cffdc49d639ad916c1f"
    )
    assert mechanics["module"]["nominal_wheel_diameter_m"] == 0.2
    assert mechanics["drive"]["selected_reducer_ratio"] == 17.68
    assert mechanics["steering"]["steering_support_gear_teeth"] == 108
    assert mechanics["steering"]["external_encoder_pinion_gear_teeth"] == 27
    assert mechanics["steering"]["external_encoder_ring_gear_teeth"] == 108
    assert mechanics["steering"]["external_encoder_turns_per_axis_turn"] == 4.0
    assert mechanics["steering"]["external_encoder_gearing_verified"] is True
    assert mechanics["steering"]["reducer_ratio"] == 35.0
    assert mechanics["steering"]["derived_motor_to_steering_axis_ratio"] == "TBD"
    assert mechanics["steering"]["derived_ratio_verified"] is False
    assert mechanics["steering"]["mechanical_min_angle_rad"] == "TBD"
    assert mechanics["steering"]["mechanical_max_angle_rad"] == "TBD"
    assert "per-module loaded effective wheel radius" in (
        mechanics["required_vehicle_calibration"]
    )

    controller = yaml.safe_load(
        (BRINGUP_DIR.parent / "swerve_driver/config/controller.draft.yaml").read_text(
            encoding="utf-8"
        )
    )
    parameters = controller["swerve_controller"]["ros__parameters"]
    assert parameters["calibration_verified"] is False
    assert parameters["wheel_radius"] == ["TBD"] * 4
    assert parameters["steering_min"] == ["TBD"] * 4
    assert parameters["steering_max"] == ["TBD"] * 4


def test_unverified_swerve_mechanics_remain_a_runtime_blocker(tmp_path: Path):
    document = _load_document()
    chassis = document["modules"]["swerve_chassis"]
    chassis["hardware_identity"] = {"checked": True}
    chassis["profile_ref"] = {"checked": True}

    selected = _module().select_hardware(
        _write_document(tmp_path, document),
        physical_profile="chassis_only",
        control_scope="chassis_only",
    )

    assert "module swerve_chassis contains TBD or unverified values" in (
        selected.runtime_blockers
    )


def test_hardware_owner_refs_preserve_every_transport_during_chassis_rename():
    document = _load_document()
    transports = {item["transport"] for item in document["modules"].values()}

    assert set(document["owner_refs"]) == transports
    manifest = _module().load_machine_manifest(MACHINE_PATH)
    assert manifest.owner_refs["ethercat"] == "robot_hw_ethercat"
    assert manifest.owner_refs["canopen"] == "robot_hw_canopen"
    assert manifest.owner_refs[manifest.modules["head_gimbal"].transport]


def test_arms_only_selection_derives_sixteen_actuators_with_fourteen_csp_and_two_pp():
    module = _module()

    selected = module.select_hardware(
        MACHINE_PATH,
        physical_profile="arms_only",
        control_scope="arms_only",
    )

    assert selected.active_modules == ("arms",)
    assert selected.inactive_modules == ()
    assert selected.ethercat_master_id == 0
    assert selected.ethercat_ring_positions == tuple(range(18))
    assert selected.canopen_node_ids == ()
    assert selected.actuator_count == 16
    assert selected.mode_counts == {8: 14, 1: 2}
    assert selected.group_counts == {
        "arms.csp": 14,
        "arms.gripper_pp": 2,
    }
    assert [(item.module, item.mode_group) for item in selected.jtc_groups] == [
        ("arms", "csp")
    ]
    assert ("arms", "gripper_pp") in {
        (item.module, item.mode_group) for item in selected.actuator_groups
    }
    assert selected.jtc_mode_counts == {8: 14}
    assert selected.required_state_modules == ()
    assert selected.validation_status == "draft"
    assert selected.hardware_options == {"force_sensors": "none"}
    assert selected.bus_requirements == {
        "ethercat": "required",
        "canopen": "not_required",
        "damiao_can": "not_required",
    }
    assert selected.fault_dependencies == ()


def test_dual_bluepoint_option_adds_two_state_only_sensors_without_changing_actuators():
    selected = _module().select_hardware(
        MACHINE_PATH,
        physical_profile="arms_only",
        control_scope="arms_only",
        hardware_options={"force_sensors": "bluepoint_dual"},
    )

    assert selected.hardware_options == {"force_sensors": "bluepoint_dual"}
    assert selected.physical_modules == ("arms", "wrist_force_sensors")
    assert selected.active_modules == ("arms", "wrist_force_sensors")
    assert selected.inactive_modules == ()
    assert selected.actuator_count == 16
    assert selected.state_sensor_count == 2
    assert selected.required_state_modules == ("wrist_force_sensors",)
    assert selected.ethercat_ring_positions is None
    assert "EtherCAT ring positions are TBD" in selected.runtime_blockers


def test_bluepoint_option_is_rejected_for_profiles_without_arms():
    with pytest.raises(_module().MachineProfileError, match="not allowed"):
        _module().select_hardware(
            MACHINE_PATH,
            physical_profile="chassis_only",
            control_scope="chassis_only",
            hardware_options={"force_sensors": "bluepoint_dual"},
        )


def test_arms_updown_adds_only_one_csp_axis():
    module = _module()

    selected = module.select_hardware(
        MACHINE_PATH,
        physical_profile="arms_updown",
        control_scope="arms_updown",
    )

    assert selected.active_modules == ("arms", "updown")
    assert selected.actuator_count == 17
    assert selected.mode_counts == {8: 15, 1: 2}
    assert selected.jtc_mode_counts == {8: 15}


def test_instance_expansion_rejects_an_incorrect_per_arm_split(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["modules"]["arms"]["per_instance_mode_groups"]["csp"] = 6
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="expand to mode counts"):
        module.load_machine_manifest(path)


@pytest.mark.parametrize(
    "scope", ["arms_only", "arms_updown", "chassis_only", "head_only"]
)
def test_full_physical_profile_requires_full_scope(scope: str):
    module = _module()

    with pytest.raises(module.MachineProfileError, match="not allowed"):
        module.select_hardware(
            MACHINE_PATH,
            physical_profile="full_robot",
            control_scope=scope,
        )


def test_chassis_scope_requires_four_external_canopen_encoder_states():
    module = _module()

    selected = module.select_hardware(
        MACHINE_PATH,
        physical_profile="chassis_only",
        control_scope="chassis_only",
    )

    assert selected.active_modules == (
        "swerve_chassis", "swerve_encoders", "active_suspension"
    )
    assert selected.actuator_count == 9
    assert selected.mode_counts == {8: 5, 9: 4}
    assert selected.required_state_modules == ("swerve_encoders",)
    assert selected.state_sensor_count == 4
    assert selected.group_counts == {
        "swerve_chassis.steering_csp": 4,
        "swerve_chassis.drive_csv": 4,
        "active_suspension.csp": 1,
    }
    assert selected.canopen_node_ids is None
    assert selected.global_readiness == "partial_scope"
    assert selected.bus_requirements == {
        "ethercat": "required",
        "canopen": "required",
        "damiao_can": "not_required",
    }
    assert [(item.source_module, item.affected_modules, item.reaction)
            for item in selected.fault_dependencies] == [
        ("active_suspension", ("swerve_chassis",), "stop_and_inhibit")
    ]
    assert [(binding.module, binding.name, binding.plugin, binding.package, binding.config_file)
            for binding in selected.controllers] == [
        ("swerve_chassis", "swerve_controller", "swerve_driver/SwerveController",
         "swerve_driver", "config/controller.draft.yaml")
    ]
    assert selected.controllers[0].required_state_modules == ("swerve_encoders",)


def test_arm_only_scope_never_claims_swerve_controller():
    selected = _module().select_hardware(
        MACHINE_PATH, physical_profile="arms_only", control_scope="arms_only"
    )
    assert selected.controllers == ()


def test_full_physical_robot_enables_every_physical_actuator_and_fault_dependency():
    selected = _module().select_hardware(
        MACHINE_PATH, physical_profile="full_robot", control_scope="full"
    )

    assert [binding.name for binding in selected.controllers] == ["swerve_controller"]
    assert selected.required_state_modules == ("swerve_encoders",)
    assert selected.inactive_modules == ()
    assert selected.actuator_count == 28
    assert selected.mode_counts == {8: 20, 1: 2, 9: 4}
    assert selected.bus_requirements == {
        "ethercat": "required",
        "canopen": "required",
        "damiao_can": "required",
    }
    assert [(item.source_module, item.affected_modules, item.reaction)
            for item in selected.fault_dependencies] == [
        ("active_suspension", ("swerve_chassis",), "stop_and_inhibit"),
        ("swerve_chassis", ("arms",), "stop"),
    ]


def test_swerve_controller_binding_requires_existing_module_and_encoder(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["controllers"]["swerve_chassis"]["required_state_modules"] = []
    with pytest.raises(module.MachineProfileError, match="required state"):
        module.load_machine_manifest(_write_document(tmp_path, document))

    document = _load_document()
    document["controllers"]["swerve_chassis"]["module"] = "missing_chassis"
    with pytest.raises(module.MachineProfileError, match="unknown module"):
        module.load_machine_manifest(_write_document(tmp_path, document))


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("unknown_source", "unknown source module"),
        ("sensor_affected", "require actuator modules"),
        ("unknown_reaction", "reaction must be one of"),
        ("duplicate", "duplicates a fault dependency"),
    ],
)
def test_fault_dependency_policy_fails_closed(
    tmp_path: Path, mutation: str, message: str
):
    module = _module()
    document = _load_document()
    dependencies = document["safety_policy"]["fault_dependencies"]
    if mutation == "unknown_source":
        dependencies[0]["source_module"] = "missing_module"
    elif mutation == "sensor_affected":
        dependencies[0]["affected_modules"] = ["swerve_encoders"]
    elif mutation == "unknown_reaction":
        dependencies[0]["reaction"] = "continue"
    elif mutation == "duplicate":
        dependencies.append(copy.deepcopy(dependencies[0]))
    else:  # pragma: no cover - protects the test table itself
        raise AssertionError(mutation)

    with pytest.raises(module.MachineProfileError, match=message):
        module.load_machine_manifest(_write_document(tmp_path, document))


def test_controller_bindings_are_optional_for_other_v1_machine_variants(tmp_path: Path):
    document = _load_document()
    document["robot_variant"] = "other_machine"
    del document["controllers"]

    selected = _module().select_hardware(
        _write_document(tmp_path, document),
        physical_profile="chassis_only",
        control_scope="chassis_only",
    )
    assert selected.controllers == ()


def test_alfa_v3_cannot_omit_its_swerve_controller_binding(tmp_path: Path):
    document = _load_document()
    del document["controllers"]

    with pytest.raises(_module().MachineProfileError, match="swerve chassis controller"):
        _module().load_machine_manifest(_write_document(tmp_path, document))


def test_swerve_binding_matches_installed_draft_controller_contract():
    binding = _module().load_machine_manifest(MACHINE_PATH).controllers["swerve_chassis"]
    config_path = BRINGUP_DIR.parent / binding.package / binding.config_file
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    manager = config["controller_manager"]["ros__parameters"]
    params = config[binding.name]["ros__parameters"]

    assert manager["update_rate"] == 250
    assert manager[binding.name]["type"] == binding.plugin
    assert params["calibration_verified"] is False
    assert params["steering_joints"] == ["TBD"] * 4
    assert params["drive_joints"] == ["TBD"] * 4
    assert params["steering_encoders"] == [
        "front_left_steering_encoder",
        "front_right_steering_encoder",
        "rear_left_steering_encoder",
        "rear_right_steering_encoder",
    ]


def test_draft_swerve_config_blocks_runtime_even_if_other_facts_are_ready(tmp_path: Path):
    document = _load_document()
    document["status"] = "ready"
    for item in document["modules"].values():
        item["status"] = "ready"
        item["pending_facts"] = []
        item["hardware_identity"] = {"checked": True}
        item["profile_ref"] = {"checked": True}
        item["mechanical_parameters"] = {"checked": True}
        for group in item["mode_groups"]:
            group["certified"] = True
    profile = document["profiles"]["chassis_only"]
    profile["status"] = "ready"
    profile["pending_facts"] = []
    profile["layout"]["ethercat"]["status"] = "ready"
    profile["layout"]["ethercat"]["ring_positions"] = list(range(8))
    profile["layout"]["canopen"]["status"] = "ready"
    profile["layout"]["canopen"]["node_ids"] = [1, 2, 3, 4]
    module = _module()
    with pytest.raises(module.MachineProfileError, match="draft config"):
        module.select_hardware(
            _write_document(tmp_path, document),
            physical_profile="chassis_only",
            control_scope="chassis_only",
            require_runtime_ready=True,
        )


def test_head_only_has_no_ethercat_master_and_uses_damiao_can():
    module = _module()

    selected = module.select_hardware(
        MACHINE_PATH,
        physical_profile="head_only",
        control_scope="head_only",
    )

    assert selected.active_modules == ("head_gimbal",)
    assert selected.ethercat_master_id is None
    assert selected.ethercat_ring_positions == ()
    assert selected.damiao_can_node_ids is None
    assert selected.actuator_count == 2
    assert selected.transports == ("damiao_can",)


def test_full_scope_keeps_non_cia402_head_group_in_group_counts():
    module = _module()

    selected = module.select_hardware(
        MACHINE_PATH,
        physical_profile="full_robot",
        control_scope="full",
    )

    assert selected.actuator_count == 28
    assert selected.mode_counts == {8: 20, 1: 2, 9: 4}
    assert selected.group_counts["head_gimbal.vendor_can"] == 2


def test_draft_profile_is_allowed_for_static_validation_but_rejected_for_runtime():
    module = _module()

    with pytest.raises(module.MachineProfileError, match="not runtime-ready"):
        module.select_hardware(
            MACHINE_PATH,
            physical_profile="arms_only",
            control_scope="arms_only",
            require_runtime_ready=True,
        )


def test_tbd_placeholder_still_blocks_runtime_even_if_status_fields_are_marked_ready(
    tmp_path: Path,
):
    module = _module()
    document = _load_document()
    document["status"] = "ready"
    for item in document["modules"].values():
        item["status"] = "ready"
        item["pending_facts"] = []
    for item in document["profiles"].values():
        item["status"] = "ready"
        item["pending_facts"] = []
        for layout in item["layout"].values():
            if layout["status"] != "absent":
                layout["status"] = "ready"
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="TBD"):
        module.select_hardware(
            path,
            physical_profile="arms_only",
            control_scope="arms_only",
            require_runtime_ready=True,
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("unknown_module", "unknown module"),
        ("duplicate_profile_module", "duplicate module"),
        ("scope_not_allowed", "cannot allow"),
        ("missing_encoder_dependency", "requires module"),
        ("scope_missing_encoder_dependency", "requires module"),
        ("invalid_mode_count", "count must be a positive integer"),
    ],
)
def test_manifest_mutations_fail_closed(tmp_path: Path, mutation: str, message: str):
    module = _module()
    document = _load_document()

    if mutation == "unknown_module":
        document["profiles"]["arms_only"]["modules"].append("missing")
    elif mutation == "duplicate_profile_module":
        document["profiles"]["arms_only"]["modules"].append("arms")
    elif mutation == "scope_not_allowed":
        document["profiles"]["arms_only"]["allowed_scopes"] = ["head_only"]
    elif mutation == "missing_encoder_dependency":
        document["profiles"]["chassis_only"]["modules"].remove("swerve_encoders")
    elif mutation == "scope_missing_encoder_dependency":
        document["scopes"]["chassis_only"]["enabled_modules"].remove("swerve_encoders")
    elif mutation == "invalid_mode_count":
        document["modules"]["arms"]["mode_groups"][0]["count"] = 0
    else:  # pragma: no cover - protects the test table itself
        raise AssertionError(mutation)

    path = _write_document(tmp_path, document)
    with pytest.raises(module.MachineProfileError, match=message):
        module.load_machine_manifest(path)


def test_unscanned_profile_layouts_remain_tbd_after_arm_bench_confirmation():
    document = _load_document()

    for profile_name, profile in document["profiles"].items():
        ethercat = profile["layout"]["ethercat"]
        if profile_name == "arms_only":
            assert ethercat["ring_positions"] == list(range(18))
        elif "ethercat" in profile["modules"] or any(
            module_name in document["modules"]
            and document["modules"][module_name]["transport"] == "ethercat"
            for module_name in profile["modules"]
        ):
            assert ethercat["ring_positions"] == "TBD"
        else:
            assert ethercat["status"] == "absent"


@pytest.mark.parametrize(
    "side,connector,esc_port,first_position",
    [("left", "X2", 3, 1), ("right", "X3", 1, 9)],
)
def test_confirmed_arm_chain_assigns_j1_to_j7_csp_then_pp_gripper(
    side, connector, esc_port, first_position
):
    layout = yaml.safe_load(ARM_LAYOUT_PATH.read_text(encoding="utf-8"))
    branch = layout["branches"][side]
    axes = branch["axes"]

    assert (branch["connector"], branch["main_esc_port"]) == (connector, esc_port)
    assert [axis["physical_joint"] for axis in axes] == [
        "J1", "J2", "J3", "J4", "J5", "J6", "J7", "gripper"
    ]
    assert [axis["ring_position"] for axis in axes] == list(
        range(first_position, first_position + 8)
    )
    assert [axis["mode_of_operation"] for axis in axes] == [8] * 7 + [1]
    assert [axis["robot_model_joint"] for axis in axes] == [
        *(f"{side}_joint{index}" for index in range(1, 8)),
        f"{side}_moving_jaw_joint",
    ]


def test_confirmed_arm_inventory_matches_manifest_without_admitting_runtime():
    module = _module()
    manifest = module.load_machine_manifest(MACHINE_PATH)
    selected = module.select_hardware(
        manifest, physical_profile="arms_only", control_scope="arms_only"
    )
    layout = yaml.safe_load(ARM_LAYOUT_PATH.read_text(encoding="utf-8"))
    axes = [axis for branch in layout["branches"].values() for axis in branch["axes"]]
    responders = layout["extra_responders"]
    positions = [item["ring_position"] for item in [*axes, *responders]]

    assert layout["robot_variant"] == manifest.variant
    assert layout["physical_profile"] == selected.physical_profile
    assert layout["master_id"] == selected.ethercat_master_id
    assert len(axes) == selected.actuator_count
    assert {
        mode: sum(axis["mode_of_operation"] == mode for axis in axes)
        for mode in selected.mode_counts
    } == dict(selected.mode_counts)
    assert [item["ring_position"] for item in responders] == [0, 17]
    assert all("mode_of_operation" not in item for item in responders)
    assert len(positions) == len(set(positions)) == len(selected.ethercat_ring_positions)
    assert sorted(positions) == list(selected.ethercat_ring_positions)
    assert all(
        branch["identity_ref"] in layout["identities"]
        for branch in layout["branches"].values()
    )
    assert all(item["identity_ref"] in layout["identities"] for item in responders)
    identity_ref = manifest.modules["arms"].hardware_identity
    assert (
        BRINGUP_DIR.parent / identity_ref["package"] / identity_ref["file"]
    ) == ARM_LAYOUT_PATH
    group, key = identity_ref["key"].split(".")
    assert layout[group][key] == {
        "vendor_id": 0x5A65726F,
        "product_code": 0x00029252,
        "revision_number": 1,
    }
    assert layout["status"] == "draft"
    assert layout["mechanical_parameters"] == "TBD"
    hardware_dir = ARM_LAYOUT_PATH.parents[2]
    csp = yaml.safe_load((hardware_dir / layout["drive_profiles"]["csp"]).read_text())
    pp = yaml.safe_load((hardware_dir / layout["drive_profiles"]["gripper_pp"]).read_text())
    assert csp["verified"] is False
    assert csp["rpdo"][0]["channels"][0]["factor"] == "TBD"
    assert pp["pp"]["verified"] is False
    assert pp["pp"]["counts_per_metre"] == "TBD"
    assert pp["pp"]["max_force"] == "TBD"
    assert all(sdo["value"] == "TBD" for sdo in pp["sdo"] if sdo["index"] != 0x6060)
    with pytest.raises(module.MachineProfileError, match="not runtime-ready"):
        module.select_hardware(
            manifest, physical_profile="arms_only", control_scope="arms_only",
            require_runtime_ready=True,
        )


def test_encoder_usage_policy_is_scoped_to_confirmed_physical_axes():
    layout = yaml.safe_load(ARM_LAYOUT_PATH.read_text(encoding="utf-8"))
    assigned = {
        axis["ring_position"]: axis["encoder_policy"]
        for branch in layout["branches"].values()
        for axis in branch["axes"]
        if "encoder_policy" in axis
    }

    assert assigned == {
        3: "single_turn", 8: "multiturn_required",
        12: "single_turn", 16: "multiturn_required",
    }
    assert set(assigned.values()) == set(layout["encoder_policies"])


def test_encoder_policies_keep_calibration_and_fault_checks_required():
    layout = yaml.safe_load(ARM_LAYOUT_PATH.read_text(encoding="utf-8"))
    single = layout["encoder_policies"]["single_turn"]
    multi = layout["encoder_policies"]["multiturn_required"]

    assert single["requires_multiturn_retention"] is False
    assert single["startup_battery_reset_codes"] == [0x730F]
    assert single["startup_reset_method"] == "cia402_fault_reset"
    assert single["reset_owner"] == "enable_manager"
    assert single["require_nonwrapping_output_range"] is True
    assert single["output_count_interval"] == "TBD"
    assert single["range_verified"] is False
    assert multi["requires_multiturn_retention"] is True
    assert multi["startup_battery_reset_codes"] == []
    assert multi["require_valid_battery"] is True
    assert multi["require_valid_multiturn_reference"] is True
    assert multi["battery_verified"] is False
    assert multi["reference_verified"] is False
    for policy in (single, multi):
        assert policy["require_fault_free_status"] is True
        assert policy["auto_reset_load_encoder"] is False


def test_manifest_loader_does_not_mutate_source_document(tmp_path: Path):
    module = _module()
    document = _load_document()
    original = copy.deepcopy(document)
    path = _write_document(tmp_path, document)

    module.load_machine_manifest(path)

    assert document == original


def test_duplicate_yaml_keys_are_rejected(tmp_path: Path):
    module = _module()
    path = tmp_path / "duplicate.yaml"
    path.write_text(
        "schema_version: 1\n"
        "robot_variant: alfa_v3\n"
        "robot_variant: alfa_v3\n",
        encoding="utf-8",
    )

    with pytest.raises(module.MachineProfileError, match="duplicate mapping key"):
        module.load_machine_manifest(path)


def test_explicit_ring_layout_must_be_complete_and_zero_based(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["profiles"]["arms_only"]["layout"]["ethercat"]["ring_positions"] = [0, 2]
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="complete 0-based"):
        module.load_machine_manifest(path)


def test_all_ethercat_modules_in_a_profile_share_one_master(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["modules"]["updown"]["master_id"] = 1
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="share one master"):
        module.load_machine_manifest(path)


def test_profile_cannot_include_an_absent_module(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["modules"]["updown"]["status"] = "absent"
    document["profiles"]["arms_updown"]["status"] = "ready"
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="absent module"):
        module.load_machine_manifest(path)


def test_state_sensor_requires_an_explicit_authority(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["modules"]["swerve_encoders"]["authority"] = None
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="requires an authority"):
        module.load_machine_manifest(path)


def test_functional_module_list_cannot_include_feedback_resource(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["functional_modules"].append("swerve_encoders")
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="functional_modules"):
        module.load_machine_manifest(path)


def test_selected_transport_cannot_have_an_empty_layout(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["profiles"]["arms_only"]["layout"]["ethercat"]["ring_positions"] = []
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="at least one ring position"):
        module.load_machine_manifest(path)


def test_pp_gripper_group_cannot_enter_the_streaming_jtc(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["scopes"]["arms_only"]["jtc_groups"] = [
        {"module": "arms", "mode_group": "gripper_pp"}
    ]
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="only CSP position"):
        module.load_machine_manifest(path)


def test_damiao_can_uses_standard_can_id_range_not_canopen_node_range(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["profiles"]["head_only"]["layout"]["damiao_can"]["node_ids"] = [
        0x180,
        0x280,
    ]
    path = _write_document(tmp_path, document)

    manifest = module.load_machine_manifest(path)

    assert manifest.profiles["head_only"].layout.damiao_can.node_ids == (
        0x180,
        0x280,
    )


def test_damiao_can_rejects_extended_can_ids(tmp_path: Path):
    module = _module()
    document = _load_document()
    document["profiles"]["head_only"]["layout"]["damiao_can"]["node_ids"] = [
        0x800,
        0x801,
    ]
    path = _write_document(tmp_path, document)

    with pytest.raises(module.MachineProfileError, match="0..0x7FF"):
        module.load_machine_manifest(path)
