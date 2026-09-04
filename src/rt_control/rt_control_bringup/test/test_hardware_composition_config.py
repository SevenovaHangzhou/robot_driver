"""ELECTRI-94 hardware-composition configuration contract tests."""

from __future__ import annotations

import copy
import importlib
import importlib.util
from pathlib import Path
import sys
from typing import Any
import xml.etree.ElementTree as ET

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch_ros.actions import Node as LaunchNode
import pytest
import yaml


BRINGUP_DIR = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BRINGUP_DIR.parents[2]
CANONICAL_ETHERCAT_VARIANT = (
    REPOSITORY_ROOT / "src/rt_control/robot_hw_ethercat/variants/alfa_v1.yaml"
)
CANONICAL_CANOPEN_VARIANT = (
    REPOSITORY_ROOT / "src/rt_control/robot_hw_canopen/variants/alfa_v1.yaml"
)
LAUNCH_FILE = BRINGUP_DIR / "launch/rt_control.launch.py"
CONTROLLERS_CONFIG = BRINGUP_DIR / "config/controllers.yaml"
CMAKE_FILE = BRINGUP_DIR / "CMakeLists.txt"
PACKAGE_MANIFEST = BRINGUP_DIR / "package.xml"
PYTHON_PACKAGE = BRINGUP_DIR / "rt_control_bringup"

if str(BRINGUP_DIR) not in sys.path:
    sys.path.insert(0, str(BRINGUP_DIR))


def _implementation():
    return importlib.import_module("rt_control_bringup.hardware_composition")


def test_canonical_variants_load_and_flatten_for_diagnostics() -> None:
    module = _implementation()

    composition = module.load_hardware_variants(
        CANONICAL_ETHERCAT_VARIANT, CANONICAL_CANOPEN_VARIANT
    )

    assert composition.schema_version == 1
    assert composition.ethercat.expected_responders == 18
    assert [
        (axis.joint_name, axis.ring_position)
        for axis in composition.ethercat.axes
    ] == [
        ("right_joint1", 1),
        ("right_joint2", 2),
        ("right_joint3", 3),
        ("right_joint4", 4),
        ("right_joint5", 5),
        ("right_joint6", 6),
        ("left_joint1", 7),
        ("left_joint2", 8),
        ("left_joint3", 9),
        ("left_joint4", 10),
        ("left_joint5", 11),
        ("left_joint6", 12),
        ("turn", 16),
        ("updown", 17),
    ]
    assert [
        (sensor.sensor_name, sensor.ring_position)
        for sensor in composition.ethercat.sensors
    ] == [
        ("right_force_sensor", 14),
        ("left_force_sensor", 15),
    ]
    assert [
        (node.joint_name, node.node_id) for node in composition.canopen.nodes
    ] == [("left_track_joint", 2), ("right_track_joint", 3)]
    assert composition.diagnostics_parameters() == {
        "ethercat_joint_names": [
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
        ],
        "ethercat_ring_positions": [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 17],
        "ethercat_sensor_names": ["right_force_sensor", "left_force_sensor"],
        "ethercat_sensor_ring_positions": [14, 15],
        "ethercat_expected_responders": 18,
        "canopen_node_ids": [2, 3],
    }


def test_launch_loads_selected_hardware_variants_before_nodes_and_only_wires_diagnostics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _implementation()
    spec = importlib.util.spec_from_file_location("rt_control_launch_for_test", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    expected_parameters = {
        "ethercat_joint_names": ["arm_a"],
        "ethercat_ring_positions": [1],
        "ethercat_sensor_names": ["force_a"],
        "ethercat_sensor_ring_positions": [2],
        "ethercat_expected_responders": 4,
        "canopen_node_ids": [2],
    }
    events: list[tuple[str, Any]] = []
    nodes: list[dict[str, Any]] = []
    node_actions: list[LaunchNode] = []

    class _FakeComposition:
        def diagnostics_parameters(self) -> dict[str, Any]:
            return copy.deepcopy(expected_parameters)

    class _RecordingNode(LaunchNode):
        def __init__(self, **kwargs: Any) -> None:
            super().__init__(**kwargs)
            self.kwargs = kwargs
            events.append(("node", kwargs.get("executable")))
            nodes.append(kwargs)
            node_actions.append(self)

    installed_shares = {
        "rt_control_bringup": Path("/opt/rt-control/share/rt_control_bringup"),
        "robot_hw_ethercat": Path("/opt/rt-control/share/robot_hw_ethercat"),
        "robot_hw_canopen": Path("/opt/rt-control/share/robot_hw_canopen"),
    }

    def _load(ethercat_path: Path, canopen_path: Path) -> _FakeComposition:
        events.append(("load", (ethercat_path, canopen_path)))
        return _FakeComposition()

    def _validate(_composition: _FakeComposition, path: Path) -> None:
        events.append(("validate", path))

    monkeypatch.setattr(launch_module, "Node", _RecordingNode)
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda package_name: str(installed_shares[package_name]),
    )
    monkeypatch.setattr(launch_module, "load_hardware_variants", _load)
    monkeypatch.setattr(
        launch_module, "validate_controller_compatibility", _validate
    )

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_sim_time": "false",
            "use_mock_hardware": "true",
            "ethercat_variant": "test_ecat",
            "canopen_variant": "test_can",
            "start_plc": "false",
            "start_bms": "false",
        }
    )
    actions = launch_module._launch_setup(context)

    assert events[0] == (
        "load",
        (
            installed_shares["robot_hw_ethercat"]
            / "variants/test_ecat.yaml",
            installed_shares["robot_hw_canopen"] / "variants/test_can.yaml",
        ),
    )
    assert events[1] == (
        "validate",
        installed_shares["rt_control_bringup"] / "config/controllers.yaml",
    )
    assert events[2][0] == "node"
    required_active_controllers = [
        "joint_state_broadcaster",
        "rt_internal_state_broadcaster",
        "diff_drive_controller",
    ]
    ordered_controller_names = required_active_controllers + [
        "whole_body_jtc",
        "enable_manager",
    ]
    spawners = {
        action.kwargs.get("arguments", [None])[0]: action
        for action in node_actions
        if action.kwargs.get("package") == "controller_manager"
        and action.kwargs.get("executable") == "spawner"
    }
    assert set(spawners) == set(ordered_controller_names)
    top_level_spawners = [
        action.kwargs.get("arguments", [None])[0]
        for action in actions
        if isinstance(action, _RecordingNode)
        and action.kwargs.get("executable") == "spawner"
    ]
    assert top_level_spawners == ["joint_state_broadcaster"]

    for index, controller_name in enumerate(ordered_controller_names):
        spawner = spawners[controller_name]
        failed_exit = ProcessExited(
            action=spawner,
            name=f"{controller_name}_spawner",
            cmd=[],
            cwd=None,
            env=None,
            pid=1,
            returncode=1,
        )
        matching_handlers = [
            action.event_handler
            for action in actions
            if isinstance(action, RegisterEventHandler)
            and action.event_handler.matches(failed_exit)
        ]
        assert len(matching_handlers) == 1, controller_name

        failure_actions = matching_handlers[0].handle(failed_exit, LaunchContext())
        shutdown_events = [
            action.event
            for action in failure_actions or []
            if isinstance(action, EmitEvent) and isinstance(action.event, Shutdown)
        ]
        assert len(shutdown_events) == 1, controller_name
        assert controller_name in shutdown_events[0].reason
        failure_markers = [
            action
            for action in failure_actions or []
            if isinstance(action, OpaqueFunction)
        ]
        assert len(failure_markers) == 1, controller_name

        successful_exit = ProcessExited(
            action=spawner,
            name=f"{controller_name}_spawner",
            cmd=[],
            cwd=None,
            env=None,
            pid=1,
            returncode=0,
        )
        success_actions = matching_handlers[0].handle(
            successful_exit, LaunchContext()
        )
        if index + 1 < len(ordered_controller_names):
            expected_next = ordered_controller_names[index + 1]
            assert success_actions == [spawners[expected_next]], controller_name
        else:
            assert success_actions == [], controller_name

    diagnostics = next(
        node
        for node in nodes
        if node.get("package") == "rt_diagnostics"
        and node.get("executable") == "rt_diagnostics_node"
    )
    diagnostics_mappings = [
        parameter
        for parameter in diagnostics["parameters"]
        if isinstance(parameter, dict)
    ]
    merged_diagnostics = {
        key: value for mapping in diagnostics_mappings for key, value in mapping.items()
    }
    assert {
        key: merged_diagnostics[key] for key in expected_parameters
    } == expected_parameters

    topology_keys = set(expected_parameters)
    for node in nodes:
        if node is diagnostics:
            continue
        parameter_keys = {
            key
            for parameter in node.get("parameters", [])
            if isinstance(parameter, dict)
            for key in parameter
        }
        assert topology_keys.isdisjoint(parameter_keys), node.get("executable")

    status_adapter = next(
        node
        for node in nodes
        if node.get("package") == "control_api_adapter"
        and node.get("executable") == "rt_status_adapter"
    )
    assert all(
        topology_keys.isdisjoint(parameter)
        for parameter in status_adapter["parameters"]
        if isinstance(parameter, dict)
    )
    assert module is not None


def test_required_spawner_failure_makes_launch_service_fail() -> None:
    spec = importlib.util.spec_from_file_location(
        "rt_control_launch_spawner_failure_for_test", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    spawner = ExecuteProcess(cmd=["/bin/sh", "-c", "exit 7"])
    handler = RegisterEventHandler(
        launch_module.OnProcessExit(
            target_action=spawner,
            on_exit=launch_module._start_next_spawner_or_stop(
                "test_controller", "ACTIVE", None
            ),
        )
    )
    launch_service = LaunchService()
    launch_service.include_launch_description(
        LaunchDescription([handler, spawner])
    )

    assert launch_service.run() != 0


@pytest.mark.parametrize(
    "value",
    (
        "True",
        "0",
        " false",
        "false ",
        "false canopen_variant_dir:=/tmp/untrusted",
    ),
)
def test_launch_rejects_noncanonical_mock_selector_before_package_lookup(
    value: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    spec = importlib.util.spec_from_file_location(
        "rt_control_launch_invalid_mock_for_test", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)
    monkeypatch.setattr(
        launch_module,
        "get_package_share_directory",
        lambda _package: pytest.fail("package lookup happened before validation"),
    )
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_mock_hardware": value,
            "ethercat_variant": "alfa_v1",
            "canopen_variant": "alfa_v1",
        }
    )

    with pytest.raises(ValueError, match="use_mock_hardware must be true or false"):
        launch_module._launch_setup(context)


def test_build_installs_python_loader_config_and_declares_runtime_dependencies() -> None:
    cmake = CMAKE_FILE.read_text(encoding="utf-8")
    manifest = ET.parse(PACKAGE_MANIFEST).getroot()
    dependencies = {
        element.text
        for element in manifest
        if element.tag in {"buildtool_depend", "exec_depend"}
    }

    assert "find_package(ament_cmake_python REQUIRED)" in cmake
    assert "ament_python_install_package(${PROJECT_NAME})" in cmake
    assert "DIRECTORY launch config urdf" in cmake
    assert "test_hardware_composition_config.py" in cmake
    assert dependencies >= {"ament_cmake_python", "ament_index_python", "python3-yaml"}
    assert (PYTHON_PACKAGE / "__init__.py").is_file()
    assert (PYTHON_PACKAGE / "hardware_composition.py").is_file()


def test_controller_preflight_accepts_the_canonical_composition() -> None:
    module = _implementation()
    composition = module.load_hardware_variants(
        CANONICAL_ETHERCAT_VARIANT, CANONICAL_CANOPEN_VARIANT
    )

    module.validate_controller_compatibility(composition, CONTROLLERS_CONFIG)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("jsb_order", "joint_state_broadcaster"),
        ("jtc_missing", "whole_body_jtc"),
        ("jtc_velocity", "command_interfaces"),
        ("tolerance_count", "position_tolerances"),
        ("batch_missing", "enable batches"),
        ("ready_unknown", "disable terminal"),
        ("left_wrong", "left_wheel_names"),
        ("right_wrong", "right_wheel_names"),
        ("wheels_per_side", "wheels_per_side"),
        ("wheels_per_side_bool", "wheels_per_side"),
    ],
)
def test_controller_preflight_rejects_selected_variant_mismatch(
    tmp_path: Path, mutation: str, message: str
) -> None:
    module = _implementation()
    composition = module.load_hardware_variants(
        CANONICAL_ETHERCAT_VARIANT, CANONICAL_CANOPEN_VARIANT
    )
    controllers = yaml.safe_load(CONTROLLERS_CONFIG.read_text(encoding="utf-8"))

    if mutation == "jsb_order":
        controllers["joint_state_broadcaster"]["ros__parameters"]["joints"].reverse()
    elif mutation == "jtc_missing":
        controllers["whole_body_jtc"]["ros__parameters"]["joints"].pop()
    elif mutation == "jtc_velocity":
        controllers["whole_body_jtc"]["ros__parameters"][
            "command_interfaces"
        ] = ["velocity"]
    elif mutation == "tolerance_count":
        controllers["whole_body_jtc"]["ros__parameters"][
            "trajectory_start_consistency_check"
        ]["position_tolerances"].pop()
    elif mutation == "batch_missing":
        controllers["enable_manager"]["ros__parameters"][
            "enable_batch_joint_names"
        ].pop()
    elif mutation == "ready_unknown":
        controllers["enable_manager"]["ros__parameters"][
            "ready_to_switch_on_disable_terminal_joints"
        ].append("unknown_axis")
    elif mutation == "left_wrong":
        controllers["diff_drive_controller"]["ros__parameters"][
            "left_wheel_names"
        ] = ["right_track_joint"]
    elif mutation == "right_wrong":
        controllers["diff_drive_controller"]["ros__parameters"][
            "right_wheel_names"
        ] = ["left_track_joint"]
    elif mutation == "wheels_per_side":
        controllers["diff_drive_controller"]["ros__parameters"][
            "wheels_per_side"
        ] = 2
    elif mutation == "wheels_per_side_bool":
        controllers["diff_drive_controller"]["ros__parameters"][
            "wheels_per_side"
        ] = True

    path = tmp_path / "controllers.yaml"
    path.write_text(yaml.safe_dump(controllers, sort_keys=False), encoding="utf-8")
    with pytest.raises(module.HardwareCompositionError, match=message):
        module.validate_controller_compatibility(composition, path)


def _valid_ethercat_variant() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "variant": "test_ecat",
        "system": "ecat_arms",
        "extra_responders": [
            {"ring_position": 0},
            {"ring_position": 4},
        ],
        "sensors": [
            {
                "sensor_name": "force_a",
                "family": "x503",
                "ring_position": 2,
                "profile": "x503_a",
            },
        ],
        "axes": [
            {
                "joint_name": "arm_a",
                "family": "zeroerr",
                "ring_position": 1,
                "profile": "zeroerr_a",
                "mode_of_operation": 8,
            },
            {
                "joint_name": "arm_b",
                "family": "ti5",
                "ring_position": 3,
                "profile": "ti5_b",
                "mode_of_operation": 8,
            },
        ],
    }


def _valid_canopen_variant() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "variant": "test_can",
        "system": "canopen_mobile_axes",
        "nodes": [
            {
                "joint_name": "track_left",
                "node_id": 2,
                "operation_mode": 3,
                "side": "left",
                "profile": "ld2_drive",
            },
            {
                "joint_name": "track_right",
                "node_id": 3,
                "operation_mode": 3,
                "side": "right",
                "profile": "ld2_drive",
            },
        ],
    }


def _write_variant(path: Path, document: dict[str, Any]) -> Path:
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def test_hardware_composition_is_derived_from_hardware_owned_variants(
    tmp_path: Path,
) -> None:
    module = _implementation()
    ethercat_path = _write_variant(
        tmp_path / "test_ecat.yaml", _valid_ethercat_variant()
    )
    canopen_path = _write_variant(
        tmp_path / "test_can.yaml", _valid_canopen_variant()
    )

    composition = module.load_hardware_variants(ethercat_path, canopen_path)

    assert composition.ethercat.expected_responders == 5
    assert [
        (axis.joint_name, axis.ring_position)
        for axis in composition.ethercat.axes
    ] == [("arm_a", 1), ("arm_b", 3)]
    assert [
        (sensor.sensor_name, sensor.ring_position)
        for sensor in composition.ethercat.sensors
    ] == [("force_a", 2)]
    assert [
        (node.joint_name, node.node_id) for node in composition.canopen.nodes
    ] == [("track_left", 2), ("track_right", 3)]


@pytest.mark.parametrize(
    "variant",
    ["", "../alfa_v1", "alfa-v1", "/tmp/alfa_v1", "ALFA_V1"],
)
def test_variant_descriptor_path_rejects_uncontrolled_names(
    tmp_path: Path, variant: str
) -> None:
    module = _implementation()

    with pytest.raises(module.HardwareCompositionError, match="variant"):
        module.variant_descriptor_path(tmp_path, variant)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("filename_mismatch", "filename"),
        ("wrong_ecat_system", "ecat_arms"),
        ("wrong_can_system", "canopen_mobile_axes"),
        ("bad_schema_type", "integer"),
        ("empty_axes", "at least one axis"),
        ("empty_nodes", "at least one node"),
        ("bad_joint_name", "joint_name"),
        ("duplicate_joint", "joint_name"),
        ("duplicate_ring", "ring_position"),
        ("duplicate_sensor", "sensor_name"),
        ("cross_resource_name", "resource names"),
        ("incomplete_ring", "complete 0-based ring"),
        ("bool_ring", "integer"),
        ("bad_side", "left.*right"),
        ("bool_node", "integer"),
        ("duplicate_node", "node_id"),
        ("cross_bus_joint", "across buses"),
    ],
)
def test_hardware_variant_descriptors_fail_closed(
    tmp_path: Path, mutation: str, message: str
) -> None:
    module = _implementation()
    ethercat = _valid_ethercat_variant()
    canopen = _valid_canopen_variant()
    ethercat_filename = "test_ecat.yaml"

    if mutation == "filename_mismatch":
        ethercat_filename = "different.yaml"
    elif mutation == "wrong_ecat_system":
        ethercat["system"] = "other"
    elif mutation == "wrong_can_system":
        canopen["system"] = "other"
    elif mutation == "bad_schema_type":
        ethercat["schema_version"] = True
    elif mutation == "empty_axes":
        ethercat["axes"] = []
    elif mutation == "empty_nodes":
        canopen["nodes"] = []
    elif mutation == "bad_joint_name":
        ethercat["axes"][0]["joint_name"] = " arm_a"
    elif mutation == "duplicate_joint":
        ethercat["axes"][1]["joint_name"] = "arm_a"
    elif mutation == "duplicate_ring":
        ethercat["extra_responders"][0]["ring_position"] = 1
    elif mutation == "duplicate_sensor":
        ethercat["sensors"].append(copy.deepcopy(ethercat["sensors"][0]))
    elif mutation == "cross_resource_name":
        ethercat["sensors"][0]["sensor_name"] = "arm_a"
    elif mutation == "incomplete_ring":
        ethercat["extra_responders"][1]["ring_position"] = 5
    elif mutation == "bool_ring":
        ethercat["axes"][0]["ring_position"] = True
    elif mutation == "bad_side":
        canopen["nodes"][0]["side"] = "front"
    elif mutation == "bool_node":
        canopen["nodes"][0]["node_id"] = True
    elif mutation == "duplicate_node":
        canopen["nodes"][1]["node_id"] = 2
    elif mutation == "cross_bus_joint":
        canopen["nodes"][0]["joint_name"] = "arm_a"

    ethercat_path = _write_variant(tmp_path / ethercat_filename, ethercat)
    canopen_path = _write_variant(tmp_path / "test_can.yaml", canopen)

    with pytest.raises(module.HardwareCompositionError, match=message):
        module.load_hardware_variants(ethercat_path, canopen_path)


def test_composition_projection_ignores_hardware_private_metadata(
    tmp_path: Path,
) -> None:
    module = _implementation()
    ethercat = _valid_ethercat_variant()
    canopen = _valid_canopen_variant()
    ethercat["profile_registry_revision"] = "owner-private"
    ethercat["extra_responders"][0]["identity"] = "owner-private"
    ethercat["axes"][0]["certification_evidence"] = "owner-private"
    ethercat["sensors"][0]["calibration_state"] = "owner-private"
    canopen["bus_generation_policy"] = "owner-private"
    canopen["nodes"][0]["dcf_identity"] = "owner-private"

    composition = module.load_hardware_variants(
        _write_variant(tmp_path / "test_ecat.yaml", ethercat),
        _write_variant(tmp_path / "test_can.yaml", canopen),
    )

    assert [axis.joint_name for axis in composition.ethercat.axes] == [
        "arm_a",
        "arm_b",
    ]
    assert [sensor.sensor_name for sensor in composition.ethercat.sensors] == [
        "force_a"
    ]
    assert [node.joint_name for node in composition.canopen.nodes] == [
        "track_left",
        "track_right",
    ]


def test_variant_loader_rejects_duplicate_and_non_scalar_yaml_keys(
    tmp_path: Path,
) -> None:
    module = _implementation()
    canopen_path = _write_variant(
        tmp_path / "test_can.yaml", _valid_canopen_variant()
    )
    duplicate_path = _write_variant(
        tmp_path / "test_ecat.yaml", _valid_ethercat_variant()
    )
    duplicate_path.write_text(
        duplicate_path.read_text(encoding="utf-8") + "system: ecat_arms\n",
        encoding="utf-8",
    )

    with pytest.raises(module.HardwareCompositionError, match="duplicate"):
        module.load_hardware_variants(duplicate_path, canopen_path)

    duplicate_path.write_text("? [not, scalar]\n: rejected\n", encoding="utf-8")
    with pytest.raises(module.HardwareCompositionError, match="scalar"):
        module.load_hardware_variants(duplicate_path, canopen_path)


def test_variant_loader_rejects_missing_descriptor(tmp_path: Path) -> None:
    module = _implementation()
    canopen_path = _write_variant(
        tmp_path / "test_can.yaml", _valid_canopen_variant()
    )

    with pytest.raises(module.HardwareCompositionError, match="could not read"):
        module.load_hardware_variants(tmp_path / "test_ecat.yaml", canopen_path)
