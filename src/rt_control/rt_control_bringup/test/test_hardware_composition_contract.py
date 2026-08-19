"""ELECTRI-94 characterization and target-ownership tests.

The expanded graphs below freeze the component names and hardware resources.
EtherCAT mock mode intentionally has the same declared interface schema as
real mode; CANopen keeps its documented dynamic-real/static-mock difference.
The final two tests describe the Franka-style ownership boundary: a hardware
package owns both its real and mock system macro, while bringup only composes
those macros.
"""

from __future__ import annotations

import ast
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
BRINGUP_DIR = REPOSITORY_ROOT / "src/rt_control/rt_control_bringup"
BRINGUP_XACRO = BRINGUP_DIR / "urdf/rt_control.urdf.xacro"
BRINGUP_LAUNCH = BRINGUP_DIR / "launch/rt_control.launch.py"
ETHERCAT_VARIANT_CONFIG = (
    REPOSITORY_ROOT / "src/rt_control/robot_hw_ethercat/variants/alfa_v1.yaml"
)
CANOPEN_VARIANT_CONFIG = (
    REPOSITORY_ROOT / "src/rt_control/robot_hw_canopen/variants/alfa_v1.yaml"
)
CONTROLLERS_CONFIG = BRINGUP_DIR / "config/controllers.yaml"
LEGACY_BRINGUP_MOCK_XACRO = BRINGUP_DIR / "urdf/mock.ros2_control.xacro"
ECAT_TYPED_CONFIG_PATCH = (
    REPOSITORY_ROOT
    / "patches/ecat_icube/0004-use-component-parameters-for-ec-modules.patch"
)
CANOPEN_DERIVED_TOPOLOGY_PATCH = (
    REPOSITORY_ROOT
    / "patches/ros2_canopen/0005-derive-motor-topology-from-hardware-info.patch"
)
CANOPEN_BUS_CONFIG = (
    REPOSITORY_ROOT / "src/rt_control/robot_hw_canopen/config/bus.yml"
)

sys.path.insert(0, str(BRINGUP_DIR))
from rt_control_bringup.hardware_composition import load_hardware_variants  # noqa: E402

PACKAGE_DIRS = {
    "robot_description": REPOSITORY_ROOT / "src/description/robot_description",
    "robot_hw_ethercat": REPOSITORY_ROOT / "src/rt_control/robot_hw_ethercat",
    "robot_hw_canopen": REPOSITORY_ROOT / "src/rt_control/robot_hw_canopen",
    "rt_control_bringup": BRINGUP_DIR,
}

HARDWARE_CONTRACTS = {
    "robot_hw_ethercat": {
        "component": "ecat_arms",
        "real_plugin": "ethercat_driver/EthercatDriver",
    },
    "robot_hw_canopen": {
        "component": "canopen_mobile_axes",
        "real_plugin": "canopen_ros2_control/Cia402System",
    },
}
MOCK_PLUGIN = "mock_components/GenericSystem"
XACRO_NAMESPACE = "http://www.ros.org/wiki/xacro"
XACRO_MACRO_TAG = f"{{{XACRO_NAMESPACE}}}macro"

ECAT_JOINTS = (
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
)
ECAT_DIGITAL_IO_JOINTS = frozenset(
    {
        "right_joint1",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "left_joint1",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "turn",
    }
)
CANOPEN_JOINTS = ("left_track_joint", "right_track_joint")
ECAT_SENSOR_INTERFACES = {
    "ethercat_domain": ("process_data_age_ms",),
    "ethercat_master": ("link_up", "slaves_responding", "wc_error_count"),
    **{
        f"ethercat_slave_{ring_position}": ("al_state",)
        for ring_position in (*range(1, 13), 14, 15)
    },
}
ECAT_AXIS_BINDINGS = (
    ("right_joint1", 1, "zeroerr_j1"),
    ("right_joint2", 2, "ti5_right_joint2"),
    ("right_joint3", 3, "ti5_j3"),
    ("right_joint4", 4, "zeroerr_right_joint4"),
    ("right_joint5", 5, "zeroerr_right_joint5"),
    ("right_joint6", 6, "right_j6"),
    ("left_joint1", 7, "zeroerr_j1"),
    ("left_joint2", 8, "ti5_j2"),
    ("left_joint3", 9, "ti5_left_joint3"),
    ("left_joint4", 10, "zeroerr_j4"),
    ("left_joint5", 11, "zeroerr_left_joint5"),
    ("left_joint6", 12, "left_j6"),
    ("turn", 14, "turn"),
    ("updown", 15, "xmc_updown_sw511"),
)


def _create_source_overlay(prefix: Path) -> None:
    """Expose source package shares to ``$(find package)`` without a build."""

    package_index = prefix / "share/ament_index/resource_index/packages"
    package_index.mkdir(parents=True)
    for package_name, package_dir in PACKAGE_DIRS.items():
        (package_index / package_name).write_text("", encoding="utf-8")
        (prefix / "share" / package_name).symlink_to(
            package_dir, target_is_directory=True
        )


def _run_bringup_xacro(
    *,
    mock: bool,
    tmp_path: Path,
    ethercat_variant: str = "alfa_v1",
    canopen_variant: str = "alfa_v1",
) -> subprocess.CompletedProcess[str]:
    xacro = shutil.which("xacro")
    assert xacro is not None, "ROS 2 xacro must be installed to verify this contract"

    prefix = tmp_path / ("mock_overlay" if mock else "real_overlay")
    _create_source_overlay(prefix)
    environment = os.environ.copy()
    existing_prefixes = environment.get("AMENT_PREFIX_PATH")
    environment["AMENT_PREFIX_PATH"] = (
        f"{prefix}:{existing_prefixes}" if existing_prefixes else str(prefix)
    )
    result = subprocess.run(
        [
            xacro,
            str(BRINGUP_XACRO),
            f"use_mock_hardware:={'true' if mock else 'false'}",
            f"ethercat_variant:={ethercat_variant}",
            f"canopen_variant:={canopen_variant}",
            f"canopen_authority_bus:={CANOPEN_BUS_CONFIG}",
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    return result


def _expand_bringup(*, mock: bool, tmp_path: Path) -> ET.Element:
    result = _run_bringup_xacro(mock=mock, tmp_path=tmp_path)
    assert result.returncode == 0, (
        f"xacro expansion failed for mock={mock}:\n{result.stderr}"
    )
    return ET.fromstring(result.stdout)


@pytest.fixture(scope="module")
def expanded_real(tmp_path_factory: pytest.TempPathFactory) -> ET.Element:
    return _expand_bringup(mock=False, tmp_path=tmp_path_factory.mktemp("real"))


@pytest.fixture(scope="module")
def expanded_mock(tmp_path_factory: pytest.TempPathFactory) -> ET.Element:
    return _expand_bringup(mock=True, tmp_path=tmp_path_factory.mktemp("mock"))


def _systems(root: ET.Element) -> dict[str, ET.Element]:
    systems = root.findall(".//ros2_control")
    names = [system.attrib.get("name") for system in systems]
    assert len(names) == len(set(names)), f"duplicate ros2_control names: {names}"
    return {system.attrib["name"]: system for system in systems}


def _interfaces(resource: ET.Element, kind: str) -> tuple[str, ...]:
    return tuple(
        interface.attrib["name"]
        for interface in resource.findall(f"{kind}_interface")
    )


def _joints(system: ET.Element) -> dict[str, ET.Element]:
    return {joint.attrib["name"]: joint for joint in system.findall("joint")}


def _sensors(system: ET.Element) -> dict[str, ET.Element]:
    return {sensor.attrib["name"]: sensor for sensor in system.findall("sensor")}


def _parameters(element: ET.Element) -> dict[str, str]:
    parameters = element.findall("param")
    names = [parameter.attrib["name"] for parameter in parameters]
    assert len(names) == len(set(names)), f"duplicate parameters: {names}"
    return {
        parameter.attrib["name"]: (parameter.text or "").strip()
        for parameter in parameters
    }


def _added_patch_text(path: Path) -> str:
    assert path.is_file(), f"missing frozen upstream patch: {path}"
    return "\n".join(
        line[1:]
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )


class _DuplicateKeyRejectingLoader(yaml.SafeLoader):
    """Safe YAML loader that fails before duplicate keys can overwrite data."""


def _construct_unique_mapping(
    loader: _DuplicateKeyRejectingLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict[object, object]:
    loader.flatten_mapping(node)
    mapping: dict[object, object] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        assert key not in mapping, f"duplicate YAML key: {key!r}"
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_DuplicateKeyRejectingLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _load_canopen_bus_topology(bus_yaml: str) -> dict[str, tuple[int, int]]:
    try:
        document = yaml.load(bus_yaml, Loader=_DuplicateKeyRejectingLoader)
    except yaml.YAMLError as error:
        raise AssertionError(f"invalid CANopen bus YAML: {error}") from error

    assert isinstance(document, dict), "CANopen bus YAML root must be a mapping"
    nodes = document.get("nodes")
    assert isinstance(nodes, dict), "CANopen bus YAML must contain a nodes mapping"

    topology: dict[str, tuple[int, int]] = {}
    for joint_name, configuration in nodes.items():
        assert isinstance(joint_name, str), "CANopen node names must be strings"
        assert isinstance(configuration, dict), (
            f"CANopen node {joint_name!r} configuration must be a mapping"
        )
        for field_name in ("node_id", "operation_mode"):
            assert field_name in configuration, (
                f"CANopen node {joint_name!r} is missing {field_name}"
            )
            assert type(configuration[field_name]) is int, (
                f"CANopen node {joint_name!r} {field_name} must be an integer"
            )
        topology[joint_name] = (
            configuration["node_id"],
            configuration["operation_mode"],
        )
    return topology


def _assert_canopen_topology_alignment(
    expanded_topology: dict[str, tuple[int, int]], bus_yaml: str
) -> None:
    bus_topology = _load_canopen_bus_topology(bus_yaml)
    expanded_names = set(expanded_topology)
    bus_names = set(bus_topology)

    missing = sorted(expanded_names - bus_names)
    extra = sorted(bus_names - expanded_names)
    assert not missing, f"missing bus nodes: {missing}"
    assert not extra, f"extra bus nodes: {extra}"

    for joint_name in sorted(expanded_names):
        expected = expanded_topology[joint_name]
        actual = bus_topology[joint_name]
        assert actual == expected, (
            f"CANopen topology mismatch for {joint_name}: "
            f"expected {expected}, found {actual}"
        )


@pytest.mark.parametrize("fixture_name", ("expanded_real", "expanded_mock"))
def test_bringup_composes_exactly_two_named_systems(
    fixture_name: str, request: pytest.FixtureRequest
) -> None:
    systems = _systems(request.getfixturevalue(fixture_name))

    assert set(systems) == {"ecat_arms", "canopen_mobile_axes"}
    assert {name: system.attrib["type"] for name, system in systems.items()} == {
        "ecat_arms": "system",
        "canopen_mobile_axes": "system",
    }


def test_real_ethercat_joint_and_sensor_interface_contract(
    expanded_real: ET.Element,
) -> None:
    system = _systems(expanded_real)["ecat_arms"]
    assert system.findtext("hardware/plugin") == "ethercat_driver/EthercatDriver"

    joints = _joints(system)
    assert tuple(joints) == ECAT_JOINTS
    for joint_name, joint in joints.items():
        if joint_name in ECAT_DIGITAL_IO_JOINTS:
            expected_command = ("position", "digital_outputs", "control_word")
            expected_state = ("position", "digital_inputs", "status_word")
        else:
            expected_command = ("position", "control_word")
            expected_state = ("position", "status_word")
        assert _interfaces(joint, "command") == expected_command, joint_name
        assert _interfaces(joint, "state") == expected_state, joint_name

    sensors = _sensors(system)
    assert tuple(sensors) == tuple(ECAT_SENSOR_INTERFACES)
    assert {
        name: _interfaces(sensor, "state") for name, sensor in sensors.items()
    } == ECAT_SENSOR_INTERFACES


def test_real_ethercat_driver_configuration_contract(
    expanded_real: ET.Element,
) -> None:
    system = _systems(expanded_real)["ecat_arms"]
    assert _parameters(system.find("hardware")) == {
        "master_id": "0",
        "control_frequency": "250",
        "startup_bus_timeout_ms": "70000",
        "preload_timeout_ms": "5000",
    }

    actual_bindings = []
    for joint_name, joint in _joints(system).items():
        assert joint.findall("ec_module") == [], (
            "EtherCAT configuration must arrive through typed HardwareInfo "
            "component parameters, not a second parse of original_xml"
        )
        parameters = _parameters(joint)
        assert parameters["ec_module.name"] == f"{joint_name}_drive"
        assert parameters["ec_module.plugin"] == (
            "ethercat_generic_plugins/EcCiA402Drive"
        )
        assert parameters["ec_module.alias"] == "0"
        assert parameters["ec_module.mode_of_operation"] == "8"
        config_path = Path(parameters["ec_module.slave_config"])
        assert config_path.parent.name == "slaves"
        actual_bindings.append(
            (
                joint_name,
                int(parameters["ec_module.position"]),
                config_path.stem,
            )
        )
    assert tuple(actual_bindings) == ECAT_AXIS_BINDINGS


def test_mock_ethercat_joint_and_sensor_interface_contract(
    expanded_mock: ET.Element,
) -> None:
    system = _systems(expanded_mock)["ecat_arms"]
    assert system.findtext("hardware/plugin") == MOCK_PLUGIN

    joints = _joints(system)
    assert tuple(joints) == ECAT_JOINTS
    for joint_name, joint in joints.items():
        if joint_name in ECAT_DIGITAL_IO_JOINTS:
            expected_command = ("position", "digital_outputs", "control_word")
            expected_state = ("position", "digital_inputs", "status_word")
        else:
            expected_command = ("position", "control_word")
            expected_state = ("position", "status_word")
        assert _interfaces(joint, "command") == expected_command, joint_name
        assert _interfaces(joint, "state") == expected_state, joint_name

    sensors = _sensors(system)
    assert tuple(sensors) == tuple(ECAT_SENSOR_INTERFACES)
    assert {
        name: _interfaces(sensor, "state") for name, sensor in sensors.items()
    } == ECAT_SENSOR_INTERFACES
    assert system.findall(".//ec_module") == []
    for joint in joints.values():
        assert _parameters(joint.find("state_interface[@name='position']")) == {
            "initial_value": "0.0"
        }
        assert _parameters(joint.find("state_interface[@name='status_word']")) == {
            "initial_value": "64.0"
        }


def test_real_and_mock_canopen_joint_interface_contract(
    expanded_real: ET.Element, expanded_mock: ET.Element
) -> None:
    real_system = _systems(expanded_real)["canopen_mobile_axes"]
    mock_system = _systems(expanded_mock)["canopen_mobile_axes"]
    assert real_system.findtext("hardware/plugin") == (
        "canopen_ros2_control/Cia402System"
    )
    assert mock_system.findtext("hardware/plugin") == MOCK_PLUGIN

    real_joints = _joints(real_system)
    mock_joints = _joints(mock_system)
    assert tuple(real_joints) == CANOPEN_JOINTS
    assert tuple(mock_joints) == CANOPEN_JOINTS

    # Cia402System exports these interfaces dynamically from the generated bus
    # configuration; their absence in the real URDF is therefore intentional.
    assert {
        name: (_interfaces(joint, "command"), _interfaces(joint, "state"))
        for name, joint in real_joints.items()
    } == {name: ((), ()) for name in CANOPEN_JOINTS}
    assert {
        name: (_interfaces(joint, "command"), _interfaces(joint, "state"))
        for name, joint in mock_joints.items()
    } == {
        name: (("velocity",), ("position", "velocity"))
        for name in CANOPEN_JOINTS
    }

    hardware_parameters = _parameters(real_system.find("hardware"))
    assert hardware_parameters["can_interface_name"] == "can0"
    for parameter_name, file_name in (
        ("bus_config", "bus.yml"),
        ("master_config", "master.dcf"),
        ("master_bin", "master.bin"),
    ):
        config_path = Path(hardware_parameters[parameter_name])
        assert config_path.is_absolute()
        assert config_path.parts[-4:] == (
            "share",
            "robot_hw_canopen",
            "config",
            file_name,
        )
    assert {
        name: _parameters(joint) for name, joint in real_joints.items()
    } == {
        "left_track_joint": {"node_id": "2", "operation_mode": "3"},
        "right_track_joint": {"node_id": "3", "operation_mode": "3"},
    }
    assert {
        name: _parameters(joint) for name, joint in mock_joints.items()
    } == {
        "left_track_joint": {"node_id": "2", "operation_mode": "3"},
        "right_track_joint": {"node_id": "3", "operation_mode": "3"},
    }


def _system_macro_plugins(package_dir: Path) -> tuple[list[str], list[str]]:
    macro_names: list[str] = []
    plugins: list[str] = []
    for xacro_path in sorted((package_dir / "urdf").glob("*.xacro")):
        root = ET.parse(xacro_path).getroot()
        for macro in root.findall(f".//{XACRO_MACRO_TAG}"):
            if not macro.findall(".//ros2_control"):
                continue
            macro_names.append(macro.attrib["name"])
            plugins.extend(
                plugin.text.strip()
                for plugin in macro.findall(".//ros2_control/hardware//plugin")
                if plugin.text and plugin.text.strip()
            )
    return macro_names, plugins


@pytest.mark.parametrize("package_name", tuple(HARDWARE_CONTRACTS))
def test_hardware_packages_expose_callable_real_and_mock_system_macros(
    package_name: str,
) -> None:
    package_dir = PACKAGE_DIRS[package_name]
    contract = HARDWARE_CONTRACTS[package_name]
    macro_names, plugins = _system_macro_plugins(package_dir)

    assert macro_names, (
        f"{package_name} must wrap its ros2_control system in a callable xacro macro"
    )
    assert contract["real_plugin"] in plugins, (
        f"{package_name} system macro must own its real hardware plugin branch"
    )
    assert MOCK_PLUGIN in plugins, (
        f"{package_name} system macro must own its mock hardware plugin branch"
    )


def test_bringup_owns_composition_only_not_raw_mock_hardware_schema() -> None:
    assert not LEGACY_BRINGUP_MOCK_XACRO.exists(), (
        "raw mock ros2_control schema belongs in robot_hw_* packages, not bringup"
    )

    raw_system_files = []
    for xacro_path in sorted((BRINGUP_DIR / "urdf").glob("*.xacro")):
        if ET.parse(xacro_path).getroot().findall(".//ros2_control"):
            raw_system_files.append(xacro_path.name)
    assert raw_system_files == [], (
        f"bringup must only compose hardware macros; raw systems found in {raw_system_files}"
    )

    wrapper = BRINGUP_XACRO.read_text(encoding="utf-8")
    assert "$(find robot_hw_ethercat)" in wrapper
    assert "$(find robot_hw_canopen)" in wrapper
    assert "$(find rt_control_bringup)/urdf/mock.ros2_control.xacro" not in wrapper


def test_bringup_declares_and_passes_through_explicit_hardware_variants() -> None:
    root = ET.parse(BRINGUP_XACRO).getroot()
    argument_tag = f"{{{XACRO_NAMESPACE}}}arg"
    arguments = {
        argument.attrib["name"]: argument.attrib.get("default")
        for argument in root.findall(argument_tag)
    }
    assert arguments["ethercat_variant"] == "alfa_v1"
    assert arguments["canopen_variant"] == "alfa_v1"

    ethercat_call = root.find(f"{{{XACRO_NAMESPACE}}}rt_control_ethercat_system")
    canopen_call = root.find(f"{{{XACRO_NAMESPACE}}}rt_control_canopen_system")
    assert ethercat_call is not None
    assert canopen_call is not None
    assert ethercat_call.attrib["variant"] == "$(arg ethercat_variant)"
    assert canopen_call.attrib["variant"] == "$(arg canopen_variant)"


def test_launch_declares_and_forwards_hardware_variant_arguments() -> None:
    launch_source = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    launch_tree = ast.parse(launch_source)

    launch_configurations = {}
    declared_defaults = {}
    command_calls = []
    for node in ast.walk(launch_tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id == "LaunchConfiguration":
            parent_assignment = next(
                (
                    assignment
                    for assignment in ast.walk(launch_tree)
                    if isinstance(assignment, ast.Assign)
                    and assignment.value is node
                    and len(assignment.targets) == 1
                    and isinstance(assignment.targets[0], ast.Name)
                ),
                None,
            )
            if (
                parent_assignment is not None
                and node.args
                and isinstance(node.args[0], ast.Constant)
            ):
                launch_configurations[parent_assignment.targets[0].id] = (
                    node.args[0].value
                )
        elif node.func.id == "DeclareLaunchArgument":
            if not node.args or not isinstance(node.args[0], ast.Constant):
                continue
            default = next(
                (
                    keyword.value.value
                    for keyword in node.keywords
                    if keyword.arg == "default_value"
                    and isinstance(keyword.value, ast.Constant)
                ),
                None,
            )
            declared_defaults[node.args[0].value] = default
        elif node.func.id == "Command":
            command_calls.append(node)

    for variant_name in ("ethercat_variant", "canopen_variant"):
        assert launch_configurations[variant_name] == variant_name
        assert declared_defaults[variant_name] == "alfa_v1"

    assert len(command_calls) == 1
    command_arguments = command_calls[0].args[0]
    assert isinstance(command_arguments, ast.List)
    for variant_name in ("ethercat_variant", "canopen_variant"):
        label_index = next(
            index
            for index, argument in enumerate(command_arguments.elts)
            if isinstance(argument, ast.Constant)
            and argument.value == f" {variant_name}:="
        )
        forwarded_value = command_arguments.elts[label_index + 1]
        assert isinstance(forwarded_value, ast.Name)
        assert forwarded_value.id == variant_name


@pytest.mark.parametrize(
    ("ethercat_variant", "canopen_variant", "rejected_variant"),
    (
        ("not_a_production_variant", "alfa_v1", "not_a_production_variant"),
        ("alfa_v1", "not_a_production_variant", "not_a_production_variant"),
    ),
    ids=("ethercat", "canopen"),
)
def test_bringup_fails_closed_on_unknown_hardware_variant(
    ethercat_variant: str,
    canopen_variant: str,
    rejected_variant: str,
    tmp_path: Path,
) -> None:
    result = _run_bringup_xacro(
        mock=True,
        tmp_path=tmp_path,
        ethercat_variant=ethercat_variant,
        canopen_variant=canopen_variant,
    )

    assert result.returncode != 0
    assert rejected_variant in result.stderr


def test_ethercat_upstream_patch_consumes_component_parameters() -> None:
    added = _added_patch_text(ECAT_TYPED_CONFIG_PATCH)

    assert "getEcModuleParam(info_.joints[j])" in added
    assert "getEcModuleParam(info_.gpios[g])" in added
    assert "getEcModuleParam(info_.sensors[s])" in added
    assert "kEcModuleParameterPrefix" in added
    assert '"ec_module."' in added
    assert "original_xml" not in added
    assert "tinyxml2" not in added


def test_canopen_upstream_patch_derives_every_lifecycle_consumer_from_hardware_info() -> None:
    added = _added_patch_text(CANOPEN_DERIVED_TOPOLOGY_PATCH)

    assert "motor_configurations_" in added
    assert "isConfiguredMotorNode(id)" in added
    assert "for (const auto & configuration : motor_configurations_)" in added
    assert "for (const uint8_t node_id : {2U, 3U})" not in added
    assert "id != 2U && id != 3U" not in added
    assert "configurations.size() != 2U" not in added


def test_real_canopen_joint_topology_matches_bus_configuration(
    expanded_real: ET.Element,
) -> None:
    system = _systems(expanded_real)["canopen_mobile_axes"]
    expanded_topology = {
        joint_name: (
            int(_parameters(joint)["node_id"]),
            int(_parameters(joint)["operation_mode"]),
        )
        for joint_name, joint in _joints(system).items()
    }

    _assert_canopen_topology_alignment(
        expanded_topology,
        CANOPEN_BUS_CONFIG.read_text(encoding="utf-8"),
    )


def test_diagnostics_composition_matches_expanded_hardware_graph(
    expanded_real: ET.Element, expanded_mock: ET.Element
) -> None:
    composition = load_hardware_variants(
        ETHERCAT_VARIANT_CONFIG, CANOPEN_VARIANT_CONFIG
    )
    real_systems = _systems(expanded_real)
    mock_systems = _systems(expanded_mock)

    ecat_topology = [
        (joint_name, int(_parameters(joint)["ec_module.position"]))
        for joint_name, joint in _joints(real_systems["ecat_arms"]).items()
    ]
    assert [
        (axis.joint_name, axis.ring_position)
        for axis in composition.ethercat.axes
    ] == ecat_topology

    master = _sensors(mock_systems["ecat_arms"])["ethercat_master"]
    responders = master.find(
        "state_interface[@name='slaves_responding']/param[@name='initial_value']"
    )
    assert responders is not None
    assert int(float(responders.text or "")) == composition.ethercat.expected_responders

    canopen_topology = [
        (joint_name, int(_parameters(joint)["node_id"]))
        for joint_name, joint in _joints(real_systems["canopen_mobile_axes"]).items()
    ]
    assert [
        (node.joint_name, node.node_id) for node in composition.canopen.nodes
    ] == canopen_topology


def test_selected_ethercat_joints_match_the_robot_model_logical_abi(
    expanded_real: ET.Element,
) -> None:
    composition = load_hardware_variants(
        ETHERCAT_VARIANT_CONFIG, CANOPEN_VARIANT_CONFIG
    )
    model_joints = {
        joint.attrib["name"]: joint.attrib["type"]
        for joint in expanded_real.findall("./joint")
        if joint.attrib.get("type") in {"revolute", "prismatic"}
    }
    selected_joints = {axis.joint_name for axis in composition.ethercat.axes}

    assert set(model_joints) == selected_joints
    assert model_joints["updown"] == "prismatic"
    assert all(
        joint_type == "revolute"
        for joint_name, joint_type in model_joints.items()
        if joint_name != "updown"
    )


def test_enable_manager_policy_covers_the_composed_ethercat_axes_once() -> None:
    composition = load_hardware_variants(
        ETHERCAT_VARIANT_CONFIG, CANOPEN_VARIANT_CONFIG
    )
    controllers = yaml.safe_load(CONTROLLERS_CONFIG.read_text(encoding="utf-8"))
    parameters = controllers["enable_manager"]["ros__parameters"]
    managed_joints = parameters["managed_joints"]
    flat_batches = parameters["enable_batch_joint_names"]
    batch_sizes = parameters["enable_batch_sizes"]
    ready_terminal = parameters["ready_to_switch_on_disable_terminal_joints"]

    expected_joints = [axis.joint_name for axis in composition.ethercat.axes]
    assert managed_joints == expected_joints
    assert controllers["whole_body_jtc"]["ros__parameters"]["joints"] == expected_joints
    assert sum(batch_sizes) == len(flat_batches)
    assert len(flat_batches) == len(set(flat_batches))
    assert set(flat_batches) == set(expected_joints)
    assert set(ready_terminal) == {
        "right_joint2",
        "right_joint3",
        "left_joint2",
        "left_joint3",
    }


def test_canopen_topology_alignment_is_independent_of_node_order() -> None:
    expanded_topology = {
        "left_track_joint": (2, 3),
        "right_track_joint": (3, 3),
    }
    reversed_bus_nodes = """
nodes:
  right_track_joint:
    operation_mode: 3
    node_id: 3
  left_track_joint:
    operation_mode: 3
    node_id: 2
"""

    _assert_canopen_topology_alignment(expanded_topology, reversed_bus_nodes)


@pytest.mark.parametrize(
    ("bus_nodes", "expected_error"),
    (
        (
            """
nodes:
  left_track_joint: {node_id: 2, operation_mode: 3}
""",
            "missing bus nodes.*right_track_joint",
        ),
        (
            """
nodes:
  left_track_joint: {node_id: 2, operation_mode: 3}
  right_track_joint: {node_id: 3, operation_mode: 3}
  spare_track_joint: {node_id: 4, operation_mode: 3}
""",
            "extra bus nodes.*spare_track_joint",
        ),
        (
            """
nodes:
  left_track_joint: {node_id: 9, operation_mode: 3}
  right_track_joint: {node_id: 3, operation_mode: 3}
""",
            r"left_track_joint.*expected.*\(2, 3\).*found.*\(9, 3\)",
        ),
        (
            """
nodes:
  left_track_joint: {node_id: 2, operation_mode: 1}
  right_track_joint: {node_id: 3, operation_mode: 3}
""",
            r"left_track_joint.*expected.*\(2, 3\).*found.*\(2, 1\)",
        ),
    ),
    ids=("missing", "extra", "node-id-mismatch", "mode-mismatch"),
)
def test_canopen_topology_alignment_fails_closed_on_drift(
    bus_nodes: str, expected_error: str
) -> None:
    expanded_topology = {
        "left_track_joint": (2, 3),
        "right_track_joint": (3, 3),
    }

    with pytest.raises(AssertionError, match=expected_error):
        _assert_canopen_topology_alignment(expanded_topology, bus_nodes)


def test_canopen_bus_yaml_rejects_duplicate_keys() -> None:
    duplicate_node = """
nodes:
  left_track_joint: {node_id: 2, operation_mode: 3}
  left_track_joint: {node_id: 3, operation_mode: 3}
"""

    with pytest.raises(AssertionError, match="duplicate YAML key.*left_track_joint"):
        _load_canopen_bus_topology(duplicate_node)
