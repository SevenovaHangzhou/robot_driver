"""Contracts for the hardware-owned CANopen variant registry."""

from copy import deepcopy
import importlib.util
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_DIR / "urdf/canopen.ros2_control.xacro"
VARIANT_DIR = PACKAGE_DIR / "variants"
VARIANT_PATH = VARIANT_DIR / "alfa_v1.yaml"
BUS_PATH = PACKAGE_DIR / "config/bus.yml"
VALIDATOR_PATH = PACKAGE_DIR / "cmake/validate_canopen_variant.py"
CMAKE_PATH = PACKAGE_DIR / "CMakeLists.txt"
XACRO_NAMESPACE = "http://www.ros.org/wiki/xacro"
MACRO_TAG = f"{{{XACRO_NAMESPACE}}}macro"
CANOPEN_JOINTS = ("left_track_joint", "right_track_joint")
NODE_FIELDS = {
    "joint_name",
    "node_id",
    "operation_mode",
    "side",
    "profile",
}

VALIDATOR_SPEC = importlib.util.spec_from_file_location(
    "robot_hw_canopen_variant_validator", VALIDATOR_PATH
)
assert VALIDATOR_SPEC is not None and VALIDATOR_SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(VALIDATOR_SPEC)
VALIDATOR_SPEC.loader.exec_module(VALIDATOR)


def _load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def _write_yaml(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def _expand(
    *,
    variant: str,
    mock: bool,
    tmp_path: Path,
    variant_dir: Path = VARIANT_DIR,
    config_dir: Path = PACKAGE_DIR / "config",
) -> subprocess.CompletedProcess[str]:
    xacro = shutil.which("xacro")
    assert xacro is not None, "ROS 2 xacro must be installed to verify this contract"

    wrapper = tmp_path / "canopen_variant_test.urdf.xacro"
    wrapper.write_text(
        f"""<?xml version="1.0"?>
<robot name="canopen_variant_test" xmlns:xacro="{XACRO_NAMESPACE}">
  <xacro:include filename="{XACRO_PATH}"/>
  <xacro:rt_control_canopen_system
    variant="{variant}"
    use_mock_hardware="{'true' if mock else 'false'}"/>
</robot>
""",
        encoding="utf-8",
    )
    return subprocess.run(
        [
            xacro,
            str(wrapper),
            f"canopen_config_dir:={config_dir}",
            f"canopen_authority_bus:={config_dir / 'bus.yml'}",
            f"canopen_variant_dir:={variant_dir}",
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def _system(result: subprocess.CompletedProcess[str]) -> ET.Element:
    assert result.returncode == 0, result.stderr
    root = ET.fromstring(result.stdout)
    systems = root.findall(".//ros2_control")
    assert len(systems) == 1
    return systems[0]


def _interfaces(joint: ET.Element, kind: str) -> tuple[str, ...]:
    return tuple(
        interface.attrib["name"]
        for interface in joint.findall(f"{kind}_interface")
    )


def _run_validator(
    *, variant_path: Path, bus_path: Path
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "python3",
            str(VALIDATOR_PATH),
            "--variant",
            str(variant_path),
            "--bus",
            str(bus_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def test_alfa_v1_descriptor_is_the_canopen_registration_point() -> None:
    descriptor = _load_yaml(VARIANT_PATH)

    assert set(descriptor) == {"schema_version", "variant", "system", "nodes"}
    assert descriptor["schema_version"] == 1
    assert descriptor["variant"] == "alfa_v1"
    assert descriptor["system"] == "canopen_mobile_axes"
    assert [set(node) for node in descriptor["nodes"]] == [NODE_FIELDS, NODE_FIELDS]
    assert descriptor["nodes"] == [
        {
            "joint_name": "left_track_joint",
            "node_id": 2,
            "operation_mode": 3,
            "side": "left",
            "profile": "ld2_drive",
        },
        {
            "joint_name": "right_track_joint",
            "node_id": 3,
            "operation_mode": 3,
            "side": "right",
            "profile": "ld2_drive",
        },
    ]


def test_public_macro_loads_variants_without_per_variant_xacro_macros() -> None:
    root = ET.parse(XACRO_PATH).getroot()
    macros = {macro.attrib["name"]: macro for macro in root.findall(MACRO_TAG)}
    concrete_system_macros = {
        name
        for name, macro in macros.items()
        if name.startswith("rt_control_canopen_")
        and name.endswith("_system")
        and name != "rt_control_canopen_system"
    }

    assert concrete_system_macros == set()
    assert "variant" in macros["rt_control_canopen_system"].attrib["params"].split()
    assert "canopen_variant_dir" in XACRO_PATH.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("mock", "plugin"),
    (
        (False, "canopen_ros2_control/Cia402System"),
        (True, "mock_components/GenericSystem"),
    ),
    ids=("real", "mock"),
)
def test_alfa_v1_descriptor_drives_real_and_mock_joint_contract(
    mock: bool, plugin: str, tmp_path: Path
) -> None:
    system = _system(_expand(variant="alfa_v1", mock=mock, tmp_path=tmp_path))

    assert system.attrib == {"name": "canopen_mobile_axes", "type": "system"}
    assert system.findtext("hardware/plugin") == plugin
    joints = system.findall("joint")
    assert tuple(joint.attrib["name"] for joint in joints) == CANOPEN_JOINTS
    assert {
        joint.attrib["name"]: (
            joint.findtext("param[@name='node_id']"),
            joint.findtext("param[@name='operation_mode']"),
        )
        for joint in joints
    } == {
        "left_track_joint": ("2", "3"),
        "right_track_joint": ("3", "3"),
    }
    if mock:
        assert {
            joint.attrib["name"]: (
                _interfaces(joint, "command"),
                _interfaces(joint, "state"),
            )
            for joint in joints
        } == {
            joint_name: (("velocity",), ("position", "velocity"))
            for joint_name in CANOPEN_JOINTS
        }
    else:
        assert all(
            _interfaces(joint, "command") == ()
            and _interfaces(joint, "state") == ()
            for joint in joints
        )


@pytest.mark.parametrize("mock", (False, True), ids=("real", "mock"))
def test_descriptor_node_order_is_consumed_by_the_joint_loop(
    mock: bool, tmp_path: Path
) -> None:
    descriptor = _load_yaml(VARIANT_PATH)
    descriptor["nodes"].reverse()
    variant_dir = tmp_path / "variants"
    _write_yaml(variant_dir / "alfa_v1.yaml", descriptor)

    system = _system(
        _expand(
            variant="alfa_v1",
            mock=mock,
            tmp_path=tmp_path,
            variant_dir=variant_dir,
        )
    )

    assert tuple(joint.attrib["name"] for joint in system.findall("joint")) == tuple(
        reversed(CANOPEN_JOINTS)
    )


@pytest.mark.parametrize("mock", (False, True), ids=("real", "mock"))
def test_unknown_canopen_variant_fails_closed_during_expansion(
    mock: bool, tmp_path: Path
) -> None:
    result = _expand(
        variant="not_a_production_variant", mock=mock, tmp_path=tmp_path
    )

    assert result.returncode != 0
    assert "not_a_production_variant" in result.stderr


def _unknown_root_field(descriptor: dict) -> None:
    descriptor["unexpected"] = True


def _unknown_node_field(descriptor: dict) -> None:
    descriptor["nodes"][0]["unexpected"] = True


def _missing_node_field(descriptor: dict) -> None:
    del descriptor["nodes"][0]["profile"]


def _duplicate_node_id(descriptor: dict) -> None:
    descriptor["nodes"][1]["node_id"] = descriptor["nodes"][0]["node_id"]


def _duplicate_joint_name(descriptor: dict) -> None:
    descriptor["nodes"][1]["joint_name"] = descriptor["nodes"][0]["joint_name"]


def _unsupported_mode(descriptor: dict) -> None:
    descriptor["nodes"][0]["operation_mode"] = 9


def _unsupported_profile(descriptor: dict) -> None:
    descriptor["nodes"][0]["profile"] = "unregistered_drive"


def _invalid_side(descriptor: dict) -> None:
    descriptor["nodes"][0]["side"] = "port"


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (_unknown_root_field, "root fields"),
        (_unknown_node_field, "node fields"),
        (_missing_node_field, "node fields"),
        (_duplicate_node_id, "duplicate CANopen node_id"),
        (_duplicate_joint_name, "duplicate CANopen joint_name"),
        (_unsupported_mode, "operation_mode 3"),
        (_unsupported_profile, "profile ld2_drive"),
        (_invalid_side, "side left/right"),
    ),
    ids=(
        "unknown-root-field",
        "unknown-node-field",
        "missing-node-field",
        "duplicate-node-id",
        "duplicate-joint-name",
        "unsupported-mode",
        "unsupported-profile",
        "invalid-side",
    ),
)
def test_invalid_descriptor_fails_closed_during_xacro_expansion(
    mutate, message: str, tmp_path: Path
) -> None:
    descriptor = deepcopy(_load_yaml(VARIANT_PATH))
    mutate(descriptor)
    variant_dir = tmp_path / "variants"
    variant_path = variant_dir / "alfa_v1.yaml"
    _write_yaml(variant_path, descriptor)

    with pytest.raises(VALIDATOR.ValidationError):
        VALIDATOR.validate(variant_path, BUS_PATH)

    result = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_dir,
    )

    assert result.returncode != 0
    assert message in result.stderr


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("schema_version", 2, "schema_version 1"),
        ("variant", "other", "descriptor variant"),
        ("system", "other_system", "system canopen_mobile_axes"),
    ),
)
def test_descriptor_identity_mismatch_fails_closed(
    field: str, value, message: str, tmp_path: Path
) -> None:
    descriptor = deepcopy(_load_yaml(VARIANT_PATH))
    descriptor[field] = value
    variant_dir = tmp_path / "variants"
    variant_path = variant_dir / "alfa_v1.yaml"
    _write_yaml(variant_path, descriptor)

    with pytest.raises(VALIDATOR.ValidationError):
        VALIDATOR.validate(variant_path, BUS_PATH)

    result = _expand(
        variant="alfa_v1",
        mock=True,
        tmp_path=tmp_path,
        variant_dir=variant_dir,
    )

    assert result.returncode != 0
    assert message in result.stderr


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (
            lambda bus: bus["nodes"]["left_track_joint"].update(node_id=9),
            "bus.yml node_id/operation_mode",
        ),
        (
            lambda bus: bus["nodes"].pop("right_track_joint"),
            "bus.yml node names",
        ),
        (
            lambda bus: bus["defaults"].update(dcf="eds/other_drive.eds"),
            "bus.yml defaults.dcf",
        ),
        (
            lambda bus: bus["defaults"].update(driver="other/Driver"),
            "bus.yml defaults.driver",
        ),
        (
            lambda bus: bus["defaults"].update(package="other_package"),
            "bus.yml defaults.package",
        ),
    ),
    ids=("node-id", "node-set", "profile", "driver", "package"),
)
def test_bus_alignment_mismatch_fails_closed_during_xacro_expansion(
    mutate, message: str, tmp_path: Path
) -> None:
    bus = deepcopy(_load_yaml(BUS_PATH))
    mutate(bus)
    config_dir = tmp_path / "config"
    bus_path = config_dir / "bus.yml"
    _write_yaml(bus_path, bus)

    with pytest.raises(VALIDATOR.ValidationError):
        VALIDATOR.validate(VARIANT_PATH, bus_path)

    result = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        config_dir=config_dir,
    )

    assert result.returncode != 0
    assert message in result.stderr


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (
            lambda bus: bus["nodes"]["left_track_joint"].update(mandatory=False),
            "mandatory must be true",
        ),
        (
            lambda bus: bus["nodes"]["left_track_joint"].pop("mandatory"),
            "mandatory must be true",
        ),
        (
            lambda bus: bus["nodes"]["left_track_joint"].update(
                dcf=bus["defaults"]["dcf"]
            ),
            "must not override dcf/driver/package",
        ),
        (
            lambda bus: bus["nodes"]["left_track_joint"].update(
                driver=bus["defaults"]["driver"]
            ),
            "must not override dcf/driver/package",
        ),
        (
            lambda bus: bus["nodes"]["left_track_joint"].update(
                package=bus["defaults"]["package"]
            ),
            "must not override dcf/driver/package",
        ),
    ),
    ids=(
        "mandatory-false",
        "mandatory-missing",
        "dcf-override",
        "driver-override",
        "package-override",
    ),
)
def test_bus_node_cannot_weaken_or_override_profile_contract(
    mutate, message: str, tmp_path: Path
) -> None:
    bus = deepcopy(_load_yaml(BUS_PATH))
    mutate(bus)
    config_dir = tmp_path / "config"
    bus_path = config_dir / "bus.yml"
    _write_yaml(bus_path, bus)

    with pytest.raises(VALIDATOR.ValidationError, match=message):
        VALIDATOR.validate(VARIANT_PATH, bus_path)

    result = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        config_dir=config_dir,
    )
    assert result.returncode != 0
    assert message in result.stderr


@pytest.mark.parametrize(
    "invalid_joint_name",
    (
        "../escape",
        "left/track",
        r"left\track",
        ".hidden",
        " left_track",
        "left_track ",
        "1left_track",
        "left-track",
        "履带",
    ),
    ids=(
        "parent-escape",
        "slash",
        "backslash",
        "dot-prefix",
        "leading-space",
        "trailing-space",
        "digit-prefix",
        "hyphen",
        "non-ascii",
    ),
)
def test_synchronized_descriptor_and_bus_reject_unsafe_joint_name(
    invalid_joint_name: str, tmp_path: Path
) -> None:
    descriptor = deepcopy(_load_yaml(VARIANT_PATH))
    bus = deepcopy(_load_yaml(BUS_PATH))
    original_joint_name = descriptor["nodes"][0]["joint_name"]
    descriptor["nodes"][0]["joint_name"] = invalid_joint_name
    bus["nodes"][invalid_joint_name] = bus["nodes"].pop(original_joint_name)

    variant_dir = tmp_path / "variants"
    config_dir = tmp_path / "config"
    variant_path = variant_dir / "alfa_v1.yaml"
    bus_path = config_dir / "bus.yml"
    _write_yaml(variant_path, descriptor)
    _write_yaml(bus_path, bus)

    with pytest.raises(VALIDATOR.ValidationError, match="ROS-safe identifier"):
        VALIDATOR.validate(variant_path, bus_path)

    result = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_dir,
        config_dir=config_dir,
    )
    assert result.returncode != 0
    assert "ROS-safe identifier" in result.stderr


def test_synchronized_descriptor_and_bus_reject_static_artifact_name(
    tmp_path: Path,
) -> None:
    descriptor = deepcopy(_load_yaml(VARIANT_PATH))
    bus = deepcopy(_load_yaml(BUS_PATH))
    original_joint_name = descriptor["nodes"][0]["joint_name"]
    descriptor["nodes"][0]["joint_name"] = "master"
    bus["nodes"]["master"] = bus["nodes"].pop(original_joint_name)

    variant_dir = tmp_path / "variants"
    config_dir = tmp_path / "config"
    variant_path = variant_dir / "alfa_v1.yaml"
    bus_path = config_dir / "bus.yml"
    _write_yaml(variant_path, descriptor)
    _write_yaml(bus_path, bus)

    with pytest.raises(VALIDATOR.ValidationError, match="reserved build artifact"):
        VALIDATOR.validate(variant_path, bus_path)

    result = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_dir,
        config_dir=config_dir,
    )
    assert result.returncode != 0
    assert "reserved build artifact" in result.stderr


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("node_id", 2.0),
        ("node_id", True),
        ("operation_mode", 3.0),
        ("operation_mode", True),
    ),
    ids=("float-node-id", "bool-node-id", "float-mode", "bool-mode"),
)
def test_bus_node_identity_requires_exact_integer_types(
    field: str, value, tmp_path: Path
) -> None:
    bus = deepcopy(_load_yaml(BUS_PATH))
    bus["nodes"]["left_track_joint"][field] = value
    config_dir = tmp_path / "config"
    bus_path = config_dir / "bus.yml"
    _write_yaml(bus_path, bus)

    with pytest.raises(VALIDATOR.ValidationError, match=f"{field} must be an integer"):
        VALIDATOR.validate(VARIANT_PATH, bus_path)

    result = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        config_dir=config_dir,
    )
    assert result.returncode != 0
    assert f"{field} must be an integer" in result.stderr


def test_build_validator_accepts_authoritative_descriptor_and_bus() -> None:
    descriptor = VALIDATOR.validate(VARIANT_PATH, BUS_PATH)
    result = _run_validator(variant_path=VARIANT_PATH, bus_path=BUS_PATH)

    assert descriptor["variant"] == "alfa_v1"
    assert result.returncode == 0, result.stderr
    assert "validated CANopen variant alfa_v1" in result.stdout


def test_build_validator_rejects_duplicate_yaml_keys(tmp_path: Path) -> None:
    duplicate_bus = tmp_path / "bus.yml"
    duplicate_bus.write_text(
        BUS_PATH.read_text(encoding="utf-8")
        + "\nnodes:\n  duplicate: {node_id: 4, operation_mode: 3}\n",
        encoding="utf-8",
    )

    with pytest.raises(VALIDATOR.ValidationError, match="duplicate YAML key"):
        VALIDATOR.validate(VARIANT_PATH, duplicate_bus)

    result = _run_validator(variant_path=VARIANT_PATH, bus_path=duplicate_bus)

    assert result.returncode != 0
    assert "duplicate YAML key" in result.stderr


@pytest.mark.parametrize("print_bins", (False, True), ids=("summary", "node-bins"))
def test_build_validator_cli_success_path(
    print_bins: bool, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    arguments = [
        str(VALIDATOR_PATH),
        "--variant",
        str(VARIANT_PATH),
        "--bus",
        str(BUS_PATH),
    ]
    if print_bins:
        arguments.append("--print-node-bin-names")
    monkeypatch.setattr(VALIDATOR.sys, "argv", arguments)

    assert VALIDATOR.main() == 0
    output = capsys.readouterr()
    if print_bins:
        assert output.out.splitlines() == [
            "left_track_joint.bin",
            "right_track_joint.bin",
        ]
    else:
        assert "validated CANopen variant alfa_v1 with 2 node(s)" in output.out
    assert output.err == ""


def test_build_validator_cli_failure_path(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        VALIDATOR.sys,
        "argv",
        [
            str(VALIDATOR_PATH),
            "--variant",
            str(tmp_path / "missing.yaml"),
            "--bus",
            str(BUS_PATH),
        ],
    )

    assert VALIDATOR.main() == 1
    output = capsys.readouterr()
    assert output.out == ""
    assert "CANopen variant validation failed" in output.err


def test_cmake_validates_every_registered_variant_and_derives_joint_bins() -> None:
    cmake = CMAKE_PATH.read_text(encoding="utf-8")
    xacro = XACRO_PATH.read_text(encoding="utf-8")

    assert "install(DIRECTORY variants DESTINATION share/${PROJECT_NAME})" in cmake
    assert "file(GLOB CANOPEN_VARIANT_PATHS CONFIGURE_DEPENDS" in cmake
    assert "foreach(CANOPEN_VARIANT_PATH IN LISTS CANOPEN_VARIANT_PATHS)" in cmake
    assert "set(CANOPEN_VARIANT_PATH" not in cmake
    assert "BYPRODUCTS" in cmake
    assert "generation.stamp" in cmake
    assert "bus.authority.yml" in cmake
    assert "copy_if_different" in cmake
    assert 'name="canopen_authority_bus"' in xacro
    assert "xacro.load_yaml('$(arg canopen_authority_bus)')" in xacro
    assert "left_track_joint.bin" not in cmake
    assert "right_track_joint.bin" not in cmake
