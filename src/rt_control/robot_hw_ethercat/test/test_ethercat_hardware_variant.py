"""Contracts for the hardware-owned EtherCAT variant descriptor."""

from copy import deepcopy
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_DIR / "urdf/ecat.ros2_control.xacro"
VARIANT_PATH = PACKAGE_DIR / "variants/alfa_v1.yaml"
PROFILE_DIR = PACKAGE_DIR / "config/slaves"
FAMILY_REGISTRY_PATH = PACKAGE_DIR / "config/families.yaml"
VALIDATOR_PATH = PACKAGE_DIR / "scripts/validate_ethercat_variants.py"
XACRO_NAMESPACE = "http://www.ros.org/wiki/xacro"
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
ECAT_RING_POSITIONS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 17)
X503_SENSORS = (
    ("right_force_sensor", 14, "x503_right"),
    ("left_force_sensor", 15, "x503_left"),
)
X503_STATE_INTERFACES = (
    "channel_1_raw",
    "channel_2_raw",
    "channel_3_raw",
    "channel_4_raw",
    "channel_5_raw",
    "channel_6_raw",
    "sample_code_1_raw",
    "sample_code_2_raw",
    "sample_code_3_raw",
    "sample_code_4_raw",
    "sample_code_5_raw",
    "sample_code_6_raw",
)


def _write_variant(directory: Path, descriptor: dict) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "alfa_v1.yaml"
    path.write_text(yaml.safe_dump(descriptor, sort_keys=False), encoding="utf-8")
    return path


def _copy_profiles(directory: Path) -> Path:
    target = directory / "slaves"
    shutil.copytree(PROFILE_DIR, target)
    shutil.copy2(FAMILY_REGISTRY_PATH, directory / FAMILY_REGISTRY_PATH.name)
    return target


def _load_descriptor() -> dict:
    loaded = yaml.safe_load(VARIANT_PATH.read_text(encoding="utf-8"))
    assert isinstance(loaded, dict)
    return loaded


def _load_family_registry() -> dict:
    loaded = yaml.safe_load(FAMILY_REGISTRY_PATH.read_text(encoding="utf-8"))
    assert isinstance(loaded, dict)
    return loaded


def _validate(
    *, variant_file: Path = VARIANT_PATH, profile_dir: Path = PROFILE_DIR
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "python3",
            str(VALIDATOR_PATH),
            "--variant-file",
            str(variant_file),
            "--profile-dir",
            str(profile_dir),
        ],
        check=False,
        capture_output=True,
        text=True,
    )


def _expand(
    *,
    variant: str,
    mock: bool,
    tmp_path: Path,
    variant_dir: Path | None = None,
    profile_dir: Path | None = None,
) -> subprocess.CompletedProcess[str]:
    xacro = shutil.which("xacro")
    assert xacro is not None, "ROS 2 xacro must be installed to verify this contract"

    wrapper = tmp_path / "ethercat_variant_test.urdf.xacro"
    wrapper.write_text(
        f"""<?xml version="1.0"?>
<robot name="ethercat_variant_test" xmlns:xacro="{XACRO_NAMESPACE}">
  <xacro:include filename="{XACRO_PATH}"/>
  <xacro:rt_control_ethercat_system
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
            f"config_dir:={(profile_dir or PROFILE_DIR).parent}",
            f"variant_dir:={variant_dir or VARIANT_PATH.parent}",
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


def _assert_validation_rejected(
    *, variant_file: Path, profile_dir: Path, message: str
) -> None:
    result = _validate(variant_file=variant_file, profile_dir=profile_dir)
    assert result.returncode != 0
    assert message in result.stderr


def test_descriptor_has_the_frozen_public_schema_and_topology() -> None:
    descriptor = _load_descriptor()

    assert tuple(descriptor) == (
        "schema_version",
        "variant",
        "system",
        "extra_responders",
        "sensors",
        "axes",
    )
    assert descriptor["schema_version"] == 1
    assert descriptor["variant"] == "alfa_v1"
    assert descriptor["system"] == "ecat_arms"
    assert descriptor["extra_responders"] == [
        {"ring_position": 0},
        {"ring_position": 13},
    ]
    assert tuple(
        (sensor["sensor_name"], sensor["ring_position"], sensor["profile"])
        for sensor in descriptor["sensors"]
    ) == X503_SENSORS
    assert all(
        set(sensor) == {
            "sensor_name",
            "family",
            "ring_position",
            "profile",
            "wrench_topic",
            "raw_topic",
            "frame_id",
        }
        and sensor["family"] == "x503"
        for sensor in descriptor["sensors"]
    )
    assert tuple(axis["joint_name"] for axis in descriptor["axes"]) == ECAT_JOINTS
    assert tuple(axis["ring_position"] for axis in descriptor["axes"]) == (
        ECAT_RING_POSITIONS
    )
    assert all(
        tuple(axis) == (
            "joint_name",
            "family",
            "ring_position",
            "profile",
            "mode_of_operation",
        )
        for axis in descriptor["axes"]
    )


def test_strict_validator_accepts_the_production_descriptor() -> None:
    result = _validate()

    assert result.returncode == 0, result.stderr


def test_family_registry_reuses_profile_device_identity_without_profile_tags() -> None:
    registry = _load_family_registry()
    families = registry["families"]
    descriptor_families = {
        axis["family"] for axis in _load_descriptor()["axes"]
    } | {sensor["family"] for sensor in _load_descriptor()["sensors"]}

    assert registry["schema_version"] == 1
    assert descriptor_families <= set(families)
    assert all(
        (PROFILE_DIR / f"{family['identity_profile']}.yaml").is_file()
        for family in families.values()
    )
    assert all(
        "family" not in yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        for profile_path in PROFILE_DIR.glob("*.yaml")
    )


def test_family_public_interface_contract_matches_xacro_expansion(
    tmp_path: Path,
) -> None:
    descriptor = _load_descriptor()
    registry = _load_family_registry()
    system = _system(_expand(variant="alfa_v1", mock=True, tmp_path=tmp_path))

    for axis in descriptor["axes"]:
        joint = system.find(f"joint[@name='{axis['joint_name']}']")
        assert joint is not None
        family = registry["families"][axis["family"]]
        contract = registry["interface_contracts"][family["interface_contract"]]
        assert tuple(
            interface.attrib["name"]
            for interface in joint.findall("command_interface")
        ) == tuple(
            interface["name"]
            for interface in contract["required_command_interfaces"]
        )
        assert tuple(
            interface.attrib["name"]
            for interface in joint.findall("state_interface")
        ) == tuple(
            interface["name"]
            for interface in contract["required_state_interfaces"]
        )

    for sensor_registration in descriptor["sensors"]:
        sensor = system.find(
            f"sensor[@name='{sensor_registration['sensor_name']}']"
        )
        assert sensor is not None
        family = registry["families"][sensor_registration["family"]]
        contract = registry["interface_contracts"][family["interface_contract"]]
        assert sensor.findall("command_interface") == []
        assert tuple(
            interface.attrib["name"]
            for interface in sensor.findall("state_interface")
        ) == tuple(
            interface["name"]
            for interface in contract["required_state_interfaces"]
        )


@pytest.mark.parametrize(
    "duplicate_text",
    (
        "\nsystem: duplicate_system\n",
        "\naxes:\n  - joint_name: duplicate_axes_key\n",
        "\nsensors:\n  - sensor_name: duplicate_sensors_key\n",
    ),
    ids=("scalar", "axis-sequence", "sensor-sequence"),
)
def test_duplicate_yaml_mapping_key_is_rejected(
    duplicate_text: str, tmp_path: Path
) -> None:
    variant_file = tmp_path / "alfa_v1.yaml"
    variant_file.write_text(
        VARIANT_PATH.read_text(encoding="utf-8") + duplicate_text,
        encoding="utf-8",
    )

    _assert_validation_rejected(
        variant_file=variant_file,
        profile_dir=PROFILE_DIR,
        message="duplicate mapping key",
    )


@pytest.mark.parametrize(
    ("field", "message"),
    (
        ("joint_name", "duplicate joint_name"),
        ("ring_position", "duplicate ring_position"),
    ),
)
def test_duplicate_axis_identity_is_rejected_by_validator_and_xacro(
    field: str, message: str, tmp_path: Path
) -> None:
    descriptor = _load_descriptor()
    descriptor["axes"][1][field] = descriptor["axes"][0][field]
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    _assert_validation_rejected(
        variant_file=variant_file, profile_dir=PROFILE_DIR, message=message
    )
    expansion = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_file.parent,
    )
    assert expansion.returncode != 0
    assert message in expansion.stderr


def test_axis_and_extra_responder_cannot_share_a_ring_position(tmp_path: Path) -> None:
    descriptor = _load_descriptor()
    descriptor["extra_responders"][1]["ring_position"] = descriptor["axes"][0][
        "ring_position"
    ]
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    _assert_validation_rejected(
        variant_file=variant_file,
        profile_dir=PROFILE_DIR,
        message="duplicate ring_position",
    )
    expansion = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_file.parent,
    )
    assert expansion.returncode != 0
    assert "duplicate ring_position" in expansion.stderr


@pytest.mark.parametrize(
    ("field", "message"),
    (
        ("sensor_name", "duplicate sensor_name"),
        ("ring_position", "duplicate ring_position"),
    ),
)
def test_duplicate_sensor_identity_is_rejected_by_validator_and_xacro(
    field: str, message: str, tmp_path: Path
) -> None:
    descriptor = _load_descriptor()
    descriptor["sensors"][1][field] = descriptor["sensors"][0][field]
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    _assert_validation_rejected(
        variant_file=variant_file, profile_dir=PROFILE_DIR, message=message
    )
    expansion = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_file.parent,
    )
    assert expansion.returncode != 0
    assert message in expansion.stderr


def test_registered_ring_positions_must_form_one_contiguous_bus(tmp_path: Path) -> None:
    descriptor = _load_descriptor()
    descriptor["axes"][-1]["ring_position"] = 18
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    _assert_validation_rejected(
        variant_file=variant_file,
        profile_dir=PROFILE_DIR,
        message="ring_positions must form contiguous range 0..17",
    )


@pytest.mark.parametrize(
    ("field", "value", "message", "xacro_message"),
    (
        ("family", "unknown_family", "unsupported family", "unsupported family"),
        ("profile", "unknown_profile", "profile does not exist", "unknown_profile"),
        (
            "mode_of_operation",
            9,
            "uncertified mode_of_operation",
            "uncertified mode_of_operation",
        ),
    ),
)
def test_unknown_axis_registration_is_rejected_by_validator_and_xacro(
    field: str,
    value: object,
    message: str,
    xacro_message: str,
    tmp_path: Path,
) -> None:
    descriptor = _load_descriptor()
    descriptor["axes"][0][field] = value
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    _assert_validation_rejected(
        variant_file=variant_file, profile_dir=PROFILE_DIR, message=message
    )
    expansion = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_file.parent,
    )
    assert expansion.returncode != 0
    assert xacro_message in expansion.stderr


@pytest.mark.parametrize(
    ("joint_name", "wrong_family"),
    (
        ("right_joint1", "ti5"),
        ("right_joint2", "zeroerr"),
    ),
    ids=("zeroerr-profile-as-ti5", "ti5-profile-as-zeroerr"),
)
def test_axis_family_must_match_the_profile_device_identity(
    joint_name: str, wrong_family: str, tmp_path: Path
) -> None:
    descriptor = _load_descriptor()
    axis = next(
        candidate
        for candidate in descriptor["axes"]
        if candidate["joint_name"] == joint_name
    )
    axis["family"] = wrong_family
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    _assert_validation_rejected(
        variant_file=variant_file,
        profile_dir=PROFILE_DIR,
        message="profile device identity",
    )
    expansion = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        variant_dir=variant_file.parent,
    )
    assert expansion.returncode != 0
    assert "profile device identity" in expansion.stderr


@pytest.mark.parametrize(
    (
        "profile_name",
        "pdo_name",
        "interface_key",
        "interface_name",
        "binding_field",
        "wrong_value",
    ),
    (
        (
            "zeroerr_j1",
            "rpdo",
            "command_interface",
            "digital_outputs",
            "index",
            0x60FF,
        ),
        (
            "ti5_j2",
            "tpdo",
            "state_interface",
            "status_word",
            "sub_index",
            1,
        ),
        (
            "xmc_updown_sw511",
            "rpdo",
            "command_interface",
            "position",
            "type",
            "uint32",
        ),
        (
            "x503_right",
            "tpdo",
            "state_interface",
            "channel_1_raw",
            "sub_index",
            2,
        ),
    ),
    ids=("zeroerr-index", "ti5-sub-index", "xmc-type", "x503-sub-index"),
)
def test_family_profile_must_expose_each_required_pdo_interface_binding(
    profile_name: str,
    pdo_name: str,
    interface_key: str,
    interface_name: str,
    binding_field: str,
    wrong_value: object,
    tmp_path: Path,
) -> None:
    profile_dir = _copy_profiles(tmp_path / "config")
    profile_path = profile_dir / f"{profile_name}.yaml"
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    channel = next(
        candidate
        for pdo in profile[pdo_name]
        for candidate in pdo["channels"]
        if candidate.get(interface_key) == interface_name
    )
    channel[binding_field] = wrong_value
    profile_path.write_text(
        yaml.safe_dump(profile, sort_keys=False), encoding="utf-8"
    )

    _assert_validation_rejected(
        variant_file=VARIANT_PATH,
        profile_dir=profile_dir,
        message=f"required {interface_key} {interface_name!r}",
    )


@pytest.mark.parametrize(
    ("profile_name", "section", "value_key", "message"),
    (
        ("zeroerr_j1", "sdo", "value", "SDO 0x6060 value"),
        ("xmc_updown_sw511", "rpdo", "default", "RPDO 0x6060 default"),
    ),
    ids=("sdo", "rpdo-default"),
)
def test_every_profile_mode_copy_must_match_the_descriptor(
    profile_name: str,
    section: str,
    value_key: str,
    message: str,
    tmp_path: Path,
) -> None:
    profile_dir = _copy_profiles(tmp_path / "config")
    profile_path = profile_dir / f"{profile_name}.yaml"
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if section == "sdo":
        entry = next(item for item in profile[section] if item["index"] == 0x6060)
    else:
        entry = next(
            channel
            for pdo in profile[section]
            for channel in pdo["channels"]
            if channel["index"] == 0x6060
        )
    entry[value_key] = 9
    profile_path.write_text(
        yaml.safe_dump(profile, sort_keys=False), encoding="utf-8"
    )

    _assert_validation_rejected(
        variant_file=VARIANT_PATH, profile_dir=profile_dir, message=message
    )
    expansion = _expand(
        variant="alfa_v1",
        mock=False,
        tmp_path=tmp_path,
        profile_dir=profile_dir,
    )
    assert expansion.returncode != 0
    assert message in expansion.stderr


@pytest.mark.parametrize(
    ("mock", "plugin"),
    (
        (False, "ethercat_driver/EthercatDriver"),
        (True, "mock_components/GenericSystem"),
    ),
    ids=("real", "mock"),
)
def test_alfa_v1_preserves_stable_system_axis_and_sensor_contract(
    mock: bool, plugin: str, tmp_path: Path
) -> None:
    descriptor = _load_descriptor()
    system = _system(_expand(variant="alfa_v1", mock=mock, tmp_path=tmp_path))

    assert system.attrib == {"name": descriptor["system"], "type": "system"}
    assert system.findtext("hardware/plugin") == plugin
    joints = system.findall("joint")
    assert tuple(joint.attrib["name"] for joint in joints) == ECAT_JOINTS
    assert {
        sensor.attrib["name"]
        for sensor in system.findall("sensor")
        if sensor.attrib["name"].startswith("ethercat_slave_")
    } == {
        f"ethercat_slave_{position}"
        for position in (*ECAT_RING_POSITIONS[:-2], 14, 15, 16, 17)
    }
    for sensor_name, ring_position, profile in X503_SENSORS:
        sensor = system.find(f"sensor[@name='{sensor_name}']")
        assert sensor is not None
        assert tuple(
            interface.attrib["name"]
            for interface in sensor.findall("state_interface")
        ) == X503_STATE_INTERFACES
        assert sensor.findall("command_interface") == []
        if mock:
            assert sensor.find("param[@name='ec_module.position']") is None
        else:
            assert sensor.findtext("param[@name='ec_module.position']") == str(
                ring_position
            )
            assert Path(
                sensor.findtext("param[@name='ec_module.slave_config']") or ""
            ).stem == profile
    master = system.find("sensor[@name='ethercat_master']")
    assert master is not None
    assert master.findtext(
        "state_interface[@name='slaves_responding']/param[@name='initial_value']"
    ) == ("18" if mock else None)
    if mock:
        assert all(
            joint.find("param[@name='ec_module.mode_of_operation']") is None
            for joint in joints
        )
    else:
        assert tuple(
            int(joint.findtext("param[@name='ec_module.position']"))
            for joint in joints
        ) == ECAT_RING_POSITIONS
        assert all(
            joint.findtext("param[@name='ec_module.mode_of_operation']") == "8"
            for joint in joints
        )


def test_descriptor_is_the_only_axis_registration_table() -> None:
    xacro_source = XACRO_PATH.read_text(encoding="utf-8")

    assert "xacro.load_yaml" in xacro_source
    assert "variants/" not in xacro_source
    assert all(joint_name not in xacro_source for joint_name in ECAT_JOINTS)
    assert all(
        sensor_name not in xacro_source
        for sensor_name, _ring_position, _profile in X503_SENSORS
    )


@pytest.mark.parametrize("mock", (False, True), ids=("real", "mock"))
def test_unknown_ethercat_variant_fails_closed_during_expansion(
    mock: bool, tmp_path: Path
) -> None:
    result = _expand(
        variant="not_a_production_variant", mock=mock, tmp_path=tmp_path
    )

    assert result.returncode != 0
    assert "not_a_production_variant" in result.stderr


@pytest.mark.parametrize(
    "variant",
    ("../alfa_v1", "/tmp/alfa_v1", "Alfa_v1", "alfa-v1"),
)
def test_public_xacro_rejects_variant_path_syntax(
    variant: str, tmp_path: Path
) -> None:
    result = _expand(variant=variant, mock=True, tmp_path=tmp_path)

    assert result.returncode != 0
    assert "invalid EtherCAT variant identifier" in result.stderr


def test_mutating_descriptor_changes_expansion_without_editing_xacro(
    tmp_path: Path,
) -> None:
    descriptor = deepcopy(_load_descriptor())
    descriptor["axes"][0]["joint_name"] = "registered_test_joint"
    variant_file = _write_variant(tmp_path / "variants", descriptor)

    system = _system(
        _expand(
            variant="alfa_v1",
            mock=True,
            tmp_path=tmp_path,
            variant_dir=variant_file.parent,
        )
    )

    assert system.find("joint[@name='registered_test_joint']") is not None
