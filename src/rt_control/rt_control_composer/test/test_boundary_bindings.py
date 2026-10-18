"""Configuration changes cross public bindings without private-spec coupling or hardware IO."""

from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest
import yaml

from rt_control_composer.bindings import bind_parameters
from rt_control_composer.compose import compose
from robot_hw_can import rt_control_renderer as CAN
from robot_hw_canopen import rt_control_renderer as CANOPEN
from robot_hw_ethercat import rt_control_renderer as ETHERCAT

RT = Path(__file__).resolve().parents[2]


def load(path):
    return yaml.safe_load(path.read_text())


def save(path, value):
    path.write_text(yaml.safe_dump(value, sort_keys=False))


@pytest.fixture
def config(tmp_path, monkeypatch):
    root = tmp_path / "facts"
    shutil.copytree(RT / "rt_control_machine_config", root)
    shares = {path.name: path for path in RT.iterdir() if path.is_dir()}
    shares["rt_control_machine_config"] = root
    shares["robot_description"] = RT.parent / "description/robot_description"
    for owner, name in (
        (CAN, "robot_hw_can"),
        (CANOPEN, "robot_hw_canopen"),
        (ETHERCAT, "robot_hw_ethercat"),
    ):
        monkeypatch.setattr(owner, "_share", lambda name=name: RT / name)
    return root, dict(
        robot_variant="Alfa_V3",
        machine_unit="Alfa_V3-01",
        package_share=lambda name: str(shares[name]),
    )


def test_chassis_binds_facts_by_instance_after_bus_and_calibration_reordering(config, tmp_path):
    root, args = config
    path = root / "units/Alfa_V3-Advantech-01/swerve_calibration.yaml"
    unit = load(path)
    for index, item in enumerate(unit["spec"]["modules"]):
        item.update(
            effective_wheel_radius_m=0.10 + index * 0.01,
            module_x_m=index * 0.2,
            module_y_m=-index * 0.3,
        )
    expected = deepcopy(unit["spec"]["modules"])
    unit["spec"]["modules"].reverse()
    save(path, unit)
    path = root / "machines/Alfa_V3/swerve_chassis.yaml"
    machine = load(path)
    renamed = machine["spec"]["axes"][0]
    old_joint = renamed["mock_joint_name"]
    renamed["mock_joint_name"] = renamed["robot_model_joint"] = "test_steering_a"
    model = root / "machines/Alfa_V3/chassis.mock.urdf"
    model.write_text(model.read_text().replace(old_joint, renamed["mock_joint_name"]))
    machine["spec"]["axes"].reverse()
    save(path, machine)
    path = root / "machines/Alfa_V3/swerve_encoders.yaml"
    encoders = load(path)
    old_name = encoders["spec"]["nodes"][0]["resource_name"]
    encoders["spec"]["nodes"][0]["resource_name"] = "angle_sensor_a"
    encoders["spec"]["nodes"].reverse()
    save(path, encoders)
    path = root / "units/Alfa_V3-Advantech-01/deployment.yaml"
    deployment = load(path)
    deployment["canopen"]["eds_files"]["angle_sensor_a"] = deployment["canopen"]["eds_files"].pop(
        old_name
    )
    save(path, deployment)
    result = compose(
        **{**args, "machine_unit": "Alfa_V3-Advantech-01"},
        physical_profile="chassis_only",
        control_scope="chassis_only",
        runtime_dir=tmp_path / "out",
    )
    params = result.controllers["swerve_controller"]["ros__parameters"]
    for parameter, source in (
        ("wheel_radius", "effective_wheel_radius_m"),
        ("module_x", "module_x_m"),
        ("module_y", "module_y_m"),
    ):
        assert params[parameter] == [item[source] for item in expected]
    assert params["steering_encoders"][0] == "angle_sensor_a"
    axes = {
        (axis["module_position"], axis["role"]): axis["mock_joint_name"]
        for axis in machine["spec"]["axes"]
    }
    for role in ("steering", "drive"):
        assert params[role + "_joints"] == [
            axes[item["module_position"], role] for item in expected
        ]
    # Bound from the verified chassis data and unit calibration, not a controller default.
    assert params["calibration_verified"] is True
    unit["verified"] = False
    save(root / "units/Alfa_V3-Advantech-01/swerve_calibration.yaml", unit)
    mock = compose(
        **{**args, "machine_unit": "Alfa_V3-Advantech-01"},
        physical_profile="chassis_only",
        control_scope="chassis_only",
        runtime_dir=tmp_path / "unverified",
    )
    assert mock.controllers["swerve_controller"]["ros__parameters"]["calibration_verified"] is False


def test_head_renaming_reaches_hardware_and_all_controllers(config, tmp_path):
    root, args = config
    machine_dir = root / "machines/Alfa_V3"
    path = machine_dir / "head_gimbal.yaml"
    head = load(path)
    names = ["test_yaw", "test_pitch"]
    for joint, name in zip(head["spec"]["joints"], names):
        joint["robot_model_joint"] = name
    head["spec"]["control_name"] = "test_pair_control"
    save(path, head)
    # A synthetic model in the test directory, never production geometry or calibration.
    (machine_dir / "test.urdf").write_text(
        '<robot name="test">' + "".join(f'<joint name="{name}"/>' for name in names) + "</robot>"
    )
    save(
        machine_dir / "test.limits.yaml",
        {
            "joints": {
                name: {"lower_position_rad": -0.1, "upper_position_rad": 0.1} for name in names
            }
        },
    )
    manifest = load(machine_dir / "manifest.yaml")
    manifest["scopes"]["head_only"]["mock_robot_model"] = {
        "package": "rt_control_machine_config",
        "xacro_file": "machines/Alfa_V3/test.urdf",
        "joint_limits_file": "machines/Alfa_V3/test.limits.yaml",
    }
    save(machine_dir / "manifest.yaml", manifest)
    result = compose(
        **args,
        physical_profile="head_only",
        control_scope="head_only",
        runtime_dir=tmp_path / "out",
    )
    system = ET.fromstring(result.robot_description).find("ros2_control")
    assert [joint.attrib["name"] for joint in system.findall("joint")] == names
    assert system.find("gpio").attrib["name"] == "test_pair_control"
    for name in ("joint_state_broadcaster", "head_position_controller", "damiao_head_manager"):
        assert result.controllers[name]["ros__parameters"]["joints"] == names
    assert (
        result.controllers["damiao_head_manager"]["ros__parameters"]["control_name"]
        == "test_pair_control"
    )


def test_force_private_spec_can_change_without_composer_changes(config, tmp_path, monkeypatch):
    root, args = config
    path = root / "machines/Alfa_V3/wrist_force_sensors.yaml"
    machine = load(path)
    machine["spec"] = {"private_revision_two": machine["spec"]}
    save(path, machine)
    original = ETHERCAT.render

    def revised_owner(request):
        translated = deepcopy(request)
        translated["machine"]["spec"] = translated["machine"]["spec"]["private_revision_two"]
        return original(translated)

    monkeypatch.setattr(ETHERCAT, "render", revised_owner)
    result = compose(
        **args,
        physical_profile="force_bench",
        control_scope="force_bench",
        runtime_dir=tmp_path / "out",
        startup_id="test",
    )
    for side in ("left", "right"):
        params = result.controllers[f"{side}_wrist_force_sensor_broadcaster"]["ros__parameters"]
        assert params["sensor_name"] == f"{side}_wrist_force_sensor"
        assert params["startup_id"] == "test"
        assert side in params["wrench_topic"]


@pytest.mark.parametrize("failure", ["instance", "duplicate_fact", "resource", "missing", "tbd"])
def test_bindings_reject_incomplete_or_conflicting_sources(failure):
    binding = {
        "instance_order": ["a", "b"],
        "parameters": {
            "joints": {"module": "actuators", "export": "names", "resource_kind": "joints"}
        },
    }
    owners = {
        "actuators": {
            "parameter_exports": {"names": {"a": "j1", "b": "j2"}},
            "joints": [ET.Element("joint", name=name) for name in ("j1", "j2")],
        }
    }
    parameters = {}
    if failure == "instance":
        binding["instance_order"] = ["a", "a"]
    elif failure == "duplicate_fact":
        parameters["joints"] = ["j1", "j2"]
    elif failure == "resource":
        owners["actuators"]["parameter_exports"]["names"]["b"] = "absent"
    elif failure == "missing":
        owners["actuators"]["parameter_exports"] = {}
    else:
        owners["actuators"]["parameter_exports"]["names"]["b"] = "TBD"
    with pytest.raises(ValueError):
        bind_parameters(binding, parameters, owners, mock=False)
