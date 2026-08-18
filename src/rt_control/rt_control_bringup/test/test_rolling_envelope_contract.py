from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


AXIS_ORDER = [
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
]

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
ENVELOPE_PATH = PACKAGE_ROOT / "config" / "rolling_envelope_provisional.yaml"
JOINT_LIMITS_PATH = PACKAGE_ROOT / "config" / "joint_limits.yaml"
URDF_PATH = (
    REPOSITORY_ROOT
    / "src"
    / "description"
    / "robot_description"
    / "urdf"
    / "robot.urdf.xacro"
)


@pytest.fixture(scope="module")
def envelope():
    return yaml.safe_load(ENVELOPE_PATH.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def configured_joint_limits():
    return yaml.safe_load(JOINT_LIMITS_PATH.read_text(encoding="utf-8"))["joints"]


@pytest.fixture(scope="module")
def urdf_joint_limits():
    root = ET.parse(URDF_PATH).getroot()
    limits = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        limit = joint.find("limit")
        if name in AXIS_ORDER and limit is not None:
            limits[name] = {
                "lower": float(limit.attrib["lower"]),
                "upper": float(limit.attrib["upper"]),
            }
    return limits


def test_metadata_declares_estimated_provisional_authority(envelope):
    metadata = envelope["metadata"]
    assert metadata["schema_version"] == 1
    assert metadata["limits_source"] == "provisional"
    assert metadata["status"] == "ESTIMATED_NOT_MEASURED"
    assert "DOES NOT REPLACE BENCH MEASUREMENT" in metadata["warning"]
    assert "bq-138" in metadata["commissioning_schedule"].lower()


def test_position_bounds_are_exact_conservative_intersections(
    envelope, configured_joint_limits, urdf_joint_limits
):
    assert set(urdf_joint_limits) == set(AXIS_ORDER)
    assert [axis["name"] for axis in envelope["axes"]] == AXIS_ORDER

    for axis in envelope["axes"]:
        name = axis["name"]
        configured = configured_joint_limits[name]
        if name == "updown":
            configured_lower = configured["lower_position_m"]
            configured_upper = configured["upper_position_m"]
        else:
            configured_lower = configured["lower_position_rad"]
            configured_upper = configured["upper_position_rad"]
        expected_lower = max(urdf_joint_limits[name]["lower"], configured_lower)
        expected_upper = min(urdf_joint_limits[name]["upper"], configured_upper)
        assert axis["position_lower"] == pytest.approx(expected_lower, abs=1.0e-12)
        assert axis["position_upper"] == pytest.approx(expected_upper, abs=1.0e-12)
        assert axis["position_lower"] + axis["position_margin_lower"] < (
            axis["position_upper"] - axis["position_margin_upper"]
        )


@pytest.mark.parametrize("axis_index", range(14), ids=AXIS_ORDER)
def test_provisional_kinematic_values_follow_frozen_rules(envelope, axis_index):
    axis = envelope["axes"][axis_index]
    if axis["name"] == "updown":
        expected_velocity = 0.09
        expected_acceleration = 0.5
        expected_margin = 0.005
    else:
        expected_velocity = 0.2617993877991494
        expected_acceleration = 0.75
        expected_margin = 0.008726646259971648

    assert axis["velocity_positive"] == expected_velocity
    assert axis["velocity_negative"] == expected_velocity
    assert axis["acceleration_positive"] == expected_acceleration
    assert axis["acceleration_negative"] == expected_acceleration
    assert axis["stop_acceleration_positive"] == expected_acceleration
    assert axis["stop_acceleration_negative"] == expected_acceleration
    assert axis["position_margin_lower"] == expected_margin
    assert axis["position_margin_upper"] == expected_margin
