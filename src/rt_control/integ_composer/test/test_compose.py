"""Frozen old outputs and the new integration boundary; no hardware is accessed."""

from copy import deepcopy
import filecmp
from hashlib import sha256
import importlib.util
from pathlib import Path
import shutil
import sys

import pytest
import yaml

from integ_composer.canonical import canonical_files
from integ_composer.compose import compose, preflight
from integ_composer.renderer_registry import project, render_checked, select_renderer
from integ_composer.schema.cross_checks import merge_systems
from integ_composer.schema.primitives import load_yaml, validate_data
from integ_composer.schema.manifest_load import read_manifest

PACKAGE = Path(__file__).resolve().parents[1]
RT = PACKAGE.parent
GOLDEN = PACKAGE / "test/golden"
_SPEC = importlib.util.spec_from_file_location(
    "config_validator", PACKAGE / "scripts/validate_config_files.py"
)
VALIDATOR = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(VALIDATOR)
BASELINE = yaml.safe_load((GOLDEN / "manifest.yaml").read_text())
sys.path.insert(0, str(RT / "hw_ethercat"))
from hw_ethercat import rt_control_renderer as OWNER


@pytest.fixture
def shares(tmp_path, monkeypatch):
    data = tmp_path / "machine_config"
    shutil.copytree(RT / "hwcfg_machines", data)
    monkeypatch.setattr(OWNER, "_share", lambda: RT / "hw_ethercat")
    lookup = {path.name: path for path in RT.iterdir() if path.is_dir()}
    lookup["hwcfg_machines"] = data
    lookup["robot_description"] = RT.parent / "description/robot_description"
    return lookup


def materialize(result):
    """Compose is pure; the tests write its files to compare with frozen output trees."""
    for name, text in result.files.items():
        path = Path(result.runtime_dir) / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return result


def inputs(shares, *, stationary=False, verified=False):
    data = shares["hwcfg_machines"]
    deployment_path = (data / "units/Alfa_V3-01/deployment.yaml")
    deployment = load_yaml(deployment_path)
    manifest = read_manifest(data / "machines" / deployment["robot_variant"] / "manifest.yaml")
    mode = "stationary_enable" if stationary else "normal"
    scope_name = next(
        name for name, scope in manifest["scopes"].items() if scope.get("render_mode") == mode
    )
    scope = manifest["scopes"][scope_name]
    profile = next(
        name
        for name, profile in manifest["profiles"].items()
        if scope_name in profile["allowed_scopes"]
    )
    if verified:
        old = load_yaml(GOLDEN / "verified_calibration.yaml")
        for name in scope["enabled_modules"]:
            module_path = data / manifest["modules"][name]["config_file"]
            module = load_yaml(module_path)
            module["verified"] = True
            by_name = {entry["joint_name"]: entry for entry in old["axes"]}
            for branch in module["spec"]["branches"].values():
                for axis in branch["axes"]:
                    if axis["robot_model_joint"] in by_name:
                        axis["direction"] = by_name[axis["robot_model_joint"]]["direction"]
            module_path.write_text(yaml.safe_dump(module, sort_keys=False))
    return dict(
        robot_variant=deployment["robot_variant"],
        machine_unit=deployment_path.parent.name,
        physical_profile=profile,
        control_scope=scope_name,
        package_share=lambda name: str(shares[name]),
    )


def sample_request(shares):
    """Explicit owner adapter for synthetic sample A; no production calibration."""
    request = load_yaml(PACKAGE / "test/conformance/render_request.yaml")
    arguments = inputs(shares)
    data = shares["hwcfg_machines"]
    manifest = read_manifest(data / "machines" / arguments["robot_variant"] / "manifest.yaml")
    name = manifest["scopes"][arguments["control_scope"]]["enabled_modules"][0]
    source = load_yaml(data / manifest["modules"][name]["config_file"])
    axes = [
        {key: value for key, value in axis.items() if key != "role"}
        for axis in request["machine"]["spec"]["axes"]
    ]
    request["machine"]["spec"] = {
        "family": source["spec"]["family"],
        "extra_responders": [{**source["spec"]["extra_responders"][0], "ring_position": 0}],
        "branches": {"synthetic": {"connector": "synthetic", "main_esc_port": 3, "axes": axes}},
        "drive_profiles": source["spec"]["drive_profiles"],
        "branch_mapping_verified": True,
        "direction_status": "synthetic-only",
    }
    request["machine"]["verified"] = True
    family = load_yaml(data / manifest["modules"][name]["device_refs"][0])["family"]
    for device in request["devices"].values():
        device["family"] = family
    zeros = {
        item["robot_model_joint"]: item["zero_counts"] for item in request["unit"]["spec"]["axes"]
    }
    request["unit"]["spec"] = {
        "confirmed_by": "user",
        "captured_utc": "synthetic-only",
        "position_object": 0x6064,
        "external_transmission_ratio": 1.0,
        "axes": [
            {
                "ring_position": axis["ring_position"],
                "zero_counts": zeros[axis["robot_model_joint"]],
                "digital_outputs": 0,
            }
            for axis in axes
        ],
    }
    request["unit"]["verified"] = True
    request["deployment"] = load_yaml((data / "units/Alfa_V3-01/deployment.yaml"))
    return request


@pytest.mark.parametrize(
    "case", [name for name in BASELINE["cases"] if name.startswith(("arm-", "stationary-"))]
)
def test_frozen_files_are_exactly_equivalent(shares, tmp_path, case):
    facts = BASELINE["cases"][case]
    baseline = GOLDEN / case
    actual_files = {
        path.relative_to(baseline).as_posix(): sha256(path.read_bytes()).hexdigest()
        for path in baseline.rglob("*")
        if path.is_file()
    }
    assert actual_files == facts["outputs"], "Frozen baseline was modified"
    arguments = inputs(
        shares, stationary=case.startswith("stationary"), verified=not facts["use_mock_hardware"]
    )
    runtime = tmp_path / "runtime"
    result = materialize(compose(**arguments, runtime_dir=runtime, **facts_subset(facts)))
    assert load_yaml(runtime / "controllers.yaml") == result.controllers
    # File layout is explicit: three launch outputs and every remaining runtime file.
    snapshot = tmp_path / "snapshot"
    snapshot.mkdir()
    for path in runtime.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(runtime)
        target = (
            snapshot / relative
            if relative.as_posix()
            in {"robot_description.xml", "controllers.yaml", "controller_order.yaml"}
            else snapshot / "runtime" / relative
        )
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    expected = canonical_files(baseline, "<RUNTIME_DIR>", BASELINE["rename_map"])
    actual = canonical_files(snapshot, runtime, BASELINE["rename_map"])
    assert set(expected) == set(actual), "Missing or extra output file"
    expected_dir, actual_dir = tmp_path / "canonical-old", tmp_path / "canonical-new"
    for directory, files in ((expected_dir, expected), (actual_dir, actual)):
        for relative, text in files.items():
            path = directory / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
    different = [
        name
        for name in expected
        if not filecmp.cmp(expected_dir / name, actual_dir / name, shallow=False)
    ]
    assert not different, f"Canonical output differences: {different}"
    assert result.controller_order == yaml.safe_load(
        (snapshot / "controller_order.yaml").read_text()
    )


def test_rolling_digests_are_computed_by_the_composer(shares, tmp_path):
    # The controller only compares these; robot_interfaces endpoints.yaml pins the axis set.
    result = compose(
        **inputs(shares), runtime_dir=tmp_path / "runtime", use_mock_hardware=True, jtc_only=False
    )
    parameters = result.controllers["rolling_trajectory_controller"]["ros__parameters"]
    assert parameters["axis_set_hash"] == (
        "f4c8ff8a32183d1733032494600c9f5d1e7e625673a4ad51556b3ac6c35adee4"
    )
    envelope = result.files[Path(parameters["envelope_file"]).name]
    assert parameters["limits_version"] == sha256(envelope.encode("utf-8")).hexdigest()
    assert parameters["limits_version"] == (
        "1673393117a50ed04b7b9fa99dd01dbeb171cc691e9733a65e49fb13b418baab"
    )


def facts_subset(facts):
    return {key: facts[key] for key in ("use_mock_hardware", "jtc_only")}


def test_real_calibration_rejected_while_mock_is_offline(shares, tmp_path, monkeypatch):
    arguments = inputs(shares)
    with pytest.raises(ValueError, match="calibration is not verified"):
        compose(**arguments, runtime_dir=tmp_path / "real", use_mock_hardware=False)
    monkeypatch.setattr(
        OWNER, "preflight", lambda *args, **kwargs: pytest.fail("Mock accessed preflight")
    )
    result = compose(**arguments, runtime_dir=tmp_path / "mock")
    preflight(result)
    assert not (tmp_path / "real").exists()


def test_required_tbd_rejects_real_and_mock_uses_no_guessed_scaling(shares, tmp_path):
    arguments = inputs(shares, verified=True)
    data = shares["hwcfg_machines"]
    manifest = read_manifest(data / "machines" / arguments["robot_variant"] / "manifest.yaml")
    reference = manifest["modules"][
        manifest["scopes"][arguments["control_scope"]]["enabled_modules"][0]
    ]["device_refs"][0]
    path = data / reference
    device = load_yaml(path)
    device["output_encoder_counts_per_revolution"] = "TBD"
    path.write_text(yaml.safe_dump(device))
    with pytest.raises(ValueError, match="output_encoder_counts_per_revolution"):
        compose(**arguments, runtime_dir=tmp_path / "real", use_mock_hardware=False)
    compose(**arguments, runtime_dir=tmp_path / "mock")


def test_owner_conversion_roundtrip_and_reduction_ratio_independence(shares):
    for direction in (1, -1):
        factors = OWNER.position_factors(4096, direction, 123)
        counts = factors["command_offset"] + factors["command_factor"] * 0.75
        assert counts * factors["feedback_factor"] + factors["feedback_offset"] == pytest.approx(
            0.75
        )
    data = shares["hwcfg_machines"]
    manifest = read_manifest(next((data / "machines").glob("*/manifest.yaml")))
    reference = manifest["modules"]["arms"]["device_refs"][0]
    device = {
        "family": load_yaml(data / reference)["family"],
        "output_encoder_counts_per_revolution": 4096,
        "reduction_ratio": "TBD",
    }
    assert OWNER.validate_device(device, render_mode="normal") == device
    with pytest.raises(ValueError, match="declares no position count field"):
        OWNER.validate_device({**device, "family": "unregistered"}, render_mode="normal")


def test_shared_system_conflicts_and_preflight_once(monkeypatch):
    result = load_yaml(PACKAGE / "test/conformance/render_result.yaml")
    # XML parts are irrelevant to this check; result identity and parameters are not.
    result.update(
        joints=[],
        sensors=[],
        gpios=[],
        runtime_files={},
        managed_joints=[],
        resource_claims=[],
        actuator_groups={},
    )
    result["expected_topology"] = {"preflight_required": True}
    mismatch = deepcopy(result)
    mismatch["hardware_params"]["control_frequency"] += 1
    with pytest.raises(ValueError, match="Conflicting"):
        merge_systems([("synthetic_owner", result), ("synthetic_owner", mismatch)])
    mismatch = deepcopy(result)
    mismatch["hardware_plugin"] += "_other"
    with pytest.raises(ValueError, match="Conflicting"):
        merge_systems([("synthetic_owner", result), ("synthetic_owner", mismatch)])
    systems = merge_systems([("synthetic_owner", result), ("synthetic_owner", result)])
    calls = []

    class Renderer:
        def preflight(self, topology, *, run=None):
            calls.append(topology)

    import importlib

    module = importlib.import_module("integ_composer.compose")
    monkeypatch.setattr(module, "select_renderer", lambda *args: Renderer())
    from integ_composer.model import Composed

    preflight(Composed("", {}, {}, Path("."), systems, False, {}, {}))
    assert len(calls) == 1


def test_data_files_pass_existing_validator(shares):
    root = shares["hwcfg_machines"]
    passed = VALIDATOR.validate_machine_tree(root, PACKAGE / "schema", shares.__getitem__)
    assert "PASS: machines/Alfa_V3/manifest.yaml (manifest.schema.json)" in passed
    assert any(line.startswith("PASS: units/") for line in passed)


def test_policy_values_equal_frozen_old_output():
    # The frozen old output remains the value oracle after retiring old code.

    manifest_path = next((RT / "hwcfg_machines/machines").glob("*/manifest.yaml"))
    policy = load_yaml(
        RT / "hwcfg_machines" / load_yaml(manifest_path)["policy_refs"]["motion"]["file"]
    )
    old = load_yaml(GOLDEN / "arm-mock/controllers.yaml")
    assert policy["joints"] == old["whole_body_jtc"]["ros__parameters"]["joints"]
    expected_jtc = deepcopy(old["whole_body_jtc"]["ros__parameters"])
    expected_jtc.pop("joints")
    assert policy["jtc"] == expected_jtc
    expected_rolling = deepcopy(old["rolling_trajectory_controller"]["ros__parameters"])
    # Composer-derived values, not policy data (see test_rolling_digests_are_computed_by_the_composer).
    for derived in ("envelope_file", "axis_set_hash", "limits_version"):
        expected_rolling.pop(derived)
    assert expected_rolling.pop("joints") == policy["joints"]
    assert policy["rolling"] == expected_rolling
    assert policy["enable_manager"] == {
        key: value
        for key, value in old["enable_manager"]["ros__parameters"].items()
        if key.startswith("mode_switch_")
    }


def test_sample_a_conformance_is_read_only_and_keeps_unknown_xml(shares, monkeypatch):
    request = sample_request(shares)
    before = deepcopy(request)
    import io

    open_read = io.open

    def guarded_open(file, mode="r", *args, **kwargs):
        if any(flag in mode for flag in "wax+") or "evidence" in Path(file).parts:
            pytest.fail("Renderer attempted a write or archive read")
        return open_read(file, mode, *args, **kwargs)

    monkeypatch.setattr(io, "open", guarded_open)

    def deny_write(*args, **kwargs):
        pytest.fail("Renderer attempted filesystem or device access")

    monkeypatch.setattr(Path, "write_text", deny_write)
    monkeypatch.setattr(OWNER.subprocess, "check_output", deny_write)
    result = render_checked(OWNER, request, PACKAGE / "schema", shares["hwcfg_machines"])
    assert request == before
    assert len(result["joints"]) == request["module"]["actuator_count"]
    assert [element.attrib["name"] for element in result["joints"]] == result["managed_joints"]
    first = result["joints"][0]
    first.tail = "unknown tail"
    with pytest.raises(ValueError, match="tail"):
        project(first)
    first.tail = None
    first.attrib["unknown_attribute"] = "preserve then reject"
    from jsonschema import ValidationError

    projection = {
        **result,
        "joints": [project(element) for element in result["joints"]],
        "sensors": [project(element) for element in result["sensors"]],
    }
    with pytest.raises(ValidationError):
        validate_data(
            projection,
            "render_result.schema.json",
            PACKAGE / "schema",
            shares["hwcfg_machines"],
        )


def test_sample_a_real_required_fact_and_verification(shares):
    request = sample_request(shares)
    request["use_mock_hardware"] = False
    request["machine"]["verified"] = False
    with pytest.raises(ValueError, match="not verified"):
        OWNER.render(request)
    request["machine"]["verified"] = True
    device = next(iter(request["devices"].values()))
    device["output_encoder_counts_per_revolution"] = "TBD"
    with pytest.raises(ValueError, match="output_encoder_counts_per_revolution"):
        OWNER.render(request)
    request["use_mock_hardware"] = True
    OWNER.render(request)


def test_stationary_preflight_preserves_old_checks_with_injected_cli(shares, monkeypatch):
    request = sample_request(shares)
    request["render_mode"] = "stationary_enable"
    request["machine"]["spec"]["branches"]["synthetic"]["axes"][0]["encoder_policy"] = "single_turn"
    topology = OWNER.render(request)["expected_topology"]
    calls = []
    error = {"value": 0}

    def run(command, **kwargs):
        calls.append(command)
        assert command[0] == "ethercat" and command[1] in {"master", "slaves", "upload"}
        if command[1] == "master":
            return f"Active: no\nSlaves: {len(topology['responders'])}\n"
        if command[1] == "slaves":
            blocks = []
            for identity in topology["responders"]:
                blocks.append(
                    f"=== Master {topology['master_id']}, Slave {identity['ring_position']} ===\n"
                    f"Vendor Id: {hex(identity['vendor_id'])}\nProduct code: {hex(identity['product_code'])}\n"
                    f"Revision number: {hex(identity['revision_number'])}\nState: PREOP\n"
                )
            first = topology["branches"][0]
            blocks[0] += f"  {first['main_esc_port']} MII up open yes {first['downstream']}\n"
            return "".join(blocks)
        index = int(command[-2], 16)
        position = int(command[command.index("-p") + 1])
        value = (
            64
            if index == 0x6041
            else error["value"]
            if index == 0x603F and position == topology["axes"][0]["ring_position"]
            else 0
        )
        return f"uint16 {value}\n"

    OWNER.preflight(topology, run=run)
    error["value"] = topology["axes"][0]["allowed_errors"][0]
    OWNER.preflight(topology, run=run)
    error["value"] += 1
    with pytest.raises(RuntimeError, match="Unqualified error"):
        OWNER.preflight(topology, run=run)
    assert calls


def test_owner_import_has_no_integration_dependency():
    import os
    import subprocess

    code = """
import importlib.abc
import sys
class DenyIntegration(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'integ_composer' or fullname.startswith('integ_composer.'):
            raise RuntimeError('Protocol imported integration')
sys.meta_path.insert(0, DenyIntegration())
from hw_ethercat import rt_control_renderer
assert rt_control_renderer.RENDERER_API_VERSION == 1
"""
    env = {
        **os.environ,
        "PYTHONPATH": str(RT / "hw_ethercat") + os.pathsep + os.environ["PYTHONPATH"],
    }
    result = subprocess.run(
        [sys.executable, "-c", code], env=env, text=True, capture_output=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_renderer_api_rejected_before_invocation(monkeypatch):
    import importlib

    class Renderer:
        RENDERER_API_VERSION = 2

    monkeypatch.setattr(importlib, "import_module", lambda name: Renderer())
    with pytest.raises(ValueError, match="Unsupported renderer API"):
        select_renderer({"synthetic_transport": "synthetic_owner"}, "synthetic_transport")
