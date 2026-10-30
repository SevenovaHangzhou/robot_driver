"""Offline contract acceptance: installed schemas, synthetic inputs, and rejection paths."""

import copy
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest
import yaml
from jsonschema import ValidationError

ROOT = Path(__file__).resolve().parents[1]
SCHEMA_DIR = Path(os.environ["COMPOSER_SCHEMA_DIR"])
SPEC = importlib.util.spec_from_file_location(
    "config_validator", ROOT / "scripts/validate_config_files.py"
)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)
INDEX = yaml.safe_load((ROOT / "test/fixture_index.yaml").read_text())


def validate(data, schema, share_root=ROOT / "test"):
    VALIDATOR.validate_data(data, schema, SCHEMA_DIR, share_root)


@pytest.mark.parametrize("entry", INDEX, ids=lambda entry: entry["file"])
@pytest.mark.parametrize("mutation", ["valid", "missing", "extra", "wrong_type"])
def test_synthetic_contract_accepts_valid_and_rejects_invalid_shape(entry, mutation):
    data = VALIDATOR.load_yaml(ROOT / "test" / entry["file"])
    schema = json.loads((SCHEMA_DIR / entry["schema"]).read_text())
    field = schema["required"][0]
    if mutation == "missing":
        del data[field]
    elif mutation == "extra":
        data["unexpected"] = True
    elif mutation == "wrong_type":
        data[field] = []
    if mutation == "valid":
        validate(data, entry["schema"])
    else:
        with pytest.raises(ValidationError):
            validate(data, entry["schema"])


@pytest.mark.parametrize(
    "text",
    [
        "name: a\nname: b\n",
        "outer: {name: a, name: b}\n",
        "base: &base {name: a}\ncopy: *base\n",
        "name: &unused a\n",
        "'<<': {name: a}\n",
        "---\nname: a\n---\nname: b\n",
        "values: [0, .nan]\n",
        "values: {nested: -.Inf}\n",
        "value: 1.0e+999\n",
        "date: 2026-01-01\n",
        "1: value\n",
    ],
)
def test_yaml_boundary_rejects_ambiguous_or_non_json_values(tmp_path, text):
    path = tmp_path / "input.yaml"
    path.write_text(text)
    with pytest.raises(VALIDATOR.ConfigError):
        VALIDATOR.load_yaml(path)


def test_yaml_preserves_integer_hex_and_explicit_tbd(tmp_path):
    path = tmp_path / "input.yaml"
    path.write_text("position_object: 0x6064\nunknown: TBD\ncaptured: '2026-01-01'\n")
    assert VALIDATOR.load_yaml(path) == {
        "position_object": 24676,
        "unknown": "TBD",
        "captured": "2026-01-01",
    }


@pytest.mark.parametrize("reference", ["/etc/passwd", "../other", "a/../b", "C:/data", r"a\b"])
def test_share_paths_reject_escape_in_schema_and_resolver(tmp_path, reference):
    data = VALIDATOR.load_yaml(ROOT / "test/conformance/render_result.yaml")
    data["runtime_files"] = {reference: "text"}
    with pytest.raises(ValidationError):
        validate(data, "render_result.schema.json", tmp_path)
    with pytest.raises(VALIDATOR.ConfigError):
        VALIDATOR.resolve_share_path(tmp_path, reference)


def test_share_reference_rejects_symlink_escape(tmp_path):
    share = tmp_path / "share"
    share.mkdir()
    (share / "outside").symlink_to(tmp_path, target_is_directory=True)
    data = VALIDATOR.load_yaml(ROOT / "test/conformance/render_result.yaml")
    data["runtime_files"] = {"outside/output.yaml": "text"}
    with pytest.raises(ValidationError, match="escapes"):
        validate(data, "render_result.schema.json", share)


@pytest.mark.parametrize(
    "reference",
    [
        "https://invalid/schema.json",
        "file:///etc/passwd",
        "../x.json",
        "missing.schema.json",
        "%2e%2e/x.json",
    ],
)
def test_schema_store_rejects_non_installed_refs_without_network(tmp_path, reference):
    path = tmp_path / "bad.schema.json"
    path.write_text(json.dumps({"$ref": reference}))
    with pytest.raises(VALIDATOR.ConfigError):
        VALIDATOR.validate_data({}, path.name, tmp_path, tmp_path)


def test_schema_store_rejects_scope_redirect_and_symlink(tmp_path):
    root = tmp_path / "schema"
    root.mkdir()
    path = root / "bad.schema.json"
    path.write_text(json.dumps({"$id": "https://invalid/", "type": "object"}))
    with pytest.raises(VALIDATOR.ConfigError):
        VALIDATOR.validate_data({}, path.name, root, tmp_path)
    path.unlink()
    outside = tmp_path / "outside.json"
    outside.write_text("{}")
    path.symlink_to(outside)
    with pytest.raises(VALIDATOR.ConfigError):
        VALIDATOR.validate_data({}, path.name, root, tmp_path)


@pytest.mark.parametrize(
    "field", ["output_encoder_counts_per_revolution", "reduction_ratio", "nominal_wheel_radius_m"]
)
@pytest.mark.parametrize("value", [0, -1, True, "123"])
def test_device_physical_fields_reject_invalid_values(field, value):
    data = VALIDATOR.load_yaml(ROOT / "test/fixtures/sample_a/devices/synthetic_output_drive.yaml")
    data[field] = value
    with pytest.raises(ValidationError):
        validate(data, "device.schema.json")


def test_device_can_omit_non_required_facts_and_keep_tbd():
    data = VALIDATOR.load_yaml(ROOT / "test/fixtures/sample_a/devices/synthetic_output_drive.yaml")
    data.pop("reduction_ratio")
    data["output_encoder_counts_per_revolution"] = "TBD"
    validate(data, "device.schema.json")  # Structural validation is not runtime admission.


def test_module_private_spec_is_open_but_envelope_is_closed():
    data = VALIDATOR.load_yaml(ROOT / "test/conformance/render_request.yaml")["machine"]
    data["spec"]["future_protocol_field"] = {"value": "TBD"}
    validate(data, "module_envelope.schema.json")
    data["future_protocol_field"] = True
    with pytest.raises(ValidationError):
        validate(data, "module_envelope.schema.json")


@pytest.mark.parametrize(
    "mutation",
    [
        "sensor_command",
        "unknown_tag",
        "unknown_attr",
        "param_child",
        "interface_joint",
        "body_text",
        "namespace",
    ],
)
def test_xml_projection_schema_rejects_invalid_hierarchy(mutation):
    data = VALIDATOR.load_yaml(ROOT / "test/conformance/render_result.yaml")
    joint = data["joints"][0]
    interface = joint["children"][0]
    param = interface["children"][0]
    if mutation == "sensor_command":
        data["sensors"][0]["children"][0]["tag"] = "command_interface"
    elif mutation == "unknown_tag":
        joint["tag"] = "unknown"
    elif mutation == "unknown_attr":
        interface["attrib"]["other"] = "bad"
    elif mutation == "param_child":
        param["children"].append(copy.deepcopy(interface))
    elif mutation == "interface_joint":
        interface["children"] = [copy.deepcopy(joint)]
    elif mutation == "body_text":
        joint["text"] = "invalid"
    elif mutation == "namespace":
        joint["tag"] = "{synthetic}joint"
    with pytest.raises(ValidationError):
        validate(data, "render_result.schema.json")


def test_result_allows_absent_gpios_and_preserves_order():
    data = VALIDATOR.load_yaml(ROOT / "test/conformance/render_result.yaml")
    data.pop("gpios")
    original = copy.deepcopy(data)
    validate(data, "render_result.schema.json")
    assert data == original


def test_scope_override_requires_bq_and_transport_is_data():
    data = VALIDATOR.load_yaml(ROOT / "test/fixtures/sample_a/machines/SyntheticA/manifest.yaml")
    data["owner_refs"]["novel_transport"] = "synthetic_protocol"
    data["modules"]["articulated_fixture"]["transport"] = "novel_transport"
    validate(data, "manifest.schema.json")
    data["scopes"]["bench"]["ethercat_cycle_hz_override"] = 250
    with pytest.raises(ValidationError):
        validate(data, "manifest.schema.json")
    data["scopes"]["bench"]["bq_ref"] = "SYNTHETIC-ONLY"
    validate(data, "manifest.schema.json")


def test_cli_uses_installed_share_and_reports_rejection(tmp_path):
    prefix = tmp_path / "install"
    share = prefix / "share" / "synthetic_config"
    share.mkdir(parents=True)
    resource = prefix / "share/ament_index/resource_index/packages"
    resource.mkdir(parents=True)
    (resource / "synthetic_config").touch()
    shutil.copytree(SCHEMA_DIR, share / "schema")
    source = ROOT / "test/fixtures/sample_a/devices/synthetic_output_drive.yaml"
    shutil.copy(source, share / "device.yaml")
    env = {**os.environ, "AMENT_PREFIX_PATH": str(prefix)}
    command = [
        sys.executable,
        str(SCHEMA_DIR.parents[2] / "lib/integ_composer/validate_config_files.py"),
        "--share-package",
        "synthetic_config",
        "--schema-package",
        "synthetic_config",
        "--schema",
        "device.schema.json",
        "device.yaml",
    ]
    result = subprocess.run(command, env=env, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    (share / "device.yaml").write_text("name: a\nname: b\n")
    result = subprocess.run(command, env=env, capture_output=True, text=True, check=False)
    assert result.returncode == 1
    assert "Duplicate YAML key" in result.stderr
