#!/usr/bin/env python3
"""Offline validation for machine and policy YAML, using installed Draft 7 schemas."""

import argparse
from pathlib import Path

from jsonschema import ValidationError
from jsonschema.exceptions import RefResolutionError, SchemaError
import yaml

from integ_composer.schema.manifest_load import read_manifest
from integ_composer.schema.primitives import (
    ConfigError as ConfigError,
    load_and_validate,
    load_yaml as load_yaml,
    resolve_share_path,
    validate_data as validate_data,
)


ENVELOPE_SCHEMAS = {
    "machine_module": "module_envelope",
    "unit_calibration": "unit_calibration_envelope",
}


def validate_machine_tree(share_root: Path, schema_dir: Path, share_of) -> list[str]:
    """Validate every machine-config YAML with the schema that owns it; returns PASS lines.

    `share_of(package)` resolves an owner package's share directory for policy schemas.
    """
    manifest = read_manifest(next((share_root / "machines").glob("*/manifest.yaml")))
    scopes = manifest["scopes"].values()
    # Robot model inputs and scope controller files are complete owner documents,
    # not machine envelopes.
    skipped = {s["mock_robot_model"]["joint_limits_file"] for s in scopes if "mock_robot_model" in s}
    skipped |= {s["controller_file"]["file"] for s in scopes if "controller_file" in s}
    # Owner policies have no machine envelope; each is validated by its owner's schema.
    policies = {ref["file"]: ref for ref in manifest["policy_refs"].values()}
    passed = []
    for directory in ("devices", "machines", "units"):
        for path in sorted((share_root / directory).rglob("*.yaml")):
            name = path.relative_to(share_root).as_posix()
            # Manifest parts are validated as part of their merged manifest.
            if name in skipped or path.parent.name == "manifest":
                continue
            document = read_manifest(path) if path.name == "manifest.yaml" else load_yaml(path)
            if name in policies:
                ref = policies[name]
                if "schema" in ref:
                    owner = Path(share_of(ref.get("schema_package", ref["package"])))
                    validate_data(document, ref["schema"], owner / "schema", share_root)
                    passed.append(f"PASS: {name} ({ref['schema']})")
                continue
            schema = ENVELOPE_SCHEMAS.get(document["kind"], document["kind"]) + ".schema.json"
            validate_data(document, schema, schema_dir, share_root)
            passed.append(f"PASS: {name} ({schema})")
    return passed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--share-package", default="hwcfg_machines", help="Package owning the YAML input"
    )
    parser.add_argument("--schema-package", default="integ_composer")
    parser.add_argument("--schema", help="Filename within the installed schema directory")
    parser.add_argument("files", nargs="*", help="YAML paths relative to the input package share")
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory

    try:
        share_root = Path(get_package_share_directory(args.share_package))
        schema_dir = Path(get_package_share_directory(args.schema_package)) / "schema"
        if args.files and not args.schema:
            parser.error("--schema is required when explicit files are supplied")
        if not args.files:
            for line in validate_machine_tree(share_root, schema_dir, get_package_share_directory):
                print(line)
        for name in args.files:
            path = resolve_share_path(share_root, name)
            if path.name == "manifest.yaml":
                validate_data(read_manifest(path), args.schema, schema_dir, share_root)
            else:
                load_and_validate(path, args.schema, schema_dir, share_root)
            print(f"PASS: {name} ({args.schema})")
    except (
        ValueError,
        OSError,
        LookupError,
        yaml.YAMLError,
        ValidationError,
        SchemaError,
        RefResolutionError,
    ) as error:
        parser.exit(1, f"Configuration rejected: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
