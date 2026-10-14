#!/usr/bin/env python3
"""Offline validation for machine and policy YAML, using installed Draft 7 schemas."""

import argparse
from pathlib import Path

from jsonschema import ValidationError
from jsonschema.exceptions import RefResolutionError, SchemaError
import yaml

from rt_control_composer.schema.primitives import (
    ConfigError as ConfigError,
    load_and_validate,
    load_yaml as load_yaml,
    resolve_share_path,
    validate_data as validate_data,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--share-package", required=True, help="Package owning the YAML input")
    parser.add_argument("--schema-package", default="rt_control_composer")
    parser.add_argument(
        "--schema", required=True, help="Filename within the installed schema directory"
    )
    parser.add_argument("files", nargs="+", help="YAML paths relative to the input package share")
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory

    try:
        share_root = Path(get_package_share_directory(args.share_package))
        schema_dir = Path(get_package_share_directory(args.schema_package)) / "schema"
        for name in args.files:
            path = resolve_share_path(share_root, name)
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
