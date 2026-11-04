#!/usr/bin/env python3
"""Validate machine/unit selections without rendering or accessing hardware."""

import argparse
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory

from integ_composer.manifest import validate_selection
from integ_composer.schema.manifest_load import load_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--unit", required=True)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--physical-profile")
    parser.add_argument("--control-scope")
    parser.add_argument("--require-runtime-ready", action="store_true")
    args = parser.parse_args()
    try:
        root = args.manifest.resolve().parents[2]
        schemas = Path(get_package_share_directory("integ_composer")) / "schema"
        manifest = load_manifest(args.manifest, schemas, root)
        pairs = (
            [
                (profile, scope)
                for profile, entry in manifest["profiles"].items()
                for scope in entry["allowed_scopes"]
            ]
            if args.all
            else [(args.physical_profile, args.control_scope)]
        )
        if not args.all and not all(pairs[0]):
            parser.error("--physical-profile and --control-scope are required unless --all is used")
        for profile, scope in pairs:
            validate_selection(
                robot_variant=manifest["robot_variant"],
                machine_unit=args.unit,
                physical_profile=profile,
                control_scope=scope,
                require_runtime_ready=args.require_runtime_ready,
                config_share=root,
            )
        print(f"Validated {len(pairs)} profile/scope selections for {args.unit}")
        return 0
    except (ValueError, OSError, LookupError) as error:
        print(f"Configuration rejected: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
