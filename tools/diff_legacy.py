#!/usr/bin/env python3
"""Placeholder entry point for the T-005 semantic migration gate."""

import argparse


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare frozen legacy configuration with migrated configuration (T-005)."
    )
    parser.add_argument("--legacy-root")
    parser.add_argument("--target-root")
    args = parser.parse_args()
    if args.legacy_root or args.target_root:
        parser.error("T-005 is not implemented; migration comparison must not be bypassed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
