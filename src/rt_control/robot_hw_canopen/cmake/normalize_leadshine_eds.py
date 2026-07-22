#!/usr/bin/env python3
"""Create the byte-preserving dcfgen-compatible derivative of LD2-CAN.eds."""

import argparse
import hashlib
from pathlib import Path


SOURCE_SHA256 = "e1abc580b76d0548c5eedfbb6461b9ad3f3607b90ecbc087f042cc1573cf9c08"
DERIVED_SHA256 = "f610ba8d7a3c5741a1703eb2f5112eaed55bdba204413cf67149edb0698a64be"
REPLACEMENTS = (
    (b"SupportedObjects=0x0003", b"SupportedObjects=3", 1),
    (b"SupportedObjects=0x01CA", b"SupportedObjects=458", 1),
    (b"SupportedObjects=0x0054", b"SupportedObjects=84", 1),
    (b"$NodeID", b"$NODEID", 9),
)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    args = parser.parse_args()

    source = args.input.read_bytes()
    source_hash = sha256(source)
    if source_hash != SOURCE_SHA256:
        raise SystemExit(
            f"Leadshine authority EDS hash mismatch: expected {SOURCE_SHA256}, got {source_hash}"
        )

    derived = source
    replacement_count = 0
    for old, new, expected_count in REPLACEMENTS:
        actual_count = derived.count(old)
        if actual_count != expected_count:
            raise SystemExit(
                f"Expected {expected_count} occurrences of {old!r}, found {actual_count}"
            )
        derived = derived.replace(old, new)
        replacement_count += actual_count

    derived_hash = sha256(derived)
    if derived_hash != DERIVED_SHA256:
        raise SystemExit(
            f"Derived EDS hash mismatch: expected {DERIVED_SHA256}, got {derived_hash}"
        )
    if replacement_count != 12:
        raise SystemExit(f"Expected exactly 12 replacements, made {replacement_count}")

    args.output.write_bytes(derived)
    args.log.write_text(
        "\n".join(
            (
                f"source_file={args.input}",
                f"source_sha256={source_hash}",
                f"derived_file={args.output}",
                f"derived_sha256={derived_hash}",
                "supported_objects_replacements=3",
                "node_id_case_replacements=9",
                "total_replacements=12",
                "",
            )
        ),
        encoding="utf-8",
    )
    print(f"Prepared dcfgen-compatible Leadshine EDS {derived_hash}")


if __name__ == "__main__":
    main()
