#!/usr/bin/env python3
"""Verify the supplied sibling robot model and export the remaining dependency manifest."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

import yaml


DOMAIN_TOKEN = re.compile(
    r"(?i)(?:ros2_control|hardware_plugin|"
    r"(?<![a-z0-9])(?:pdo|sdo|station|pick|place)(?![a-z0-9]))"
)


def check_robot_description_xml(relative_path: str, xml_text: str) -> list[str]:
    """Keep domain-specific data out of the shared model.

    ElementTree intentionally drops XML comments, so comments that explain the
    boundary do not trigger the content policy.
    """
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        return [f"{relative_path}: invalid robot-description XML: {exc}"]
    uncommented_xml = ET.tostring(root, encoding="unicode")
    match = DOMAIN_TOKEN.search(uncommented_xml)
    if match:
        return [
            f"{relative_path}: robot_description contains domain-specific token {match.group(0)!r}"
        ]
    return []


def model_pin(root: Path) -> dict:
    spec = yaml.safe_load((root / "deps.repos").read_text())["repositories"]["robot_description"]
    if spec.get("type") != "git" or not re.fullmatch(r"[0-9a-f]{40}", str(spec.get("version"))):
        raise ValueError(
            "deps.repos: robot_description requires a Git repository and full commit SHA"
        )
    if spec.get("url") != "https://github.com/SevenovaHangzhou/robot_description.git":
        raise ValueError(
            "deps.repos: robot_description must reference the authoritative repository"
        )
    return spec


def verify(root: Path) -> Path:
    spec = model_pin(root)
    model = root.parent / "robot_description"
    if not (model / ".git").exists():
        raise ValueError(
            f"missing model checkout: {model}; place the pinned repository beside the driver"
        )
    actual = subprocess.check_output(
        ["git", "-C", str(model), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual != spec["version"]:
        raise ValueError(f"{model}: HEAD {actual}, expected {spec['version']}; checkout preserved")
    dirty = subprocess.check_output(
        ["git", "-C", str(model), "status", "--porcelain", "--untracked-files=all"], text=True
    )
    if dirty:
        raise ValueError(f"{model}: local changes present; checkout preserved\n{dirty}")
    if not (model / "package.xml").is_file():
        raise ValueError(f"{model}: missing package.xml")
    for path in model.rglob("*"):
        if path.suffix in {".urdf", ".xacro", ".srdf"}:
            findings = check_robot_description_xml(str(path), path.read_text())
            if findings:
                raise ValueError("\n".join(findings))
    return model


def vendor_manifest(root: Path) -> dict:
    """Only the other upstream repositories are downloaded by driver builds."""
    model_pin(root)
    document = yaml.safe_load((root / "deps.repos").read_text())
    del document["repositories"]["robot_description"]
    return document


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("verify", "vendor-manifest"))
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        root = args.repo_root.resolve()
        if args.action == "vendor-manifest":
            print(yaml.safe_dump(vendor_manifest(root)), end="")
            return 0
        model = verify(root)
    except (KeyError, ValueError, OSError, subprocess.CalledProcessError, yaml.YAMLError) as exc:
        parser.exit(1, f"robot_description: {exc}\n")
    print(f"PASS: pinned model {model}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
