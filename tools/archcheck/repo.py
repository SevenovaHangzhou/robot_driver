"""Repository model shared by the rules: files, packages, layers and declared exceptions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path, PurePosixPath
import subprocess
import xml.etree.ElementTree as ET

import yaml

RULES_FILE = "tools/arch_rules.yaml"
PREFIX = "src/rt_control/"

# ARCHITECTURE.md §1.3: the package prefix names the layer. hwcfg_ is tested before hw_.
LAYER_PREFIXES = (
    ("infra_", "infra"),
    ("hwcfg_", "hw_config"),
    ("hw_", "protocol"),
    ("ctrl_", "control"),
    ("periph_", "periph"),
    ("integ_", "integration"),
)
# Lower layers must not depend on higher ones; periph stands beside them (LAYER-2).
LAYER_ORDER = {"infra": 0, "hw_config": 1, "protocol": 2, "control": 3, "integration": 4}


@dataclass(frozen=True)
class Finding:
    rule: str
    path: str
    line: int
    message: str

    def describe(self) -> str:
        return f"{self.path}:{self.line}: {self.rule} {self.message}"


def layer_of(package: str) -> str | None:
    return next((layer for prefix, layer in LAYER_PREFIXES if package.startswith(prefix)), None)


def load_texts(root: Path) -> dict[str, str]:
    """Tracked and untracked, not ignored, UTF-8 text files under root."""
    listing = subprocess.run(
        ["git", "-C", str(root), "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        check=True,
        capture_output=True,
    ).stdout
    texts = {}
    for raw in listing.split(b"\0"):
        path = raw.decode("utf-8")
        if not path:
            continue
        try:
            texts[path] = (root / path).read_text(encoding="utf-8")
        except (FileNotFoundError, IsADirectoryError, UnicodeDecodeError):
            continue  # deleted in the work tree, a submodule, or binary
    return texts


def package_inventory(texts: dict[str, str]) -> dict[str, str]:
    """Directory under src/rt_control -> package name from its package.xml."""
    packages = {}
    for path, text in texts.items():
        if not PurePosixPath(path).match(PREFIX + "*/package.xml"):
            continue
        try:
            name = ET.fromstring(text).findtext("name", "").strip()
        except ET.ParseError:
            continue  # repository_gate reports invalid XML
        packages[path.split("/")[2]] = name
    return packages


@dataclass(frozen=True)
class Rules:
    exceptions: tuple[dict, ...]
    listen_only_peripherals: frozenset[str]
    vendor_names: tuple[str, ...]


def read_rules(texts: dict[str, str]) -> tuple[Rules, list[Finding]]:
    """tools/arch_rules.yaml: approved exceptions, listen-only peripherals and vendor names."""
    empty = Rules((), frozenset(), ())
    try:
        data = yaml.safe_load(texts.get(RULES_FILE, "")) or {}
    except yaml.YAMLError as exc:
        return empty, [Finding("ARCH-CONFIG", RULES_FILE, 1, f"YAML 无效：{exc}")]
    findings = []
    exceptions = data.get("exceptions", [])
    listen_only = data.get("listen_only_peripherals", [])
    vendor_names = data.get("vendor_names", [])
    if not all(isinstance(value, list) for value in (exceptions, listen_only, vendor_names)):
        return empty, [
            Finding(
                "ARCH-CONFIG",
                RULES_FILE,
                1,
                "exceptions、listen_only_peripherals 与 vendor_names 必须是列表",
            )
        ]
    valid = []
    for entry in exceptions:
        if not isinstance(entry, dict) or set(entry) != {"rule", "path", "reason", "approved"}:
            findings.append(
                Finding(
                    "ARCH-CONFIG", RULES_FILE, 1, f"例外须含 rule/path/reason/approved：{entry!r}"
                )
            )
            continue
        valid.append(entry)
    for package in listen_only:
        if layer_of(str(package)) != "periph":
            findings.append(
                Finding(
                    "ARCH-CONFIG",
                    RULES_FILE,
                    1,
                    f"listen_only_peripherals 只列 periph 包：{package}",
                )
            )
    rules = Rules(tuple(valid), frozenset(map(str, listen_only)), tuple(map(str, vendor_names)))
    return rules, findings


def apply_exceptions(findings: list[Finding], rules: Rules, checked: set[str]) -> list[Finding]:
    """Drops findings an approved exception covers (same rule and file).

    An exception for a checked rule that no longer matches anything is reported, so the list
    only holds live, approved deviations.
    """
    result = []
    used = set()
    for finding in findings:
        match = next(
            (
                index
                for index, entry in enumerate(rules.exceptions)
                if entry["rule"] == finding.rule and entry["path"] == finding.path
            ),
            None,
        )
        if match is None:
            result.append(finding)
        else:
            used.add(match)
    for index, entry in enumerate(rules.exceptions):
        if index not in used and entry["rule"].split("-")[0] in checked:
            result.append(
                Finding(
                    "ARCH-CONFIG",
                    RULES_FILE,
                    1,
                    f"例外已不再命中，应删除：{entry['rule']} {entry['path']}",
                )
            )
    return result
