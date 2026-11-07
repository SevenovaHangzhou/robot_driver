"""Repository model shared by the rules: files, packages, layers and declared exceptions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path, PurePosixPath
import re
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
    exceptions: tuple[dict, ...] = ()
    listen_only_peripherals: frozenset[str] = frozenset()
    vendor_names: tuple[str, ...] = ()
    duplicates: tuple[dict, ...] = ()


def _config(message: str) -> Finding:
    return Finding("ARCH-CONFIG", RULES_FILE, 1, message)


def read_rules(texts: dict[str, str]) -> tuple[Rules, list[Finding]]:
    """tools/arch_rules.yaml: exceptions, listen-only peripherals, vendor names, duplicates."""
    try:
        data = yaml.safe_load(texts.get(RULES_FILE, "")) or {}
    except yaml.YAMLError as exc:
        return Rules(), [_config(f"YAML 无效：{exc}")]
    keys = ("exceptions", "listen_only_peripherals", "vendor_names", "duplicates")
    values = {key: data.get(key, []) for key in keys}
    if not all(isinstance(value, list) for value in values.values()):
        return Rules(), [_config("、".join(keys) + " 必须是列表")]
    findings = []
    exceptions = []
    for entry in values["exceptions"]:
        if isinstance(entry, dict) and set(entry) == {"rule", "path", "reason", "approved"}:
            exceptions.append(entry)
        else:
            findings.append(_config(f"例外须含 rule/path/reason/approved：{entry!r}"))
    for package in values["listen_only_peripherals"]:
        if layer_of(str(package)) != "periph":
            findings.append(_config(f"listen_only_peripherals 只列 periph 包：{package}"))
    duplicates = []
    for entry in values["duplicates"]:
        try:
            if not isinstance(entry, dict) or set(entry) != {"name", "owner", "pattern"}:
                raise ValueError("须含 name/owner/pattern")
            re.compile(entry["pattern"])
            duplicates.append(entry)
        except (ValueError, TypeError, re.error) as exc:
            findings.append(_config(f"重复实现特征无效（{exc}）：{entry!r}"))
    rules = Rules(
        tuple(exceptions),
        frozenset(map(str, values["listen_only_peripherals"])),
        tuple(map(str, values["vendor_names"])),
        tuple(duplicates),
    )
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
