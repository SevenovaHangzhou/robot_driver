"""NAME (ARCHITECTURE.md §1.1): machine, vendor and joint names only in the layers that own them.

Machine names come from hwcfg_machines/machines/<variant>/ and joint names from its module
files, so the tool keeps no second copy of machine facts; vendor and product words are the
rule parameter `vendor_names` of tools/arch_rules.yaml. Code, configuration and their
comments are checked; Markdown, tests, fixtures and schemas are not. Package names (for
example a dependency on hw_can_damiao) are references, not facts, and are ignored.
"""

from __future__ import annotations

from pathlib import PurePosixPath
import re

import yaml

from .repo import PREFIX, Finding, layer_of, package_inventory

DATA_PACKAGE = "hwcfg_machines"
MACHINES = f"{PREFIX}{DATA_PACKAGE}/machines/"
CHECKED_SUFFIXES = {
    ".py",
    ".cpp",
    ".hpp",
    ".h",
    ".cc",
    ".cxx",
    ".cmake",
    ".yaml",
    ".yml",
    ".xml",
    ".json",
    ".xacro",
    ".urdf",
    ".srdf",
    ".js",
    ".html",
    ".css",
    ".sh",
}
NOT_CHECKED_PARTS = {"test", "tests", "fixtures", "schema", "golden"}
JOINT_KEYS = ("robot_model_joint", "joint_name")
IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def _checked_file(path: str) -> bool:
    file = PurePosixPath(path)
    in_scripts = "scripts" in file.parts and file.suffix == ""
    return (
        file.suffix in CHECKED_SUFFIXES or file.name == "CMakeLists.txt" or in_scripts
    ) and not (NOT_CHECKED_PARTS & set(file.parts))


def machine_names(texts: dict[str, str]) -> set[str]:
    return {path[len(MACHINES) :].split("/")[0] for path in texts if path.startswith(MACHINES)}


def joint_names(texts: dict[str, str]) -> set[str]:
    names = set()

    def walk(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key in JOINT_KEYS and isinstance(value, str) and IDENTIFIER.match(value):
                    names.add(value)
                walk(value)
        elif isinstance(node, list):
            for value in node:
                walk(value)

    for path, text in texts.items():
        if path.startswith(MACHINES) and path.endswith((".yaml", ".yml")):
            try:
                walk(yaml.safe_load(text))
            except yaml.YAMLError:
                continue  # repository_gate reports invalid YAML
    return names


def _words(names, *, case_sensitive: bool, separators: bool = False):
    if not names:
        return None
    parts = []
    for name in sorted(names, key=len, reverse=True):
        if separators:  # Alfa_V3, alfa-v3, ALFAV3 name the same variant
            parts.append(r"[_\- ]?".join(map(re.escape, re.split(r"[_\- ]", name))))
        else:
            parts.append(re.escape(name))
    flags = 0 if case_sensitive else re.I
    return re.compile(r"(?<![A-Za-z0-9])(?:" + "|".join(parts) + r")(?![A-Za-z0-9])", flags)


def check_names(texts: dict[str, str], vendor_names: tuple[str, ...]) -> list[Finding]:
    inventory = package_inventory(texts)
    package_words = _words(set(inventory.values()), case_sensitive=True)
    rules = (
        (
            "NAME-1",
            _words(machine_names(texts), case_sensitive=False, separators=True),
            lambda package: package == DATA_PACKAGE,
            "机型名只写在 hwcfg_machines",
        ),
        (
            "NAME-2",
            re.compile("|".join(map(re.escape, vendor_names)), re.I) if vendor_names else None,
            lambda package: package == DATA_PACKAGE or layer_of(package) in {"protocol", "periph"},
            "厂家与产品名只写在 hw_*、hwcfg_machines、periph_*",
        ),
        (
            "NAME-3",
            _words(joint_names(texts), case_sensitive=True),
            lambda package: package == DATA_PACKAGE,
            "关节名只写在 hwcfg_machines",
        ),
    )
    findings = []
    for path, text in sorted(texts.items()):
        if not path.startswith(PREFIX) or not _checked_file(path):
            continue
        package = inventory.get(path.split("/")[2])
        if package is None:
            continue
        active = [
            (rule, pattern, why)
            for rule, pattern, allowed, why in rules
            if pattern and not allowed(package)
        ]
        if not active:
            continue
        for number, line in enumerate(text.splitlines(), 1):
            bare = package_words.sub("", line) if package_words else line
            for rule, pattern, why in active:
                match = pattern.search(bare)
                if match:
                    findings.append(
                        Finding(rule, path, number, f"{why}：{match[0]}；{line.strip()[:120]}")
                    )
    return findings
