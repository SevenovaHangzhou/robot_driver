"""Offline four-layer policy (SPEC v1.1 R1–R10).

Only exact, counted baseline occurrences are exempt. Tests are included for
API dependencies; R5–R7 inspect production code/configuration, not test data.
R2/R3 cover XML, C++ includes, Python imports and literal CMake find_package.
"""

from __future__ import annotations

import ast
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import PurePosixPath
import re
import xml.etree.ElementTree as ET

import yaml

CONFIG = "tools/rt_control_layers.yaml"
PREFIX = "src/rt_control/"
DATA_PACKAGE = "rt_control_machine_config"
COMPOSER = "rt_control_composer"
LAYERS = {"hw_config", "integration", "protocol", "control"}
SOURCE_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".cc", ".cxx"}
PERIPHERALS = {"bms_node", "lpms_nav3_can", "modbus_tcp_rtu485", "plc_io_modbus"}
PRODUCT_NAMES = re.compile(
    r"alfa_v[0-9]|left_joint|right_joint|head_joint|head_pitch|EcCiA402Drive|"
    r"GenericEcSlave|KincoCyclicModeSlave|zeroerr|kinco|damiao|bluepoint",
    re.I,
)
INCLUDE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"]([^>"\n]+)[>"]', re.M)
FREQUENCY = re.compile(
    r"""\b(control_frequency|update_rate|cycle_hz)\b["']?\s*[:=]\s*["']?\d"""
    r'|name=["\'](control_frequency|update_rate|cycle_hz)["\']>\s*\d'
)


@dataclass(frozen=True)
class Violation:
    rule: str
    source_package: str
    target_package: str
    dependency_type: str
    file: str
    line: int
    text: str

    def key(self) -> tuple[str, ...]:
        # Line numbers are evidence, not exemptions: inserting a line cannot
        # waive a new occurrence, and moving unchanged code keeps its identity.
        return tuple(str(v) for k, v in asdict(self).items() if k != "line")

    def describe(self) -> str:
        return (
            f"{self.file}:{self.line}: {self.rule} {self.source_package} -> "
            f"{self.target_package} ({self.dependency_type}): {self.text}"
        )


def package_inventory(texts: dict[str, str]) -> dict[str, str]:
    packages = {}
    for path, text in texts.items():
        if not PurePosixPath(path).match("src/rt_control/*/package.xml"):
            continue
        try:
            name = ET.fromstring(text).findtext("name", "").strip()
        except ET.ParseError:
            continue  # repository_gate reports invalid XML
        if name:
            packages[path.split("/")[2]] = name
    return packages


def read_policy(text: str) -> tuple[dict[str, str], list[dict], list[str]]:
    try:
        policy = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        return {}, [], [f"{CONFIG}: R1 invalid YAML: {exc}"]
    if not isinstance(policy, dict) or not isinstance(policy.get("layers"), dict):
        return {}, [], [f"{CONFIG}: R1 layers must be a mapping"]
    findings = []
    mapping: dict[str, str] = {}
    if set(policy["layers"]) != LAYERS:
        findings.append(f"{CONFIG}: R1 requires exactly the four declared layers")
    for layer, packages in policy["layers"].items():
        if not isinstance(packages, list) or any(not isinstance(p, str) for p in packages):
            findings.append(f"{CONFIG}: R1 {layer} must be a package list")
            continue
        for package in packages:
            if package in mapping:
                findings.append(f"{CONFIG}: R1 {package} appears in multiple layer entries")
            mapping[package] = layer
    baselines = policy.get("baseline_violations", [])
    if not isinstance(baselines, list):
        return mapping, [], findings + [f"{CONFIG}: baseline_violations must be a list"]
    return mapping, baselines, findings


def python_dependencies(text: str) -> list[tuple[str, str, int, str]]:
    dependencies = []
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return dependencies  # repository_gate reports invalid syntax
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Import):
            targets = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            targets = [node.module]
        for target in targets:
            dependencies.append(
                (
                    target.split(".")[0],
                    "import",
                    node.lineno,
                    text.splitlines()[node.lineno - 1].strip(),
                )
            )
    return dependencies


def code_dependencies(path: str, text: str) -> list[tuple[str, str, int, str]]:
    dependencies = []
    suffix = PurePosixPath(path).suffix
    if path.endswith("/package.xml"):
        try:
            root = ET.fromstring(text)
        except ET.ParseError:
            return []
        for node in root:
            if node.tag == "depend" or node.tag.endswith("_depend"):
                target = (node.text or "").strip()
                snippet = f"<{node.tag}>{target}</{node.tag}>"
                offset = text.find(snippet)
                line = text.count("\n", 0, max(offset, 0)) + 1
                dependencies.append((target, node.tag, line, snippet))
    if suffix in SOURCE_SUFFIXES - {".py"}:
        for match in INCLUDE.finditer(text):
            dependencies.append(
                (
                    match[1].split("/")[0],
                    "#include",
                    text.count("\n", 0, match.start()) + 1,
                    match[0].strip(),
                )
            )
    if suffix == ".py":
        dependencies.extend(python_dependencies(text))
    if path.endswith("CMakeLists.txt") or suffix == ".cmake":
        clean = re.sub(r"#[^\n]*", "", text)
        for match in re.finditer(r"\bfind_package\s*\(\s*([\w]+)", clean, re.I):
            dependencies.append(
                (match[1], "find_package", clean.count("\n", 0, match.start()) + 1, match[0])
            )
    return dependencies


def dependency_violations(
    path: str, text: str, source: str, layers: dict[str, str]
) -> list[Violation]:
    violations = []
    for target, kind, line, snippet in code_dependencies(path, text):
        rule = None
        if target == DATA_PACKAGE and kind != "exec_depend":
            rule = "R4"
        else:
            if layers.get(source) == "control" and layers.get(target) == "protocol":
                if target != "rt_control_semantic_components":
                    rule = "R2"
            elif layers.get(source) == "protocol" and layers.get(target) == "control":
                rule = "R3"
        if rule:
            violations.append(Violation(rule, source, target, kind, path, line, snippet))
    return violations


def production_file(path: str) -> bool:
    parts = PurePosixPath(path).parts
    return not ({"test", "tests", "fixtures", "schema"} & set(parts))


def content_violations(path: str, text: str, package: str) -> list[Violation]:
    file = PurePosixPath(path)
    violations = []
    source = file.suffix in SOURCE_SUFFIXES
    if package == DATA_PACKAGE and file.suffix in {".py", ".cpp", ".hpp"}:
        violations.append(Violation("R4", package, package, "code", path, 1, file.name))
    if not production_file(path):
        return violations
    allowed_frequency = (
        file.match(f"{PREFIX}{DATA_PACKAGE}/machines/*/manifest.yaml")
        or path == f"{PREFIX}{COMPOSER}/config/controller_defaults.yaml"
    )
    for number, line in enumerate(text.splitlines(), 1):
        if source and package == COMPOSER and PRODUCT_NAMES.search(line):
            violations.append(
                Violation("R5", package, "", "product-name", path, number, line.strip())
            )
        if (
            source
            and (package == COMPOSER or "renderer" in file.stem or "/renderers/" in path)
            and "evidence/" in line
        ):
            violations.append(
                Violation("R7", package, "", "evidence-path", path, number, line.strip())
            )
        if (
            not allowed_frequency
            and file.suffix in SOURCE_SUFFIXES | {".yaml", ".yml", ".xacro"}
            and not line.lstrip().startswith(("#", "//", "<!--"))
            and FREQUENCY.search(line)
        ):
            violations.append(
                Violation("R6", package, "", "frequency-literal", path, number, line.strip())
            )
    return violations + peripheral_violations(path, text, package)


def peripheral_violations(path: str, text: str, package: str) -> list[Violation]:
    violations = []
    if package in PERIPHERALS and f"/include/{package}/protocol/" in path:
        for match in INCLUDE.finditer(text):
            target = match[1].split("/")[0]
            if target.startswith("rclcpp") or target.endswith("_msgs"):
                violations.append(
                    Violation(
                        "R8",
                        package,
                        target,
                        "#include",
                        path,
                        text.count("\n", 0, match.start()) + 1,
                        match[0].strip(),
                    )
                )
    return violations


def apply_baselines(violations: list[Violation], baselines: list[dict]) -> list[str]:
    exemptions: Counter = Counter()
    findings = []
    for entry in baselines:
        try:
            fields = {key: entry[key] for key in Violation.__dataclass_fields__}
            if entry["expires"] not in {"M3d", "M4c", "M6"} or entry["rule"] not in {"R2", "R6"}:
                raise ValueError("invalid rule or expiry")
            if not all(isinstance(v, str) for k, v in fields.items() if k != "line"):
                raise ValueError("invalid baseline fields")
            exemptions[Violation(**fields).key()] += 1
        except (KeyError, TypeError, ValueError):
            findings.append(f"{CONFIG}: invalid baseline entry {entry!r}")
    for violation in violations:
        key = violation.key()
        if exemptions[key]:
            exemptions[key] -= 1
        else:
            findings.append(violation.describe())
    # Stale entries may be removed by the owning migration; they never grant a
    # whole-file exemption or allow another value/dependency to replace them.
    return findings


def check_layers(texts: dict[str, str]) -> list[str]:
    layers, baselines, findings = read_policy(texts.get(CONFIG, ""))
    inventory = package_inventory(texts)
    actual = set(inventory.values())
    for path in texts:
        if PurePosixPath(path).match("src/rt_control/*/package.xml"):
            directory = path.split("/")[2]
            if inventory.get(directory) != directory:
                findings.append(f"{path}: R1 package name must match its directory")
    for package in sorted(actual ^ layers.keys()):
        findings.append(f"{CONFIG}: R1 {package} must exist and belong to exactly one layer")
    if len(actual) != len(inventory):
        findings.append(f"{CONFIG}: R1 duplicate package names in manifests")
    violations = []
    for path, text in texts.items():
        if not path.startswith(PREFIX):
            if path.startswith("src/"):
                violations.extend(dependency_violations(path, text, "<shared>", layers))
            continue
        package = inventory.get(path.split("/")[2])
        if package is not None:
            violations.extend(dependency_violations(path, text, package, layers))
            violations.extend(content_violations(path, text, package))
    return findings + apply_baselines(violations, baselines)


def check_release_policy(texts: dict[str, str]) -> list[str]:
    findings = []
    for path, text in texts.items():
        if (
            path.startswith(("docker/", ".github/workflows/"))
            or path == "tools/bootstrap_native_dev.sh"
        ):
            if re.search(r"(?<![\w.-])python3-jsonschema\s*=", text):
                findings.append(f"{path}: R10 python3-jsonschema must not pin an apt version")
    owners = texts.get(".github/CODEOWNERS", "")
    pattern = "/src/rt_control/rt_control_machine_config/**/safety_policy.yaml"
    if not any(line.split() == [pattern, "kkozia188@gmail.com"] for line in owners.splitlines()):
        findings.append(".github/CODEOWNERS: R9 safety policy requires the designated human owner")
    return findings
