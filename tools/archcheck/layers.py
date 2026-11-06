"""LAYER (ARCHITECTURE.md §1.3), OWN (§1.1) and the interim composer NAME rule.

Dependencies are read from package.xml, C/C++ includes, Python imports and literal CMake
find_package; tests count for dependencies. Content rules read production files only.
"""

from __future__ import annotations

import ast
from pathlib import PurePosixPath
import re
import xml.etree.ElementTree as ET

from .repo import LAYER_ORDER, PREFIX, Finding, Rules, layer_of, package_inventory

DATA_PACKAGE = "hwcfg_machines"
COMPOSER = "integ_composer"
SEMANTIC_PACKAGE = "hw_semantic_components"
SOURCE_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".cc", ".cxx"}
PRODUCT_NAMES = re.compile(
    r"alfa_v[0-9]|left_joint|right_joint|head_joint|head_pitch|EcCiA402Drive|"
    r"GenericEcSlave|KincoCyclicModeSlave|zeroerr|kinco|damiao|bluepoint",
    re.I,
)
# LAYER-5: the system calls and frame builders a bus writer would need.
BUS_WRITE = re.compile(r"::(?:write|send|sendto|sendmsg)\s*\(|make_nmt_start_frame")
INCLUDE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"]([^>"\n]+)[>"]', re.M)
FREQUENCY = re.compile(
    r"""\b(control_frequency|update_rate|cycle_hz)\b["']?\s*[:=]\s*["']?\d"""
    r'|name=["\'](control_frequency|update_rate|cycle_hz)["\']>\s*\d'
)


def production_file(path: str) -> bool:
    return not ({"test", "tests", "fixtures", "schema"} & set(PurePosixPath(path).parts))


def _python_dependencies(text: str) -> list[tuple[str, str, int, str]]:
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return []  # repository_gate reports invalid syntax
    lines = text.splitlines()
    result = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            targets = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            targets = [node.module]
        else:
            continue
        for target in targets:
            result.append(
                (target.split(".")[0], "import", node.lineno, lines[node.lineno - 1].strip())
            )
    return result


def dependencies(path: str, text: str) -> list[tuple[str, str, int, str]]:
    """(target package, kind, line, snippet) for every declared or used dependency."""
    result = []
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
                line = text.count("\n", 0, max(text.find(snippet), 0)) + 1
                result.append((target, node.tag, line, snippet))
    if suffix in SOURCE_SUFFIXES - {".py"}:
        for match in INCLUDE.finditer(text):
            result.append(
                (
                    match[1].split("/")[0],
                    "#include",
                    text.count("\n", 0, match.start()) + 1,
                    match[0].strip(),
                )
            )
    if suffix == ".py":
        result.extend(_python_dependencies(text))
    if path.endswith("CMakeLists.txt") or suffix == ".cmake":
        clean = re.sub(r"#[^\n]*", "", text)
        for match in re.finditer(r"\bfind_package\s*\(\s*([\w]+)", clean, re.I):
            result.append(
                (match[1], "find_package", clean.count("\n", 0, match.start()) + 1, match[0])
            )
    return result


def _direction_problem(source: str, target: str) -> str | None:
    """Why source must not depend on target (ARCHITECTURE.md §1.3), or None."""
    source_layer, target_layer = layer_of(source), layer_of(target)
    if source_layer is None or target_layer is None or source == target:
        return None
    if source_layer == "infra" and target_layer != "infra":
        return "infra 不依赖任何域包"
    if source_layer == "periph" and target_layer in {"protocol", "control", "integration"}:
        return "periph 只依赖 infra"
    if target_layer == "periph" and source_layer != "integration" and source_layer != "periph":
        return "只有 integration 可以依赖 periph"
    if source_layer == "control" and target_layer == "protocol" and target != SEMANTIC_PACKAGE:
        return f"control 只经 {SEMANTIC_PACKAGE} 使用 protocol"
    if (
        source_layer in LAYER_ORDER
        and target_layer in LAYER_ORDER
        and LAYER_ORDER[source_layer] < LAYER_ORDER[target_layer]
    ):
        return "下层不得依赖上层"
    return None


def check_layers(texts: dict[str, str], rules: Rules) -> list[Finding]:
    inventory = package_inventory(texts)
    findings = []
    for directory, name in sorted(inventory.items()):
        path = f"{PREFIX}{directory}/package.xml"
        if name != directory:
            findings.append(
                Finding("LAYER-1", path, 1, f"包名 {name} 必须与目录名 {directory} 一致")
            )
        if layer_of(directory) is None:
            findings.append(
                Finding(
                    "LAYER-1",
                    path,
                    1,
                    f"{directory} 的前缀不属于任何一层（infra_/hwcfg_/hw_/ctrl_/periph_/integ_）",
                )
            )
    for path, text in sorted(texts.items()):
        if path.startswith(PREFIX):
            package = inventory.get(path.split("/")[2])
            if package is None:
                continue
        elif path.startswith("src/"):
            package = None  # shared packages: only the data-package rule applies
        else:
            continue
        for target, kind, line, snippet in dependencies(path, text):
            if target == DATA_PACKAGE and kind != "exec_depend" and package != DATA_PACKAGE:
                findings.append(
                    Finding(
                        "LAYER-3",
                        path,
                        line,
                        f"{DATA_PACKAGE} 只放数据，只能作为 exec_depend 引用：{snippet}",
                    )
                )
            elif package is not None:
                problem = _direction_problem(package, target)
                if problem:
                    findings.append(
                        Finding(
                            "LAYER-2",
                            path,
                            line,
                            f"{package}（{layer_of(package)}）→ {target}（{layer_of(target)}）：{problem}；{snippet}",
                        )
                    )
        if package == DATA_PACKAGE and PurePosixPath(path).suffix in {".py", ".cpp", ".hpp"}:
            findings.append(Finding("LAYER-3", path, 1, f"{DATA_PACKAGE} 不放代码文件"))
        if package and layer_of(package) == "periph" and f"/include/{package}/protocol/" in path:
            for match in INCLUDE.finditer(text):
                target = match[1].split("/")[0]
                if target.startswith("rclcpp") or target.endswith("_msgs"):
                    findings.append(
                        Finding(
                            "LAYER-4",
                            path,
                            text.count("\n", 0, match.start()) + 1,
                            f"外设协议层不依赖 ROS：{match[0].strip()}",
                        )
                    )
        if (
            package in rules.listen_only_peripherals
            and production_file(path)
            and PurePosixPath(path).suffix in SOURCE_SUFFIXES - {".py"}
        ):
            for number, line in enumerate(text.splitlines(), 1):
                if BUS_WRITE.search(line):
                    findings.append(
                        Finding(
                            "LAYER-5",
                            path,
                            number,
                            f"只读外设 {package} 不得写总线：{line.strip()}",
                        )
                    )
    return findings


def _production_lines(texts: dict[str, str]):
    """(path, package, line number, line) of production files of domain packages."""
    inventory = package_inventory(texts)
    for path, text in sorted(texts.items()):
        if not path.startswith(PREFIX) or not production_file(path):
            continue
        package = inventory.get(path.split("/")[2])
        if package is None:
            continue
        for number, line in enumerate(text.splitlines(), 1):
            yield path, package, number, line


def check_ownership(texts: dict[str, str]) -> list[Finding]:
    findings = []
    for path, package, number, line in _production_lines(texts):
        file = PurePosixPath(path)
        if (
            file.suffix in SOURCE_SUFFIXES | {".yaml", ".yml", ".xacro"}
            and not file.match(f"{PREFIX}{DATA_PACKAGE}/machines/*/manifest.yaml")
            and not file.match(f"{PREFIX}{DATA_PACKAGE}/machines/*/manifest/*.yaml")
            and path != f"{PREFIX}{COMPOSER}/config/controller_defaults.yaml"
            and not line.lstrip().startswith(("#", "//", "<!--"))
            and FREQUENCY.search(line)
        ):
            findings.append(
                Finding(
                    "OWN-1",
                    path,
                    number,
                    f"控制频率只写在 manifest 与 {COMPOSER} 默认值：{line.strip()}",
                )
            )
        if (
            file.suffix in SOURCE_SUFFIXES
            and (package == COMPOSER or "renderer" in file.stem or "/renderers/" in path)
            and "evidence/" in line
        ):
            findings.append(
                Finding("OWN-2", path, number, f"组合器与渲染器不读取 evidence/：{line.strip()}")
            )
    return findings


def check_names(texts: dict[str, str]) -> list[Finding]:
    """Interim NAME-1: machine and product names stay out of the composer's code."""
    return [
        Finding("NAME-1", path, number, f"{COMPOSER} 不写机型或产品名：{line.strip()}")
        for path, package, number, line in _production_lines(texts)
        if package == COMPOSER
        and PurePosixPath(path).suffix in SOURCE_SUFFIXES
        and PRODUCT_NAMES.search(line)
    ]
