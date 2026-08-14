#!/usr/bin/env python3
"""Repository-wide architecture and file-hygiene gate.

The same executable is used by local pre-commit hooks and CI.  Checks are
deliberately deterministic and do not access hardware or the network.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath
from typing import Sequence

import yaml


FULL_SHA = re.compile(r"^[0-9a-fA-F]{40}$")
SEMVER = re.compile(r"^[0-9]+\.[0-9]+\.[0-9]+$")
ROBOT_INTERFACES_PATH = "src/vendor/robot_interfaces"
ROBOT_INTERFACES_URL = "https://github.com/SevenovaHangzhou/robot_interfaces.git"
ROBOT_INTERFACES_PACKAGES = [
    "robot_rt_control_interfaces",
    "robot_system_interfaces",
    "robot_interfaces_qos",
]
GENERATED_PARTS = {"build", "install", "log", ".colcon", "__pycache__"}
DOMAIN_PACKAGES = {
    "bms_node",
    "control_api_adapter",
    "enable_manager",
    "plc_node",
    "robot_hw_canopen",
    "robot_hw_ethercat",
    "rt_control_bringup",
    "rt_diagnostics",
    "rt_watchdog",
}
SHARED_PACKAGE_PREFIXES = (
    "src/description/robot_description/",
    "src/interfaces/",
)
DOMAIN_TOKEN = re.compile(
    r"(?i)(?:ros2_control|hardware_plugin|"
    r"(?<![a-z0-9])(?:pdo|sdo|station|pick|place)(?![a-z0-9]))"
)
SECRET_PATTERNS = (
    re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    re.compile(r"\bghp_[A-Za-z0-9]{30,}\b"),
    re.compile(r"\bgithub_pat_[A-Za-z0-9_]{40,}\b"),
    re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
)
MERGE_MARKER = re.compile(r"^(?:<<<<<<< |>>>>>>> |\|\|\|\|\|\|\| )", re.MULTILINE)
SKIP_TEXT_SUFFIXES = {".stl", ".bin", ".dcf"}
REQUIRED_GOVERNANCE_FILES = (
    "README.md",
    "AGENTS.md",
    "collaboration-and-commit-standards.md",
    "domains/rt_control/README.md",
    "domains/rt_control/AGENTS.md",
    "domains/rt_control/PROGRESS.md",
    "domains/rt_control/BLOCKED-questions.md",
    ".pre-commit-config.yaml",
    ".github/pull_request_template.md",
    ".github/workflows/rt-control-ci.yml",
)
ROOT_DOMAIN_LEDGER_FILES = {"PROGRESS.md", "BLOCKED-questions.md"}


def check_path(relative_path: str) -> list[str]:
    """Reject generated outputs and retired package content."""
    path = PurePosixPath(relative_path)
    findings: list[str] = []
    if any(part in GENERATED_PARTS for part in path.parts) or path.suffix == ".pyc":
        findings.append(f"{relative_path}: generated artifact must not be tracked")
    if relative_path.startswith("src/rt_control/rt_watchdog/"):
        findings.append(f"{relative_path}: retired rt_watchdog must remain absent")
    if relative_path.startswith("docs/"):
        findings.append(f"{relative_path}: root docs directory must remain empty")
    if relative_path in ROOT_DOMAIN_LEDGER_FILES:
        findings.append(
            f"{relative_path}: domain progress and blocked questions must live under domains/<domain>/"
        )
    return findings


def check_text(relative_path: str, text: str) -> list[str]:
    """Check repository-owned text without rewriting it."""
    findings: list[str] = []
    if PurePosixPath(relative_path).suffix != ".patch":
        for line_number, line in enumerate(text.splitlines(), start=1):
            if line.endswith((" ", "\t")):
                findings.append(f"{relative_path}:{line_number}: trailing whitespace")
    # Patch whitespace belongs to the upstream diff and is validated by
    # applying the patch to its pinned source revision. Other hygiene and
    # secret checks still apply to patch payloads.
    if text and not text.endswith("\n"):
        findings.append(f"{relative_path}: missing newline at end of file")
    if MERGE_MARKER.search(text):
        findings.append(f"{relative_path}: merge-conflict marker found")
    if any(pattern.search(text) for pattern in SECRET_PATTERNS):
        findings.append(f"{relative_path}: possible secret or private key found")
    return findings


def check_dependency_pins(deps_text: str, versions_text: str) -> list[str]:
    """Require immutable upstream identities."""
    findings: list[str] = []
    try:
        deps = yaml.safe_load(deps_text) or {}
    except yaml.YAMLError as exc:
        return [f"deps.repos: invalid YAML: {exc}"]

    repositories = deps.get("repositories")
    if not isinstance(repositories, dict) or not repositories:
        findings.append("deps.repos: repositories must be a non-empty mapping")
    else:
        for name, spec in repositories.items():
            version = spec.get("version") if isinstance(spec, dict) else None
            if not isinstance(version, str) or not FULL_SHA.fullmatch(version):
                findings.append(
                    f"deps.repos:{name}: version must be a full 40-character commit SHA"
                )

    versions: dict[str, str] = {}
    for raw_line in versions_text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        versions[key.strip()] = value.strip()
    if not FULL_SHA.fullmatch(versions.get("IGH_COMMIT", "")):
        findings.append("versions.env: IGH_COMMIT must be a full 40-character commit SHA")
    return findings


def check_robot_interfaces_pin(deps_text: str, source_lock_text: str) -> list[str]:
    """Keep the vendored contract pin and release metadata identical."""
    try:
        deps = yaml.safe_load(deps_text) or {}
    except yaml.YAMLError as exc:
        return [f"deps.repos: invalid YAML: {exc}"]
    try:
        source_lock = yaml.safe_load(source_lock_text) or {}
    except yaml.YAMLError as exc:
        return [f"src/interfaces/source-lock.yaml: invalid YAML: {exc}"]

    findings: list[str] = []
    repositories = deps.get("repositories")
    dependency = (
        repositories.get(ROBOT_INTERFACES_PATH)
        if isinstance(repositories, dict)
        else None
    )
    if not isinstance(dependency, dict):
        findings.append(
            f"deps.repos must vendor robot_interfaces at {ROBOT_INTERFACES_PATH}"
        )
        dependency = {}

    if dependency.get("type") != "git":
        findings.append("deps.repos robot_interfaces dependency must use type git")
    if dependency.get("url") != ROBOT_INTERFACES_URL:
        findings.append(
            f"deps.repos robot_interfaces URL must be {ROBOT_INTERFACES_URL}"
        )

    dependency_commit = dependency.get("version")
    lock_commit = source_lock.get("commit")
    if not isinstance(lock_commit, str) or not FULL_SHA.fullmatch(lock_commit):
        findings.append("source-lock.yaml commit must be a full 40-character commit SHA")
    if dependency_commit != lock_commit:
        findings.append("source-lock.yaml commit must match deps.repos robot_interfaces version")
    if source_lock.get("schema_version") != 1:
        findings.append("source-lock.yaml schema_version must be 1")
    if source_lock.get("repository") != ROBOT_INTERFACES_URL:
        findings.append(f"source-lock.yaml repository must be {ROBOT_INTERFACES_URL}")
    if source_lock.get("vendor_path") != ROBOT_INTERFACES_PATH:
        findings.append(
            f"source-lock.yaml vendor_path must be {ROBOT_INTERFACES_PATH}"
        )
    if source_lock.get("vendored_packages") != ROBOT_INTERFACES_PACKAGES:
        findings.append(
            "source-lock.yaml vendored_packages must list exactly "
            + ", ".join(ROBOT_INTERFACES_PACKAGES)
        )
    contract_version = source_lock.get("contract_version")
    if not isinstance(contract_version, str) or not SEMVER.fullmatch(contract_version):
        findings.append("source-lock.yaml contract_version must be x.y.z")
    return findings


def _dependency_names(root: ET.Element) -> set[str]:
    names: set[str] = set()
    for element in root:
        tag = element.tag.rsplit("}", 1)[-1]
        if tag == "depend" or tag.endswith("_depend"):
            if element.text and element.text.strip():
                names.add(element.text.strip())
    return names


def check_manifest_dependencies(relative_path: str, xml_text: str) -> list[str]:
    """Enforce the shared -> feature -> bringup dependency direction."""
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        return [f"{relative_path}: invalid package XML: {exc}"]

    dependencies = _dependency_names(root)
    findings: list[str] = []
    if relative_path.startswith(SHARED_PACKAGE_PREFIXES):
        for dependency in sorted(dependencies & DOMAIN_PACKAGES):
            findings.append(
                f"{relative_path}: shared package must not depend on domain package {dependency}"
            )
    if (
        relative_path.startswith("src/rt_control/")
        and not relative_path.startswith("src/rt_control/rt_control_bringup/")
        and "rt_control_bringup" in dependencies
    ):
        findings.append(
            f"{relative_path}: feature package must not depend on rt_control_bringup"
        )
    return findings


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
            f"{relative_path}: robot_description contains domain-specific token "
            f"{match.group(0)!r}"
        ]
    return []


def check_interface_path(relative_path: str) -> list[str]:
    """Keep only definition-only RT-Control-private interfaces in-tree."""
    prefix = "src/interfaces/"
    if not relative_path.startswith(prefix):
        return []
    local_path = PurePosixPath(relative_path[len(prefix) :])
    if local_path.as_posix() in {"README.md", "source-lock.yaml"}:
        return []
    if not local_path.parts or local_path.parts[0] != "rt_control_interfaces":
        return [
            f"{relative_path}: public interface packages must come from "
            "src/vendor/robot_interfaces"
        ]
    forbidden_directories = {"src", "launch", "scripts", "config"}
    forbidden_suffixes = {".cpp", ".cc", ".c", ".hpp", ".hh", ".py", ".sh"}
    if (
        any(part in forbidden_directories for part in local_path.parts)
        or local_path.suffix in forbidden_suffixes
    ):
        return [
            f"{relative_path}: interface package must not contain business implementation"
        ]
    return []


def check_compose_policy(compose_text: str) -> list[str]:
    """Enforce the frozen rt-control container privilege boundary."""
    try:
        document = yaml.safe_load(compose_text) or {}
    except yaml.YAMLError as exc:
        return [f"docker/compose.yaml: invalid YAML: {exc}"]

    service = (document.get("services") or {}).get("rt-control")
    if not isinstance(service, dict):
        return ["docker/compose.yaml: services.rt-control must be defined"]

    findings: list[str] = []
    if service.get("privileged"):
        findings.append("docker/compose.yaml: privileged mode is forbidden")
    cpuset = service.get("cpuset")
    if not isinstance(cpuset, str) or "${RT_CONTROL_CPUSET:?" not in cpuset:
        findings.append(
            "docker/compose.yaml: RT_CONTROL_CPUSET must remain a required input"
        )
    allowed_devices = {"/dev/EtherCAT0:/dev/EtherCAT0"}
    devices = set(service.get("devices") or [])
    if devices != allowed_devices:
        findings.append("docker/compose.yaml: approved EtherCAT device mapping must remain exact")
    allowed_capabilities = {"SYS_NICE", "IPC_LOCK", "NET_RAW"}
    capabilities = set(service.get("cap_add") or [])
    if capabilities != allowed_capabilities:
        findings.append("docker/compose.yaml: approved capabilities must remain exact")
    allowed_volumes: set[str] = set()
    volumes = set(service.get("volumes") or [])
    if volumes != allowed_volumes:
        findings.append("docker/compose.yaml: no Docker volumes are approved")
    if "command" in service or "entrypoint" in service:
        findings.append(
            "docker/compose.yaml: command/entrypoint must not bypass the image signal gate"
        )
    return findings


def check_dockerfile_policy(dockerfile_text: str) -> list[str]:
    """Keep rt_control_start as the direct container command."""
    findings: list[str] = []
    entrypoint = 'ENTRYPOINT ["/rt-control-entrypoint.sh"]'
    command = 'CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]'
    active_lines = [
        line.strip()
        for line in dockerfile_text.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    entrypoints = [line for line in active_lines if line.upper().startswith("ENTRYPOINT ")]
    commands = [line for line in active_lines if line.upper().startswith("CMD ")]
    if not entrypoints or entrypoints[-1] != entrypoint:
        findings.append("docker/rt-control/Dockerfile: approved entrypoint is missing")
    if not commands or commands[-1] != command:
        findings.append(
            "docker/rt-control/Dockerfile: Docker CMD must execute the installed "
            "rt_control_start directly"
        )
    return findings


def _step_commands(job: object) -> str:
    if not isinstance(job, dict):
        return ""
    steps = job.get("steps")
    if not isinstance(steps, list):
        return ""
    return "\n".join(
        str(step.get("run", "")) for step in steps if isinstance(step, dict)
    )


def check_ci_workflow_policy(workflow_text: str) -> list[str]:
    """Prevent removal or reordering of the required CI gates."""
    try:
        workflow = yaml.load(workflow_text, Loader=yaml.BaseLoader) or {}
    except yaml.YAMLError as exc:
        return [f".github/workflows/rt-control-ci.yml: invalid YAML: {exc}"]

    findings: list[str] = []
    events = workflow.get("on") if isinstance(workflow, dict) else None
    if not isinstance(events, dict) or not {"pull_request", "push"}.issubset(events):
        findings.append("CI workflow must run on pull_request and main push")
    permissions = workflow.get("permissions") if isinstance(workflow, dict) else None
    if not isinstance(permissions, dict) or permissions.get("contents") != "read":
        findings.append("CI workflow must keep contents permission read-only")
    jobs = workflow.get("jobs") if isinstance(workflow, dict) else None
    if not isinstance(jobs, dict):
        return findings + ["CI workflow must define governance and build jobs"]

    governance = jobs.get("governance")
    build = jobs.get("build")
    if not isinstance(governance, dict) or not isinstance(build, dict):
        return findings + ["CI workflow must define governance and build jobs"]
    needs = build.get("needs")
    if needs != "governance" and not (
        isinstance(needs, list) and "governance" in needs
    ):
        findings.append("CI build job must depend on governance")

    governance_commands = _step_commands(governance)
    if "pre-commit run --all-files" not in governance_commands:
        findings.append("CI governance job must run pre-commit on all files")
    if "tools/pr_contract_gate.py" not in governance_commands:
        findings.append("CI governance job must enforce the pull request contract")

    build_commands = _step_commands(build)
    if "vcs import" not in build_commands:
        findings.append("CI build job must import deps.repos with vcs import")
    if "--packages-up-to" not in build_commands:
        findings.append("CI build job must limit colcon to the RT-Control package closure")
    for required_command, description in (
        ("colcon build", "colcon build"),
        ("colcon test", "colcon test"),
        ("check_urdf", "check_urdf"),
        ("tools/diff_legacy.py", "the frozen migration gate"),
    ):
        if required_command not in build_commands:
            findings.append(f"CI build job must run {description}")
    return findings


def check_precommit_policy(config_text: str) -> list[str]:
    """Require the local hook to execute the complete repository gate."""
    try:
        config = yaml.safe_load(config_text) or {}
    except yaml.YAMLError as exc:
        return [f".pre-commit-config.yaml: invalid YAML: {exc}"]

    hook: object = None
    repositories = config.get("repos") if isinstance(config, dict) else None
    if isinstance(repositories, list):
        for repository in repositories:
            if not isinstance(repository, dict) or repository.get("repo") != "local":
                continue
            hooks = repository.get("hooks")
            if not isinstance(hooks, list):
                continue
            hook = next(
                (
                    candidate
                    for candidate in hooks
                    if isinstance(candidate, dict)
                    and candidate.get("id") == "robot-quality-gate"
                ),
                None,
            )
            if hook is not None:
                break
    if not isinstance(hook, dict):
        return ["pre-commit must define the local robot-quality-gate hook"]

    findings: list[str] = []
    if hook.get("entry") != "tools/quality_gate.sh":
        findings.append("pre-commit quality gate must call tools/quality_gate.sh")
    if hook.get("language") != "system":
        findings.append("pre-commit quality gate must use the system toolchain")
    if hook.get("pass_filenames") is not False:
        findings.append("pre-commit pass_filenames must remain false")
    if hook.get("always_run") is not True:
        findings.append("pre-commit quality gate must always run")
    return findings


def _repository_files(repository_root: Path) -> list[str]:
    result = subprocess.run(
        [
            "git",
            "-C",
            str(repository_root),
            "ls-files",
            "--cached",
            "--others",
            "--exclude-standard",
            "-z",
        ],
        check=True,
        capture_output=True,
    )
    return sorted(path.decode("utf-8") for path in result.stdout.split(b"\0") if path)


def _read_text(path: Path) -> tuple[str | None, str | None]:
    data = path.read_bytes()
    if path.suffix.lower() in SKIP_TEXT_SUFFIXES or b"\0" in data:
        return None, None
    try:
        return data.decode("utf-8"), None
    except UnicodeDecodeError as exc:
        return None, f"{path}: repository text must be UTF-8: {exc}"


def _check_syntax(relative_path: str, text: str) -> list[str]:
    findings: list[str] = []
    path = PurePosixPath(relative_path)
    try:
        if path.suffix in {".yaml", ".yml", ".repos"}:
            yaml.safe_load(text)
        elif path.suffix == ".py" or relative_path.endswith(".launch.py"):
            compile(text, relative_path, "exec")
        elif path.suffix in {".xml", ".xacro", ".urdf", ".srdf"} or path.name == "package.xml":
            ET.fromstring(text)
    except (SyntaxError, ET.ParseError, yaml.YAMLError) as exc:
        findings.append(f"{relative_path}: syntax error: {exc}")
    return findings


def collect_findings(repository_root: Path) -> list[str]:
    """Run every repository-level check and return stable, sorted findings."""
    files = _repository_files(repository_root)
    file_set = set(files)
    findings: list[str] = []

    for required_path in REQUIRED_GOVERNANCE_FILES:
        if required_path not in file_set:
            findings.append(f"{required_path}: required governance file is missing")

    texts: dict[str, str] = {}
    for relative_path in files:
        findings.extend(check_path(relative_path))
        findings.extend(check_interface_path(relative_path))
        absolute_path = repository_root / relative_path
        if not absolute_path.is_file():
            continue
        text, decoding_error = _read_text(absolute_path)
        if decoding_error:
            findings.append(decoding_error.replace(str(repository_root) + "/", ""))
            continue
        if text is None:
            continue
        texts[relative_path] = text
        findings.extend(check_text(relative_path, text))
        findings.extend(_check_syntax(relative_path, text))
        if relative_path.endswith("/package.xml"):
            findings.extend(check_manifest_dependencies(relative_path, text))
        if (
            relative_path.startswith("src/description/robot_description/")
            and PurePosixPath(relative_path).suffix in {".xacro", ".urdf", ".srdf"}
        ):
            findings.extend(check_robot_description_xml(relative_path, text))

    deps_text = texts.get("deps.repos")
    versions_text = texts.get("versions.env")
    if deps_text is not None and versions_text is not None:
        findings.extend(check_dependency_pins(deps_text, versions_text))
    source_lock_text = texts.get("src/interfaces/source-lock.yaml")
    if deps_text is not None and source_lock_text is not None:
        findings.extend(check_robot_interfaces_pin(deps_text, source_lock_text))
    if "docker/compose.yaml" in texts:
        findings.extend(check_compose_policy(texts["docker/compose.yaml"]))
    if "docker/rt-control/Dockerfile" in texts:
        findings.extend(check_dockerfile_policy(texts["docker/rt-control/Dockerfile"]))
    if ".github/workflows/rt-control-ci.yml" in texts:
        findings.extend(
            check_ci_workflow_policy(texts[".github/workflows/rt-control-ci.yml"])
        )
    if ".pre-commit-config.yaml" in texts:
        findings.extend(check_precommit_policy(texts[".pre-commit-config.yaml"]))
    return sorted(set(findings))


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="repository root (defaults to the parent of tools/)",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    try:
        findings = collect_findings(args.repository_root.resolve())
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"repository gate could not run: {exc}", file=sys.stderr)
        return 2
    if findings:
        print("Repository gate failed:", file=sys.stderr)
        for finding in findings:
            print(f"- {finding}", file=sys.stderr)
        return 1
    print("PASS: repository architecture and file-hygiene gate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
