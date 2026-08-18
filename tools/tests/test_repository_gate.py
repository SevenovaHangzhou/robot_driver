#!/usr/bin/env python3

import contextlib
import io
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools import repository_gate


class RepositoryGateTest(unittest.TestCase):
    def assert_has(self, findings, fragment):
        self.assertTrue(
            any(fragment in finding for finding in findings),
            f"expected {fragment!r} in {findings!r}",
        )

    def test_generated_artifacts_and_retired_watchdog_are_rejected(self):
        for path in (
            "build/pkg/output",
            "install/pkg/setup.bash",
            "log/latest/event.log",
            "src/pkg/__pycache__/module.pyc",
        ):
            with self.subTest(path=path):
                self.assert_has(repository_gate.check_path(path), "generated artifact")

        self.assert_has(
            repository_gate.check_path("src/rt_control/rt_watchdog/package.xml"),
            "retired rt_watchdog",
        )
        for path in ("PROGRESS.md", "BLOCKED-questions.md"):
            with self.subTest(path=path):
                self.assert_has(
                    repository_gate.check_path(path),
                    "must live under domains/<domain>/",
                )
        self.assertEqual(repository_gate.check_path("src/rt_control/enable_manager/package.xml"), [])
        self.assert_has(
            repository_gate.check_path("docs/README.md"),
            "root docs directory must remain empty",
        )

    def test_text_hygiene_detects_conflicts_secrets_and_missing_newline(self):
        fake_token = "ghp_" + "a" * 32
        findings = repository_gate.check_text(
            "config/example.yaml",
            f"token: {fake_token}\n<<<<<<< HEAD\nvalue: bad ",
        )
        self.assert_has(findings, "possible secret")
        self.assert_has(findings, "merge-conflict marker")
        self.assert_has(findings, "trailing whitespace")
        self.assert_has(findings, "newline at end of file")

        self.assertEqual(
            repository_gate.check_text("patches/vendor/change.patch", "+context with space \n"),
            [],
        )
        patch_with_secret = "+token: " + fake_token + "\n"
        self.assert_has(
            repository_gate.check_text("patches/vendor/change.patch", patch_with_secret),
            "possible secret",
        )

    def test_dependency_sources_and_igh_commit_must_use_full_sha(self):
        valid_deps = """repositories:
  src/vendor/example:
    type: git
    url: https://example.invalid/example.git
    version: 0123456789abcdef0123456789abcdef01234567
"""
        valid_versions = """IGH_VERSION=stable-1.6
IGH_COMMIT=89abcdef0123456789abcdef0123456789abcdef
"""
        self.assertEqual(repository_gate.check_dependency_pins(valid_deps, valid_versions), [])

        findings = repository_gate.check_dependency_pins(
            valid_deps.replace("0123456789abcdef0123456789abcdef01234567", "main"),
            valid_versions.replace("89abcdef0123456789abcdef0123456789abcdef", "deadbeef"),
        )
        self.assert_has(findings, "must be a full 40-character commit SHA")
        self.assert_has(findings, "IGH_COMMIT must be a full 40-character commit SHA")

    def test_robot_interfaces_vendor_pin_matches_release_metadata(self):
        contract_sha = "0123456789abcdef0123456789abcdef01234567"
        deps = f"""repositories:
  src/vendor/robot_interfaces:
    type: git
    url: https://github.com/SevenovaHangzhou/robot_interfaces.git
    version: {contract_sha}
"""
        source_lock = f"""schema_version: 1
repository: https://github.com/SevenovaHangzhou/robot_interfaces.git
commit: {contract_sha}
contract_version: 0.6.1
vendor_path: src/vendor/robot_interfaces
vendored_packages:
  - robot_rt_control_interfaces
  - robot_system_interfaces
  - robot_interfaces_qos
"""
        self.assertEqual(
            repository_gate.check_robot_interfaces_pin(deps, source_lock), []
        )

        mismatched = source_lock.replace(contract_sha, "a" * 40, 1)
        self.assert_has(
            repository_gate.check_robot_interfaces_pin(deps, mismatched),
            "must match deps.repos",
        )
        mirrored = source_lock.replace("vendored_packages", "mirrored_packages")
        self.assert_has(
            repository_gate.check_robot_interfaces_pin(deps, mirrored),
            "vendored_packages must list exactly",
        )
        without_qos = source_lock.replace("  - robot_interfaces_qos\n", "")
        self.assert_has(
            repository_gate.check_robot_interfaces_pin(deps, without_qos),
            "vendored_packages must list exactly",
        )

    def test_shared_packages_cannot_depend_on_domain_implementation(self):
        manifest = """<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>0.1.0</version>
  <exec_depend>rt_control_bringup</exec_depend>
</package>
"""
        self.assert_has(
            repository_gate.check_manifest_dependencies(
                "src/description/robot_description/package.xml", manifest
            ),
            "shared package must not depend on domain package",
        )

        feature_manifest = manifest.replace("robot_description", "enable_manager").replace(
            "src/description/robot_description", "src/rt_control/enable_manager"
        )
        self.assert_has(
            repository_gate.check_manifest_dependencies(
                "src/rt_control/enable_manager/package.xml", feature_manifest
            ),
            "feature package must not depend on rt_control_bringup",
        )

    def test_robot_description_ignores_comments_but_rejects_domain_data(self):
        comment_only = """<?xml version="1.0"?>
<robot name="robot">
  <!-- ros2_control belongs to rt_control_bringup. -->
  <link name="base_link"/>
</robot>
"""
        self.assertEqual(
            repository_gate.check_robot_description_xml("robot.urdf.xacro", comment_only), []
        )

        polluted = comment_only.replace(
            '<link name="base_link"/>', '<link name="pick_station"/>'
        )
        self.assert_has(
            repository_gate.check_robot_description_xml("robot.urdf.xacro", polluted),
            "domain-specific token",
        )

    def test_interface_package_rejects_business_implementation(self):
        self.assert_has(
            repository_gate.check_interface_path(
                "src/interfaces/rt_control_interfaces/src/task_planner.cpp"
            ),
            "interface package must not contain business implementation",
        )
        self.assertEqual(
            repository_gate.check_interface_path(
                "src/interfaces/rt_control_interfaces/srv/RtEnable.srv"
            ),
            [],
        )
        self.assert_has(
            repository_gate.check_interface_path(
                "src/interfaces/robot_rt_control_interfaces/srv/SetControlEnabled.srv"
            ),
            "public interface packages must come from src/vendor/robot_interfaces",
        )
        self.assert_has(
            repository_gate.check_interface_path(
                "src/interfaces/robot_rt_control_interfaces/scripts/adapter.py"
            ),
            "public interface packages must come from src/vendor/robot_interfaces",
        )
        self.assert_has(
            repository_gate.check_interface_path(
                "src/interfaces/rt_control_interfaces/task_planner.py"
            ),
            "interface package must not contain business implementation",
        )
        self.assertEqual(
            repository_gate.check_interface_path(
                "src/vendor/robot_interfaces/qos/robot_interfaces_qos/__init__.py"
            ),
            [],
        )

    def test_compose_policy_rejects_privilege_and_requires_cpu_gate(self):
        valid = """services:
  rt-control:
    cpuset: "${RT_CONTROL_CPUSET:?required}"
    devices: ["/dev/EtherCAT0:/dev/EtherCAT0"]
    cap_add: [SYS_NICE, IPC_LOCK, NET_RAW]
"""
        self.assertEqual(repository_gate.check_compose_policy(valid), [])

        unsafe = valid.replace(
            '    cpuset: "${RT_CONTROL_CPUSET:?required}"\n',
            "    cpuset: \"0\"\n    privileged: true\n",
        )
        findings = repository_gate.check_compose_policy(unsafe)
        self.assert_has(findings, "privileged mode is forbidden")
        self.assert_has(findings, "RT_CONTROL_CPUSET must remain a required input")

        incomplete = valid.replace(
            '    devices: ["/dev/EtherCAT0:/dev/EtherCAT0"]\n', "    devices: []\n"
        ).replace("    cap_add: [SYS_NICE, IPC_LOCK, NET_RAW]\n", "    cap_add: [SYS_NICE]\n")
        findings = repository_gate.check_compose_policy(incomplete)
        self.assert_has(findings, "device mapping must remain exact")
        self.assert_has(findings, "capabilities must remain exact")

        with_volume = valid + '    volumes: ["./docker/cyclonedds.xml:/etc/cyclonedds.xml:ro"]\n'
        self.assert_has(
            repository_gate.check_compose_policy(with_volume),
            "no Docker volumes are approved",
        )

    def test_dockerfile_must_keep_the_signal_gate_as_direct_command(self):
        valid = """FROM ros:humble-ros-base
ENTRYPOINT ["/rt-control-entrypoint.sh"]
CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]
"""
        self.assertEqual(repository_gate.check_dockerfile_policy(valid), [])

        bypassed = valid.replace(
            'CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]',
            'CMD ["ros2", "launch", "rt_control_bringup", "rt_control.launch.py"]',
        )
        self.assert_has(
            repository_gate.check_dockerfile_policy(bypassed),
            "Docker CMD must execute the installed rt_control_start directly",
        )

        commented_bypass = (
            valid.replace(
                'CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]',
                '# CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]',
            )
            + 'CMD ["ros2", "launch", "rt_control_bringup", "rt_control.launch.py"]\n'
        )
        self.assert_has(
            repository_gate.check_dockerfile_policy(commented_bypass),
            "Docker CMD must execute the installed rt_control_start directly",
        )

    def test_ci_workflow_must_preserve_governance_before_build(self):
        valid = """on:
  pull_request:
  push:
permissions:
  contents: read
jobs:
  governance:
    steps:
      - run: pre-commit run --all-files
      - run: python3 tools/pr_contract_gate.py --event-path event.json
  build:
    needs: governance
    steps:
      - run: vcs import . < deps.repos
      - run: RT_CONTROL_NATIVE_WS=/tmp/rt-control-ci-ws tools/bootstrap_native_dev.sh prepare
      - run: ./configure --prefix=/usr/local/etherlab --disable-kernel
      - run: rosdep install --dependency-types test
      - run: colcon build --packages-up-to rt_control_bringup
      - run: colcon test --packages-up-to rt_control_bringup
      - run: check_urdf robot.urdf
      - run: python3 tools/diff_legacy.py
  container:
    needs: governance
    steps:
      - run: |
          source versions.env
          docker build --file docker/rt-control/Dockerfile \
            --build-arg IGH_VERSION="${IGH_VERSION}" \
            --build-arg IGH_COMMIT="${IGH_COMMIT}" \
            --tag rt-control:ci .
      - run: |
          strip_ansi() { sed -E $'s/\\033\\[[0-9;]*[mK]//g'; }
          docker run --detach --name rt-control-ci-mock --network none \
            rt-control:ci \
            /opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start \
            use_mock_hardware:=true
          controller_state="$(timeout 8 docker exec rt-control-ci-mock \
            bash -lc 'ros2 control list_controllers' | strip_ansi)"
          grep -Eq '^joint_state_broadcaster[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^rt_internal_state_broadcaster[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^diff_drive_controller[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^enable_manager[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^whole_body_jtc[[:space:]].*inactive[[:space:]]*$' <<<"${controller_state}"
          docker stop --time 100 rt-control-ci-mock
          container_logs="$(docker logs rt-control-ci-mock 2>&1)"
          printf '%s\\n' "${container_logs}"
          exit_code="$(docker inspect --format '{{.State.ExitCode}}' rt-control-ci-mock)"
          test "${exit_code}" -eq 0
          grep -F 'rt_control shutdown disable result: ok=true' <<<"${container_logs}"
          if grep -Eq 'UNCLEAN_SHUTDOWN|\\[ERROR\\]|\\[FATAL\\]' <<<"${container_logs}"; then
            exit 1
          fi
      - if: always()
        run: docker rm --force rt-control-ci-mock || true
"""
        self.assertEqual(repository_gate.check_ci_workflow_policy(valid), [])

        weakened = valid.replace("    needs: governance\n", "").replace(
            "      - run: colcon test --packages-up-to rt_control_bringup\n", ""
        )
        findings = repository_gate.check_ci_workflow_policy(weakened)
        self.assert_has(findings, "build job must depend on governance")
        self.assert_has(findings, "build job must run colcon test")

        unprepared = valid.replace(
            "      - run: RT_CONTROL_NATIVE_WS=/tmp/rt-control-ci-ws "
            "tools/bootstrap_native_dev.sh prepare\n",
            "",
        ).replace(
            "      - run: ./configure --prefix=/usr/local/etherlab --disable-kernel\n",
            "",
        )
        findings = repository_gate.check_ci_workflow_policy(unprepared)
        self.assert_has(findings, "apply and verify frozen vendor patches")
        self.assert_has(findings, "build the pinned IgH userspace dependency")

        no_test_dependencies = valid.replace(
            "      - run: rosdep install --dependency-types test\n", ""
        )
        self.assert_has(
            repository_gate.check_ci_workflow_policy(no_test_dependencies),
            "install ROS test dependencies",
        )

        for weakened, expected in (
            (
                valid.replace("  container:\n", "  image_check:\n"),
                "define a container job",
            ),
            (
                valid.replace(
                    "  container:\n    needs: governance\n",
                    "  container:\n",
                ),
                "container job must depend on governance",
            ),
            (
                valid.replace("docker build", "docker image inspect"),
                "build the production Dockerfile",
            ),
            (
                valid.replace("--build-arg IGH_VERSION", "--label IGH_VERSION"),
                "use the pinned IgH build arguments",
            ),
            (
                valid.replace("--build-arg IGH_COMMIT", "--label IGH_COMMIT"),
                "use the pinned IgH build arguments",
            ),
            (
                valid.replace("--network none", "--network host"),
                "isolate the Mock container network",
            ),
            (
                valid.replace(
                    "--network none", "--network none --device /dev/EtherCAT0"
                ),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none --cap-add SYS_NICE"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network=host"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none --ipc=host"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none --pid host"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none --mount type=bind"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none --pid=host"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none --volume /:/host"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("--network none", "--network none -v /:/host"),
                "must not grant hardware or elevated privileges",
            ),
            (
                valid.replace("use_mock_hardware:=true", "use_mock_hardware:=false"),
                "start the built image with Mock hardware",
            ),
            (
                valid.replace(
                    "/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start",
                    "ros2 launch rt_control_bringup rt_control.launch.py",
                ),
                "exercise the installed signal-gated entrypoint",
            ),
            (
                valid.replace("docker stop --time 100", "docker kill"),
                "exercise orderly container shutdown",
            ),
            (
                valid.replace("docker rm --force", "docker inspect"),
                "clean up the Mock container",
            ),
            (
                valid.replace(
                    'container_logs="$(docker logs rt-control-ci-mock 2>&1)"\n',
                    "",
                ),
                "capture Mock container logs",
            ),
            (
                valid.replace(" | strip_ansi", ""),
                "normalize controller CLI output",
            ),
            (
                valid.replace(
                    "strip_ansi() { sed -E $'s/\\033\\[[0-9;]*[mK]//g'; }",
                    "strip_ansi() { cat; }",
                ),
                "normalize controller CLI output",
            ),
            (
                valid.replace("timeout 8 docker exec", "docker exec"),
                "bound each controller readiness query",
            ),
            (
                valid.replace("active[[:space:]]*$", "active$"),
                "allow trailing whitespace in controller states",
            ),
            (
                valid.replace(
                    "rt_control shutdown disable result: ok=true",
                    "/rt/disable ok=true",
                ),
                "verify the real shutdown success marker",
            ),
            (
                valid.replace('test "${exit_code}" -eq 0', "true"),
                "assert successful Mock container exit",
            ),
            (
                valid.replace(
                    "UNCLEAN_SHUTDOWN|\\[ERROR\\]|\\[FATAL\\]",
                    "UNCLEAN_SHUTDOWN",
                ),
                "reject unclean or error-level Mock logs",
            ),
            (
                valid.replace(
                    'container_logs="$(docker logs rt-control-ci-mock 2>&1)"\n'
                    '          printf \'%s\\n\' "${container_logs}"\n'
                    '          exit_code="$(docker inspect',
                    'exit_code="$(docker inspect',
                ),
                "capture Mock logs before asserting exit status",
            ),
        ):
            with self.subTest(expected=expected):
                self.assert_has(
                    repository_gate.check_ci_workflow_policy(weakened), expected
                )

    def test_precommit_must_call_the_full_repository_gate(self):
        valid = """repos:
  - repo: local
    hooks:
      - id: robot-quality-gate
        entry: tools/quality_gate.sh
        language: system
        pass_filenames: false
        always_run: true
"""
        self.assertEqual(repository_gate.check_precommit_policy(valid), [])
        weakened = valid.replace("pass_filenames: false", "pass_filenames: true")
        self.assert_has(
            repository_gate.check_precommit_policy(weakened),
            "pass_filenames must remain false",
        )

    def test_quality_gate_must_run_driver_variant_projection_gate(self):
        valid = """#!/usr/bin/env bash
python3 tools/repository_gate.py
python3 -m tools.driver_variant_source_projections --repository-root "${repository_root}"
"""
        self.assertEqual(repository_gate.check_quality_gate_policy(valid), [])

        missing = valid.replace(
            'python3 -m tools.driver_variant_source_projections --repository-root "${repository_root}"\n',
            "",
        )
        self.assert_has(
            repository_gate.check_quality_gate_policy(missing),
            "must run the driver-variant projection gate",
        )

        duplicate_projection = valid + valid.splitlines()[2] + "\n"
        self.assert_has(
            repository_gate.check_quality_gate_policy(duplicate_projection),
            "driver-variant projection gate exactly once",
        )

        before_repository_gate = "\n".join(reversed(valid.splitlines())) + "\n"
        self.assert_has(
            repository_gate.check_quality_gate_policy(before_repository_gate),
            "must run after repository_gate",
        )

        for weakened in (
            valid.replace("python3 tools/repository_gate.py\n", ""),
            valid.replace(
                "python3 tools/repository_gate.py",
                "# python3 tools/repository_gate.py",
            ),
            valid.replace(
                "python3 tools/repository_gate.py\n",
                "python3 tools/repository_gate.py\n"
                "python3 tools/repository_gate.py\n",
            ),
        ):
            with self.subTest(weakened=weakened):
                self.assert_has(
                    repository_gate.check_quality_gate_policy(weakened),
                    "must run repository_gate exactly once",
                )

    def make_valid_repository(self, root: Path):
        files = {
            "README.md": "# robot_driver\n",
            "AGENTS.md": "# Contract\n",
            "collaboration-and-commit-standards.md": "# Collaboration\n",
            "domains/rt_control/README.md": "# rt-control\n",
            "domains/rt_control/AGENTS.md": "# rt-control contract\n",
            "domains/rt_control/PROGRESS.md": "# rt-control progress\n",
            "domains/rt_control/BLOCKED-questions.md": "# rt-control blockers\n",
            ".pre-commit-config.yaml": """repos:
  - repo: local
    hooks:
      - id: robot-quality-gate
        entry: tools/quality_gate.sh
        language: system
        pass_filenames: false
        always_run: true
""",
            ".github/pull_request_template.md": "# Review\n",
            "tools/quality_gate.sh": """#!/usr/bin/env bash
python3 tools/repository_gate.py
python3 -m tools.driver_variant_source_projections --repository-root "${repository_root}"
""",
            ".github/workflows/rt-control-ci.yml": """on:
  pull_request:
  push:
permissions:
  contents: read
jobs:
  governance:
    steps:
      - run: pre-commit run --all-files
      - run: python3 tools/pr_contract_gate.py --event-path event.json
  build:
    needs: governance
    steps:
      - run: vcs import . < deps.repos
      - run: RT_CONTROL_NATIVE_WS=/tmp/rt-control-ci-ws tools/bootstrap_native_dev.sh prepare
      - run: ./configure --prefix=/usr/local/etherlab --disable-kernel
      - run: rosdep install --dependency-types test
      - run: colcon build --packages-up-to rt_control_bringup
      - run: colcon test --packages-up-to rt_control_bringup
      - run: check_urdf robot.urdf
      - run: python3 tools/diff_legacy.py
  container:
    needs: governance
    steps:
      - run: |
          source versions.env
          docker build --file docker/rt-control/Dockerfile \
            --build-arg IGH_VERSION="${IGH_VERSION}" \
            --build-arg IGH_COMMIT="${IGH_COMMIT}" \
            --tag rt-control:ci .
      - run: |
          strip_ansi() { sed -E $'s/\\033\\[[0-9;]*[mK]//g'; }
          docker run --detach --name rt-control-ci-mock --network none \
            rt-control:ci \
            /opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start \
            use_mock_hardware:=true
          controller_state="$(timeout 8 docker exec rt-control-ci-mock \
            bash -lc 'ros2 control list_controllers' | strip_ansi)"
          grep -Eq '^joint_state_broadcaster[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^rt_internal_state_broadcaster[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^diff_drive_controller[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^enable_manager[[:space:]].*active[[:space:]]*$' <<<"${controller_state}"
          grep -Eq '^whole_body_jtc[[:space:]].*inactive[[:space:]]*$' <<<"${controller_state}"
          docker stop --time 100 rt-control-ci-mock
          container_logs="$(docker logs rt-control-ci-mock 2>&1)"
          printf '%s\\n' "${container_logs}"
          exit_code="$(docker inspect --format '{{.State.ExitCode}}' rt-control-ci-mock)"
          test "${exit_code}" -eq 0
          grep -F 'rt_control shutdown disable result: ok=true' <<<"${container_logs}"
          if grep -Eq 'UNCLEAN_SHUTDOWN|\\[ERROR\\]|\\[FATAL\\]' <<<"${container_logs}"; then
            exit 1
          fi
      - if: always()
        run: docker rm --force rt-control-ci-mock || true
""",
            "deps.repos": """repositories:
  src/vendor/robot_interfaces:
    type: git
    url: https://github.com/SevenovaHangzhou/robot_interfaces.git
    version: 0123456789abcdef0123456789abcdef01234567
""",
            "versions.env": """IGH_VERSION=stable-1.6
IGH_COMMIT=89abcdef0123456789abcdef0123456789abcdef
""",
            "docker/compose.yaml": """services:
  rt-control:
    cpuset: "${RT_CONTROL_CPUSET:?required}"
    devices: ["/dev/EtherCAT0:/dev/EtherCAT0"]
    cap_add: [SYS_NICE, IPC_LOCK, NET_RAW]
""",
            "docker/rt-control/Dockerfile": """FROM ros:humble-ros-base
ENTRYPOINT ["/rt-control-entrypoint.sh"]
CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]
""",
            "src/description/robot_description/package.xml": """<package format="3">
  <name>robot_description</name>
  <version>0.1.0</version>
  <exec_depend>xacro</exec_depend>
</package>
""",
            "src/description/robot_description/urdf/robot.urdf.xacro": """<robot name="robot">
  <!-- ros2_control remains outside this package. -->
  <link name="base_link"/>
</robot>
""",
            "src/interfaces/rt_control_interfaces/msg/Example.msg": "string value\n",
            "src/interfaces/source-lock.yaml": """schema_version: 1
repository: https://github.com/SevenovaHangzhou/robot_interfaces.git
commit: 0123456789abcdef0123456789abcdef01234567
contract_version: 0.6.1
vendor_path: src/vendor/robot_interfaces
vendored_packages:
  - robot_rt_control_interfaces
  - robot_system_interfaces
  - robot_interfaces_qos
""",
            "tools/example.py": "print('ok')\n",
        }
        for relative_path, content in files.items():
            path = root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")
        subprocess.run(["git", "init", "-q", str(root)], check=True)

    def test_collect_findings_accepts_a_minimal_compliant_repository(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.make_valid_repository(root)
            self.assertEqual(repository_gate.collect_findings(root), [])

    def test_collect_findings_rejects_a_bypassed_projection_gate(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.make_valid_repository(root)
            (root / "tools/quality_gate.sh").write_text(
                "#!/usr/bin/env bash\npython3 tools/repository_gate.py\n",
                encoding="utf-8",
            )
            self.assert_has(
                repository_gate.collect_findings(root),
                "must run the driver-variant projection gate",
            )

    def test_collect_findings_reports_missing_governance_and_invalid_syntax(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            subprocess.run(["git", "init", "-q", str(root)], check=True)
            invalid_python = root / "tools/broken.py"
            invalid_python.parent.mkdir(parents=True)
            invalid_python.write_text("if True print('broken')\n", encoding="utf-8")
            findings = repository_gate.collect_findings(root)
            self.assert_has(findings, "required governance file is missing")
            self.assert_has(findings, "syntax error")

    def test_main_reports_success_policy_failure_and_execution_failure(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self.make_valid_repository(root)
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                self.assertEqual(
                    repository_gate.main(["--repository-root", str(root)]), 0
                )
            self.assertIn("PASS", stdout.getvalue())

            (root / "build/generated.txt").parent.mkdir()
            (root / "build/generated.txt").write_text("generated\n", encoding="utf-8")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(
                    repository_gate.main(["--repository-root", str(root)]), 1
                )
            self.assertIn("generated artifact", stderr.getvalue())

        with tempfile.TemporaryDirectory() as non_repository:
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(
                    repository_gate.main(["--repository-root", non_repository]), 2
                )
            self.assertIn("could not run", stderr.getvalue())

    def test_format_parsers_report_malformed_input(self):
        self.assert_has(
            repository_gate.check_dependency_pins("repositories: [", "IGH_COMMIT=bad\n"),
            "invalid YAML",
        )
        self.assert_has(
            repository_gate.check_manifest_dependencies("package.xml", "<package>"),
            "invalid package XML",
        )
        self.assert_has(
            repository_gate.check_robot_description_xml("robot.urdf.xacro", "<robot>"),
            "invalid robot-description XML",
        )
        self.assert_has(repository_gate.check_compose_policy("services: ["), "invalid YAML")


if __name__ == "__main__":
    unittest.main()
