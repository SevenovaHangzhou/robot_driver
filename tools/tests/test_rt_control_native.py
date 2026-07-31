#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "tools" / "rt_control_native.sh"
BOOTSTRAP = ROOT / "tools" / "bootstrap_native_dev.sh"
IGH_INSTALLER = ROOT / "hostsetup" / "igh-install.sh"
SIGNAL_GATE = ROOT / "src/rt_control/rt_control_bringup/scripts/rt_control_start"
ONECLICK = ROOT / "tools" / "rt_control_native_oneclick.sh"


class NativeLauncherContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = LAUNCHER.read_text(encoding="utf-8")

    def test_runtime_is_fixed_to_shared_domain_and_default_fastdds_transports(self):
        self.assertIn('readonly expected_ros_domain_id="0"', self.text)
        self.assertIn('ROS_DOMAIN_ID="${expected_ros_domain_id}"', self.text)
        self.assertIn('RMW_IMPLEMENTATION="rmw_fastrtps_cpp"', self.text)
        self.assertIn('-u FASTRTPS_DEFAULT_PROFILES_FILE', self.text)
        self.assertIn('-u FASTDDS_DEFAULT_PROFILES_FILE', self.text)
        self.assertIn('-u CYCLONEDDS_URI', self.text)
        self.assertNotIn("fastdds_udp_only.xml", self.text)

    def test_native_runtime_cannot_overlap_the_container_runtime(self):
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("reject_running_container"), body.index("launch_native"))

    def test_plain_start_does_not_enable_or_reset_faults(self):
        start = self.text.index("start_native()")
        stop = self.text.index("enable_native()", start)
        body = self.text[start:stop]
        self.assertNotIn("call_rt_service enable", body)
        self.assertNotIn("call_rt_service reset_fault", body)

    def test_start_and_enable_requires_confirmation_and_never_resets_faults(self):
        start = self.text.index("start_and_enable_native()")
        stop = self.text.index("recover_power_loss_native()", start)
        body = self.text[start:stop]
        self.assertLess(
            body.index("confirm_enable_authorization"),
            body.index("start_native"),
        )
        self.assertLess(body.index("start_native"), body.index("call_rt_service enable"))
        self.assertNotIn("call_rt_service reset_fault", body)

    def test_reset_fault_is_only_used_by_the_explicit_recovery_entrypoint(self):
        self.assertIn("enable|disable|reset_fault", self.text)
        self.assertIn("recover-power-loss) recover_power_loss_native", self.text)

        start = self.text.index("recover_power_loss_native()")
        stop = self.text.index("stop_native()", start)
        body = self.text[start:stop]
        self.assertEqual(body.count("call_rt_service reset_fault"), 1)
        self.assertLess(body.index("confirm_recovery_authorization"), body.index("start_native preauthorized"))
        self.assertLess(
            body.index("wait_for_enable_manager_reset_ready"),
            body.index("call_rt_service reset_fault"),
        )
        self.assertLess(body.index("call_rt_service reset_fault"), body.index("check_axis_states disabled"))
        self.assertLess(body.index("check_axis_states disabled"), body.index("call_rt_service enable"))
        self.assertLess(body.index("call_rt_service enable"), body.index("check_axis_states enabled"))

    def test_stop_requests_disable_before_signalling_runtime(self):
        start = self.text.index("stop_native()")
        stop = self.text.index("status_native()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("call_rt_service disable"), body.index("kill -TERM"))

    def test_verified_realtime_cpu_is_applied_to_the_runtime(self):
        self.assertIn('readonly expected_cpuset="14"', self.text)
        self.assertIn('taskset --cpu-list "${expected_cpuset}"', self.text)

    def test_runtime_sources_ros_setup_without_nounset(self):
        start = self.text.index("source_runtime_environment()")
        stop = self.text.index("runtime_env()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("set +u"), body.index("source /opt/ros/humble/setup.bash"))
        self.assertLess(body.index('source "${install_root}/setup.bash"'), body.index("set -u"))

    def test_symlink_install_signal_gate_source_is_executable(self):
        self.assertTrue(
            SIGNAL_GATE.stat().st_mode & 0o111,
            "rt_control_start must be executable because native symlink-install points to it",
        )


class NativeBootstrapContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = BOOTSTRAP.read_text(encoding="utf-8")

    def test_bootstrap_uses_frozen_dependency_manifest_and_refuses_partial_vendor_tree(self):
        self.assertIn('dependency_manifest="${repository_root}/deps.repos"', self.text)
        self.assertIn("refuse_partial_vendor_tree", self.text)
        self.assertIn("verify_vendor_heads", self.text)
        self.assertNotIn("git reset", self.text)
        self.assertNotIn("git checkout", self.text)

    def test_patches_are_idempotent_and_never_rewrite_vendor_history(self):
        self.assertIn('apply --reverse --check "${patch_file}"', self.text)
        self.assertIn('apply --check "${patch_file}"', self.text)
        self.assertIn("verify_frozen_vendor_trees", self.text)
        self.assertIn('GIT_INDEX_FILE="${actual_index}"', self.text)
        self.assertNotIn("git am", self.text)

        start = self.text.index("prepare_sources()")
        stop = self.text.index("source_build_environment()", start)
        body = self.text[start:stop]
        self.assertIn("if (verify_frozen_vendor_trees)", body)
        self.assertLess(
            body.index("if (verify_frozen_vendor_trees)"),
            body.index("apply_frozen_patches"),
        )

    def test_build_is_incremental_and_kept_outside_the_repository(self):
        self.assertIn("--symlink-install", self.text)
        self.assertIn('build_base="${workspace_root}/build"', self.text)
        self.assertIn('install_base="${workspace_root}/install"', self.text)
        self.assertIn('log_base="${workspace_root}/log"', self.text)

    def test_bootstrap_sources_ros_setup_without_nounset(self):
        start = self.text.index("source_build_environment()")
        stop = self.text.index("dependency_paths()", start)
        body = self.text[start:stop]
        self.assertLess(body.index("set +u"), body.index("source /opt/ros/humble/setup.bash"))
        self.assertLess(body.index("source /opt/ros/humble/setup.bash"), body.index("set -u"))


class NativeHostSetupContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.installer_text = IGH_INSTALLER.read_text(encoding="utf-8")
        cls.bootstrap_text = BOOTSTRAP.read_text(encoding="utf-8")
        cls.launcher_text = LAUNCHER.read_text(encoding="utf-8")

    def test_host_igh_install_writes_the_dependency_identity_used_by_native_checks(self):
        expected_path = "/usr/local/share/rt-control/dependency-versions.env"
        self.assertIn(expected_path, self.installer_text)
        self.assertIn(expected_path, self.bootstrap_text)
        self.assertIn(expected_path, self.launcher_text)
        self.assertIn("IGH_VERSION=%s\\nIGH_COMMIT=%s\\n", self.installer_text)
        self.assertIn('install -d -m 0755 /usr/local/share/rt-control', self.installer_text)


class NativeOneClickRecoveryContractTest(unittest.TestCase):
    def test_oneclick_script_delegates_to_the_explicit_recovery_entrypoint(self):
        self.assertTrue(ONECLICK.exists(), "native one-click recovery script is missing")
        self.assertTrue(
            ONECLICK.stat().st_mode & 0o111,
            "native one-click recovery script must be executable",
        )
        text = ONECLICK.read_text(encoding="utf-8")
        self.assertIn('exec "${script_dir}/rt_control_native.sh" recover-power-loss "$@"', text)


if __name__ == "__main__":
    unittest.main()
