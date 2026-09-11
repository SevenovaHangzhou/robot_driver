import contextlib
import importlib.util
import io
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "tools/rt_control_native.sh"
HELPER = ROOT / "tools/rt_control_thread_affinity.py"


class NativeStartupRealtimeTest(unittest.TestCase):
    def run_start(self, scenario="success"):
        definitions = LAUNCHER.read_text().rsplit('\nparse_launcher_args "$@"', 1)[0]
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            source = workspace / "launcher-functions.sh"
            source.write_text(definitions)
            environment = os.environ.copy()
            environment.update(TEST_ROOT=temporary, TEST_CASE=scenario, RT_CONTROL_NATIVE_WS=temporary)
            script = r'''
source "$TEST_ROOT/launcher-functions.sh"
event() { printf '%s\n' "$1" >> "$TEST_ROOT/events"; }
for function in verify_target_identity verify_realtime_host verify_workspace \
    source_runtime_environment verify_runtime_dependency_closure verify_bus_services \
    reject_running_container cleanup_stale_pid_file verify_realtime_cpu_guard \
    verify_idle_ethercat prepare_can_interfaces verify_pcie_can_interface; do
    eval "$function() { :; }"
done
native_pid() {
    [[ "$TEST_CASE" != "exited" && -f "$TEST_ROOT/running" ]] || return 1
    [[ "$TEST_CASE" != "exec_pending" || -f "$TEST_ROOT/identity_ready" ]] || return 1
    printf '123\n'
}
launch_native() {
    event launch
    touch "$TEST_ROOT/running"
    mkdir -p "$runtime_root"
    printf '123\n' > "$pid_file"
}
pgrep() { [[ "$TEST_CASE" != "exited" && "$TEST_CASE" != "kernel_timeout" ]]; }
kill() { [[ "$TEST_CASE" != "exited" ]]; }
sleep() {
    if [[ "$TEST_CASE" == "exec_pending" ]]; then
        touch "$TEST_ROOT/identity_ready"
    else
        SECONDS=$((SECONDS + 151))
    fi
}
configure_and_verify_ethercat_op_thread() {
    event kernel
    [[ "$TEST_CASE" != "kernel_failure" ]] || return 1
    touch "$TEST_ROOT/kernel"
}
pin_controller_update_thread() {
    event pin
    [[ -f "$TEST_ROOT/kernel" && "$TEST_CASE" != "pin_failure" ]] || return 1
    touch "$TEST_ROOT/pinned"
}
wait_for_enable_service() {
    event services
    [[ -f "$TEST_ROOT/pinned" ]]
}
verify_controllers_before_enable() { event controllers; }
verify_operational_ethercat() { event operational; }
terminate_failed_start() { event stop; rm -f "$TEST_ROOT/running"; }
call_rt_service() { event "forbidden-$1"; return 1; }
start_native preauthorized
'''
            result = subprocess.run(
                ["bash", "-c", script], env=environment, capture_output=True,
                text=True, timeout=5, check=False,
            )
            events = (workspace / "events").read_text().splitlines()
            running = (workspace / "running").exists()
        return result, events, running

    def test_plain_start_prepares_realtime_before_readiness_without_enable(self):
        result, events, running = self.run_start()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(events, ["launch", "kernel", "pin", "services", "controllers", "operational"])
        self.assertTrue(running)
        self.assertIn("READY:", result.stdout)

    def test_failed_realtime_configuration_stops_without_readiness(self):
        for scenario, expected in [
            ("kernel_failure", ["launch", "kernel", "stop"]),
            ("pin_failure", ["launch", "kernel", "pin", "stop"]),
            ("exited", ["launch", "stop"]),
            ("kernel_timeout", ["launch", "stop"]),
        ]:
            with self.subTest(scenario=scenario):
                result, events, running = self.run_start(scenario)
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(events, expected)
                self.assertFalse(running)
                self.assertNotIn("READY:", result.stdout)

    def test_waits_for_the_launched_process_to_complete_exec(self):
        result, events, running = self.run_start("exec_pending")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(events, ["launch", "kernel", "pin", "services", "controllers", "operational"])
        self.assertTrue(running)


class AffinityStartupWaitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("startup_affinity_under_test", HELPER)
        cls.helper = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.helper
        spec.loader.exec_module(cls.helper)

    def invoke(self, inventories, clock):
        helper = self.helper
        argv = [str(HELPER), "--rt-cpu", "14", "--housekeeping-cpus", "0-3", "--deadline", "1"]

        def pin_when_ready(*_arguments):
            return helper.select_update_thread(123, 80), {}

        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(sys, "argv", argv))
            stack.enter_context(mock.patch.object(helper.time, "monotonic", side_effect=clock))
            sleeper = stack.enter_context(mock.patch.object(helper.time, "sleep"))
            stack.enter_context(mock.patch.object(helper, "find_ros2_control_pid", return_value=123))
            inventory = stack.enter_context(mock.patch.object(helper, "realtime_thread_inventory", side_effect=inventories))
            stack.enter_context(mock.patch.object(helper, "describe_inventory", return_value="test inventory"))
            stack.enter_context(mock.patch.object(helper, "pin_threads", side_effect=pin_when_ready))
            verifier = stack.enter_context(mock.patch.object(helper, "verify_affinity"))
            stack.enter_context(mock.patch.object(helper, "wait_for_target_processor"))
            stack.enter_context(mock.patch.object(helper, "thread_name", return_value="ros2_control_no"))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            result = helper.main()
        return result, inventory.call_count, sleeper.call_count, verifier.call_count

    def test_waits_until_the_update_thread_is_created(self):
        update = self.helper.ThreadSnapshot(321, "ros2_control_no", 1, 80, 1, {0, 1})
        result, inspections, sleeps, verifications = self.invoke([[], [update]], [0, 0, 0.5])
        self.assertEqual(result, 0)
        self.assertEqual((inspections, sleeps, verifications), (2, 1, 1))

    def test_startup_main_thread_is_not_the_control_update_thread(self):
        startup = self.helper.ThreadSnapshot(123, "ros2_control_no", 1, 80, 14, {14})
        update = self.helper.ThreadSnapshot(321, "ros2_control_no", 1, 80, 1, {0, 1})
        result, inspections, sleeps, verifications = self.invoke(
            [[startup], [update]], [0, 0, 0.5])
        self.assertEqual(result, 0)
        self.assertEqual((inspections, sleeps, verifications), (2, 1, 1))

    def test_waits_for_temporary_handoff_thread_to_exit_before_pinning(self):
        handoff = self.helper.ThreadSnapshot(322, "ecat-handoff", 1, 80, 14, {14})
        update = self.helper.ThreadSnapshot(321, "ros2_control_no", 1, 80, 1, {0, 1})
        result, inspections, sleeps, verifications = self.invoke(
            [[handoff, update], [update]], [0, 0, 0.5])
        self.assertEqual(result, 0)
        self.assertEqual((inspections, sleeps, verifications), (2, 1, 1))

    def test_missing_update_thread_times_out_without_pinning(self):
        result, inspections, sleeps, verifications = self.invoke([[]], [0, 0, 2])
        self.assertEqual(result, 1)
        self.assertEqual((inspections, sleeps, verifications), (1, 1, 0))

    def test_ambiguous_update_threads_are_not_retried_or_pinned(self):
        updates = [self.helper.ThreadSnapshot(tid, "ros2_control_no", 1, 80, 1, {0, 1})
                   for tid in [321, 322]]
        result, _inspections, sleeps, verifications = self.invoke([updates, updates], [0, 0])
        self.assertEqual(result, 1)
        self.assertEqual((sleeps, verifications), (0, 0))


class IgHRealtimeBuildContractTest(unittest.TestCase):
    def test_fifo_master_build_uses_high_resolution_blocking_waits(self):
        installer = (ROOT / "hostsetup/igh-install.sh").read_text()
        configure = installer.split("./configure \\\n", 1)[1].split("\nmake ", 1)[0]
        self.assertIn("--enable-hrtimer", configure)
        self.assertIn("IGH_HRTIMER=1", installer)

    def test_host_and_native_preflight_reject_the_old_master_build(self):
        for path in [ROOT / "hostsetup/verify-host.sh", LAUNCHER]:
            with self.subTest(path=path.name):
                self.assertIn("IGH_HRTIMER=1", path.read_text())

    def test_loaded_module_identity_includes_build_flags(self):
        launcher = LAUNCHER.read_text()
        self.assertIn("/sys/module/ec_master/notes/.note.gnu.build-id", launcher)
        self.assertIn("--dump-section .note.gnu.build-id=/dev/stdout", launcher)


if __name__ == "__main__":
    unittest.main()
