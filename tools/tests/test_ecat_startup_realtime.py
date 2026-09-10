import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


class StartupSchedulingTest(unittest.TestCase):
    def test_scoped_scheduling_restores_success_and_failure_paths(self):
        vendor = os.environ.get("ECAT_STARTUP_RT_SOURCE")
        if not vendor:
            self.skipTest("set ECAT_STARTUP_RT_SOURCE to the patched vendor tree")
        header = Path(vendor) / "ethercat_driver/include/ethercat_driver/startup_realtime.hpp"
        self.assertTrue(header.is_file())
        program = r'''
#include <sched.h>
#include <cerrno>
#include <cassert>
#include <cstdlib>
#include <stdexcept>
int policy = SCHED_OTHER, priority = 0, fail_fifo = 0;
cpu_set_t affinity;
int fake_getscheduler(pid_t) { return policy; }
int fake_getparam(pid_t, sched_param *p) { p->sched_priority = priority; return 0; }
int fake_getaffinity(pid_t, size_t, cpu_set_t *p) { *p = affinity; return 0; }
int fake_setscheduler(pid_t, int p, const sched_param *s) {
  if (p == SCHED_FIFO && fail_fifo) { errno = EPERM; return -1; }
  policy = p; priority = s->sched_priority; return 0;
}
int fake_setaffinity(pid_t, size_t, const cpu_set_t *p) { affinity = *p; return 0; }
#define sched_getscheduler fake_getscheduler
#define sched_getparam fake_getparam
#define sched_getaffinity fake_getaffinity
#define sched_setscheduler fake_setscheduler
#define sched_setaffinity fake_setaffinity
#include "ethercat_driver/startup_realtime.hpp"
void original() {
  assert(policy == SCHED_OTHER && priority == 0);
  assert(CPU_COUNT(&affinity) == 1 && CPU_ISSET(2, &affinity));
}
int main() {
  CPU_ZERO(&affinity); CPU_SET(2, &affinity);
  unsetenv("RT_CONTROL_ECAT_STARTUP_CPU");
  unsetenv("RT_CONTROL_ECAT_STARTUP_PRIORITY");
  { ethercat_driver::StartupRealtime guard; assert(!guard.enabled()); }
  original();
  setenv("RT_CONTROL_ECAT_STARTUP_CPU", "14", 1);
  bool threw = false;
  try { ethercat_driver::StartupRealtime guard; } catch (const std::exception &) { threw = true; }
  assert(threw); original();
  setenv("RT_CONTROL_ECAT_STARTUP_PRIORITY", "80", 1);
  {
    ethercat_driver::StartupRealtime guard;
    assert(guard.enabled() && policy == SCHED_FIFO && priority == 80);
    assert(CPU_COUNT(&affinity) == 1 && CPU_ISSET(14, &affinity));
    assert(guard.restore() == 0); original();
  }
  try { ethercat_driver::StartupRealtime guard; throw std::runtime_error("activation failed"); }
  catch (const std::runtime_error &) {}
  original();
  fail_fifo = 1; threw = false;
  try { ethercat_driver::StartupRealtime guard; } catch (const std::exception &) { threw = true; }
  assert(threw); original(); fail_fifo = 0;
  for (const char *value : {"-1", "1024", "14junk", "", "99999999999999999999"}) {
    setenv("RT_CONTROL_ECAT_STARTUP_CPU", value, 1); threw = false;
    try { ethercat_driver::StartupRealtime guard; } catch (const std::exception &) { threw = true; }
    assert(threw); original();
  }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "startup.cpp"
            source.write_text(program)
            executable = Path(directory) / "startup"
            result = subprocess.run([
                shutil.which("c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-I", str(header.parents[1]), str(source), "-o", str(executable),
            ], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_native_parameters_and_patch_registration(self):
        launcher = (ROOT / "tools/rt_control_native.sh").read_text()
        self.assertIn('RT_CONTROL_ECAT_STARTUP_CPU="${expected_cpuset}"', launcher)
        self.assertIn('RT_CONTROL_ECAT_STARTUP_PRIORITY="${expected_controller_update_rt_priority}"', launcher)
        for file in ["tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"]:
            self.assertEqual((ROOT / file).read_text().count(
                "patches/ecat_icube/0009-scope-startup-realtime-scheduling.patch"), 2)
