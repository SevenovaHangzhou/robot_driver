import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "patches/igh/experimental/0003-dc-offset-trigger-100us.patch"


class InitialOffsetTriggerTest(unittest.TestCase):
    def test_recorded_submillisecond_offsets_and_boundary_cases(self):
        source_path = os.environ.get("IGH_TRIGGER_SOURCE")
        if not source_path:
            self.skipTest("set IGH_TRIGGER_SOURCE to the experimental master/fsm_master.c")
        source = Path(source_path).read_text()
        threshold = re.search(r"^#define EC_SYSTEM_TIME_TOLERANCE_NS\s+(\d+)\s*$", source, re.MULTILINE)
        self.assertIsNotNone(threshold)
        functions = []
        for width in [32, 64]:
            start = source.index(f"u64 ec_fsm_master_dc_offset{width}(\n")
            functions.append(source[start:source.index("\n}\n", start) + 3])
        program = r'''
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
typedef uint64_t u64;
typedef uint32_t u32;
typedef int64_t s64;
typedef int32_t s32;
typedef struct { u64 app_time; } ec_master_t;
typedef struct { ec_master_t *master; } ec_slave_t;
typedef struct { ec_slave_t *slave; } ec_fsm_master_t;
#define EC_ABS(value) ((value) < 0 ? -(value) : (value))
#define EC_SLAVE_DBG(slave, ...) ((void)(slave))
''' + f"#define EC_SYSTEM_TIME_TOLERANCE_NS {threshold.group(1)}\n" + "\n".join(functions)
        program += r'''
int main(void) {
  const struct { s64 difference, adjustment; } cases[] = {
    {808100, 808100}, {-277009, -277009}, {129773, 129773}, {-41723, 0},
    {0, 0}, {99999, 0}, {-99999, 0}, {100000, 0}, {-100000, 0},
    {100001, 100001}, {-100001, -100001}, {1000000, 1000000},
    {2000000, 2000000}, {-2000000, -2000000}
  };
  ec_master_t master = {9876543210ULL};
  ec_slave_t slave = {&master};
  ec_fsm_master_t fsm = {&slave};
  const u64 sent = 0x100000020ULL;
  for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    const u64 system = sent - cases[i].difference;
    const u64 old = 50000;
    const u64 expected = old + cases[i].adjustment;
    const u64 a = ec_fsm_master_dc_offset32(&fsm, system, old, sent);
    const u64 b = ec_fsm_master_dc_offset64(&fsm, system, old, sent);
    if (a != (u32)expected || b != expected) {
      fprintf(stderr, "case %u difference=%" PRId64 ": offsets %" PRIu64
              "/%" PRIu64 " expected %" PRIu64 "/%" PRIu64 "\n",
              i, cases[i].difference, a, b, (u64)(u32)expected, expected);
      return 1;
    }
  }
  return 0;
}
'''
        compiler = shutil.which("cc")
        self.assertIsNotNone(compiler)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trigger.c"
            executable = Path(directory) / "trigger"
            path.write_text(program)
            result = subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                                     str(path), "-o", str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_experiment_changes_only_the_initial_offset_trigger(self):
        patch = PATCH.read_text()
        self.assertEqual(patch.count("diff --git"), 1)
        self.assertIn("a/master/fsm_master.c b/master/fsm_master.c", patch)
        changes = [line for line in patch.splitlines()
                   if line.startswith(("+", "-")) and not line.startswith(("+++", "---"))]
        self.assertEqual(changes, ["-#define EC_SYSTEM_TIME_TOLERANCE_NS 1000000",
                                   "+#define EC_SYSTEM_TIME_TOLERANCE_NS 100000"])

    def test_unvalidated_experiment_is_not_in_default_builds(self):
        for file in ["hostsetup/igh-install.sh", "tools/rt_control_native.sh",
                     "hostsetup/verify-host.sh", "docker/rt-control/Dockerfile"]:
            self.assertNotIn(PATCH.name, (ROOT / file).read_text(), file)
