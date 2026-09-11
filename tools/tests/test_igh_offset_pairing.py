import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / "patches/igh/0002-dc-offset-use-sent-application-time.patch"


class OffsetPairingRegressionTest(unittest.TestCase):
    def test_processing_delay_does_not_change_the_read_time_pair(self):
        source_path = os.environ.get("IGH_OFFSET_SOURCE")
        if not source_path:
            self.skipTest("set IGH_OFFSET_SOURCE to the frozen master/fsm_master.c")
        source = Path(source_path).read_text()
        functions = []
        arguments = []
        for width in [32, 64]:
            start = source.index(f"u64 ec_fsm_master_dc_offset{width}(\n")
            end = source.index("\n}\n", start) + 3
            body = source[start:end]
            functions.append(body)
            arguments.append("sent" if "u64 app_time_sent" in body else "ticks")
        threshold = re.search(r"^#define EC_SYSTEM_TIME_TOLERANCE_NS\s+(\d+)\s*$", source, re.MULTILINE)
        self.assertIsNotNone(threshold)
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
#define HZ 1000
#define EC_ABS(value) ((value) < 0 ? -(value) : (value))
#define EC_SLAVE_DBG(slave, ...) ((void)(slave))
''' + f"\n#define EC_SYSTEM_TIME_TOLERANCE_NS {threshold.group(1)}\n" + "\n".join(functions)
        for width, argument in zip([32, 64], arguments):
            program += f'''
static u64 apply{width}(ec_fsm_master_t *fsm, u64 system, u64 old, u64 sent, unsigned long ticks) {{
  (void)sent; (void)ticks;
  return ec_fsm_master_dc_offset{width}(fsm, system, old, {argument});
}}
'''
        program += r'''
int main(void) {
  const struct { u64 sent, system, old, now, expected; unsigned long ticks; } cases[] = {
    {1000000000, 1000000000, 35000000, 1004000000, 35000000, 6},
    {1000000000, 1002000000, 35000000, 1004000000, 33000000, 6},
    {1000000000, 998000000, 35000000, 1004000000, 37000000, 6},
    {1000000000, 1000010000, 35000000, 1004000000, 35000000, 6},
    {0x100000010ULL, 0xffd00010ULL, 42, 0x1003d0910ULL, 0x30002a, 6}
  };
  ec_master_t master = {0};
  ec_slave_t slave = {&master};
  ec_fsm_master_t fsm = {&slave};
  for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    master.app_time = cases[i].now;
    u64 a = apply32(&fsm, cases[i].system, cases[i].old, cases[i].sent, cases[i].ticks);
    u64 b = apply64(&fsm, cases[i].system, cases[i].old, cases[i].sent, cases[i].ticks);
    if (a != (u32)cases[i].expected || b != cases[i].expected) {
      fprintf(stderr, "case %u: offsets %" PRIu64 "/%" PRIu64 " expected %" PRIu64,
              i, a, b, cases[i].expected);
      return 1;
    }
  }
  return 0;
}
'''
        compiler = shutil.which("cc")
        self.assertIsNotNone(compiler)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offset_test.c"
            executable = Path(directory) / "offset_test"
            path.write_text(program)
            build = subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                                    str(path), "-o", str(executable)], capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_patch_preserves_sync_tolerance_and_slave_configuration(self):
        patch = PATCH.read_text()
        changed = "\n".join(line[1:] for line in patch.splitlines()
                            if line.startswith(("+", "-")) and not line.startswith(("+++", "---")))
        for token in ["EC_SYSTEM_TIME_TOLERANCE_NS", "EC_DC_SYNC_WAIT_MS", "EC_DC_MAX_SYNC_DIFF_NS"]:
            self.assertNotIn(token, changed)
        self.assertNotIn("fsm_slave_config.c", patch)
        self.assertIn("+        app_time_sent = master->app_time;", patch)
        self.assertLess(patch.index("+            datagram->app_time_sent = app_time_sent;"),
                        patch.index("smp_store_release(&datagram->state, EC_DATAGRAM_SENT)"))

    def test_build_and_runtime_identity_include_the_offset_patch(self):
        for path in ["hostsetup/igh-install.sh", "hostsetup/verify-host.sh",
                     "tools/rt_control_native.sh", "docker/rt-control/Dockerfile"]:
            text = (ROOT / path).read_text()
            self.assertTrue("IGH_DC_OFFSET_PATCH_SHA256" in text, path)
            self.assertTrue(PATCH.name in text, path)
