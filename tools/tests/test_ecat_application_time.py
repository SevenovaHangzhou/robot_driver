import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATCH = "patches/ecat_icube/0008-use-monotonic-application-time.patch"


class ApplicationTimeRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source_path = Path(os.environ.get(
            "ECAT_TIMEBASE_SOURCE",
            ROOT / "src/vendor/ecat_icube/ethercat_interface/src/ec_master.cpp",
        ))
        if not source_path.is_file():
            raise unittest.SkipTest("Import the frozen vendor tree or set ECAT_TIMEBASE_SOURCE")
        cls.source = source_path.read_text()

    def test_actual_conversion_preserves_monotonic_uptime(self):
        macro = re.search(r"^#define EC_NEWTIMEVAL2NANO\(TV\) \\\n[^\n]+", self.source, re.MULTILINE)
        self.assertIsNotNone(macro, "application-time conversion macro was not found")
        compiler = shutil.which("c++")
        self.assertIsNotNone(compiler, "a C++ compiler is required for this regression")
        program = "#include <cstdint>\n#include <ctime>\n#include <iostream>\n" + macro.group(0) + r'''
int main() {
  const timespec probes[] = {{0, 0}, {0, 999999999}, {1, 1},
                            {3600, 123456789}, {4294967296LL, 999999999}};
  const uint64_t expected[] = {0ULL, 999999999ULL, 1000000001ULL,
                              3600123456789ULL, 4294967296999999999ULL};
  for (unsigned int i = 0; i < sizeof(probes) / sizeof(probes[0]); ++i) {
    const uint64_t actual = EC_NEWTIMEVAL2NANO(probes[i]);
    if (actual != expected[i]) {
      std::cerr << "case " << i << ": " << actual << " != " << expected[i];
      return 1;
    }
  }
}
'''
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "time_test.cpp"
            executable = Path(directory) / "time_test"
            source.write_text(program)
            build = subprocess.run(
                [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)],
                text=True, capture_output=True, check=False,
            )
            self.assertEqual(build.returncode, 0, build.stderr)
            result = subprocess.run([str(executable)], text=True, capture_output=True, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_setup_and_cyclic_paths_use_the_same_clock(self):
        for method in ["addSlave", "activate", "update", "writeData"]:
            with self.subTest(method=method):
                body = self.source.split(f"EcMaster::{method}(", 1)[1].split("\n}\n", 1)[0]
                self.assertIn("ecrt_master_application_time", body)
                self.assertIn("clock_gettime(CLOCK_MONOTONIC", body)
                self.assertNotIn("CLOCK_REALTIME", body)


class ApplicationTimePatchRegistrationTest(unittest.TestCase):
    def test_native_and_docker_apply_and_verify_the_timebase_patch(self):
        self.assertTrue((ROOT / PATCH).is_file())
        for name in ["tools/bootstrap_native_dev.sh", "docker/rt-control/Dockerfile"]:
            text = (ROOT / name).read_text()
            self.assertEqual(text.count(PATCH), 2, name)
            self.assertLess(text.index("0007-configure-etherlab-prefix.patch"), text.index(PATCH))

    def test_patch_does_not_change_dc_cycles_or_pdos(self):
        patch = (ROOT / PATCH).read_text()
        changes = "\n".join(line[1:] for line in patch.splitlines()
                            if line.startswith(("+", "-")) and not line.startswith(("+++", "---")))
        for forbidden in ["ecrt_slave_config_dc", "interval_", "processData", "ecrt_master_send("]:
            self.assertNotIn(forbidden, changes)


if __name__ == "__main__":
    unittest.main()
