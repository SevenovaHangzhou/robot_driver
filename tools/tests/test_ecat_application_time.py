import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


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


if __name__ == "__main__":
    unittest.main()
