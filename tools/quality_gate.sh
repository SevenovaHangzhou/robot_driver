#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

cd "${repository_root}"

for required_command in git bash python3; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Required quality-gate command is missing: ${required_command}" >&2
    exit 2
  fi
done

if ! python3 -c 'import coverage, yaml' >/dev/null 2>&1; then
  echo "Required Python modules are missing: coverage and/or yaml" >&2
  echo "Install Ubuntu packages python3-coverage and python3-yaml." >&2
  exit 2
fi

python3 tools/repository_gate.py
python3 -m tools.driver_variant_source_projections --repository-root "${repository_root}"

mapfile -t shell_files < <(
  git ls-files --cached --others --exclude-standard -- \
    '*.sh' \
    'src/rt_control/rt_control_bringup/scripts/rt_control_start'
)
if (( ${#shell_files[@]} > 0 )); then
  bash -n "${shell_files[@]}"
fi

if command -v shellcheck >/dev/null 2>&1; then
  shellcheck --severity=warning "${shell_files[@]}"
elif [[ "${REQUIRE_SHELLCHECK:-0}" == "1" ]]; then
  echo "ShellCheck is required in this environment but is not installed." >&2
  exit 2
else
  echo "NOTICE: shellcheck is unavailable locally; CI will enforce it." >&2
fi

python3 -m coverage erase
python3 -m coverage run --branch \
  --source=tools \
  -m pytest tools/tests -q
python3 -m coverage report --fail-under=80 \
  --include='tools/repository_gate.py,tools/pr_contract_gate.py,tools/driver_variant.py,tools/driver_variant_projections.py,tools/driver_variant_source_projections.py'
echo "NOTICE: informational coverage for remaining tools scripts (not gated):" >&2
python3 -m coverage report --include='tools/diff_legacy.py,tools/rt_control_axis_state_check.py,tools/rt_control_thread_affinity.py,tools/release_test_runner.py,tools/scoped_tests.py,tools/rt_perf_capture.py' || true

bash tools/check_ecat_sync_shutdown_policy.sh
bash tools/check_rt_control_ipc_launcher_policy.sh

echo "PASS: local robot_driver RT-Control quality gate"
