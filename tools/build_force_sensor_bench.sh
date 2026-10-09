#!/usr/bin/env bash
# Build only the raw force bench and its pinned, patched EtherCAT dependencies.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bench_root="${1:-/tmp/rt-control-force-bench}"
set +u
source /opt/ros/humble/setup.bash
set -u
python3 - "${repo_root}" "${bench_root}" <<'PY'
from pathlib import Path
import hashlib
import shutil
import subprocess
import sys

repo, target = map(Path, sys.argv[1:])
source = next((p for p in (repo / 'src/vendor/ecat_icube', repo / 'src/src/vendor/ecat_icube')
               if (p / '.git').exists()), None)
if source is None:
    raise SystemExit('Import the deps.repos EtherCAT dependency first')
expected = '1390be742986f4e898ca112e49bb24805be9899a'
actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source, text=True).strip()
if actual != expected:
    raise SystemExit('EtherCAT dependency does not match the frozen deps.repos commit')
patches = sorted((repo / 'patches/ecat_icube').glob('*.patch'))
digest = hashlib.sha256(expected.encode() + b''.join(p.read_bytes() for p in patches)).hexdigest()
vendor = target / 'vendor'
marker = target / 'vendor-patch.sha256'
if vendor.exists():
    if not marker.exists() or marker.read_text().strip() != digest:
        raise SystemExit('Use a new build directory: existing vendor patch identity differs')
else:
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=source, text=True).strip():
        raise SystemExit('Use a clean frozen EtherCAT dependency; existing source changes are preserved')
    shutil.copytree(source, vendor, ignore=shutil.ignore_patterns('.git'))
    for patch in patches:
        subprocess.run(['git', 'apply', '--check', str(patch)], cwd=vendor, check=True)
        subprocess.run(['git', 'apply', str(patch)], cwd=vendor, check=True)
    marker.write_text(digest + '\n')
PY
qos_source="${repo_root}/src/vendor/robot_interfaces/qos"
if [[ ! -d "${qos_source}" ]]; then
  qos_source="${repo_root}/src/src/vendor/robot_interfaces/qos"
fi
colcon --log-base "${bench_root}/log" build \
  --base-paths "${bench_root}/vendor" "${qos_source}" \
    "${repo_root}/src/rt_control/rt_control_semantic_components" \
    "${repo_root}/src/rt_control/rt_force_torque_broadcaster" \
    "${repo_root}/src/rt_control/robot_hw_ethercat" \
    "${repo_root}/src/rt_control/rt_control_bringup" \
  --build-base "${bench_root}/build" --install-base "${bench_root}/install" \
  --packages-select robot_interfaces_qos ethercat_interface ethercat_driver \
    ethercat_generic_slave ethercat_generic_cia402_drive \
    rt_control_semantic_components rt_force_torque_broadcaster \
    robot_hw_ethercat rt_control_bringup \
  --cmake-args -DBUILD_TESTING=OFF
printf 'Built raw force bench. Source %s/install/setup.bash before launching.\n' "${bench_root}"
