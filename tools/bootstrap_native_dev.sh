#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_root="${RT_CONTROL_NATIVE_WS:-$(cd -- "${repository_root}/.." && pwd)}"
dependency_manifest="${repository_root}/deps.repos"
build_base="${workspace_root}/build"
install_base="${workspace_root}/install"
log_base="${workspace_root}/log"
vendor_root="${workspace_root}/src/vendor"

readonly repository_root workspace_root dependency_manifest
readonly build_base install_base log_base vendor_root

readonly -a runtime_packages=(
  robot_motion_interfaces
  robot_rt_control_interfaces
  robot_system_interfaces
  robot_interfaces_qos
  bms_node
  canopen_master_driver
  canopen_ros2_control
  control_api_adapter
  ethercat_driver
  ethercat_generic_cia402_drive
  joint_trajectory_controller
  plc_node
  robot_description
  rt_control_interfaces
  robot_hw_ethercat
  robot_hw_canopen
  enable_manager
  rt_diagnostics
  rolling_trajectory_controller
  rt_control_bringup
)

info()
{
  printf '[rt-control-native-bootstrap] %s\n' "$*"
}

fail()
{
  printf '[rt-control-native-bootstrap] FAIL: %s\n' "$*" >&2
  exit 1
}

usage()
{
  cat <<'EOF'
Usage:
  tools/bootstrap_native_dev.sh doctor
  tools/bootstrap_native_dev.sh prepare
  tools/bootstrap_native_dev.sh install-deps
  tools/bootstrap_native_dev.sh build [colcon package-selection arguments]
  tools/bootstrap_native_dev.sh all
  tools/bootstrap_native_dev.sh env

RT_CONTROL_NATIVE_WS defaults to the parent of this repository. The supported
target layout is /home/ar/rt-control-dev/robot with workspace outputs and
vendor repositories under /home/ar/rt-control-dev.

prepare never resets, checks out, or cleans an existing vendor repository.
install-deps is the only command that may invoke apt through rosdep/sudo.
EOF
}

verify_workspace_layout()
{
  [[ -f "${dependency_manifest}" ]] || fail "missing ${dependency_manifest}"
  if [[ "${workspace_root}" == "${repository_root}" ||
    "${workspace_root}" == "${repository_root}/"* ]]; then
    fail "RT_CONTROL_NATIVE_WS must be outside the repository: ${workspace_root}"
  fi
}

require_commands()
{
  local command
  for command in "$@"; do
    command -v "${command}" >/dev/null 2>&1 || fail "missing command: ${command}"
  done
}

manifest_rows()
{
  python3 - "${dependency_manifest}" <<'PY'
import sys
from pathlib import Path

import yaml

document = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
repositories = document.get("repositories") or {}
for relative_path, spec in repositories.items():
    version = str(spec.get("version", ""))
    if not version:
        raise SystemExit(f"missing pinned version for {relative_path}")
    print(f"{relative_path}\t{version}")
PY
}

refuse_partial_vendor_tree()
{
  local existing=0
  local expected=0
  local relative_path
  local version
  while IFS=$'\t' read -r relative_path version; do
    ((expected += 1))
    if [[ -e "${workspace_root}/${relative_path}" ]]; then
      ((existing += 1))
    fi
  done < <(manifest_rows)

  if (( existing != 0 && existing != expected )); then
    fail "partial vendor tree (${existing}/${expected}); preserve it and resolve manually"
  fi
}

verify_vendor_heads()
{
  local actual
  local relative_path
  local repository
  local version
  while IFS=$'\t' read -r relative_path version; do
    repository="${workspace_root}/${relative_path}"
    [[ -d "${repository}/.git" ]] || fail "missing vendor repository: ${repository}"
    actual="$(git -C "${repository}" rev-parse HEAD)"
    [[ "${actual}" == "${version}" ]] ||
      fail "${relative_path} is at ${actual}, expected frozen ${version}; no checkout was attempted"
  done < <(manifest_rows)

  [[ -d "${workspace_root}/src/vendor/robot_interfaces/robot_motion_interfaces" ]] ||
    fail "robot_interfaces vendor is missing robot_motion_interfaces"
  [[ -d "${workspace_root}/src/vendor/robot_interfaces/robot_rt_control_interfaces" ]] ||
    fail "robot_interfaces vendor is missing robot_rt_control_interfaces"
  [[ -d "${workspace_root}/src/vendor/robot_interfaces/robot_system_interfaces" ]] ||
    fail "robot_interfaces vendor is missing robot_system_interfaces"
  [[ -d "${workspace_root}/src/vendor/robot_interfaces/qos" ]] ||
    fail "robot_interfaces vendor is missing robot_interfaces_qos"
}

apply_patch_once()
{
  local patch_file="${repository_root}/$1"
  local vendor_repository="${workspace_root}/$2"
  [[ -f "${patch_file}" ]] || fail "missing patch: ${patch_file}"

  if git -C "${vendor_repository}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
    info "already applied: ${patch_file#${repository_root}/}"
    return
  fi
  git -C "${vendor_repository}" apply --check "${patch_file}" ||
    fail "patch does not apply cleanly and is not already applied: ${patch_file}"
  git -C "${vendor_repository}" apply "${patch_file}"
  info "applied: ${patch_file#${repository_root}/}"
}

apply_frozen_patches()
{
  apply_patch_once \
    patches/ecat_icube/0001-rt-control-preload-and-diagnostics.patch \
    src/vendor/ecat_icube
  apply_patch_once \
    patches/ecat_icube/0002-wait-for-complete-bus-before-preload.patch \
    src/vendor/ecat_icube
  apply_patch_once \
    patches/ecat_icube/0003-orderly-master-deactivation.patch \
    src/vendor/ecat_icube
  apply_patch_once \
    patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch \
    src/vendor/ros2_canopen
  apply_patch_once \
    patches/ros2_canopen/0002-lely-preconfigured-txqlen.patch \
    src/vendor/ros2_canopen
  apply_patch_once \
    patches/ros2_canopen/0003-quiesce-callbacks-before-driver-removal.patch \
    src/vendor/ros2_canopen
  apply_patch_once \
    patches/ros2_canopen/0004-name-canopen-master-loop-thread.patch \
    src/vendor/ros2_canopen
  apply_patch_once \
    patches/ros2_controllers/0001-jtc-start-consistency.patch \
    src/vendor/ros2_controllers
  apply_patch_once \
    patches/ros2_controllers/0002-use-contract-qos-profiles.patch \
    src/vendor/ros2_controllers
}

verify_patched_vendor_tree()
{
  local vendor_repository="${workspace_root}/$1"
  shift
  local actual_index
  local actual_tree
  local expected_index
  local expected_tree
  local patch_path
  local temporary_directory

  temporary_directory="$(mktemp -d)"
  expected_index="${temporary_directory}/expected.index"
  actual_index="${temporary_directory}/actual.index"

  if ! GIT_INDEX_FILE="${expected_index}" git -C "${vendor_repository}" read-tree HEAD; then
    rmdir -- "${temporary_directory}"
    fail "cannot initialize expected tree for ${vendor_repository}"
  fi
  for patch_path in "$@"; do
    if ! GIT_INDEX_FILE="${expected_index}" git -C "${vendor_repository}" \
      apply --cached "${repository_root}/${patch_path}"; then
      rm -f -- "${expected_index}"
      rmdir -- "${temporary_directory}"
      fail "cannot construct frozen patched tree for ${vendor_repository}"
    fi
  done
  expected_tree="$(
    GIT_INDEX_FILE="${expected_index}" git -C "${vendor_repository}" write-tree
  )"

  if ! GIT_INDEX_FILE="${actual_index}" git -C "${vendor_repository}" read-tree HEAD ||
    ! GIT_INDEX_FILE="${actual_index}" git -C "${vendor_repository}" add -A -- .; then
    rm -f -- "${expected_index}" "${actual_index}"
    rmdir -- "${temporary_directory}"
    fail "cannot inspect working tree for ${vendor_repository}"
  fi
  actual_tree="$(
    GIT_INDEX_FILE="${actual_index}" git -C "${vendor_repository}" write-tree
  )"

  rm -f -- "${expected_index}" "${actual_index}"
  rmdir -- "${temporary_directory}"
  [[ "${actual_tree}" == "${expected_tree}" ]] ||
    fail "${vendor_repository} contains changes outside the frozen patch set"
}

verify_frozen_vendor_trees()
{
  verify_patched_vendor_tree src/vendor/ecat_icube \
    patches/ecat_icube/0001-rt-control-preload-and-diagnostics.patch \
    patches/ecat_icube/0002-wait-for-complete-bus-before-preload.patch \
    patches/ecat_icube/0003-orderly-master-deactivation.patch
  verify_patched_vendor_tree src/vendor/ros2_canopen \
    patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch \
    patches/ros2_canopen/0002-lely-preconfigured-txqlen.patch \
    patches/ros2_canopen/0003-quiesce-callbacks-before-driver-removal.patch \
    patches/ros2_canopen/0004-name-canopen-master-loop-thread.patch
  verify_patched_vendor_tree src/vendor/ros2_controllers \
    patches/ros2_controllers/0001-jtc-start-consistency.patch \
    patches/ros2_controllers/0002-use-contract-qos-profiles.patch
}

prepare_sources()
{
  local first_repository="${vendor_root}/ecat_icube"
  verify_workspace_layout
  require_commands git python3 vcs
  python3 -c 'import yaml' >/dev/null 2>&1 || fail "missing Python yaml module"
  refuse_partial_vendor_tree

  if [[ ! -e "${first_repository}" ]]; then
    mkdir -p "${workspace_root}"
    info "importing frozen vendor repositories into ${workspace_root}"
    vcs import --shallow --retry 5 --workers 1 "${workspace_root}" < "${dependency_manifest}"
  else
    info "preserving existing complete vendor tree"
  fi

  verify_vendor_heads
  if (verify_frozen_vendor_trees) >/dev/null 2>&1; then
    info "frozen patch set is already applied exactly"
  else
    apply_frozen_patches
  fi
  verify_vendor_heads
  verify_frozen_vendor_trees
}

source_build_environment()
{
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f "${install_base}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${install_base}/setup.bash"
  fi
  set -u
  export PATH="/usr/local/etherlab/bin:${PATH}"
  export LD_LIBRARY_PATH="/usr/local/etherlab/lib:${LD_LIBRARY_PATH:-}"
}

dependency_paths()
{
  colcon list \
    --base-paths "${repository_root}/src" "${vendor_root}" \
    --paths-only \
    --packages-up-to "${runtime_packages[@]}"
}

install_dependencies()
{
  local -a paths
  verify_workspace_layout
  require_commands colcon rosdep sudo
  verify_vendor_heads
  verify_frozen_vendor_trees
  source_build_environment
  mapfile -t paths < <(dependency_paths)
  (( ${#paths[@]} > 0 )) || fail "no dependency paths resolved"
  info "rosdep may install host packages through sudo; this is the explicit mutating step"
  rosdep install \
    --from-paths "${paths[@]}" \
    --ignore-src \
    --rosdistro humble \
    --skip-keys ethercat_driver \
    --dependency-types build \
    --dependency-types buildtool \
    --dependency-types build_export \
    --dependency-types buildtool_export \
    --dependency-types exec \
    -r -y
}

runtime_qos_profile_smoke()
{
  python3 - <<'PY'
from robot_interfaces_qos import (
    control,
    diagnostic,
    fast_state,
    latched,
    rolling_command,
    rolling_state,
    state,
)

for name, factory in (
    ("control", control),
    ("rolling_command", rolling_command),
    ("rolling_state", rolling_state),
    ("fast_state", fast_state),
    ("state", state),
    ("latched", latched),
    ("diagnostic", diagnostic),
):
    if factory() is None:
        raise RuntimeError(f"QoS profile factory returned None: {name}")
PY
}

verify_runtime_install()
{
  local package
  source_build_environment
  for package in "${runtime_packages[@]}"; do
    ros2 pkg prefix "${package}" >/dev/null 2>&1 ||
      fail "full native build did not install runtime package: ${package}"
  done
  runtime_qos_profile_smoke ||
    fail "full native build did not install the robot_interfaces_qos Python API"
  info "PASS: full native runtime package and QoS Python closure is installed"
}

build_workspace()
{
  local -a selection
  local verify_full_runtime=0
  verify_workspace_layout
  require_commands colcon git
  verify_vendor_heads
  verify_frozen_vendor_trees
  source_build_environment
  mkdir -p "${build_base}" "${install_base}" "${log_base}"

  if (( $# > 0 )); then
    selection=("$@")
  else
    selection=(--packages-up-to "${runtime_packages[@]}" --cmake-clean-cache)
    verify_full_runtime=1
  fi

  colcon --log-base "${log_base}" build \
    --base-paths "${repository_root}/src" "${vendor_root}" \
    --build-base "${build_base}" \
    --install-base "${install_base}" \
    --merge-install \
    --symlink-install \
    "${selection[@]}"

  if (( verify_full_runtime == 1 )); then
    verify_runtime_install
  fi
}

doctor()
{
  local metadata="/usr/local/share/rt-control/dependency-versions.env"
  verify_workspace_layout
  require_commands bash git python3 vcs colcon rosdep
  python3 -c 'import yaml' >/dev/null 2>&1 || fail "missing Python yaml module"
  [[ -f /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble is not installed"
  [[ -d /usr/local/etherlab/lib ]] || fail "missing IgH userspace library"
  [[ -f "${metadata}" ]] || fail "missing IgH dependency identity: ${metadata}"
  refuse_partial_vendor_tree
  if [[ -d "${vendor_root}" ]]; then
    verify_vendor_heads
    verify_frozen_vendor_trees
  fi
  info "repository=${repository_root}"
  info "workspace=${workspace_root}"
  info "build=${build_base}"
  info "install=${install_base}"
  info "PASS: native development prerequisites are present"
}

print_environment()
{
  printf 'source /opt/ros/humble/setup.bash\n'
  printf 'source %q\n' "${install_base}/setup.bash"
}

case "${1:-}" in
  doctor)
    doctor
    ;;
  prepare)
    prepare_sources
    ;;
  install-deps)
    install_dependencies
    ;;
  build)
    shift
    build_workspace "$@"
    ;;
  all)
    prepare_sources
    build_workspace
    ;;
  env)
    print_environment
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
