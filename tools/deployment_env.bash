# Source this file; it never starts hardware or changes a ROS domain.
_driver_deployment_env() {
  local driver_repo driver_ws driver_install ruckig_prefix sdk_prefix
  driver_repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd) || return 1
  driver_ws=${ROBOT_DRIVER_WS:-$(dirname "$driver_repo")}
  driver_install=${ROBOT_DRIVER_INSTALL:-$driver_ws/install}
  if [[ ! -f $driver_install/local_setup.bash ]]; then
    echo "Missing driver install: $driver_install/local_setup.bash" >&2
    return 1
  fi
  # local_setup avoids sourcing build-machine underlays stored in setup.bash.
  source /opt/ros/humble/setup.bash || return 1
  source "$driver_install/local_setup.bash" || return 1
  ruckig_prefix=${ROBOT_RUCKIG_PREFIX:-$driver_ws/deps/ruckig/opt/ros/humble}
  if [[ -d $ruckig_prefix ]]; then
    export CMAKE_PREFIX_PATH="$ruckig_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$ruckig_prefix/lib/x86_64-linux-gnu:$ruckig_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
  sdk_prefix=${ROBOT_LIVOX_SDK_PREFIX:-}
  if [[ -n $sdk_prefix ]]; then
    export CMAKE_PREFIX_PATH="$sdk_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$sdk_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
}
_driver_deployment_env
_driver_deployment_result=$?
unset -f _driver_deployment_env
if (( _driver_deployment_result != 0 )); then
  unset _driver_deployment_result
  return 1
fi
unset _driver_deployment_result
