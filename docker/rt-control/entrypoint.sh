#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/rt_control_ws/install/setup.bash

exec "$@"
