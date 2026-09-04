FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG IGH_VERSION
ARG IGH_COMMIT

LABEL org.opencontainers.image.version="${IGH_VERSION}" \
      io.rt-control.igh.commit="${IGH_COMMIT}" \
      io.rt-control.igh.preserve-pdo-config="true"

RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy \
    && test -n "${IGH_VERSION}" \
    && [[ "${IGH_COMMIT}" =~ ^[0-9a-f]{40}$ ]] \
    && sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
      -e 's|http://security.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
      /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      autoconf \
      automake \
      build-essential \
      ca-certificates \
      git \
      libtool \
      pkg-config \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-humble-diagnostic-updater \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-robot-state-publisher \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-xacro \
      util-linux \
    && test -x /opt/ros/humble/lib/robot_state_publisher/robot_state_publisher \
    && rm -rf /var/lib/apt/lists/*

# The UID1000 production override uses /home/user for ROS launch logs.
# Create it in the image rather than relying on a host bind mount.
RUN install -d -m 0750 -o 1000 -g 1000 /home/user

WORKDIR /tmp
COPY patches/igh/0001-preserve-verified-pdo-config.patch \
  /tmp/0001-preserve-verified-pdo-config.patch
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy \
    && git init ethercat \
    && cd ethercat \
    && git remote add origin https://gitlab.com/etherlab.org/ethercat.git \
    && git fetch --depth 1 origin "${IGH_COMMIT}" \
    && git checkout --detach FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${IGH_COMMIT}" \
    && git apply --check /tmp/0001-preserve-verified-pdo-config.patch \
    && git apply /tmp/0001-preserve-verified-pdo-config.patch \
    && ./bootstrap \
    && ./configure --prefix=/usr/local/etherlab --disable-kernel \
    && make -j"$(nproc)" \
    && make install \
    && install -d /usr/local/share/rt-control \
    && igh_patch_sha256="$(sha256sum \
      /tmp/0001-preserve-verified-pdo-config.patch | awk '{print $1}')" \
    && printf 'IGH_VERSION=%s\nIGH_COMMIT=%s\nIGH_PRESERVE_PDO_PATCH_SHA256=%s\n' \
      "${IGH_VERSION}" "${IGH_COMMIT}" "${igh_patch_sha256}" \
      > /usr/local/share/rt-control/dependency-versions.env \
    && ldconfig \
    && rm -rf /tmp/ethercat \
    && rm -f /tmp/0001-preserve-verified-pdo-config.patch

ENV PATH="/usr/local/etherlab/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/etherlab/lib"

WORKDIR /opt/rt_control_ws
COPY . src/robot

RUN git config --global http.version HTTP/1.1 \
    && vcs import --shallow --retry 5 --workers 1 /opt/rt_control_ws < src/robot/deps.repos \
    && test -d src/vendor/ecat_icube \
    && test -d src/vendor/ros2_canopen \
    && test -d src/vendor/ros2_controllers \
    && test -d src/vendor/robot_interfaces/robot_rt_control_interfaces \
    && test -d src/vendor/robot_interfaces/robot_system_interfaces \
    && test -d src/vendor/robot_interfaces/qos \
    && test -f src/vendor/robot_interfaces/contract/views/rt_control.md \
    && git -C src/vendor/ecat_icube apply --check \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0001-rt-control-preload-and-diagnostics.patch \
    && git -C src/vendor/ecat_icube apply \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0001-rt-control-preload-and-diagnostics.patch \
    && git -C src/vendor/ecat_icube apply --check \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0002-wait-for-complete-bus-before-preload.patch \
    && git -C src/vendor/ecat_icube apply \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0002-wait-for-complete-bus-before-preload.patch \
    && git -C src/vendor/ecat_icube apply --check \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0003-orderly-master-deactivation.patch \
    && git -C src/vendor/ecat_icube apply \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0003-orderly-master-deactivation.patch \
    && git -C src/vendor/ecat_icube apply --check \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0004-preserve-fixed-pdo-config.patch \
    && git -C src/vendor/ecat_icube apply \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0004-preserve-fixed-pdo-config.patch \
    && git -C src/vendor/ecat_icube apply --check \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0005-use-component-parameters-for-ec-modules.patch \
    && git -C src/vendor/ecat_icube apply \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0005-use-component-parameters-for-ec-modules.patch \
    && git -C src/vendor/ecat_icube apply --check \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0006-validate-component-module-parameters.patch \
    && git -C src/vendor/ecat_icube apply \
      /opt/rt_control_ws/src/robot/patches/ecat_icube/0006-validate-component-module-parameters.patch \
    && git -C src/vendor/ros2_canopen apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch \
    && git -C src/vendor/ros2_canopen apply \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0001-rt-control-lifecycle-and-emcy-stop.patch \
    && git -C src/vendor/ros2_canopen apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0002-lely-preconfigured-txqlen.patch \
    && git -C src/vendor/ros2_canopen apply \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0002-lely-preconfigured-txqlen.patch \
    && git -C src/vendor/ros2_canopen apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0003-quiesce-callbacks-before-driver-removal.patch \
    && git -C src/vendor/ros2_canopen apply \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0003-quiesce-callbacks-before-driver-removal.patch \
    && git -C src/vendor/ros2_canopen apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0004-name-canopen-master-loop-thread.patch \
    && git -C src/vendor/ros2_canopen apply \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0004-name-canopen-master-loop-thread.patch \
    && git -C src/vendor/ros2_canopen apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0005-derive-motor-topology-from-hardware-info.patch \
    && git -C src/vendor/ros2_canopen apply \
      /opt/rt_control_ws/src/robot/patches/ros2_canopen/0005-derive-motor-topology-from-hardware-info.patch \
    && git -C src/vendor/ros2_controllers apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_controllers/0001-jtc-start-consistency.patch \
    && git -C src/vendor/ros2_controllers apply \
      /opt/rt_control_ws/src/robot/patches/ros2_controllers/0001-jtc-start-consistency.patch \
    && git -C src/vendor/ros2_controllers apply --check \
      /opt/rt_control_ws/src/robot/patches/ros2_controllers/0002-use-contract-qos-profiles.patch \
    && git -C src/vendor/ros2_controllers apply \
      /opt/rt_control_ws/src/robot/patches/ros2_controllers/0002-use-contract-qos-profiles.patch

ENV ROSDISTRO_INDEX_URL="https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"

RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy \
    && source /opt/ros/humble/setup.bash \
    && if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && sed -i \
      -e 's|https://raw.githubusercontent.com/|https://mirrors.tuna.tsinghua.edu.cn/github-raw/|g' \
      -e '\|/releases/fuerte.yaml|d' \
      /etc/ros/rosdep/sources.list.d/20-default.list
RUN unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy ALL_PROXY all_proxy \
    && source /opt/ros/humble/setup.bash \
    && rosdep update --rosdistro humble \
    && apt-get update \
    && rt_control_packages=( \
      robot_rt_control_interfaces \
      robot_system_interfaces \
      robot_interfaces_qos \
      bms_node \
      canopen_master_driver \
      canopen_ros2_control \
      control_api_adapter \
      ethercat_driver \
      ethercat_generic_cia402_drive \
      joint_trajectory_controller \
      plc_node \
      robot_description \
      rt_control_interfaces \
      robot_hw_ethercat \
      robot_hw_canopen \
      enable_manager \
      rt_diagnostics \
      rt_control_bringup \
    ) \
    && mapfile -t rt_control_dependency_paths < <( \
      colcon list --base-paths src --paths-only \
        --packages-up-to "${rt_control_packages[@]}" \
    ) \
    && (( ${#rt_control_dependency_paths[@]} > 0 )) \
    && rosdep install \
      --from-paths "${rt_control_dependency_paths[@]}" \
      --ignore-src \
      --rosdistro humble \
      --skip-keys ethercat_driver \
      --dependency-types build \
      --dependency-types buildtool \
      --dependency-types build_export \
      --dependency-types buildtool_export \
      --dependency-types exec \
      -r -y \
    && colcon --log-base /opt/rt_control_ws/log build \
      --base-paths src \
      --install-base /opt/rt_control_ws/install \
      --build-base /opt/rt_control_ws/build \
      --merge-install \
      --packages-up-to "${rt_control_packages[@]}" \
      --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /var/lib/apt/lists/* /root/.ros/rosdep

COPY docker/rt-control/entrypoint.sh /rt-control-entrypoint.sh
ENTRYPOINT ["/rt-control-entrypoint.sh"]
CMD ["/opt/rt_control_ws/install/lib/rt_control_bringup/rt_control_start"]
