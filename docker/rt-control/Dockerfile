FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG IGH_VERSION

RUN test -n "${IGH_VERSION}" \
    && sed -i \
      -e 's|http://archive.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
      -e 's|http://security.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
      /etc/apt/sources.list \
    && test -f /usr/share/ros-apt-source/ros2.sources \
    && sed -i \
      -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
      -e 's|https://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
      -e 's/^Types: deb deb-src$/Types: deb/' \
      /usr/share/ros-apt-source/ros2.sources \
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
      ros-humble-diagnostic-updater \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-robot-state-publisher \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 --branch "${IGH_VERSION}" \
      https://gitlab.com/etherlab.org/ethercat.git ethercat \
    && cd ethercat \
    && ./bootstrap \
    && ./configure --prefix=/usr/local --disable-kernel \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/ethercat

WORKDIR /opt/rt_control_ws
COPY . src/robot

ENV ROSDISTRO_INDEX_URL="https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"

RUN source /opt/ros/humble/setup.bash \
    && if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && sed -i \
      's|https://raw.githubusercontent.com/|https://mirrors.tuna.tsinghua.edu.cn/github-raw/|g' \
      /etc/ros/rosdep/sources.list.d/20-default.list
RUN source /opt/ros/humble/setup.bash \
    && rosdep update --rosdistro humble \
    && rosdep install \
      --from-paths src/robot/src \
      --ignore-src \
      --rosdistro humble \
      --skip-keys ethercat_driver \
      -r -y \
    && colcon --log-base /opt/rt_control_ws/log build \
      --base-paths src/robot/src \
      --install-base /opt/rt_control_ws/install \
      --build-base /opt/rt_control_ws/build \
      --merge-install \
    && rm -rf /var/lib/apt/lists/* /root/.ros/rosdep

ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

COPY docker/rt-control/entrypoint.sh /rt-control-entrypoint.sh
ENTRYPOINT ["/rt-control-entrypoint.sh"]
CMD ["ros2", "launch", "rt_control_bringup", "rt_control.launch.py"]
