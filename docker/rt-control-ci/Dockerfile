FROM ros:humble-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG IGH_VERSION
ARG IGH_COMMIT

LABEL org.opencontainers.image.title="robot_driver rt-control CI base" \
      org.opencontainers.image.description="Pinned ROS Humble build tools and EtherLab userspace for RT-Control CI" \
      io.rt-control.igh.version="${IGH_VERSION}" \
      io.rt-control.igh.commit="${IGH_COMMIT}"

RUN test -n "${IGH_VERSION}" && [[ "${IGH_COMMIT}" =~ ^[0-9a-f]{40}$ ]] \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      autoconf \
      automake \
      build-essential \
      ca-certificates \
      ccache \
      git \
      libtool \
      liburdfdom-tools \
      pkg-config \
      python3-colcon-common-extensions \
      python3-coverage \
      python3-pip \
      python3-pytest \
      python3-rosdep \
      python3-vcstool \
      python3-yaml \
      ripgrep \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-xacro \
      shellcheck \
    && python3 -m pip install --no-cache-dir pre-commit==3.7.1 \
    && rm -rf /var/lib/apt/lists/*

COPY patches/igh/0001-preserve-verified-pdo-config.patch /tmp/0001.patch
COPY patches/igh/0002-dc-offset-use-sent-application-time.patch /tmp/0002.patch
COPY patches/igh/0003-preop-only-coe.patch /tmp/0003.patch

RUN git init /tmp/ethercat \
    && git -C /tmp/ethercat remote add origin https://gitlab.com/etherlab.org/ethercat.git \
    && git -C /tmp/ethercat fetch --depth 1 origin "${IGH_COMMIT}" \
    && git -C /tmp/ethercat checkout --detach FETCH_HEAD \
    && test "$(git -C /tmp/ethercat rev-parse HEAD)" = "${IGH_COMMIT}" \
    && git -C /tmp/ethercat apply --check /tmp/0001.patch \
    && git -C /tmp/ethercat apply /tmp/0001.patch \
    && git -C /tmp/ethercat apply --check /tmp/0002.patch \
    && git -C /tmp/ethercat apply /tmp/0002.patch \
    && git -C /tmp/ethercat apply --check /tmp/0003.patch \
    && git -C /tmp/ethercat apply /tmp/0003.patch \
    && cd /tmp/ethercat \
    && ./bootstrap \
    && ./configure --prefix=/usr/local/etherlab --disable-kernel \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/ethercat /tmp/0001.patch /tmp/0002.patch /tmp/0003.patch

ENV PATH="/usr/local/etherlab/bin:${PATH}" \
    LD_LIBRARY_PATH="/usr/local/etherlab/lib" \
    CCACHE_DIR="/ci-cache/ccache"
