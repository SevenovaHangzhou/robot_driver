#!/usr/bin/env bash
# ZLG PCIe-9140I SocketCAN bring-up for Alfa_V3-Advantech-01, installed as
# /usr/local/sbin/rt-control-zlg-can (user 2026-11-05; can2 head added 2026-11-22, ELECTRI-234):
# can0 left battery 250 kbit/s, can1 right battery 250 kbit/s, can2 head gimbal 1 Mbit/s,
# can3 steering encoders + IMU 500 kbit/s.
set -euo pipefail
modprobe zpcican
for _ in $(seq 1 50); do [[ -e /sys/class/net/can3 ]] && break; sleep 0.1; done
up() {
  ip link set "$1" down
  ip link set "$1" type can bitrate "$2" restart-ms 100
  ip link set "$1" txqueuelen 128
  ip link set "$1" up
}
up can0 250000
up can1 250000
up can2 1000000
up can3 500000
