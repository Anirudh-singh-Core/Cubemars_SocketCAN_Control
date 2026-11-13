#!/usr/bin/env bash
# Manual setup for this project's Jetson Orin Nano J17 CAN0. No installation/autostart.
set -euo pipefail
if [[ ${1:-} == '-h' || ${1:-} == '--help' || ${1:-} == '-help' ]]; then
  echo 'Usage: sudo bash setup_jetson_can.sh --confirm-stopped'
  echo 'Orin Nano J17 only. Stop motor programs first. Configures classic can0 at 1 Mbps; sends no motor frame.'
  exit 0
fi
if [[ $EUID -ne 0 || ${1:-} != '--confirm-stopped' ]]; then
  echo 'Require sudo and --confirm-stopped. Read --help.' >&2; exit 2
fi
model=$(tr -d '\0' </proc/device-tree/model)
if [[ "$model" != *'Orin Nano'* ]]; then
  echo "Unverified Jetson model: $model. Pin addresses are board-specific." >&2; exit 1
fi
owner_home=$(getent passwd "${SUDO_USER:-root}" | cut -d: -f6)
owner_user=${SUDO_USER:-root}
owner_group=$(id -gn "$owner_user")
install -d -o "$owner_user" -g "$owner_group" "$owner_home/.cache/handyflie"
if [[ ! -e "$owner_home/.cache/handyflie/can0.lock" ]]; then
  install -o "$owner_user" -g "$owner_group" -m 600 /dev/null "$owner_home/.cache/handyflie/can0.lock"
fi
# Shared with Session and the legacy beta; do not reset a cooperating active owner.
exec 9>>"$owner_home/.cache/handyflie/can0.lock"
flock -n 9 || { echo 'CAN session owns can0; close it before setup.' >&2; exit 1; }
modprobe can
modprobe can_raw
modprobe mttcan
ip link set dev can0 down
busybox devmem 0x0c303018 32 0xc458
busybox devmem 0x0c303010 32 0xc400
ip link set dev can0 type can bitrate 1000000 loopback off listen-only off fd off berr-reporting on restart-ms 0
ip link set dev can0 up
ip -details -statistics link show can0
