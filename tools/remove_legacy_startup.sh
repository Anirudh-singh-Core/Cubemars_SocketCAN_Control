#!/usr/bin/env bash
# Explicit administrator cleanup. Never called by import, CLI, or shell startup.
set -euo pipefail
if [[ ${1:-} == '-h' || ${1:-} == '--help' || ${1:-} == '-help' ]]; then
  echo 'Usage: sudo bash remove_legacy_startup.sh --confirm-stopped'
  echo 'Stop motor programs and secure the mechanism first. Removes only HandyFlie CAN boot setup.'
  exit 0
fi
if [[ $EUID -ne 0 || ${1:-} != '--confirm-stopped' ]]; then
  echo 'Require sudo and --confirm-stopped. Read --help.' >&2; exit 2
fi
owner_home=$(getent passwd "${SUDO_USER:-root}" | cut -d: -f6)
if [[ -e "$owner_home/.cache/handyflie/can0.lock" ]]; then
  exec 9>>"$owner_home/.cache/handyflie/can0.lock"
  flock -n 9 || { echo 'CAN session owns can0; close it before cleanup.' >&2; exit 1; }
fi
service=/etc/systemd/system/jetson-can0.service
modules=/etc/modules-load.d/jetson-can.conf
if [[ -f "$service" ]]; then
  if ! grep -q 'Description=Jetson CAN0 pin setup and classic CAN at 1 Mbps' "$service"; then
    echo 'Unexpected service contents; leaving it untouched.' >&2; exit 1
  fi
  systemctl disable --now jetson-can0.service
  rm "$service"
fi
if [[ -f "$modules" ]]; then
  if [[ "$(cat "$modules")" != $'can\ncan_raw\nmttcan' ]]; then
    echo 'Unexpected module configuration; leaving it untouched.' >&2; exit 1
  fi
  rm "$modules"
fi
systemctl daemon-reload
systemctl reset-failed jetson-can0.service 2>/dev/null || true
echo 'Custom CAN boot setup removed. Explicit interface setup is now required.'
