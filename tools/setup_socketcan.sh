#!/usr/bin/env bash
# Explicit setup for an existing physical SocketCAN interface, including Pi 5 adapters.
set -euo pipefail
usage() {
  echo 'Usage: sudo bash tools/setup_socketcan.sh --confirm-stopped [can0]'
  echo 'Configures an existing physical CAN interface at 1 Mbps; sends no motor frame.'
  echo 'Install your adapter/HAT driver first. No GPIO, pinmux, overlays or boot hooks are changed.'
}
if [[ ${1:-} == '-h' || ${1:-} == '--help' || ${1:-} == '-help' ]]; then usage; exit 0; fi
if [[ $EUID -ne 0 || ${1:-} != '--confirm-stopped' || $# -gt 2 ]]; then usage >&2; exit 2; fi
channel=${2:-can0}
if [[ ! $channel =~ ^[a-zA-Z0-9_-]{1,15}$ ]]; then echo 'Invalid interface name' >&2; exit 2; fi
# Validate before bringing anything down. Never configure a non-CAN network link.
ip -details -json link show dev "$channel" | python3 -c 'import json,sys; x=json.load(sys.stdin); sys.exit(0 if x and x[0].get("linkinfo",{}).get("info_kind")=="can" else "Require an existing physical CAN interface")'
owner_user=${SUDO_USER:-root}
owner_home=$(getent passwd "$owner_user" | cut -d: -f6)
owner_group=$(id -gn "$owner_user")
[[ -n $owner_home ]] || { echo 'Cannot resolve owner home' >&2; exit 1; }
install -d -o "$owner_user" -g "$owner_group" "$owner_home/.cache/handyflie"
lock="$owner_home/.cache/handyflie/$channel.lock"
if [[ ! -e $lock ]]; then install -o "$owner_user" -g "$owner_group" -m 600 /dev/null "$lock"; fi
exec 9>>"$lock"
flock -n 9 || { echo 'An active controller owns this channel; close it first.' >&2; exit 1; }
ip link set dev "$channel" down
ip link set dev "$channel" type can bitrate 1000000 loopback off listen-only off fd off restart-ms 0
ip link set dev "$channel" up
ip -details -statistics link show dev "$channel"
