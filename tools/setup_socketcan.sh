#!/usr/bin/env bash
# One explicit setup path. Never called by the GUI, imports, or terminal startup.
set -euo pipefail
usage() {
  echo 'Usage: sudo bash setup_socketcan.sh --confirm-stopped [--jetson-orin-nano] [--recover-driver] [can0]'
  echo 'Configures classic SocketCAN at 1 Mbps. Stop controllers and support joints first.'
  echo '--jetson-orin-nano loads mttcan and configures the verified Orin Nano J17 pins.'
  echo 'Without that flag, the physical CAN interface/driver must already exist (e.g. Pi 5 HAT).'
  echo '--recover-driver explicitly reloads mttcan; requires --jetson-orin-nano and stopped motors.'
  echo 'No motor frames, automatic recovery, services, or shell hooks.'
  echo 'An already correctly configured healthy interface is left untouched.'
}
jetson=false
recover=false
confirmed=false
channel=can0
channel_set=false
for arg in "$@"; do
  case "$arg" in
    -h|--help|-help) usage; exit 0 ;;
    --confirm-stopped) confirmed=true ;;
    --jetson-orin-nano) jetson=true ;;
    --recover-driver) recover=true ;;
    --*) usage >&2; exit 2 ;;
    *)
      if $channel_set || [[ ! $arg =~ ^[a-zA-Z0-9_-]{1,15}$ ]]; then usage >&2; exit 2; fi
      channel=$arg; channel_set=true ;;
  esac
done
if $recover && ! $jetson; then echo 'Driver recovery requires --jetson-orin-nano.' >&2; exit 2; fi
if [[ $EUID -ne 0 ]] || ! $confirmed; then usage >&2; exit 2; fi
if $jetson; then
  [[ $channel == can0 ]] || { echo 'Verified J17 setup supports can0 only.' >&2; exit 1; }
  model=$(tr -d '\0' </proc/device-tree/model)
  [[ $model == *'Orin Nano'* ]] || { echo "Unverified board: $model" >&2; exit 1; }
  command -v busybox >/dev/null
fi
owner_user=${SUDO_USER:-root}
owner_home=$(getent passwd "$owner_user" | cut -d: -f6)
owner_group=$(id -gn "$owner_user")
[[ -n $owner_home ]] || { echo 'Cannot resolve owner home' >&2; exit 1; }
install -d -o "$owner_user" -g "$owner_group" "$owner_home/.cache/handyflie"
lock="$owner_home/.cache/handyflie/$channel.lock"
if [[ ! -e $lock ]]; then install -o "$owner_user" -g "$owner_group" -m 600 /dev/null "$lock"; fi
exec 9>>"$lock"
flock -n 9 || { echo 'Close the controller before setup; this channel is owned.' >&2; exit 1; }
if $recover; then
  # Reloading a shared driver affects every bound controller, including down ports.
  # Refuse unless its only bound device is the verified Orin Nano CAN0 controller.
  python3 - <<'CHECK_RECOVERY'
from pathlib import Path
base=Path('/sys/bus/platform/drivers/mttcan')
bound=[p for p in base.iterdir() if p.is_symlink() and p.name!='module'] if base.exists() else []
if any(p.name!='c310000.mttcan' for p in bound):
    raise SystemExit('Refusing recovery: mttcan is bound to another controller.')
ports={p.name for device in bound for p in (device/'net').glob('*')}
if ports-{'can0'}:
    raise SystemExit('Refusing recovery: mttcan has other CAN interfaces.')
net=Path('/sys/class/net/can0')
if net.exists() and (net/'device/driver').resolve()!=base.resolve():
    raise SystemExit('Refusing recovery: can0 does not belong to mttcan.')
if Path('/sys/module/mttcan').exists() and not Path('/sys/module/mttcan/refcnt').exists():
    raise SystemExit('mttcan appears built into the kernel; module reload is unavailable.')
CHECK_RECOVERY
  trap 'echo "Recovery failed. CAN may be down; no motors were enabled or resumed." >&2' ERR
  echo 'Explicit CAN recovery: interrupting can0 and reloading mttcan once.'
  if [[ -e /sys/class/net/can0 ]]; then
    ip -details -statistics link show dev can0
    ip link set dev can0 down
  fi
  if [[ -d /sys/module/mttcan ]]; then modprobe -r mttcan; fi
fi
if $jetson; then
  modprobe can
  modprobe can_raw
  modprobe mttcan
fi
# Normal setup never resets an unhealthy controller. Recovery above is explicit.
state=$(ip -details -json link show dev "$channel" | python3 -c '
import json,sys
x=json.load(sys.stdin)[0]; link=x.get("linkinfo",{}); d=link.get("info_data",{})
if link.get("info_kind")!="can":sys.exit("Require an existing physical CAN interface")
if d.get("state") not in ("ERROR-ACTIVE","STOPPED"):
    sys.exit("CAN is unhealthy: "+str(d.get("state"))+". Inspect errors; setup will not reset this fault.")
normal=not set(d.get("ctrlmode",[])) & {"LOOPBACK","LISTEN-ONLY","FD","ONE-SHOT"}
ready=("UP" in x.get("flags",[]) and d.get("state")=="ERROR-ACTIVE"
       and d.get("bittiming",{}).get("bitrate")==1000000
       and d.get("restart_ms")==0 and normal
       and (sys.argv[1]!="true" or "BERR-REPORTING" in d.get("ctrlmode",[])))
print("ready" if ready else "configure")' "$jetson")
pins_ready=true
if $jetson; then
  tx_pin=$(busybox devmem 0x0c303010 32)
  rx_pin=$(busybox devmem 0x0c303018 32)
  if (( tx_pin != 0xc400 || rx_pin != 0xc458 )); then pins_ready=false; fi
fi
if [[ $state == ready ]] && $pins_ready; then
  echo 'CAN already ready; no reset or configuration change.'
else
  ip link set dev "$channel" down
  if $jetson && ! $pins_ready; then
    busybox devmem 0x0c303018 32 0xc458
    busybox devmem 0x0c303010 32 0xc400
  fi
  reporting=()
  if $jetson; then reporting=(berr-reporting on); fi
  ip link set dev "$channel" type can bitrate 1000000 loopback off listen-only off one-shot off fd off "${reporting[@]}" restart-ms 0
  ip link set dev "$channel" up
fi
ip -details -statistics link show dev "$channel"
# Report success only after re-reading the configured interface.
ip -details -json link show dev "$channel" | python3 -c '
import json,sys
x=json.load(sys.stdin)[0]; d=x.get("linkinfo",{}).get("info_data",{})
normal=not set(d.get("ctrlmode",[])) & {"LOOPBACK","LISTEN-ONLY","FD","ONE-SHOT"}
if not ("UP" in x.get("flags",[]) and d.get("state")=="ERROR-ACTIVE" and normal
        and d.get("bittiming",{}).get("bitrate")==1000000 and d.get("restart_ms")==0):
    sys.exit("CAN verification failed; inspect the state above. No automatic retry.")
if sys.argv[1]=="true" and ("BERR-REPORTING" not in d.get("ctrlmode",[]) or any(d.get("berr_counter",{}).values())):
    sys.exit("CAN recovery verification failed: error reporting missing or error counters nonzero.")
print("CAN ready at 1 Mbps. No motor commands sent; motor operation remains explicit.")' "$recover"
