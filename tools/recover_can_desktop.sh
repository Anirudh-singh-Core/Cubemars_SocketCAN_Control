#!/usr/bin/env bash
# Interactive desktop entry point; all privileged CAN work stays in setup_socketcan.sh.
set -u
if [[ ${1:-} == -h || ${1:-} == --help || ${1:-} == -help ]]; then
  echo 'Usage: bash recover_can_desktop.sh'
  echo 'Confirm stopped/supported motors, authenticate with sudo, reload Jetson CAN0 once.'
  echo 'Does not launch/enable motors, reboot, or install background tasks.'
  exit 0
fi
if [[ $# -ne 0 ]]; then echo 'Unknown argument; use --help.' >&2; exit 2; fi
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
printf '%s\n' 'Recover Jetson CAN0 without reboot' '' \
  'Support all joints and ensure motors are stopped/disabled.' \
  'Disconnect the GUI session and close every other CAN sender first.' \
  'This interrupts CAN and reloads its driver. It does not disable a powered motor for you.' \
  'No motor will be enabled or automatically resumed.' ''
read -r -p 'Type RECOVER to confirm the motors are stopped and supported: ' answer || exit 1
if [[ $answer != RECOVER ]]; then echo 'Cancelled; CAN unchanged.'; exit 0; fi
sudo bash "$script_dir/setup_socketcan.sh" --confirm-stopped --jetson-orin-nano --recover-driver can0
result=$?
if [[ $result -eq 0 ]]; then
  printf '\n%s\n' 'Recovery complete. In beta V2, connect explicitly when ready.' \
    'This clears CAN state; it does not prove the underlying communication fault is fixed.'
else
  printf '\n%s\n' 'Recovery did not complete. Read the error above; no automatic retry.'
fi
read -r -p 'Press Enter to close this window. ' _ || true
exit "$result"
