# CAN setup tools

| File | Purpose | Where to edit |
| --- | --- | --- |
| [setup_socketcan.sh](setup_socketcan.sh) | Sole privileged 1 Mbps setup path; optional explicit mttcan recovery | Board guard, pin registers, interface flags, final verification |
| [recover_can_desktop.sh](recover_can_desktop.sh) | Interactive stopped-system confirmation and sudo invocation | Operator prompts; keep hardware work in setup_socketcan.sh |
| [../Recover_can](../Recover_can) | Portable executable entry point from repository root | Path resolution only; no duplicated setup logic |

All scripts accept `--help`, `-h`, and `-help`. Use `bash script --help` to inspect
without touching CAN. See [Jetson instructions](../docs/jetson.md) before running.
Normal setup and recovery are separate deliberate actions. They install no
background tasks and transmit no motor commands.

Validate edits with `bash -n` and `python -m pytest tests/test_setup_tool.py -q`.
Setup tests substitute commands and filesystem fixtures; they do not reconfigure
the host. Never add a recovery loop or invoke setup automatically from Session.
