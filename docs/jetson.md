# Jetson Orin Nano: install, configure, recover

This setup script targets the project's Orin Nano native CAN0/J17 configuration.
It checks the board model before applying pin registers. Other carriers/JetPack
configurations need their own validated pinmux; do not bypass that check.
Use a 3.3 V logic-compatible CAN transceiver, appropriate CAN bus termination,
common signal reference, and motor power separately from Jetson logic power.
Verify the carrier's connector pinout before wiring.

## Install from a clone

```bash
git clone https://github.com/Anirudh-singh-Core/Cubemars_SocketCAN_Control.git
cd Cubemars_SocketCAN_Control
sudo apt-get update
sudo apt-get install python3-venv python3-pip iproute2 can-utils busybox kmod util-linux
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
```

Private repositories require your own GitHub credentials/access.

## Normal setup after boot

Stop/disconnect controllers and support all joints before interface changes.
The setup script does not disable motors for you.

```bash
sudo bash tools/setup_socketcan.sh --confirm-stopped --jetson-orin-nano can0
.venv/bin/python -m handycan status --channel can0
```

This loads CAN modules, configures the known pins and sets classic CAN to 1 Mbps.
An already matching healthy interface is left alone. It installs no boot service
or shell hook. ERROR-ACTIVE is the normal healthy CAN state; inspect error counters
as well. Interface readiness alone does not establish communication with motors.

Edit [examples/motor.json](../examples/motor.json) for actual IDs, model, signed
angle limits, speed, torque and gains. See the [configuration guide](../examples/README.md).
Do not run other CAN senders while a Session owns the channel.

## Explicit motor communication test

This next command **enables, polls and attempts to disable all configured motors**.
Zero gains do not hold a joint; support the mechanism. Neutral torque has a small
quantization offset, and firmware startup behavior must be validated on your hardware.

```bash
.venv/bin/python -m handycan test --config examples/motor.json --active
```

The terminal result is Pass/Fail; detailed evidence is saved to logs/can_test_latest.json.
Without `--active`, the test does not establish active motor feedback.

## Recover without rebooting the Jetson

Only after stopping/disabled motors, supporting joints and closing all controllers:

```bash
./Recover_can
```

Or `bash Recover_can` if your checkout lost executable permissions. It asks you
to type RECOVER, then uses sudo to reload mttcan once and reconfigure CAN0.
It refuses a conflicting session lease or an unsupported shared-driver layout.
It does not enable motors, launch the GUI, or automatically resume operation.
It works from any working directory when invoked by its full path.

Recovery clears interface state; it does not repair bad wiring, power, bitrate,
termination, or software faults. Keep the failed run's logs before recovery.
A built-in/non-unloadable mttcan driver cannot be recovered by this reload path.

## Editing and verification

Change defaults in `src/handycan/config.py`, protocol profiles in `protocol.py`,
and platform setup in `tools/setup_socketcan.sh`. The [source guide](../src/handycan/README.md)
explains constraints and relevant tests. Run `.venv/bin/python -m pytest -q` before
hardware testing. Offline tests never certify physical timing or motor safety.
