# Raspberry Pi 5

The controller uses Linux SocketCAN through python-can, not NVIDIA-specific
Python libraries. Pi 5 software portability is supported; physical Pi 5 motor
operation and timing have not been tested by this project.

## Hardware and OS

Use 64-bit Raspberry Pi OS Bookworm or newer, with Python 3.10+ and iproute2.
Connect a CAN controller plus transceiver: either a native SocketCAN USB-CAN
adapter, or a Pi-5-compatible SPI CAN HAT/module with its Linux driver configured.
The existing SN65HVD230 board is only a transceiver; connecting it directly to
Pi GPIO does not provide the CAN controller that the Jetson supplies internally.
A USB serial adapter using a proprietary protocol is not automatically SocketCAN.

For SPI hardware, follow that exact board vendor's Pi 5 instructions. Oscillator,
chip-select, interrupt GPIO, overlay and SPI speed depend on the board. Consult
the installed `/boot/firmware/overlays/README` (or your OS's equivalent) for the
kernel's `mcp2515`/`mcp251xfd` overlays. This repository deliberately does not
invent wiring or write a universal overlay. Reboot only if the hardware/overlay
installation requires it. Normal controller startup needs no reboot.

Never run `setup_jetson_can.sh` on a Pi. It is guarded for a specific Orin Nano
pinmux. Use the generic setup below after your Pi adapter appears as `can0`.

## Install from the repository root

```bash
sudo apt update
sudo apt install python3-venv python3-pip iproute2 can-utils
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[test]'
```

## Explicit link setup and test

Stop all motor-control programs and support the mechanism first. The setup
script checks that the interface is physical CAN and uses the controller's
cooperative channel lock. It does not alter startup files, overlays or GPIO and
sends no motor commands.

```bash
sudo bash tools/setup_socketcan.sh --confirm-stopped can0
.venv/bin/python -m handycan status --channel can0
```

If the adapter is `can1`, use that name both here and in your motor JSON config.
The configured link must support normal classic CAN at 1 Mbps and report
ERROR-ACTIVE. This package does not support FD motor frames. Some drivers expose
fewer error counters than others; a test cannot prove the absence of errors that
the driver does not report. Error-frame reporting can be enabled separately if
supported by your adapter; setup does not force an unsupported reporting mode.

Review `examples/motor.json` for your mechanism before the following active test.
It enables neutral zero-gain polling and attempts disable afterward; it releases
holding and is not a motion trajectory.

```bash
.venv/bin/python -m handycan test --config examples/motor.json --id 1 --active
.venv/bin/python -m handycan monitor --config examples/motor.json --id 1 --active --seconds 10
```

The same Session/JointTarget APIs and signed limits are used on Jetson and Pi.
USB/SPI latency, driver behavior and scheduler load affect achievable frequency;
1000 Hz is a requested rate, not a hard real-time promise. Inspect missed slots,
feedback age, CAN errors and transmission timing before increasing robot load.
Do not increase traffic beyond the library's aggregate bus budget.

## Offline checks

```bash
.venv/bin/python -m pytest -q
```

These use scripted transports, not physical CAN or a motor simulator runtime.
No CAN interface or powered motor is needed.

## Primary references

- [Raspberry Pi 5 specifications and supported OS](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [Official Raspberry Pi kernel overlay definitions](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/arch/arm/boot/dts/overlays/README)
- [python-can SocketCAN backend](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html)
- [Linux SocketCAN documentation](https://docs.kernel.org/networking/can.html)
