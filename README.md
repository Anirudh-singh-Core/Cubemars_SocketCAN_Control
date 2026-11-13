# GUI beta V2 — without Continuous API

This branch adds the optional browser GUI. **The GUI does not expose continuous
joint-target streaming** (`start_stream`, `stream_targets`, `end_stream`). It
supports Enable/Monitor, discrete quintic Move & hold, Go Zero recovery, Set Zero,
filters, shared-session CAN Test and diagnostics. The underlying controller library
retains its existing streaming API for Python callers; no library functionality
has been removed. The controller-only `main` branch remains unchanged.

## Run this branch

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -B Motor_Controller_beta_V2.py
```

Open <http://127.0.0.1:8767>. Startup is disconnected and sends no motor commands.
Use your platform's explicit CAN setup first; see [Pi 5 setup](docs/raspberry-pi-5.md).
For remote access use `ssh -N -L 8767:127.0.0.1:8767 USER@DEVICE_IP` and open that
localhost URL on your computer. No browser heartbeat is enabled: closing the
page does not stop an active motor. Use STOP ALL before leaving.

[GUI instructions](motor_controller_beta_v2/README.md)

---

# HandyCAN

A small, hardware-only controller library for **CubeMars AK70-10 V2 KV100 in MIT
mode**, using `python-can` and Linux SocketCAN at **1 Mbps**. This repository contains the controller library, CLI, offline tests and setup
guides. The controller library does not require the optional GUI supplied on this branch.

The core has no GUI dependency, simulator runtime, automatic filter selection, shell startup hooks,
background boot installation, implicit motor enable, or automatic reconnect.
Importing the library sends nothing. `Session.open()` validates and opens CAN but
sends nothing. Hardware activation always requires an explicit API call or
`--active` in the CLI.

## Structure

| Module | Responsibility |
| --- | --- |
| `interface.py` | Read-only interface status and readiness checks |
| `transport.py` | Cooperative channel lease and persistent `python-can.Bus` |
| `protocol.py` | Pure MIT encoding/decoding, including feedback identity checks |
| `session.py` | One I/O worker, transmission schedule, motor states, stop/health supervision |
| `limits.py`, `motion.py` | Caller-defined limits and elapsed-time quintic profiles |
| `streaming.py` | Immutable degree-based joint target samples for external planners |
| `diagnostics.py` | CAN test through the existing session, with per-phase error evidence |
| `filters.py` | Existing low-pass, Adaptive EMA, and scalar Kalman math |
| `journal.py` | Bounded asynchronous JSONL history; disk I/O outside the CAN worker |
| `cli.py` | Explicit status, test, and monitoring commands |

One receiver reads the bus. The same worker sends all motor commands and routes
feedback by CAN ID. CAN Test borrows the session; it never opens a second bus.
An operation lock excludes diagnostics from public enable/move/zero operations.
Stop bypasses that lock and the normal command queue. No `Notifier` is used:
the worker is the sole `recv()` consumer, preventing competing readers.

A same-user channel lock is acquired once per session, at
`~/.cache/handyflie/can0.lock`, compatible with the previous beta's lock. It is
cooperative: `cansend`, old standalone scripts, and other Linux users can bypass
it. Never run another command sender at the same time. `candump` can observe.

## Install

Python 3.10+ on Linux; the tested dependency is python-can 4.6.1.
From this directory:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
```

If your OS lacks `venv`/`ensurepip`, install its Python venv package first.
Use the virtual environment interpreter for all commands; no global `can` alias
or system Python replacement is installed.

For Raspberry Pi 5, follow the [hardware and setup guide](docs/raspberry-pi-5.md).
An existing physical SocketCAN interface on either board can be configured with
`sudo bash tools/setup_socketcan.sh --confirm-stopped can0`. This does not install
a hardware driver or change pinmux.

## Remove the previous boot configuration (once)

Stop motor programs and secure the mechanism. Review and run:

```bash
sudo bash tools/remove_legacy_startup.sh --confirm-stopped
```

This removes only the recognized `jetson-can0.service` and
`/etc/modules-load.d/jetson-can.conf`. Stopping the service brings can0 down.
It does not uninstall kernel drivers or edit unrelated shell configuration.
The old user-owned `~/.local/bin/can` helper was backed up and removed during
migration. No CAN entries were found in the inspected shell startup files.

Only use this migration helper if you previously installed that exact legacy
service. The Python package itself does not install or enable a service.

## Explicit CAN setup after boot

For the verified **Jetson Orin Nano / J17** wiring in this project only:

```bash
sudo bash tools/setup_jetson_can.sh --confirm-stopped
```

This checks the board model, acquires the channel lease, loads the modules,
configures the known pin registers, and brings can0 up at 1 Mbps. No motor frame
is sent. It is manual and idempotent; never run it during control. Other boards
need their own validated pinmux configuration. BUS-OFF recovery is also explicit;
no automatic motor resume is implemented. Opening a terminal does not run setup.

## Status, communication test, monitor

Commands are shown from this directory with `.venv/bin/python`; substitute your
environment's Python. All public commands support `-h`, `-help`, and `--help`.

```bash
.venv/bin/python -m handycan status --channel can0
.venv/bin/python -m handycan test --config examples/motor.json --id 1
```

The default test is receive-only and fails if it cannot observe existing matching
external feedback. It does not poll a disabled motor.

**Review the sample motor limits for your mechanism before active use.** The
numbers in `examples/motor.json` are examples, not established safe limits.

```bash
.venv/bin/python -m handycan test --config examples/motor.json --id 1 --active
.venv/bin/python -m handycan monitor --config examples/motor.json --id 1 --active --seconds 10
```

Both active commands enable the motor with zero gains and nominal zero torque.
That releases holding and has MIT quantization residuals; support the mechanism.
They attempt disable on completion/exception/Ctrl+C. The monitor prints at 20 Hz;
CAN polling targets the configured control rate (1000 Hz in the example).

The test prints only Pass/Fail. Details overwrite `logs/can_test_latest.json`;
the session history logs every phase. A test does not grant permanent proof of
health and is not a mandatory stored prerequisite for a new session. Every open
validates the interface, and enabled operation supervises fresh feedback.

## Reusable API

`Session` is the common owner used by higher-level applications. Configure one
`Motor` per ID. `Motor.from_degrees(...)` requires explicit joint bounds, speed,
torque, Kp and Kd ceilings. The underlying `MotorLimits` accepts signed bounds and
optional current estimates with a caller-verified output torque constant.

| API | Behavior |
| --- | --- |
| `Session(motors, channel='can0', control_hz=1000)` | Construct only; no hardware or worker startup |
| `open()` / context manager | Validate, lease, open one bus, start worker and read-only health monitor |
| `enable(id)` | Stage neutral command, submit FC, await feedback in worker; return indicates submission only |
| `wait_for_feedback(id, timeout=1)` | Wait for fresh feedback without additional transmissions |
| `get_encoder_feedback(id, source='raw')` | Fresh timestamped feedback in degrees; raises if stale/missing/faulted |
| `move(id, target_deg, duration, kp=..., kd=..., ff_torque=...)` | Validate raw start/path, execute quintic, hold final target |
| `go_zero(id, duration, kp=..., kd=...)` | Direct signed quintic recovery to absolute zero, even from outside joint limits, then hold |
| `set_zero(id, confirm=True)` | Requires disabled motor; submit FE, clear feedback and filters; retain numeric limits relative to new zero |
| `disable(id)` | Submit FD; no claim of motor execution confirmation |
| `stop()` | Cancel queued commands and attempt FD for active motors |
| `snapshot()` | States, feedback ages, fault, timing and logging counters |
| `can_test(session, id, active=False)` | Exclusive observation or explicit neutral polling test through this session |
| `close()` | Stop workers, attempt disable of active motors, close bus, release lease, flush history |

Motion input is signed absolute degrees in [-360, 360]. -30 means counterclockwise
from motor zero and is displayed as 330; switching -30 to +330 can request one
full turn. Joint limits are signed linear intervals from motor zero: −90..+90
allows 90 degrees each way; −360..+360 allows a full turn each way. A literal
0..360 interval permits only nonnegative targets. No modulo equivalence is used
for limit checks: with −90..+90 limits, −30 is valid and +330 is rejected. The
backend retains signed radians; only `position_single_turn_deg` wraps to [0,360).
Quintic paths run through the signed interval, without a shortest-angle shortcut.
Moving from −360 to +360 is two revolutions of travel, not a zero-length move. Motion uses existing nominal and quantized command checks. The old
allowance of one CAN position count at encoded boundaries is preserved.
Externally moving the measured joint outside its position interval sets a warning;
position alone does not disable. Other measured limits remain enforced.

This first fresh package includes CAN diagnostics and motion, but does not port
the old GUI, simulation, autonomous filter ranking, or dynamic sweep CLI. Existing
recordings and the old sweep remain preserved in the parent workspace; do not run
old transmitters concurrently with a session. Their previous CAN ownership rules
are not a substitute for this session.

## Filtered encoder output

Configure the filter explicitly, supplying verified stationary variance in raw
MIT CAN counts squared (for example from a previously reviewed sweep recording):

- `set_filter(id, 'lowpass', rest_variance_counts_squared=..., cutoff=...)`
- `set_filter(id, 'adaptive_ema', rest_variance_counts_squared=..., min_hz=..., max_hz=...)`
- `set_filter(id, 'kalman', rest_variance_counts_squared=..., q_per_second=...)`

Then call `get_encoder_feedback(id, source='kalman')` (or the selected name).
Outputs remain invalid until a new feedback sample is processed. Source must
match the configured filter; there is no silent fallback. States are independent
per motor and reset on enable; Set Zero clears calibration. Calibration supplied
by a caller must belong to that motor and reference; this API validates the number,
not the provenance of a recording.

Position alone is filtered; velocity and torque remain the received measurements.
Raw position is included alongside filtered position. Filtering occurs before
single-turn wrapping. The hardware encoder is 14-bit; MIT's transmitted position
field remains 16-bit over ±12.5 rad. No bit shifting, fabricated encoder counts,
or altered CAN scale is applied. Control limits and trajectories use raw feedback.
No filter winners or performance conclusions are generated.

## Errors and timing

- Startup requires UP, normal classic CAN, 1 Mbps, ERROR-ACTIVE.
- No initial feedback within 1 second, stale feedback after the configured timeout,
  driver fault byte, detected competing local transmitter, BUS-OFF, transport
  failure, runtime limit failure, or scheduler stall faults the session.
- Faults stop further scheduled commands and attempt disable of active motors.
  FD delivery cannot be guaranteed over a broken bus. Uncertain send failure
  is never reported as a confirmed disabled motor.
- Transient CAN error frames are logged; the independent interface inspector
  checks every 0.5 s with a bounded subprocess timeout. CAN Test fails on new
  errors even if valid motor replies continue.
- Faults require closing the session, resolving the cause and explicitly opening
  another session; there is no automatic reconnect or replay of motor commands.
- There is no browser heartbeat. Motor feedback supervision is always active.
- Scheduling uses monotonic absolute deadlines and skips missed slots rather than
  sending bursts. A stall above max(50 ms, 3 periods) triggers a fault.
- Feedback contains both the kernel wall-clock timestamp and local monotonic time.
  Old queued frames are rejected. Timestamp conversion assumes stable system time;
  system clock changes can affect timestamp validation.
- Logging is bounded (20,000 queued events by default); overload increments a
  visible dropped counter instead of blocking control. Disk failures are exposed
  in `snapshot()['logging']`. Unlimited lossless history is not promised.
- `1000 Hz` is a **best-effort target**, not hard real time. Timing statistics
  describe software scheduling/submission, not independently measured wire timing.
  CAN arbitration, hardware errors, OS scheduling and Python runtime still matter.
- Aggregate requested traffic is checked against a conservative 80% budget.

## Tests

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -p no:cacheprovider
```

Tests use scripted transport fixtures and patched python-can construction, never
SocketCAN hardware. These are test doubles, not a distributed motor simulator.
Tests do not validate the physical wiring, loaded timing, actual torque, or motor
response. Hardware tests must be initiated explicitly by the operator.

## Origin and references

MIT encoding, software limits and filter math were carried forward from this
project's existing working files; the new communication and orchestration layer
is independent of the legacy code. See the official
[python-can SocketCAN backend](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html)
and [Linux SocketCAN documentation](https://docs.kernel.org/networking/can.html).
No external robotics driver code was copied. No open-source license has been selected yet; publication alone does not grant
reuse rights. Choose a license and review ownership before an open-source release.


## Continuous joint-target streaming (IK / outer controllers)

Import `JointTarget` and use `Session.start_stream`, `stream_targets`, and
`end_stream`. Keep one session open for the full operation. This uses the existing
CAN worker and bus, with no additional receiver, socket, or GUI dependency.

`JointTarget(position_deg, velocity_deg_s, kp, kd, ff_torque=0)` is immutable.
Position is signed absolute motor-zero degrees (−360..360), velocity is degrees/s,
gains retain MIT units (Nm/rad, Nm·s/rad), and feedforward torque is Nm. Kp must be
positive for this position-streaming API. It does not solve IK or generate a
trajectory: the higher-level planner supplies smooth position/velocity samples.

API:

```python
from handycan import Session, JointTarget

# session is already open, and all selected motors have been explicitly enabled
# and have fresh feedback. initial_targets is {motor_id: JointTarget(...)}.
token = session.start_stream(
    initial_targets,
    timeout=0.1,              # example: maximum accepted-update age, seconds
    max_step_deg=0.5,         # example: maximum change from last applied target
    timeout_action="disable", # REQUIRED choice: "disable" or "hold"
)

# Inside your paced planner loop, supply EVERY member of the same group.
# planned_targets maps each motor ID to a JointTarget from your planner.
session.stream_targets(token, planned_targets, sequence=1)
# Next iteration uses sequence=2, then 3, etc. No time.sleep is added by this API.

# At the end, choose explicitly:
session.end_stream(token, action="hold")     # or action="disable"
```

The numeric timeout/step values above are examples to tune for the planner,
mechanism and requested update rate. The timeout must be at least two control
periods and at most one second. Group entry requires MONITORING or HOLDING motors
and a first target close to their fresh measured positions. It never enables a
motor. Start with measured signed positions and zero desired velocities, then
advance the planner. A MOVING motor must finish its existing quintic first.

Only one streaming group is allowed per session. Other axes may continue their
existing operation, subject to the session's aggregate traffic budget. The group
cannot be changed midstream; end it and explicitly start another group.

### Scheduling, admission and synchronization

- `start_stream` returns a unique token and starts at sequence 0. Subsequent
  sequences must be strictly increasing integers. Old tokens cannot resume a
  stopped, timed-out or replaced stream.
- `stream_targets` copies and validates the complete group under the session lock.
  It replaces one pending sample; it does not enqueue a history of samples and
  does not wait for transmission. Obsolete pending updates are counted.
- All members must pass identity, fresh raw feedback, signed joint limits, velocity, gains,
  feedforward and estimated combined MIT torque checks before acceptance. Target
  steps are bounded by `max_step_deg` and by configured velocity times elapsed
  time since the last applied sample (minimum one control period, plus the
  existing one-CAN-count boundary tolerance). There are no acceleration/jerk
  limits in this API; the planner must enforce those requirements.
- Pending replacements are checked against the last applied sample, not an
  intermediate discarded sample, so bursts cannot bypass the step limit.
  Every cycle rechecks command limits against current raw feedback for the
  complete cycle before any frame is sent.
- The CAN worker uses the newest accepted group at its next deadline and repeats
  those position/velocity/gain/torque setpoints until a newer sample arrives or
  the command watchdog expires. It does not extrapolate or interpolate between
  planner samples. Nonzero desired velocity is an impedance target, not a
  position integrator. A lower-rate planner must provide appropriately spaced
  samples or implement interpolation upstream.
- Group commands share a software control cycle, but classic CAN transmits frames
  sequentially. This is not simultaneous motor execution, and there is no motor
  execution acknowledgement. A transport failure can occur after only part of
  the group was sent; the session faults and attempts disable.
- Acceptance returns `execution_verified=False`. `snapshot()['stream']` exposes
  accepted/applied sequences, replaced updates and command age. Applied means all
  frames were submitted to python-can, not independently verified on the wire.
  JSONL logs include accepted target groups, applied sequences and stream endings.

### Ending and fault policy

The command watchdog measures time since the latest accepted group. Rejected
updates do not refresh it; even a late producer arriving before the worker next
runs cannot revive an expired stream. This is independent of browser heartbeat
and motor feedback supervision.

- `timeout_action="disable"`: expiration faults the session and attempts to
  disable all active session motors. Resolve the cause and reopen explicitly.
- `timeout_action="hold"`: expiration ends the stream and holds its last applied
  group positions with zero desired velocity, retaining each target's gains and
  feedforward torque. Pending samples are discarded. Hold commands must still
  pass limits and fresh-feedback checks. Hold failure faults and attempts disable.
  Hold is an immediate change of setpoints, not a planned braking trajectory.
- `end_stream(..., action="hold")` uses the same hold behavior; `"disable"`
  attempts disable for every group member. Holding before any sample was applied
  is rejected; disable is still available.
- Disabling a single streaming member ends and disables the entire group, avoiding
  an unintentionally partially controlled group. Other monitored axes are not
  disabled by a successful explicit group end.
- Stop All, close, stale motor feedback, driver faults and transport faults cancel
  the stream. Feedback/safety faults override the selected hold-on-command-timeout
  policy. Invalid caller samples raise without replacing the accepted sample;
  the existing sample remains subject to its original deadline.

Use `with Session(...) as session:` or `try/finally: session.close()` around the
higher-level controller so exceptions attempt disable and release ownership.
Disconnect the GUI before a standalone controller opens this channel. Streaming
is a Python API; there is no new GUI button or CLI motion command. Restart a
long-running Python application to import the updated library. Aggregate CAN
traffic limits still apply; 1000 Hz per joint is not feasible for arbitrary motor
counts and is never a hard real-time guarantee.

### Go Zero recovery outside joint limits

`go_zero` is a dedicated recovery operation: +92° returns directly through +90°
to 0°, and −92° returns through −90° to 0°. It uses the fresh measured signed
position, never the wrapped display or a shortest-angle shortcut. Only this
trajectory is allowed a temporary position corridor between its starting point
and zero; configured limits are unchanged. Zero may be reached even if it is
outside the configured position interval. Ordinary `move(..., 0, ...)` and
streaming retain normal position-limit enforcement.

Recovery requires an enabled MONITORING/HOLDING motor, fresh valid feedback,
valid gains, a duration consistent with velocity limits, and valid runtime
combined-torque checks. These checks are not bypassed; a feedback/torque fault
can still stop recovery. The generated position targets progress toward zero
and hold at arrival; this cannot guarantee physical direction against external
forces or momentum. Recovery does not start while another trajectory or stream
is running. Measured out-of-range position alone remains a warning.
