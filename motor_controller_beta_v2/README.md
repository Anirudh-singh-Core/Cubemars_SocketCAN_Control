# Controller beta V2

Optional browser GUI for `can_controller/src/handycan`. The older beta GUIs are preserved and do not use this new backend.

## Install and launch (GUI branch)

From the cloned `gui-without-continuous-api` branch, install the core first:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[test]'
.venv/bin/python -B Motor_Controller_beta_V2.py
```

From another working directory use absolute paths to that checkout's interpreter
and launcher. No machine-specific username or home directory is required.

Open <http://127.0.0.1:8767>. Use `--port` to change the port and `--help` for launch help. For another computer, forward this loopback port using SSH:

```bash
ssh -N -L 8767:127.0.0.1:8767 YOUR_USER@JETSON_IP
```

Replace JETSON_IP with your Jetson's address, then open the same localhost URL on that computer.

## Operating sequence

1. Add **all motor rows before connecting**, select each actual model (AK70-10 V2 or AK80-6 V2), and enter IDs and your mechanism's signed joint limits from motor zero, speed, torque and gain limits. Use −90..+90 for 90 degrees each way or −360..+360 for a full turn each way. A literal 0..360 only permits nonnegative targets. Apply configuration. This opens no CAN socket.
2. Inspect interface, then Connect. The interface must already be configured for normal classic CAN at 1 Mbps. Connect opens one shared session and transmits no motor commands.
3. With every joint supported, explicitly run CAN Test for all configured motors individually and together. It enables neutral polling, checks matching fresh motor replies and interface error counters, then attempts disable. Its phase report distinguishes communication from link quality; an interface being UP alone cannot pass the test.
4. Explicitly Enable / Monitor for live feedback. Enter target degrees, duration and gains for Move & hold or Go Zero. Motion requests require confirmation and validated limits. Targets are signed absolute angles; −30° displays as 330°. Arrival holds the target until Disable or Stop.
5. Disable before Set current position as zero. This keeps initialization and numeric limits, clears filter state, and permits re-enabling without restarting configuration.

CAN Test is available only while every motor is disabled. The current core enforces interface health and fresh feedback for control; it does not require a previous CAN Test PASS to enable monitoring. Do not interpret Connect as verified motor communication.

Stop All is available independently of other GUI actions. It requests disable through the session; delivery cannot be guaranteed across a failed physical bus. Browser heartbeat is disabled as requested: closing a browser tab does not stop monitoring or holding. Shutting down the server attempts to disable active motors and closes the session.

## Structure and data

`Motor_Controller_beta_V2.py` → `motor_controller_beta_v2/web.py` → `app.py` → `handycan.Session` → one `python-can` SocketCAN bus. The session owns RX, TX scheduling, limits, watchdogs and fault handling. CAN Test receives that same session; it never opens a separate socket. The GUI cannot run terminal CAN setup or automatically reconnect/resume a faulted session.

HTTP status updates are approximately 5 Hz and only read cached feedback. They create no additional motor polls. The session independently targets the configured control rate (default 1000 Hz); this is best-effort Python/Linux scheduling, not a hard real-time guarantee. Missed deadlines and maximum transmission intervals are visible.

The live graph uses the existing shared `graph_logger.browser_plot_javascript` renderer. It retains about 60 seconds of browser-sampled data; it is not a full-rate trace. The session journal separately queues command, frame and feedback history to `diagnostic_logs/beta_V2/*.jsonl`. Active CAN Test reports are saved alongside it as `*.can-test.json`. Dropped log records and write errors are displayed, so completeness is not assumed. Existing recordings are not removed.

Raw, low-pass, adaptive EMA and scalar Kalman feedback are available. Filter activation requires a stationary variance supplied by the user and explicit tuning; there is no automatic filter recommendation. Filters affect selected API/display feedback, while raw feedback remains the source for motion and safety checks. The old encoder sweep and filter-comparison workflows are not yet ported into this GUI and must not run alongside the connected session.

## Validation

Core and GUI integration tests use scripted transport and send no hardware frames.
Run from the GUI branch root after installation:

```bash
.venv/bin/python -B -m pytest -q -p no:cacheprovider tests motor_controller_beta_v2/tests
```

See the root VALIDATION.md for the latest local results and their limits.

The HTTP server binds only to loopback and checks Host, Origin and a per-process control token. It is intended for one trusted operator; an SSH tunnel is required for remote browser access. It is not a multi-user deployment or a certified industrial safety controller.

Signed-angle update: the main position readout and graph retain the signed angle; the additional single-turn readout wraps it to 0–360 degrees. Limit checks never wrap a requested target. For −90..+90 limits, −30 is accepted and +330 is rejected even though both display 330 degrees single-turn. Measured position outside the interval still only warns; other runtime faults retain their existing disable behavior. Reinitialize with your intended signed limits after restarting; existing numeric limits are not silently expanded.

Go Zero recovery: if externally moved past a position limit (for example +92°
with a +90° maximum), Go Zero follows the direct signed quintic path to 0°.
Its temporary trajectory corridor does not change configured limits. Speed,
gains, combined torque and fresh-feedback checks still apply. Normal Move
requests do not receive this exception.


There is one manual setup entry point:
`sudo bash tools/setup_socketcan.sh --confirm-stopped --jetson-orin-nano can0`.
Run it after boot with controllers stopped. It leaves an already matching healthy
interface untouched; normal setup does not reload mttcan or clear a CAN fault.
For deliberate stopped-system recovery, use the desktop **Recover CAN** launcher
or add `--recover-driver` to that command. Recovery does not resume the GUI or motors.
There is no prerequisite to enable/disable motor 1 before configuring other motors.
After disconnect, applying a different motor list creates a fresh session at the
next Connect, without running CAN setup. Offline regression tests cover direct
three-motor connection, motor 3 enabled first, reversed configuration order, and
one-motor enable/disable/disconnect followed by three-motor configuration.
These tests do not establish the physical cause of the observed CAN bit errors.
Session-open logs include the actual motor profiles, limits, rate, and interface
snapshot so a later run can be compared without guessing which settings it used.

## Sequential startup + asynchronous operation — 2026-11-18

Restart the Python GUI process to load this update; refreshing the browser alone
will not replace the running backend. Connect remains read-only. Startup enables
and verifies each motor sequentially. Normal monitoring, quintic control and
streaming use scheduled TX with an independent RX worker on the same python-can
socket. A delayed motor reply does not gate the next command; per-motor feedback
freshness is supervised separately. One unconfirmed TX per ID bounds backlog.

The fatal two-period unsent-control deadline has been removed. A temporary TX-full
result skips that sample; the next tick computes the current target. Actual TX
failure, TX-confirmation timeout, stale feedback and driver faults still fault the
session. CAN Test sequentially tests IDs and then exercises this asynchronous
operating path together. No automatic reset/reconnect or boot setup was added.

**Initialize → Communication timing** exposes all TimingPolicy fields in ms.
Each motor row exposes **Feedback-loss timeout** in seconds. These defaults are
host policies, not validated motor/Jetson specifications. Disconnect before edits,
then Apply configuration and Connect. Settings appear in session/test logs.
The core README lists defaults, effects, validation constraints and the separate
caller-specified streaming command watchdog. Browser heartbeat remains disabled.

For two motors, test 500 Hz before 1000 Hz. For three, test 500 Hz before 800 Hz;
the existing 80% conservative bandwidth guard still rejects three at 1000 Hz.
No physical multi-motor validation was performed for this change.

## Edit the GUI

| File | Responsibility / edit points |
| --- | --- |
| `Motor_Controller_beta_V2.py` | Entry point, CLI help, port and orderly shutdown |
| `motor_controller_beta_v2/app.py` | GUI actions → shared Session; configuration, diagnostics, cached status |
| `motor_controller_beta_v2/web.py` | Loopback HTTP server, token/origin validation and static routes |
| `motor_controller_beta_v2/static/index.html` | Motor rows/model selector, controls, units, plots and cached-status refresh |
| `graph_logger.py` | Shared browser plotting renderer; core full-rate history uses handycan.journal |
| `motor_controller_beta_v2/tests/test_gui.py` | Offline adapter and HTTP integration tests |

Communication uses one shared python-can/Linux SocketCAN socket, sequential
verified startup, then independent TX scheduling and RX capture. The RX inbox
avoids waiting on GUI/controller state locks. This is threaded asynchronous I/O;
GUI HTTP calls are not CAN sockets. The GUI does not expose the continuous target
streaming API, although the core library provides it to Python callers.

After Python edits, stop the server with joints supported and restart it;
refreshing the browser does not reload the backend. Use the root `Recover_can`
launcher only for deliberate stopped-system Jetson recovery, not ordinary Connect.
