# GUI without Continuous API

This GUI has no continuous streaming controls or endpoints. The Python controller
library retains its streaming API; GUI motion uses discrete quintic requests.
Launch from this repository root.

# Controller beta V2

Optional browser GUI for `src/handycan`. The older beta GUIs are preserved and do not use this new backend.

Run on the Jetson, from any working directory:

```bash
.venv/bin/python -B Motor_Controller_beta_V2.py
```

Open <http://127.0.0.1:8767>. Use `--port` to change the port and `--help` for launch help. For another computer, forward this loopback port using SSH:

```bash
ssh -N -L 8767:127.0.0.1:8767 anirudh@JETSON_IP
```

Replace JETSON_IP with your Jetson's address, then open the same localhost URL on that computer.

## Operating sequence

1. Enter IDs and your mechanism's signed joint limits from motor zero, speed, torque and gain limits. Use −90..+90 for 90 degrees each way or −360..+360 for a full turn each way. A literal 0..360 only permits nonnegative targets. Apply configuration. This opens no CAN socket.
2. Inspect interface, then Connect. The interface must already be configured for normal classic CAN at 1 Mbps. Connect opens one shared session and transmits no motor commands.
3. With the joint supported, explicitly run CAN Test. It enables neutral polling, checks matching fresh motor replies and interface error counters, then attempts disable. Its phase report distinguishes communication from link quality; an interface being UP alone cannot pass the test.
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

63 offline tests pass across the core and GUI adapter/HTTP integration, including rejection of continuous-streaming GUI actions. They cover no implicit transmission, explicit activation, shared-session diagnostics, Set Zero/re-enable, stop bypassing the action mutex, command validation and HTTP request isolation. Scripted transport tests do not verify a physical motor or 1000 Hz performance. A headless browser also passed the full offline configuration/test/monitor/filter/zero/re-enable/stop workflow, page reload and 390 px mobile layout checks. The running hardware-mode page was separately checked while disconnected; no hardware actions were invoked.

Run offline checks:

```bash
.venv/bin/python -B -m pytest -q -p no:cacheprovider tests motor_controller_beta_v2/tests
```

The HTTP server binds only to loopback and checks Host, Origin and a per-process control token. It is intended for one trusted operator; an SSH tunnel is required for remote browser access. It is not a multi-user deployment or a certified industrial safety controller.

Signed-angle update: the main position readout and graph retain the signed angle; the additional single-turn readout wraps it to 0–360 degrees. Limit checks never wrap a requested target. For −90..+90 limits, −30 is accepted and +330 is rejected even though both display 330 degrees single-turn. Measured position outside the interval still only warns; other runtime faults retain their existing disable behavior. Reinitialize with your intended signed limits after restarting; existing numeric limits are not silently expanded.

Go Zero recovery: if externally moved past a position limit (for example +92°
with a +90° maximum), Go Zero follows the direct signed quintic path to 0°.
Its temporary trajectory corridor does not change configured limits. Speed,
gains, combined torque and fresh-feedback checks still apply. Normal Move
requests do not receive this exception.
