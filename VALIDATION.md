# Validation — 2026-11-12

32 offline tests passed (5.46 seconds). Dependency validation passed; CLI help and
read-only status were checked, and both administrator scripts passed bash syntax
checks. The new package was tested offline using scripted I/O fixtures. No physical CAN
socket was opened by tests, no motor commands were transmitted, and no interface
setup/restart was executed by the assistant. The CLI status command is read-only.

Covered: codec identity/echo/type validation; ownership exclusion/release; explicit
activation; one receiving/sending worker across two motors; uncertain enable and
disable submission; startup timeout; stale and queued-old feedback; driver faults;
BUS-OFF error frames; control limits and holding; filter freshness and raw-state
separation; passive test silence; active test reuse and per-phase error attribution;
competing local sender exclusion; operator stop during diagnostics; JSONL history.

No claim is made that the new transport resolves the previously observed physical
CAN error increments. The phase reports are intended to localize those errors in
an operator-authorized hardware test. Stable 1 kHz timing is not hardware-validated.

Removal of pre-existing legacy boot services is an explicit administrator operation.

## Continuous joint streaming update

47 offline tests passed across `can_controller/tests` and
`motor_controller_beta_v2/tests` (8.11 seconds). Ten new streaming integration
checks cover explicit activation, configuration validation, latest-group
replacement, single-worker I/O, atomic group rejection, sequence ordering,
position slew and velocity constraints, hold/disable command timeouts, late
producer rejection, whole-group disable, Stop token invalidation, feedback-fault
priority and complete-cycle preflight before transmission. All use scripted
transport; no physical motor commands or CAN setup changes were made for this
update. Live multi-motor timing and mechanical behavior remain unverified.

## Signed limits update — 2026-11-13

59 offline core and GUI tests passed (8.62 seconds).
Signed degree configurations now use linear limits (−360 through +360). Added
offline coverage for asymmetric ranges, exact endpoints, invalid ranges, wrapped
alias rejection, full signed travel, streaming rejection, and out-of-range
measured-position warnings without automatic disable. No hardware commands sent.

## Go Zero recovery — 2026-11-13

62 offline tests passed (9.13 seconds). New coverage verifies +92° and −92°
recovery to zero, monotonic signed targets, runtime encoding outside the usual
position interval, holding at arrival, unchanged configured limits, ordinary
move rejection, and retained speed/gain/activation checks. No physical motor
commands were sent.

## Raspberry Pi 5 portability and repository packaging

The same Linux SocketCAN backend is used on both platforms. The generic manual
setup script configures an already-installed CAN interface without Jetson pinmux
or GPIO assumptions. Pi 5 wiring, adapter installation and physical timing remain
unverified; see docs/raspberry-pi-5.md. GitHub CI is configured for offline Python
3.10–3.13 testing; its remote results are not claimed until it has run.

## Shared CAN pipeline — 2026-11-18

110 offline tests passed in 32.80 seconds across core and beta V2 GUI tests.
GUI JavaScript syntax, CLI/GUI help, and whitespace/diff checks passed. No CAN
socket was opened on real hardware, no motor commands were sent, and no interface
reset or running GUI restart was performed by Codex.

New checks cover staged startup without periodic bursts; fresh probe qualification;
TX confirmation identity, missing/late confirmation and deadlines; optional absent
special-command replies; bounded TX-full handling; per-ID outstanding exchanges;
disable draining and cancellation of unsent targets; Stop during startup; cancelled
Set Zero/shutdown waiters; own echoes versus other local senders; RX overflow and
truncation; and the driver IFF_ECHO requirement. Repeated all-motor diagnostics use
the same session in forward/reverse order at 500/1000 Hz for two motors and 500/800
Hz for three motors. The existing bandwidth rejection at three motors/1000 Hz
remains tested. The reproduced second-motor-enable error stops before motor 3.

These are scripted transport and deterministic pipeline tests, not timing or
electrical validation. The 10 ms special-command response window and TX deadline
are host policies to validate on the actual driver/firmware. MIT replies cannot
uniquely acknowledge individual commands. A clean physical multi-motor run without
new CAN errors is still required before claiming the reported fault is resolved.

## Sequential startup / asynchronous runtime — 2026-11-18

The full 120-test offline core/GUI suite passed in 33.61 seconds. Subsequent
targeted checks passed for CLI timing configuration, GUI timing round-trip, and
partial streaming submission recovery with the latest target group (122 unique
tests across these runs). JavaScript syntax, CLI help and git diff whitespace
checks passed. No live CAN socket, motor command, reset or GUI restart was run.

Normal control now releases its per-ID TX slot on driver confirmation without
waiting for a motor reply. Independent RX/TX thread identity and lifecycle checks
passed. Tests demonstrate continuing monitoring/streaming during delayed feedback,
then the retained freshness fault; serialized startup; repeated queue-full samples
being dropped without stale replay or a 2T fault; independent motor progress;
complete-group application bookkeeping after partial submissions; and policy
validation/configuration. The receiver remains alive during shutdown drain.

TimingPolicy defaults and per-motor feedback timeout remain unvalidated host
policies. They are visible/configurable in the GUI, Python and CLI JSON, and
recorded in session/test logs. Real multi-motor timing, motion stability, optional
special-command response windows and the original ERROR-PASSIVE problem still
require explicit operator-controlled hardware validation.

## 2026-11-19 publication verification

130 offline core + GUI tests passed in 34.44 seconds on the Jetson's Python 3.12
virtual environment. No real CAN interface was opened by these tests and no motor
commands, setup or recovery were executed. Shell syntax and launcher/CLI help were
checked. This is software regression evidence, not a hardware reliability or
real-time qualification.

New RX regression tests cover timely kernel echoes processed late, genuinely late
or missing echoes, receiver-stall detection, bounded priority handling, explicit
inbox overflow, and reception while the Session state lock is held. Firmware
encoding defaults remain AK70-10 V2 ±25 Nm and AK80-6 V2 ±12 Nm. The named constants
are protocol scaling values, not user operating limits.

The receive checkpoint fix and recovery scripts still require a deliberate
hardware acceptance run. No claim is made that every observed multi-motor fault
has been resolved. Previous dated results above describe historical revisions.
