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
