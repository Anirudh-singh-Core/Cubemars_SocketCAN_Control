# Controller source: file-by-file editing guide

The public API is synchronous Python methods backed by independent TX/RX threads.
Calls such as `enable()` wait for a bounded operation result; normal monitoring
and motion then run asynchronously. This package does not use asyncio.

## Communication ownership

`Session` owns one `Transport`, which opens one python-can SocketCAN bus/socket.
Only `Session._receive_loop` reads it. It captures frames into `ReceiveInbox`
without taking the Session state lock, decoding feedback, or writing logs.
The TX/state worker consumes confirmations/errors first, then feedback, before
advancing `Pipeline` and generating current targets. All sends go through that
worker. CAN Test borrows this same session. HTTP status reads cached state only.

The cooperative lock excludes another participating session. Linux tools can
bypass it; do not run another CAN sender concurrently. One socket does not add
wire traffic. Driver echoes are local; motor feedback occupies the physical bus.

## Configuration and public entry points

| File | Main interfaces and behavior | Edit here for | Check afterwards |
| --- | --- | --- | --- |
| [__init__.py](__init__.py) | Public exports | Exposing an existing API | Import/API tests |
| [__main__.py](__main__.py) | `python -m handycan` entry | CLI delegation only | `python -m handycan --help` |
| [cli.py](cli.py) | `load_config`, `main`; status/test/monitor | JSON parsing, explicit CLI actions and help | CLI/core tests; help without hardware |
| [config.py](config.py) | `Motor.from_degrees`, `TimingPolicy`, `validate` | Configuration validation, timing defaults, bus budget | Core and pipeline tests |
| [session.py](session.py) | `Session`; enable/disable/zero/move/stream/filter/snapshot | State transitions and controller scheduling | Entire offline suite |
| [errors.py](errors.py) | Typed exceptions | Distinguishing failures at API boundaries | Error-path tests |

The caller supplies operating joint/speed/torque/gain limits. The GUI exposes
TimingPolicy in milliseconds; Python/JSON use seconds. Changing a timing default
requires documenting why; longer deadlines do not repair a congested or faulty bus.

## Wire protocol and transport

| File | Main interfaces and behavior | Edit here for | Check afterwards |
| --- | --- | --- | --- |
| [protocol.py](protocol.py) | Model constants, `MODEL_RANGES`, `encode_mit_command`, `decode`, special frames | Verified firmware packing/scaling and new motor models | Codec endpoints, model round trips, malformed frames |
| [transport.py](transport.py) | `ChannelLease`, `Transport`, `decode_datagram`, `TxConfirmation` | Socket setup, raw receive metadata, local echo/error handling | Transport tests; no extra receive consumer |
| [receive.py](receive.py) | `ReceiveInbox`, `Received` | Bounded capture queue, priority selection, socket drain checkpoints | test_receive.py |
| [pipeline.py](pipeline.py) | `Pipeline`, `Exchange` | Ordered startup/zero/disable, one outstanding TX per ID | test_pipeline.py and test_receive.py |
| [interface.py](interface.py) | `inspect_interface`, `require_ready` | Read-only Linux interface readiness checks | Core/setup tests |

### Add or edit a motor profile

1. Verify the exact firmware's MIT field sizes, position/velocity/torque and gain
   ranges, feedback layout and IDs. Do not substitute rated/peak torque for an
   encoding range.
2. In `protocol.py`, add a named model section with maximum velocity (rad/s) and
   torque (Nm), then register it in `MODEL_RANGES`. Update the unknown-model error.
3. If position/gain ranges or feedback layouts differ, extend the codec/profile
   explicitly; adding only a torque tuple is insufficient.
4. Add endpoint/round-trip/invalid-frame tests. Set `model` in the JSON or
   `Motor.from_degrees`. On the GUI branch also update the model dropdown.

Existing encoding maxima remain 25 Nm (AK70-10 V2) and 12 Nm (AK80-6 V2).
Use caller `torque_max` to restrict operation; it cannot exceed what the codec can
represent. Changing an encoding constant scales both commands and decoded feedback.

### Enable / Monitor is not a position hold

Startup sends neutral MIT → enable FC → neutral feedback probe, with TX
confirmation and bounded special-response windows. Startup is serialized across
IDs. Monitoring sends neutral MIT frames at the scheduled rate. Kp/Kd are zero;
nominal zero torque encodes slightly negative because zero lies between integer
counts. No current-angle hold is requested. Support joints under external load;
firmware startup behavior still requires hardware validation. TX confirmation
proves local transmission completion, not execution by a particular motor.

## Motion, diagnostics and logging

| File | Main interfaces and behavior | Edit here for | Check afterwards |
| --- | --- | --- | --- |
| [limits.py](limits.py) | `MotorLimits`, `MotorFeedback`, command/feedback checks | Signed boundaries, combined torque checks, quantization tolerance | Signed limits and zero recovery tests |
| [motion.py](motion.py) | `Trajectory.sample` | Elapsed-time quintic target generation | Trajectory/limits tests; no assumed fixed timestep |
| [streaming.py](streaming.py) | `JointTarget` | Immutable planner samples and degree conversion | Streaming tests |
| [filters.py](filters.py) | LowPass, AdaptiveEMA, Kalman1D | Explicit filter math/tuning | Filter tests; raw feedback stays in motion checks |
| [diagnostics.py](diagnostics.py) | `can_test` | Per-ID and combined phases, evidence and Pass/Fail criteria | Core/pipeline/GUI diagnostic tests |
| [journal.py](journal.py) | `Journal.record`, `close` | Bounded queued JSONL history | Verify history, drops/errors and shutdown behavior |

Go Zero has a temporary direct signed corridor from an out-of-limits measured
position to zero; it does not expand normal motion limits. Position excursions
alone warn, whereas other configured runtime faults may request disable. Streaming
keeps latest targets rather than replaying an old queue; watchdog behavior is
explicit at stream start. There is no atomic simultaneous multi-motor CAN frame.

The logger writes from a background thread. Its bounded queue can drop records;
`logging.dropped` and `logging.error` must remain visible. Do not claim complete
history when either indicates loss. TX-confirmed records distinguish kernel TX,
RX capture and state-worker dispatch delays.

## Verify edits

From repository root: `python -B -m pytest -q -p no:cacheprovider tests`.
See [test map](../../tests/README.md) and [validation](../../VALIDATION.md).
All current regression tests are offline. Hardware qualification is separate,
requires an explicitly prepared mechanism, and must measure actual rates/errors.
