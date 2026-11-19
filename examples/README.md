# Motor configuration

[motor.json](motor.json) is a one-motor example, not a validated mechanism configuration.
CLI: `python -m handycan test --config examples/motor.json --active` explicitly enables
and polls motors; review the [Jetson guide](../docs/jetson.md) first.

| Field | Meaning / units |
| --- | --- |
| channel | Existing Linux CAN interface, normally can0 |
| control_hz | Requested per-motor command rate; shared bus budget is validated |
| motors | One row per motor, with unique motor_id |
| motor_id / reply_id | Command arbitration ID / feedback arbitration ID |
| model | AK70-10-V2 or AK80-6-V2; omitted means AK70-10-V2 |
| joint_min / joint_max | Signed absolute degrees from motor zero; no target wrapping |
| speed_max | Degrees per second |
| torque_max | Operating torque limit in Nm, separate from firmware encoding range |
| kp_max / kd_max | Maximum permitted MIT gains, in the motor's radian-based convention |
| feedback_timeout | Maximum feedback age in seconds; default 0.1 |
| timing | Optional TimingPolicy fields, all in seconds; root README lists defaults |

For multiple motors, duplicate a row inside `motors` and change IDs/model/limits.
Set the rate for the combined bus budget; the current conservative validator
rejects three motors at 1000 Hz. Configure all rows before connecting.
In Python, use `Motor.from_degrees(...)`; JSON is loaded by `cli.load_config()`.
The GUI uses its own configured rows, not this JSON automatically.

Adding a new model requires documented firmware encoding ranges and feedback
format checks in [protocol.py](../src/handycan/protocol.py), codec regression tests,
and a GUI dropdown entry on the GUI branch. A rated torque value is not necessarily
the MIT encoding maximum. Never tune encoding constants to impose operating limits.
