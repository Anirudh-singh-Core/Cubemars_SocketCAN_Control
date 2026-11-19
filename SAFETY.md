# Experimental hardware controller

Supported codec profiles: CubeMars AK70-10 V2 and AK80-6 V2 in MIT mode. Match the
actual motor firmware; a similar product name alone does not establish protocol
compatibility. Jetson pin setup targets the documented Orin Nano/J17 arrangement;
Pi 5 requires an appropriate external CAN controller and Linux driver.

- Enabling can allow movement. Neutral monitoring uses zero gains and does not
  hold a loaded joint. Torque quantization and firmware startup behavior matter.
- Set Zero changes the reference. Review joint limits and mechanism alignment
  before further motion. Explicit move/stream commands can produce torque.
- A software STOP/disable is a CAN request. A failed bus, driver or process can
  prevent delivery. Use an independent hardware means to stop hazardous motion.
- Software torque checks are not a certified hardware torque limiter. Select
  mechanism-specific angle, speed, torque and gain limits before enabling.
- The GUI's browser heartbeat is disabled: closing the tab does not stop holding
  or monitoring. Explicitly stop/disconnect and shut down the controller.
- Setup/recovery interrupts CAN; it does not disable a powered motor for you.
  Stop and support the mechanism before running Recover_can.
- Linux/Python scheduling is best effort. Offline tests do not validate physical
  motion, requested loop frequency, electrical reliability or emergency stopping.

See [VALIDATION.md](VALIDATION.md) for evidence and limitations. Begin any new
hardware qualification with supported joints and controlled operating limits;
do not infer safe operation merely from a passing communication test.
