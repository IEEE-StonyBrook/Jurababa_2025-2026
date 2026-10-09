# Subsystem change checklists

Reference consulted when editing one subsystem. Each list is what to verify
*before* the patch, because these are the mistakes that pass review, compile
clean, and then drive the robot into a wall.

Firmware bugs almost always come from one of: wiring, pin mapping, units,
timing, sensor validity, sign convention, state ownership, initialisation order.

## Motors — `firmware/drivers/motor.*`, `control/drivetrain.*`

- PWM bounds and saturation behaviour.
- Direction pin polarity; left/right sign conventions.
- Brake vs coast.
- Whether the command is normalised duty, percent, raw PWM, or voltage-equivalent.
- Whether negative velocity means reverse here or is handled upstream.
- What the motors do when a sensor fails.

Assume neither unlimited PWM nor left/right symmetry.

## Encoders — `firmware/drivers/encoder.*`

- Tick sign convention; left vs right orientation.
- Ticks per revolution, wheel diameter, gear ratio.
- Overflow and reset behaviour.
- Whether velocity comes from absolute ticks or tick deltas.

Encoder bugs masquerade as PID bugs. Suspect the encoder first.

## Sensors — `firmware/drivers/tof.*`, `imu.*`, `line_sensor.*`, `beacon_ir.*`

- Units, thresholds, filtering.
- Invalid readings and timeout behaviour.
- Sensor orientation and startup calibration.
- I2C/UART failure behaviour.
- Whether a reading is raw, filtered, or fused.

Treat every reading as possibly invalid.

## Control — `firmware/control/*`

- Loop frequency and `dt` units.
- PID windup; derivative noise.
- Feedforward sign; interaction between feedforward and feedback.
- Velocity saturation and acceleration limits.
- Stop conditions and tolerance thresholds.

A control loop should fail gracefully, not heroically.

### Loop dt: use `LOOP_FREQUENCY_HZ`, never measure it

`dt_s` is `LOOP_INTERVAL_S = 1 / LOOP_FREQUENCY_HZ` (500 Hz). Keep it that way:

1. **Determinism.** The main loop paces with `sleep_until(next_tick)`, so the
   interval *is* `LOOP_INTERVAL_S` by construction.
2. **Latency.** Per-tick clock reads cost cycles and make the controller chase
   its own measurement noise.
3. **Correctness.** kP, kD, kV and kA were derived against the design dt.

A loop running long is a loop to fix — remove blocking calls, use cooperative
state machines, keep `sleep_ms` out of trial bodies — not a dt to measure.

Sensor sample rates are separate. The IMU emits packets at exactly 100 Hz and
its omega cache uses `IMU_PACKET_HZ`, computed once per packet in the UART ISR.
That is the IMU's true cadence; using 500 Hz there would alias the signal.

## Multicore — `firmware/app/*`

- Which core owns each piece of state.
- Whether data is copied or shared, and whether it can be stale.
- Whether updates are atomic enough.
- Whether commands can be dropped, and whether their order matters.
- Whether a blocking call can disturb the 500 Hz loop.

Core 1 stays predictable and timing-conscious.

## Maze — `firmware/maze/*`

- Coordinate and direction conventions.
- Wall bit encoding; whether walls are stored bidirectionally.
- Bounds checks, start cell, goal cells.
- Unknown-wall assumptions; explored vs unexplored cells.

A wall added between cells A and B leaves both cells consistent, unless the
existing design deliberately stores walls one-way.

## Navigation — `firmware/navigation/*`

- Whether the path includes start and goal.
- Row-major vs x/y; absolute vs relative directions.
- How turns are encoded (L/F/R/B).
- Whether diagonal paths stay physically feasible.
- Whether flood fill handles unreachable cells.
- Whether the A* heuristic is admissible.
- Whether unknown walls are treated optimistically or pessimistically.
