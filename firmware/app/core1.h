#ifndef APP_CORE1_H
#define APP_CORE1_H

class Battery;

/**
 * @brief Core 1 entry point — owns the 500 Hz real-time control loop.
 *
 * Constructs all motion-side hardware (motors, encoders, IMU, ToFs),
 * builds a `Drivetrain` and `Robot`, then runs `Robot::update()` plus
 * `processCommands()` once per tick. ToFs are read at 50 Hz (every 10
 * ticks) and published through `SensorHub`.
 *
 * Owns these wires for the cross-core motion-complete handshake:
 *   - Sets `MotionState::active = true` whenever it accepts a motion
 *     command from `CommandHub`.
 *   - Clears `MotionState::active` once `Robot::motionComplete()` is
 *     true AND no further commands are pending in the FIFO.
 *
 * Only launched in ToF sensor mode. In line-sensor mode the motor pins
 * are owned by `LineFollower` on Core 0 — launching both would race on
 * the same H-bridge.
 */
void core1Entry();

/**
 * @brief Hand the shared battery pointer to Core 1 before launch.
 *
 * Core 1 does not allocate the battery; main constructs it and passes
 * a pointer here so Core 1's `Drivetrain` can read pack voltage for
 * voltage-feedforward compensation. Must be called before
 * `multicore_launch_core1(core1Entry)`.
 */
void core1Configure(Battery* battery);

#endif
