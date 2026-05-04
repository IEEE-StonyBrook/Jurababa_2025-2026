#ifndef APP_CORE1_H
#define APP_CORE1_H

class Battery;

/**
 * @brief Core 1 entry point — owns the 500 Hz real-time control loop.
 *
 * Constructs all motion-side hardware (motors, encoders, IMU, ToFs),
 * builds a `Drivetrain` and `Robot`, then runs `Robot::update()` plus a
 * MotorLab-style cooperative motion server once per tick. UKMARS-style
 * multi-stage mouse actions (`move_ahead`, smooth turns, turn-back centre
 * correction) are advanced as state machines inside the same tick. ToFs are
 * read at 50 Hz (every 10 ticks) and published through `SensorHub`.
 *
 * Core0 queues commands in shared memory. Core1 accepts one command only when
 * the Robot/search action is idle, publishes the accepted ID, and marks that
 * same ID complete only after the physical motion finishes. STOP is an
 * out-of-band flag checked before queued work.
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
