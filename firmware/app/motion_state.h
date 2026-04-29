#ifndef APP_MOTION_STATE_H
#define APP_MOTION_STATE_H

/**
 * @brief Cross-core flag set by Core 1 while a motion command is in flight.
 *
 * Core 1 sets `active = true` when it accepts a CommandHub motion (move/turn)
 * and clears it once Robot::motionComplete() reports done. Core 0 polls this
 * to back-pressure command emission so the 8-deep multicore FIFO never
 * blocks `multicore_fifo_push_blocking`.
 *
 * `volatile` (not std::atomic) is sufficient: on RP2040 (Cortex-M0+) aligned
 * single-byte reads/writes are atomic at the hardware level, and there is
 * exactly one writer (Core 1) and one reader (Core 0). The volatile qualifier
 * exists to stop the compiler from hoisting the read out of the spin loop on
 * Core 0 (`while (MotionState::active) { ... }`), which would deadlock.
 */
struct MotionState
{
    static inline volatile bool active = false;
};

#endif
