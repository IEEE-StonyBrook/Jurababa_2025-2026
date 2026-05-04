#ifndef APP_MOTION_STATE_H
#define APP_MOTION_STATE_H

#include <cstdint>

enum class MotionResult : uint8_t
{
    None,
    Completed,
    Stopped,
    Rejected,
    Error
};

/**
 * @brief Core1-published motion lifecycle state.
 *
 * Core0 queues commands with monotonically increasing IDs. Core1 accepts one
 * command at a time, publishes the accepted/current ID, and only advances
 * `completed_command_id` when the matching Robot/search action is genuinely
 * finished. STOP is out-of-band and may complete the current command as
 * `Stopped` without waiting for any queued command.
 */
struct MotionState
{
    static inline volatile bool         active               = false;
    static inline volatile uint16_t     accepted_command_id  = 0;
    static inline volatile uint16_t     completed_command_id = 0;
    static inline volatile uint16_t     current_command_id   = 0;
    static inline volatile MotionResult last_result          = MotionResult::None;
    static inline volatile uint8_t      queue_depth          = 0;
};

#endif
