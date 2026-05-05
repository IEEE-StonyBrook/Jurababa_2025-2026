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

    // Latched by Core1 when a continuous exploration move crosses the
    // configured wall-check position. Core0 waits for the command id, copies
    // this sample into FirmwareApi, and can queue the next action while Core1
    // continues toward the cell center.
    static inline volatile bool     wall_check_ready        = false;
    static inline volatile uint16_t wall_check_command_id   = 0;
    static inline volatile uint16_t wall_check_sequence     = 0;
    static inline volatile int16_t  wall_check_left_mm      = 0;
    static inline volatile int16_t  wall_check_front_mm     = 0;
    static inline volatile int16_t  wall_check_right_mm     = 0;
    static inline volatile float    wall_check_yaw_deg      = 0.0f;
    static inline volatile uint32_t wall_check_timestamp_ms = 0;
};

#endif
