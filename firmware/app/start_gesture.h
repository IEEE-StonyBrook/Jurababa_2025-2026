#ifndef APP_START_GESTURE_H
#define APP_START_GESTURE_H

#include <cstdint>

class Bluetooth;
class ToF;

/**
 * @brief Outcome of the start-gesture detectors.
 *
 * `TOF_WAVE`: front-ToF wave only (legacy single-sensor entry path).
 * `FRONT_WAVE`: synonym for TOF_WAVE used by the competition detector.
 * `RIGHT_WAVE`: right-ToF wave only — competition Stage 3 (fast cardinal).
 * `LEFT_WAVE`: left-ToF wave only — competition Stage 5 (fast diagonal).
 * `SERIAL_G`: 'G' / 'g' received over USB-CDC stdin.
 * `SERIAL_H`: 'H' / 'h' — sim shortcut for RIGHT_WAVE.
 * `SERIAL_J`: 'J' / 'j' — sim shortcut for LEFT_WAVE.
 * `BT_START`: `Bluetooth::Command::START` received over UART0.
 * `CANCELLED`: `Bluetooth::Command::HALT` received — caller should abort the
 *              behavior instead of starting motion.
 */
enum class StartTrigger
{
    TOF_WAVE,
    FRONT_WAVE = TOF_WAVE,
    RIGHT_WAVE,
    LEFT_WAVE,
    SERIAL_G,
    SERIAL_H,
    SERIAL_J,
    BT_START,
    CANCELLED
};

/**
 * @brief Block until the operator signals "go" via front-ToF wave or serial.
 *
 * Original single-sensor entry path used by non-competition flows. The ToF
 * gesture is a hysteresis state machine: hand inside `low_mm`, then back
 * past `high_mm`. Trigger fires on the rising edge so the robot only starts
 * after the hand is removed.
 *
 * Polls every 20 ms. Always honors USB 'G' and `Bluetooth::Command::START`.
 * Skips the ToF check entirely when `front_tof == nullptr` (line-sensor
 * mode shares I2C0 with the ToFs, so they're not readable then).
 */
StartTrigger waitForStartGesture(Bluetooth* bt, ToF* front_tof, uint32_t low_mm = 80,
                                 uint32_t high_mm = 110);

/**
 * @brief Block until the operator signals one of three competition gestures.
 *
 * Reads three independent hysteresis state machines in parallel — one per
 * ToF — and returns whichever fires first:
 *   front -> FRONT_WAVE  (Stage 1+2 in the COMP loop)
 *   right -> RIGHT_WAVE  (Stage 3)
 *   left  -> LEFT_WAVE   (Stage 5)
 *
 * No grouping window: each gesture is geographically distinct so an operator
 * waving on one side cannot accidentally trigger a different stage even if
 * their hand briefly clips an unrelated sensor's range — the per-sensor
 * machine still requires a full close-then-far transition. Whichever sensor
 * completes its transition first wins.
 *
 * Honors USB 'G'/'H'/'J' as simulator shortcuts for FRONT/RIGHT/LEFT, plus
 * `Bluetooth::Command::START` (front) and `Bluetooth::Command::HALT`.
 *
 * Any of the ToF pointers may be nullptr if that sensor is unavailable.
 * The detector still functions on the remaining sensors.
 */
StartTrigger waitForCompetitionGesture(Bluetooth* bt, ToF* front_tof, ToF* right_tof, ToF* left_tof,
                                       uint32_t low_mm = 80, uint32_t high_mm = 110);

#endif
