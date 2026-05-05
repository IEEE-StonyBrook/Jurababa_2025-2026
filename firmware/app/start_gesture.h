#ifndef APP_START_GESTURE_H
#define APP_START_GESTURE_H

#include <cstdint>

class Bluetooth;
class ToF;

/**
 * @brief Outcome of `waitForStartGesture`.
 *
 * `TOF_WAVE`: operator waved their hand close-then-far in front of the robot.
 * `SERIAL_G`: 'G' / 'g' received over USB-CDC stdin.
 * `BT_START`: `Bluetooth::Command::START` received over UART0.
 * `CANCELLED`: `Bluetooth::Command::HALT` received — caller should abort the
 *              behavior instead of starting motion.
 */
enum class StartTrigger
{
    TOF_WAVE,
    SERIAL_G,
    BT_START,
    CANCELLED
};

/**
 * @brief Block until the operator signals "go" via wave-gesture or serial.
 *
 * The ToF gesture is a hysteresis state machine: the operator brings their
 * hand inside `low_mm`, then moves it back past `high_mm`. The trigger fires
 * on the second transition (rising edge), not on the first — that way the
 * robot only starts *after* the hand is removed.
 *
 * Polls every 20 ms. Always honors USB 'G' and `Bluetooth::Command::START`.
 * Skips the ToF check entirely when `front_tof_available == false` (line
 * sensor mode shares I2C0 with the front ToF, so the ToF isn't readable).
 *
 * @param bt                  Bluetooth driver, or nullptr to disable BT polling.
 * @param front_tof_available True iff Core 1 is publishing valid front-ToF data.
 * @param low_mm              Hand-close threshold (default 80 mm).
 * @param high_mm             Hand-clear threshold (default 110 mm); must exceed `low_mm`.
 */
StartTrigger waitForStartGesture(Bluetooth* bt, ToF* front_tof, uint32_t low_mm = 80,
                                 uint32_t high_mm = 110);

#endif
