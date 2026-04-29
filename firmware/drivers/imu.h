#ifndef DRIVERS_IMU_H
#define DRIVERS_IMU_H

#include <cmath>
#include <cstdint>

#include "common/log.h"
#include "config/config.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

/**
 * @brief Inertial Measurement Unit (IMU) interface for BNO085 sensor
 *
 * Provides UART-based communication with BNO085 IMU using RVC (Rotation Vector
 * Compressed) protocol. Reads yaw angle via interrupt-driven packet reception
 * with checksum validation.
 *
 * API mirrors mazerunner-core's `Encoders` rotation interface — substituting an
 * IMU as the rotation source. Caller invokes update() once per control tick to
 * sample yaw and refresh rot_change; robot_omega() / robot_rot_change() then
 * return cached values.
 */
class IMU
{
  public:
    /**
     * @brief Constructs IMU interface with UART configuration
     * @param uart_rx_pin UART receive pin for IMU communication
     */
    explicit IMU(int uart_rx_pin);

    /**
     * @brief Per-tick sampler. Call once per control loop tick.
     *
     * Samples current yaw and updates m_rot_change_deg_ as the per-tick yaw
     * delta. Mirrors mazerunner's `Encoders::update()`.
     */
    void update();

    /**
     * @brief Returns current yaw angle normalized to [-180, 180] degrees
     *        (relative to last reset()).
     */
    float robot_angle();

    /**
     * @brief Returns angular velocity (deg/s).
     *
     * Equals `m_rot_change_deg_ * LOOP_FREQUENCY_HZ`, same shape as
     * mazerunner's `robot_omega() = LOOP_FREQUENCY * m_rot_change`.
     * Updated by update().
     */
    float robot_omega();

    /**
     * @brief Returns the per-tick yaw delta (deg) cached by the last update().
     *        Mirrors mazerunner's `Encoders::robot_rot_change()`.
     */
    float robot_rot_change();

    /**
     * @brief Resets yaw offset to make current heading = 0 degrees.
     *        Also zeros rot_change tracking.
     */
    void reset();

  private:
    const int    uart_rx_pin_;
    volatile int packet_buffer_index_;
    uint8_t      packet_buffer_[IMU_PACKET_LEN];
    bool         yaw_data_ready_;
    float        current_yaw_degrees_;
    float        yaw_reset_offset_;

    // Filtering state for noise reduction
    float filtered_yaw_degrees_; // EMA-filtered yaw
    float prev_raw_yaw_degrees_; // Previous raw yaw for outlier detection
    bool  first_reading_;        // Skip outlier check on first reading

    // Per-tick rotation tracking (mazerunner Encoders shape)
    float prev_rot_yaw_;     // yaw at last update() call
    float m_rot_change_deg_; // per-tick yaw delta (deg)

    static IMU* imu_instance_;

    void        setup_uart();
    void        setup_interrupt();
    void        process_receive_data();
    void        parse_packet_and_extract_yaw();
    static void uart_interrupt_handler();
};

#endif
