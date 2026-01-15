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
     * @brief Returns current yaw angle normalized to [-180, 180] degrees
     * @return Yaw angle in degrees (relative to reset offset)
     */
    float yaw();

    /**
     * @brief Resets yaw offset to make current heading = 0 degrees
     */
    void resetYaw();

    /**
     * @brief Calculates new yaw after adding angle offset
     * @param degrees_to_add Angle delta to add (degrees)
     * @return New yaw normalized to [-180, 180] degrees
     */
    float yawAfterAdding(float degrees_to_add);

  private:
    const int    uart_rx_pin_;
    volatile int packet_buffer_index_;
    uint8_t      packet_buffer_[IMU_PACKET_LEN];
    bool         yaw_data_ready_;
    float        current_yaw_degrees_;
    float        yaw_reset_offset_;

    static IMU* imu_instance_;

    void        setupUART();
    void        setupInterrupt();
    void        processReceiveData();
    void        parsePacketAndExtractYaw();
    static void uartInterruptHandler();
};

#endif
