#ifndef IMU_H
#define IMU_H

#include <cmath>
#include <cstdint>

#include "Common/LogSystem.h"
#include "Platform/Pico/Config.h"
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
    float getIMUYawDegreesNeg180ToPos180();

    /**
     * @brief Resets yaw offset to make current heading = 0 degrees
     *
     * Stores current yaw as zero reference for future readings
     */
    void resetIMUYawToZero();

    /**
     * @brief Calculates new yaw after adding angle offset
     * @param degrees_to_add Angle delta to add (degrees)
     * @return New yaw normalized to [-180, 180] degrees
     */
    float getNewYawAfterAddingDegrees(float degrees_to_add);

  private:
    // UART configuration
    const int uart_rx_pin_;  // UART receive pin

    // Packet reception state
    volatile int packet_buffer_index_;  // Current index in receive buffer
    uint8_t packet_buffer_[IMU_PACKET_LEN];  // Packet receive buffer
    bool yaw_data_ready_;  // True when valid yaw has been received

    // Yaw angle state
    float current_yaw_degrees_;  // Latest yaw reading in [-180, 180]
    float yaw_reset_offset_;  // Zero offset for yaw calibration

    // Static instance for interrupt handler access
    static IMU* imu_instance_;

    /**
     * @brief Initializes UART hardware for IMU communication
     */
    void setupUARTCommunication();

    /**
     * @brief Configures UART receive interrupt
     */
    void setupUARTInterrupt();

    /**
     * @brief Processes incoming UART data in interrupt context
     */
    void processUARTReceiveData();

    /**
     * @brief Parses packet buffer and extracts yaw angle
     */
    void parsePacketAndExtractYaw();

    /**
     * @brief Static interrupt handler for UART receive events
     */
    static void uartInterruptHandler();
};
#endif