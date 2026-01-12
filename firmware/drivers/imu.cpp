#include "drivers/imu.h"

#include <math.h>

#ifdef USE_MULTICORE_SENSORS
#include "app/multicore.h"
#endif

#include "common/log.h"

IMU* IMU::imu_instance_ = nullptr;

IMU::IMU(int uart_rx_pin)
    : uart_rx_pin_(uart_rx_pin),
      packet_buffer_index_(0),
      yaw_data_ready_(false),
      current_yaw_degrees_(0.0f),
      yaw_reset_offset_(0.0f)
{
    setupUART();
    setupInterrupt();
    LOG_DEBUG("IMU initialized successfully");
}

void IMU::setupUART()
{
    imu_instance_ = this;
    uart_init(IMU_UART_ID, IMU_BAUD_RATE);
    gpio_set_function((uint)uart_rx_pin_, GPIO_FUNC_UART);
    uart_set_baudrate(IMU_UART_ID, IMU_BAUD_RATE);
    uart_set_hw_flow(IMU_UART_ID, false, false);
    uart_set_format(IMU_UART_ID, IMU_DATA_BITS, IMU_STOP_BITS, IMU_PARITY);
    uart_set_fifo_enabled(IMU_UART_ID, true);
}

void IMU::setupInterrupt()
{
    uart_set_irq_enables(IMU_UART_ID, true, false);
    irq_set_exclusive_handler(IMU_UART_IRQ, IMU::uartInterruptHandler);
    irq_set_enabled(IMU_UART_IRQ, true);
    LOG_DEBUG("IMU UART interrupt configured");
}

void IMU::uartInterruptHandler()
{
    if (imu_instance_ != nullptr)
    {
        imu_instance_->processReceiveData();
    }
}

void IMU::processReceiveData()
{
    while (uart_is_readable(IMU_UART_ID))
    {
        uint8_t received_byte = uart_getc(IMU_UART_ID);

        if (packet_buffer_index_ == 0 && received_byte != IMU_HDR0)
            continue;

        packet_buffer_[packet_buffer_index_++] = received_byte;

        if (packet_buffer_index_ >= IMU_PACKET_LEN)
        {
            if (packet_buffer_[IMU_IDX_HDR0] == IMU_HDR0)
                parsePacketAndExtractYaw();

            packet_buffer_index_ = 0;
        }
    }
}

void IMU::parsePacketAndExtractYaw()
{
    packet_buffer_index_ = 0;

    uint8_t checksum = 0;
    for (int i = IMU_CHKSUM_FIRST; i <= IMU_CHKSUM_LAST; i++)
        checksum += packet_buffer_[i];

    if (checksum == packet_buffer_[IMU_IDX_CHECKSUM])
    {
        int16_t raw_yaw = (packet_buffer_[IMU_IDX_YAW_H] << 8) | packet_buffer_[IMU_IDX_YAW_L];
        float yaw_degrees = static_cast<float>(raw_yaw) / IMU_RAW_TO_DEGREES_DIVISOR;
        current_yaw_degrees_ = fmodf(yaw_degrees + 180.0f, 360.0f) - 180.0f;
        yaw_data_ready_ = true;
    }
}

float IMU::yaw()
{
    if (!yaw_data_ready_)
    {
        LOG_DEBUG("IMU yaw not ready, returning 0");
        return 0.0f;
    }
    return current_yaw_degrees_ - yaw_reset_offset_;
}

void IMU::resetYaw()
{
    if (!yaw_data_ready_)
    {
        LOG_DEBUG("Cannot reset yaw - IMU not ready");
        return;
    }
    LOG_DEBUG("Resetting IMU yaw to zero");
    yaw_reset_offset_ = current_yaw_degrees_;
}

float IMU::yawAfterAdding(float degrees_to_add)
{
    degrees_to_add = fmodf(degrees_to_add, 360.0f);
    float new_yaw = yaw() + degrees_to_add;

    if (new_yaw > 180.0f)
        new_yaw -= 360.0f;
    else if (new_yaw < -180.0f)
        new_yaw += 360.0f;

    return new_yaw;
}
