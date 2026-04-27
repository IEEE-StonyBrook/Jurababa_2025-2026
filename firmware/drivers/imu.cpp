#include "drivers/imu.h"

#include <math.h>

#ifdef USE_MULTICORE_SENSORS
#include "app/multicore.h"
#endif

#include "common/log.h"

IMU* IMU::imu_instance_ = nullptr;

IMU::IMU(int uart_rx_pin)
    : uart_rx_pin_(uart_rx_pin), packet_buffer_index_(0), yaw_data_ready_(false),
      current_yaw_degrees_(0.0f), yaw_reset_offset_(0.0f), filtered_yaw_degrees_(0.0f),
      prev_raw_yaw_degrees_(0.0f), first_reading_(true)
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

        // Check first header byte (0xAA)
        if (packet_buffer_index_ == 0 && received_byte != IMU_HDR0)
            continue;

        // Check second header byte (also 0xAA in RVC mode)
        if (packet_buffer_index_ == 1 && received_byte != IMU_HDR1)
        {
            packet_buffer_index_ = 0; // Reset and start over
            continue;
        }

        packet_buffer_[packet_buffer_index_++] = received_byte;

        if (packet_buffer_index_ >= IMU_PACKET_LEN)
        {
            parsePacketAndExtractYaw();
            packet_buffer_index_ = 0;
        }
    }
}

void IMU::parsePacketAndExtractYaw()
{
    // Validate checksum
    uint8_t checksum = 0;
    for (int i = IMU_CHKSUM_FIRST; i <= IMU_CHKSUM_LAST; i++)
        checksum += packet_buffer_[i];

    if (checksum != packet_buffer_[IMU_IDX_CHECKSUM])
        return; // Bad checksum, discard packet

    int16_t raw_yaw     = (packet_buffer_[IMU_IDX_YAW_H] << 8) | packet_buffer_[IMU_IDX_YAW_L];
    float   yaw_degrees = static_cast<float>(raw_yaw) / IMU_RAW_TO_DEGREES_DIVISOR;

    // Apply mounting-orientation sign correction so positive yaw = CCW (left).
    yaw_degrees *= IMU_YAW_SIGN;

    // Normalize to [-180, 180]
    yaw_degrees = fmodf(yaw_degrees + 180.0f, 360.0f) - 180.0f;

    // Outlier rejection: skip if change is physically impossible
    if (!first_reading_)
    {
        float delta = yaw_degrees - prev_raw_yaw_degrees_;
        // Normalize delta to [-180, 180]
        if (delta > 180.0f)
            delta -= 360.0f;
        if (delta < -180.0f)
            delta += 360.0f;

        if (std::fabs(delta) > IMU_MAX_YAW_DELTA_PER_SAMPLE)
            return; // Reject outlier - physically impossible change
    }

    prev_raw_yaw_degrees_ = yaw_degrees;
    first_reading_        = false;

    // Apply EMA filter to smooth yaw readings
    if (!yaw_data_ready_)
    {
        filtered_yaw_degrees_ = yaw_degrees; // Initialize filter on first valid reading
    }
    else
    {
        float alpha = IMU_YAW_FILTER_ALPHA;
        // Handle wraparound for filtering (compute shortest path)
        float diff = yaw_degrees - filtered_yaw_degrees_;
        if (diff > 180.0f)
            diff -= 360.0f;
        if (diff < -180.0f)
            diff += 360.0f;
        filtered_yaw_degrees_ += alpha * diff;
        // Renormalize to [-180, 180]
        if (filtered_yaw_degrees_ > 180.0f)
            filtered_yaw_degrees_ -= 360.0f;
        if (filtered_yaw_degrees_ < -180.0f)
            filtered_yaw_degrees_ += 360.0f;
    }

    current_yaw_degrees_ = filtered_yaw_degrees_;
    yaw_data_ready_      = true;
}

float IMU::yaw()
{
    if (!yaw_data_ready_)
    {
        // LOG_DEBUG("IMU yaw not ready, returning 0");
        return 0.0f;
    }

    // Compute yaw relative to reset offset
    float result = current_yaw_degrees_ - yaw_reset_offset_;

    // Normalize result to [-180, 180]
    if (result > 180.0f)
        result -= 360.0f;
    else if (result < -180.0f)
        result += 360.0f;

    return result;
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
    float new_yaw  = yaw() + degrees_to_add;

    if (new_yaw > 180.0f)
        new_yaw -= 360.0f;
    else if (new_yaw < -180.0f)
        new_yaw += 360.0f;

    return new_yaw;
}
