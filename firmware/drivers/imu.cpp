#include "drivers/imu.h"

#include <math.h>

#include "common/log.h"

IMU* IMU::imu_instance_ = nullptr;

namespace
{
float normalize_yaw_delta(float delta)
{
    if (delta > 180.0f)
        return delta - 360.0f;
    if (delta < -180.0f)
        return delta + 360.0f;
    return delta;
}
} // namespace

IMU::IMU(int uart_rx_pin)
    : uart_rx_pin_(uart_rx_pin), packet_buffer_index_(0), yaw_data_ready_(false),
      current_yaw_degrees_(0.0f), yaw_reset_offset_(0.0f), filtered_yaw_degrees_(0.0f),
      prev_raw_yaw_degrees_(0.0f), first_reading_(true), last_packet_yaw_(0.0f),
      cached_omega_degps_(0.0f), m_rot_change_deg_(0.0f), new_yaw_sample_pending_(false),
      packet_seq_(0)
{
    setup_uart();
    setup_interrupt();
    LOG_DEBUG("IMU initialized successfully");
}

void IMU::setup_uart()
{
    imu_instance_ = this;
    uart_init(IMU_UART_ID, IMU_BAUD_RATE);
    gpio_set_function((uint)uart_rx_pin_, GPIO_FUNC_UART);
    uart_set_baudrate(IMU_UART_ID, IMU_BAUD_RATE);
    uart_set_hw_flow(IMU_UART_ID, false, false);
    uart_set_format(IMU_UART_ID, IMU_DATA_BITS, IMU_STOP_BITS, IMU_PARITY);
    uart_set_fifo_enabled(IMU_UART_ID, true);
}

void IMU::setup_interrupt()
{
    uart_set_irq_enables(IMU_UART_ID, true, false);
    irq_set_exclusive_handler(IMU_UART_IRQ, IMU::uart_interrupt_handler);
    irq_set_enabled(IMU_UART_IRQ, true);
    LOG_DEBUG("IMU UART interrupt configured");
}

void IMU::uart_interrupt_handler()
{
    if (imu_instance_ != nullptr)
    {
        imu_instance_->process_receive_data();
    }
}

void IMU::process_receive_data()
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
            parse_packet_and_extract_yaw();
            packet_buffer_index_ = 0;
        }
    }
}

void IMU::parse_packet_and_extract_yaw()
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

    // Compute per-packet yaw delta and omega at the BNO085's true 100 Hz cadence.
    // This is the authoritative rate — multiplying by LOOP_FREQUENCY_HZ in the
    // main loop instead would alias (4 of 5 ticks see "no change", omega = 0).
    if (yaw_data_ready_)
    {
        float delta         = normalize_yaw_delta(current_yaw_degrees_ - last_packet_yaw_);
        m_rot_change_deg_   = delta;
        cached_omega_degps_ = delta * IMU_PACKET_HZ;
    }
    last_packet_yaw_ = current_yaw_degrees_;

    yaw_data_ready_         = true;
    new_yaw_sample_pending_ = true; // ISR-context: tells main loop a fresh packet is available
    packet_seq_++;                  // monotonic ISR-side counter for edge-detection consumers
}

float IMU::robot_angle()
{
    if (!yaw_data_ready_)
    {
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

void IMU::reset()
{
    if (!yaw_data_ready_)
    {
        LOG_DEBUG("Cannot reset yaw - IMU not ready");
        return;
    }
    LOG_DEBUG("Resetting IMU yaw to zero");
    yaw_reset_offset_   = current_yaw_degrees_;
    last_packet_yaw_    = current_yaw_degrees_;
    m_rot_change_deg_   = 0.0f;
    cached_omega_degps_ = 0.0f;
}

void IMU::update()
{
    // No-op: rotation tracking is now ISR-side (parse_packet_and_extract_yaw)
    // at the BNO085's true 100 Hz cadence. Sampling at the 500 Hz loop rate
    // would alias the signal — see config/sensors.h IMU_PACKET_HZ comment.
}

float IMU::robot_omega()
{
    return cached_omega_degps_;
}

float IMU::robot_rot_change()
{
    return m_rot_change_deg_;
}

bool IMU::has_new_yaw_sample()
{
    // Race-free read-and-clear: the GCC atomic exchange is one instruction
    // logically, so the UART ISR cannot slip a fresh packet between our
    // read and our clear. On Cortex-M0+ this compiles to a brief
    // disable-IRQ / load / store / restore-IRQ sequence; same effect as
    // a critical section, but expresses the intent ("atomic swap") clearly.
    return __atomic_exchange_n(&new_yaw_sample_pending_, false, __ATOMIC_SEQ_CST);
}

uint32_t IMU::packet_seq() const
{
    // Plain volatile read. Single-word load is atomic on Cortex-M0+, and we
    // don't need a clear — consumers compare against a previously-stashed
    // value to detect edges.
    return packet_seq_;
}
