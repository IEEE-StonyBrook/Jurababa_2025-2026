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
     * @brief Per-tick sampler. No-op for rotation tracking under the new
     *        IMU model — kept for API parity with mazerunner's Encoders.
     *
     * Omega and per-packet yaw delta are now computed in the UART ISR at the
     * BNO085's true 100 Hz packet cadence (see parse_packet_and_extract_yaw).
     * Sampling at the 500 Hz loop rate would alias the signal.
     */
    void update();

    /**
     * @brief Returns current yaw angle normalized to [-180, 180] degrees
     *        (relative to last reset()).
     */
    float robot_angle();

    /**
     * @brief Returns angular velocity (deg/s), cached in the UART ISR.
     *
     * Equals `avg_delta_yaw_per_packet * IMU_PACKET_HZ`, where avg_delta is
     * the IMU_DELTA_AVG_LENGTH-tap moving average of per-packet yaw deltas.
     * Mirrors the encoder path: filter the delta the PD consumes, not the
     * upstream yaw position — so no extra phase lag enters the D-term.
     * Held flat between packets (ZOH), so consumers see a steady value
     * rather than a spike of zeros.
     */
    float robot_omega();

    /**
     * @brief Returns the per-PACKET yaw delta (deg), MA-smoothed over
     *        IMU_DELTA_AVG_LENGTH packets. Updated in the UART ISR, held flat
     *        between packets — matches the rotation PD's gating on
     *        has_new_yaw_sample(). Same role as Drivetrain::fwdChangeMm() on
     *        the linear side.
     */
    float robot_rot_change();

    /**
     * @brief Read-and-clear: returns true exactly once per fresh BNO085 packet.
     *
     * Used by the rotation PID gate. Forward (encoder-paced) control runs
     * every 500 Hz tick; rotation control fires only when this returns true,
     * matching the IMU's ~100 Hz packet cadence. Between packets, the rotation
     * controller's last output is held (zero-order hold).
     *
     * Called from the main control loop; the underlying flag is set from the
     * UART ISR. Implementation must handle ISR/main concurrency.
     */
    bool has_new_yaw_sample();

    /**
     * @brief Monotonic counter incremented once per accepted packet (ISR-side).
     *        Wraps at 2^32 (~14 months at 100 Hz). Read-only — use a local
     *        `last_seq` and compare for edge detection without consuming the
     *        has_new_yaw_sample() pending flag (which is owned by the
     *        rotation PID gate).
     */
    uint32_t packet_seq() const;

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

    // Outlier-rejection state. Raw (unfiltered) yaw is fed straight through —
    // no EMA on position. The MA below filters the delta instead, mirroring
    // the encoder path so the rotation D-term never sees re-differentiated
    // smoothing.
    float prev_raw_yaw_degrees_; // Previous raw yaw for outlier detection
    bool  first_reading_;        // Skip outlier check on first reading

    // Per-PACKET rotation tracking (computed in UART ISR at 100 Hz cadence).
    // Cortex-M0+ single-word loads/stores make plain float read-from-main /
    // write-from-ISR atomic enough for these scalars (same model as
    // current_yaw_degrees_).
    float last_packet_yaw_;    // yaw at the last accepted packet
    float cached_omega_degps_; // avg_delta_yaw * IMU_PACKET_HZ
    float m_rot_change_deg_;   // averaged per-packet yaw delta, held between packets

    // Moving-average ring buffer over per-packet yaw deltas. Structurally
    // mirrors Drivetrain::update()'s encoder averager (motorlab/encoders.h),
    // but with float entries — yaw deltas are real-valued degrees, and a
    // single packet can carry several degrees during a 360°/s turn (well past
    // int8_t range). Subtract oldest, add newest, advance index — O(1).
    float   delta_history_[IMU_DELTA_AVG_LENGTH];
    float   delta_history_total_;
    uint8_t delta_history_index_;

    // Set by parse_packet_and_extract_yaw() (ISR context) when a fresh packet
    // is accepted. Read-and-cleared by has_new_yaw_sample() (main context).
    volatile bool new_yaw_sample_pending_;

    // ISR-side monotonic packet counter. Incremented once per accepted packet
    // alongside new_yaw_sample_pending_. Read-only via packet_seq(); used by
    // DriverLab TURN-STEP to gate Tm-fit samples on packet edges without
    // racing the rotation PID's has_new_yaw_sample() consumer.
    volatile uint32_t packet_seq_;

    static IMU* imu_instance_;

    void        setup_uart();
    void        setup_interrupt();
    void        process_receive_data();
    void        parse_packet_and_extract_yaw();
    static void uart_interrupt_handler();
};

#endif
