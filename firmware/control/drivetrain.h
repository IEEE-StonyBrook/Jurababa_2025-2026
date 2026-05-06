#ifndef CONTROL_DRIVETRAIN_H
#define CONTROL_DRIVETRAIN_H

#include <cstdint>

#include "common/log.h"
#include "common/utils.h"
#include "config/motion.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/motor.h"

/**
 * @brief Differential drive system controller
 *
 * Manages two-wheeled differential drive system with integrated motor control,
 * encoder feedback, and velocity estimation. Provides feedforward compensation
 * for motor nonlinearities (friction, back-EMF).
 */
class Drivetrain
{
  public:
    /**
     * @brief Constructs drivetrain controller with hardware components
     */
    Drivetrain(Motor* left_motor, Motor* right_motor, Encoder* left_encoder, Encoder* right_encoder,
               Battery* battery = nullptr);

    void reset();

    /**
     * @brief Returns total distance traveled by a wheel
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @return Distance in millimeters
     */
    float   position(WheelSide side);
    int32_t ticks(WheelSide side) const;

    /**
     * @brief Returns wheel velocity
     * @return Velocity in millimeters per second
     */
    float velocity(WheelSide side);

    /**
     * @brief Calculates feedforward duty cycle for target velocity and acceleration
     *
     * Uses empirically-tuned coefficients: Kv (back-EMF), Ks (friction), Ka (inertia)
     */
    float feedforward(WheelSide side, float speed_mmps, float accel_mmps2 = 0.0f);

    /**
     * @brief Overrides feedforward coefficients for runtime tuning (DriverLab)
     *
     * Sets both forward and reverse coefficients for the given wheel side.
     */
    void setFeedforward(WheelSide side, float kv, float ks, float ka);

    /**
     * @brief Reads encoders, computes per-tick deltas and velocity estimates.
     *        Runs on the fixed control tick — no dt argument.
     */
    void update();

    /**
     * @brief Average per-tick forward position delta in mm, since the last update().
     *        This is the "measured_change" fed to the forward PD controller.
     */
    float fwdChangeMm() const { return fwd_change_mm_; }

    /**
     * @brief Sets motor voltages (scaled by current battery voltage)
     */
    void setVoltage(float left_volts, float right_volts);

    void stop();

  private:
    // Returns current battery voltage; falls back to DEFAULT_BATTERY_VOLTAGE
    // when no battery driver is wired (sim, bench testing). Consumed by
    // setVoltage() to scale per-wheel volts.
    float batteryVoltage() const;

    Motor*   left_motor_;
    Motor*   right_motor_;
    Encoder* left_encoder_;
    Encoder* right_encoder_;
    Battery* battery_;

    int32_t prev_left_ticks_  = 0;
    int32_t prev_right_ticks_ = 0;

    float left_velocity_mmps_  = 0.0f;
    float right_velocity_mmps_ = 0.0f;

    // 8-tap moving average of per-tick deltas (mm) refreshed by update();
    // consumed by Motion's PD loop via fwdChangeMm(). Mirrors
    // ukmars/motorlab/src/encoders.h::m_fwd_change.
    float fwd_change_mm_ = 0.0f;

    // 8-tap moving averager state, mirroring ukmars/motorlab/src/encoders.h.
    // Operates on integer tick deltas so the ring buffer can store int8_t
    // exactly; conversion to mm happens after averaging. Same window for
    // both wheels keeps fwd_change_mm_ consistent with the per-wheel
    // velocities.
    int8_t left_history_[ENCODER_AVERAGER_LENGTH]  = {};
    int8_t right_history_[ENCODER_AVERAGER_LENGTH] = {};
    int    left_history_total_                     = 0;
    int    right_history_total_                    = 0;
    int    averager_index_                         = 0;

    // Feedforward coefficients (initialized from config, mutable for tuning)
    struct FFCoeffs
    {
        float kv, ks, ka;
    };
    FFCoeffs ff_fwd_left_, ff_fwd_right_;
};

#endif
