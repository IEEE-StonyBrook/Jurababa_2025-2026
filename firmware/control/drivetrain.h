#ifndef CONTROL_DRIVETRAIN_H
#define CONTROL_DRIVETRAIN_H

#include <cstdint>

#include "common/log.h"
#include "common/utils.h"
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
    float position(WheelSide side);

    /**
     * @brief Returns wheel velocity
     * @return Velocity in millimeters per second
     */
    float velocity(WheelSide side);

    /**
     * @brief Returns distance traveled since last call (incremental tracking)
     * @return Distance delta in millimeters since last call
     */
    float delta(WheelSide side);

    /**
     * @brief Calculates feedforward duty cycle for target velocity and acceleration
     *
     * Uses empirically-tuned coefficients: Kv (back-EMF), Ks (friction), Ka (inertia)
     */
    float feedforward(WheelSide side, float speed_mmps, float accel_mmps2 = 0.0f);

    /**
     * @brief Updates velocity estimates from encoder readings
     * @param dt Time step in seconds since last update
     */
    void update(float dt);

    /**
     * @brief Sets motor duty cycles
     */
    void setDuty(float left, float right);

    /**
     * @brief Sets motor voltages (scaled by current battery voltage)
     */
    void setVoltage(float left_volts, float right_volts);

    /**
     * @brief Returns current battery voltage reading
     */
    float batteryVoltage() const;

    void stop();

  private:
    Motor*   left_motor_;
    Motor*   right_motor_;
    Encoder* left_encoder_;
    Encoder* right_encoder_;
    Battery* battery_;

    int32_t prev_left_ticks_  = 0;
    int32_t prev_right_ticks_ = 0;

    float left_velocity_mmps_  = 0.0f;
    float right_velocity_mmps_ = 0.0f;

    float last_left_pos_mm_  = 0.0f;
    float last_right_pos_mm_ = 0.0f;
};

#endif
