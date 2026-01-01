#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cstdint>
#include <string>

#include "Common/LogSystem.h"
#include "Platform/Pico/Robot/Encoder.h"
#include "Platform/Pico/Robot/Motor.h"
#include "Platform/Pico/Robot/RobotUtils.h"

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
     * @param left_motor Pointer to left motor controller
     * @param right_motor Pointer to right motor controller
     * @param left_encoder Pointer to left wheel encoder
     * @param right_encoder Pointer to right wheel encoder
     */
    Drivetrain(Motor* left_motor, Motor* right_motor,
               Encoder* left_encoder, Encoder* right_encoder);

    /**
     * @brief Resets encoder counts and velocity state
     */
    void reset();

    // ============= Type-Safe API (Preferred) ============= //

    /**
     * @brief Returns total distance traveled by a wheel
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @return Distance in millimeters
     */
    float getMotorDistanceMM(WheelSide side);

    /**
     * @brief Returns wheel velocity
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @return Velocity in millimeters per second
     */
    float getMotorVelocityMMps(WheelSide side);

    /**
     * @brief Calculates feedforward duty cycle for target velocity
     *
     * Uses empirically-tuned velocity gain (Kv) and static friction (Ks)
     * coefficients to compensate for motor nonlinearities.
     *
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @param wheel_speed_mm_per_second Target velocity in mm/s
     * @return Feedforward duty cycle [-1.0, 1.0]
     */
    float getFeedforward(WheelSide side, float wheel_speed_mm_per_second);

    // ============= Legacy String API (Deprecated) ============= //
    // These overloads maintain backward compatibility
    // TODO: Remove once all calling code is updated to use enum-based API

    float getMotorDistanceMM(std::string side);
    float getMotorVelocityMMps(std::string side);
    float getFeedforward(std::string side, float wheel_speed_mm_per_second);

    // ============= Common Operations ============= //

    /**
     * @brief Updates velocity estimates from encoder readings
     * @param time_delta Time step in seconds since last update
     */
    void updateVelocities(float time_delta);

    /**
     * @brief Sets motor duty cycles
     * @param left_duty Left motor duty cycle [-1.0, 1.0]
     * @param right_duty Right motor duty cycle [-1.0, 1.0]
     */
    void setDuty(float left_duty, float right_duty);

    /**
     * @brief Immediately stops both motors
     */
    void stop();

  private:
    // Motor controllers
    Motor* left_motor_;
    Motor* right_motor_;

    // Wheel encoders
    Encoder* left_encoder_;
    Encoder* right_encoder_;

    // Previous encoder readings for velocity computation
    int32_t previous_left_ticks_ = 0;
    int32_t previous_right_ticks_ = 0;

    // Current velocity estimates in mm/s
    float left_velocity_mm_per_second_ = 0.0f;
    float right_velocity_mm_per_second_ = 0.0f;
};

#endif