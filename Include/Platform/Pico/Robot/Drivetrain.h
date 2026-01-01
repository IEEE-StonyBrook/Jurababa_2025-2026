#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cstdint>
#include <string>

#include "Common/LogSystem.h"
#include "Platform/Pico/Robot/Encoder.h"
#include "Platform/Pico/Robot/Motor.h"
#include "Platform/Pico/Robot/RobotUtils.h"

class Drivetrain
{
  public:
    // Construct drivetrain with motors and encoders
    Drivetrain(Motor* lMotor, Motor* rMotor, Encoder* lEncoder, Encoder* rEncoder);

    // Reset encoder counts and internal state
    void reset();

    // ============= Type-Safe API (Preferred) ============= //

    /**
     * @brief Returns total distance traveled by a motor (type-safe)
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @return Distance in millimeters
     */
    float getMotorDistanceMM(WheelSide side);

    /**
     * @brief Returns motor velocity (type-safe)
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @return Velocity in millimeters per second
     */
    float getMotorVelocityMMps(WheelSide side);

    /**
     * @brief Calculates feedforward duty for target velocity (type-safe)
     * @param side Which wheel (WheelSide::LEFT or WheelSide::RIGHT)
     * @param wheelSpeed Target velocity in mm/s
     * @return Feedforward duty cycle [-1.0, 1.0]
     */
    float getFeedforward(WheelSide side, float wheelSpeed);

    // ============= Legacy String API (Deprecated) ============= //
    // These overloads maintain backward compatibility
    // TODO: Remove once all calling code is updated

    float getMotorDistanceMM(std::string side);
    float getMotorVelocityMMps(std::string side);
    float getFeedforward(std::string side, float wheelSpeed);

    // ============= Common Operations ============= //

    // Updates wheel velocities; call every control tick with dt in seconds
    void updateVelocities(float dt);

    // Sets duty cycles for both motors in range [-1.0, 1.0]
    void setDuty(float leftDuty, float rightDuty);

    // Immediately stop both motors
    void stop();

  private:
    // Motor drivers
    Motor* leftMotor;
    Motor* rightMotor;

    // Encoders attached to each wheel
    Encoder* leftEncoder;
    Encoder* rightEncoder;

    // Previous encoder ticks for velocity computation
    int32_t prevLeftTicks  = 0;
    int32_t prevRightTicks = 0;
    
    // Current wheel velocities in mm/s
    float leftVelocityMMps  = 0.0f;
    float rightVelocityMMps = 0.0f;
};

#endif