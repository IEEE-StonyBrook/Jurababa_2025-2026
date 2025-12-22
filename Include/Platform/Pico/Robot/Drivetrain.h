#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cstdint>
#include <string>

#include "../../../Include/Common/LogSystem.h"
#include "../../../Include/Platform/Pico/Robot/Encoder.h"
#include "../../../Include/Platform/Pico/Robot/Motor.h"

class Drivetrain
{
  public:
    // Construct drivetrain with motors and encoders
    Drivetrain(Motor* lMotor, Motor* rMotor, Encoder* lEncoder, Encoder* rEncoder);

    // Reset encoder counts and internal state
    void reset();

    // Returns total distance traveled by a motor in millimeters
    // Side must be "left" or "right"
    float getMotorDistanceMM(std::string side);

    // Updates wheel velocities; call every control tick with dt in seconds
    void updateVelocities(float dt);

    // Returns motor velocity in millimeters per second
    // Side must be "left" or "right"
    float getMotorVelocityMMps(std::string side);

    // Feedforward term for motor control
    // Converts desired wheel speed into base motor command
    float getFeedforward(std::string side, float wheelSpeed);

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