#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"

Drivetrain::Drivetrain(Motor* lMotor, Motor* rMotor, Encoder* lEncoder, Encoder* rEncoder)
    : leftMotor(lMotor), rightMotor(rMotor), leftEncoder(lEncoder), rightEncoder(rEncoder)
{
}

void Drivetrain::reset()
{
    leftEncoder->reset();
    rightEncoder->reset();
}

float Drivetrain::getMotorDistanceMM(std::string side)
{
    // Checks if left or right side.
    if (side != "left" && side != "right")
    {
        LOG_ERROR("Invalid side inputted for distance retrieval: " + side);
        return 0.0f;
    }

    // Calculations for distance.
    bool isLeft = (side == "left");
    return (isLeft ? leftEncoder->getTickCount() : rightEncoder->getTickCount()) * MM_PER_TICK;
}

float Drivetrain::getMotorVelocityMMps(std::string side, float dt)
{
    // Checks if left or right side.
    if (side != "left" && side != "right")
    {
        LOG_ERROR("Invalid side inputted for velocity retrieval: " + side);
        return 0.0f;
    }

    if (dt <= 0.0f)
    {
        LOG_ERROR("Invalid dt for velocity retrieval");
        return 0.0f;
    }

    // Select side
    bool isLeft = (side == "left");

    // Current ticks
    int32_t currTicks = isLeft ? leftEncoder->getTickCount() : rightEncoder->getTickCount();

    // Reference to previous ticks
    int32_t& prevTicks = isLeft ? prevLeftTicks : prevRightTicks;

    // Delta
    int32_t deltaTicks = currTicks - prevTicks;
    prevTicks          = currTicks;

    // Convert to mm/s
    return (deltaTicks * MM_PER_TICK) / dt;
}

float Drivetrain::getFeedforward(std::string side, float wheelSpeed)
{
    if (side != "left" && side != "right")
    {
        LOG_ERROR("Invalid side inputted for feedforward calculation: " + side);
        return 0.0f;
    }

    // Feedforward deadzone near zero to prevent buzzing/creep
    const float FF_DEADZONE_MMPS = 10.0f;
    if (fabs(wheelSpeed) < FF_DEADZONE_MMPS)
        return 0.0f;

    bool isLeft = (side == "left");

    if (wheelSpeed > 0.0f)
    {
        return (isLeft ? FORWARD_KVL : FORWARD_KVR) * wheelSpeed +
               (isLeft ? FORWARD_KSL : FORWARD_KSR);
    }
    else // wheelSpeed < 0.0f (deadzone handled above)
    {
        return (isLeft ? REVERSE_KVL : REVERSE_KVR) * wheelSpeed -
               (isLeft ? REVERSE_KSL : REVERSE_KSR);
    }
}

void Drivetrain::setDuty(float leftDuty, float rightDuty)
{
    leftMotor->applyDuty(leftDuty);
    rightMotor->applyDuty(rightDuty);
}

void Drivetrain::stop()
{
    leftMotor->stopMotor();
    rightMotor->stopMotor();
}