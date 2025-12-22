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

void Drivetrain::updateVelocities(float dt)
{
    if (dt <= 0.0f)
        return;

    int32_t currLeftTicks  = leftEncoder->getTickCount();
    int32_t currRightTicks = rightEncoder->getTickCount();

    int32_t deltaLeftTicks  = currLeftTicks - prevLeftTicks;
    int32_t deltaRightTicks = currRightTicks - prevRightTicks;

    prevLeftTicks  = currLeftTicks;
    prevRightTicks = currRightTicks;

    leftVelocityMMps  = (deltaLeftTicks * MM_PER_TICK) / dt;
    rightVelocityMMps = (deltaRightTicks * MM_PER_TICK) / dt;
}

float Drivetrain::getMotorVelocityMMps(std::string side)
{
    if (side == "left")
        return leftVelocityMMps;
    if (side == "right")
        return rightVelocityMMps;

    LOG_ERROR("Invalid side for velocity read: " + side);
    return 0.0f;
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