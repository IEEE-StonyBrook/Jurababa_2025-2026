#include "Platform/Pico/Robot/Drivetrain.h"
#include "Platform/Pico/Config.h"

Drivetrain::Drivetrain(Motor* lMotor, Motor* rMotor, Encoder* lEncoder, Encoder* rEncoder)
    : leftMotor(lMotor), rightMotor(rMotor), leftEncoder(lEncoder), rightEncoder(rEncoder)
{
}

void Drivetrain::reset()
{
    leftEncoder->reset();
    rightEncoder->reset();

    prevLeftTicks  = leftEncoder->getTickCount();
    prevRightTicks = rightEncoder->getTickCount();

    leftVelocityMMps  = 0.0f;
    rightVelocityMMps = 0.0f;
}

// ============================================================
// Type-Safe API (Preferred)
// ============================================================

float Drivetrain::getMotorDistanceMM(WheelSide side)
{
    bool isLeft = (side == WheelSide::LEFT);
    return (isLeft ? leftEncoder->getTickCount() : rightEncoder->getTickCount()) * MM_PER_TICK;
}

float Drivetrain::getMotorVelocityMMps(WheelSide side)
{
    return (side == WheelSide::LEFT) ? leftVelocityMMps : rightVelocityMMps;
}

float Drivetrain::getFeedforward(WheelSide side, float wheelSpeed)
{
    // Apply deadzone to prevent motor buzzing at near-zero speeds
    if (std::fabs(wheelSpeed) < DRIVETRAIN_FF_DEADZONE_MMPS)
        return 0.0f;

    bool isLeft = (side == WheelSide::LEFT);

    // Forward direction: positive duty
    if (wheelSpeed > 0.0f)
    {
        float kv = isLeft ? FORWARD_KVL : FORWARD_KVR;
        float ks = isLeft ? FORWARD_KSL : FORWARD_KSR;
        return kv * wheelSpeed + ks;
    }
    // Reverse direction: negative duty
    else
    {
        float kv = isLeft ? REVERSE_KVL : REVERSE_KVR;
        float ks = isLeft ? REVERSE_KSL : REVERSE_KSR;
        return kv * wheelSpeed - ks;  // Note: wheelSpeed is negative, ks is positive
    }
}

// ============================================================
// Legacy String API (Deprecated - for backward compatibility)
// ============================================================

float Drivetrain::getMotorDistanceMM(std::string side)
{
    if (side == "left")
        return getMotorDistanceMM(WheelSide::LEFT);
    if (side == "right")
        return getMotorDistanceMM(WheelSide::RIGHT);

    LOG_ERROR("Drivetrain::getMotorDistanceMM - Invalid side: " + side);
    return 0.0f;
}

void Drivetrain::updateVelocities(float dt)
{
    // Ignore invalid time steps
    if (dt < DRIVETRAIN_MIN_DT)
        return;

    // Read encoder counts
    int32_t currLeftTicks  = leftEncoder->getTickCount();
    int32_t currRightTicks = rightEncoder->getTickCount();

    // Calculate deltas since last update
    int32_t deltaLeftTicks  = currLeftTicks - prevLeftTicks;
    int32_t deltaRightTicks = currRightTicks - prevRightTicks;

    prevLeftTicks  = currLeftTicks;
    prevRightTicks = currRightTicks;

    // Convert ticks to velocity (mm/s)
    leftVelocityMMps  = (deltaLeftTicks * MM_PER_TICK) / dt;
    rightVelocityMMps = (deltaRightTicks * MM_PER_TICK) / dt;

    // Sanity check for unreasonably high velocities (indicates encoder glitch or bad dt)
    if (leftVelocityMMps > DRIVETRAIN_MAX_VELOCITY_MMPS ||
        rightVelocityMMps > DRIVETRAIN_MAX_VELOCITY_MMPS)
    {
        LOG_ERROR("Drivetrain velocity spike detected | L=" +
                  std::to_string(leftVelocityMMps) + " R=" +
                  std::to_string(rightVelocityMMps) + " mm/s | deltaTicks L=" +
                  std::to_string(deltaLeftTicks) + " R=" + std::to_string(deltaRightTicks) +
                  " | dt=" + std::to_string(dt));
    }
}

float Drivetrain::getMotorVelocityMMps(std::string side)
{
    if (side == "left")
        return getMotorVelocityMMps(WheelSide::LEFT);
    if (side == "right")
        return getMotorVelocityMMps(WheelSide::RIGHT);

    LOG_ERROR("Drivetrain::getMotorVelocityMMps - Invalid side: " + side);
    return 0.0f;
}

float Drivetrain::getFeedforward(std::string side, float wheelSpeed)
{
    if (side == "left")
        return getFeedforward(WheelSide::LEFT, wheelSpeed);
    if (side == "right")
        return getFeedforward(WheelSide::RIGHT, wheelSpeed);

    LOG_ERROR("Drivetrain::getFeedforward - Invalid side: " + side);
    return 0.0f;
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