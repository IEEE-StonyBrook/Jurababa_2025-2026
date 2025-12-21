// Robot.cpp
#include "../../../Include/Platform/Pico/Robot/Robot.h"
#include "../../../Include/Common/PIDController.h"
#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../../../Include/Platform/Pico/Robot/Sensors.h"

Robot::Robot(Drivetrain* drivetrain, Sensors* sensors)
    : drivetrain(drivetrain), sensors(sensors), speedPID(), yawPID()
{
    // Starting gains (Tune on your robot)
    speedPID.setGains(0.003f, 0.0f, 0.0f);
    speedPID.setOutputLimit(1.0f);
    speedPID.setDeadband(0.0f);
    speedPID.setDerivativeFilterAlpha(0.9f);

    yawPID.setGains(0.03f, 0.0f, 0.001f);
    yawPID.setOutputLimit(1.0f);
    yawPID.setDeadband(1.5f);
    yawPID.setDerivativeFilterAlpha(0.9f);

    reset();
}

void Robot::reset()
{
    drivetrain->reset();
    sensors->resetYaw();

    speedPID.reset();
    yawPID.reset();

    mode              = Mode::Idle;
    targetForwardMMps = 0.0f;
    targetYawDeg      = 0.0f;

    driveStartDistMM  = 0.0f;
    driveTargetDistMM = 0.0f;

    prevLeftDuty  = 0.0f;
    prevRightDuty = 0.0f;

    motionDone = true;
}

void Robot::stop()
{
    mode              = Mode::Idle;
    targetForwardMMps = 0.0f;

    prevLeftDuty  = 0.0f;
    prevRightDuty = 0.0f;

    motionDone = true;
    drivetrain->stop();
}

void Robot::driveStraightMMps(float forwardMMps, float holdYawDeg)
{
    mode              = Mode::DriveStraight;
    targetForwardMMps = forwardMMps;
    targetYawDeg      = wrapDeg(holdYawDeg);

    speedPID.reset();
    yawPID.reset();

    motionDone = false;
}

void Robot::driveDistanceMM(float distanceMM, float forwardMMps)
{
    if (distanceMM < 0.0f)
        distanceMM = -distanceMM;

    mode = Mode::DriveDistance;

    // Hold the nearest 45-degree heading during the segment
    targetYawDeg = snapToNearest45Deg(sensors->getYaw());

    driveStartDistMM =
        0.5f * (drivetrain->getMotorDistanceMM("left") + drivetrain->getMotorDistanceMM("right"));
    driveTargetDistMM = distanceMM;

    targetForwardMMps = forwardMMps;

    speedPID.reset();
    yawPID.reset();

    motionDone = false;
}

void Robot::turnToYawDeg(float targetYawDeg)
{
    mode               = Mode::TurnInPlace;
    targetForwardMMps  = 0.0f;
    this->targetYawDeg = wrapDeg(targetYawDeg);

    yawPID.reset();

    motionDone = false;
}

void Robot::turn45Degrees(std::string side)
{
    if (side != "left" && side != "right")
        return;

    // Convention: "left" means -45, "right" means +45
    float delta = (side == "left") ? -45.0f : +45.0f;

    float currYaw = sensors->getYaw();
    float desired = wrapDeg(currYaw + delta);

    float snapped = snapToNearest45Deg(desired);

    turnToYawDeg(snapped);
}

float Robot::applySlew(float cmd, float& prevCmd, float dt)
{
    float maxDelta = maxDutySlewPerSec * dt;

    float delta = cmd - prevCmd;
    if (delta > maxDelta)
        delta = maxDelta;
    if (delta < -maxDelta)
        delta = -maxDelta;

    prevCmd += delta;
    return prevCmd;
}

void Robot::update(float dt)
{
    if (dt <= 0.0f)
        return;

    float yaw = sensors->getYaw();

    switch (mode)
    {
        case Mode::Idle:
        {
            drivetrain->stop();
            motionDone = true;
        }
        break;

        case Mode::TurnInPlace:
        {
            float yawError = wrapDeg(targetYawDeg - yaw);

            float uOmega = yawPID.calculateOutput(yawError, dt);
            uOmega       = clampAbs(uOmega, maxDuty);

            float leftCmd  = -uOmega;
            float rightCmd = +uOmega;

            leftCmd  = applySlew(leftCmd, prevLeftDuty, dt);
            rightCmd = applySlew(rightCmd, prevRightDuty, dt);

            drivetrain->setDuty(leftCmd, rightCmd);

            if (std::fabs(yawError) <= yawToleranceDeg)
            {
                stop();
                motionDone = true;
            }
        }
        break;

        case Mode::DriveStraight:
        {
            float vL   = drivetrain->getMotorVelocityMMps("left", dt);
            float vR   = drivetrain->getMotorVelocityMMps("right", dt);
            float vAvg = 0.5f * (vL + vR);

            // Feedforward based on desired forward wheel speed
            float ffL = drivetrain->getFeedforward("left", targetForwardMMps);
            float ffR = drivetrain->getFeedforward("right", targetForwardMMps);

            // Feedback: forward speed correction and yaw correction
            float speedError = targetForwardMMps - vAvg;
            float uV         = speedPID.calculateOutput(speedError, dt);

            float yawError = wrapDeg(targetYawDeg - yaw);
            float uOmega   = yawPID.calculateOutput(yawError, dt);

            // Mix: base feedforward + feedback
            float leftCmd  = ffL + (uV - uOmega);
            float rightCmd = ffR + (uV + uOmega);

            leftCmd  = clampAbs(leftCmd, maxDuty);
            rightCmd = clampAbs(rightCmd, maxDuty);

            leftCmd  = applySlew(leftCmd, prevLeftDuty, dt);
            rightCmd = applySlew(rightCmd, prevRightDuty, dt);

            drivetrain->setDuty(leftCmd, rightCmd);

            motionDone = false;
        }
        break;

        case Mode::DriveDistance:
        {
            float currDist = 0.5f * (drivetrain->getMotorDistanceMM("left") +
                                     drivetrain->getMotorDistanceMM("right"));
            float traveled = std::fabs(currDist - driveStartDistMM);

            float remaining = driveTargetDistMM - traveled;
            if (remaining <= 0.0f)
            {
                stop();
                motionDone = true;
                break;
            }

            // Simple slowdown near the end to reduce overshoot
            float slowdownDistMM = 80.0f;
            float scale = (remaining < slowdownDistMM) ? (remaining / slowdownDistMM) : 1.0f;
            if (scale < 0.2f)
                scale = 0.2f;

            float vL   = drivetrain->getMotorVelocityMMps("left", dt);
            float vR   = drivetrain->getMotorVelocityMMps("right", dt);
            float vAvg = 0.5f * (vL + vR);

            float targetV = targetForwardMMps * scale;

            // Feedforward based on desired forward wheel speed
            float ffL = drivetrain->getFeedforward("left", targetV);
            float ffR = drivetrain->getFeedforward("right", targetV);

            // Feedback: speed correction and yaw correction
            float speedError = targetV - vAvg;
            float uV         = speedPID.calculateOutput(speedError, dt);

            float yawError = wrapDeg(targetYawDeg - yaw);
            float uOmega   = yawPID.calculateOutput(yawError, dt);

            float leftCmd  = ffL + (uV - uOmega);
            float rightCmd = ffR + (uV + uOmega);

            leftCmd  = clampAbs(leftCmd, maxDuty);
            rightCmd = clampAbs(rightCmd, maxDuty);

            leftCmd  = applySlew(leftCmd, prevLeftDuty, dt);
            rightCmd = applySlew(rightCmd, prevRightDuty, dt);

            drivetrain->setDuty(leftCmd, rightCmd);

            motionDone = false;
        }
        break;
    }
}

bool Robot::isMotionDone() const
{
    return motionDone;
}

void Robot::setSpeedGains(float kp, float ki, float kd)
{
    speedPID.setGains(kp, ki, kd);
}

void Robot::setYawGains(float kp, float ki, float kd)
{
    yawPID.setGains(kp, ki, kd);
}

void Robot::setMaxDuty(float maxDuty)
{
    this->maxDuty = (maxDuty < 0.0f) ? 0.0f : ((maxDuty > 1.0f) ? 1.0f : maxDuty);
    speedPID.setOutputLimit(this->maxDuty);
    yawPID.setOutputLimit(this->maxDuty);
}

void Robot::setMaxDutySlewPerSec(float maxDutyChangePerSec)
{
    maxDutySlewPerSec = (maxDutyChangePerSec < 0.0f) ? 0.0f : maxDutyChangePerSec;
}

void Robot::setYawToleranceDeg(float tolDeg)
{
    yawToleranceDeg = (tolDeg < 0.0f) ? 0.0f : tolDeg;
}

float Robot::wrapDeg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

float Robot::clampAbs(float value, float maxAbs)
{
    if (value > maxAbs)
        return maxAbs;
    if (value < -maxAbs)
        return -maxAbs;
    return value;
}

float Robot::snapToNearest45Deg(float yawDeg)
{
    float snapped = 45.0f * std::round(yawDeg / 45.0f);
    snapped       = wrapDeg(snapped);

    if (snapped == -180.0f)
        snapped = 180.0f;

    return snapped;
}