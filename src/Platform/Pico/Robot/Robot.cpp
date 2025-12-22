// Robot.cpp
#include "../../../Include/Platform/Pico/Robot/Robot.h"
#include "../../../Include/Common/PIDController.h"
#include "../../../Include/Platform/Pico/Config.h"
#include "../../../Include/Platform/Pico/Robot/Drivetrain.h"
#include "../../../Include/Platform/Pico/Robot/Sensors.h"

Robot::Robot(Drivetrain* drivetrain, Sensors* sensors)
    : drivetrain(drivetrain), sensors(sensors), yawPID(), leftWheelPID(), rightWheelPID()
{
    // Yaw PID: Output is wheel speed differential (mm/s)
    yawPID.setGains(YAW_KP, YAW_KI, YAW_KD);
    yawPID.setOutputLimit(maxYawDiffMMps);
    yawPID.setDeadband(1.5f);
    yawPID.setDerivativeFilterAlpha(0.9f);

    // Wheel PIDs: Output is duty correction
    leftWheelPID.setGains(LEFT_WHEEL_KP, LEFT_WHEEL_KI, LEFT_WHEEL_KD);
    rightWheelPID.setGains(RIGHT_WHEEL_KP, RIGHT_WHEEL_KI, RIGHT_WHEEL_KD);
    leftWheelPID.setOutputLimit(1.0f);
    rightWheelPID.setOutputLimit(1.0f);
    leftWheelPID.setDeadband(0.0f);
    rightWheelPID.setDeadband(0.0f);
    leftWheelPID.setDerivativeFilterAlpha(0.9f);
    rightWheelPID.setDerivativeFilterAlpha(0.9f);

    reset();
}

void Robot::reset()
{
    drivetrain->reset();
    sensors->resetYaw();

    yawPID.reset();
    leftWheelPID.reset();
    rightWheelPID.reset();

    mode = Mode::Idle;

    targetForwardMMps = 0.0f;
    targetYawDeg      = 0.0f;

    targetLeftMMps  = 0.0f;
    targetRightMMps = 0.0f;

    driveStartLeftDistMM  = 0.0f;
    driveStartRightDistMM = 0.0f;
    driveTargetDistMM     = 0.0f;

    prevLeftDuty  = 0.0f;
    prevRightDuty = 0.0f;

    motionDone = true;
}

void Robot::stop()
{
    mode = Mode::Idle;

    targetForwardMMps = 0.0f;
    targetLeftMMps    = 0.0f;
    targetRightMMps   = 0.0f;

    prevLeftDuty  = 0.0f;
    prevRightDuty = 0.0f;

    motionDone = true;
    drivetrain->stop();
}

void Robot::driveStraightMMps(float forwardMMps, float holdYawDeg)
{
    mode = Mode::DriveStraight;

    targetForwardMMps = forwardMMps;
    targetYawDeg      = wrapDeg(holdYawDeg);

    yawPID.reset();
    leftWheelPID.reset();
    rightWheelPID.reset();

    motionDone = false;
}

void Robot::driveDistanceMM(float distanceMM, float forwardMMps)
{
    if (distanceMM < 0.0f)
        distanceMM = -distanceMM;

    mode = Mode::DriveDistance;

    targetYawDeg = snapToNearest45Deg(sensors->getYaw());

    driveStartLeftDistMM  = drivetrain->getMotorDistanceMM("left");
    driveStartRightDistMM = drivetrain->getMotorDistanceMM("right");
    driveTargetDistMM     = distanceMM;

    targetForwardMMps = forwardMMps;

    yawPID.reset();
    leftWheelPID.reset();
    rightWheelPID.reset();

    motionDone = false;
}

void Robot::turnToYawDeg(float targetYawDeg)
{
    mode = Mode::TurnInPlace;

    targetForwardMMps  = 0.0f;
    this->targetYawDeg = wrapDeg(targetYawDeg);

    yawPID.reset();
    leftWheelPID.reset();
    rightWheelPID.reset();

    motionDone = false;
}

void Robot::turn45Degrees(std::string side)
{
    if (side != "left" && side != "right")
        return;

    float delta = (side == "left") ? -45.0f : +45.0f;

    float currYaw = sensors->getYaw();
    float desired = wrapDeg(currYaw + delta);

    turnToYawDeg(snapToNearest45Deg(desired));
}

bool Robot::isMotionDone() const
{
    return motionDone;
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

void Robot::setWheelVelocityTargetsMMps(float vLeftMMps, float vRightMMps)
{
    targetLeftMMps  = clampAbs(vLeftMMps, maxWheelSpeedMMps);
    targetRightMMps = clampAbs(vRightMMps, maxWheelSpeedMMps);
}

void Robot::commandBaseAndYaw(float vBaseMMps, float dt)
{
    float yaw      = sensors->getYaw();
    float yawError = wrapDeg(targetYawDeg - yaw);

    // yawPID output is differential wheel speed (mm/s)
    float vDiffMMps = yawPID.calculateOutput(yawError, dt);
    vDiffMMps       = clampAbs(vDiffMMps, maxYawDiffMMps);

    setWheelVelocityTargetsMMps(vBaseMMps - vDiffMMps, vBaseMMps + vDiffMMps);
    runWheelVelocityControl(dt);
}

void Robot::runWheelVelocityControl(float dt)
{
    // Update measured velocities
    drivetrain->updateVelocities(dt);

    // Measure wheel speeds once per tick
    float vL = drivetrain->getMotorVelocityMMps("left");
    float vR = drivetrain->getMotorVelocityMMps("right");
    // LOG_DEBUG("Measured Velocities - Left: " + std::to_string(vL) + " mm/s | Right: " +
    // std::to_string(vR) + " mm/s");

    // Feedforward (mm/s -> duty) + feedback (PID output in duty)
    float ffL = drivetrain->getFeedforward("left", targetLeftMMps);
    float ffR = drivetrain->getFeedforward("right", targetRightMMps);
    // LOG_DEBUG("FF Left: " + std::to_string(ffL) + " | FF Right: " + std::to_string(ffR));

    float eL = targetLeftMMps - vL;
    float eR = targetRightMMps - vR;

    float   fbL = leftWheelPID.calculateOutput(eL, dt);
    float   fbR = rightWheelPID.calculateOutput(eR, dt);
    static int ctr = 0;
    if ((ctr++ % 10) == 0)
    {

        LOG_DEBUG("FB Left: " + std::to_string(fbL) + " | FB Right: " + std::to_string(fbR));
    }

    float leftDuty  = ffL + fbL;
    float rightDuty = ffR + fbR;

    leftDuty  = clampAbs(leftDuty, maxDuty);
    rightDuty = clampAbs(rightDuty, maxDuty);

    leftDuty  = applySlew(leftDuty, prevLeftDuty, dt);
    rightDuty = applySlew(rightDuty, prevRightDuty, dt);

    drivetrain->setDuty(leftDuty, rightDuty);
}

void Robot::update(float dt)
{
    if (dt <= 0.0f)
        return;

    // LOG_DEBUG("Robot Update: Mode = " + std::to_string(static_cast<int>(mode)));
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
            float yaw      = sensors->getYaw();
            float yawError = wrapDeg(targetYawDeg - yaw);

            commandBaseAndYaw(0.0f, dt);

            if (std::fabs(yawError) <= yawToleranceDeg)
            {
                stop();
                motionDone = true;
            }
        }
        break;

        case Mode::DriveStraight:
        {
            commandBaseAndYaw(targetForwardMMps, dt);
            motionDone = false;
        }
        break;

        case Mode::DriveDistance:
        {
            float currLeft  = drivetrain->getMotorDistanceMM("left");
            float currRight = drivetrain->getMotorDistanceMM("right");

            float traveledLeft  = std::fabs(currLeft - driveStartLeftDistMM);
            float traveledRight = std::fabs(currRight - driveStartRightDistMM);

            float remainingLeft  = driveTargetDistMM - traveledLeft;
            float remainingRight = driveTargetDistMM - traveledRight;

            float remaining = (remainingLeft < remainingRight) ? remainingLeft : remainingRight;
            if (remaining <= 0.0f)
            {
                stop();
                motionDone = true;
                break;
            }

            float scale = (remaining < slowdownDistMM) ? (remaining / slowdownDistMM) : 1.0f;
            if (scale < minSlowdownScale)
                scale = minSlowdownScale;

            float vBase = targetForwardMMps * scale;
            commandBaseAndYaw(vBase, dt);

            motionDone = false;
        }
        break;
    }
}

void Robot::setYawGains(float kp, float ki, float kd)
{
    yawPID.setGains(kp, ki, kd);
}

void Robot::setWheelGains(float kp, float ki, float kd)
{
    leftWheelPID.setGains(kp, ki, kd);
    rightWheelPID.setGains(kp, ki, kd);
}

void Robot::setMaxDuty(float maxDuty)
{
    this->maxDuty = (maxDuty < 0.0f) ? 0.0f : ((maxDuty > 1.0f) ? 1.0f : maxDuty);
    leftWheelPID.setOutputLimit(this->maxDuty);
    rightWheelPID.setOutputLimit(this->maxDuty);
}

void Robot::setMaxDutySlewPerSec(float maxDutyChangePerSec)
{
    maxDutySlewPerSec = (maxDutyChangePerSec < 0.0f) ? 0.0f : maxDutyChangePerSec;
}

void Robot::setMaxWheelSpeedMMps(float maxWheelSpeedMMps)
{
    this->maxWheelSpeedMMps = (maxWheelSpeedMMps < 0.0f) ? 0.0f : maxWheelSpeedMMps;
}

void Robot::setMaxYawDiffMMps(float maxYawDiffMMps)
{
    this->maxYawDiffMMps = (maxYawDiffMMps < 0.0f) ? 0.0f : maxYawDiffMMps;
    yawPID.setOutputLimit(this->maxYawDiffMMps);
}

void Robot::setYawToleranceDeg(float tolDeg)
{
    yawToleranceDeg = (tolDeg < 0.0f) ? 0.0f : tolDeg;
}

void Robot::setSlowdownDistMM(float slowdownDistMM)
{
    this->slowdownDistMM = (slowdownDistMM < 1.0f) ? 1.0f : slowdownDistMM;
}

void Robot::setMinSlowdownScale(float minScale)
{
    if (minScale < 0.0f)
        minScale = 0.0f;
    if (minScale > 1.0f)
        minScale = 1.0f;
    minSlowdownScale = minScale;
}

float Robot::wrapDeg(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

float Robot::snapToNearest45Deg(float yawDeg)
{
    float snapped = 45.0f * std::round(yawDeg / 45.0f);
    snapped       = wrapDeg(snapped);
    if (snapped == -180.0f)
        snapped = 180.0f;
    return snapped;
}

float Robot::clampAbs(float value, float maxAbs)
{
    if (value > maxAbs)
        return maxAbs;
    if (value < -maxAbs)
        return -maxAbs;
    return value;
}