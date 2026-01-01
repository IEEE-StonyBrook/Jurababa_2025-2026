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
    yawPID.setDeadband(0.1f);
    yawPID.setDerivativeFilterAlpha(0.9f);
    yawPID.setIntegralLimit(maxYawDiffMMps * 0.2f);

    // Wheel PIDs: Output is duty correction
    leftWheelPID.setGains(LEFT_WHEEL_KP, LEFT_WHEEL_KI, LEFT_WHEEL_KD);
    rightWheelPID.setGains(RIGHT_WHEEL_KP, RIGHT_WHEEL_KI, RIGHT_WHEEL_KD);
    leftWheelPID.setOutputLimit(1.0f);
    rightWheelPID.setOutputLimit(1.0f);
    leftWheelPID.setDeadband(0.0f);
    rightWheelPID.setDeadband(0.0f);
    leftWheelPID.setDerivativeFilterAlpha(0.9f);
    rightWheelPID.setDerivativeFilterAlpha(0.9f);
    leftWheelPID.setIntegralLimit(0.2f);
    rightWheelPID.setIntegralLimit(0.2f);

    reset();
}

void Robot::reset()
{
    drivetrain->reset();
    sensors->resetYaw();

    yawPID.reset();
    leftWheelPID.reset();
    rightWheelPID.reset();

    mode     = Mode::Idle;
    prevMode = Mode::Idle;

    targetForwardMMps = 0.0f;
    targetYawDeg      = 0.0f;

    targetLeftMMps  = 0.0f;
    targetRightMMps = 0.0f;

    driveStartLeftDistMM  = 0.0f;
    driveStartRightDistMM = 0.0f;
    driveTargetDistMM     = 0.0f;

    prevLeftDuty  = 0.0f;
    prevRightDuty = 0.0f;

    vBaseCmdMMps = 0.0f;

    motionDone = true;
}

void Robot::stop()
{
    mode     = Mode::Idle;
    prevMode = Mode::Idle;

    targetForwardMMps = 0.0f;
    targetLeftMMps    = 0.0f;
    targetRightMMps   = 0.0f;

    prevLeftDuty  = 0.0f;
    prevRightDuty = 0.0f;

    vBaseCmdMMps = 0.0f;

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
    mode       = Mode::TurnInPlace;
    motionDone = false;

    targetForwardMMps  = 0.0f;
    this->targetYawDeg = wrapDeg(targetYawDeg);

    // If you don't do this, the robot will "snap" to 0 before turning
    this->currentYawSetpoint = sensors->getYaw();

    yawPID.reset();
    leftWheelPID.reset();
    rightWheelPID.reset();
}

void Robot::turn45Degrees(std::string side, int times)
{
    if (side != "left" && side != "right")
        return;

    float delta = (side == "left") ? -45.0f : +45.0f;

    float currYaw = sensors->getYaw();
    float desired = wrapDeg(currYaw + delta * static_cast<float>(times));

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

float Robot::rampToward(float desired, float current, float maxDelta)
{
    float diff = desired - current;
    if (diff > maxDelta)
        diff = maxDelta;
    if (diff < -maxDelta)
        diff = -maxDelta;
    return current + diff;
}

void Robot::commandBaseAndYaw(float vBaseMMps, float dt)
{
    // 1. Linear Velocity Profiling (Already good)
    vBaseCmdMMps = rampToward(vBaseMMps, vBaseCmdMMps, maxBaseAccelMMps2 * dt);

    // 2. Yaw Profiling: Move the setpoint toward the final targetYawDeg
    float yawToGoal  = wrapDeg(targetYawDeg - currentYawSetpoint);
    float maxStepYaw = maxAngularVelDegps * dt;

    // Increment the setpoint toward the final goal
    currentYawSetpoint += clampAbs(yawToGoal, maxStepYaw);
    currentYawSetpoint = wrapDeg(currentYawSetpoint);

    // 3. Compute error based on the PROFILE (not the final goal)
    float actualYaw        = sensors->getYaw();
    float profiledYawError = wrapDeg(currentYawSetpoint - actualYaw);

    // 4. Run PID
    float vDiffMMps = yawPID.calculateOutput(profiledYawError, dt);

    // NEW: Ensure the turn speed is high enough to actually move the wheels
    // but only if there is actually a meaningful error to correct.
    float minTurnSpeed = 250.0f; // Adjust this until the robot actually turns
    if (std::fabs(profiledYawError) > 0.5f)
    { // If error > 0.5 degrees
        if (std::fabs(vDiffMMps) < minTurnSpeed)
        {
            // Apply minimum speed in the direction the PID wants to go
            vDiffMMps = (vDiffMMps > 0) ? minTurnSpeed : -minTurnSpeed;
        }
    }

    vDiffMMps = clampAbs(vDiffMMps, maxYawDiffMMps);

    setWheelVelocityTargetsMMps(vBaseCmdMMps + vDiffMMps, vBaseCmdMMps - vDiffMMps);
    // static int ctr = 0;
    // if ((ctr++ % 25) == 0)
    // {
    //     LOG_DEBUG("Velocity Targets | Left: " + std::to_string(vBaseCmdMMps + vDiffMMps) +
    //               " mm/s | Right: " + std::to_string(vBaseCmdMMps - vDiffMMps) + " mm/s");
    // }
    runWheelVelocityControl(dt);
}

void Robot::runWheelVelocityControl(float dt)
{
    // Measure wheel speeds once per tick
    float vL = drivetrain->getMotorVelocityMMps("left");
    float vR = drivetrain->getMotorVelocityMMps("right");

    // Feedforward (mm/s -> duty) + feedback (PID output in duty)
    float ffL = drivetrain->getFeedforward("left", targetLeftMMps);
    float ffR = drivetrain->getFeedforward("right", targetRightMMps);

    float eL = targetLeftMMps - vL;
    float eR = targetRightMMps - vR;

    float fbL = leftWheelPID.calculateOutput(eL, dt);
    float fbR = rightWheelPID.calculateOutput(eR, dt);

    float leftDuty  = ffL + fbL;
    float rightDuty = ffR + fbR;
    // static int ctr       = 0;
    // if ((ctr++ % 25) == 0)
    // {
    //     LOG_DEBUG("Duty Left: " + std::to_string(leftDuty) +
    //               " | Duty Right: " + std::to_string(rightDuty));
    // }

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

    // Update measured velocities (linear and angular)
    drivetrain->updateVelocities(dt);
    sensors->update(dt);
    static int ctr = 0;
    if ((ctr++ % 10) == 0)
    {
        LOG_DEBUG(
            "Robot Update | LeftVel: " + std::to_string(drivetrain->getMotorVelocityMMps("left")) +
            " mm/s | RightVel: " + std::to_string(drivetrain->getMotorVelocityMMps("right")) +
            " mm/s | Yaw: " + std::to_string(sensors->getYaw()) +
            " deg | AngularVel: " + std::to_string(sensors->getAngularVelocityDegps()) + " deg/s");
    }

    if (mode != prevMode)
    {
        switch (mode)
        {
            case Mode::Idle:
                vBaseCmdMMps = 0.0f;
                break;

            case Mode::TurnInPlace:
                vBaseCmdMMps = 0.0f;
                break;

            case Mode::DriveStraight:
            case Mode::DriveDistance:
                // Only reset when coming from Idle/Turn. Otherwise preserve for smooth changes.
                if (prevMode == Mode::Idle || prevMode == Mode::TurnInPlace)
                    vBaseCmdMMps = 0.0f;
                break;
        }
        prevMode = mode;
    }

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
            // Update the motor targets using the profile logic above
            commandBaseAndYaw(0.0f, dt);

            float actualYaw  = sensors->getYaw();
            float yawError   = wrapDeg(targetYawDeg - actualYaw);
            float angularVel = sensors->getAngularVelocityDegps();
            static int ctr        = 0;
            if ((ctr++ % 25) == 0)
            {
                LOG_DEBUG("TurnInPlace | TargetYaw: " + std::to_string(targetYawDeg) +
                          " | ActualYaw: " + std::to_string(actualYaw) +
                          " | YawError: " + std::to_string(yawError) +
                          " | AngularVel: " + std::to_string(angularVel));
            }

            // Condition 1: Are we at the degree?
            bool positionReached = std::fabs(yawError) <= yawToleranceDeg;

            // Condition 2: Have we stopped swinging?
            bool isStable = std::fabs(angularVel) <= 3.0f; // 3 deg/s threshold

            if (positionReached && isStable)
            {
                motionDone = true;
                // We keep commandBaseAndYaw running so it "holds" the position
                // until the mouse decides its next move.
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

            float traveled = (std::fabs(currLeft - driveStartLeftDistMM) +
                              std::fabs(currRight - driveStartRightDistMM)) /
                             2.0f;

            float remaining = driveTargetDistMM - traveled;

            if (remaining <= 0.5f)
            { // Small threshold
                stop();
                motionDone = true;
                break;
            }

            // Physics: v^2 = u^2 + 2as -> v = sqrt(2 * a * distance)
            // This calculates the maximum speed you can be going and still stop in time
            float vMaxPossible = std::sqrt(2.0f * maxBaseAccelMMps2 * remaining);

            // Target is the lesser of our desired cruise speed or the physical limit
            float vBase = std::min(targetForwardMMps, vMaxPossible);

            // Ensure we don't drop below a minimum speed that would stall the motors
            if (vBase < minVelocityMMps)
            {
                vBase = minVelocityMMps;
            }
            static int ctr = 0;
            if (ctr++ % 25 == 0)
            {
                LOG_DEBUG("DriveDistance | Target: " + std::to_string(driveTargetDistMM) +
                          " mm | Traveled: " + std::to_string(traveled) +
                          " mm | Remaining: " + std::to_string(remaining) + " mm");
                LOG_DEBUG("vBase set to " + std::to_string(vBase) + " mm/s");
            }

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