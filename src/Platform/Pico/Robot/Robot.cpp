#include "Platform/Pico/Robot/Robot.h"
#include "Common/LogSystem.h"
#include "Common/PIDController.h"
#include "Platform/Pico/Config.h"
#include "Platform/Pico/Robot/Drivetrain.h"
#include "Platform/Pico/Robot/Sensors.h"

Robot::Robot(Drivetrain* drivetrain, Sensors* sensors)
    : drivetrain(drivetrain),
      sensors(sensors),
      yawPID(),
      leftWheelPID(),
      rightWheelPID(),
      // Load configuration from Config.h
      yawToleranceDeg(ROBOT_YAW_TOLERANCE_DEG),
      maxDuty(ROBOT_MAX_DUTY),
      maxDutySlewPerSec(ROBOT_MAX_DUTY_SLEW_PER_SEC),
      maxWheelSpeedMMps(ROBOT_MAX_WHEEL_SPEED_MMPS),
      maxYawDiffMMps(ROBOT_MAX_YAW_DIFF_MMPS),
      maxAngularVelDegps(ROBOT_MAX_ANGULAR_VEL_DEGPS),
      minCruiseVelocityMMps(ROBOT_MIN_CRUISE_VELOCITY_MMPS),
      finalApproachSpeedMMps(ROBOT_FINAL_APPROACH_SPEED_MMPS),
      maxBaseAccelMMps2(ROBOT_BASE_ACCEL_MMPS2)
{
    // Configure yaw PID (output is wheel speed differential in mm/s)
    yawPID.setGains(YAW_KP, YAW_KI, YAW_KD);
    yawPID.setOutputLimit(maxYawDiffMMps);
    yawPID.setDeadband(0.1f);
    yawPID.setDerivativeFilterAlpha(0.9f);
    yawPID.setIntegralLimit(maxYawDiffMMps * 0.2f);

    // Configure wheel velocity PIDs (output is duty cycle correction)
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
    targetYawDeg      = RobotUtils::wrapAngle180(holdYawDeg);

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

    // Snap to nearest 45-degree heading for maze alignment
    targetYawDeg = RobotUtils::snapTo45Degrees(sensors->getYaw());

    // Record starting position for distance tracking (using type-safe API)
    driveStartLeftDistMM  = drivetrain->getMotorDistanceMM(WheelSide::LEFT);
    driveStartRightDistMM = drivetrain->getMotorDistanceMM(WheelSide::RIGHT);
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
    this->targetYawDeg = RobotUtils::wrapAngle180(targetYawDeg);

    // Initialize yaw setpoint to current position to avoid sudden snap
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
    float desired = RobotUtils::wrapAngle180(currYaw + delta * static_cast<float>(times));

    turnToYawDeg(RobotUtils::snapTo45Degrees(desired));
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
    targetLeftMMps  = RobotUtils::clampAbs(vLeftMMps, maxWheelSpeedMMps);
    targetRightMMps = RobotUtils::clampAbs(vRightMMps, maxWheelSpeedMMps);
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

float Robot::applyYawProfiling(float dt)
{
    // Profile yaw setpoint toward final target (prevents sudden turns)
    float yawToGoal  = RobotUtils::wrapAngle180(targetYawDeg - currentYawSetpoint);
    float maxStepYaw = maxAngularVelDegps * dt;

    currentYawSetpoint += RobotUtils::clampAbs(yawToGoal, maxStepYaw);
    currentYawSetpoint = RobotUtils::wrapAngle180(currentYawSetpoint);

    // Calculate yaw error from profiled setpoint (not final target)
    float actualYaw        = sensors->getYaw();
    float profiledYawError = RobotUtils::wrapAngle180(currentYawSetpoint - actualYaw);

    return profiledYawError;
}

float Robot::calculateYawCorrection(float yawError, float dt)
{
    // Run PID to get yaw correction (output is wheel speed differential)
    float vDiffMMps = yawPID.calculateOutput(yawError, dt);

    // Enforce minimum turn speed to overcome static friction
    // Only apply when error is significant and PID output is non-zero but below threshold
    if (std::fabs(yawError) > ROBOT_YAW_ERROR_THRESHOLD_DEG)
    {
        if (std::fabs(vDiffMMps) < ROBOT_MIN_TURN_SPEED_MMPS && std::fabs(vDiffMMps) > 1.0f)
        {
            // Maintain direction but boost to minimum
            vDiffMMps = (vDiffMMps > 0) ? ROBOT_MIN_TURN_SPEED_MMPS : -ROBOT_MIN_TURN_SPEED_MMPS;
        }
    }

    // Clamp differential to safe limits
    return RobotUtils::clampAbs(vDiffMMps, maxYawDiffMMps);
}

void Robot::commandBaseAndYaw(float vBaseMMps, float dt)
{
    // Step 1: Ramp base velocity smoothly toward target
    vBaseCmdMMps = rampToward(vBaseMMps, vBaseCmdMMps, maxBaseAccelMMps2 * dt);

    // Step 2: Profile yaw and calculate error
    float yawError = applyYawProfiling(dt);

    // Step 3: Calculate yaw correction (PID + minimum speed enforcement)
    float vDiffMMps = calculateYawCorrection(yawError, dt);

    // Step 4: Convert to individual wheel targets and execute
    setWheelVelocityTargetsMMps(vBaseCmdMMps + vDiffMMps, vBaseCmdMMps - vDiffMMps);
    runWheelVelocityControl(dt);
}

void Robot::runWheelVelocityControl(float dt)
{
    // Measure current wheel velocities (using type-safe API)
    float vLeft  = drivetrain->getMotorVelocityMMps(WheelSide::LEFT);
    float vRight = drivetrain->getMotorVelocityMMps(WheelSide::RIGHT);

    // Calculate feedforward duty from target velocities (using type-safe API)
    float ffLeft  = drivetrain->getFeedforward(WheelSide::LEFT, targetLeftMMps);
    float ffRight = drivetrain->getFeedforward(WheelSide::RIGHT, targetRightMMps);

    // Calculate velocity errors for PID
    float errorLeft  = targetLeftMMps - vLeft;
    float errorRight = targetRightMMps - vRight;

    // Calculate PID corrections (feedback)
    float fbLeft  = leftWheelPID.calculateOutput(errorLeft, dt);
    float fbRight = rightWheelPID.calculateOutput(errorRight, dt);

    // Combine feedforward + feedback
    float leftDuty  = ffLeft + fbLeft;
    float rightDuty = ffRight + fbRight;

    // Clamp to safe duty cycle limits
    leftDuty  = RobotUtils::clampAbs(leftDuty, maxDuty);
    rightDuty = RobotUtils::clampAbs(rightDuty, maxDuty);

    // Apply slew rate limiting to prevent sudden changes
    leftDuty  = applySlew(leftDuty, prevLeftDuty, dt);
    rightDuty = applySlew(rightDuty, prevRightDuty, dt);

    // Send duty cycles to motors
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

    // Dispatch to appropriate mode handler
    switch (mode)
    {
        case Mode::Idle:
            handleIdleMode(dt);
            break;

        case Mode::TurnInPlace:
            handleTurnInPlaceMode(dt);
            break;

        case Mode::DriveStraight:
            handleDriveStraightMode(dt);
            break;

        case Mode::DriveDistance:
            handleDriveDistanceMode(dt);
            break;
    }
}

// ============================================================
// Helper: Calculate Target Velocity for Distance-Based Motion
// ============================================================

float Robot::calculateTargetVelocityForDistance(float remaining, float vActual)
{
    // Calculate stopping distance from minimum cruise speed to zero
    float stoppingDistFromMin = RobotUtils::calculateStoppingDistance(
        minCruiseVelocityMMps, 0.0f, maxBaseAccelMMps2
    );

    float vBase;

    if (remaining > stoppingDistFromMin)
    {
        // Phase 1: Far from target - decelerate to minimum cruise speed
        // Physics: Can travel at speed that allows stopping at the transition point
        vBase = RobotUtils::calculateMaxVelocityForDistance(
            remaining - stoppingDistFromMin, maxBaseAccelMMps2, minCruiseVelocityMMps
        );
        vBase = std::min(targetForwardMMps, vBase);
    }
    else
    {
        // Phase 2: Close to target - decelerate from minimum cruise to final approach speed
        vBase = RobotUtils::calculateMaxVelocityForDistance(
            remaining, maxBaseAccelMMps2, 0.0f
        );

        // Enforce minimum final approach speed to prevent stalling
        if (vBase < finalApproachSpeedMMps)
        {
            vBase = finalApproachSpeedMMps;
        }
    }

    return vBase;
}

// ============================================================
// Mode Handlers
// ============================================================

void Robot::handleIdleMode(float dt)
{
    (void)dt;  // Unused in idle mode
    drivetrain->stop();
    motionDone = true;
}

void Robot::handleTurnInPlaceMode(float dt)
{
    // Execute yaw control (base velocity = 0)
    commandBaseAndYaw(0.0f, dt);

    // Check completion criteria
    float actualYaw  = sensors->getYaw();
    float yawError   = RobotUtils::wrapAngle180(targetYawDeg - actualYaw);
    float angularVel = sensors->getAngularVelocityDegps();

    static int logCounter = 0;
    if ((logCounter++ % 25) == 0)
    {
        LOG_DEBUG("TurnInPlace | TargetYaw: " + std::to_string(targetYawDeg) +
                  " | ActualYaw: " + std::to_string(actualYaw) +
                  " | YawError: " + std::to_string(yawError) +
                  " | AngularVel: " + std::to_string(angularVel));
    }

    // Motion is done when both position and angular velocity criteria are met
    bool positionReached = std::fabs(yawError) <= yawToleranceDeg;
    bool isStable = std::fabs(angularVel) <= ROBOT_TURN_STABILITY_DEGPS;

    if (positionReached && isStable)
    {
        motionDone = true;
        // Continue holding position until next command
    }
}

void Robot::handleDriveStraightMode(float dt)
{
    commandBaseAndYaw(targetForwardMMps, dt);
    motionDone = false;  // Continuous mode, never auto-completes
}

void Robot::handleDriveDistanceMode(float dt)
{
    // Measure distance traveled (using type-safe API)
    float currLeft  = drivetrain->getMotorDistanceMM(WheelSide::LEFT);
    float currRight = drivetrain->getMotorDistanceMM(WheelSide::RIGHT);

    float traveled = (std::fabs(currLeft - driveStartLeftDistMM) +
                      std::fabs(currRight - driveStartRightDistMM)) / 2.0f;

    float remaining = driveTargetDistMM - traveled;

    // Measure actual velocity (using type-safe API)
    float vLeft  = drivetrain->getMotorVelocityMMps(WheelSide::LEFT);
    float vRight = drivetrain->getMotorVelocityMMps(WheelSide::RIGHT);
    float vActual = (std::fabs(vLeft) + std::fabs(vRight)) / 2.0f;

    // Check completion: close to target AND moving slowly
    if (remaining <= ROBOT_STOPPING_DISTANCE_MM && vActual < ROBOT_STOPPING_VELOCITY_MMPS)
    {
        stop();
        motionDone = true;
        return;
    }

    // Calculate target velocity using two-phase deceleration profile
    float vBase = calculateTargetVelocityForDistance(remaining, vActual);

    static int logCounter = 0;
    if (logCounter++ % 25 == 0)
    {
        LOG_DEBUG("DriveDistance | Target: " + std::to_string(driveTargetDistMM) +
                  " mm | Traveled: " + std::to_string(traveled) +
                  " mm | Remaining: " + std::to_string(remaining) + " mm");
        LOG_DEBUG("vBase: " + std::to_string(vBase) + " mm/s | vActual: " +
                  std::to_string(vActual) + " mm/s");
    }

    commandBaseAndYaw(vBase, dt);
    motionDone = false;
}

// ============================================================
// Setters / Configuration
// ============================================================

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

// Note: wrapAngle180(), snapTo45Degrees(), and clampAbs() are now in RobotUtils.h