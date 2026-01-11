#include "Platform/Pico/Robot/Robot.h"
#include "Common/LogSystem.h"
#include "Platform/Pico/Config.h"
#include "Platform/Pico/Robot/Drivetrain.h"
#include "Platform/Pico/Robot/Sensors.h"

Robot::Robot(Drivetrain* drivetrain, Sensors* sensors)
    : drivetrain_(drivetrain),
      sensors_(sensors),
      forwardController_(),
      rotationController_()
{
    // Configure forward controller (PD only: Ki=0 for position control)
    forwardController_.setGains(FWD_KP, 0.0f, FWD_KD);
    forwardController_.setOutputLimit(ROBOT_MAX_DUTY);
    forwardController_.setDeadband(0.0f);
    forwardController_.setDerivativeFilterAlpha(0.9f);

    // Configure rotation controller (PD only: Ki=0 for angle control)
    rotationController_.setGains(ROT_KP, 0.0f, ROT_KD);
    rotationController_.setOutputLimit(ROBOT_MAX_DUTY);
    rotationController_.setDeadband(0.0f);
    rotationController_.setDerivativeFilterAlpha(0.9f);

    reset();
}

void Robot::reset()
{
    drivetrain_->reset();
    sensors_->resetYaw();

    forwardController_.reset();
    rotationController_.reset();
    forwardProfile_.reset();
    rotationProfile_.reset();

    state_ = MotionState::Idle;

    forwardError_     = 0.0f;
    rotationError_    = 0.0f;
    prevForwardError_  = 0.0f;
    prevRotationError_ = 0.0f;

    targetForwardVelocityMMps_      = 0.0f;
    targetAngularVelocityDegps_     = 0.0f;
    targetForwardAccelerationMMps2_ = 0.0f;
    targetYawDeg_                   = 0.0f;

    prev_left_volts_  = 0.0f;
    prev_right_volts_ = 0.0f;

    motionDone_ = true;
}

// ============================================================
// Basic Motion Commands
// ============================================================

void Robot::moveDistance(float distanceMM, float maxVelocityMMps, float accelerationMMps2)
{
    state_ = MotionState::MovingForward;

    // Get current position baseline
    float currentPos = (drivetrain_->getMotorDistanceMM(WheelSide::LEFT) +
                       drivetrain_->getMotorDistanceMM(WheelSide::RIGHT)) / 2.0f;

    // Start forward motion profile
    forwardProfile_.start(distanceMM, maxVelocityMMps, accelerationMMps2, currentPos);
    rotationProfile_.reset();

    // Snap to nearest 45-degree heading for maze alignment
    targetYawDeg_ = RobotUtils::snapTo45Degrees(sensors_->getYaw());

    // Reset controllers for clean start
    forwardController_.reset();
    rotationController_.reset();
    forwardError_     = 0.0f;
    rotationError_    = 0.0f;
    prevForwardError_  = 0.0f;
    prevRotationError_ = 0.0f;

    // Initialize velocity targets
    targetForwardVelocityMMps_      = 0.0f;  // Will be updated by profile
    targetAngularVelocityDegps_     = 0.0f;  // Maintain heading
    targetForwardAccelerationMMps2_ = 0.0f;

    motionDone_ = false;
}

void Robot::turnInPlace(float degrees, float maxVelocityDegps, float accelerationDegps2)
{
    state_ = MotionState::TurningInPlace;

    // Get current angle baseline
    float currentAngle = sensors_->getYaw();

    // Start rotation profile
    rotationProfile_.start(degrees, maxVelocityDegps, accelerationDegps2, currentAngle);
    forwardProfile_.reset();

    // Reset controllers for clean start
    forwardController_.reset();
    rotationController_.reset();
    forwardError_     = 0.0f;
    rotationError_    = 0.0f;
    prevForwardError_  = 0.0f;
    prevRotationError_ = 0.0f;

    // Initialize velocity targets
    targetForwardVelocityMMps_      = 0.0f;  // No forward motion
    targetAngularVelocityDegps_     = 0.0f;  // Will be updated by profile
    targetForwardAccelerationMMps2_ = 0.0f;

    motionDone_ = false;
}

void Robot::stopAtCenter()
{
    state_ = MotionState::Stopping;

    // Reset profiles to stop motion
    forwardProfile_.reset();
    rotationProfile_.reset();

    // Set zero velocity targets (controllers remain active for position hold)
    targetForwardVelocityMMps_      = 0.0f;
    targetAngularVelocityDegps_     = 0.0f;
    targetForwardAccelerationMMps2_ = 0.0f;

    // Keep errors intact for smooth settling
    // Controllers will naturally decay errors as robot stops
}

void Robot::stop()
{
    state_ = MotionState::Idle;

    // Reset profiles
    forwardProfile_.reset();
    rotationProfile_.reset();

    // Reset all targets
    targetForwardVelocityMMps_      = 0.0f;
    targetAngularVelocityDegps_     = 0.0f;
    targetForwardAccelerationMMps2_ = 0.0f;

    // Reset controllers and errors for full stop
    forwardController_.reset();
    rotationController_.reset();
    forwardError_     = 0.0f;
    rotationError_    = 0.0f;
    prevForwardError_  = 0.0f;
    prevRotationError_ = 0.0f;

    // Stop motors immediately
    prev_left_volts_  = 0.0f;
    prev_right_volts_ = 0.0f;
    drivetrain_->stop();

    motionDone_ = true;
}

// ============================================================
// Cell Navigation (Convenience Wrappers)
// ============================================================

void Robot::moveToNextCell()
{
    moveDistance(CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
}

void Robot::turnLeft90()
{
    turnInPlace(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::turnRight90()
{
    turnInPlace(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::turnAround()
{
    // Alternate direction like mazerunner-core to reduce drift
    static bool turnRight = true;
    float angle = turnRight ? 180.0f : -180.0f;
    turnRight = !turnRight;

    turnInPlace(angle, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

// ============================================================
// Advanced Motion
// ============================================================

void Robot::smoothTurn(float degrees, float radiusMM)
{
    state_ = MotionState::SmoothTurning;

    // Calculate arc length from turn angle and radius
    float arcLengthMM = std::fabs(degrees) * (M_PI / 180.0f) * radiusMM;

    // Get current baselines
    float currentPos   = (drivetrain_->getMotorDistanceMM(WheelSide::LEFT) +
                         drivetrain_->getMotorDistanceMM(WheelSide::RIGHT)) / 2.0f;
    float currentAngle = sensors_->getYaw();

    // Start both profiles simultaneously for coordinated motion
    forwardProfile_.start(arcLengthMM, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                        ROBOT_BASE_ACCEL_MMPS2, currentPos);
    rotationProfile_.start(degrees, ROBOT_MAX_TURN_SPEED_DEGPS,
                         ROBOT_BASE_ANGULAR_ACCEL_DEGPS2, currentAngle);

    // Reset controllers for clean start
    forwardController_.reset();
    rotationController_.reset();
    forwardError_     = 0.0f;
    rotationError_    = 0.0f;
    prevForwardError_  = 0.0f;
    prevRotationError_ = 0.0f;

    // Initialize velocity targets (will be updated by profiles)
    targetForwardVelocityMMps_      = 0.0f;
    targetAngularVelocityDegps_     = 0.0f;
    targetForwardAccelerationMMps2_ = 0.0f;

    motionDone_ = false;

    LOG_DEBUG("SmoothTurn | Angle: " + std::to_string(degrees) +
              " deg | Radius: " + std::to_string(radiusMM) +
              " mm | Arc: " + std::to_string(arcLengthMM) + " mm");
}

void Robot::backToWall(float maxDistanceMM)
{
    state_ = MotionState::MovingForward;

    // Move backward slowly until wall detected or max distance reached
    float currentPos = (drivetrain_->getMotorDistanceMM(WheelSide::LEFT) +
                       drivetrain_->getMotorDistanceMM(WheelSide::RIGHT)) / 2.0f;

    // Negative distance for reverse motion
    forwardProfile_.start(-maxDistanceMM, ROBOT_BACKUP_SPEED_MMPS,
                        ROBOT_BASE_ACCEL_MMPS2, currentPos);
    rotationProfile_.reset();

    // Snap to current heading
    targetYawDeg_ = RobotUtils::snapTo45Degrees(sensors_->getYaw());

    // Reset controllers
    forwardController_.reset();
    rotationController_.reset();
    forwardError_     = 0.0f;
    rotationError_    = 0.0f;
    prevForwardError_  = 0.0f;
    prevRotationError_ = 0.0f;

    // Initialize velocity targets
    targetForwardVelocityMMps_      = 0.0f;
    targetAngularVelocityDegps_     = 0.0f;
    targetForwardAccelerationMMps2_ = 0.0f;

    motionDone_ = false;

    LOG_DEBUG("BackToWall | Max distance: " + std::to_string(maxDistanceMM) + " mm");
}

void Robot::centerWithWalls()
{
    // Use ToF sensors to adjust lateral position for centering
    // This is a position adjustment, not continuous motion

    float leftDistMM  = sensors_->getLeftDistanceMM();
    float rightDistMM = sensors_->getRightDistanceMM();

    // Calculate centering offset
    float lateralErrorMM = (rightDistMM - leftDistMM) / 2.0f;

    // Only center if walls are detected on both sides
    if (leftDistMM > TOF_MAX_RANGE_MM || rightDistMM > TOF_MAX_RANGE_MM)
    {
        LOG_DEBUG("CenterWithWalls | Cannot center - wall(s) not detected");
        return;
    }

    // TODO: Mazerunner-core uses sensor fusion for better wall following
    // Small lateral adjustment using rotation controller as steering correction
    // This adds to rotation error during next motion command
    rotationError_ += lateralErrorMM * CENTERING_CORRECTION_GAIN;

    LOG_DEBUG("CenterWithWalls | Left: " + std::to_string(leftDistMM) +
              " mm | Right: " + std::to_string(rightDistMM) +
              " mm | Offset: " + std::to_string(lateralErrorMM) + " mm");
}

// ============================================================
// Status Queries
// ============================================================

bool Robot::isMotionComplete() const
{
    return motionDone_;
}

float Robot::getRemainingDistance() const
{
    return forwardProfile_.getRemainingDistance();
}

float Robot::getRemainingAngle() const
{
    return rotationProfile_.getRemainingDistance();  // Same method for angular distance
}

// ============================================================
// Control Loop (called at 100Hz from Core1)
// ============================================================

void Robot::updateControl(float dt)
{
    if (dt <= 0.0f)
        return;

    // Update sensor measurements
    drivetrain_->updateVelocities(dt);
    sensors_->update(dt);

    // Periodic logging
    static int logCounter = 0;
    if ((logCounter++ % 100) == 0)  // Every 1 second at 100Hz
    {
        LOG_DEBUG("Robot | State: " + getStateName() +
                  " | ForwardVel: " + std::to_string(targetForwardVelocityMMps_) +
                  " mm/s | AngularVel: " + std::to_string(targetAngularVelocityDegps_) +
                  " deg/s | Yaw: " + std::to_string(sensors_->getYaw()) + " deg");
    }

    // Update motion profiles based on current state
    switch (state_)
    {
        case MotionState::MovingForward:
            updateForwardProfile(dt);
            checkForwardCompletion();
            break;

        case MotionState::TurningInPlace:
            updateRotationProfile(dt);
            checkRotationCompletion();
            break;

        case MotionState::SmoothTurning:
            updateForwardProfile(dt);
            updateRotationProfile(dt);
            checkSmoothTurnCompletion();
            break;

        case MotionState::Stopping:
            // No profile updates - just let controllers settle
            checkStoppingCompletion();
            break;

        case MotionState::Idle:
            // Do nothing
            return;
    }

    // Run position control loop (all states except Idle)
    if (state_ != MotionState::Idle)
    {
        runPositionControl(dt);
    }
}

// ============================================================
// Profile Update Methods
// ============================================================

void Robot::updateForwardProfile(float dt)
{
    // Get current position from encoders
    float currentPos = (drivetrain_->getMotorDistanceMM(WheelSide::LEFT) +
                       drivetrain_->getMotorDistanceMM(WheelSide::RIGHT)) / 2.0f;

    // Update profile
    forwardProfile_.update(currentPos, dt);

    // Update target velocity and acceleration for control loop
    targetForwardVelocityMMps_      = forwardProfile_.getTargetVelocity();
    targetForwardAccelerationMMps2_ = forwardProfile_.getTargetAcceleration();
}

void Robot::updateRotationProfile(float dt)
{
    // Get current angle from IMU
    float currentAngle = sensors_->getYaw();

    // Update profile
    rotationProfile_.update(currentAngle, dt);

    // Update target angular velocity
    targetAngularVelocityDegps_ = rotationProfile_.getTargetVelocity();
}

// ============================================================
// Completion Check Methods
// ============================================================

void Robot::checkForwardCompletion()
{
    if (forwardProfile_.isFinished())
    {
        // Check if we need to stop due to wall detection (for backToWall)
        if (state_ == MotionState::MovingForward && targetForwardVelocityMMps_ < 0.0f)
        {
            // Backing up - check for wall contact
            float frontDistMM = sensors_->getFrontDistanceMM();
            if (frontDistMM < WALL_CONTACT_THRESHOLD_MM)
            {
                stop();
                motionDone_ = true;

                // Reset odometry - we know we're BACK_WALL_TO_CENTER mm from center
                sensors_->resetPosition();
                LOG_DEBUG("BackToWall | Wall contact detected, position reset");
                return;
            }
        }

        // Normal completion
        stopAtCenter();
        motionDone_ = true;
    }
}

void Robot::checkRotationCompletion()
{
    if (rotationProfile_.isFinished())
    {
        // Additional stability check - verify angular velocity is low
        float angularVel = sensors_->getAngularVelocityDegps();

        if (std::fabs(angularVel) <= ROBOT_TURN_STABILITY_DEGPS)
        {
            stopAtCenter();
            motionDone_ = true;
        }
    }
}

void Robot::checkSmoothTurnCompletion()
{
    // Smooth turn complete when BOTH profiles finish
    if (forwardProfile_.isFinished() && rotationProfile_.isFinished())
    {
        stopAtCenter();
        motionDone_ = true;
    }
}

void Robot::checkStoppingCompletion()
{
    // Check if robot has settled
    float vLeft  = drivetrain_->getMotorVelocityMMps(WheelSide::LEFT);
    float vRight = drivetrain_->getMotorVelocityMMps(WheelSide::RIGHT);
    float vAvg   = (std::fabs(vLeft) + std::fabs(vRight)) / 2.0f;

    float angularVel = sensors_->getAngularVelocityDegps();

    // Both linear and angular velocity must be low
    if (vAvg < ROBOT_STOPPING_VELOCITY_MMPS &&
        std::fabs(angularVel) <= ROBOT_TURN_STABILITY_DEGPS)
    {
        stop();
        motionDone_ = true;
    }
}

// ============================================================
// Position Control Loop (replaces runWheelVelocityControl)
// ============================================================

void Robot::runPositionControl(float dt)
{
    // Step 1: Get encoder deltas (actual movement this cycle)
    float left_delta  = drivetrain_->getMotorDeltaMM(WheelSide::LEFT);
    float right_delta = drivetrain_->getMotorDeltaMM(WheelSide::RIGHT);
    float forward_delta = (left_delta + right_delta) / 2.0f;

    // Step 2: Get IMU delta (actual rotation this cycle)
    float rotation_delta = sensors_->getYawDelta();

    // Step 3: Update forward controller (incremental error accumulation)
    float expected_forward = targetForwardVelocityMMps_ * dt;
    forwardError_ += (expected_forward - forward_delta);

    // Step 4: Update rotation controller (incremental error accumulation)
    float expected_rotation = targetAngularVelocityDegps_ * dt;
    rotationError_ += (expected_rotation - rotation_delta);

    // TODO: Mazerunner-core adds steering correction here from wall sensors
    // rotationError_ += calculateSteeringCorrection();

    // Step 5: Calculate PD control outputs (normalized to [-1, 1])
    float forward_output  = forwardController_.calculateOutput(forwardError_, dt);
    float rotation_output = rotationController_.calculateOutput(rotationError_, dt);

    // Step 6: Combine for differential drive and scale to voltage
    float left_volts  = (forward_output - rotation_output) * MAX_VOLTAGE;
    float right_volts = (forward_output + rotation_output) * MAX_VOLTAGE;

    // Step 7: Calculate wheel velocities and accelerations for feedforward
    float wheelbase_radius = WHEEL_BASE_MM / 2.0f;
    float tangential_vel   = targetAngularVelocityDegps_ * (M_PI / 180.0f) * wheelbase_radius;

    float left_vel  = targetForwardVelocityMMps_ - tangential_vel;
    float right_vel = targetForwardVelocityMMps_ + tangential_vel;

    // Calculate accelerations (for Ka feedforward term)
    static float prev_left_vel  = 0.0f;
    static float prev_right_vel = 0.0f;
    float left_accel  = (left_vel - prev_left_vel) / dt;
    float right_accel = (right_vel - prev_right_vel) / dt;
    prev_left_vel  = left_vel;
    prev_right_vel = right_vel;

    // Step 8: Add feedforward (Kv + Ks + Ka) scaled to voltage
    float ff_left  = drivetrain_->getFeedforward(WheelSide::LEFT, left_vel, left_accel) * MAX_VOLTAGE;
    float ff_right = drivetrain_->getFeedforward(WheelSide::RIGHT, right_vel, right_accel) * MAX_VOLTAGE;

    left_volts  += ff_left;
    right_volts += ff_right;

    // Step 9: Clamp to safe voltage limits (6V max)
    left_volts  = RobotUtils::clampAbs(left_volts, MAX_VOLTAGE);
    right_volts = RobotUtils::clampAbs(right_volts, MAX_VOLTAGE);

    // Step 10: Apply slew rate limiting to prevent sudden changes
    left_volts  = applySlew(left_volts, prev_left_volts_, dt);
    right_volts = applySlew(right_volts, prev_right_volts_, dt);

    // Step 11: Send voltages to motors (scaled by battery voltage internally)
    drivetrain_->setVoltage(left_volts, right_volts);

    // Step 12: Update previous errors for derivative calculation
    prevForwardError_  = forwardError_;
    prevRotationError_ = rotationError_;
}

// ============================================================
// Helper Methods
// ============================================================

float Robot::applySlew(float cmd, float& prev_cmd, float dt)
{
    float max_delta = ROBOT_MAX_VOLTS_SLEW_PER_SEC * dt;

    float delta = cmd - prev_cmd;
    if (delta > max_delta)
        delta = max_delta;
    if (delta < -max_delta)
        delta = -max_delta;

    prev_cmd += delta;
    return prev_cmd;
}

std::string Robot::getStateName() const
{
    switch (state_)
    {
        case MotionState::Idle:           return "Idle";
        case MotionState::MovingForward:  return "MovingForward";
        case MotionState::TurningInPlace: return "TurningInPlace";
        case MotionState::SmoothTurning:  return "SmoothTurning";
        case MotionState::Stopping:       return "Stopping";
        default:                          return "Unknown";
    }
}

// ============================================================
// Controller Tuning
// ============================================================

void Robot::setForwardGains(float kp, float ki, float kd)
{
    forwardController_.setGains(kp, ki, kd);
}

void Robot::setRotationGains(float kp, float ki, float kd)
{
    rotationController_.setGains(kp, ki, kd);
}

// ============================================================
// Legacy API (Deprecated - for backward compatibility)
// ============================================================

void Robot::driveStraightMMps(float forwardMMps, float holdYawDeg)
{
    // TODO: Implement using new architecture or mark as unsupported
    LOG_ERROR("driveStraightMMps is deprecated, use moveDistance() instead");
}

void Robot::driveDistanceMM(float distanceMM, float forwardMMps)
{
    LOG_DEBUG("driveDistanceMM (deprecated) | Calling moveDistance() instead");
    moveDistance(distanceMM, forwardMMps, ROBOT_BASE_ACCEL_MMPS2);
}

void Robot::turnToYawDeg(float targetYawDeg)
{
    LOG_DEBUG("turnToYawDeg (deprecated) | Calling turnInPlace() instead");
    float currentYaw = sensors_->getYaw();
    float deltaDeg = RobotUtils::wrapAngle180(targetYawDeg - currentYaw);
    turnInPlace(deltaDeg, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::turn45Degrees(std::string side, int times)
{
    LOG_DEBUG("turn45Degrees (deprecated) | Calling turnInPlace() instead");
    float delta = (side == "left") ? -45.0f : +45.0f;
    turnInPlace(delta * static_cast<float>(times), ROBOT_MAX_TURN_SPEED_DEGPS,
                ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::arcTurnToYawDeg(float targetYawDeg, float arcLengthMM, float baseVelocityMMps)
{
    LOG_DEBUG("arcTurnToYawDeg (deprecated) | Calling smoothTurn() instead");
    float currentYaw = sensors_->getYaw();
    float deltaDeg = RobotUtils::wrapAngle180(targetYawDeg - currentYaw);

    // Estimate radius from arc length and angle
    float radiusMM = arcLengthMM / (std::fabs(deltaDeg) * (M_PI / 180.0f));
    smoothTurn(deltaDeg, radiusMM);
}

void Robot::arcTurn90Degrees(std::string side)
{
    LOG_DEBUG("arcTurn90Degrees (deprecated) | Calling smoothTurn() instead");
    float degrees = (side == "left") ? 90.0f : -90.0f;
    smoothTurn(degrees, ARC_TURN_RADIUS_MM);
}

void Robot::arcTurn45Degrees(std::string side)
{
    LOG_DEBUG("arcTurn45Degrees (deprecated) | Calling smoothTurn() instead");
    float degrees = (side == "left") ? 45.0f : -45.0f;
    smoothTurn(degrees, ARC_TURN_RADIUS_MM);
}

void Robot::update(float dt)
{
    // Wrapper for updateControl
    updateControl(dt);
}

bool Robot::isMotionDone() const
{
    // Wrapper for isMotionComplete
    return isMotionComplete();
}
