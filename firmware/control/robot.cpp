#include "control/robot.h"

#include "common/log.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "drivers/imu.h"
#include "drivers/tof.h"

namespace
{
float normalizeYawDelta(float delta)
{
    if (delta > 180.0f)
        return delta - 360.0f;
    if (delta < -180.0f)
        return delta + 360.0f;
    return delta;
}
} // namespace

Robot::Robot(Drivetrain* drivetrain, IMU* imu, ToF* left_tof, ToF* front_tof, ToF* right_tof)
    : drivetrain_(drivetrain), imu_(imu), left_tof_(left_tof), front_tof_(front_tof),
      right_tof_(right_tof), forward_controller_(), rotation_controller_()
{
    forward_controller_.setGains(FWD_KP, 0.0f, FWD_KD);
    forward_controller_.setOutputLimit(ROBOT_MAX_DUTY);
    forward_controller_.setDeadband(0.0f);
    forward_controller_.setDerivativeFilterAlpha(0.9f);

    rotation_controller_.setGains(ROT_KP, 0.0f, ROT_KD);
    rotation_controller_.setOutputLimit(ROBOT_MAX_DUTY);
    rotation_controller_.setDeadband(0.0f);
    rotation_controller_.setDerivativeFilterAlpha(0.9f);

    reset();
}

void Robot::reset()
{
    drivetrain_->reset();
    resetYaw();

    forward_controller_.reset();
    rotation_controller_.reset();
    forward_profile_.reset();
    rotation_profile_.reset();

    state_ = MotionState::Idle;

    forward_error_       = 0.0f;
    rotation_error_      = 0.0f;
    prev_forward_error_  = 0.0f;
    prev_rotation_error_ = 0.0f;

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;
    target_yaw_deg_             = 0.0f;

    prev_left_volts_  = 0.0f;
    prev_right_volts_ = 0.0f;

    motion_done_ = true;
}

// === Sensor Methods (merged from Sensors class) ===

bool Robot::wallLeft()
{
    return left_tof_->distance() < TOF_LEFT_WALL_THRESHOLD_MM;
}

bool Robot::wallFront()
{
    return front_tof_->distance() < TOF_FRONT_WALL_THRESHOLD_MM;
}

bool Robot::wallRight()
{
    return right_tof_->distance() < TOF_RIGHT_WALL_THRESHOLD_MM;
}

float Robot::yaw()
{
    return imu_->yaw();
}

float Robot::omega()
{
    return omega_degps_;
}

float Robot::yawDelta()
{
    float current       = yaw();
    float delta         = normalizeYawDelta(current - last_yaw_for_delta_);
    last_yaw_for_delta_ = current;
    return delta;
}

void Robot::resetYaw()
{
    imu_->resetYaw();
    last_yaw_for_delta_ = 0.0f;
    prev_yaw_           = 0.0f;
    omega_degps_        = 0.0f;
}

float Robot::frontDistance()
{
    return front_tof_->distance();
}

float Robot::leftDistance()
{
    return left_tof_->distance();
}

float Robot::rightDistance()
{
    return right_tof_->distance();
}

void Robot::updateSensors(float dt)
{
    if (dt < 0.0001f)
        return;

    float current   = yaw();
    float delta     = normalizeYawDelta(current - prev_yaw_);
    float raw_omega = delta / dt;

    float alpha  = SENSORS_ANGULAR_VEL_FILTER_ALPHA;
    omega_degps_ = alpha * raw_omega + (1.0f - alpha) * omega_degps_;

    prev_yaw_ = current;
}

// === Motion Commands ===

void Robot::moveDistance(float distance_mm, float max_vel_mmps, float accel_mmps2)
{
    state_ = MotionState::MovingForward;

    float current_pos =
        (drivetrain_->position(WheelSide::LEFT) + drivetrain_->position(WheelSide::RIGHT)) / 2.0f;

    forward_profile_.start(distance_mm, max_vel_mmps, accel_mmps2, current_pos);
    rotation_profile_.reset();

    target_yaw_deg_ = utils::snapTo45(yaw());

    forward_controller_.reset();
    rotation_controller_.reset();
    forward_error_       = 0.0f;
    rotation_error_      = 0.0f;
    prev_forward_error_  = 0.0f;
    prev_rotation_error_ = 0.0f;

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;

    motion_done_ = false;
}

void Robot::turnInPlace(float degrees, float max_vel_degps, float accel_degps2)
{
    state_ = MotionState::TurningInPlace;

    float current_angle = yaw();

    rotation_profile_.start(degrees, max_vel_degps, accel_degps2, current_angle);
    forward_profile_.reset();

    forward_controller_.reset();
    rotation_controller_.reset();
    forward_error_       = 0.0f;
    rotation_error_      = 0.0f;
    prev_forward_error_  = 0.0f;
    prev_rotation_error_ = 0.0f;

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;

    motion_done_ = false;
}

void Robot::stopAtCenter()
{
    state_ = MotionState::Stopping;

    forward_profile_.reset();
    rotation_profile_.reset();

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;
}

void Robot::stop()
{
    state_ = MotionState::Idle;

    forward_profile_.reset();
    rotation_profile_.reset();

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;

    forward_controller_.reset();
    rotation_controller_.reset();
    forward_error_       = 0.0f;
    rotation_error_      = 0.0f;
    prev_forward_error_  = 0.0f;
    prev_rotation_error_ = 0.0f;

    prev_left_volts_  = 0.0f;
    prev_right_volts_ = 0.0f;
    drivetrain_->stop();

    motion_done_ = true;
}

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
    static bool turn_right = true;
    float       angle      = turn_right ? 180.0f : -180.0f;
    turn_right             = !turn_right;

    turnInPlace(angle, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::smoothTurn(float degrees, float radius_mm)
{
    state_ = MotionState::SmoothTurning;

    float arc_length_mm = std::fabs(degrees) * (M_PI / 180.0f) * radius_mm;

    float current_pos =
        (drivetrain_->position(WheelSide::LEFT) + drivetrain_->position(WheelSide::RIGHT)) / 2.0f;
    float current_angle = yaw();

    forward_profile_.start(arc_length_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2,
                           current_pos);
    rotation_profile_.start(degrees, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2,
                            current_angle);

    forward_controller_.reset();
    rotation_controller_.reset();
    forward_error_       = 0.0f;
    rotation_error_      = 0.0f;
    prev_forward_error_  = 0.0f;
    prev_rotation_error_ = 0.0f;

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;

    motion_done_ = false;

    LOG_DEBUG("SmoothTurn | Angle: " + std::to_string(degrees) + " deg | Radius: " +
              std::to_string(radius_mm) + " mm | Arc: " + std::to_string(arc_length_mm) + " mm");
}

void Robot::backToWall(float max_distance_mm)
{
    state_ = MotionState::MovingForward;

    float current_pos =
        (drivetrain_->position(WheelSide::LEFT) + drivetrain_->position(WheelSide::RIGHT)) / 2.0f;

    forward_profile_.start(-max_distance_mm, ROBOT_BACKUP_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2,
                           current_pos);
    rotation_profile_.reset();

    target_yaw_deg_ = utils::snapTo45(yaw());

    forward_controller_.reset();
    rotation_controller_.reset();
    forward_error_       = 0.0f;
    rotation_error_      = 0.0f;
    prev_forward_error_  = 0.0f;
    prev_rotation_error_ = 0.0f;

    target_forward_vel_mmps_    = 0.0f;
    target_angular_vel_degps_   = 0.0f;
    target_forward_accel_mmps2_ = 0.0f;

    motion_done_ = false;

    LOG_DEBUG("BackToWall | Max distance: " + std::to_string(max_distance_mm) + " mm");
}

void Robot::centerWithWalls()
{
    float left_dist  = leftDistance();
    float right_dist = rightDistance();

    float lateral_error = (right_dist - left_dist) / 2.0f;

    if (left_dist > TOF_MAX_RANGE_MM || right_dist > TOF_MAX_RANGE_MM)
    {
        LOG_DEBUG("CenterWithWalls | Cannot center - wall(s) not detected");
        return;
    }

    rotation_error_ += lateral_error * CENTERING_CORRECTION_GAIN;

    LOG_DEBUG("CenterWithWalls | Left: " + std::to_string(left_dist) +
              " mm | Right: " + std::to_string(right_dist) +
              " mm | Offset: " + std::to_string(lateral_error) + " mm");
}

bool Robot::motionComplete() const
{
    return motion_done_;
}

float Robot::remainingDistance() const
{
    return forward_profile_.remaining();
}

float Robot::remainingAngle() const
{
    return rotation_profile_.remaining();
}

void Robot::updateControl(float dt)
{
    if (dt <= 0.0f)
        return;

    drivetrain_->update(dt);
    updateSensors(dt);

    static int log_counter = 0;
    if ((log_counter++ % 100) == 0)
    {
        LOG_DEBUG("Robot | State: " + stateName() +
                  " | ForwardVel: " + std::to_string(target_forward_vel_mmps_) +
                  " mm/s | AngularVel: " + std::to_string(target_angular_vel_degps_) +
                  " deg/s | Yaw: " + std::to_string(yaw()) + " deg");
    }

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
            checkStoppingCompletion();
            break;

        case MotionState::Idle:
            return;
    }

    if (state_ != MotionState::Idle)
    {
        runPositionControl(dt);
    }
}

void Robot::updateForwardProfile(float dt)
{
    float current_pos =
        (drivetrain_->position(WheelSide::LEFT) + drivetrain_->position(WheelSide::RIGHT)) / 2.0f;

    forward_profile_.update(current_pos, dt);

    target_forward_vel_mmps_    = forward_profile_.velocity();
    target_forward_accel_mmps2_ = forward_profile_.acceleration();
}

void Robot::updateRotationProfile(float dt)
{
    float current_angle = yaw();

    rotation_profile_.update(current_angle, dt);

    target_angular_vel_degps_ = rotation_profile_.velocity();
}

void Robot::checkForwardCompletion()
{
    if (forward_profile_.finished())
    {
        if (state_ == MotionState::MovingForward && target_forward_vel_mmps_ < 0.0f)
        {
            float front_dist = frontDistance();
            if (front_dist < WALL_CONTACT_THRESHOLD_MM)
            {
                stop();
                motion_done_        = true;
                last_yaw_for_delta_ = yaw(); // Reset position tracking
                LOG_DEBUG("BackToWall | Wall contact detected, position reset");
                return;
            }
        }

        stopAtCenter();
        motion_done_ = true;
    }
}

void Robot::checkRotationCompletion()
{
    if (rotation_profile_.finished())
    {
        float angular_vel = omega();

        if (std::fabs(angular_vel) <= ROBOT_TURN_STABILITY_DEGPS)
        {
            stopAtCenter();
            motion_done_ = true;
        }
    }
}

void Robot::checkSmoothTurnCompletion()
{
    if (forward_profile_.finished() && rotation_profile_.finished())
    {
        stopAtCenter();
        motion_done_ = true;
    }
}

void Robot::checkStoppingCompletion()
{
    float v_left  = drivetrain_->velocity(WheelSide::LEFT);
    float v_right = drivetrain_->velocity(WheelSide::RIGHT);
    float v_avg   = (std::fabs(v_left) + std::fabs(v_right)) / 2.0f;

    float angular_vel = omega();

    if (v_avg < ROBOT_STOPPING_VELOCITY_MMPS &&
        std::fabs(angular_vel) <= ROBOT_TURN_STABILITY_DEGPS)
    {
        stop();
        motion_done_ = true;
    }
}

void Robot::runPositionControl(float dt)
{
    float left_delta    = drivetrain_->delta(WheelSide::LEFT);
    float right_delta   = drivetrain_->delta(WheelSide::RIGHT);
    float forward_delta = (left_delta + right_delta) / 2.0f;

    float rotation_delta = yawDelta();

    float expected_forward = target_forward_vel_mmps_ * dt;
    forward_error_ += (expected_forward - forward_delta);

    float expected_rotation = target_angular_vel_degps_ * dt;
    rotation_error_ += (expected_rotation - rotation_delta);

    float forward_output  = forward_controller_.compute(forward_error_, dt);
    float rotation_output = rotation_controller_.compute(rotation_error_, dt);

    float left_volts  = (forward_output - rotation_output) * MAX_VOLTAGE;
    float right_volts = (forward_output + rotation_output) * MAX_VOLTAGE;

    float wheelbase_radius = WHEEL_BASE_MM / 2.0f;
    float tangential_vel   = target_angular_vel_degps_ * (M_PI / 180.0f) * wheelbase_radius;

    float left_vel  = target_forward_vel_mmps_ - tangential_vel;
    float right_vel = target_forward_vel_mmps_ + tangential_vel;

    static float prev_left_vel  = 0.0f;
    static float prev_right_vel = 0.0f;
    float        left_accel     = (left_vel - prev_left_vel) / dt;
    float        right_accel    = (right_vel - prev_right_vel) / dt;
    prev_left_vel               = left_vel;
    prev_right_vel              = right_vel;

    float ff_left = drivetrain_->feedforward(WheelSide::LEFT, left_vel, left_accel) * MAX_VOLTAGE;
    float ff_right =
        drivetrain_->feedforward(WheelSide::RIGHT, right_vel, right_accel) * MAX_VOLTAGE;

    left_volts += ff_left;
    right_volts += ff_right;

    left_volts  = utils::clampAbs(left_volts, MAX_VOLTAGE);
    right_volts = utils::clampAbs(right_volts, MAX_VOLTAGE);

    left_volts  = applySlew(left_volts, prev_left_volts_, dt);
    right_volts = applySlew(right_volts, prev_right_volts_, dt);

    drivetrain_->setVoltage(left_volts, right_volts);

    prev_forward_error_  = forward_error_;
    prev_rotation_error_ = rotation_error_;
}

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

std::string Robot::stateName() const
{
    switch (state_)
    {
        case MotionState::Idle:
            return "Idle";
        case MotionState::MovingForward:
            return "MovingForward";
        case MotionState::TurningInPlace:
            return "TurningInPlace";
        case MotionState::SmoothTurning:
            return "SmoothTurning";
        case MotionState::Stopping:
            return "Stopping";
        default:
            return "Unknown";
    }
}

void Robot::setForwardGains(float kp, float ki, float kd)
{
    forward_controller_.setGains(kp, ki, kd);
}

void Robot::setRotationGains(float kp, float ki, float kd)
{
    rotation_controller_.setGains(kp, ki, kd);
}

void Robot::update(float dt)
{
    updateControl(dt);
}

bool Robot::isMotionDone() const
{
    return motionComplete();
}
