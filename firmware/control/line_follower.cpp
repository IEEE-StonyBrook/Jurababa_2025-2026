#include "control/line_follower.h"

#include "common/log.h"

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

LineFollower::LineFollower(Drivetrain* drivetrain, LineSensor* line_sensor, IMU* imu,
                           Battery* battery)
    : drivetrain_(drivetrain), line_sensor_(line_sensor), imu_(imu), battery_(battery)
{
}

void LineFollower::reset()
{
    state_               = State::Idle;
    prev_position_error_ = 0.0f;
    target_yaw_          = 0.0f;
    turn_start_yaw_      = 0.0f;
    turn_degrees_        = 0.0f;
    turn_done_           = true;
    prev_intersection_   = false;
    last_intersection_ms_ = 0;

    drivetrain_->stop();
}

void LineFollower::startFollowing()
{
    state_               = State::FollowingLine;
    prev_position_error_ = 0.0f;
    turn_done_           = true;
}

void LineFollower::update(float dt)
{
    if (dt <= 0.0f)
        return;

    // Read sensor each tick
    line_sensor_->read();

    switch (state_)
    {
        case State::FollowingLine:
            followLine(dt);
            break;

        case State::TurningLeft:
        case State::TurningRight:
            updateTurn(dt);
            break;

        case State::Stopping:
        case State::Idle:
            break;
    }
}

void LineFollower::followLine(float dt)
{
    float position_error = line_sensor_->getPosition();

    // PD steering computation
    float derivative = (position_error - prev_position_error_) / dt;
    float steering   = LINE_KP * position_error + LINE_KD * derivative;

    prev_position_error_ = position_error;

    // Clamp steering to reasonable range
    steering = utils::clampAbs(steering, 1.0f);

    // Base forward speed with differential steering
    float base_speed = LINE_FOLLOW_BASE_SPEED_MMPS;

    // Compute feedforward voltages for both wheels at base speed
    float left_ff =
        drivetrain_->feedforward(WheelSide::LEFT, base_speed, 0.0f) * MAX_VOLTAGE;
    float right_ff =
        drivetrain_->feedforward(WheelSide::RIGHT, base_speed, 0.0f) * MAX_VOLTAGE;

    // Apply steering as differential voltage
    float steering_volts = steering * MAX_VOLTAGE * 0.3f;

    float left_volts  = left_ff - steering_volts;
    float right_volts = right_ff + steering_volts;

    left_volts  = utils::clampAbs(left_volts, MAX_VOLTAGE);
    right_volts = utils::clampAbs(right_volts, MAX_VOLTAGE);

    drivetrain_->setVoltage(left_volts, right_volts);
}

bool LineFollower::isIntersectionDetected()
{
    bool current = line_sensor_->detectIntersection();

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // Rising edge with debounce
    if (current && !prev_intersection_ &&
        (now_ms - last_intersection_ms_) > LINE_INTERSECTION_DEBOUNCE_MS)
    {
        prev_intersection_    = true;
        last_intersection_ms_ = now_ms;
        return true;
    }

    if (!current)
    {
        prev_intersection_ = false;
    }

    return false;
}

void LineFollower::turnLeft90()
{
    state_          = State::TurningLeft;
    turn_start_yaw_ = imu_->yaw();
    turn_degrees_   = -90.0f;
    target_yaw_     = utils::wrapAngle180(turn_start_yaw_ + turn_degrees_);
    turn_done_      = false;

    drivetrain_->stop();
    LOG_DEBUG("LineFollower: Turning left 90°");
}

void LineFollower::turnRight90()
{
    state_          = State::TurningRight;
    turn_start_yaw_ = imu_->yaw();
    turn_degrees_   = 90.0f;
    target_yaw_     = utils::wrapAngle180(turn_start_yaw_ + turn_degrees_);
    turn_done_      = false;

    drivetrain_->stop();
    LOG_DEBUG("LineFollower: Turning right 90°");
}

void LineFollower::updateTurn(float dt)
{
    float current_yaw = imu_->yaw();
    float yaw_error   = normalizeYawDelta(target_yaw_ - current_yaw);

    // Simple proportional turn control
    float turn_output = ROT_KP * yaw_error;
    turn_output       = utils::clampAbs(turn_output, 1.0f);

    float turn_volts = turn_output * MAX_VOLTAGE * 0.5f;

    // Differential drive: spin in place
    drivetrain_->setVoltage(-turn_volts, turn_volts);

    // Check completion
    if (std::fabs(yaw_error) < ROBOT_YAW_TOLERANCE_DEG)
    {
        drivetrain_->stop();
        turn_done_ = true;
        state_     = State::FollowingLine;
        prev_position_error_ = 0.0f;
        LOG_DEBUG("LineFollower: Turn complete, resuming line follow");
    }
}

void LineFollower::stop()
{
    state_ = State::Stopping;
    drivetrain_->stop();
    turn_done_ = true;
}

bool LineFollower::isMotionDone() const
{
    if (state_ == State::TurningLeft || state_ == State::TurningRight)
        return turn_done_;
    return true;
}

LineFollower::State LineFollower::state() const
{
    return state_;
}
