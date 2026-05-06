#include "control/line_follower.h"

#include "common/log.h"

LineFollower::LineFollower(LineSensor* line_sensor, Motion* motion)
    : line_sensor_(line_sensor), motion_(motion)
{
}

void LineFollower::reset()
{
    state_                = State::Idle;
    turn_done_            = true;
    prev_intersection_    = false;
    last_intersection_ms_ = 0;
    resetControlHistory();

    motion_->emergency_stop();
}

void LineFollower::startFollowing()
{
    state_     = State::FollowingLine;
    turn_done_ = true;
    resetControlHistory();
    motion_->reset_drive_system();
    motion_->start_move(LINE_FOLLOW_RUN_DISTANCE_MM, LINE_FOLLOW_BASE_SPEED_MMPS, 0.0f,
                        ROBOT_BASE_ACCEL_MMPS2);
}

void LineFollower::update(float dt)
{
    if (dt <= 0.0f)
        return;

    line_sensor_->read();

    switch (state_)
    {
        case State::FollowingLine:
            followLine(dt);
            break;

        case State::TurningLeft:
        case State::TurningRight:
            updateTurn();
            break;

        case State::Stopping:
        case State::Idle:
            break;
    }
}

void LineFollower::followLine(float dt)
{
    const uint32_t now_ms       = to_ms_since_boot(get_absolute_time());
    const bool     line_present = line_sensor_->on_line();

    if (line_present)
    {
        latest_line_position_ = line_sensor_->get_position();
        latest_line_error_    = -latest_line_position_;
        last_line_seen_ms_    = now_ms;
        line_seen_            = true;
        line_lost_            = false;
    }
    else
    {
        line_lost_ = true;
        if (!line_seen_)
        {
            stop();
            return;
        }
    }

    const uint32_t lost_ms = line_present ? 0 : now_ms - last_line_seen_ms_;
    if (lost_ms > LINE_LOST_STOP_MS)
    {
        stop();
        return;
    }
    if (lost_ms > LINE_LOST_HOLD_MS)
    {
        latest_steering_degps_ = 0.0f;
        motion_->set_line_steering_adjustment_degps(0.0f, true);
        return;
    }

    if (!filter_initialized_)
    {
        filtered_line_error_ = latest_line_error_;
        prev_line_error_     = filtered_line_error_;
        filter_initialized_  = true;
    }
    else if (line_present)
    {
        filtered_line_error_ +=
            LINE_ERROR_FILTER_ALPHA * (latest_line_error_ - filtered_line_error_);
    }

    const float line_error_delta_per_s = (filtered_line_error_ - prev_line_error_) / dt;
    prev_line_error_                   = filtered_line_error_;

    float steering_degps = LINE_STEERING_KP_DEGPS_PER_SENSOR * filtered_line_error_ +
                           LINE_STEERING_KD_DEG_PER_SENSOR * line_error_delta_per_s;
    steering_degps = utils::clampAbs(steering_degps, LINE_STEERING_LIMIT_DEGPS);
    latest_steering_degps_ = steering_degps;
    motion_->set_line_steering_adjustment_degps(steering_degps, true);
}

bool LineFollower::isIntersectionDetected()
{
    bool current = line_sensor_->detect_intersection();

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
    state_     = State::TurningLeft;
    turn_done_ = false;

    motion_->clear_line_steering_adjustment();
    motion_->spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
    LOG_DEBUG("LineFollower: Turning left 90°");
}

void LineFollower::turnRight90()
{
    state_     = State::TurningRight;
    turn_done_ = false;

    motion_->clear_line_steering_adjustment();
    motion_->spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
    LOG_DEBUG("LineFollower: Turning right 90°");
}

void LineFollower::updateTurn()
{
    if (motion_->turn_finished())
    {
        startFollowing();
        LOG_DEBUG("LineFollower: Turn complete, resuming line follow");
    }
}

void LineFollower::stop()
{
    state_ = State::Stopping;
    motion_->clear_line_steering_adjustment();
    motion_->emergency_stop();
    turn_done_ = true;
    resetControlHistory();
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

void LineFollower::resetControlHistory()
{
    prev_line_error_      = 0.0f;
    filtered_line_error_  = 0.0f;
    latest_line_position_ = 0.0f;
    latest_line_error_    = 0.0f;
    latest_steering_degps_ = 0.0f;
    filter_initialized_   = false;
    line_seen_            = false;
    line_lost_            = false;
    last_line_seen_ms_    = 0;
    motion_->clear_line_steering_adjustment();
}

uint8_t LineFollower::rawByte() const
{
    return line_sensor_->rawByte();
}

uint8_t LineFollower::activeMask() const
{
    return line_sensor_->activeMask();
}

bool LineFollower::linePresent() const
{
    return line_sensor_->on_line();
}

bool LineFollower::lineLost() const
{
    return line_lost_;
}

float LineFollower::linePosition() const
{
    return latest_line_position_;
}

float LineFollower::lineError() const
{
    return latest_line_error_;
}

float LineFollower::filteredLineError() const
{
    return filtered_line_error_;
}

float LineFollower::steeringAdjustmentDegps() const
{
    return latest_steering_degps_;
}
