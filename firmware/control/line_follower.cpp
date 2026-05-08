#include "control/line_follower.h"

#include <ctype.h>
#include <math.h>

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
    last_line_seen_ms_ = to_ms_since_boot(get_absolute_time());
    line_seen_         = true;
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
    static constexpr float kLineLostReacquireGain         = 0.6f;
    static constexpr float kMajorCorrectionThresholdRatio = 0.8f;
    const uint32_t         now_ms                         = to_ms_since_boot(get_absolute_time());
    const bool             line_present                   = line_sensor_->on_line();

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
        const float recovery_dir = (prev_line_error_ >= 0.0f) ? 1.0f : -1.0f;
        latest_steering_degps_ = recovery_dir * LINE_STEERING_LIMIT_DEGPS * kLineLostReacquireGain;
        if (!recovery_active_)
        {
            LOG_INFO("LineFollower: recovery start lost_ms=" << lost_ms << " steer="
                                                             << latest_steering_degps_ << " deg/s");
            recovery_active_ = true;
        }
        motion_->set_line_steering_adjustment_degps(latest_steering_degps_, true);
        return;
    }
    if (recovery_active_)
    {
        LOG_INFO("LineFollower: recovery end lost_ms=" << lost_ms);
        recovery_active_ = false;
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

    const float major_correction_threshold =
        LINE_STEERING_LIMIT_DEGPS * kMajorCorrectionThresholdRatio;
    const bool major_correction_now = fabsf(steering_degps) >= major_correction_threshold;
    if (major_correction_now && !major_correction_active_)
    {
        LOG_INFO("LineFollower: major correction steer=" << steering_degps
                                                         << " deg/s err=" << filtered_line_error_);
        major_correction_active_ = true;
    }
    else if (!major_correction_now && major_correction_active_)
    {
        LOG_INFO("LineFollower: correction settled steer=" << steering_degps << " deg/s");
        major_correction_active_ = false;
    }

    latest_steering_degps_ = steering_degps;
    motion_->set_line_steering_adjustment_degps(steering_degps, true);

    evaluateIntersectionCommand(now_ms);
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
    motion_->spin_turn(90.0f, LINE_TURN_SPEED_DEGPS, LINE_TURN_ACCEL_DEGPS2);
    LOG_DEBUG("LineFollower: Turning left 90°");
}

void LineFollower::turnRight90()
{
    state_     = State::TurningRight;
    turn_done_ = false;

    motion_->clear_line_steering_adjustment();
    motion_->spin_turn(-90.0f, LINE_TURN_SPEED_DEGPS, LINE_TURN_ACCEL_DEGPS2);
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

void LineFollower::evaluateIntersectionCommand(uint32_t now_ms)
{
    if (now_ms < intersection_lockout_end_ms_)
        return;

    if (!isIntersectionDetected())
        return;

    intersection_lockout_end_ms_ = now_ms + LINE_INTERSECTION_LOCKOUT_MS;

    if (route_index_ >= route_length_)
    {
        LOG_INFO("LineFollower: intersection -> default forward");
        return;
    }

    const char command = route_[route_index_++];
    if (command == 'L')
    {
        LOG_INFO("LineFollower: intersection -> route L");
        turnLeft90();
        return;
    }

    if (command == 'R')
    {
        LOG_INFO("LineFollower: intersection -> route R");
        turnRight90();
        return;
    }

    LOG_INFO("LineFollower: intersection -> route F");
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
    prev_line_error_             = 0.0f;
    filtered_line_error_         = 0.0f;
    latest_line_position_        = 0.0f;
    latest_line_error_           = 0.0f;
    latest_steering_degps_       = 0.0f;
    filter_initialized_          = false;
    line_seen_                   = false;
    line_lost_                   = false;
    last_line_seen_ms_           = 0;
    intersection_lockout_end_ms_ = 0;
    recovery_active_             = false;
    major_correction_active_     = false;
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

bool LineFollower::setRoute(const char* route)
{
    clearRoute();
    if (route == nullptr)
        return true;

    uint8_t write_index = 0;
    for (uint8_t i = 0; route[i] != '\0'; ++i)
    {
        const char c = static_cast<char>(toupper(static_cast<unsigned char>(route[i])));
        if (c == ' ' || c == '\t' || c == ',')
            continue;
        if (c != 'L' && c != 'F' && c != 'R')
            return false;
        if (write_index >= kMaxRouteLength)
            return false;
        route_[write_index++] = c;
    }

    route_length_       = write_index;
    route_[write_index] = '\0';
    route_index_        = 0;
    return true;
}

void LineFollower::clearRoute()
{
    route_length_ = 0;
    route_index_  = 0;
    route_[0]     = '\0';
}

const char* LineFollower::route() const
{
    return route_;
}

uint8_t LineFollower::routeIndex() const
{
    return route_index_;
}

bool LineFollower::routeHasRemaining() const
{
    return route_index_ < route_length_;
}
