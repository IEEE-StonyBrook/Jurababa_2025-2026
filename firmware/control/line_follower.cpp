#include "control/line_follower.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common/log.h"

namespace
{
uint8_t pathMaskForCommand(char command)
{
    switch (command)
    {
        case 'L':
            return LineSensor::PATH_LEFT;
        case 'F':
            return LineSensor::PATH_FORWARD;
        case 'R':
            return LineSensor::PATH_RIGHT;
        default:
            return LineSensor::PATH_NONE;
    }
}

const char* pathsText(uint8_t paths_mask, char* buffer, size_t buffer_size)
{
    if (buffer_size == 0)
        return "";

    size_t index = 0;
    if ((paths_mask & LineSensor::PATH_LEFT) != 0 && index + 1 < buffer_size)
        buffer[index++] = 'L';
    if ((paths_mask & LineSensor::PATH_FORWARD) != 0 && index + 1 < buffer_size)
        buffer[index++] = 'F';
    if ((paths_mask & LineSensor::PATH_RIGHT) != 0 && index + 1 < buffer_size)
        buffer[index++] = 'R';
    if (index == 0 && index + 1 < buffer_size)
        buffer[index++] = '-';
    buffer[index] = '\0';
    return buffer;
}

const char* hexByteText(uint8_t value, char* buffer, size_t buffer_size)
{
    if (buffer_size == 0)
        return "";
    snprintf(buffer, buffer_size, "0x%02X", value);
    return buffer;
}

const char* yesNo(bool value)
{
    return value ? "yes" : "no";
}
} // namespace

LineFollower::LineFollower(LineSensor* line_sensor, Motion* motion)
    : line_sensor_(line_sensor), motion_(motion)
{
    if (!setRoute(LINE_FOLLOW_ROUTE))
    {
        clearRoute();
        LOG_WARNING("LineFollower: invalid LINE_FOLLOW_ROUTE; route cleared");
    }
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
    route_index_       = 0;
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    last_line_seen_ms_ = now;
    line_seen_         = true;

    // Reset run statistics so each LINE START produces an independent debrief.
    run_start_ms_        = now;
    run_end_ms_          = 0;
    peak_abs_error_      = 0.0f;
    peak_abs_steering_   = 0.0f;
    peak_velocity_mmps_  = 0.0f;
    min_velocity_mmps_   = 0.0f;
    min_velocity_seeded_ = false;
    intersection_count_  = 0;
    recovery_count_      = 0;
    saturation_ticks_    = 0;

    motion_->reset_drive_system();
    // start_move sets up the forward profile with a generous accel so that
    // per-tick set_target_velocity() updates from the speed scheduler track
    // smoothly rather than jolting. Top speed is the hard cap; the scheduler
    // pushes the actual cruise speed down on |error| from there.
    motion_->start_move(LINE_FOLLOW_RUN_DISTANCE_MM, LINE_MAX_SPEED_MMPS, 0.0f,
                        ROBOT_BASE_ACCEL_MMPS2);
    motion_->set_target_velocity(LINE_TARGET_SPEED_MMPS);
    latest_target_speed_mmps_ = LINE_TARGET_SPEED_MMPS;
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

    // Position update — sensor's get_position() already holds last-known
    // when the line is briefly invisible, so we only refresh the cached
    // error when a real reading is in.
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

    // Lost-line state machine. Recovery steers full-authority toward the
    // last side the line was on, while clamping forward speed to MIN.
    const uint32_t lost_ms = line_present ? 0 : now_ms - last_line_seen_ms_;
    if (lost_ms > LINE_LOST_STOP_MS)
    {
        stop();
        return;
    }
    if (lost_ms > LINE_LOST_HOLD_MS)
    {
        const float recovery_dir  = (prev_line_error_ >= 0.0f) ? 1.0f : -1.0f;
        latest_steering_degps_    = recovery_dir * LINE_OMEGA_LIMIT_DEGPS * LINE_RECOVERY_AUTHORITY;
        latest_target_speed_mmps_ = LINE_MIN_SPEED_MMPS;
        if (!recovery_active_)
        {
            ++recovery_count_;
            LOG_INFO("LineFollower: recovery start lost_ms=" << lost_ms << " steer="
                                                             << latest_steering_degps_ << " deg/s"
                                                             << " count=" << recovery_count_);
            recovery_active_ = true;
        }
        motion_->set_line_steering_adjustment_degps(latest_steering_degps_, true);
        motion_->set_target_velocity(latest_target_speed_mmps_);
        return;
    }
    if (recovery_active_)
    {
        LOG_INFO("LineFollower: recovery end lost_ms=" << lost_ms);
        recovery_active_ = false;
    }

    // Low-pass filter on the position error. Filter feeds both the P term
    // (via the predictive lookahead) and the D term, so noise rejection
    // and phase lag affect them coherently.
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

    const float de_dt = (filtered_line_error_ - prev_line_error_) / dt;
    prev_line_error_  = filtered_line_error_;

    // Predictive (lookahead) error: e_pred = e + tau * de/dt. This
    // substitutes for physically mounting the sensor bar ahead of the
    // wheel axle, which would have given the same phase lead for free.
    const float e_pred = filtered_line_error_ + LINE_LOOKAHEAD_TIME_S * de_dt;

    // Gain scheduling vs current forward velocity. Higher v -> smaller
    // Kp/Kd so the spatial response (deg of rotation per mm of travel)
    // stays roughly constant across the speed envelope.
    const float v_now   = motion_->velocity();
    const float v_abs   = fabsf(v_now);
    const float v_sched = (v_abs > LINE_GAIN_SCHED_FLOOR_MMPS) ? v_abs : LINE_GAIN_SCHED_FLOOR_MMPS;
    const float scale   = LINE_GAIN_REF_SPEED_MMPS / v_sched;
    const float kp_eff  = LINE_KP_BASE_DEGPS_PER_SLOT * scale;
    const float kd_eff  = LINE_KD_BASE_DEG_PER_SLOT * scale;

    float steering_degps = kp_eff * e_pred + kd_eff * de_dt;

    // Branch bias during the post-intersection capture window. Adds a
    // hard rotational kick so the robot commits to the chosen branch
    // before the centroid catches up.
    if (branch_direction_ != BranchDirection::None)
    {
        if (now_ms < branch_capture_end_ms_)
        {
            const float branch_bias = (branch_direction_ == BranchDirection::Left)
                                          ? LINE_BRANCH_STEER_BIAS_DEGPS
                                          : -LINE_BRANCH_STEER_BIAS_DEGPS;
            steering_degps += branch_bias;
        }
        else
        {
            branch_direction_ = BranchDirection::None;
        }
    }

    steering_degps = utils::clampAbs(steering_degps, LINE_OMEGA_LIMIT_DEGPS);

    // Speed scheduling on |filtered error|. Brakes before corners by
    // pulling the forward profile's target velocity down; clamped to
    // [MIN, MAX] so the robot never crawls or exceeds the safe envelope.
    float v_target =
        LINE_TARGET_SPEED_MMPS - LINE_BRAKE_GAIN_MMPS_PER_SLOT * fabsf(filtered_line_error_);
    if (v_target < LINE_MIN_SPEED_MMPS)
        v_target = LINE_MIN_SPEED_MMPS;
    if (v_target > LINE_MAX_SPEED_MMPS)
        v_target = LINE_MAX_SPEED_MMPS;

    latest_steering_degps_    = steering_degps;
    latest_target_speed_mmps_ = v_target;

    // Run-statistics accumulation (PATH-style aggregates for debrief).
    const float abs_err = fabsf(filtered_line_error_);
    if (abs_err > peak_abs_error_)
        peak_abs_error_ = abs_err;
    const float abs_steer = fabsf(steering_degps);
    if (abs_steer > peak_abs_steering_)
        peak_abs_steering_ = abs_steer;
    if (abs_steer >= LINE_OMEGA_LIMIT_DEGPS - 0.5f)
        ++saturation_ticks_;
    if (v_abs > peak_velocity_mmps_)
        peak_velocity_mmps_ = v_abs;
    if (!min_velocity_seeded_ || v_abs < min_velocity_mmps_)
    {
        min_velocity_mmps_   = v_abs;
        min_velocity_seeded_ = true;
    }

    motion_->set_line_steering_adjustment_degps(steering_degps, true);
    motion_->set_target_velocity(v_target);

    evaluateIntersectionCommand(now_ms);
}

bool LineFollower::isIntersectionDetected()
{
    bool current = line_sensor_->intersectionEvent().valid;

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

void LineFollower::evaluateIntersectionCommand(uint32_t now_ms)
{
    if (now_ms < intersection_lockout_end_ms_)
        return;

    LineSensor::IntersectionEvent event = line_sensor_->intersectionEvent();
    if (!event.valid)
        return;

    if ((now_ms - last_intersection_ms_) <= LINE_INTERSECTION_DEBOUNCE_MS)
        return;

    last_intersection_ms_        = now_ms;
    intersection_lockout_end_ms_ = now_ms + LINE_INTERSECTION_LOCKOUT_MS;
    last_intersection_event_     = event;
    ++intersection_count_;

    char    command       = 'F';
    uint8_t command_index = route_index_;
    if (route_index_ >= route_length_)
    {
        last_route_command_        = command;
        last_route_choice_matched_ = true;
        char paths_buffer[4];
        char peak_buffer[5];
        LOG_INFO("Line: event paths="
                 << pathsText(event.paths_mask, paths_buffer, sizeof(paths_buffer))
                 << " route=end -> F peak="
                 << hexByteText(event.raw_peak_mask, peak_buffer, sizeof(peak_buffer))
                 << " dt=" << event.elapsed_ms << "ms");
        return;
    }

    command                    = route_[route_index_++];
    last_route_command_        = command;
    last_route_choice_matched_ = (event.paths_mask & pathMaskForCommand(command)) != 0;

    char paths_buffer[4];
    char peak_buffer[5];
    LOG_INFO(
        "Line: event paths=" << pathsText(event.paths_mask, paths_buffer, sizeof(paths_buffer))
                             << " route[" << static_cast<int>(command_index) << "]=" << command
                             << " match=" << yesNo(last_route_choice_matched_) << " peak="
                             << hexByteText(event.raw_peak_mask, peak_buffer, sizeof(peak_buffer))
                             << " dt=" << event.elapsed_ms << "ms");

    if (!last_route_choice_matched_)
    {
        LOG_WARNING("Line WARN: route["
                    << static_cast<int>(command_index) << "]=" << command << " not in paths="
                    << pathsText(event.paths_mask, paths_buffer, sizeof(paths_buffer))
                    << "; executing anyway");
    }

    if (command == 'L')
    {
        branch_direction_      = BranchDirection::Left;
        branch_capture_end_ms_ = now_ms + LINE_BRANCH_CAPTURE_MS;
        return;
    }

    if (command == 'R')
    {
        branch_direction_      = BranchDirection::Right;
        branch_capture_end_ms_ = now_ms + LINE_BRANCH_CAPTURE_MS;
        return;
    }
}

void LineFollower::stop()
{
    // Freeze run end before resetControlHistory wipes derived state, so the
    // CLI can still print a debrief from runDurationMs() / peak* accessors.
    if (run_start_ms_ != 0 && run_end_ms_ == 0)
        run_end_ms_ = to_ms_since_boot(get_absolute_time());

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
    latest_target_speed_mmps_    = 0.0f;
    filter_initialized_          = false;
    line_seen_                   = false;
    line_lost_                   = false;
    last_line_seen_ms_           = 0;
    branch_direction_            = BranchDirection::None;
    branch_capture_end_ms_       = 0;
    intersection_lockout_end_ms_ = 0;
    recovery_active_             = false;
    last_intersection_event_     = {};
    last_route_command_          = '-';
    last_route_choice_matched_   = true;
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

float LineFollower::targetSpeedMmps() const
{
    return latest_target_speed_mmps_;
}

bool LineFollower::setRoute(const char* route)
{
    if (route == nullptr)
    {
        clearRoute();
        return true;
    }

    char    pending_route[kMaxRouteLength + 1] = {};
    uint8_t write_index                        = 0;
    for (uint8_t i = 0; route[i] != '\0'; ++i)
    {
        const char c = static_cast<char>(toupper(static_cast<unsigned char>(route[i])));
        if (c == ' ' || c == '\t' || c == ',')
            continue;
        if (c != 'L' && c != 'F' && c != 'R')
            return false;
        if (write_index >= kMaxRouteLength)
            return false;
        pending_route[write_index++] = c;
    }

    memcpy(route_, pending_route, write_index);
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

uint8_t LineFollower::currentPathsMask() const
{
    return line_sensor_->currentPathsMask();
}

bool LineFollower::lastIntersectionValid() const
{
    return last_intersection_event_.valid;
}

uint8_t LineFollower::lastIntersectionPathsMask() const
{
    return last_intersection_event_.paths_mask;
}

uint8_t LineFollower::lastIntersectionRawPeakMask() const
{
    return last_intersection_event_.raw_peak_mask;
}

uint32_t LineFollower::lastIntersectionElapsedMs() const
{
    return last_intersection_event_.elapsed_ms;
}

char LineFollower::lastRouteCommand() const
{
    return last_route_command_;
}

bool LineFollower::lastRouteChoiceMatched() const
{
    return last_route_choice_matched_;
}

uint32_t LineFollower::runDurationMs() const
{
    if (run_start_ms_ == 0)
        return 0;
    const uint32_t end = (run_end_ms_ != 0) ? run_end_ms_ : to_ms_since_boot(get_absolute_time());
    return end - run_start_ms_;
}

float LineFollower::peakAbsErrorSlots() const
{
    return peak_abs_error_;
}

float LineFollower::peakAbsSteeringDegps() const
{
    return peak_abs_steering_;
}

float LineFollower::peakVelocityMmps() const
{
    return peak_velocity_mmps_;
}

float LineFollower::minVelocityMmps() const
{
    return min_velocity_seeded_ ? min_velocity_mmps_ : 0.0f;
}

uint32_t LineFollower::intersectionCount() const
{
    return intersection_count_;
}

uint32_t LineFollower::recoveryCount() const
{
    return recovery_count_;
}

uint32_t LineFollower::saturationTicks() const
{
    return saturation_ticks_;
}
