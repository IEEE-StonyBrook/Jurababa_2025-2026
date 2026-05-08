#include "control/motion.h"

#include <cmath>

#include "common/utils.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "drivers/imu.h"
#include "pico/stdlib.h"

Motion::Motion(Drivetrain* drivetrain, IMU* imu)
    : drivetrain_(drivetrain), imu_(imu), forward_controller_(FWD_KP, FWD_KD, LOOP_FREQUENCY_HZ),
      rotation_controller_(ROT_KP, ROT_KD, LOOP_FREQUENCY_HZ)
{
    forward_controller_.setOutputLimit(MAX_VOLTAGE);
    rotation_controller_.setOutputLimit(MAX_VOLTAGE);

    reset();
}

void Motion::reset()
{
    reset_drive_system();
}

void Motion::reset_drive_system()
{
    motion_sequence_active_ = false;
    clear_heading_hold();
    drivetrain_->stop();
    drivetrain_->reset();
    imu_->reset();
    forward_.reset();
    rotation_.reset();
    resetControlHistory();
    steering_mode_ = tof_wall::SteeringMode::STEERING_OFF;
}

void Motion::reset_drive_control()
{
    drivetrain_->stop();
    drivetrain_->reset();
    forward_.reset();
    rotation_.reset();
    resetControlHistory();
}

void Motion::resetControlHistory()
{
    forward_controller_.reset();
    rotation_controller_.reset();

    prev_left_cmd_vel_mmps_               = 0.0f;
    prev_right_cmd_vel_mmps_              = 0.0f;
    side_error_prev_norm_                 = 0.0f;
    side_error_prev_valid_                = false;
    latest_wall_state_                    = {};
    latest_steering_adjustment_degps_     = 0.0f;
    latest_heading_hold_adjustment_degps_ = 0.0f;
    line_steering_adjustment_degps_       = 0.0f;
    line_steering_valid_                  = false;
    latest_line_rotation_volts_           = 0.0f;
    latest_line_differential_volts_       = 0.0f;
    latest_motor_differential_volts_      = 0.0f;
    resetWallSteeringStats();
}

void Motion::set_wall_distances(float left_mm, float front_mm, float right_mm)
{
    set_wall_distances(left_mm, front_mm, right_mm, left_mm, front_mm, right_mm);
}

void Motion::set_wall_distances(float left_raw_mm, float front_raw_mm, float right_raw_mm,
                                float left_filtered_mm, float front_filtered_mm,
                                float right_filtered_mm)
{
    left_raw_wall_mm_  = left_raw_mm;
    front_raw_wall_mm_ = front_raw_mm;
    right_raw_wall_mm_ = right_raw_mm;
    left_wall_mm_      = left_filtered_mm;
    front_wall_mm_     = front_filtered_mm;
    right_wall_mm_     = right_filtered_mm;
}

void Motion::set_steering_mode(tof_wall::SteeringMode mode)
{
    side_error_prev_norm_             = 0.0f;
    side_error_prev_valid_            = false;
    latest_steering_adjustment_degps_ = 0.0f;
    steering_mode_                    = mode;
}

tof_wall::SteeringMode Motion::steeringMode() const
{
    return steering_mode_;
}

bool Motion::wallLeft()
{
    return tof_wall::wallLeft(left_wall_mm_);
}

bool Motion::wallFront()
{
    return tof_wall::wallFront(front_wall_mm_);
}

bool Motion::wallRight()
{
    return tof_wall::wallRight(right_wall_mm_);
}

float Motion::leftDistance()
{
    return left_wall_mm_;
}

float Motion::frontDistance()
{
    return front_wall_mm_;
}

float Motion::rightDistance()
{
    return right_wall_mm_;
}

float Motion::leftRawDistance()
{
    return left_raw_wall_mm_;
}

float Motion::frontRawDistance()
{
    return front_raw_wall_mm_;
}

float Motion::rightRawDistance()
{
    return right_raw_wall_mm_;
}

tof_wall::WallState Motion::wallSteeringState() const
{
    return latest_wall_state_;
}

float Motion::wallSteeringAdjustmentDegps() const
{
    return latest_steering_adjustment_degps_;
}

Motion::WallSteeringStats Motion::wallSteeringStats() const
{
    WallSteeringStats stats;
    stats.peak_degps = wall_steering_peak_degps_;
    stats.average_degps =
        wall_steering_samples_ > 0
            ? wall_steering_sum_degps_ / static_cast<float>(wall_steering_samples_)
            : 0.0f;
    stats.samples        = wall_steering_samples_;
    stats.source_changes = wall_steering_source_changes_;
    stats.both_samples   = wall_steering_both_samples_;
    stats.single_samples = wall_steering_single_samples_;
    return stats;
}

void Motion::resetWallSteeringStats()
{
    wall_steering_peak_degps_        = 0.0f;
    wall_steering_sum_degps_         = 0.0f;
    wall_steering_samples_           = 0;
    wall_steering_source_changes_    = 0;
    wall_steering_both_samples_      = 0;
    wall_steering_single_samples_    = 0;
    wall_steering_prev_source_       = tof_wall::SteeringSource::None;
    wall_steering_prev_source_valid_ = false;
}

void Motion::set_line_steering_adjustment_degps(float adjustment_degps, bool valid)
{
    line_steering_adjustment_degps_ = valid ? adjustment_degps : 0.0f;
    line_steering_valid_            = valid;
}

void Motion::clear_line_steering_adjustment()
{
    line_steering_adjustment_degps_ = 0.0f;
    line_steering_valid_            = false;
}

float Motion::lineSteeringAdjustmentDegps() const
{
    return line_steering_adjustment_degps_;
}

bool Motion::lineSteeringValid() const
{
    return line_steering_valid_;
}

float Motion::lineRotationVolts() const
{
    return latest_line_rotation_volts_;
}

float Motion::lineDifferentialVolts() const
{
    return latest_line_differential_volts_;
}

float Motion::motorDifferentialVolts() const
{
    return latest_motor_differential_volts_;
}

void Motion::set_heading_hold(float target_yaw_deg)
{
    heading_hold_enabled_          = true;
    heading_hold_target_yaw_deg_   = utils::wrapAngle180(target_yaw_deg);
    latest_heading_hold_error_deg_ = utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
}

void Motion::clear_heading_hold()
{
    heading_hold_enabled_                 = false;
    heading_hold_target_yaw_deg_          = 0.0f;
    latest_heading_hold_error_deg_        = 0.0f;
    latest_heading_hold_adjustment_degps_ = 0.0f;
}

bool Motion::headingHoldActive() const
{
    return heading_hold_enabled_;
}

float Motion::headingHoldTargetYawDeg() const
{
    return heading_hold_target_yaw_deg_;
}

float Motion::headingHoldErrorDeg() const
{
    return latest_heading_hold_error_deg_;
}

float Motion::headingHoldAdjustmentDegps() const
{
    return latest_heading_hold_adjustment_degps_;
}

float Motion::position() const
{
    return forward_.position();
}

float Motion::wheel_position(WheelSide side)
{
    return drivetrain_->position(side);
}

int32_t Motion::encoder_ticks(WheelSide side) const
{
    return drivetrain_->ticks(side);
}

float Motion::velocity() const
{
    return forward_.velocity();
}

float Motion::acceleration() const
{
    return forward_.acceleration();
}

float Motion::angle() const
{
    return rotation_.position();
}

float Motion::omega() const
{
    return rotation_.velocity();
}

float Motion::alpha() const
{
    return rotation_.acceleration();
}

float Motion::yaw_deg() const
{
    return imu_->robot_angle();
}

float Motion::yaw_rate_degps() const
{
    return imu_->robot_omega();
}

void Motion::set_target_velocity(float velocity_mmps)
{
    forward_.setTargetSpeed(velocity_mmps);
}

void Motion::set_final_velocity(float velocity_mmps)
{
    forward_.setFinalSpeed(velocity_mmps);
}

void Motion::extend_move(float distance_mm)
{
    forward_.extendTarget(distance_mm);
}

void Motion::begin_motion_sequence()
{
    reset_drive_control();
    clear_heading_hold();
    set_steering_mode(tof_wall::SteeringMode::STEERING_OFF);
    motion_sequence_active_ = true;
}

void Motion::end_motion_sequence()
{
    motion_sequence_active_ = false;
    clear_heading_hold();
    drivetrain_->stop();
    forward_.reset();
    rotation_.reset();
    resetControlHistory();
}

void Motion::start_move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                        float accel_mmps2)
{
    forward_.start(distance_mm, velocity(), top_speed_mmps, final_speed_mmps, accel_mmps2);
}

bool Motion::move_finished() const
{
    return !forward_.active();
}

void Motion::move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                  float accel_mmps2)
{
    start_move(distance_mm, top_speed_mmps, final_speed_mmps, accel_mmps2);
}

void Motion::start_turn(float degrees, float top_speed_degps, float final_speed_degps,
                        float accel_degps2)
{
    rotation_.start(degrees, rotation_.velocity(), top_speed_degps, final_speed_degps,
                    accel_degps2);
}

bool Motion::turn_finished() const
{
    return !rotation_.active();
}

void Motion::turn(float degrees, float top_speed_degps, float final_speed_degps, float accel_degps2)
{
    start_turn(degrees, top_speed_degps, final_speed_degps, accel_degps2);
}

void Motion::spin_turn(float degrees, float omega_degps, float alpha_degps2)
{
    forward_.setFinalSpeed(0.0f);
    forward_.setTargetSpeed(0.0f);
    while (std::fabs(forward_.velocity()) > 1e-3f)
        sleep_ms(2);
    rotation_.reset();
    rotation_.start(degrees, omega_degps, 0.0f, alpha_degps2);
}

void Motion::turn_IP180()
{
    static int direction = 1;
    direction *= -1;
    spin_turn(direction * 180.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Motion::turn_IP90R()
{
    spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Motion::turn_IP90L()
{
    spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Motion::stop_at(float position_mm)
{
    const float remaining_mm = position_mm - forward_.position();
    forward_.start(remaining_mm, forward_.velocity(), std::fabs(forward_.velocity()), 0.0f,
                   forward_.acceleration());
}

void Motion::stop_after(float distance_mm)
{
    forward_.start(distance_mm, forward_.velocity(), std::fabs(forward_.velocity()), 0.0f,
                   forward_.acceleration());
}

void Motion::set_position(float position_mm)
{
    forward_.setPosition(position_mm);
}

void Motion::adjust_forward_position(float delta_mm)
{
    forward_.adjustPosition(delta_mm);
}

void Motion::turn_smooth(int turn_id)
{
    if (turn_id < 0 || turn_id >= SMOOTH_TURN_PARAM_COUNT)
        return;

    const SmoothTurnParameters& params = SMOOTH_TURN_PARAMS[turn_id];
    const float                 radius_mm =
        (std::fabs(params.angle_deg) > 1e-3f)
            ? ((params.speed_mmps / params.omega_degps) * (180.0f / static_cast<float>(M_PI)))
            : 0.0f;

    if (radius_mm <= 0.0f)
        return;

    const float arc_length_mm = std::fabs(params.angle_deg) * (M_PI / 180.0f) * radius_mm;
    forward_.start(arc_length_mm, params.speed_mmps, params.speed_mmps, ROBOT_BASE_ACCEL_MMPS2);
    rotation_.start(params.angle_deg, params.omega_degps, 0.0f, params.alpha_degps2);
}

void Motion::update()
{
    // Mazerunner systick.update() shape:
    //   encoders.update(); motion.update(); motors.update_controllers(...)
    // We add imu_->update() because IMU owns rotation tracking
    // (substitutes mazerunner's encoder-derived robot_rot_change).
    drivetrain_->update();
    imu_->update();
    forward_.update();
    rotation_.update();

    runPositionControl();
}

void Motion::runPositionControl()
{
    const bool forward_commanded  = forward_.active() || std::fabs(forward_.velocity()) > 1e-3f;
    const bool rotation_commanded = rotation_.active() || std::fabs(rotation_.velocity()) > 1e-3f;

    if (!forward_commanded && !rotation_commanded)
    {
        drivetrain_->stop();
        if (motion_sequence_active_)
        {
            latest_wall_state_ =
                tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
            latest_steering_adjustment_degps_     = 0.0f;
            latest_heading_hold_adjustment_degps_ = 0.0f;
            latest_line_rotation_volts_           = 0.0f;
            latest_line_differential_volts_       = 0.0f;
            latest_motor_differential_volts_      = 0.0f;
            if (heading_hold_enabled_)
                latest_heading_hold_error_deg_ =
                    utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
            return;
        }
        resetControlHistory();
        latest_wall_state_ =
            tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
        latest_steering_adjustment_degps_     = 0.0f;
        latest_heading_hold_adjustment_degps_ = 0.0f;
        latest_line_rotation_volts_           = 0.0f;
        latest_line_differential_volts_       = 0.0f;
        latest_motor_differential_volts_      = 0.0f;
        if (heading_hold_enabled_)
            latest_heading_hold_error_deg_ =
                utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
        return;
    }

    float fwd_change_mm  = drivetrain_->fwdChangeMm();
    float rot_change_deg = imu_->robot_rot_change(); // per-tick delta (omega * dt)
    float fwd_velocity   = forward_.velocity();
    float rot_velocity   = rotation_.velocity();

    // Single-rate PD — both forward and rotation update every 500 Hz tick,
    // exactly like UKMARS mazerunner-core. Rotation feedback comes from the
    // IMU as a held-flat per-tick delta; this removes the 10 ms staircase
    // that the old 100 Hz gate produced in rotation_output_ and stops the
    // step-input-driven oscillation on spin turns.
    const float forward_output =
        forward_commanded ? forward_controller_.update(fwd_velocity, fwd_change_mm) : 0.0f;
    const bool straight_move       = forward_.active() && !rotation_.active();
    float      wall_adjustment     = 0.0f;
    float      heading_adjustment  = 0.0f;
    float      steering_adjustment = 0.0f;
    const bool line_steering_used = straight_move && line_steering_valid_;
    if (line_steering_used)
    {
        steering_adjustment                   = line_steering_adjustment_degps_;
        latest_heading_hold_adjustment_degps_ = 0.0f;
    }
    else
    {
        wall_adjustment =
            (TOF_STEERING_ENABLE && steering_mode_ != tof_wall::SteeringMode::STEERING_OFF)
                ? wallSteeringAdjustment(fwd_velocity, rot_velocity)
                : 0.0f;
        heading_adjustment  = headingHoldAdjustment(fwd_velocity, rot_velocity);
        steering_adjustment = wall_adjustment + heading_adjustment;
    }
#if !TOF_STEERING_ENABLE
    wallSteeringAdjustment(fwd_velocity, rot_velocity); // diagnostics only
#endif
    const float rotation_output =
        rotation_controller_.update(rot_velocity, rot_change_deg, steering_adjustment);
    // Diagnostic only: the direct one-tick voltage contribution of the line
    // steering term before the rotation controller's accumulated yaw error.
    latest_line_rotation_volts_ =
        line_steering_used
            ? utils::clampAbs((ROT_KP * line_steering_adjustment_degps_ * LOOP_INTERVAL_S) +
                                  (ROT_KD * line_steering_adjustment_degps_),
                              MAX_VOLTAGE)
            : 0.0f;
    latest_line_differential_volts_ = 2.0f * latest_line_rotation_volts_;

    // Mix forward + rotation outputs into per-wheel volts (mazerunner shape).
    float left_volts  = forward_output - rotation_output;
    float right_volts = forward_output + rotation_output;

    float wheelbase_radius = WHEEL_BASE_MM / 2.0f;
    float tangential_vel   = rot_velocity * (M_PI / 180.0f) * wheelbase_radius;

    float left_vel  = fwd_velocity - tangential_vel;
    float right_vel = fwd_velocity + tangential_vel;

    // Per-wheel acceleration via per-loop diff. Multiplying by LOOP_FREQUENCY
    // converts the per-loop velocity diff into mm/s².
    float left_accel         = (left_vel - prev_left_cmd_vel_mmps_) * LOOP_FREQUENCY_HZ;
    float right_accel        = (right_vel - prev_right_cmd_vel_mmps_) * LOOP_FREQUENCY_HZ;
    prev_left_cmd_vel_mmps_  = left_vel;
    prev_right_cmd_vel_mmps_ = right_vel;

    left_volts += drivetrain_->feedforward(WheelSide::LEFT, left_vel, left_accel);
    right_volts += drivetrain_->feedforward(WheelSide::RIGHT, right_vel, right_accel);

    // Clamp at hardware boundary — no slew limit (would blunt FF response).
    left_volts  = utils::clampAbs(left_volts, MAX_VOLTAGE);
    right_volts = utils::clampAbs(right_volts, MAX_VOLTAGE);
    latest_motor_differential_volts_ = right_volts - left_volts;

    drivetrain_->setVoltage(left_volts, right_volts);
}

float Motion::wallSteeringAdjustment(float fwd_velocity_mmps, float rot_velocity_degps)
{
    const bool straight_move = forward_.active() && !rotation_.active() &&
                               fwd_velocity_mmps > 1.0f && std::fabs(rot_velocity_degps) < 1.0f;
    if (!straight_move)
    {
        side_error_prev_valid_ = false;
        latest_wall_state_ =
            tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
        latest_steering_adjustment_degps_ = 0.0f;
        return 0.0f;
    }

    latest_wall_state_ =
        tof_wall::evaluate(left_wall_mm_, front_wall_mm_, right_wall_mm_, steering_mode_);
    if (!latest_wall_state_.steering_allowed)
    {
        side_error_prev_valid_            = false;
        latest_steering_adjustment_degps_ = 0.0f;
        return 0.0f;
    }

    const float side_error_delta_norm_per_s =
        side_error_prev_valid_
            ? (latest_wall_state_.side_error_norm - side_error_prev_norm_) * LOOP_FREQUENCY_HZ
            : 0.0f;
    side_error_prev_norm_             = latest_wall_state_.side_error_norm;
    side_error_prev_valid_            = true;
    latest_steering_adjustment_degps_ = tof_wall::steeringAdjustmentDegps(
        latest_wall_state_.side_error_norm, side_error_delta_norm_per_s);
    recordWallSteeringStats(latest_steering_adjustment_degps_, latest_wall_state_.source);
    return latest_steering_adjustment_degps_;
}

void Motion::recordWallSteeringStats(float adjustment_degps, tof_wall::SteeringSource source)
{
    if (wall_steering_samples_ == 0 ||
        std::fabs(adjustment_degps) > std::fabs(wall_steering_peak_degps_))
        wall_steering_peak_degps_ = adjustment_degps;

    wall_steering_sum_degps_ += adjustment_degps;
    ++wall_steering_samples_;

    if (source == tof_wall::SteeringSource::Both)
        ++wall_steering_both_samples_;
    else if (source == tof_wall::SteeringSource::Left || source == tof_wall::SteeringSource::Right)
        ++wall_steering_single_samples_;

    if (wall_steering_prev_source_valid_ && source != wall_steering_prev_source_)
        ++wall_steering_source_changes_;
    wall_steering_prev_source_       = source;
    wall_steering_prev_source_valid_ = true;
}

float Motion::headingHoldAdjustment(float fwd_velocity_mmps, float rot_velocity_degps)
{
    const bool straight_move = forward_.active() && !rotation_.active() &&
                               fwd_velocity_mmps > 1.0f && std::fabs(rot_velocity_degps) < 1.0f;
    if (!heading_hold_enabled_ || !straight_move)
    {
        latest_heading_hold_adjustment_degps_ = 0.0f;
        if (heading_hold_enabled_)
            latest_heading_hold_error_deg_ =
                utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
        return 0.0f;
    }

    latest_heading_hold_error_deg_ = utils::wrapAngle180(heading_hold_target_yaw_deg_ - yaw_deg());
#if MAZE_HEADING_HOLD_ENABLE
    latest_heading_hold_adjustment_degps_ =
        utils::clampAbs(latest_heading_hold_error_deg_ * MAZE_HEADING_HOLD_KP_DEGPS_PER_DEG,
                        MAZE_HEADING_HOLD_MAX_DEGPS);
#else
    latest_heading_hold_adjustment_degps_ = 0.0f;
#endif
    return latest_heading_hold_adjustment_degps_;
}

void Motion::stop()
{
    drivetrain_->stop();
}

void Motion::disable_drive()
{
    clear_heading_hold();
    drivetrain_->stop();
}

void Motion::emergency_stop()
{
    reset_drive_system();
    disable_drive();
}

void Motion::wait_until_position(float position_mm)
{
    while (position() < position_mm)
        sleep_ms(2);
}

void Motion::wait_until_distance(float distance_mm)
{
    const float target_mm = position() + distance_mm;
    wait_until_position(target_mm);
}
