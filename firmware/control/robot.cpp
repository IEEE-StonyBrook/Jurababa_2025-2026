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
      right_tof_(right_tof), forward_controller_(FWD_KP, FWD_KD, LOOP_FREQUENCY_HZ),
      rotation_controller_(ROT_KP, ROT_KD, ROTATION_LOOP_HZ)
{
    forward_controller_.setOutputLimit(MAX_VOLTAGE);
    rotation_controller_.setOutputLimit(MAX_VOLTAGE);

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

    // Errors only zero on a full Robot::reset() (e.g. boot, DriverLab trial start).
    // Per mazerunner-core: errors must integrate continuously across motions
    // so small residuals from motion N feed into motion N+1's command.
    forward_error_  = 0.0f;
    rotation_error_ = 0.0f;

    target_forward_vel_mmps_  = 0.0f;
    target_angular_vel_degps_ = 0.0f;

    prev_left_volts_         = 0.0f;
    prev_right_volts_        = 0.0f;
    prev_left_cmd_vel_mmps_  = 0.0f;
    prev_right_cmd_vel_mmps_ = 0.0f;

    steering_adjustment_ = 0.0f;
    rotation_output_     = 0.0f;
    motion_done_         = true;
}

// === Sensors ===

bool Robot::wallLeft()
{
    return left_tof_->get_distance() < TOF_LEFT_WALL_THRESHOLD_MM;
}

bool Robot::wallFront()
{
    return front_tof_->get_distance() < TOF_FRONT_WALL_THRESHOLD_MM;
}

bool Robot::wallRight()
{
    return right_tof_->get_distance() < TOF_RIGHT_WALL_THRESHOLD_MM;
}

float Robot::yaw()
{
    return imu_->robot_angle();
}

float Robot::omega()
{
    return imu_->robot_omega();
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
    imu_->reset();
    last_yaw_for_delta_ = 0.0f;
}

float Robot::frontDistance()
{
    return front_tof_->get_distance();
}

float Robot::leftDistance()
{
    return left_tof_->get_distance();
}

float Robot::rightDistance()
{
    return right_tof_->get_distance();
}

// === Motion Commands ===

void Robot::moveDistance(float distance_mm, float max_vel_mmps, float accel_mmps2)
{
    forward_profile_.start(distance_mm, max_vel_mmps, 0.0f, accel_mmps2);
    rotation_profile_.reset();
    motion_done_ = false;
}

void Robot::turnInPlace(float degrees, float max_vel_degps, float accel_degps2)
{
    rotation_profile_.start(degrees, max_vel_degps, 0.0f, accel_degps2);
    forward_profile_.reset();
    motion_done_ = false;
}

void Robot::stop()
{
    forward_profile_.reset();
    rotation_profile_.reset();

    target_forward_vel_mmps_  = 0.0f;
    target_angular_vel_degps_ = 0.0f;

    // Explicit halt: clear errors AND zero the H-bridge. Distinct from
    // "motion complete", which leaves the controller holding pose.
    forward_controller_.reset();
    rotation_controller_.reset();
    forward_error_  = 0.0f;
    rotation_error_ = 0.0f;

    prev_left_volts_         = 0.0f;
    prev_right_volts_        = 0.0f;
    prev_left_cmd_vel_mmps_  = 0.0f;
    prev_right_cmd_vel_mmps_ = 0.0f;
    steering_adjustment_     = 0.0f;
    rotation_output_         = 0.0f;
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
    float arc_length_mm = std::fabs(degrees) * (M_PI / 180.0f) * radius_mm;

    forward_profile_.start(arc_length_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, 0.0f,
                           ROBOT_BASE_ACCEL_MMPS2);
    rotation_profile_.start(degrees, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f,
                            ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
    motion_done_ = false;

    LOG_DEBUG("SmoothTurn | Angle: " + std::to_string(degrees) + " deg | Radius: " +
              std::to_string(radius_mm) + " mm | Arc: " + std::to_string(arc_length_mm) + " mm");
}

void Robot::backToWall(float max_distance_mm)
{
    forward_profile_.start(-max_distance_mm, ROBOT_BACKUP_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
    rotation_profile_.reset();
    motion_done_ = false;

    LOG_DEBUG("BackToWall | Max distance: " + std::to_string(max_distance_mm) + " mm");
}

void Robot::centerWithWalls()
{
    float left_dist  = leftDistance();
    float right_dist = rightDistance();

    if (left_dist > TOF_MAX_RANGE_MM || right_dist > TOF_MAX_RANGE_MM)
    {
        steering_adjustment_ = 0.0f;
        LOG_DEBUG("CenterWithWalls | Cannot center - wall(s) not detected");
        return;
    }

    // Mazerunner-style: feed the rotation PD an additive omega rate. The
    // controller integrates `steering_adjustment * LOOP_INTERVAL_S` into its
    // error each tick, so a momentary lateral offset produces a transient
    // correction that decays — not the perpetual bias the old direct-injection
    // pattern caused.
    float lateral_error  = (right_dist - left_dist) / 2.0f;
    steering_adjustment_ = lateral_error * CENTERING_CORRECTION_GAIN;

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

void Robot::update()
{
    // Mazerunner systick.update() shape:
    //   encoders.update(); motion.update(); motors.update_controllers(...)
    // We add imu_->update() because IMU owns rotation tracking
    // (substitutes mazerunner's encoder-derived robot_rot_change).
    drivetrain_->update();
    imu_->update();
    forward_profile_.update();
    rotation_profile_.update();

    target_forward_vel_mmps_  = forward_profile_.velocity();
    target_angular_vel_degps_ = rotation_profile_.velocity();

    motion_done_ = forward_profile_.finished() && rotation_profile_.finished();

    runPositionControl();
}

void Robot::runPositionControl()
{
    if (control_mode_ == ControlMode::Disabled)
    {
        // Caller is driving the H-bridge directly (DriverLab OL/STEP/TURN-OL/
        // TURN-STEP). Skip PD (target=0 would poison m_error_), skip FF, and
        // do NOT call drivetrain_->setVoltage() — that would clobber the
        // manual set_motor_volts() the trial just issued. drivetrain_->update()
        // and the profiles already ran in Robot::update(), so encoder-derived
        // distance/speed remain observable for CSV logging.
        return;
    }

    float fwd_change_mm  = drivetrain_->fwdChangeMm();
    float rot_change_deg = imu_->robot_rot_change();

    // Read-and-clear the IMU fresh-sample flag exactly once per tick.
    // Rotation control fires only when this is true; otherwise rotation_output_
    // holds its last value (zero-order hold between IMU packets).
    const bool fresh_yaw = imu_->has_new_yaw_sample();

    // PD outputs (volts). FeedforwardOnly skips PD; mirror error integration
    // for diagnostics so CSV reflects open-loop tracking error.
    float forward_output = 0.0f;
    if (control_mode_ != ControlMode::FeedforwardOnly)
    {
        forward_output = forward_controller_.update(target_forward_vel_mmps_, fwd_change_mm);
        if (fresh_yaw)
        {
            rotation_output_ = rotation_controller_.update(target_angular_vel_degps_,
                                                           rot_change_deg, steering_adjustment_);
        }
    }
    else
    {
        forward_error_ += target_forward_vel_mmps_ * LOOP_INTERVAL_S - fwd_change_mm;
        if (fresh_yaw)
        {
            rotation_error_ += target_angular_vel_degps_ * ROTATION_INTERVAL_S - rot_change_deg +
                               steering_adjustment_ * ROTATION_INTERVAL_S;
        }
    }

    // Mix forward + rotation outputs into per-wheel volts (mazerunner shape).
    // rotation_output_ is the last-computed value (held between IMU packets).
    float left_volts  = forward_output - rotation_output_;
    float right_volts = forward_output + rotation_output_;

    if (control_mode_ != ControlMode::FeedbackOnly)
    {
        float wheelbase_radius = WHEEL_BASE_MM / 2.0f;
        float tangential_vel   = target_angular_vel_degps_ * (M_PI / 180.0f) * wheelbase_radius;

        float left_vel  = target_forward_vel_mmps_ - tangential_vel;
        float right_vel = target_forward_vel_mmps_ + tangential_vel;

        // Per-wheel acceleration via per-loop diff. Multiplying by LOOP_FREQUENCY
        // converts the per-loop velocity diff into mm/s².
        float left_accel         = (left_vel - prev_left_cmd_vel_mmps_) * LOOP_FREQUENCY_HZ;
        float right_accel        = (right_vel - prev_right_cmd_vel_mmps_) * LOOP_FREQUENCY_HZ;
        prev_left_cmd_vel_mmps_  = left_vel;
        prev_right_cmd_vel_mmps_ = right_vel;

        float ff_left  = drivetrain_->feedforward(WheelSide::LEFT, left_vel, left_accel);
        float ff_right = drivetrain_->feedforward(WheelSide::RIGHT, right_vel, right_accel);

        left_volts += ff_left;
        right_volts += ff_right;
    }

    // Clamp at hardware boundary — no slew limit (would blunt FF response).
    left_volts  = utils::clampAbs(left_volts, MAX_VOLTAGE);
    right_volts = utils::clampAbs(right_volts, MAX_VOLTAGE);

    drivetrain_->setVoltage(left_volts, right_volts);

    prev_left_volts_  = left_volts;
    prev_right_volts_ = right_volts;

    forward_error_  = forward_controller_.error();
    rotation_error_ = rotation_controller_.error();
}

void Robot::setForwardGains(float kp, float /*ki*/, float kd)
{
    forward_controller_.setGains(kp, kd);
}

void Robot::setRotationGains(float kp, float /*ki*/, float kd)
{
    rotation_controller_.setGains(kp, kd);
}

void Robot::setControlMode(ControlMode mode)
{
    control_mode_ = mode;
}

void Robot::setFeedforward(WheelSide side, float kv, float ks, float ka)
{
    drivetrain_->setFeedforward(side, kv, ks, ka);
}
