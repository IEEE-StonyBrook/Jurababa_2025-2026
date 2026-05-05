#include "control/robot.h"

#include <cmath>

#include "common/utils.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "drivers/imu.h"
#include "drivers/tof.h"

Robot::Robot(Drivetrain* drivetrain, IMU* imu, ToF* left_tof, ToF* front_tof, ToF* right_tof)
    : drivetrain_(drivetrain), imu_(imu), left_tof_(left_tof), front_tof_(front_tof),
      right_tof_(right_tof), forward_controller_(FWD_KP, FWD_KD, LOOP_FREQUENCY_HZ),
      rotation_controller_(ROT_KP, ROT_KD, LOOP_FREQUENCY_HZ)
{
    forward_controller_.setOutputLimit(MAX_VOLTAGE);
    rotation_controller_.setOutputLimit(MAX_VOLTAGE);

    reset();
}

void Robot::reset()
{
    reset_drive_system();
}

void Robot::reset_drive_system()
{
    drivetrain_->stop();
    drivetrain_->reset();
    imu_->reset();
    forward_.reset();
    rotation_.reset();
    forward_controller_.reset();
    rotation_controller_.reset();

    prev_left_cmd_vel_mmps_  = 0.0f;
    prev_right_cmd_vel_mmps_ = 0.0f;
}

bool Robot::wallLeft()
{
    const float distance_mm = left_tof_->get_distance();
    return distance_mm > 0.0f && distance_mm < TOF_LEFT_WALL_THRESHOLD_MM;
}

bool Robot::wallRight()
{
    const float distance_mm = right_tof_->get_distance();
    return distance_mm > 0.0f && distance_mm < TOF_RIGHT_WALL_THRESHOLD_MM;
}

float Robot::frontDistance()
{
    return front_tof_->get_distance();
}

float Robot::position() const
{
    return forward_.position();
}

float Robot::velocity() const
{
    return forward_.velocity();
}

float Robot::acceleration() const
{
    return forward_.acceleration();
}

float Robot::angle() const
{
    return imu_->robot_angle();
}

float Robot::omega() const
{
    return imu_->robot_omega();
}

float Robot::alpha() const
{
    return rotation_.acceleration();
}

void Robot::set_target_velocity(float velocity_mmps)
{
    forward_.setTargetSpeed(velocity_mmps);
}

void Robot::start_move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                       float accel_mmps2)
{
    forward_.start(distance_mm, velocity(), top_speed_mmps, final_speed_mmps, accel_mmps2);
}

bool Robot::move_finished() const
{
    return !forward_.active();
}

void Robot::move(float distance_mm, float top_speed_mmps, float final_speed_mmps, float accel_mmps2)
{
    start_move(distance_mm, top_speed_mmps, final_speed_mmps, accel_mmps2);
}

void Robot::start_turn(float degrees, float top_speed_degps, float final_speed_degps,
                       float accel_degps2)
{
    rotation_.start(degrees, rotation_.velocity(), top_speed_degps, final_speed_degps,
                    accel_degps2);
}

bool Robot::turn_finished() const
{
    return !rotation_.active();
}

void Robot::turn(float degrees, float top_speed_degps, float final_speed_degps, float accel_degps2)
{
    start_turn(degrees, top_speed_degps, final_speed_degps, accel_degps2);
}

void Robot::spin_turn(float degrees, float omega_degps, float alpha_degps2)
{
    forward_.reset();
    rotation_.start(degrees, omega_degps, 0.0f, alpha_degps2);
}

void Robot::turn_IP180()
{
    static int direction = 1;
    direction *= -1;
    spin_turn(direction * 180.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::turn_IP90R()
{
    spin_turn(90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::turn_IP90L()
{
    spin_turn(-90.0f, ROBOT_MAX_TURN_SPEED_DEGPS, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void Robot::set_position(float position_mm)
{
    forward_.setPosition(position_mm);
}

void Robot::adjust_forward_position(float delta_mm)
{
    forward_.adjustPosition(delta_mm);
}

void Robot::turn_smooth(int turn_id)
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

void Robot::update()
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

void Robot::runPositionControl()
{
    if (!forward_.active() && !rotation_.active())
    {
        drivetrain_->stop();
        forward_controller_.reset();
        rotation_controller_.reset();
        prev_left_cmd_vel_mmps_  = 0.0f;
        prev_right_cmd_vel_mmps_ = 0.0f;
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
    const float forward_output  = forward_controller_.update(fwd_velocity, fwd_change_mm);
    const float rotation_output = rotation_controller_.update(rot_velocity, rot_change_deg);

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

    drivetrain_->setVoltage(left_volts, right_volts);
}

void Robot::stop()
{
    drivetrain_->stop();
}

void Robot::emergency_stop()
{
    reset_drive_system();
}
