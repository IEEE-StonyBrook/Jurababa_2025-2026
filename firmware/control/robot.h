#ifndef CONTROL_ROBOT_H
#define CONTROL_ROBOT_H

#include "common/tof_wall_utils.h"
#include "control/pid.h"
#include "control/profile.h"

class Drivetrain;
class IMU;

/**
 * @brief High-level motion controller (mazerunner-core Motion style)
 *
 * Per-tick: drivetrain.update() → forward.update() → rotation.update()
 * → runPositionControl(). Motion complete when both profiles report finished().
 *
 * Yaw and omega come from the IMU (deliberate divergence from mazerunner, which
 * derives both from differential encoders). Forward position comes from encoders.
 */
class Robot
{
  public:
    Robot(Drivetrain* drivetrain, IMU* imu);

    void reset();

    // === Wall sensing (ToF) ===
    void                set_wall_distances(float left_mm, float front_mm, float right_mm);
    bool                wallLeft();
    bool                wallRight();
    float               leftDistance();
    float               frontDistance();
    float               rightDistance();
    tof_wall::WallState wallSteeringState() const;
    float               wallSteeringAdjustmentDegps() const;

    // === Mazerunner-core compatible motion names ===
    void  reset_drive_system();
    void  stop();
    void  emergency_stop();
    float position() const;
    float velocity() const;
    float acceleration() const;
    float angle() const;
    float omega() const;
    float alpha() const;
    void  set_target_velocity(float velocity_mmps);
    void  set_final_velocity(float velocity_mmps);
    void  extend_move(float distance_mm);
    void  start_move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                     float accel_mmps2);
    bool  move_finished() const;
    void  move(float distance_mm, float top_speed_mmps, float final_speed_mmps, float accel_mmps2);
    void  start_turn(float degrees, float top_speed_degps, float final_speed_degps,
                     float accel_degps2);
    bool  turn_finished() const;
    void  turn(float degrees, float top_speed_degps, float final_speed_degps, float accel_degps2);
    void  spin_turn(float degrees, float omega_degps, float alpha_degps2);
    void  turn_IP180();
    void  turn_IP90R();
    void  turn_IP90L();
    void  set_position(float position_mm);
    void  adjust_forward_position(float delta_mm);
    void  turn_smooth(int turn_id);

    // === Control Loop ===
    // Caller must invoke update() exactly once per tick from a deterministic
    // scheduler pinned to LOOP_FREQUENCY_HZ.
    void update();

  private:
    void  runPositionControl();
    float wallSteeringAdjustment(float fwd_velocity_mmps, float rot_velocity_degps);

    // Hardware
    Drivetrain* drivetrain_;
    IMU*        imu_;

    // Motion profiles. Names intentionally match UKMARS Motion::forward/rotation.
    Profile forward_;
    Profile rotation_;

    // Controllers — both at LOOP_FREQUENCY_HZ (single-rate, UKMARS-faithful).
    // Rotation PD consumes a per-tick yaw delta from imu_->robot_rot_change()
    // (cached_omega * dt), so the IMU's true 100 Hz sample rate is presented
    // as a held-flat per-tick increment instead of a 100 Hz gate. Each PID
    // instance still carries its own loop rate so KD * diff * loop_frequency
    // scales correctly per axis if the rates ever diverge.
    PID forward_controller_;
    PID rotation_controller_;

    // Per-wheel commanded velocity from previous tick — feeds ACC_FF as
    // `(v - prev_v) * LOOP_FREQUENCY_HZ`.
    float prev_left_cmd_vel_mmps_  = 0.0f;
    float prev_right_cmd_vel_mmps_ = 0.0f;

    // ToFs are physically read at 50 Hz by Core 1, then cached here for the
    // 500 Hz controller. This mirrors UKMARS' "sensors update once per tick,
    // motors consume cached steering feedback" shape without doing I2C inside
    // runPositionControl().
    float left_wall_mm_  = 0.0f;
    float front_wall_mm_ = 0.0f;
    float right_wall_mm_ = 0.0f;

    tof_wall::WallState latest_wall_state_{};
    float               latest_steering_adjustment_degps_ = 0.0f;
    float               side_error_prev_norm_             = 0.0f;
    bool                side_error_prev_valid_            = false;
};

#endif
