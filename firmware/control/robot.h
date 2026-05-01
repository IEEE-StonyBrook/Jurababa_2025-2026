#ifndef CONTROL_ROBOT_H
#define CONTROL_ROBOT_H

#include <cmath>
#include <string>

#include "common/utils.h"
#include "control/pid.h"
#include "control/profile.h"

class Drivetrain;
class IMU;
class ToF;

/**
 * @brief Control mode for selecting which parts of the control loop are active.
 *
 * Full:             FF + PD (default, used in Normal Mode)
 * FeedforwardOnly:  FF only, PD zeroed (for characterization)
 * FeedbackOnly:     PD only, FF zeroed (for characterization)
 * Disabled:         Skip PD/FF/setVoltage entirely. Caller drives the H-bridge
 *                   directly via Motor::set_motor_volts(). Mirrors mazerunner /
 *                   motorlab `disable_controllers() + set_closed_loop(false)`
 *                   semantics. Required by DriverLab open-loop trials so
 *                   Robot::update() doesn't fight manual voltage commands.
 */
enum class ControlMode
{
    Full,
    FeedforwardOnly,
    FeedbackOnly,
    Disabled
};

/**
 * @brief High-level robot controller (mazerunner-core style)
 *
 * Per-tick: drivetrain.update() → forward_profile.update() → rotation_profile.update()
 * → runPositionControl(). Motion complete when both profiles report finished().
 *
 * Yaw and omega come from the IMU (deliberate divergence from mazerunner, which
 * derives both from differential encoders). Forward position comes from encoders.
 */
class Robot
{
  public:
    Robot(Drivetrain* drivetrain, IMU* imu, ToF* left_tof, ToF* front_tof, ToF* right_tof);

    void reset();

    // === Wall Detection (ToF) ===
    bool wallLeft();
    bool wallFront();
    bool wallRight();

    // === IMU Readings ===
    float yaw();      // Current heading [-180, 180]
    float omega();    // Angular velocity (deg/s) — raw per-tick delta * LOOP_FREQUENCY_HZ
    float yawDelta(); // Change since last call
    void  resetYaw();

    // === ToF Distances ===
    float frontDistance();
    float leftDistance();
    float rightDistance();

    // === Motion Commands ===
    void moveDistance(float distance_mm, float max_vel_mmps, float accel_mmps2);
    void turnInPlace(float degrees, float max_vel_degps, float accel_degps2);
    void stop();

    // === Cell Navigation (Convenience Wrappers) ===
    void moveToNextCell();
    void turnLeft90();
    void turnRight90();
    void turnAround();

    // === Advanced Motion ===
    void smoothTurn(float degrees, float radius_mm);
    void backToWall(float max_distance_mm);
    void centerWithWalls();

    // === Status Queries ===
    bool  motionComplete() const;
    float remainingDistance() const;
    float remainingAngle() const;

    // === Control Loop ===
    // Caller must invoke update() exactly once per tick from a deterministic
    // scheduler pinned to LOOP_FREQUENCY_HZ.
    void update();

    // === Controller Tuning (DriverLab) ===
    // `ki` is accepted for API compatibility but is ignored — the controller is PD.
    void setForwardGains(float kp, float ki, float kd);
    void setRotationGains(float kp, float ki, float kd);
    void setControlMode(ControlMode mode);
    void setFeedforward(WheelSide side, float kv, float ks, float ka);

    // === Diagnostic Accessors (DriverLab CSV logging) ===
    float       targetForwardVel() const { return target_forward_vel_mmps_; }
    float       targetAngularVel() const { return target_angular_vel_degps_; }
    float       forwardError() const { return forward_error_; }
    float       rotationError() const { return rotation_error_; }
    float       lastLeftVolts() const { return prev_left_volts_; }
    float       lastRightVolts() const { return prev_right_volts_; }
    Drivetrain* drivetrain() const { return drivetrain_; }

  private:
    ControlMode control_mode_ = ControlMode::Full;

    void runPositionControl();

    // Hardware
    Drivetrain* drivetrain_;
    IMU*        imu_;
    ToF*        left_tof_;
    ToF*        front_tof_;
    ToF*        right_tof_;

    // Motion profiling
    Profile forward_profile_;
    Profile rotation_profile_;

    // Controllers — forward at LOOP_FREQUENCY_HZ (encoder-paced), rotation at
    // ROTATION_LOOP_HZ (IMU-paced). Each PID instance carries its own loop
    // rate so KD * diff * loop_frequency scales correctly per axis.
    PID forward_controller_;
    PID rotation_controller_;

    // Last rotation PD output. Held between IMU packets (zero-order hold)
    // because the rotation PID runs at IMU cadence (~100 Hz) while the rest
    // of the control loop runs at 500 Hz. Lives at member scope so it
    // survives across ticks.
    float rotation_output_ = 0.0f;

    // Diagnostic mirrors of controller error (mazerunner exposes these directly).
    float forward_error_  = 0.0f;
    float rotation_error_ = 0.0f;

    // Per-tick targets read from the profiles each update().
    float target_forward_vel_mmps_  = 0.0f;
    float target_angular_vel_degps_ = 0.0f;

    // Last applied motor voltage (CSV logging).
    float prev_left_volts_  = 0.0f;
    float prev_right_volts_ = 0.0f;

    // Per-wheel commanded velocity from previous tick — feeds ACC_FF as
    // `(v - prev_v) * LOOP_FREQUENCY_HZ`.
    float prev_left_cmd_vel_mmps_  = 0.0f;
    float prev_right_cmd_vel_mmps_ = 0.0f;

    // Caller-driven yawDelta() tracker. Per-tick rotation state (rot_change,
    // omega) lives in the IMU driver — Robot is a pass-through.
    float last_yaw_for_delta_ = 0.0f;

    // Sensor-driven steering correction injected into rotation PD as an additive
    // omega rate (mazerunner's `steering_adjustment` parameter). Populated by
    // centerWithWalls(); zeroed elsewhere.
    float steering_adjustment_ = 0.0f;

    bool motion_done_ = true;
};

#endif
