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
 * @brief High-level robot controller with position-based control
 *
 * Implements mazerunner-core style dual PD controllers for position and
 * rotation control with trapezoidal motion profiling. Includes integrated
 * sensor access for wall detection and heading tracking.
 */
class Robot
{
  public:
    Robot(Drivetrain* drivetrain, IMU* imu,
          ToF* left_tof, ToF* front_tof, ToF* right_tof);

    void reset();

    // === Wall Detection (from IMU/ToF) ===
    bool wallLeft();
    bool wallFront();
    bool wallRight();

    // === IMU Readings ===
    float yaw();               // Current heading [-180, 180]
    float omega();             // Angular velocity (deg/s)
    float yawDelta();          // Change since last call
    void resetYaw();

    // === ToF Distances ===
    float frontDistance();
    float leftDistance();
    float rightDistance();

    // === Basic Motion Commands ===
    void moveDistance(float distance_mm, float max_vel_mmps, float accel_mmps2);
    void turnInPlace(float degrees, float max_vel_degps, float accel_degps2);
    void stopAtCenter();
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
    bool motionComplete() const;
    float remainingDistance() const;
    float remainingAngle() const;

    // === Control Loop ===
    void updateControl(float dt);
    void update(float dt);

    // === Controller Tuning ===
    void setForwardGains(float kp, float ki, float kd);
    void setRotationGains(float kp, float ki, float kd);

    // Legacy API
    bool isMotionDone() const;

  private:
    enum class MotionState
    {
        Idle,
        MovingForward,
        TurningInPlace,
        SmoothTurning,
        Stopping
    };

    MotionState state_ = MotionState::Idle;

    void updateSensors(float dt);
    void updateForwardProfile(float dt);
    void updateRotationProfile(float dt);

    void checkForwardCompletion();
    void checkRotationCompletion();
    void checkSmoothTurnCompletion();
    void checkStoppingCompletion();

    void runPositionControl(float dt);
    float applySlew(float cmd, float& prev_cmd, float dt);
    std::string stateName() const;

    // Hardware
    Drivetrain* drivetrain_;
    IMU* imu_;
    ToF* left_tof_;
    ToF* front_tof_;
    ToF* right_tof_;

    // Motion profiling
    Profile forward_profile_;
    Profile rotation_profile_;

    // Controllers
    PID forward_controller_;
    PID rotation_controller_;

    // Control state
    float forward_error_ = 0.0f;
    float rotation_error_ = 0.0f;
    float prev_forward_error_ = 0.0f;
    float prev_rotation_error_ = 0.0f;

    float target_forward_vel_mmps_ = 0.0f;
    float target_angular_vel_degps_ = 0.0f;
    float target_forward_accel_mmps2_ = 0.0f;
    float target_yaw_deg_ = 0.0f;

    float prev_left_volts_ = 0.0f;
    float prev_right_volts_ = 0.0f;

    // Sensor state (merged from Sensors class)
    float prev_yaw_ = 0.0f;
    float omega_degps_ = 0.0f;
    float last_yaw_for_delta_ = 0.0f;

    bool motion_done_ = true;
};

#endif
