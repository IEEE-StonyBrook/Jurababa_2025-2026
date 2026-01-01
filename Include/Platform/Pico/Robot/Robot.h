#ifndef ROBOT_H
#define ROBOT_H

#include "Common/PIDController.h"
#include "Platform/Pico/Robot/RobotUtils.h"
#include <cmath>
#include <string>

class Drivetrain;
class Sensors;

class Robot
{
  public:
    // Construct robot "Brain" using drivetrain + sensors (Hardware wrappers)
    Robot(Drivetrain* drivetrain, Sensors* sensors);

    // Reset drivetrain/sensors and clear controller state (Call when switching modes)
    void reset();

    // Immediately stop motors and enter Idle mode
    void stop();

    // Drive forward at a target speed while holding a target yaw
    // forwardMMps is millimeters/second, holdYawDeg is degrees [-180, 180]
    void driveStraightMMps(float forwardMMps, float holdYawDeg);

    // Drive a fixed distance while holding the nearest 45-degree heading
    // distanceMM is always positive; forwardMMps sets direction by sign
    void driveDistanceMM(float distanceMM, float forwardMMps);

    // Turn in place to a target yaw using IMU yaw feedback
    // targetYawDeg is degrees [-180, 180]
    void turnToYawDeg(float targetYawDeg);

    // Turn to the nearest 45-degree heading after applying a +/-45 step
    // side must be "left" or "right"
    void turn45Degrees(std::string side, int times);

    // Call every control tick (e.g. 5ms or 10ms) to run controllers and update motor outputs
    void update(float dt);

    // Returns true when the current motion goal is satisfied
    bool isMotionDone() const;

    // Tuning
    void setYawGains(float kp, float ki, float kd);
    void setWheelGains(float kp, float ki, float kd);

    // Limits
    void setMaxDuty(float maxDuty);
    void setMaxDutySlewPerSec(float maxDutyChangePerSec);
    void setMaxWheelSpeedMMps(float maxWheelSpeedMMps);
    void setMaxYawDiffMMps(float maxYawDiffMMps);

    // Completion criteria
    void setYawToleranceDeg(float tolDeg);

    // DriveDistance shaping
    void setSlowdownDistMM(float slowdownDistMM);
    void setMinSlowdownScale(float minScale);

  private:
    enum class Mode
    {
        Idle,
        DriveStraight,
        DriveDistance,
        TurnInPlace
    };

    // ============= Motion Mode Handlers ============= //
    void handleIdleMode(float dt);
    void handleTurnInPlaceMode(float dt);
    void handleDriveStraightMode(float dt);
    void handleDriveDistanceMode(float dt);

    // ============= Control Building Blocks ============= //

    /**
     * @brief Applies slew rate limit to commanded duty cycle
     *
     * Prevents sudden duty changes that could cause wheel slip or mechanical stress.
     */
    float applySlew(float cmd, float& prevCmd, float dt);

    /**
     * @brief Ramps current value toward desired target at maximum rate
     *
     * Used for smooth velocity profiling during acceleration/deceleration.
     */
    float rampToward(float desired, float current, float maxDelta);

    /**
     * @brief Sets target velocities for both wheels
     *
     * Clamps to maximum safe wheel speed.
     */
    void setWheelVelocityTargetsMMps(float vLeftMMps, float vRightMMps);

    /**
     * @brief Profiles base velocity and yaw, converts to wheel targets
     *
     * Handles ramping, yaw profiling, PID control, and minimum speed enforcement.
     */
    void commandBaseAndYaw(float vBaseMMps, float dt);

    /**
     * @brief Profiles the yaw setpoint smoothly toward target
     *
     * Prevents sudden turns by ramping yaw setpoint at max angular velocity.
     * @return Profiled yaw error for PID control
     */
    float applyYawProfiling(float dt);

    /**
     * @brief Calculates yaw correction with minimum speed enforcement
     *
     * Runs PID on yaw error and enforces minimum turn speed to overcome friction.
     * @param yawError Profiled yaw error in degrees
     * @param dt Time step in seconds
     * @return Wheel speed differential (mm/s)
     */
    float calculateYawCorrection(float yawError, float dt);

    /**
     * @brief Runs low-level wheel velocity control with feedforward + PID
     *
     * Outputs duty cycles to drivetrain motors.
     */
    void runWheelVelocityControl(float dt);

    /**
     * @brief Calculates target velocity for distance-based motion
     *
     * Implements two-phase deceleration profile accounting for minimum viable speed.
     */
    float calculateTargetVelocityForDistance(float remaining, float vActual);

  private:
    Drivetrain* drivetrain;
    Sensors*    sensors;

    // High-level yaw controller: Output is wheel speed differential (mm/s)
    PIDController yawPID;

    // Low-level wheel speed controllers: Output is duty correction
    PIDController leftWheelPID;
    PIDController rightWheelPID;

    Mode mode     = Mode::Idle;
    Mode prevMode = Mode::Idle;

    // Motion targets
    float targetForwardMMps = 0.0f;
    float targetYawDeg      = 0.0f;

    // Wheel targets (consumed by runWheelVelocityControl)
    float targetLeftMMps  = 0.0f;
    float targetRightMMps = 0.0f;

    // DriveDistance state (Per wheel for robustness)
    float driveStartLeftDistMM  = 0.0f;
    float driveStartRightDistMM = 0.0f;
    float driveTargetDistMM     = 0.0f;

    // ============= Runtime State ============= //
    float prevLeftDuty  = 0.0f;
    float prevRightDuty = 0.0f;

    float vBaseCmdMMps       = 0.0f;  // Ramped base velocity command
    float currentYawSetpoint = 0.0f;  // Profiled yaw target (moves smoothly toward targetYawDeg)

    bool motionDone = true;
};

#endif