#ifndef ROBOT_H
#define ROBOT_H

#include "Common/MotionProfile.h"
#include "Common/PIDController.h"
#include "Platform/Pico/Robot/RobotUtils.h"
#include <cmath>
#include <string>

class Drivetrain;
class Sensors;

/**
 * @brief High-level robot controller with position-based control
 *
 * Implements mazerunner-core style dual PD controllers for position and
 * rotation control with trapezoidal motion profiling. Clean movement API
 * for maze navigation.
 */
class Robot
{
  public:
    /**
     * @brief Construct robot controller with hardware components
     * @param drivetrain Pointer to drivetrain interface
     * @param sensors Pointer to sensor interface
     */
    Robot(Drivetrain* drivetrain, Sensors* sensors);

    /**
     * @brief Reset all controllers and state
     */
    void reset();

    // ============================================================
    // Basic Motion Commands
    // ============================================================

    /**
     * @brief Move forward/backward a specified distance with profiling
     *
     * @param distanceMM Distance to travel (positive=forward, negative=backward)
     * @param maxVelocityMMps Maximum velocity during motion
     * @param accelerationMMps2 Acceleration/deceleration rate
     */
    void moveDistance(float distanceMM, float maxVelocityMMps, float accelerationMMps2);

    /**
     * @brief Turn in place by specified angle with profiling
     *
     * @param degrees Angle to turn (positive=CCW, negative=CW)
     * @param maxVelocityDegps Maximum angular velocity
     * @param accelerationDegps2 Angular acceleration/deceleration rate
     */
    void turnInPlace(float degrees, float maxVelocityDegps, float accelerationDegps2);

    /**
     * @brief Stop at cell center with position hold
     */
    void stopAtCenter();

    /**
     * @brief Immediately stop motors and reset state
     */
    void stop();

    // ============================================================
    // Cell Navigation (Convenience Wrappers)
    // ============================================================

    /**
     * @brief Move exactly one cell forward (180mm)
     */
    void moveToNextCell();

    /**
     * @brief Turn 90 degrees left (CCW)
     */
    void turnLeft90();

    /**
     * @brief Turn 90 degrees right (CW)
     */
    void turnRight90();

    /**
     * @brief Turn 180 degrees (alternates direction to reduce drift)
     */
    void turnAround();

    // ============================================================
    // Advanced Motion
    // ============================================================

    /**
     * @brief Execute coordinated arc turn (forward + rotation)
     *
     * @param degrees Angle to turn
     * @param radiusMM Turn radius
     */
    void smoothTurn(float degrees, float radiusMM);

    /**
     * @brief Reverse until wall contact, then recalibrate position
     *
     * @param maxDistanceMM Maximum distance to travel backward
     */
    void backToWall(float maxDistanceMM);

    /**
     * @brief Adjust lateral position using ToF sensors
     */
    void centerWithWalls();

    // ============================================================
    // Status Queries
    // ============================================================

    /**
     * @brief Check if current motion is complete
     * @return true if motion finished, false otherwise
     */
    bool isMotionComplete() const;

    /**
     * @brief Get remaining distance in current motion
     * @return Remaining distance in mm
     */
    float getRemainingDistance() const;

    /**
     * @brief Get remaining angle in current rotation
     * @return Remaining angle in degrees
     */
    float getRemainingAngle() const;

    // ============================================================
    // Control Loop
    // ============================================================

    /**
     * @brief Main control update (call at 100Hz from Core1)
     *
     * @param dt Time step in seconds since last update
     */
    void updateControl(float dt);

    // ============================================================
    // Controller Tuning
    // ============================================================

    void setForwardGains(float kp, float ki, float kd);
    void setRotationGains(float kp, float ki, float kd);

    // ============================================================
    // Legacy API (Deprecated - for backward compatibility)
    // ============================================================
    // TODO: Remove once all calling code is migrated to new API

    void driveStraightMMps(float forwardMMps, float holdYawDeg);
    void driveDistanceMM(float distanceMM, float forwardMMps);
    void turnToYawDeg(float targetYawDeg);
    void turn45Degrees(std::string side, int times);
    void arcTurnToYawDeg(float targetYawDeg, float arcLengthMM, float baseVelocityMMps);
    void arcTurn90Degrees(std::string side);
    void arcTurn45Degrees(std::string side);
    void update(float dt);  // Wrapper for updateControl
    bool isMotionDone() const;  // Wrapper for isMotionComplete

  private:
    // ============================================================
    // Motion State
    // ============================================================

    enum class MotionState
    {
        Idle,           // Not moving
        MovingForward,  // Linear motion (forward or backward)
        TurningInPlace, // Rotating on center axis
        SmoothTurning,  // Coordinated arc turn
        Stopping        // Settling to final position
    };

    MotionState state_ = MotionState::Idle;

    // ============================================================
    // Profile Update Methods
    // ============================================================

    void updateForwardProfile(float dt);
    void updateRotationProfile(float dt);

    // ============================================================
    // Completion Check Methods
    // ============================================================

    void checkForwardCompletion();
    void checkRotationCompletion();
    void checkSmoothTurnCompletion();
    void checkStoppingCompletion();

    // ============================================================
    // Control Methods
    // ============================================================

    /**
     * @brief Position control loop with incremental error accumulation
     *
     * Implements mazerunner-core dual PD controller pattern with
     * feedforward compensation.
     *
     * @param dt Time step in seconds
     */
    void runPositionControl(float dt);

    /**
     * @brief Apply slew rate limiting to duty cycle
     */
    float applySlew(float cmd, float& prevCmd, float dt);

    /**
     * @brief Get state name for logging
     */
    std::string getStateName() const;

    // ============================================================
    // Hardware Interfaces
    // ============================================================

    Drivetrain* drivetrain;
    Sensors*    sensors;

    // ============================================================
    // Motion Profiles (Trapezoidal velocity generation)
    // ============================================================

    MotionProfile forwardProfile_;   // Linear motion profiling
    MotionProfile rotationProfile_;  // Angular motion profiling

    // ============================================================
    // Position Controllers (PD-based, mazerunner-core pattern)
    // ============================================================

    PIDController forwardController_;   // Forward position control (Ki=0 for PD)
    PIDController rotationController_;  // Rotation angle control (Ki=0 for PD)

    // Incremental error accumulation (mazerunner-core pattern)
    float forwardError_     = 0.0f;
    float rotationError_    = 0.0f;
    float prevForwardError_  = 0.0f;
    float prevRotationError_ = 0.0f;

    // ============================================================
    // Target Tracking (for feedforward)
    // ============================================================

    float targetForwardVelocityMMps_      = 0.0f;  // From forward profile
    float targetAngularVelocityDegps_     = 0.0f;  // From rotation profile
    float targetForwardAccelerationMMps2_ = 0.0f;  // For Ka feedforward

    float targetYawDeg_ = 0.0f;  // Target heading for alignment

    // ============================================================
    // Runtime State
    // ============================================================

    float prevLeftDuty_  = 0.0f;  // For slew rate limiting
    float prevRightDuty_ = 0.0f;

    bool motionDone_ = true;  // Motion completion flag
};

#endif
