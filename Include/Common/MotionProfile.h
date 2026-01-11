#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

/**
 * @file MotionProfile.h
 * @brief Trapezoidal motion profile generator for smooth velocity control
 *
 * Generates smooth acceleration, cruise, and deceleration phases for position-based
 * motion control. Inspired by UKMARS mazerunner-core motion profiling.
 */

class MotionProfile
{
  public:
    /**
     * @brief Motion profile states
     */
    enum class State
    {
        Idle,          // Not running
        Accelerating,  // Increasing velocity
        Cruising,      // Constant velocity
        Decelerating,  // Decreasing velocity
        Finished       // Target reached
    };

    /**
     * @brief Constructor
     */
    MotionProfile();

    /**
     * @brief Start a new motion profile
     *
     * @param targetDistance Total distance to travel (mm or degrees)
     * @param maxVelocity Maximum velocity during cruise phase (mm/s or deg/s)
     * @param acceleration Acceleration/deceleration rate (mm/s² or deg/s²)
     * @param initialPosition Starting position (mm or degrees)
     */
    void start(float targetDistance, float maxVelocity, float acceleration, float initialPosition);

    /**
     * @brief Update profile based on current position
     *
     * @param currentPosition Current position from feedback (mm or degrees)
     * @param dt Time step since last update (seconds)
     */
    void update(float currentPosition, float dt);

    /**
     * @brief Get target velocity at current point in profile
     *
     * @return Target velocity (mm/s or deg/s)
     */
    float getTargetVelocity() const;

    /**
     * @brief Get target acceleration at current point in profile
     *
     * @return Target acceleration (mm/s² or deg/s²)
     */
    float getTargetAcceleration() const;

    /**
     * @brief Get remaining distance to target
     *
     * @return Remaining distance (mm or degrees)
     */
    float getRemainingDistance() const;

    /**
     * @brief Check if profile has completed
     *
     * @return true if motion is complete, false otherwise
     */
    bool isFinished() const;

    /**
     * @brief Get current profile state
     *
     * @return Current State enum value
     */
    State getState() const;

    /**
     * @brief Reset profile to idle state
     */
    void reset();

  private:
    // Profile state
    State state_;

    // Profile parameters
    float targetDistance_;     // Total distance to travel
    float maxVelocity_;        // Maximum velocity
    float acceleration_;       // Acceleration/deceleration rate
    float initialPosition_;    // Starting position

    // Runtime tracking
    float currentVelocity_;       // Current velocity setpoint
    float currentAcceleration_;   // Current acceleration setpoint

    // Pre-calculated profile distances
    float accelDistance_;   // Distance covered during acceleration
    float decelDistance_;   // Distance covered during deceleration
    float cruiseDistance_;  // Distance covered at constant velocity
    float cruiseVelocity_;  // Actual cruise velocity (may be < maxVelocity for short moves)

    // Completion tracking
    float positionTolerance_;   // Position tolerance for completion (mm or deg)
    float velocityTolerance_;   // Velocity tolerance for completion (mm/s or deg/s)

    /**
     * @brief Calculate profile parameters during start()
     */
    void calculateProfileParameters();

    /**
     * @brief Determine profile state based on distance traveled
     *
     * @param distanceTraveled Distance from initial position
     */
    void updateState(float distanceTraveled);
};

#endif  // MOTIONPROFILE_H
