#ifndef MOTORLAB_PROFILE_H
#define MOTORLAB_PROFILE_H

/**
 * @file MotorLabProfile.h
 * @brief Motion profile generator for motor lab trials
 *
 * Generates trapezoidal motion profiles for position control during
 * motor characterization trials. Simpler than the full MotionProfile
 * class, focused on motorlab-specific functionality.
 *
 * Based on UKMARS motorlab by Peter Harrison.
 */

/**
 * @brief Profile states
 */
enum class MotorLabProfileState {
    IDLE,           // Not running
    ACCELERATING,   // Ramping up velocity
    CRUISING,       // Constant velocity
    BRAKING,        // Ramping down velocity
    FINISHED        // Motion complete
};

/**
 * @brief Simple trapezoidal motion profile for motorlab
 *
 * Generates position, velocity, and acceleration setpoints for
 * controlled motion during calibration trials.
 */
class MotorLabProfile {
  public:
    MotorLabProfile();

    /**
     * @brief Start a new motion profile
     *
     * @param distance Target distance to travel (degrees or mm)
     * @param top_speed Maximum velocity during cruise (units/s)
     * @param acceleration Acceleration rate (units/s²)
     * @param final_speed Speed at end of profile (typically 0)
     */
    void start(float distance, float top_speed, float acceleration, float final_speed = 0.0f);

    /**
     * @brief Update profile for one time step
     *
     * Call this once per control loop iteration to advance
     * the profile and update setpoints.
     *
     * @param dt Time step in seconds
     */
    void update(float dt);

    /**
     * @brief Get current position setpoint
     * @return Position in profile units (degrees or mm)
     */
    float getPosition() const { return position_; }

    /**
     * @brief Get current velocity setpoint
     * @return Velocity in units/s
     */
    float getSpeed() const { return speed_; }

    /**
     * @brief Get current acceleration setpoint
     *
     * Returns positive during acceleration phase, negative during braking,
     * and zero during cruise or idle.
     *
     * @return Acceleration in units/s²
     */
    float getAcceleration() const;

    /**
     * @brief Check if profile has completed
     * @return true if finished, false otherwise
     */
    bool isFinished() const { return state_ == MotorLabProfileState::FINISHED; }

    /**
     * @brief Get current profile state
     * @return Current state enum
     */
    MotorLabProfileState getState() const { return state_; }

    /**
     * @brief Reset profile to idle state
     */
    void reset();

    /**
     * @brief Calculate braking distance
     *
     * Returns distance needed to decelerate from current speed to
     * final speed at configured deceleration rate.
     *
     * @return Braking distance in profile units
     */
    float getBrakingDistance() const;

  private:
    // Profile state
    MotorLabProfileState state_;

    // Profile parameters
    float target_distance_;
    float top_speed_;
    float final_speed_;
    float acceleration_;  // Acceleration/deceleration rate

    // Runtime variables
    float position_;      // Current position setpoint
    float speed_;         // Current velocity setpoint

    // Sign of motion (for bidirectional support)
    float direction_;     // +1.0 or -1.0
};

#endif  // MOTORLAB_PROFILE_H
