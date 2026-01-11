#ifndef MOTORLAB_PROFILE_H
#define MOTORLAB_PROFILE_H

/**
 * @file MotorLabProfile.h
 * @brief Time-based trapezoidal profile for motor lab trials
 *
 * WHY THIS IS SEPARATE FROM MotionProfile:
 * =========================================
 *
 * MotionProfile (Common/MotionProfile.h):
 *   - Position-based: uses encoder feedback to determine current state
 *   - update(currentPosition, dt) - relies on external position tracking
 *   - Better for closed-loop control where actual position drives transitions
 *   - Used by Robot class for real motion control
 *
 * MotorLabProfile (this class):
 *   - Time-based: generates setpoints purely from elapsed time
 *   - update(dt) - self-contained, no external feedback needed
 *   - Better for open-loop testing where you want predictable waveforms
 *   - Used by MotorLab for generating test profiles
 *
 * EXAMPLE USAGE:
 *   MotorLabProfile profile;
 *   profile.start(90.0f, 200.0f, 500.0f, 0.0f);  // 90mm at 200mm/s, 500mm/s^2
 *
 *   while (!profile.isFinished()) {
 *       profile.update(dt);
 *       float setpoint_pos   = profile.getPosition();      // mm
 *       float setpoint_speed = profile.getSpeed();         // mm/s
 *       float setpoint_accel = profile.getAcceleration();  // mm/s^2
 *       // Apply feedforward: V = bias + speed_ff * speed + acc_ff * accel
 *   }
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
 * @brief Time-based trapezoidal motion profile
 *
 * Generates position (mm), velocity (mm/s), and acceleration (mm/s²)
 * setpoints for controlled motion during calibration trials.
 *
 * All units are in mm for consistency with Config.h constants.
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
