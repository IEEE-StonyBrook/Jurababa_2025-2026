#ifndef CONTROL_PROFILE_H
#define CONTROL_PROFILE_H

/**
 * @brief Trapezoidal motion profile generator for smooth velocity control
 *
 * Generates smooth acceleration, cruise, and deceleration phases for position-based
 * motion control. Inspired by UKMARS mazerunner-core motion profiling.
 */
class Profile
{
  public:
    enum class State
    {
        Idle,
        Accelerating,
        Cruising,
        Decelerating,
        Finished
    };

    Profile();

    /**
     * @brief Start a new motion profile
     * @param target_distance Total distance to travel (mm or degrees)
     * @param max_velocity Maximum velocity during cruise phase
     * @param acceleration Acceleration/deceleration rate
     * @param initial_position Starting position
     */
    void start(float target_distance, float max_velocity, float acceleration, float initial_position);

    /**
     * @brief Update profile based on current position
     * @param current_position Current position from feedback
     * @param dt Time step since last update (seconds)
     */
    void update(float current_position, float dt);

    /**
     * @brief Target velocity at current point in profile
     */
    float velocity() const;

    /**
     * @brief Target acceleration at current point in profile
     */
    float acceleration() const;

    /**
     * @brief Remaining distance to target
     */
    float remaining() const;

    /**
     * @brief Check if profile has completed
     */
    bool finished() const;

    /**
     * @brief Current profile state
     */
    State state() const;

    /**
     * @brief Reset profile to idle state
     */
    void reset();

  private:
    State state_;
    float target_distance_;
    float max_velocity_;
    float acceleration_;
    float initial_position_;

    float current_velocity_;
    float current_acceleration_;

    float accel_distance_;
    float decel_distance_;
    float cruise_distance_;
    float cruise_velocity_;

    float position_tolerance_;
    float velocity_tolerance_;

    void calculateProfileParameters();
    void updateState(float distance_traveled);
};

#endif
