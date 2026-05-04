#ifndef CONTROL_PROFILE_H
#define CONTROL_PROFILE_H

/**
 * @brief Trapezoidal motion profile generator (mazerunner-core style)
 *
 * Time-based: each tick advances `current_velocity_` by `accel * LOOP_INTERVAL_S`
 * and advances `current_position_` by `current_velocity_ * LOOP_INTERVAL_S`.
 * State transitions test the COMMANDED `current_position_` against the braking-
 * distance criterion, never measured position. This decouples profile timing
 * from controller tracking error.
 *
 * Units are scalar (mm or deg). Caller integrates controller tracking against
 * measured position separately.
 */
class Profile
{
  public:
    enum class State
    {
        Idle,
        Accelerating,
        Braking,
        Finished
    };

    Profile();

    /**
     * @brief Start a new motion profile.
     * @param target_distance  Total signed distance (mm or deg). Sign sets direction.
     * @param top_speed        Cruise speed magnitude.
     * @param final_speed      Terminal speed magnitude (>=0). Default 0.
     *                         Non-zero enables chained moves through cells.
     * @param acceleration     Accel/decel magnitude.
     */
    void start(float target_distance, float top_speed, float final_speed, float acceleration);
    void start(float target_distance, float start_speed, float top_speed, float final_speed,
               float acceleration);

    /** @brief Advance profile by one fixed-rate tick. */
    void update();

    float velocity() const { return current_velocity_; }
    float acceleration() const { return current_acceleration_; }
    float position() const { return current_position_; }
    float remaining() const;
    bool  finished() const { return state_ == State::Finished; }
    State state() const { return state_; }

    void reset();
    void setTargetSpeed(float speed);
    void setPosition(float position);
    void adjustPosition(float delta);

  private:
    State state_;
    int   direction_;
    float target_distance_;
    float target_speed_;
    float top_speed_;
    float final_speed_;
    float acceleration_;
    float current_velocity_;
    float current_acceleration_;
    float current_position_;

    float brakingDistance() const; // (v^2 - final_speed^2) / (2a)
};

#endif
