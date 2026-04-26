#ifndef DRIVER_LAB_PROFILE_H
#define DRIVER_LAB_PROFILE_H

/**
 * @brief Time-based trapezoidal profile for motor lab trials
 *
 * Unlike control/profile.h which is position-based (uses encoder feedback),
 * this is purely time-based for generating predictable test waveforms.
 */

enum class DriverLabProfileState
{
    IDLE,
    ACCELERATING,
    CRUISING,
    BRAKING,
    FINISHED
};

class DriverLabProfile
{
  public:
    DriverLabProfile();

    void start(float distance, float top_speed, float acceleration, float final_speed = 0.0f);
    void update(float dt);
    void reset();

    float position() const { return position_; }
    float speed() const { return speed_; }
    float acceleration() const;
    float brakingDistance() const;

    bool                  finished() const { return state_ == DriverLabProfileState::FINISHED; }
    DriverLabProfileState state() const { return state_; }

  private:
    DriverLabProfileState state_;
    float                 target_distance_;
    float                 top_speed_;
    float                 final_speed_;
    float                 acceleration_;
    float                 position_;
    float                 speed_;
    float                 direction_;
};

#endif
