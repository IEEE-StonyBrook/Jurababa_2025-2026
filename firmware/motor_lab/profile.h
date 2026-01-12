#ifndef MOTOR_LAB_PROFILE_H
#define MOTOR_LAB_PROFILE_H

/**
 * @brief Time-based trapezoidal profile for motor lab trials
 *
 * Unlike control/profile.h which is position-based (uses encoder feedback),
 * this is purely time-based for generating predictable test waveforms.
 */

enum class MotorLabProfileState
{
    IDLE,
    ACCELERATING,
    CRUISING,
    BRAKING,
    FINISHED
};

class MotorLabProfile
{
  public:
    MotorLabProfile();

    void start(float distance, float top_speed, float acceleration, float final_speed = 0.0f);
    void update(float dt);
    void reset();

    float position() const { return position_; }
    float speed() const { return speed_; }
    float acceleration() const;
    float brakingDistance() const;

    bool finished() const { return state_ == MotorLabProfileState::FINISHED; }
    MotorLabProfileState state() const { return state_; }

  private:
    MotorLabProfileState state_;
    float target_distance_;
    float top_speed_;
    float final_speed_;
    float acceleration_;
    float position_;
    float speed_;
    float direction_;
};

#endif
