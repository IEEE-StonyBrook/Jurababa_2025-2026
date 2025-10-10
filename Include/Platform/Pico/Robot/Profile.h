#ifndef PROFILE_H_
#define PROFILE_H_

#include <cstdint>

/**
 * @brief Motion profile for trapezoidal velocity control.
 *
 * This class generates smooth velocity commands (accel, cruise, brake)
 * based on a desired distance, speed, and acceleration. It relies on
 * odometry for actual position feedback.
 */
class Profile
{
  public:
    enum class State
    {
        Idle,         ///< Not running
        Accelerating, ///< Speeding up
        Cruising,     ///< Constant velocity
        Braking,      ///< Slowing down to final speed
        Finished      ///< Motion completed
    };

    Profile();

    /** Reset profile to idle state. */
    void reset();

    /**
     * @brief Start a new motion profile.
     * @param distance    Target distance (absolute mm or deg).
     * @param maxSpeed    Max speed allowed.
     * @param finalSpeed  Speed at the end of motion.
     * @param acceleration Positive acceleration (mm/s² or deg/s²).
     * @param startPos     Current odometry position at start (mm or deg).
     */
    void start(float distance, float maxSpeed, float finalSpeed, float acceleration,
               float startPos);

    /**
     * @brief Update profile based on current odometry.
     * @param currentPos Current odometry reading (mm or deg).
     */
    void update(float currentPos);

    /** Stop immediately and mark as finished. */
    void stop();

    /** @return True if motion has finished. */
    bool isFinished() const;

    /** @return Current commanded speed. */
    float speed() const;

    /** @return Configured acceleration. */
    float acceleration() const;

  private:
    /** @return Remaining distance to target. */
    float remainingDistance(float currentPos) const;

    /** @return Braking distance required at current speed. */
    float brakingDistance() const;

    State state_;
    float speed_;
    float targetSpeed_;
    float finalSpeed_;
    float distance_;      ///< Target distance (always positive)
    float startPosition_; ///< Odometry position at profile start
    float acceleration_;
    float invAcceleration_;
    int   directionSign_;
};

#endif