#ifndef CONTROL_LINE_FOLLOWER_H
#define CONTROL_LINE_FOLLOWER_H

#include <cstdint>

#include "common/utils.h"
#include "config/config.h"
#include "control/motion.h"
#include "drivers/line_sensor.h"

/**
 * @brief Line-following controller with PD steering and intersection handling
 *
 * Uses LineSensor position error to command desired angular velocity. The
 * existing Motion forward/rotation controllers own all motor feedback and
 * voltage mixing.
 */
class LineFollower
{
  public:
    enum class State
    {
        Idle,
        FollowingLine,
        TurningLeft,
        TurningRight,
        Stopping
    };

    /**
     * @brief Constructs line follower controller
     * @param line_sensor I2C line sensor
     * @param motion Existing Motion controller that owns motor feedback
     */
    LineFollower(LineSensor* line_sensor, Motion* motion);

    /**
     * @brief Resets controller state and PD internals
     */
    void reset();

    /**
     * @brief Start following the line
     */
    void startFollowing();

    /**
     * @brief Execute one control iteration
     * @param dt Time step in seconds
     */
    void update(float dt);

    /**
     * @brief Returns true on rising edge of intersection detection (with debounce)
     */
    bool isIntersectionDetected();

    /**
     * @brief Begin a 90-degree left turn in place
     */
    void turnLeft90();

    /**
     * @brief Begin a 90-degree right turn in place
     */
    void turnRight90();

    /**
     * @brief Stop all motors immediately
     */
    void stop();

    /**
     * @brief Returns true if current motion (turn) is complete
     */
    bool isMotionDone() const;

    /**
     * @brief Returns current state
     */
    State state() const;

    uint8_t rawByte() const;
    uint8_t activeMask() const;
    bool    linePresent() const;
    bool    lineLost() const;
    float   linePosition() const;
    float   lineError() const;
    float   filteredLineError() const;
    float   steeringAdjustmentDegps() const;

  private:
    void followLine(float dt);
    void updateTurn();
    void resetControlHistory();

    LineSensor* line_sensor_;
    Motion*     motion_;

    State state_ = State::Idle;

    // Line steering diagnostics/state.
    float    prev_line_error_      = 0.0f;
    float    filtered_line_error_  = 0.0f;
    float    latest_line_position_ = 0.0f;
    float    latest_line_error_    = 0.0f;
    float    latest_steering_degps_ = 0.0f;
    bool     filter_initialized_   = false;
    bool     line_seen_            = false;
    bool     line_lost_            = false;
    uint32_t last_line_seen_ms_    = 0;

    bool turn_done_ = true;

    // Intersection debounce
    bool     prev_intersection_  = false;
    uint32_t last_intersection_ms_ = 0;
};

#endif // CONTROL_LINE_FOLLOWER_H
