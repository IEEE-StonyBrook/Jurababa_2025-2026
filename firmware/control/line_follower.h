#ifndef CONTROL_LINE_FOLLOWER_H
#define CONTROL_LINE_FOLLOWER_H

#include <cmath>
#include <string>

#include "common/utils.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "control/pid.h"
#include "control/profile.h"
#include "drivers/imu.h"
#include "drivers/line_sensor.h"

/**
 * @brief Line-following controller with PD steering and intersection handling
 *
 * Uses LineSensor position error to steer the robot along a line via
 * differential voltage to the drivetrain. Supports intersection detection
 * with debounce and 90-degree in-place turns using IMU heading.
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
     * @param drivetrain Drivetrain for motor control
     * @param line_sensor I2C line sensor
     * @param imu IMU for heading during turns
     * @param battery Battery for voltage scaling (may be nullptr)
     */
    LineFollower(Drivetrain* drivetrain, LineSensor* line_sensor, IMU* imu, Battery* battery);

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

  private:
    void followLine(float dt);
    void updateTurn(float dt);

    Drivetrain* drivetrain_;
    LineSensor* line_sensor_;
    IMU*        imu_;
    Battery*    battery_;

    State state_ = State::Idle;

    // PD steering
    float prev_position_error_ = 0.0f;

    // Turn tracking
    float target_yaw_       = 0.0f;
    float turn_start_yaw_   = 0.0f;
    float turn_degrees_     = 0.0f;
    bool  turn_done_        = true;

    // Intersection debounce
    bool     prev_intersection_  = false;
    uint32_t last_intersection_ms_ = 0;
};

#endif // CONTROL_LINE_FOLLOWER_H
