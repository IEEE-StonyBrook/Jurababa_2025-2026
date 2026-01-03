#ifndef SENSORS_H
#define SENSORS_H

#include "Platform/Pico/Robot/IMU.h"
#include "Platform/Pico/Robot/ToF.h"

/**
 * @brief Sensor aggregation and processing layer
 *
 * Provides high-level interface to robot sensors (IMU and ToF distance sensors).
 * Handles wall detection thresholding and angular velocity estimation from
 * IMU yaw readings.
 */
class Sensors
{
  public:
    /**
     * @brief Constructs sensor interface with hardware components
     * @param imu Pointer to IMU interface
     * @param left_tof Pointer to left ToF sensor
     * @param front_tof Pointer to front ToF sensor
     * @param right_tof Pointer to right ToF sensor
     */
    Sensors(IMU* imu, ToF* left_tof, ToF* front_tof, ToF* right_tof);

    /**
     * @brief Checks if wall detected on left side
     * @return True if distance below threshold
     */
    bool isWallLeft();

    /**
     * @brief Checks if wall detected in front
     * @return True if distance below threshold
     */
    bool isWallFront();

    /**
     * @brief Checks if wall detected on right side
     * @return True if distance below threshold
     */
    bool isWallRight();

    /**
     * @brief Returns current robot heading
     * @return Yaw angle in degrees, range [-180, 180]
     */
    float getYaw();

    /**
     * @brief Returns estimated angular velocity
     * @return Angular velocity in degrees per second (filtered)
     */
    float getAngularVelocityDegps();

    /**
     * @brief Returns yaw change since last call (incremental tracking)
     *
     * This method tracks angular position changes between calls, enabling
     * incremental error accumulation pattern for rotation control.
     *
     * @return Angular delta in degrees since last call
     */
    float getYawDelta();

    /**
     * @brief Returns front ToF sensor distance
     * @return Distance in millimeters
     */
    float getFrontDistanceMM();

    /**
     * @brief Returns left ToF sensor distance
     * @return Distance in millimeters
     */
    float getLeftDistanceMM();

    /**
     * @brief Returns right ToF sensor distance
     * @return Distance in millimeters
     */
    float getRightDistanceMM();

    /**
     * @brief Resets IMU yaw to zero at current heading
     */
    void resetYaw();

    /**
     * @brief Resets position for odometry recalibration
     *
     * Called after backing into wall to reset known position reference.
     */
    void resetPosition();

    /**
     * @brief Updates angular velocity estimate
     * @param time_delta Time step in seconds since last update
     */
    void update(float time_delta);

  private:
    // Sensor hardware interfaces
    IMU* imu_;
    ToF* left_tof_;
    ToF* front_tof_;
    ToF* right_tof_;

    // Angular velocity estimation state
    float previous_yaw_ = 0.0f;
    float current_angular_velocity_deg_per_second_ = 0.0f;

    // Yaw delta tracking for incremental error accumulation
    float last_yaw_for_delta_ = 0.0f;
};

#endif