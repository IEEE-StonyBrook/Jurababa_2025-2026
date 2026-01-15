#ifndef APP_MULTICORE_H
#define APP_MULTICORE_H

#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/stdlib.h"

#include <array>
#include <cstdint>

#define USE_MULTICORE_SENSORS

/**
 * @brief Bitmask for selective sensor updates
 */
enum class SensorMask : uint32_t
{
    ALL            = 0,
    LEFT_ENCODER   = 1 << 0,
    RIGHT_ENCODER  = 1 << 1,
    TOF_LEFT       = 1 << 2,
    TOF_FRONT      = 1 << 3,
    TOF_RIGHT      = 1 << 4,
    IMU_YAW        = 1 << 5,
    TOF_LEFT_WALL  = 1 << 6,
    TOF_FRONT_WALL = 1 << 7,
    TOF_RIGHT_WALL = 1 << 8
};

/**
 * @brief Shared sensor data between cores
 */
struct SensorData
{
    int32_t    left_encoder  = 0;
    int32_t    right_encoder = 0;
    int16_t    tof_left_mm   = 0;
    int16_t    tof_front_mm  = 0;
    int16_t    tof_right_mm  = 0;
    bool       wall_left     = false;
    bool       wall_front    = false;
    bool       wall_right    = false;
    float      imu_yaw       = 0.0f;
    uint64_t   timestamp_ms  = 0;
    SensorMask valid         = SensorMask::ALL;
};

/**
 * @brief Double-buffered sensor hub for lock-free multicore reads
 *
 * Producer core writes to back buffer, flips index, consumer core
 * reads from front buffer. Uses mutex only for publish operation.
 */
class SensorHub
{
  public:
    static inline void init() { mutex_init(&mutex()); }

    static inline void publish(const SensorData& data)
    {
        mutex_enter_blocking(&mutex());

        SensorData& back = buffers_[backIndex()];
        back             = buffers_[active_]; // Start from current for partial updates

        SensorMask mask = data.valid;
        if (mask == SensorMask::ALL)
        {
            back = data;
        }
        else
        {
            if ((uint32_t)mask & (uint32_t)SensorMask::LEFT_ENCODER)
                back.left_encoder = data.left_encoder;
            if ((uint32_t)mask & (uint32_t)SensorMask::RIGHT_ENCODER)
                back.right_encoder = data.right_encoder;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_LEFT)
                back.tof_left_mm = data.tof_left_mm;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_FRONT)
                back.tof_front_mm = data.tof_front_mm;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_RIGHT)
                back.tof_right_mm = data.tof_right_mm;
            if ((uint32_t)mask & (uint32_t)SensorMask::IMU_YAW)
                back.imu_yaw = data.imu_yaw;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_LEFT_WALL)
                back.wall_left = data.wall_left;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_FRONT_WALL)
                back.wall_front = data.wall_front;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_RIGHT_WALL)
                back.wall_right = data.wall_right;
        }

        back.timestamp_ms = data.timestamp_ms;
        back.valid        = mask;
        flipBuffers();

        mutex_exit(&mutex());
    }

    static inline void snapshot(SensorData& out)
    {
        uint8_t idx = active_; // Atomic on RP2040
        out         = buffers_[idx];
    }

  private:
    static inline SensorData       buffers_[2] = {};
    static inline volatile uint8_t active_     = 0;

    static inline uint8_t backIndex() { return active_ ^ 1; }
    static inline void    flipBuffers() { active_ ^= 1; }

    static inline mutex_t& mutex()
    {
        static mutex_t m;
        return m;
    }
};

#endif
