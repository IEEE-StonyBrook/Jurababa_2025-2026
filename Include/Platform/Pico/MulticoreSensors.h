#pragma once

#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <cstdint>
#include <array>
#include <functional>
#define USE_MULTICORE_SENSORS


enum class SensorMask : uint32_t {
    ALL = 0,
    LEFT_ENCODER = 1 << 0,
    RIGHT_ENCODER = 1 << 1,
    TOF_LEFT = 1 << 2,
    TOF_FRONT = 1 << 3,
    TOF_RIGHT = 1 << 4,
    IMU_YAW = 1 << 5,
    TOF_LEFT_EXIST = 1 << 6,
    TOF_FRONT_EXIST = 1 << 7,
    TOF_RIGHT_EXIST = 1 << 8
};

// Shared sensor data layout. Adjust fields to match your actual sensors.
struct MulticoreSensorData {
    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;
    int16_t tof_left_mm = 0;
    int16_t tof_front_mm = 0;
    int16_t tof_right_mm = 0;
    bool tof_left_exist = false;
    bool tof_front_exist = false;
    bool tof_right_exist = false;
    float imu_yaw = 0.0f;
    uint64_t timestamp_ms = 0;

    SensorMask valid_sensors = SensorMask::ALL;
};

// Header-only singleton-style hub for publishing/snapshotting sensor data
// across cores using pico mutex. Keep the critical sections short: publish
// or snapshot copies the whole struct under mutex, then returns.
class MulticoreSensorHub {
public:
    // Initialize the hub (must be called before use)
    static inline void init() {
        mutex_init(&get_mutex());
    }

    // Publish a new snapshot from the producer core.
    // This copies `data` into the shared store while holding the mutex.
    static inline void publish(const MulticoreSensorData &data) {
        mutex_enter_blocking(&get_mutex());

        MulticoreSensorData &back = buffers[get_back_index()];
        // Start from current front buffer to preserve unchanged fields
        back = buffers[active_index];

        SensorMask mask = data.valid_sensors;
        if (mask == SensorMask::ALL) {
            back = data; // full overwrite
        } else {
            if ((uint32_t)mask & (uint32_t)SensorMask::LEFT_ENCODER)
                back.left_encoder_count = data.left_encoder_count;
            if ((uint32_t)mask & (uint32_t)SensorMask::RIGHT_ENCODER)
                back.right_encoder_count = data.right_encoder_count;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_LEFT)
                back.tof_left_mm = data.tof_left_mm;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_FRONT)
                back.tof_front_mm = data.tof_front_mm;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_RIGHT)
                back.tof_right_mm = data.tof_right_mm;
            if ((uint32_t)mask & (uint32_t)SensorMask::IMU_YAW)
                back.imu_yaw = data.imu_yaw;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_LEFT_EXIST)
                back.tof_left_exist = data.tof_left_exist;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_FRONT_EXIST)
                back.tof_front_exist = data.tof_front_exist;
            if ((uint32_t)mask & (uint32_t)SensorMask::TOF_RIGHT_EXIST)
                back.tof_right_exist = data.tof_right_exist;
        }

        // Always update timestamp + mask
        back.timestamp_ms = data.timestamp_ms;
        back.valid_sensors = mask;

        flip_buffers();

        mutex_exit(&get_mutex());
    }

    // Take a fast snapshot of the latest published data into `out`.
    static inline void snapshot(MulticoreSensorData &out) {
        uint8_t idx = active_index; // volatile, atomic on RP2040
        out = buffers[idx];
    }

    // (snapshot API above remains; callers that need atomic multi-field
    // reads can call snapshot() which performs a guarded copy.)

    // // Helper to launch a publisher function on core1. The function should
    // // repeatedly read sensors and call MulticoreSensorHub::publish(local).
    // // Returns true if launch succeeded.
    // static inline bool launch_publisher_on_core1(void (*fn)()) {
    //     multicore_launch_core1(fn);
    //     return true;
    // }

    // static inline void multicore_fifo_push_blocking(uint32_t value) {
    //     multicore_fifo_push_blocking(value);
    // }

    // static inline uint32_t multicore_fifo_pop_blocking() {
    //     return multicore_fifo_pop_blocking();
    // }

private:
    static inline MulticoreSensorData buffers[2] = {};
    static inline volatile uint8_t active_index = 0;

    static inline uint8_t get_back_index() { return active_index ^ 1; }

    static inline void flip_buffers() { active_index ^= 1; }

    static inline mutex_t &get_mutex() {
        static mutex_t m;
        return m;
    }
};
