#pragma once

#include "pico/mutex.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <cstdint>
#include <array>
#include <functional>
#define USE_MULTICORE_SENSORS

// Shared sensor data layout. Adjust fields to match your actual sensors.
struct MulticoreSensorData {
    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;
    int16_t tof_left_mm = 0;
    int16_t tof_front_mm = 0;
    int16_t tof_right_mm = 0;
    float imu_yaw = 0.0f;
    float imu_pitch = 0.0f;
    float imu_roll = 0.0f;
    uint64_t timestamp_ms = 0;
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
        get_shared() = data;
        mutex_exit(&get_mutex());
    }

    // Take a fast snapshot of the latest published data into `out`.
    // This copies the shared struct while holding the mutex briefly.
    static inline void snapshot(MulticoreSensorData &out) {
        mutex_enter_blocking(&get_mutex());
        out = get_shared();
        mutex_exit(&get_mutex());
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
    // Store the shared data in a static variable with internal linkage.
    static inline MulticoreSensorData &get_shared() {
        static MulticoreSensorData s = {};
        return s;
    }

    static inline mutex_t &get_mutex() {
        static mutex_t m = {};
        return m;
    }
};
