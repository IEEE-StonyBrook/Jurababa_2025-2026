#ifndef APP_FIRMWARE_API_H
#define APP_FIRMWARE_API_H

#include "app/api.h"

/**
 * @brief Hardware-backed API: bridges maze-solver wall queries to real ToFs.
 *
 * Base `API::wallLeft/Front/Right` returns false on hardware (it has no
 * sensor handle). `FirmwareApi` overrides those three to read the latest
 * `SensorData` snapshot published by Core 1 and compares against thresholds
 * in `firmware/config/sensors.h`.
 *
 * Reads come exclusively from `SensorHub::snapshot()` because Core 1 owns
 * the physical I2C — we never touch a ToF object directly from Core 0. Wall
 * decisions use the shared UKMARS-style ToF calibration helper so the maze
 * model, CLI diagnostics, and Robot steering agree on the same thresholds.
 */
class FirmwareApi : public API
{
  public:
    using API::API;

    bool wallLeft() override;
    bool wallFront() override;
    bool wallRight() override;
    void captureWallSample() override;
    void setWallSample(int16_t left_mm, int16_t front_mm, int16_t right_mm) override;
    void clearWallSample() override;
    bool wallSample(int16_t& left_mm, int16_t& front_mm, int16_t& right_mm) override;

  private:
    bool    use_wall_sample_      = false;
    int16_t wall_sample_left_mm_  = 0;
    int16_t wall_sample_front_mm_ = 0;
    int16_t wall_sample_right_mm_ = 0;
};

#endif
