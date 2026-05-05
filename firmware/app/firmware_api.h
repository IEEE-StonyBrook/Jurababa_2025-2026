#ifndef APP_FIRMWARE_API_H
#define APP_FIRMWARE_API_H

#include "app/api.h"

/**
 * @brief Hardware-backed API: bridges maze-solver wall queries to live ToFs.
 *
 * Reads ToF distances directly from the Robot pointer (single-core: Robot
 * runs in the 500 Hz hardware-timer ISR; we read its cached `*Distance()`
 * getters from the main thread). Wall decisions go through the shared
 * UKMARS-style ToF threshold helper so the maze model, CLI diagnostics, and
 * Robot steering agree.
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
