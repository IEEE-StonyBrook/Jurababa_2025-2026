#ifndef APP_FIRMWARE_MOUSE_H
#define APP_FIRMWARE_MOUSE_H

#include <cstdint>

#include "app/mouse.h"
#include "config/sensors.h"

#if TOF_FILTER_WINDOW != 3
#error "FirmwareMouse ToF median filter expects TOF_FILTER_WINDOW == 3"
#endif

class ToF;

/**
 * @brief Hardware-backed Mouse: bridges maze-solver wall queries to live ToFs.
 *
 * Reads ToF distances directly from the Motion pointer (single-core: Motion
 * runs in the 500 Hz hardware-timer ISR; we read its cached `*Distance()`
 * getters from the main thread). Wall decisions go through the shared
 * UKMARS-style ToF threshold helper so the maze model, CLI diagnostics, and
 * Motion steering agree.
 */
class FirmwareMouse : public Mouse
{
  public:
    using Mouse::Mouse;

    bool wallLeft() override;
    bool wallFront() override;
    bool wallRight() override;
    void serviceSensors() override;

    void setTofSensors(ToF* left_tof, ToF* front_tof, ToF* right_tof);

  private:
    class Median3Filter
    {
      public:
        float update(float sample_mm);
        void  reset();

      private:
        float   samples_[TOF_FILTER_WINDOW] = {};
        uint8_t next_                       = 0;
        uint8_t count_                      = 0;
    };

    ToF*          left_tof_  = nullptr;
    ToF*          front_tof_ = nullptr;
    ToF*          right_tof_ = nullptr;
    Median3Filter left_filter_;
    Median3Filter front_filter_;
    Median3Filter right_filter_;
    uint32_t      next_tof_poll_ms_ = 0;
};

#endif
