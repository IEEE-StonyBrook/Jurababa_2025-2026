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
 * the physical I2C — we never touch a ToF object directly from Core 0.
 *
 * A ToF reading of 0 mm means "invalid / out of range" on the VL53L0X, so
 * the predicate is `0 < distance_mm < threshold_mm` — zero conservatively
 * reports "no wall", which is the same convention mazerunner-core uses.
 */
class FirmwareApi : public API
{
  public:
    using API::API;

    bool wallLeft() override;
    bool wallFront() override;
    bool wallRight() override;
};

#endif
