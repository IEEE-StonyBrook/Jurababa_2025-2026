#include "app/firmware_api.h"

#include "app/multicore.h"
#include "config/sensors.h"

namespace
{
inline bool wallFromReading(int16_t mm, int threshold_mm)
{
    // VL53L0X returns 0 for invalid/out-of-range. Treat zero as "no wall"
    // (open) so a momentary sensor dropout never injects bogus walls into
    // the maze model.
    return mm > 0 && mm < threshold_mm;
}
} // namespace

bool FirmwareApi::wallLeft()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    return wallFromReading(snap.tof_left_mm, TOF_LEFT_WALL_THRESHOLD_MM);
}

bool FirmwareApi::wallFront()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    return wallFromReading(snap.tof_front_mm, TOF_FRONT_WALL_THRESHOLD_MM);
}

bool FirmwareApi::wallRight()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    return wallFromReading(snap.tof_right_mm, TOF_RIGHT_WALL_THRESHOLD_MM);
}
