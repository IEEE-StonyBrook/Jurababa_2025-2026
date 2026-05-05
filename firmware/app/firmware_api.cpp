#include "app/firmware_api.h"

#include "app/multicore.h"
#include "common/tof_wall_utils.h"

bool FirmwareApi::wallLeft()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    return tof_wall::wallLeft(snap.tof_left_mm);
}

bool FirmwareApi::wallFront()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    return tof_wall::wallFront(snap.tof_front_mm);
}

bool FirmwareApi::wallRight()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    return tof_wall::wallRight(snap.tof_right_mm);
}
