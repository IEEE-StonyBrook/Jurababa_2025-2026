#include "app/firmware_api.h"

#include "app/multicore.h"
#include "common/tof_wall_utils.h"

bool FirmwareApi::wallLeft()
{
    if (use_wall_sample_)
        return tof_wall::wallLeft(wall_sample_left_mm_);

    SensorData snap;
    SensorHub::snapshot(snap);
    return tof_wall::wallLeft(snap.tof_left_mm);
}

bool FirmwareApi::wallFront()
{
    if (use_wall_sample_)
        return tof_wall::wallFront(wall_sample_front_mm_);

    SensorData snap;
    SensorHub::snapshot(snap);
    return tof_wall::wallFront(snap.tof_front_mm);
}

bool FirmwareApi::wallRight()
{
    if (use_wall_sample_)
        return tof_wall::wallRight(wall_sample_right_mm_);

    SensorData snap;
    SensorHub::snapshot(snap);
    return tof_wall::wallRight(snap.tof_right_mm);
}

void FirmwareApi::captureWallSample()
{
    SensorData snap;
    SensorHub::snapshot(snap);
    setWallSample(snap.tof_left_mm, snap.tof_front_mm, snap.tof_right_mm);
}

void FirmwareApi::setWallSample(int16_t left_mm, int16_t front_mm, int16_t right_mm)
{
    wall_sample_left_mm_  = left_mm;
    wall_sample_front_mm_ = front_mm;
    wall_sample_right_mm_ = right_mm;
    use_wall_sample_      = true;
}

void FirmwareApi::clearWallSample()
{
    use_wall_sample_ = false;
}

bool FirmwareApi::wallSample(int16_t& left_mm, int16_t& front_mm, int16_t& right_mm)
{
    if (use_wall_sample_)
    {
        left_mm  = wall_sample_left_mm_;
        front_mm = wall_sample_front_mm_;
        right_mm = wall_sample_right_mm_;
        return true;
    }

    SensorData snap;
    SensorHub::snapshot(snap);
    left_mm  = snap.tof_left_mm;
    front_mm = snap.tof_front_mm;
    right_mm = snap.tof_right_mm;
    return true;
}
