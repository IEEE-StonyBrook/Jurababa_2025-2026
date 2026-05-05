#include "app/firmware_api.h"

#include "common/tof_wall_utils.h"
#include "control/robot.h"

bool FirmwareApi::wallLeft()
{
    if (use_wall_sample_)
        return tof_wall::wallLeft(wall_sample_left_mm_);
    return robot() != nullptr && tof_wall::wallLeft(robot()->leftDistance());
}

bool FirmwareApi::wallFront()
{
    if (use_wall_sample_)
        return tof_wall::wallFront(wall_sample_front_mm_);
    return robot() != nullptr && tof_wall::wallFront(robot()->frontDistance());
}

bool FirmwareApi::wallRight()
{
    if (use_wall_sample_)
        return tof_wall::wallRight(wall_sample_right_mm_);
    return robot() != nullptr && tof_wall::wallRight(robot()->rightDistance());
}

void FirmwareApi::captureWallSample()
{
    if (robot() == nullptr)
        return;
    setWallSample(static_cast<int16_t>(robot()->leftDistance()),
                  static_cast<int16_t>(robot()->frontDistance()),
                  static_cast<int16_t>(robot()->rightDistance()));
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

    if (robot() == nullptr)
        return false;

    left_mm  = static_cast<int16_t>(robot()->leftDistance());
    front_mm = static_cast<int16_t>(robot()->frontDistance());
    right_mm = static_cast<int16_t>(robot()->rightDistance());
    return true;
}
