#include "app/firmware_api.h"

#include "common/tof_wall_utils.h"
#include "config/sensors.h"
#include "control/robot.h"
#include "drivers/tof.h"
#include "pico/stdlib.h"

bool FirmwareApi::wallLeft()
{
    serviceSensors();
    if (use_wall_sample_)
        return tof_wall::wallLeft(wall_sample_left_mm_);
    return robot() != nullptr && tof_wall::wallLeft(robot()->leftDistance());
}

bool FirmwareApi::wallFront()
{
    serviceSensors();
    if (use_wall_sample_)
        return tof_wall::wallFront(wall_sample_front_mm_);
    return robot() != nullptr && tof_wall::wallFront(robot()->frontDistance());
}

bool FirmwareApi::wallRight()
{
    serviceSensors();
    if (use_wall_sample_)
        return tof_wall::wallRight(wall_sample_right_mm_);
    return robot() != nullptr && tof_wall::wallRight(robot()->rightDistance());
}

void FirmwareApi::captureWallSample()
{
    serviceSensors();
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
    serviceSensors();

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

void FirmwareApi::serviceSensors()
{
    if (robot() == nullptr || left_tof_ == nullptr || front_tof_ == nullptr ||
        right_tof_ == nullptr)
        return;

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms < next_tof_poll_ms_)
        return;

    next_tof_poll_ms_    = now_ms + TOF_MEASUREMENT_PERIOD_MS;
    const float left_mm  = left_tof_->get_distance();
    const float front_mm = front_tof_->get_distance();
    const float right_mm = right_tof_->get_distance();
    robot()->set_wall_distances(left_mm, front_mm, right_mm);
}

void FirmwareApi::setTofSensors(ToF* left_tof, ToF* front_tof, ToF* right_tof)
{
    left_tof_         = left_tof;
    front_tof_        = front_tof;
    right_tof_        = right_tof;
    next_tof_poll_ms_ = 0;
}
