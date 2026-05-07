#include "app/firmware_mouse.h"

#include "common/tof_wall_utils.h"
#include "config/sensors.h"
#include "control/motion.h"
#include "drivers/tof.h"
#include "pico/stdlib.h"

namespace
{
float median3(float a, float b, float c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a))
        return b;
    if ((b <= a && a <= c) || (c <= a && a <= b))
        return a;
    return c;
}
} // namespace

float FirmwareMouse::Median3Filter::update(float sample_mm)
{
    samples_[next_] = sample_mm;
    next_           = static_cast<uint8_t>((next_ + 1) % TOF_FILTER_WINDOW);
    if (count_ < TOF_FILTER_WINDOW)
    {
        ++count_;
        return sample_mm;
    }
    return median3(samples_[0], samples_[1], samples_[2]);
}

void FirmwareMouse::Median3Filter::reset()
{
    next_  = 0;
    count_ = 0;
}

bool FirmwareMouse::wallLeft()
{
    serviceSensors();
    return motion() != nullptr && tof_wall::wallLeft(motion()->leftDistance());
}

bool FirmwareMouse::wallFront()
{
    serviceSensors();
    return motion() != nullptr && tof_wall::wallFront(motion()->frontDistance());
}

bool FirmwareMouse::wallRight()
{
    serviceSensors();
    return motion() != nullptr && tof_wall::wallRight(motion()->rightDistance());
}

void FirmwareMouse::serviceSensors()
{
    if (motion() == nullptr || left_tof_ == nullptr || front_tof_ == nullptr ||
        right_tof_ == nullptr)
        return;

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms < next_tof_poll_ms_)
        return;

    next_tof_poll_ms_        = now_ms + TOF_MEASUREMENT_PERIOD_MS;
    const float left_raw_mm  = left_tof_->get_distance();
    const float front_raw_mm = front_tof_->get_distance();
    const float right_raw_mm = right_tof_->get_distance();
    const float left_mm      = left_filter_.update(left_raw_mm);
    const float front_mm     = front_filter_.update(front_raw_mm);
    const float right_mm     = right_filter_.update(right_raw_mm);
    motion()->set_wall_distances(left_raw_mm, front_raw_mm, right_raw_mm, left_mm, front_mm,
                                 right_mm);
}

void FirmwareMouse::setTofSensors(ToF* left_tof, ToF* front_tof, ToF* right_tof)
{
    left_tof_         = left_tof;
    front_tof_        = front_tof;
    right_tof_        = right_tof;
    next_tof_poll_ms_ = 0;
    left_filter_.reset();
    front_filter_.reset();
    right_filter_.reset();
}
