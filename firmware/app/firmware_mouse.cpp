#include "app/firmware_mouse.h"

#include "common/tof_wall_utils.h"
#include "config/sensors.h"
#include "control/motion.h"
#include "drivers/tof.h"
#include "pico/stdlib.h"

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

    next_tof_poll_ms_    = now_ms + TOF_MEASUREMENT_PERIOD_MS;
    const float left_mm  = left_tof_->get_distance();
    const float front_mm = front_tof_->get_distance();
    const float right_mm = right_tof_->get_distance();
    motion()->set_wall_distances(left_mm, front_mm, right_mm);
}

void FirmwareMouse::setTofSensors(ToF* left_tof, ToF* front_tof, ToF* right_tof)
{
    left_tof_         = left_tof;
    front_tof_        = front_tof;
    right_tof_        = right_tof;
    next_tof_poll_ms_ = 0;
}
