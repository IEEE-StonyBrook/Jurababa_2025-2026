#include "app/core1.h"

#include <cstdint>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "app/commands.h"
#include "app/motion_state.h"
#include "app/multicore.h"
#include "common/log.h"
#include "config/config.h"
#include "control/drivetrain.h"
#include "control/robot.h"
#include "drivers/battery.h"
#include "drivers/encoder.h"
#include "drivers/imu.h"
#include "drivers/motor.h"
#include "drivers/tof.h"

namespace
{
Battery* g_battery = nullptr;

/**
 * @brief Translate a CommandHub packet into a Robot motion call.
 *
 * Drains the FIFO each tick. Skips non-STOP commands while a motion is
 * still in progress (Robot rejects overlapping commands anyway), but
 * always honors STOP — that's the e-stop path from `Cli::pollHaltOnly`.
 *
 * Turn-unit convention: TURN_LEFT/TURN_RIGHT carry a count of 45° steps,
 * matching `API::turnLeft45/turnLeft90` (which send param=1 / param=2)
 * and `Mouse::turn45Steps`. The pre-refactor code multiplied by 90° here,
 * which silently doubled every turn on hardware.
 */
void processCommands(Robot* robot)
{
    CommandPacket cmd;
    bool          saw_stop = false;

    while (CommandHub::receiveNonBlocking(cmd))
    {
        if (cmd.type == CommandType::STOP)
        {
            saw_stop = true;
            continue;
        }

        if (!robot->motionComplete())
            continue;

        // Mark motion in flight as soon as we accept a command. Core 0's
        // `waitForMotionComplete` on the API side spins on this flag.
        MotionState::active = true;

        switch (cmd.type)
        {
            case CommandType::MOVE_FWD_HALF:
                robot->moveDistance(HALF_CELL_MM, 400.0f, 1000.0f);
                break;

            case CommandType::MOVE_FWD:
                robot->moveDistance(cmd.param * CELL_SIZE_MM, 400.0f, 1000.0f);
                break;

            case CommandType::CENTER_FROM_EDGE:
                robot->moveDistance(TO_CENTER_DISTANCE_MM, 400.0f, 1000.0f);
                break;

            case CommandType::TURN_LEFT:
                // 45° step convention — see header comment.
                robot->turnInPlace(-45.0f * cmd.param, 360.0f, 720.0f);
                break;

            case CommandType::TURN_RIGHT:
                robot->turnInPlace(45.0f * cmd.param, 360.0f, 720.0f);
                break;

            case CommandType::TURN_ARBITRARY:
                robot->turnInPlace(static_cast<float>(cmd.param), 360.0f, 720.0f);
                break;

            case CommandType::ARC_TURN_LEFT_90:
                robot->smoothTurn(-90.0f, 45.0f);
                break;

            case CommandType::ARC_TURN_RIGHT_90:
                robot->smoothTurn(90.0f, 45.0f);
                break;

            case CommandType::ARC_TURN_LEFT_45:
                robot->smoothTurn(-45.0f, 45.0f);
                break;

            case CommandType::ARC_TURN_RIGHT_45:
                robot->smoothTurn(45.0f, 45.0f);
                break;

            case CommandType::SNAPSHOT:
            case CommandType::NONE:
            default:
                break;
        }
    }

    if (saw_stop)
    {
        robot->stop();
        // Drop the in-flight flag so Core 0 wakes from `waitForMotionComplete`
        // immediately rather than spinning until the profile coasts to zero.
        MotionState::active = false;
    }
}
} // namespace

void core1Configure(Battery* battery)
{
    g_battery = battery;
}

void core1Entry()
{
    Encoder left_encoder(pio0, PIN_ENCODER_L_A, false);
    Encoder right_encoder(pio0, PIN_ENCODER_R_A, true);
    ToF     left_tof(PIN_TOF_LEFT_XSHUT, 'L');
    ToF     front_tof(PIN_TOF_FRONT_XSHUT, 'F');
    ToF     right_tof(PIN_TOF_RIGHT_XSHUT, 'R');
    IMU     imu(PIN_IMU_RX);
    Motor   left_motor(PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, true);
    Motor   right_motor(PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, true);

    Drivetrain drivetrain(&left_motor, &right_motor, &left_encoder, &right_encoder, g_battery);
    Robot      robot(&drivetrain, &imu, &left_tof, &front_tof, &right_tof);

    LOG_DEBUG("Core1: Hardware initialized");
    robot.reset();

    multicore_fifo_push_blocking(1); // Handshake: Core 0 unblocks once we're ready.

    const uint32_t  CONTROL_PERIOD_US = static_cast<uint32_t>(LOOP_INTERVAL_S * 1.0e6f);
    absolute_time_t next_tick         = make_timeout_time_us(CONTROL_PERIOD_US);

    // ToF reads happen at 50 Hz, not 500 Hz. The control loop never reads ToF
    // per tick — only `BackToWall` does, and it's a rare path.
    int       tof_publish_counter = 0;
    const int TOF_PUBLISH_DIVIDER = 10; // 500 Hz / 10 = 50 Hz

    int16_t cached_tof_left_mm  = 0;
    int16_t cached_tof_front_mm = 0;
    int16_t cached_tof_right_mm = 0;

    while (true)
    {
        if (g_battery)
            g_battery->update(); // Producer for the Battery filter in ToF/Robot mode.
        robot.update();
        processCommands(&robot);

        // Clear the in-flight flag once motion has finished AND no follow-up
        // command is queued. The pending-check matters: without it, Core 0
        // could observe `active == false` in the brief window between its
        // `send()` and our next `processCommands` pickup, then dispatch a
        // second motion before the first even started.
        if (robot.motionComplete() && !CommandHub::hasPending())
            MotionState::active = false;

        if (++tof_publish_counter >= TOF_PUBLISH_DIVIDER)
        {
            tof_publish_counter = 0;
            cached_tof_left_mm  = static_cast<int16_t>(left_tof.get_distance());
            cached_tof_front_mm = static_cast<int16_t>(front_tof.get_distance());
            cached_tof_right_mm = static_cast<int16_t>(right_tof.get_distance());
        }

        SensorData sensor_data{};
        sensor_data.left_encoder  = left_encoder.ticks();
        sensor_data.right_encoder = right_encoder.ticks();
        sensor_data.tof_left_mm   = cached_tof_left_mm;
        sensor_data.tof_front_mm  = cached_tof_front_mm;
        sensor_data.tof_right_mm  = cached_tof_right_mm;
        sensor_data.imu_yaw       = imu.robot_angle();
        sensor_data.timestamp_ms  = to_ms_since_boot(get_absolute_time());
        SensorHub::publish(sensor_data);

        sleep_until(next_tick);
        next_tick = delayed_by_us(next_tick, CONTROL_PERIOD_US);
    }
}
