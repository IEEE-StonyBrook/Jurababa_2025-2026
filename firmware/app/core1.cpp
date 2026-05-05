#include "app/core1.h"

#include <cmath>
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

enum class SearchActionState
{
    Idle,
    MoveAhead,
    SmoothLeadIn,
    SmoothTurning,
    SmoothExit,
    SearchForward,
    TurnBackStopAtCentre,
    TurnBackAdjust,
    TurnBackSpin,
    TurnBackExit
};

struct SearchActionSequence
{
    SearchActionState    state = SearchActionState::Idle;
    SmoothTurnParameters params{};
    float                lead_in_mm             = 0.0f;
    float                exit_mm                = 0.0f;
    int                  turn_id                = 0;
    uint32_t             deadline_ms            = 0;
    bool                 has_front_wall         = false;
    bool                 set_sensing_position   = false;
    bool                 wall_check_sent        = false;
    bool                 fallback_stop          = false;
    float                wall_check_position_mm = 0.0f;
    float                target_distance_mm     = 0.0f;
};

SearchActionSequence g_search_action;
uint16_t             g_active_command_id = 0;

void completeActiveCommand(MotionResult result);

uint32_t nowMs()
{
    return to_ms_since_boot(get_absolute_time());
}

bool timeoutExpired(uint32_t deadline_ms)
{
    return static_cast<int32_t>(nowMs() - deadline_ms) >= 0;
}

bool searchActionActive()
{
    return g_search_action.state != SearchActionState::Idle;
}

bool robotIdle(const Robot* robot)
{
    return robot->move_finished() && robot->turn_finished();
}

bool validFrontDistance(float front_mm)
{
    return front_mm > 0.0f && front_mm <= TOF_MAX_RANGE_MM;
}

bool frontWallVisible(Robot* robot)
{
    const float front_mm = robot->frontDistance();
    return validFrontDistance(front_mm) && front_mm < TOF_FRONT_WALL_THRESHOLD_MM;
}

bool commandContinuesSearchAhead(CommandType type)
{
    return type == CommandType::SEARCH_ADVANCE;
}

void acceptChainedSearchCommand(const CommandPacket& cmd)
{
    g_active_command_id              = cmd.id;
    MotionState::accepted_command_id = cmd.id;
    MotionState::current_command_id  = cmd.id;
    MotionState::last_result         = MotionResult::None;
    MotionState::active              = true;
    MotionState::wall_check_ready    = false;
}

float brakingDistanceMm(float velocity_mmps)
{
    const float v = std::fabs(velocity_mmps);
    return (v * v) / (2.0f * ROBOT_BASE_ACCEL_MMPS2);
}

void publishWallCheck(Robot* robot)
{
    MotionState::wall_check_left_mm      = static_cast<int16_t>(robot->leftDistance());
    MotionState::wall_check_front_mm     = static_cast<int16_t>(robot->frontDistance());
    MotionState::wall_check_right_mm     = static_cast<int16_t>(robot->rightDistance());
    MotionState::wall_check_yaw_deg      = robot->angle();
    MotionState::wall_check_timestamp_ms = nowMs();
    MotionState::wall_check_sequence++;
    MotionState::wall_check_command_id = g_active_command_id;
    MotionState::wall_check_ready      = true;
}

void publishSteeringDiagnostics(Robot* robot)
{
    const tof_wall::WallState wall_state   = robot->wallSteeringState();
    MotionState::steering_source           = static_cast<uint8_t>(wall_state.source);
    MotionState::steering_allowed          = wall_state.steering_allowed;
    MotionState::steering_front_blocked    = wall_state.front_blocked;
    MotionState::steering_left_error_mm    = wall_state.left_error_mm;
    MotionState::steering_right_error_mm   = wall_state.right_error_mm;
    MotionState::steering_side_error_mm    = wall_state.side_error_mm;
    MotionState::steering_adjustment_degps = robot->wallSteeringAdjustmentDegps();
    MotionState::steering_yaw_deg          = robot->angle();
    MotionState::steering_timestamp_ms     = nowMs();
}

void startSearchForward(Robot* robot, float distance_mm, float wall_check_position_mm)
{
    g_search_action.wall_check_position_mm = wall_check_position_mm;
    g_search_action.target_distance_mm     = distance_mm;
    g_search_action.wall_check_sent        = false;
    g_search_action.fallback_stop          = false;
    robot->start_move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_MAX_SEARCH_SPEED_MMPS,
                      ROBOT_BASE_ACCEL_MMPS2);
    g_search_action.state = SearchActionState::SearchForward;
}

void updateSearchForwardDecision(Robot* robot)
{
    if (!g_search_action.wall_check_sent)
        return;

    CommandPacket pending{};
    if (CommandHub::peek(pending))
    {
        if (!g_search_action.fallback_stop && commandContinuesSearchAhead(pending.type))
        {
            CommandHub::receiveNonBlocking(pending);
            completeActiveCommand(MotionResult::Completed);
            acceptChainedSearchCommand(pending);
            g_search_action.wall_check_position_mm += CELL_SIZE_MM;
            g_search_action.target_distance_mm += CELL_SIZE_MM;
            g_search_action.wall_check_sent = false;
            robot->extend_move(CELL_SIZE_MM);
            robot->set_final_velocity(ROBOT_MAX_SEARCH_SPEED_MMPS);
        }
        else
        {
            robot->set_final_velocity(0.0f);
        }
        return;
    }

    const float remaining_mm = g_search_action.target_distance_mm - std::fabs(robot->position());
    const float fallback_distance_mm = brakingDistanceMm(robot->velocity()) + 10.0f;
    if (remaining_mm <= fallback_distance_mm)
    {
        g_search_action.fallback_stop = true;
        robot->set_final_velocity(0.0f);
    }
}

void startArcTurn(Robot* robot, float degrees, float radius_mm)
{
    const float arc_length_mm = std::fabs(degrees) * (M_PI / 180.0f) * radius_mm;
    robot->start_move(arc_length_mm, ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS,
                      ROBOT_MAX_SMOOTH_TURN_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
    robot->start_turn(degrees, ROBOT_MAX_TURN_SPEED_DEGPS, 0.0f, ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
}

void startSmoothTurnSequence(Robot* robot, int turn_id)
{
    if (turn_id < 0 || turn_id >= SMOOTH_TURN_PARAM_COUNT)
        return;

    g_search_action.turn_id = turn_id;
    g_search_action.params  = SMOOTH_TURN_PARAMS[turn_id];
    g_search_action.lead_in_mm =
        (CELL_SIZE_MM + HALF_CELL_MM - g_search_action.params.entry_offset_mm) -
        SENSING_POSITION_MM;
    g_search_action.exit_mm =
        SENSING_POSITION_MM - (HALF_CELL_MM + g_search_action.params.exit_offset_mm);

    if (g_search_action.lead_in_mm > 0.125f)
    {
        robot->start_move(g_search_action.lead_in_mm, g_search_action.params.speed_mmps,
                          g_search_action.params.speed_mmps, ROBOT_BASE_ACCEL_MMPS2);
        g_search_action.state = SearchActionState::SmoothLeadIn;
        return;
    }

    robot->turn_smooth(turn_id);
    g_search_action.state = SearchActionState::SmoothTurning;
}

void startSmoothArc(Robot* robot)
{
    robot->turn_smooth(g_search_action.turn_id);
    g_search_action.state = SearchActionState::SmoothTurning;
}

void startMoveAheadSequence(Robot* robot)
{
    const bool already_running       = std::fabs(robot->velocity()) > 1.0f;
    const bool near_sensing_position = robot->position() > HALF_CELL_MM;

    if (already_running || near_sensing_position)
    {
        const float adjusted_position_mm = robot->position() - CELL_SIZE_MM;
        const float remaining_mm         = SENSING_POSITION_MM - adjusted_position_mm;
        robot->adjust_forward_position(-CELL_SIZE_MM);

        if (remaining_mm > 0.125f)
        {
            robot->start_move(remaining_mm, ROBOT_MAX_SEARCH_SPEED_MMPS,
                              ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
            g_search_action.set_sensing_position = true;
            g_search_action.state                = SearchActionState::MoveAhead;
        }
        else
        {
            robot->set_position(SENSING_POSITION_MM);
            g_search_action.state = SearchActionState::Idle;
        }
        return;
    }

    robot->start_move(CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
    g_search_action.set_sensing_position = false;
    g_search_action.state                = SearchActionState::MoveAhead;
}

void startTurnBackSpin(Robot* robot, bool move_to_sensing_after_turn)
{
    robot->turn_IP180();
    g_search_action.exit_mm =
        move_to_sensing_after_turn ? (SENSING_POSITION_MM - HALF_CELL_MM) : 0.0f;
    g_search_action.state = SearchActionState::TurnBackSpin;
}

void startTurnBackSequence(Robot* robot)
{
    const bool already_running       = std::fabs(robot->velocity()) > 1.0f;
    const bool near_sensing_position = robot->position() > HALF_CELL_MM;

    if (!already_running && !near_sensing_position)
    {
        startTurnBackSpin(robot, false);
        return;
    }

    g_search_action.has_front_wall = frontWallVisible(robot);
    const float remaining_mm       = (CELL_SIZE_MM + HALF_CELL_MM) - robot->position();
    if (remaining_mm > 0.125f)
    {
        const float final_speed_mmps =
            g_search_action.has_front_wall ? STOP_AT_CENTRE_FRONT_SPEED_MMPS : 0.0f;
        robot->start_move(remaining_mm, robot->velocity(), final_speed_mmps,
                          ROBOT_BASE_ACCEL_MMPS2);
        g_search_action.deadline_ms =
            nowMs() + (g_search_action.has_front_wall ? STOP_AT_CENTRE_FRONT_TIMEOUT_MS
                                                      : STOP_AT_CENTRE_PROFILE_TIMEOUT_MS);
        g_search_action.state = SearchActionState::TurnBackStopAtCentre;
        return;
    }

    startTurnBackSpin(robot, true);
}

void finishTurnBackStopAtCentre(Robot* robot)
{
    robot->reset_drive_system();

    if (!g_search_action.has_front_wall)
    {
        startTurnBackSpin(robot, true);
        return;
    }

    const float front_mm = robot->frontDistance();
    if (!validFrontDistance(front_mm))
    {
        startTurnBackSpin(robot, true);
        return;
    }

    const float error_mm = front_mm - FRONT_REFERENCE_MM;
    if (error_mm > FRONT_CORRECTION_TOLERANCE_MM)
    {
        robot->start_move(FRONT_CORRECTION_STEP_MM, FRONT_CORRECTION_SPEED_MMPS, 0.0f,
                          FRONT_CORRECTION_ACCEL_MMPS2);
        g_search_action.state = SearchActionState::TurnBackAdjust;
    }
    else if (error_mm < -FRONT_CORRECTION_TOLERANCE_MM)
    {
        robot->start_move(-FRONT_CORRECTION_STEP_MM, FRONT_CORRECTION_SPEED_MMPS, 0.0f,
                          FRONT_CORRECTION_ACCEL_MMPS2);
        g_search_action.state = SearchActionState::TurnBackAdjust;
    }
    else
    {
        startTurnBackSpin(robot, true);
    }
}

void tickSearchActionSequence(Robot* robot)
{
    if (!searchActionActive())
        return;

    if (g_search_action.state == SearchActionState::SearchForward)
    {
        if (!g_search_action.wall_check_sent &&
            std::fabs(robot->position()) >= g_search_action.wall_check_position_mm)
        {
            publishWallCheck(robot);
            g_search_action.wall_check_sent = true;
        }
        updateSearchForwardDecision(robot);
    }

    if (g_search_action.state == SearchActionState::SmoothLeadIn)
    {
        float trigger_mm = g_search_action.params.front_trigger_mm;
        if (robot->wallLeft())
            trigger_mm -= EXTRA_WALL_ADJUST_MM;
        if (robot->wallRight())
            trigger_mm -= EXTRA_WALL_ADJUST_MM;

        const float front_mm = robot->frontDistance();
        if (trigger_mm > 0.0f && front_mm > 0.0f && front_mm <= TOF_MAX_RANGE_MM &&
            front_mm <= trigger_mm)
        {
            startSmoothArc(robot);
            return;
        }
    }

    if (g_search_action.state == SearchActionState::TurnBackStopAtCentre &&
        g_search_action.has_front_wall)
    {
        const float front_mm = robot->frontDistance();
        if ((validFrontDistance(front_mm) && front_mm <= FRONT_REFERENCE_MM) ||
            timeoutExpired(g_search_action.deadline_ms))
        {
            finishTurnBackStopAtCentre(robot);
            return;
        }
    }

    if (!robotIdle(robot))
        return;

    switch (g_search_action.state)
    {
        case SearchActionState::MoveAhead:
            if (g_search_action.set_sensing_position)
                robot->set_position(SENSING_POSITION_MM);
            g_search_action.state = SearchActionState::Idle;
            break;

        case SearchActionState::SmoothLeadIn:
            startSmoothArc(robot);
            break;

        case SearchActionState::SmoothTurning:
            if (std::fabs(g_search_action.exit_mm) > 0.125f)
            {
                robot->start_move(g_search_action.exit_mm, g_search_action.params.speed_mmps,
                                  ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
                g_search_action.state = SearchActionState::SmoothExit;
            }
            else
            {
                robot->set_position(SENSING_POSITION_MM);
                g_search_action.state = SearchActionState::Idle;
            }
            break;

        case SearchActionState::SmoothExit:
            robot->set_position(SENSING_POSITION_MM);
            g_search_action.state = SearchActionState::Idle;
            break;

        case SearchActionState::SearchForward:
            g_search_action.state = SearchActionState::Idle;
            break;

        case SearchActionState::TurnBackStopAtCentre:
            if (!g_search_action.has_front_wall || timeoutExpired(g_search_action.deadline_ms))
                finishTurnBackStopAtCentre(robot);
            break;

        case SearchActionState::TurnBackAdjust:
            robot->reset_drive_system();
            startTurnBackSpin(robot, true);
            break;

        case SearchActionState::TurnBackSpin:
            if (g_search_action.exit_mm > 0.125f)
            {
                robot->start_move(g_search_action.exit_mm, ROBOT_MAX_SEARCH_SPEED_MMPS,
                                  ROBOT_MAX_SEARCH_SPEED_MMPS, ROBOT_BASE_ACCEL_MMPS2);
                g_search_action.state = SearchActionState::TurnBackExit;
            }
            else
            {
                g_search_action.state = SearchActionState::Idle;
            }
            break;

        case SearchActionState::TurnBackExit:
            robot->set_position(SENSING_POSITION_MM);
            g_search_action.state = SearchActionState::Idle;
            break;

        case SearchActionState::Idle:
            break;
    }
}

void completeActiveCommand(MotionResult result)
{
    if (g_active_command_id == 0)
        return;

    MotionState::last_result          = result;
    MotionState::completed_command_id = g_active_command_id;
    MotionState::current_command_id   = 0;
    MotionState::active               = false;
    g_active_command_id               = 0;
}

void acceptCommand(const CommandPacket& cmd, Robot* robot)
{
    g_active_command_id              = cmd.id;
    MotionState::accepted_command_id = cmd.id;
    MotionState::current_command_id  = cmd.id;
    MotionState::last_result         = MotionResult::None;
    MotionState::active              = true;
    MotionState::wall_check_ready    = false;

    switch (cmd.type)
    {
        case CommandType::MOVE_FWD_HALF:
            robot->move(HALF_CELL_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
            break;

        case CommandType::MOVE_FWD:
            robot->move(cmd.param * CELL_SIZE_MM, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f,
                        ROBOT_BASE_ACCEL_MMPS2);
            break;

        case CommandType::TURN_LEFT:
            robot->spin_turn(-45.0f * cmd.param, ROBOT_MAX_TURN_SPEED_DEGPS,
                             ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
            break;

        case CommandType::TURN_RIGHT:
            robot->spin_turn(45.0f * cmd.param, ROBOT_MAX_TURN_SPEED_DEGPS,
                             ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
            break;

        case CommandType::TURN_ARBITRARY:
            robot->spin_turn(static_cast<float>(cmd.param), ROBOT_MAX_TURN_SPEED_DEGPS,
                             ROBOT_BASE_ANGULAR_ACCEL_DEGPS2);
            break;

        case CommandType::ARC_TURN_LEFT_90:
            startSmoothTurnSequence(robot, SS90EL);
            break;

        case CommandType::ARC_TURN_RIGHT_90:
            startSmoothTurnSequence(robot, SS90ER);
            break;

        case CommandType::ARC_TURN_LEFT_45:
            startArcTurn(robot, -45.0f, 45.0f);
            break;

        case CommandType::ARC_TURN_RIGHT_45:
            startArcTurn(robot, 45.0f, 45.0f);
            break;

        case CommandType::TURN_SMOOTH:
            startSmoothTurnSequence(robot, cmd.param);
            break;

        case CommandType::MOVE_AHEAD:
            startMoveAheadSequence(robot);
            break;

        case CommandType::TURN_BACK:
            startTurnBackSequence(robot);
            break;

        case CommandType::MOVE_MM:
        {
            const float distance_mm = static_cast<float>(cmd.param) / 10.0f;
            robot->move(distance_mm, ROBOT_MAX_SEARCH_SPEED_MMPS, 0.0f, ROBOT_BASE_ACCEL_MMPS2);
            break;
        }

        case CommandType::SEARCH_START_FROM_WALL_CHECK:
            startSearchForward(robot, CELL_SIZE_MM + WALL_CHECK_TO_CENTER_MM, CELL_SIZE_MM);
            break;

        case CommandType::SEARCH_ADVANCE:
            startSearchForward(robot, CELL_SIZE_MM, CENTER_TO_NEXT_WALL_CHECK_MM);
            break;

        case CommandType::STOP:
            CommandHub::requestStop();
            completeActiveCommand(MotionResult::Stopped);
            break;

        case CommandType::NONE:
        default:
            completeActiveCommand(MotionResult::Rejected);
            break;
    }
}

void processCommands(Robot* robot)
{
    if (CommandHub::stopRequested())
    {
        CommandHub::clearStopRequested();
        CommandHub::clear();
        robot->emergency_stop();
        g_search_action.state = SearchActionState::Idle;
        completeActiveCommand(MotionResult::Stopped);
        MotionState::last_result = MotionResult::Stopped;
        return;
    }

    if (g_active_command_id != 0 && robotIdle(robot) && !searchActionActive())
        completeActiveCommand(MotionResult::Completed);

    if (g_active_command_id != 0 || !robotIdle(robot) || searchActionActive())
        return;

    CommandPacket cmd;
    if (CommandHub::receiveNonBlocking(cmd))
        acceptCommand(cmd, robot);
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
    Robot      robot(&drivetrain, &imu);

    LOG_DEBUG("Core1: Hardware initialized");
    robot.reset();

    multicore_fifo_push_blocking(1); // Handshake: Core 0 unblocks once we're ready.

    const uint32_t  CONTROL_PERIOD_US = static_cast<uint32_t>(LOOP_INTERVAL_S * 1.0e6f);
    absolute_time_t next_tick         = make_timeout_time_us(CONTROL_PERIOD_US);

    // ToF publishes happen at 50 Hz, not 500 Hz. Robot consumes these cached
    // distances for UKMARS-style wall steering and smooth-turn triggers so the
    // 500 Hz loop never performs I2C reads.
    int       tof_publish_counter = 0;
    const int TOF_PUBLISH_DIVIDER = 10; // 500 Hz / 10 = 50 Hz

    int16_t cached_tof_left_mm  = static_cast<int16_t>(TOF_OUT_OF_RANGE_MM);
    int16_t cached_tof_front_mm = static_cast<int16_t>(TOF_OUT_OF_RANGE_MM);
    int16_t cached_tof_right_mm = static_cast<int16_t>(TOF_OUT_OF_RANGE_MM);
    robot.set_wall_distances(cached_tof_left_mm, cached_tof_front_mm, cached_tof_right_mm);

    while (true)
    {
        if (g_battery)
            g_battery->update(); // Producer for the Battery filter in ToF/Robot mode.
        robot.update();
        publishSteeringDiagnostics(&robot);
        tickSearchActionSequence(&robot);
        processCommands(&robot);

        if (++tof_publish_counter >= TOF_PUBLISH_DIVIDER)
        {
            tof_publish_counter = 0;
            cached_tof_left_mm  = static_cast<int16_t>(left_tof.get_distance());
            cached_tof_front_mm = static_cast<int16_t>(front_tof.get_distance());
            cached_tof_right_mm = static_cast<int16_t>(right_tof.get_distance());
            robot.set_wall_distances(cached_tof_left_mm, cached_tof_front_mm, cached_tof_right_mm);
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
