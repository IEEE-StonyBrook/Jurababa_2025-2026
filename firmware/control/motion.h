#ifndef CONTROL_MOTION_H
#define CONTROL_MOTION_H

#include <cstdint>

#include "common/tof_wall_utils.h"
#include "config/motion.h"
#include "control/pid.h"
#include "control/profile.h"
#include "control/smooth_turn_geometry.h"

class Drivetrain;
class IMU;

/**
 * @brief High-level motion controller (mazerunner-core Motion style)
 *
 * Per-tick: drivetrain.update() → forward.update() → rotation.update()
 * → runPositionControl(). Motion complete when both profiles report finished().
 *
 * `angle()` and `omega()` intentionally match mazerunner-core profile semantics.
 * Physical yaw remains available separately through `yaw_deg()` / `yaw_rate_degps()`.
 */
class Motion
{
  public:
    struct WallSteeringStats
    {
        float    peak_degps     = 0.0f;
        float    average_degps  = 0.0f;
        uint32_t samples        = 0;
        uint32_t source_changes = 0;
        uint32_t both_samples   = 0;
        uint32_t single_samples = 0;
    };

    /**
     * @brief Live, CLI-editable smooth turn parameters.
     *
     * RAM shadow of the config/motion.h defaults so a corner can be tuned over
     * serial without a rebuild, same pattern as LineFollower::RuntimeTuning.
     * Left and right offsets stay independent: tire loading and motor
     * asymmetry make the two sides differ.
     */
    struct SmoothTurnTuning
    {
        float speed_mmps            = ROBOT_SEARCH_TURN_SPEED_MMPS;
        float omega_degps           = ROBOT_SMOOTH_TURN_OMEGA_DEGPS;
        float alpha_degps2          = ROBOT_SMOOTH_TURN_ALPHA_DEGPS2;
        float entry_offset_left_mm  = 0.0f;
        float exit_offset_left_mm   = 0.0f;
        float entry_offset_right_mm = 0.0f;
        float exit_offset_right_mm  = 0.0f;
    };

    /**
     * @brief One editable smooth turn knob, named as the CLI spells it.
     *
     * Single source of truth: the setter validates against this table and the
     * CLI prints and lists it. Adding a knob means adding one row here.
     */
    struct SmoothTurnField
    {
        const char* name;
        float SmoothTurnTuning::* member;
        float                     min_value;
        float                     max_value;
    };

    static constexpr SmoothTurnField SMOOTH_TURN_FIELDS[] = {
        {"SPEED", &SmoothTurnTuning::speed_mmps, 50.0f, 600.0f},
        {"OMEGA", &SmoothTurnTuning::omega_degps, 30.0f, 600.0f},
        {"ALPHA", &SmoothTurnTuning::alpha_degps2, 100.0f, 5000.0f},
        {"ENTRY_L", &SmoothTurnTuning::entry_offset_left_mm, -40.0f, 40.0f},
        {"EXIT_L", &SmoothTurnTuning::exit_offset_left_mm, -40.0f, 40.0f},
        {"ENTRY_R", &SmoothTurnTuning::entry_offset_right_mm, -40.0f, 40.0f},
        {"EXIT_R", &SmoothTurnTuning::exit_offset_right_mm, -40.0f, 40.0f},
    };

    Motion(Drivetrain* drivetrain, IMU* imu);

    void reset();

    // === Wall sensing (ToF) ===
    void set_wall_distances(float left_mm, float front_mm, float right_mm);
    void set_wall_distances(float left_raw_mm, float front_raw_mm, float right_raw_mm,
                            float left_filtered_mm, float front_filtered_mm,
                            float right_filtered_mm);
    void set_steering_mode(tof_wall::SteeringMode mode);
    tof_wall::SteeringMode steeringMode() const;
    bool                   wallLeft();
    bool                   wallFront();
    bool                   wallRight();
    bool                   see_left_wall() { return wallLeft(); }
    bool                   see_front_wall() { return wallFront(); }
    bool                   see_right_wall() { return wallRight(); }
    float                  leftDistance();
    float                  frontDistance();
    float                  rightDistance();
    float                  leftRawDistance();
    float                  frontRawDistance();
    float                  rightRawDistance();
    tof_wall::WallState    wallSteeringState() const;
    float                  wallSteeringAdjustmentDegps() const;
    WallSteeringStats      wallSteeringStats() const;
    void                   resetWallSteeringStats();
    void                   set_line_steering_adjustment_degps(float adjustment_degps, bool valid);
    void                   clear_line_steering_adjustment();
    float                  lineSteeringAdjustmentDegps() const;
    bool                   lineSteeringValid() const;
    float                  lineRotationVolts() const;
    float                  lineDifferentialVolts() const;
    float                  motorDifferentialVolts() const;
    void                   set_heading_hold(float target_yaw_deg);
    void                   clear_heading_hold();
    bool                   headingHoldActive() const;
    float                  headingHoldTargetYawDeg() const;
    float                  headingHoldErrorDeg() const;
    float                  headingHoldAdjustmentDegps() const;

    // === Mazerunner-core compatible motion names ===
    void    reset_drive_system();
    void    reset_drive_control();
    void    stop();
    void    disable_drive();
    void    emergency_stop();
    float   position() const;
    float   wheel_position(WheelSide side);
    int32_t encoder_ticks(WheelSide side) const;
    float   velocity() const;
    float   acceleration() const;
    float   angle() const;
    float   omega() const;
    float   alpha() const;
    float   yaw_deg() const;
    float   yaw_rate_degps() const;
    void    set_target_velocity(float velocity_mmps);
    void    set_final_velocity(float velocity_mmps);
    void    extend_move(float distance_mm);
    bool    move_settled(float velocity_tolerance_mmps = 1.0f) const;
    bool    turn_settled(float velocity_tolerance_degps = 1.0f) const;
    bool    stop_forward(float accel_mmps2, uint32_t timeout_ms);
    bool    wait_until_forward_settled(uint32_t timeout_ms);
    void    begin_motion_sequence();
    void    end_motion_sequence();
    void    start_move(float distance_mm, float top_speed_mmps, float final_speed_mmps,
                       float accel_mmps2);
    bool    move_finished() const;
    void move(float distance_mm, float top_speed_mmps, float final_speed_mmps, float accel_mmps2);
    void start_turn(float degrees, float top_speed_degps, float final_speed_degps,
                    float accel_degps2);
    bool turn_finished() const;
    void turn(float degrees, float top_speed_degps, float final_speed_degps, float accel_degps2);
    bool spin_turn(float degrees, float omega_degps, float alpha_degps2);
    bool turn_IP180();
    bool turn_IP90R();
    bool turn_IP90L();
    void stop_at(float position_mm);
    void stop_after(float distance_mm);
    void set_position(float position_mm);
    void adjust_forward_position(float delta_mm);
    bool wait_until_position(float position_mm, uint32_t timeout_ms);
    void wait_until_position(float position_mm);
    bool wait_until_distance(float distance_mm, uint32_t timeout_ms);
    void wait_until_distance(float distance_mm);

    // === Smooth (arc) turns ===
    // One continuous forward profile carries the robot through entry straight,
    // arc and exit straight without stopping. See smooth_turn_geometry.h.
    //
    // Caller contract: arm this when the robot is HALF_CELL_MM before the turn
    // cell's centre, and clear heading hold for the whole turn — the exit
    // straight runs with rotation idle on the NEW heading, so a stale hold
    // target would steer the robot back out of the corner.
    const SmoothTurnTuning& smoothTurnTuning() const { return smooth_turn_; }
    bool                    set_smooth_turn_value(const char* name, float value);
    void                    reset_smooth_turn_tuning();
    SmoothTurnGeometry      smoothTurnGeometry(bool turn_left) const;
    // Returns false (and arms nothing) when the tuned geometry does not fit a
    // cell. A caller that has already checked smoothTurnGeometry().valid may
    // ignore it; one that has not must check, or the turn silently no-ops.
    bool start_smooth_turn(bool turn_left);

    // === Control Loop ===
    // Caller must invoke update() exactly once per tick from a deterministic
    // scheduler pinned to LOOP_FREQUENCY_HZ.
    void update();

  private:
    void  runPositionControl();
    float wallSteeringAdjustment(float fwd_velocity_mmps, float rot_velocity_degps);
    float headingHoldAdjustment(float fwd_velocity_mmps, float rot_velocity_degps);
    void  resetControlHistory();
    void  recordWallSteeringStats(float adjustment_degps, tof_wall::SteeringSource source);

    // Hardware
    Drivetrain* drivetrain_;
    IMU*        imu_;

    // Motion profiles. Names intentionally match UKMARS Motion::forward/rotation.
    Profile forward_;
    Profile rotation_;

    // Controllers — both at LOOP_FREQUENCY_HZ (single-rate, UKMARS-faithful).
    // Rotation PD consumes a per-tick yaw delta from imu_->robot_rot_change()
    // (cached_omega * dt), so the IMU's true 100 Hz sample rate is presented
    // as a held-flat per-tick increment instead of a 100 Hz gate. Each PID
    // instance still carries its own loop rate so KD * diff * loop_frequency
    // scales correctly per axis if the rates ever diverge.
    PID forward_controller_;
    PID rotation_controller_;

    // Per-wheel commanded velocity from previous tick — feeds ACC_FF as
    // `(v - prev_v) * LOOP_FREQUENCY_HZ`.
    float prev_left_cmd_vel_mmps_  = 0.0f;
    float prev_right_cmd_vel_mmps_ = 0.0f;
    bool  motion_sequence_active_  = false;

    SmoothTurnTuning smooth_turn_{};

    /**
     * @brief An arc armed by start_smooth_turn(), fired later by update().
     *
     * Everything the arc needs is snapshotted at arm time, so it stays
     * consistent with the forward distance start_smooth_turn() has already
     * committed even if the tuning changes in between. update() runs in the
     * 500 Hz timer ISR, so it must never read live tuning here.
     *
     * Keeping the whole turn on one forward profile means move_finished()
     * never flickers true mid-turn.
     */
    struct PendingArc
    {
        float trigger_mm   = 0.0f; // Forward profile position at which the arc starts.
        float angle_deg    = 0.0f;
        float omega_degps  = 0.0f;
        float alpha_degps2 = 0.0f;
        bool  armed        = false;
    };
    PendingArc pending_arc_{};

    // ToFs are physically read and filtered outside Motion. Filtered values
    // feed wall detection/steering; raw values are kept only for diagnostics.
    float left_raw_wall_mm_  = 0.0f;
    float front_raw_wall_mm_ = 0.0f;
    float right_raw_wall_mm_ = 0.0f;
    float left_wall_mm_      = 0.0f;
    float front_wall_mm_     = 0.0f;
    float right_wall_mm_     = 0.0f;

    tof_wall::WallState      latest_wall_state_{};
    tof_wall::SteeringMode   steering_mode_ = tof_wall::SteeringMode::STEERING_OFF;
    float                    latest_steering_adjustment_degps_ = 0.0f;
    float                    side_error_prev_norm_             = 0.0f;
    bool                     side_error_prev_valid_            = false;
    float                    wall_steering_peak_degps_         = 0.0f;
    float                    wall_steering_sum_degps_          = 0.0f;
    uint32_t                 wall_steering_samples_            = 0;
    uint32_t                 wall_steering_source_changes_     = 0;
    uint32_t                 wall_steering_both_samples_       = 0;
    uint32_t                 wall_steering_single_samples_     = 0;
    tof_wall::SteeringSource wall_steering_prev_source_        = tof_wall::SteeringSource::None;
    bool                     wall_steering_prev_source_valid_  = false;
    float                    line_steering_adjustment_degps_   = 0.0f;
    bool                     line_steering_valid_              = false;
    float                    latest_line_rotation_volts_       = 0.0f;
    float                    latest_line_differential_volts_   = 0.0f;
    float                    latest_motor_differential_volts_  = 0.0f;

    bool  heading_hold_enabled_                 = false;
    float heading_hold_target_yaw_deg_          = 0.0f;
    float latest_heading_hold_error_deg_        = 0.0f;
    float latest_heading_hold_adjustment_degps_ = 0.0f;
};

#endif
