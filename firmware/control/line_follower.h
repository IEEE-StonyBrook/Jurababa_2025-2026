#ifndef CONTROL_LINE_FOLLOWER_H
#define CONTROL_LINE_FOLLOWER_H

#include <cstdint>

#include "common/utils.h"
#include "config/config.h"
#include "control/motion.h"
#include "drivers/line_sensor.h"

/**
 * @brief Line-following controller with PD steering and intersection handling
 *
 * Uses LineSensor position error to command desired angular velocity. The
 * existing Motion forward/rotation controllers own all motor feedback and
 * voltage mixing.
 */
class LineFollower
{
  public:
    struct RuntimeTuning
    {
        float kp_base_degps_per_slot = LINE_KP_BASE_DEGPS_PER_SLOT;
        float kd_base_deg_per_slot   = LINE_KD_BASE_DEG_PER_SLOT;
        float target_speed_mmps      = LINE_TARGET_SPEED_MMPS;
        float max_speed_mmps         = LINE_MAX_SPEED_MMPS;
        float min_speed_mmps         = LINE_MIN_SPEED_MMPS;
    };

    enum class State
    {
        Idle,
        FollowingLine,
        TurningLeft,
        TurningRight,
        Stopping
    };

    /**
     * @brief Constructs line follower controller
     * @param line_sensor I2C line sensor
     * @param motion Existing Motion controller that owns motor feedback
     */
    LineFollower(LineSensor* line_sensor, Motion* motion);

    /**
     * @brief Resets controller state and PD internals
     */
    void reset();

    /**
     * @brief Start following the line
     */
    void startFollowing();

    /**
     * @brief Execute one control iteration
     * @param dt Time step in seconds
     */
    void update(float dt);

    /**
     * @brief Returns true on rising edge of intersection detection (with debounce)
     */
    bool isIntersectionDetected();

    /**
     * @brief Begin a 90-degree left turn in place
     */
    void turnLeft90();

    /**
     * @brief Begin a 90-degree right turn in place
     */
    void turnRight90();

    /**
     * @brief Stop all motors immediately
     */
    void stop();

    /**
     * @brief Returns true if current motion (turn) is complete
     */
    bool isMotionDone() const;

    /**
     * @brief Returns current state
     */
    State state() const;

    uint8_t     rawByte() const;
    uint8_t     activeMask() const;
    bool        linePresent() const;
    bool        lineLost() const;
    float       linePosition() const;
    float       lineError() const;
    float       filteredLineError() const;
    float       steeringAdjustmentDegps() const;
    float       targetSpeedMmps() const;
    const RuntimeTuning& runtimeTuning() const;
    void                 resetRuntimeTuning();
    bool                 setRuntimeTuningValue(const char* name, float value);
    bool        setRoute(const char* route);
    void        clearRoute();
    const char* route() const;
    uint8_t     routeIndex() const;
    bool        routeHasRemaining() const;
    uint8_t     currentPathsMask() const;
    bool        lastIntersectionValid() const;
    uint8_t     lastIntersectionPathsMask() const;
    uint8_t     lastIntersectionRawPeakMask() const;
    uint32_t    lastIntersectionElapsedMs() const;
    char        lastRouteCommand() const;
    bool        lastRouteChoiceMatched() const;

    // Run-statistics — accumulated from startFollowing() and frozen on stop().
    // Modeled after PATH's per-segment + summary diagnostics so a single
    // run can be debriefed end-to-end from the serial log.
    uint32_t runDurationMs() const;
    float    peakAbsErrorSlots() const;
    float    peakAbsSteeringDegps() const;
    float    peakVelocityMmps() const;
    float    minVelocityMmps() const;
    uint32_t intersectionCount() const;
    uint32_t recoveryCount() const;
    uint32_t saturationTicks() const; // ticks where |steer| == OMEGA_LIMIT

  private:
    enum class BranchDirection
    {
        None,
        Left,
        Right
    };

    void followLine(float dt);
    void updateTurn();
    void resetControlHistory();
    void evaluateIntersectionCommand(uint32_t now_ms);
    static RuntimeTuning defaultRuntimeTuning();

    LineSensor* line_sensor_;
    Motion*     motion_;

    State state_ = State::Idle;

    // Line steering diagnostics/state.
    float    prev_line_error_          = 0.0f;
    float    filtered_line_error_      = 0.0f;
    float    latest_line_position_     = 0.0f;
    float    latest_line_error_        = 0.0f;
    float    latest_steering_degps_    = 0.0f;
    float    latest_target_speed_mmps_ = 0.0f;
    bool     filter_initialized_       = false;
    bool     line_seen_                = false;
    bool     line_lost_                = false;
    uint32_t last_line_seen_ms_        = 0;
    RuntimeTuning tuning_;

    bool turn_done_ = true;

    // Intersection debounce
    bool     prev_intersection_    = false;
    uint32_t last_intersection_ms_ = 0;

    static constexpr uint8_t      kMaxRouteLength              = 64;
    char                          route_[kMaxRouteLength + 1]  = {};
    uint8_t                       route_length_                = 0;
    uint8_t                       route_index_                 = 0;
    BranchDirection               branch_direction_            = BranchDirection::None;
    uint32_t                      branch_capture_end_ms_       = 0;
    uint32_t                      intersection_lockout_end_ms_ = 0;
    bool                          recovery_active_             = false;
    LineSensor::IntersectionEvent last_intersection_event_;
    char                          last_route_command_        = '-';
    bool                          last_route_choice_matched_ = true;

    // Run statistics (accumulated while state == FollowingLine).
    uint32_t run_start_ms_        = 0;
    uint32_t run_end_ms_          = 0;
    float    peak_abs_error_      = 0.0f;
    float    peak_abs_steering_   = 0.0f;
    float    peak_velocity_mmps_  = 0.0f;
    float    min_velocity_mmps_   = 0.0f;
    bool     min_velocity_seeded_ = false;
    uint32_t intersection_count_  = 0;
    uint32_t recovery_count_      = 0;
    uint32_t saturation_ticks_    = 0;
};

#endif // CONTROL_LINE_FOLLOWER_H
