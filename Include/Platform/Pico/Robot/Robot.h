#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <string>
#include "../../../Common/PIDController.h"

class Drivetrain;
class Sensors;

class Robot
{
  public:
    // Construct robot "Brain" using drivetrain + sensors (Hardware wrappers)
    Robot(Drivetrain* drivetrain, Sensors* sensors);

    // Reset drivetrain/sensors and clear controller state (Call when switching modes)
    void reset();

    // Immediately stop motors and enter Idle mode
    void stop();

    // Drive forward at a target speed while holding a target yaw
    // forwardMMps is millimeters/second, holdYawDeg is degrees [-180, 180]
    void driveStraightMMps(float forwardMMps, float holdYawDeg);

    // Drive a fixed distance while holding the nearest 45-degree heading
    // distanceMM is always positive; forwardMMps sets direction by sign
    void driveDistanceMM(float distanceMM, float forwardMMps);

    // Turn in place to a target yaw using IMU yaw feedback
    // targetYawDeg is degrees [-180, 180]
    void turnToYawDeg(float targetYawDeg);

    // Turn to the nearest 45-degree heading after applying a +/-45 step
    // side must be "left" or "right"
    void turn45Degrees(std::string side, int times);

    // Call every control tick (e.g. 5ms or 10ms) to run controllers and update motor outputs
    void update(float dt);

    // Returns true when the current motion goal is satisfied
    bool isMotionDone() const;

    // Tuning
    void setYawGains(float kp, float ki, float kd);
    void setWheelGains(float kp, float ki, float kd);

    // Limits
    void setMaxDuty(float maxDuty);
    void setMaxDutySlewPerSec(float maxDutyChangePerSec);
    void setMaxWheelSpeedMMps(float maxWheelSpeedMMps);
    void setMaxYawDiffMMps(float maxYawDiffMMps);

    // Completion criteria
    void setYawToleranceDeg(float tolDeg);

    // DriveDistance shaping
    void setSlowdownDistMM(float slowdownDistMM);
    void setMinSlowdownScale(float minScale);

  private:
    enum class Mode
    {
        Idle,
        DriveStraight,
        DriveDistance,
        TurnInPlace
    };

    // Computes shortest angle difference in degrees (Result in [-180, 180])
    static float wrapDeg(float deg);

    // Snaps an angle to the nearest multiple of 45 degrees (Result in [-180, 180])
    static float snapToNearest45Deg(float yawDeg);

    // Clamps to [-maxAbs, +maxAbs]
    static float clampAbs(float value, float maxAbs);

    // Applies slew rate limit to commanded duty
    float applySlew(float cmd, float& prevCmd, float dt);

    // Sets wheel velocity targets (mm/s)
    void setWheelVelocityTargetsMMps(float vLeftMMps, float vRightMMps);

    // Ramps a value toward a desired target by maxDelta
    float rampToward(float desired, float current, float maxDelta);

    // Converts base speed + yaw correction into wheel targets and runs low-level wheel control
    void commandBaseAndYaw(float vBaseMMps, float dt);

    // Runs two wheel PIDs + feedforward and outputs duty to drivetrain
    void runWheelVelocityControl(float dt);

  private:
    Drivetrain* drivetrain;
    Sensors*    sensors;

    // High-level yaw controller: Output is wheel speed differential (mm/s)
    PIDController yawPID;

    // Low-level wheel speed controllers: Output is duty correction
    PIDController leftWheelPID;
    PIDController rightWheelPID;

    Mode mode = Mode::Idle;
    Mode prevMode = Mode::Idle;

    // Motion targets
    float targetForwardMMps = 0.0f;
    float targetYawDeg      = 0.0f;

    // Wheel targets (consumed by runWheelVelocityControl)
    float targetLeftMMps  = 0.0f;
    float targetRightMMps = 0.0f;

    // DriveDistance state (Per wheel for robustness)
    float driveStartLeftDistMM  = 0.0f;
    float driveStartRightDistMM = 0.0f;
    float driveTargetDistMM     = 0.0f;

    // Completion criteria
    float yawToleranceDeg = 1.0f;

    // Limits
    float maxDuty           = 1.0f;
    float maxDutySlewPerSec = 10.0f;
    float maxWheelSpeedMMps = 800.0f;
    float maxYawDiffMMps    = 400.0f; // yawPID output limit in mm/s differential

    // Slew state
    float prevLeftDuty  = 0.0f;
    float prevRightDuty = 0.0f;

    // DriveDistance shaping
    float slowdownDistMM   = 80.0f;
    float minSlowdownScale = 0.2f;

    // Ramping state
    float vBaseCmdMMps      = 0.0f;    // ramped command actually used
    float maxBaseAccelMMps2 = 600.0f; // tune: 1500–5000

    bool motionDone = true;
};

#endif