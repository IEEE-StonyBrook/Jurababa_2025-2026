// Robot.h
#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <string>

class Drivetrain;
class Sensors;
class PIDController;

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
    void turn45Degrees(std::string side);

    // Call every control tick (e.g. 5ms or 10ms) to run PIDs and update motor outputs
    void update(float dt);

    // Returns true when the current motion goal is satisfied
    bool isMotionDone() const;

    // Allows tuning without changing code
    void setSpeedGains(float kp, float ki, float kd);
    void setYawGains(float kp, float ki, float kd);

    // Output clamps for motor commands (Duty expected in [-1, 1])
    void setMaxDuty(float maxDuty);

    // Limits how fast duty can change per second (Slew rate limiting)
    void setMaxDutySlewPerSec(float maxDutyChangePerSec);

    // Tolerance for considering a turn complete
    void setYawToleranceDeg(float tolDeg);

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

    // Clamps to [-maxAbs, +maxAbs]
    static float clampAbs(float value, float maxAbs);

    // Snaps an angle to the nearest multiple of 45 degrees (Result in [-180, 180])
    static float snapToNearest45Deg(float yawDeg);

    // Applies slew rate limit to commanded duty
    float applySlew(float cmd, float& prevCmd, float dt);

  private:
    Drivetrain* drivetrain;
    Sensors*    sensors;

    PIDController speedPID; // Controls forward speed (Based on average wheel speed)
    PIDController yawPID;   // Controls heading/turning (Based on IMU yaw)

    Mode mode = Mode::Idle;

    // Current motion targets
    float targetForwardMMps = 0.0f;
    float targetYawDeg      = 0.0f;

    // DriveDistance state
    float driveStartDistMM  = 0.0f;
    float driveTargetDistMM = 0.0f;

    // Completion criteria
    float yawToleranceDeg = 2.0f;

    // Motor command clamp (Duty expected to be [-1, 1])
    float maxDuty = 1.0f;

    // Slew rate limit (Duty units per second)
    float maxDutySlewPerSec = 10.0f;

    // Previous output commands for slew limiting
    float prevLeftDuty  = 0.0f;
    float prevRightDuty = 0.0f;

    bool motionDone = true;
};

#endif