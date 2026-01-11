#include "Common/MotionProfile.h"
#include <cmath>

// Default tolerances
static constexpr float DEFAULT_POSITION_TOLERANCE = 2.0f;   // mm or deg
static constexpr float DEFAULT_VELOCITY_TOLERANCE = 30.0f;  // mm/s or deg/s

MotionProfile::MotionProfile()
    : state_(State::Idle),
      targetDistance_(0.0f),
      maxVelocity_(0.0f),
      acceleration_(0.0f),
      initialPosition_(0.0f),
      currentVelocity_(0.0f),
      currentAcceleration_(0.0f),
      accelDistance_(0.0f),
      decelDistance_(0.0f),
      cruiseDistance_(0.0f),
      cruiseVelocity_(0.0f),
      positionTolerance_(DEFAULT_POSITION_TOLERANCE),
      velocityTolerance_(DEFAULT_VELOCITY_TOLERANCE)
{
}

void MotionProfile::start(float targetDistance, float maxVelocity, float acceleration,
                          float initialPosition)
{
    // Store parameters
    targetDistance_  = std::fabs(targetDistance);  // Work with absolute distance
    maxVelocity_     = std::fabs(maxVelocity);
    acceleration_    = std::fabs(acceleration);
    initialPosition_ = initialPosition;

    // Reset runtime state
    currentVelocity_     = 0.0f;
    currentAcceleration_ = 0.0f;

    // Calculate profile parameters
    calculateProfileParameters();

    // Start in accelerating state (unless zero distance)
    if (targetDistance_ > positionTolerance_)
    {
        state_               = State::Accelerating;
        currentAcceleration_ = acceleration_;
    }
    else
    {
        state_ = State::Finished;
    }
}

void MotionProfile::calculateProfileParameters()
{
    // Calculate distance needed to reach max velocity from rest
    // v² = v₀² + 2as  →  s = v²/(2a)
    float distToMaxVel = (maxVelocity_ * maxVelocity_) / (2.0f * acceleration_);

    // Check if we can reach max velocity (triangular vs trapezoidal profile)
    if (2.0f * distToMaxVel <= targetDistance_)
    {
        // Trapezoidal profile - we reach max velocity
        accelDistance_ = distToMaxVel;
        decelDistance_ = distToMaxVel;
        cruiseDistance_ = targetDistance_ - (accelDistance_ + decelDistance_);
        cruiseVelocity_ = maxVelocity_;
    }
    else
    {
        // Triangular profile - never reach max velocity
        // Total distance split equally between accel and decel
        accelDistance_  = targetDistance_ / 2.0f;
        decelDistance_  = targetDistance_ / 2.0f;
        cruiseDistance_ = 0.0f;

        // Calculate actual peak velocity
        // s = v²/(2a)  →  v = √(2as)
        cruiseVelocity_ = std::sqrt(2.0f * acceleration_ * accelDistance_);
    }
}

void MotionProfile::update(float currentPosition, float dt)
{
    if (state_ == State::Idle || state_ == State::Finished)
        return;

    // Calculate distance traveled from initial position
    float distanceTraveled = std::fabs(currentPosition - initialPosition_);

    // Update state based on distance
    updateState(distanceTraveled);

    // Update velocity based on state
    switch (state_)
    {
        case State::Accelerating:
            // Increase velocity
            currentVelocity_ += acceleration_ * dt;
            if (currentVelocity_ >= cruiseVelocity_)
            {
                currentVelocity_ = cruiseVelocity_;
            }
            currentAcceleration_ = acceleration_;
            break;

        case State::Cruising:
            // Maintain constant velocity
            currentVelocity_     = cruiseVelocity_;
            currentAcceleration_ = 0.0f;
            break;

        case State::Decelerating:
            // Decrease velocity
            currentVelocity_ -= acceleration_ * dt;
            if (currentVelocity_ <= 0.0f)
            {
                currentVelocity_ = 0.0f;
            }
            currentAcceleration_ = -acceleration_;
            break;

        case State::Finished:
            currentVelocity_     = 0.0f;
            currentAcceleration_ = 0.0f;
            break;

        case State::Idle:
            // Do nothing
            break;
    }
}

void MotionProfile::updateState(float distanceTraveled)
{
    // Check if motion is complete
    float remaining = targetDistance_ - distanceTraveled;

    if (remaining <= positionTolerance_ && currentVelocity_ <= velocityTolerance_)
    {
        state_ = State::Finished;
        return;
    }

    // Determine state based on distance traveled
    if (distanceTraveled < accelDistance_)
    {
        // Still accelerating
        state_ = State::Accelerating;
    }
    else if (distanceTraveled < (accelDistance_ + cruiseDistance_))
    {
        // In cruise phase
        state_ = State::Cruising;
    }
    else
    {
        // In deceleration phase
        state_ = State::Decelerating;
    }

    // Additional check: if remaining distance requires deceleration, switch to decel
    // This handles cases where we need to start braking early
    float brakingDistance = (currentVelocity_ * currentVelocity_) / (2.0f * acceleration_);

    if (remaining <= brakingDistance && state_ != State::Finished)
    {
        state_ = State::Decelerating;
    }
}

float MotionProfile::getTargetVelocity() const
{
    return currentVelocity_;
}

float MotionProfile::getTargetAcceleration() const
{
    return currentAcceleration_;
}

float MotionProfile::getRemainingDistance() const
{
    if (state_ == State::Idle || state_ == State::Finished)
        return 0.0f;

    // This is an approximation - could be improved with position tracking
    return targetDistance_ * (currentVelocity_ / cruiseVelocity_);
}

bool MotionProfile::isFinished() const
{
    return (state_ == State::Finished);
}

MotionProfile::State MotionProfile::getState() const
{
    return state_;
}

void MotionProfile::reset()
{
    state_               = State::Idle;
    targetDistance_      = 0.0f;
    maxVelocity_         = 0.0f;
    acceleration_        = 0.0f;
    initialPosition_     = 0.0f;
    currentVelocity_     = 0.0f;
    currentAcceleration_ = 0.0f;
    accelDistance_       = 0.0f;
    decelDistance_       = 0.0f;
    cruiseDistance_      = 0.0f;
    cruiseVelocity_      = 0.0f;
}
