#include "../../../Include/Platform/Pico/Robot/Motion.h"

Motion::Motion(Drivetrain* drivetrain) : drivetrain(drivetrain) {}

// Reset drivetrain and motion profiles to a known state.
void Motion::resetDriveSystem() {
  drivetrain->reset();
  forwardProfile.reset();
  rotationProfile.reset();
}

// Stop drivetrain and clear profiles.
void Motion::stop() {
  drivetrain->stop();
  forwardProfile.reset();
  rotationProfile.reset();
}

// Disable drive system (motors off).
void Motion::disableDrive() { drivetrain->stop(); }

// ---------------- Forward Motion ---------------- //

// Start a forward motion profile without blocking.
void Motion::startForward(float distanceMM, float topSpeed, float finalSpeed,
                          float accel) {
  forwardProfile.start(distanceMM, topSpeed, finalSpeed, accel);
}

// Run forward motion with option to block until finished.
void Motion::forward(float distanceMM, float topSpeed, float finalSpeed,
                     float accel, bool blocking) {
  LOG_DEBUG("Starting forward motion: " + std::to_string(distanceMM) + " mm");
  startForward(distanceMM, topSpeed, finalSpeed, accel);
  if (blocking) {
    while (!isForwardFinished()) {
      LOG_DEBUG("Pos: " + std::to_string(positionMM()) +
                " mm, Speed: " + std::to_string(velocityMMPerSec()) + " mm/s");
      update();
      sleep_ms(LOOP_INTERVAL_S * 1000);
    }
  }
}

// Check if forward motion has finished.
bool Motion::isForwardFinished() const { return forwardProfile.isFinished(); }

// ---------------- Rotational Motion ---------------- //

// Start a turn profile without blocking.
void Motion::startTurn(float angleDeg, float omega, float finalOmega,
                       float alpha) {
  rotationProfile.start(angleDeg, omega, finalOmega, alpha);
}

// Run turn motion with option to block until finished.
void Motion::turn(float angleDeg, float omega, float finalOmega, float alpha,
                  bool blocking) {
  startTurn(angleDeg, omega, finalOmega, alpha);
  if (blocking) {
    while (!isTurnFinished()) {
      update();
      sleep_ms(LOOP_INTERVAL_S * 1000);
    }
  }
}

// Check if turn motion has finished.
bool Motion::isTurnFinished() const { return rotationProfile.isFinished(); }

// ---------------- Integrated Turns ---------------- //

// Perform a smooth integrated turn (used with forward motion).
void Motion::integratedTurn(float angleDeg, float omega, float alpha) {
  rotationProfile.reset();
  rotationProfile.move(angleDeg, omega, 0, alpha);
}

// Perform an in-place spin turn (forces forward velocity to zero).
void Motion::spinTurn(float angleDeg, float omega, float alpha) {
  drivetrain->runControl(0.0f, 0.0f, 0.0f);
  integratedTurn(angleDeg, omega, alpha);
}

// ---------------- Stopping Utilities ---------------- //

// Stop at a specific target position.
void Motion::stopAt(float targetMM) {
  float remaining = targetMM - positionMM();
  forwardProfile.move(remaining, velocityMMPerSec(), 0,
                      accelerationMMPerSec2());
}

// Stop after moving a given distance.
void Motion::stopAfter(float distanceMM) {
  forwardProfile.move(distanceMM, velocityMMPerSec(), 0,
                      accelerationMMPerSec2());
}

// Wait until a target position is reached.
void Motion::waitUntilPosition(float targetMM) {
  while (positionMM() < targetMM) {
    sleep_ms(2);
  }
}

// Wait until a relative distance has been travelled.
void Motion::waitUntilDistance(float deltaMM) {
  waitUntilPosition(positionMM() + deltaMM);
}

// ---------------- Update Loop ---------------- //

// Update motion profiles and send control signals to drivetrain.
void Motion::update() {
  forwardProfile.update();
  rotationProfile.update();
  drivetrain->runControl(forwardProfile.speed(), rotationProfile.speed(), 0.0f);
}