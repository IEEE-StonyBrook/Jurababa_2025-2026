#include "../../../Include/Platform/Pico/Robot/Profile.h"

Profile forward;
Profile rotation;

Profile::Profile()
    : m_state(PS_IDLE),
      m_speed(0),
      m_position(0),
      m_sign(1),
      m_acceleration(0),
      m_one_over_acc(1),
      m_target_speed(0),
      m_final_speed(0),
      m_final_position(0) {}

void Profile::reset() {
  m_position = 0;
  m_speed = 0;
  m_target_speed = 0;
  m_state = PS_IDLE;
}

bool Profile::is_finished() {
  return m_state == PS_FINISHED;
}

void Profile::start(float distance, float top_speed, float final_speed, float acceleration) {
  m_sign = (distance < 0) ? -1 : +1;
  if (distance < 0) distance = -distance;
  if (distance < 1.0f) {
    m_state = PS_FINISHED;
    return;
  }
  if (final_speed > top_speed) final_speed = top_speed;

  m_position = 0;
  m_final_position = distance;
  m_target_speed = m_sign * fabsf(top_speed);
  m_final_speed = m_sign * fabsf(final_speed);
  m_acceleration = fabsf(acceleration);
  m_one_over_acc = (m_acceleration >= 1.0f) ? (1.0f / m_acceleration) : 1.0f;
  m_state = PS_ACCELERATING;
}

void Profile::move(float distance, float top_speed, float final_speed, float acceleration) {
  start(distance, top_speed, final_speed, acceleration);
  wait_until_finished();
}

void Profile::stop() {
  m_target_speed = 0;
  finish();
}

void Profile::finish() {
  m_speed = m_target_speed;
  m_state = PS_FINISHED;
}

void Profile::wait_until_finished() {
  while (m_state != PS_FINISHED) {
    sleep_ms(2);
  }
}

void Profile::set_state(State state) {
  m_state = state;
}

float Profile::get_braking_distance() {
  return fabsf(m_speed * m_speed - m_final_speed * m_final_speed) * 0.5f * m_one_over_acc;
}

float Profile::position() {
  return m_position;
}

float Profile::speed() {
  return m_speed;
}

float Profile::acceleration() {
  return m_acceleration;
}

void Profile::set_speed(float speed) {
  m_speed = speed;
}

void Profile::set_target_speed(float speed) {
  m_target_speed = speed;
}

void Profile::adjust_position(float adjustment) {
  m_position += adjustment;
}

void Profile::set_position(float position) {
  m_position = position;
}

void Profile::update() {
  if (m_state == PS_IDLE) return;

  float delta_v = m_acceleration * LOOP_INTERVAL;
  float remaining = fabsf(m_final_position) - fabsf(m_position);

  if (m_state == PS_ACCELERATING) {
    if (remaining < get_braking_distance()) {
      m_state = PS_BRAKING;
      if (m_final_speed == 0) {
        // small velocity to ensure reaching zero (floating point safety)
        m_target_speed = m_sign * 5.0f;
      } else {
        m_target_speed = m_final_speed;
      }
    }
  }

  // Adjust speed toward target
  if (m_speed < m_target_speed) {
    m_speed += delta_v;
    if (m_speed > m_target_speed) m_speed = m_target_speed;
  } else if (m_speed > m_target_speed) {
    m_speed -= delta_v;
    if (m_speed < m_target_speed) m_speed = m_target_speed;
  }

  // Update position
  m_position += m_speed * LOOP_INTERVAL;

  // End condition
  if (m_state != PS_FINISHED && remaining < 0.125f) {
    m_state = PS_FINISHED;
    m_target_speed = m_final_speed;
  }
}