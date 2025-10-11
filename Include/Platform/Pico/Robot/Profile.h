#ifndef PROFILE_H
#define PROFILE_H

#include "pico/stdlib.h"
#include <cmath>
#include <cstdint>

// Equivalent to the loop update rate, e.g., 1 kHz = 0.001s
#ifndef LOOP_INTERVAL
#define LOOP_INTERVAL 0.001f
#endif

class Profile {
 public:
  enum State : uint8_t {
    PS_IDLE = 0,
    PS_ACCELERATING = 1,
    PS_BRAKING = 2,
    PS_FINISHED = 3,
  };

  Profile();

  void reset();
  bool is_finished();

  void start(float distance, float top_speed, float final_speed, float acceleration);
  void move(float distance, float top_speed, float final_speed, float acceleration);
  void stop();
  void finish();
  void wait_until_finished();
  void set_state(State state);

  float get_braking_distance();
  float position();
  float speed();
  float acceleration();

  void set_speed(float speed);
  void set_target_speed(float speed);
  void adjust_position(float adjustment);
  void set_position(float position);

  void update();

 private:
  uint8_t m_state;
  float m_speed;
  float m_position;
  int8_t m_sign;
  float m_acceleration;
  float m_one_over_acc;
  float m_target_speed;
  float m_final_speed;
  float m_final_position;
};

// Global instances
extern Profile forward;
extern Profile rotation;

#endif