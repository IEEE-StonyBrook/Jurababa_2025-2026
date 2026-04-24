#ifndef MOTOR_LAB_SETTINGS_H
#define MOTOR_LAB_SETTINGS_H

#include "config/config.h"
#include <cstdint>

/**
 * @brief Motor characterization and control parameters
 *
 * Motor Model: V = kV * speed + kS
 *   - kM: Motor velocity constant (mm/s per volt) - slope of speed vs voltage
 *   - Tm: Motor time constant (mechanical lag)
 *   - kS: Static friction voltage (x-intercept of speed vs voltage line)
 *
 * Feedforward gains (voltage units):
 *   - kV: Speed feedforward (volts per mm/s) = 1/kM
 *   - kA: Acceleration feedforward (volts per mm/s^2) = Tm/kM
 *
 * Controller: PD with second-order design
 *   - zeta: Damping ratio (0.707 = critically damped)
 *   - Td: Derivative time constant
 *   - kP, kD: Derived PD gains
 */
struct MotorLabSettings
{
    float kM; // Motor velocity constant (mm/s per volt)
    float tm; // Motor mechanical time constant (seconds)

    float kS; // Static friction compensation (volts)
    float kV; // Speed feedforward (volts per mm/s) = 1/kM
    float kA; // Acceleration feedforward (volts per mm/s^2) = Tm/kM

    float zeta; // Damping ratio
    float td;   // Derivative time constant
    float kP;   // Proportional gain
    float kD;   // Derivative gain

    uint8_t control_flags;

    void initDefaults()
    {
        // Load feedforward from config/tuning.h (already in Voltage units)
        kV = (FORWARD_KVL + FORWARD_KVR) / 2.0f;
        kS = (FORWARD_KSL + FORWARD_KSR) / 2.0f;
        kA = (FORWARD_KAL + FORWARD_KAR) / 2.0f;

        // Derive motor model from feedforward
        kM = (kV > 1e-6f) ? (1.0f / kV) : 717.0f;
        tm = kA * kM; // Tm = kA / kV

        // Load PID from config/tuning.h (already in Voltage units)
        kP = FWD_KP;
        kD = FWD_KD;

        // Back-calculate zeta/td for display consistency
        td   = (kM > 1e-6f && kP > 1e-6f) ? (1.0f / (kM * kP)) : 0.1f;
        zeta = (td > 1e-6f) ? ((kD / kM + tm) / (2.0f * td)) : 0.707f;

        control_flags = 0;
    }

    void recalculateDerived()
    {
        kV = 1.0f / kM;
        kA = tm / kM;
        kP = 1.0f / (kM * td);
        kD = (2.0f * zeta * td - tm) / kM;
    }

    void print() const;
};

constexpr uint8_t CONTROL_FLAG_USE_FEEDFORWARD = 0x01;
constexpr uint8_t CONTROL_FLAG_USE_CONTROLLER  = 0x02;
constexpr uint8_t CONTROL_FLAG_FULL_CONTROL    = 0x03;

#endif
