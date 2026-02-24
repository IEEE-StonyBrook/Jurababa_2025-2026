#ifndef MOTOR_LAB_SETTINGS_H
#define MOTOR_LAB_SETTINGS_H

#include <cstdint>
#include "config/config.h"

/**
 * @brief Motor characterization and control parameters
 *
 * Motor Model: V = (1/Km) * speed + bias
 *   - Km: Motor velocity constant (mm/s per volt)
 *   - Tm: Motor time constant (mechanical lag)
 *   - bias: Static friction voltage
 *
 * Controller: PD with second-order design
 *   - zeta: Damping ratio (0.707 = critically damped)
 *   - Td: Derivative time constant
 *   - Kp, Kd: Derived PD gains
 */
struct MotorLabSettings
{
    float km; // Motor velocity constant (mm/s per volt)
    float tm; // Motor mechanical time constant (seconds)

    float bias_ff;  // Static friction compensation (volts)
    float speed_ff; // Speed feedforward (volts per mm/s) = 1/Km
    float acc_ff;   // Acceleration feedforward (volts per mm/s^2) = Tm/Km

    float zeta; // Damping ratio
    float td;   // Derivative time constant
    float kp;   // Proportional gain
    float kd;   // Derivative gain

    uint8_t control_flags;

    void initDefaults()
    {
        // Load feedforward from config/tuning.h (already in Voltage units)
        speed_ff = (FORWARD_KVL + FORWARD_KVR) / 2.0f;
        bias_ff  = (FORWARD_KSL + FORWARD_KSR) / 2.0f;
        acc_ff   = (FORWARD_KAL + FORWARD_KAR) / 2.0f;

        // Derive motor model from feedforward
        km = (speed_ff > 1e-6f) ? (1.0f / speed_ff) : 717.0f;
        tm = acc_ff * km; // Tm = acc_ff / speed_ff

        // Load PID from config/tuning.h (already in Voltage units)
        kp = FWD_KP;
        kd = FWD_KD;

        // Back-calculate zeta/td for display consistency
        td   = (km > 1e-6f && kp > 1e-6f) ? (1.0f / (km * kp)) : 0.1f;
        zeta = (td > 1e-6f) ? ((kd / km + tm) / (2.0f * td)) : 0.707f;

        control_flags = 0;
    }

    void recalculateDerived()
    {
        speed_ff = 1.0f / km;
        acc_ff   = tm / km;
        kp       = 1.0f / (km * td);
        kd       = (2.0f * zeta * td - tm) / km;
    }

    void print() const;
};

constexpr uint8_t CONTROL_FLAG_USE_FEEDFORWARD = 0x01;
constexpr uint8_t CONTROL_FLAG_USE_CONTROLLER  = 0x02;
constexpr uint8_t CONTROL_FLAG_FULL_CONTROL    = 0x03;

#endif
