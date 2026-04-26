#ifndef DRIVER_LAB_SETTINGS_H
#define DRIVER_LAB_SETTINGS_H

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
struct DriverLabSettings
{
    float kM; // Motor velocity constant (mm/s per volt) - combined average
    float tm; // Motor mechanical time constant (seconds)

    float kS; // Static friction compensation (volts) - combined average
    float kV; // Speed feedforward (volts per mm/s) = 1/kM
    float kA; // Acceleration feedforward (volts per mm/s^2) = Tm/kM

    // Per-motor characterization (from stereo OL trial)
    float kM_L; // Left motor velocity constant (mm/s per volt)
    float kM_R; // Right motor velocity constant (mm/s per volt)
    float kS_L; // Left static friction (volts)
    float kS_R; // Right static friction (volts)

    float zeta; // Damping ratio
    float td;   // Derivative time constant
    float kP;   // Proportional gain (forward)
    float kD;   // Derivative gain (forward)

    // Rotation PD control
    float turnKP; // Rotation proportional gain
    float turnKD; // Rotation derivative gain

    uint8_t control_flags;

    void initDefaults()
    {
        // Motor model from tuning.h (measured via OL and STEP trials)
        kM = MOTOR_KM;
        tm = MOTOR_TM;

        // Feedforward: derived from motor model, with per-motor kS from OL
        kV = 1.0f / kM;
        kS = (FORWARD_KSL + FORWARD_KSR) / 2.0f;
        kA = tm / kM;

        // Per-motor defaults: same as combined until OL trial separates them
        kM_L = kM;
        kM_R = kM;
        kS_L = kS;
        kS_R = kS;

        // Forward PD: design parameters and gains from tuning.h
        zeta = FWD_ZETA;
        td   = FWD_TD;
        kP   = FWD_KP;
        kD   = FWD_KD;

        // Rotation PD
        turnKP = ROT_KP;
        turnKD = ROT_KD;

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
