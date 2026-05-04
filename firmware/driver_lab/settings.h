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
    float kA_L; // Left  acceleration FF (V per mm/s^2) = Tm / kM_L
    float kA_R; // Right acceleration FF (V per mm/s^2) = Tm / kM_R

    float zeta; // Damping ratio
    float td;   // Derivative time constant
    float kP;   // Proportional gain (forward)
    float kD;   // Derivative gain (forward)

    // Rotation plant model (from TURN-OL + TURN-STEP)
    float rot_kM; // Rotational velocity constant (deg/s per volt of differential drive)
    float rot_tm; // Rotational time constant (seconds)

    // Rotation PD control (derived from rot_kM, rot_tm via the same 2nd-order
    // formula as the forward loop)
    float rot_zeta; // Rotation damping ratio
    float rot_td;   // Rotation derivative time constant
    float turnKP;   // Rotation proportional gain
    float turnKD;   // Rotation derivative gain

    uint8_t control_flags;

    void initDefaults()
    {
        // Motor model from tuning.h (measured via OL and STEP trials)
        kM = MOTOR_KM;
        tm = MOTOR_TM;

        // Per-motor model from tuning.h (from stereo OL trial).
        // kM = 1 / kV by definition; reuse the per-motor FORWARD_KV* macros so
        // L/R asymmetry from calibration survives into runtime defaults.
        kM_L = 1.0f / FORWARD_KVL;
        kM_R = 1.0f / FORWARD_KVR;
        kS_L = FORWARD_KSL;
        kS_R = FORWARD_KSR;
        kA_L = FORWARD_KAL;
        kA_R = FORWARD_KAR;

        // Combined feedforward (used in single-motor codepaths)
        kV = 1.0f / kM;
        kS = (FORWARD_KSL + FORWARD_KSR) / 2.0f;
        kA = tm / kM;

        // Forward PD: design parameters and gains from tuning.h
        zeta = FWD_ZETA;
        td   = FWD_TD;
        kP   = FWD_KP;
        kD   = FWD_KD;

        // Rotation plant from tuning.h (TODO: re-measure via TURN-OL + TURN-STEP)
        rot_kM   = ROT_KM;
        rot_tm   = ROT_TM;
        rot_zeta = ROT_ZETA;
        rot_td   = ROT_TD;

        // Rotation PD
        turnKP = ROT_KP;
        turnKD = ROT_KD;

        control_flags = 0;
    }

    // Recompute feedforward (kV, kA) and per-motor kA_L/kA_R from the motor
    // model (kM, kM_L, kM_R, tm). Only call when those source values change,
    // since the feedforward terms can also be manually overridden.
    void recalculateFeedforward()
    {
        kV   = 1.0f / kM;
        kA   = tm / kM;
        kA_L = tm / kM_L;
        kA_R = tm / kM_R;
    }

    // Recompute PD gains (kP, kD) using the motorlab / mazerunner-core
    // convention (omega_n = 4/td, td as the closed-loop "rise time" anchor).
    // MUST match the FWD_KP / FWD_KD macros in config/tuning.h or a STEP trial
    // will silently slam runtime gains far below the boot-time gains.
    //   kP = 16 * Tm / (kM * zeta^2 * td^2)
    //   kD = (8 * Tm - td) / (kM * td)
    // kD stays positive whenever td < 8*Tm (always true with td = Tm/2).
    void recalculatePD()
    {
        kP = 16.0f * tm / (kM * zeta * zeta * td * td);
        kD = (8.0f * tm - td) / (kM * td);
    }

    // Recompute rotation PD (turnKP, turnKD) using the same motorlab form
    // applied to the rotational plant (rot_kM, rot_tm). MUST match the
    // ROT_KP / ROT_KD macros in config/tuning.h. Call after TURN-OL /
    // TURN-STEP updates rot_kM / rot_tm.
    void recalculateRotation()
    {
        turnKP = 16.0f * rot_tm / (rot_kM * rot_zeta * rot_zeta * rot_td * rot_td);
        turnKD = (8.0f * rot_tm - rot_td) / (rot_kM * rot_td);
    }

    // Recompute everything. Use only when motor model parameters change.
    void recalculateDerived()
    {
        recalculateFeedforward();
        recalculatePD();
    }

    void print() const;
};

constexpr uint8_t CONTROL_FLAG_USE_FEEDFORWARD = 0x01;
constexpr uint8_t CONTROL_FLAG_USE_CONTROLLER  = 0x02;
constexpr uint8_t CONTROL_FLAG_FULL_CONTROL    = 0x03;

// OL steering correction: gentle heading-hold during forward voltage sweeps
// so the robot tracks straight without bumping walls. Symmetric trim
// (left_v = base + trim, right_v = base - trim) on the rotation plant. The
// regression uses the actual per-side voltage logged each tick, so any
// persistent trim bias is captured truthfully and does not skew kM/kS.
//
// Bandwidth deliberately slow (omega_n = 3 rad/s, zeta = 1.0). The original
// design at omega_n = 5 rad/s caused a ~5 deg limit cycle on the bench:
// rot_tm = 0.106 s puts omega_n*rot_tm ~ 0.53, which adds ~28 deg of phase
// erosion right at crossover, on top of IMU 100 Hz ZOH delay and gearbox
// stiction/backlash that the linear PD design doesn't see. Dropping to
// 3 rad/s pushes crossover well below the rotation lag pole (omega*tau ~
// 0.32) and cuts both Kp and Kd, killing the oscillation with margin.
//
// Gains derived at use site from settings_.rot_kM, mirroring
// recalculateRotation() above:
//   kp_vpdeg = omega_n^2          / rot_kM
//   kd_vpdps = 2 * zeta * omega_n / rot_kM
constexpr float OL_STEERING_OMEGA_N_RAD = 3.0f; // closed-loop bandwidth, rad/s
constexpr float OL_STEERING_ZETA        = 1.0f; // damping ratio (overdamped feel)
constexpr float OL_STEERING_TRIM_MAX_V  = 0.5f; // |trim| clamp, volts

#endif
