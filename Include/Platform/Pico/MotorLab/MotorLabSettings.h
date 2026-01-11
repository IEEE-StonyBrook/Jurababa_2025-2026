#ifndef MOTORLAB_SETTINGS_H
#define MOTORLAB_SETTINGS_H

#include <cstdint>

/**
 * @file MotorLabSettings.h
 * @brief Motor calibration settings storage for UKMARS-style motor lab
 *
 * Stores tunable parameters for motor characterization and control tuning.
 * Parameters can be modified at runtime via CLI and persisted to flash.
 *
 * Based on UKMARS motorlab by Peter Harrison.
 */

/**
 * @brief Motor characterization and control parameters
 *
 * These parameters define the motor model and controller gains used
 * for feedforward and feedback control.
 *
 * Motor Model: V = Km * omega + bias
 *   - Km: Motor velocity constant (converts speed to voltage)
 *   - Tm: Motor time constant (mechanical lag)
 *   - bias: Static friction voltage
 *
 * Controller: PD with second-order design
 *   - zeta: Damping ratio (0.707 = critically damped)
 *   - Td: Derivative time constant
 *   - Kp, Kd: Derived PD gains
 */
struct MotorLabSettings {
    // Motor model constants
    float km;       // Motor velocity constant (deg/s per volt)
    float tm;       // Motor mechanical time constant (seconds)

    // Feedforward parameters
    float bias_ff;  // Static friction compensation (volts)
    float speed_ff; // Speed feedforward (volts per deg/s) = 1/Km
    float acc_ff;   // Acceleration feedforward (volts per deg/s²) = Tm/Km

    // Controller tuning parameters
    float zeta;     // Damping ratio (typically 0.707 for critical damping)
    float td;       // Derivative time constant (typically Tm/2)
    float kp;       // Proportional gain
    float kd;       // Derivative gain

    // Control flags
    uint8_t control_flags;  // Bit flags for control options

    // Encoder calibration
    float deg_per_count;    // Degrees per encoder count

    /**
     * @brief Initialize with default values
     */
    void initDefaults() {
        // Default motor model (tune these for your specific motors)
        km = 2064.7f;           // deg/s per volt (from UKMARS defaults)
        tm = 0.325f;            // seconds

        // Derived feedforward
        speed_ff = 1.0f / km;   // volts per deg/s
        acc_ff = tm / km;       // volts per deg/s²
        bias_ff = 0.145f;       // volts (static friction)

        // Controller design
        zeta = 0.707f;          // Critically damped
        td = tm / 2.0f;         // Half of time constant

        // Derived PD gains (using second-order design formulas)
        // Kp = 1 / (Km * Td)
        // Kd = (2 * zeta * Td - Tm) / Km
        kp = 1.0f / (km * td);
        kd = (2.0f * zeta * td - tm) / km;

        // Control flags
        control_flags = 0;

        // Encoder (tune for your encoder resolution)
        deg_per_count = 360.0f / 359.7222f;  // From Config.h TICKS_PER_REVOLUTION
    }

    /**
     * @brief Recalculate derived parameters after changing base values
     *
     * Call this after modifying km, tm, zeta, or td to update
     * the feedforward and controller gains.
     */
    void recalculateDerived() {
        // Update feedforward from motor model
        speed_ff = 1.0f / km;
        acc_ff = tm / km;

        // Update PD gains from design parameters
        kp = 1.0f / (km * td);
        kd = (2.0f * zeta * td - tm) / km;
    }

    /**
     * @brief Print settings to serial
     */
    void print() const;

    // TODO: Add Flash persistence for settings (UKMARS motorlab uses EEPROM)
};

// Control flag bit definitions
constexpr uint8_t CONTROL_FLAG_USE_FEEDFORWARD  = 0x01;  // Enable feedforward
constexpr uint8_t CONTROL_FLAG_USE_CONTROLLER   = 0x02;  // Enable PD controller
constexpr uint8_t CONTROL_FLAG_FULL_CONTROL     = 0x03;  // Both FF + PD

#endif  // MOTORLAB_SETTINGS_H
