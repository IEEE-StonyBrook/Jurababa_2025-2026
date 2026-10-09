/**
 * @file smooth_turn_geometry.h
 * @brief Pure geometry of one smooth (arc) turn. No state, no hardware.
 *
 * A smooth turn spans exactly ONE CELL of forward progress: it starts
 * HALF_CELL_MM before the turn cell's centre and ends HALF_CELL_MM after it,
 * facing the new direction.
 *
 *         turn cell centre
 *               |
 *     --[entry]-+-----        entry straight: HALF_CELL_MM - radius
 *               :\            arc:            radius * angle (radians)
 *               : \  arc      exit straight:  HALF_CELL_MM - radius
 *               :  \
 *               :   |
 *               :  [exit]
 *               :   |
 *
 * A 90 deg arc tangent to both cell axes starts `radius` before the cell
 * centre and ends `radius` after it, so each straight is HALF_CELL_MM - radius.
 * Entry and exit are therefore DERIVED from speed and omega rather than
 * guessed; the two offsets are small human corrections on top (default 0) for
 * IMU lag and wheel slip.
 *
 * Tuning note: radius shrinks as omega rises at fixed speed. The turn only
 * fits inside a cell while radius <= HALF_CELL_MM, which `valid` reports.
 */
#ifndef CONTROL_SMOOTH_TURN_GEOMETRY_H
#define CONTROL_SMOOTH_TURN_GEOMETRY_H

#include "config/geometry.h"

struct SmoothTurnGeometry
{
    float angle_deg = 0.0f; // Signed: + turns left, - turns right.
    float radius_mm = 0.0f;
    float entry_mm  = 0.0f;
    float arc_mm    = 0.0f;
    float exit_mm   = 0.0f;
    bool  valid     = false;

    constexpr float total_mm() const { return entry_mm + arc_mm + exit_mm; }
};

/**
 * @brief Resolve a smooth turn's phase lengths.
 * @param speed_mmps       Constant forward speed through the whole turn.
 * @param omega_degps      Cruise yaw rate during the arc. Sets the radius.
 * @param angle_deg        Signed turn angle. Only the magnitude affects length.
 * @param entry_offset_mm  Correction added to the derived entry straight.
 * @param exit_offset_mm   Correction added to the derived exit straight.
 */
constexpr SmoothTurnGeometry smoothTurnGeometry(float speed_mmps, float omega_degps,
                                                float angle_deg, float entry_offset_mm,
                                                float exit_offset_mm)
{
    constexpr float kPi = 3.14159265f;

    SmoothTurnGeometry geo;
    if (speed_mmps <= 0.0f || omega_degps <= 0.0f || angle_deg == 0.0f)
        return geo;

    const float angle_mag_deg = (angle_deg < 0.0f) ? -angle_deg : angle_deg;

    geo.angle_deg = angle_deg;
    geo.radius_mm = (speed_mmps / omega_degps) * (180.0f / kPi);
    geo.arc_mm    = angle_mag_deg * (kPi / 180.0f) * geo.radius_mm;
    geo.entry_mm  = HALF_CELL_MM - geo.radius_mm + entry_offset_mm;
    geo.exit_mm   = HALF_CELL_MM - geo.radius_mm + exit_offset_mm;
    geo.valid     = geo.radius_mm <= HALF_CELL_MM && geo.entry_mm >= 0.0f && geo.exit_mm >= 0.0f;
    return geo;
}

// Compile-time self-check against the shipped defaults (240 mm/s, 240 deg/s):
// radius 57.30, entry/exit 32.70, arc 90.00. Fails the build if the math or
// the cell size drifts.
namespace smooth_turn_geometry_check
{
constexpr SmoothTurnGeometry kDefault = smoothTurnGeometry(240.0f, 240.0f, 90.0f, 0.0f, 0.0f);

constexpr bool near(float a, float b)
{
    return (a - b) < 0.01f && (b - a) < 0.01f;
}

static_assert(kDefault.valid, "Default smooth turn must fit inside one cell");
static_assert(near(kDefault.radius_mm, 57.2958f), "Default smooth turn radius");
static_assert(near(kDefault.arc_mm, 90.0f), "Default smooth turn arc length");
static_assert(near(kDefault.entry_mm, 32.7042f), "Default smooth turn entry straight");
static_assert(near(kDefault.entry_mm, kDefault.exit_mm),
              "Entry and exit are symmetric at 0 offset");

// A radius larger than a half cell cannot be driven inside the maze.
static_assert(!smoothTurnGeometry(240.0f, 100.0f, 90.0f, 0.0f, 0.0f).valid,
              "Oversized radius must be rejected");
} // namespace smooth_turn_geometry_check

#endif // CONTROL_SMOOTH_TURN_GEOMETRY_H
