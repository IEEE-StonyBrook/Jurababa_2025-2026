/**
 * @file geometry.h
 * @brief Robot physical dimensions and derived constants
 *
 * All measurements in millimeters. Change these values when
 * switching wheels, adjusting wheelbase, or changing gearing.
 */
#ifndef CONFIG_GEOMETRY_H
#define CONFIG_GEOMETRY_H

// ================= Wheel Geometry ================= //
#define WHEEL_DIAMETER_MM      39.8f     // Wheel diameter
#define TICKS_PER_REVOLUTION   359.7222f // Encoder ticks per wheel rotation
#define MM_PER_TICK            ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REVOLUTION)

// ================= Robot Geometry ================= //
#define WHEEL_BASE_MM          81.95f    // Distance between wheel centers
#define TO_CENTER_DISTANCE_MM  ((167.5f - WHEEL_DIAMETER_MM) / 2.0f)

// Derived: degrees of rotation per mm difference between wheels
#define DEG_PER_MM_DIFFERENCE  (180.0f / (3.14159265f * WHEEL_BASE_MM))

// ================= Maze Geometry ================= //
#define MAZE_SIZE              16        // 16x16 cell maze
#define CELL_SIZE_MM           180.0f    // Standard micromouse cell size
#define HALF_CELL_MM           (CELL_SIZE_MM / 2.0f)

// Aliases for clarity
#define CELL_DISTANCE_MM       CELL_SIZE_MM
#define HALF_CELL_DISTANCE_MM  HALF_CELL_MM

#endif  // CONFIG_GEOMETRY_H
