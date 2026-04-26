/**
 * @file geometry.h
 * @brief Physical dimensions — change when swapping hardware
 */
#ifndef CONFIG_GEOMETRY_H
#define CONFIG_GEOMETRY_H

// ===================== Wheels ===================== //
#define WHEEL_DIAMETER_MM 42.2f  // Measured outer diameter
#define WHEEL_BASE_MM     81.95f // Center-to-center between wheels

// ===================== Encoder ===================== //
#define ENCODER_CPR     12     // Counts per motor shaft revolution
#define QUADRATURE_MULT 4      // PIO counts all 4 edges (A rise/fall, B rise/fall)
#define GEAR_RATIO      29.86f // Motor revolutions per wheel revolution

// Derived
#define TICKS_PER_REVOLUTION  (ENCODER_CPR * QUADRATURE_MULT * GEAR_RATIO) // 1433.28
#define MM_PER_TICK           ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REVOLUTION)
#define DEG_PER_MM_DIFFERENCE (180.0f / (3.14159265f * WHEEL_BASE_MM))

// ===================== Maze ===================== //
#define MAZE_SIZE             16     // 16x16 cell maze
#define CELL_SIZE_MM          180.0f // Standard micromouse cell
#define HALF_CELL_MM          (CELL_SIZE_MM / 2.0f)
#define CELL_DISTANCE_MM      CELL_SIZE_MM
#define HALF_CELL_DISTANCE_MM HALF_CELL_MM

// ===================== Robot Body ===================== //
// Distance from wheel axle to front of robot (for wall alignment)
#define TO_CENTER_DISTANCE_MM ((167.5f - WHEEL_DIAMETER_MM) / 2.0f)

#endif // CONFIG_GEOMETRY_H
