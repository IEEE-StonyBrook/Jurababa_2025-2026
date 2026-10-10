/**
 * @file geometry.h
 * @brief Physical dimensions — change when swapping hardware
 */
#ifndef CONFIG_GEOMETRY_H
#define CONFIG_GEOMETRY_H

// ===================== Wheels ===================== //
#define WHEEL_DIAMETER_MM 38.8f // Effective, from 500 mm Q pushes (calipers: 37.2)
#define WHEEL_BASE_MM     83.9f // Effective, from IMU-checked spins (calipers: 84.5)

// ===================== Encoder ===================== //
#define ENCODER_CPR 12     // Counts per motor shaft revolution
#define GEAR_RATIO  29.86f // Motor revolutions per wheel revolution

// Derived
#define TICKS_PER_REVOLUTION (ENCODER_CPR * GEAR_RATIO) // 358.32
#define MM_PER_TICK_NOMINAL  ((WHEEL_DIAMETER_MM * 3.14159265f) / TICKS_PER_REVOLUTION)

// Per-wheel distance per encoder tick, calibrated the mazerunner-core way:
// run `Q` in the CLI, push the robot by hand in a straight line through
// exactly 500 mm (only enough downward pressure to stop the wheels slipping),
// press a key, and paste the printed `500.0f / counts` values here. Repeat a
// few runs and average; a longer push is more accurate. It is normal for the
// two wheels to differ. Until calibrated both use the tyre-diameter estimate.
#define MM_PER_TICK_LEFT  MM_PER_TICK_NOMINAL
#define MM_PER_TICK_RIGHT MM_PER_TICK_NOMINAL
// Mean of the two, for logs and printouts only.
#define MM_PER_TICK           ((MM_PER_TICK_LEFT + MM_PER_TICK_RIGHT) / 2.0f)
#define DEG_PER_MM_DIFFERENCE (180.0f / (3.14159265f * WHEEL_BASE_MM))

// ===================== Maze ===================== //
#define MAZE_SIZE    16     // 16x16 cell maze
#define CELL_SIZE_MM 180.0f // Standard micromouse cell
#define HALF_CELL_MM (CELL_SIZE_MM / 2.0f)

// Start with the back/wheel reference aligned to the first cell's rear wall.
#define START_BACK_REFERENCE_OFFSET_MM 6.0f
#define WALL_CHECK_OFFSET_MM           (START_BACK_REFERENCE_OFFSET_MM + (WHEEL_DIAMETER_MM / 2.0f))
#define WALL_CHECK_TO_CENTER_MM        (HALF_CELL_MM - WALL_CHECK_OFFSET_MM)
#define CENTER_TO_NEXT_WALL_CHECK_MM   (HALF_CELL_MM + WALL_CHECK_OFFSET_MM)
#define START_CENTER_DISTANCE_MM       WALL_CHECK_TO_CENTER_MM

#endif // CONFIG_GEOMETRY_H
