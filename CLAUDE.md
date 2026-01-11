# Jurababa 2025-2026 Micromouse

## Project Overview
Raspberry Pi Pico-based micromouse robot for IEEE maze-solving competitions.

## Architecture

### Operating Modes
1. **Normal Mode** (dual-core): Maze solving with real-time control
   - Core 0: Path planning, maze algorithms, Bluetooth commands
   - Core 1: Motor control at 100Hz, sensor updates

2. **MotorLab Mode** (single-core): Motor characterization and PID tuning
   - CLI interface for calibration trials
   - Outputs Config.h-compatible feedforward constants

### Mode Selection
Press 'M' within 3 seconds of startup to enter MotorLab mode. Otherwise, Normal mode starts automatically.

## Key Files

### Configuration
- `Include/Platform/Pico/Config.h`: All robot constants (wheel size, PID gains, feedforward, etc.)

### Robot Control
- `src/Platform/Pico/Robot/Robot.cpp`: High-level motion control
- `src/Platform/Pico/Robot/Drivetrain.cpp`: Motor/encoder interface, velocity estimation
- `src/Main.cpp`: Entry point with mode selection

### MotorLab
- `Include/Platform/Pico/MotorLab/MotorLab.h`: CLI interface for motor testing
- `tools/motorlab_dashboard.py`: Python GUI for calibration visualization

## Units
All velocities use **mm/s** throughout the codebase:
- Config.h feedforward constants: duty per mm/s
- MotorLab trials: mm, mm/s, mm/s^2
- Robot motion: mm, mm/s

## Build
```bash
mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk
make -j4
```

## Common Tasks

### Motor Calibration
1. Enter MotorLab mode (press 'M' at startup)
2. Run `OL` for open-loop voltage sweep to find Km
3. Run `STEP` for step response to find Tm
4. Run `EXPORT` to generate Config.h constants

### Dashboard
```bash
cd tools
python3 motorlab_dashboard.py
```
