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

## Directory Structure

```
firmware/                      # Embedded Pico firmware
sim/                           # mms-compatible simulator
third_party/                   # External dependencies
tools/                         # Python utilities

firmware/                      # All firmware source code
├── config/                    # Configuration headers (constants only)
│   ├── config.h               # Master include (includes all below)
│   ├── pins.h                 # GPIO pin assignments
│   ├── tuning.h               # Feedforward KV/KS, PID gains
│   ├── motion.h               # Speeds, accelerations, tolerances
│   ├── sensors.h              # ToF thresholds, IMU constants
│   └── geometry.h             # Wheel/robot dimensions
│
├── drivers/                   # Hardware abstraction layer
│   ├── motor.h/cpp            # PWM motor control
│   ├── encoder.h/cpp          # PIO quadrature encoder
│   ├── imu.h/cpp              # BNO085 UART interface
│   ├── tof.h/cpp              # VL53L0X I2C interface
│   └── battery.h/cpp          # ADC voltage monitor
│
├── control/                   # Motion control layer
│   ├── pid.h/cpp              # Generic PID controller
│   ├── profile.h/cpp          # Trapezoidal motion profile
│   ├── drivetrain.h/cpp       # Differential drive
│   └── robot.h/cpp            # High-level motion (integrated sensors)
│
├── maze/                      # Maze representation
│   ├── maze.h/cpp             # Maze graph with cells
│   └── mouse.h/cpp            # Virtual position tracking
│
├── navigation/                # Path planning algorithms
│   ├── flood_fill.h/cpp       # Flood fill solver
│   ├── a_star.h/cpp           # A* pathfinding
│   ├── path_converter.h/cpp   # Cell paths to LFR commands
│   ├── path_utils.h/cpp       # Path execution utilities
│   └── diagonalizer.h/cpp     # Diagonal path optimization
│
├── app/                       # Application layer
│   ├── bluetooth.h/cpp        # Wireless serial interface
│   ├── commands.h             # Inter-core command hub
│   ├── multicore.h            # Sensor data sharing
│   └── api.h/cpp              # High-level maze interface
│
├── motor_lab/                 # Motor characterization tool
│   ├── motor_lab.h/cpp        # CLI interface
│   ├── profile.h/cpp          # Time-based motion profile
│   ├── reporter.h/cpp         # CSV data logging
│   └── settings.h             # Calibration parameters
│
├── common/                    # Shared utilities
│   ├── log.h/cpp              # Logging system
│   └── utils.h                # WheelSide enum, helpers
│
├── main.cpp                   # Entry point
└── CMakeLists.txt             # Build configuration
```

## Coding Standards (Google C++ Style Based)

### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| **Files** | `snake_case.h/.cpp` | `flood_fill.cpp`, `motor_lab.h` |
| **Classes** | `PascalCase` | `Drivetrain`, `FloodFill` |
| **Functions** | `camelCase` | `updateState()`, `wallLeft()` |
| **Accessors** | No `get` prefix | `velocity()` not `getVelocity()` |
| **Mutators** | `set_` prefix | `set_target()` |
| **Constants** | `SCREAMING_SNAKE` | `MAX_VELOCITY`, `MAZE_SIZE` |
| **Member vars** | `snake_case_` | `left_motor_`, `omega_degps_` |
| **Local vars** | `snake_case` | `current_speed`, `dt` |

### Key Rules

1. **No `get` prefix for accessors** (Google style):
   ```cpp
   float velocity();     // Good
   float getVelocity();  // Bad
   ```

2. **No `is` prefix for boolean queries when clear from context**:
   ```cpp
   bool wallFront();     // Good
   bool finished();      // Good
   bool isWallFront();   // Bad
   ```

3. **Trailing underscore for member variables**:
   ```cpp
   class Robot {
       float omega_degps_;   // Good
       float m_omega;        // Bad (m_ prefix)
   };
   ```

4. **Physics naming conventions**:
   - `omega` for angular velocity (rad/s or deg/s)
   - `yaw` for heading angle
   - `dt` for time delta

5. **Action methods use imperative verbs**:
   ```cpp
   void reset();
   void stop();
   void moveDistance(float mm);
   ```

### Layer Dependencies

Each layer only depends on layers below:
```
app/ → control/, maze/, navigation/
control/ → drivers/, config/
navigation/ → maze/
maze/ → config/
drivers/ → config/
```

## Key Files

### Configuration
- `firmware/config/config.h`: Master include for all constants
- `firmware/config/tuning.h`: PID gains, feedforward constants

### Robot Control
- `firmware/control/robot.cpp`: High-level motion control with integrated sensors
- `firmware/control/drivetrain.cpp`: Motor/encoder interface, velocity estimation
- `firmware/main.cpp`: Entry point with mode selection

### MotorLab
- `firmware/motor_lab/motor_lab.h`: CLI interface for motor testing
- `tools/motorlab_dashboard.py`: Python GUI for calibration visualization

## Units
All velocities use **mm/s** throughout the codebase:
- Config.h feedforward constants: duty per mm/s
- MotorLab trials: mm, mm/s, mm/s^2
- Robot motion: mm, mm/s

## Build

### Firmware (Pico)
```bash
mkdir -p build-firmware && cd build-firmware
cmake ../firmware -DPICO_SDK_PATH=/path/to/pico-sdk
make -j4
```

### Simulator (mms)
```bash
cd sim
make
```

The simulator binary `sim/bin/simulator` is compatible with the [mms](https://github.com/mackorone/mms) micromouse simulator.

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
