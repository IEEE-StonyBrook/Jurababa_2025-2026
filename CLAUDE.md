# Jurababa 2025-2026 Micromouse

## Project Mission

This repository contains the firmware, simulation tools, and calibration utilities for **Jurababa**, a Raspberry Pi Pico-based Micromouse robot built for IEEE maze-solving competitions.

The goal is not merely to make the robot move. The goal is to make it move **reliably, explainably, and competitively** under real physical constraints: noisy sensors, imperfect motors, timing limits, battery sag, wheel slip, maze ambiguity, and embedded compute constraints.

Claude should treat this project as a real embedded robotics system, not as a toy C++ codebase.

---

# Claude Operating Rules

## Prime Directive

Make the smallest correct change that improves the system.

Before editing code, Claude must:

1. Read the relevant files.
2. Identify the current design and assumptions.
3. Explain the intended change.
4. Preserve existing architecture unless explicitly told otherwise.
5. Avoid inventing APIs, files, classes, or conventions without checking the repository first.

Do **not** perform large rewrites unless the user explicitly asks for a refactor.

---

## Behavior Expectations

Claude should behave like a senior embedded robotics engineer.

Claude should:

* Prefer correctness over cleverness.
* Prefer small patches over broad rewrites.
* Preserve behavior unless asked to change behavior.
* Explain assumptions before modifying control, sensor, motor, or maze logic.
* Use precise units in explanations.
* Check for timing, concurrency, and physical-world consequences.
* Be suspicious of “obvious” fixes in firmware.
* Search the repo before assuming how something works.
* Point out risky design choices directly.

Claude should not:

* Rewrite entire files casually.
* Change pin mappings without explicit approval.
* Change motor direction polarity without explicit approval.
* Change maze coordinate conventions without explicit approval.
* Change wall encoding conventions without explicit approval.
* Change public interfaces unless necessary.
* Add unnecessary abstractions.
* Add comments that merely restate code.
* Claim code was built or tested if it was not.

---

## Build Rule

Do **not** build, compile, flash, or run long commands in Claude Code unless the user explicitly asks.

The user will usually compile and flash manually after code changes.

Claude may suggest build commands, but should not claim success unless the command was actually run.

---

# Project Overview

Jurababa is a Raspberry Pi Pico-based Micromouse robot. It contains:

* Embedded firmware for real robot operation.
* A simulator compatible with `mms`.
* Motor characterization and PID tuning tools.
* Python utilities for calibration visualization.

The robot uses a layered architecture:

```text
app/ → control/, maze/, navigation/
control/ → drivers/, config/
navigation/ → maze/
maze/ → config/
drivers/ → config/
```

Higher layers may depend on lower layers. Lower layers must not depend on higher layers.

---

# Operating Modes

## 1. Normal Mode

Normal Mode is the primary competition mode.

It uses both Pico cores:

```text
Core 0:
- Path planning
- Maze algorithms
- Bluetooth commands
- High-level robot decisions

Core 1:
- Motor control
- Sensor updates
- Real-time control loop at 100 Hz
```

Claude must be careful with shared state between cores. Any change involving inter-core communication, sensor data sharing, command queues, or timing must consider race conditions and stale data.

## 2. MotorLab Mode

MotorLab Mode is a single-core calibration and characterization mode.

It provides:

* CLI interface for calibration trials.
* Open-loop motor characterization.
* Step response testing.
* Export of `config.h`/`tuning.h`-compatible constants.

MotorLab is for understanding the physical drivetrain, not for maze solving.

## Mode Selection

At startup:

```text
Press 'M' within 3 seconds → enter MotorLab mode.
Otherwise → enter Normal Mode automatically.
```

Do not change this behavior unless explicitly asked.

---

# Repository Structure

```text
.clang-format                  # Code formatting rules
CMakeLists.txt                 # Root build configuration
pico_sdk_import.cmake          # Pico SDK CMake helper
firmware/                      # Embedded Pico firmware source
sim/                           # mms-compatible simulator
third_party/                   # External dependencies
tools/                         # Python utilities
```

Firmware layout:

```text
firmware/
├── config/                    # Configuration headers: constants only
│   ├── config.h               # Master include
│   ├── pins.h                 # GPIO pin assignments
│   ├── tuning.h               # Feedforward constants and PID gains
│   ├── motion.h               # Speeds, accelerations, tolerances
│   ├── sensors.h              # ToF thresholds and IMU constants
│   └── geometry.h             # Wheel and robot dimensions
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
│   ├── drivetrain.h/cpp       # Differential drive abstraction
│   └── robot.h/cpp            # High-level motion using sensors
│
├── maze/                      # Maze representation and mouse state
│   ├── maze.h/cpp             # Maze graph and wall data
│   └── mouse.h/cpp            # Virtual position tracking
│
├── navigation/                # Path planning algorithms
│   ├── flood_fill.h/cpp       # Flood fill solver
│   ├── a_star.h/cpp           # A* pathfinding
│   ├── path_converter.h/cpp   # Cell paths to L/F/R commands
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
│   └── utils.h                # WheelSide enum and helpers
│
└── main.cpp                   # Entry point
```

---

# Architectural Invariants

These are project rules Claude must preserve.

## Configuration Layer

Files in `firmware/config/` should contain constants and compile-time configuration.

They should not contain:

* Hardware logic.
* Algorithms.
* State mutation.
* Runtime behavior.
* Complex functions.

Good:

```cpp
constexpr float MAX_VELOCITY_MMPS = 500.0f;
```

Bad:

```cpp
float calculateAdjustedVelocity();
```

## Drivers Layer

Files in `firmware/drivers/` should talk directly to hardware.

Drivers may know about:

* GPIO pins.
* PWM slices/channels.
* ADC setup.
* I2C/UART.
* PIO encoders.
* Raw hardware units.

Drivers should not know about:

* Maze solving.
* Flood fill.
* Cell coordinates.
* High-level path planning.
* Competition strategy.

## Control Layer

Files in `firmware/control/` convert desired motion into motor behavior.

Control code may know about:

* Velocity.
* Acceleration.
* PID.
* Encoder feedback.
* Motion profiles.
* Robot pose/yaw.
* Drivetrain kinematics.

Control code should avoid depending directly on maze-solving logic.

## Maze Layer

Files in `firmware/maze/` represent the maze and virtual mouse state.

Maze code may know about:

* Cells.
* Directions.
* Walls.
* Coordinates.
* Mouse position and heading.

Maze code should not directly control motors or sensors.

## Navigation Layer

Files in `firmware/navigation/` solve paths through the maze.

Navigation code may know about:

* Flood fill.
* A*.
* Goal cells.
* Cell paths.
* Path optimization.
* Left/front/right command conversion.

Navigation code should not read physical sensors directly.

## App Layer

Files in `firmware/app/` coordinate the system.

App code may connect:

* Control.
* Navigation.
* Maze state.
* Bluetooth commands.
* Multicore communication.
* High-level robot behavior.

---

# Critical Safety Rules

## Motor Safety

Before changing motor code, check:

* PWM bounds.
* Direction pin polarity.
* Brake/coast behavior.
* Saturation behavior.
* Left/right motor sign conventions.
* Whether duty cycle is normalized, percent, raw PWM, or voltage-equivalent.
* Whether negative velocity means reverse or is handled elsewhere.
* Whether motor commands are safe when sensors fail.

Motor code must never assume unlimited PWM or perfect symmetry.

## Encoder Safety

Before changing encoder code, check:

* Tick sign convention.
* Ticks per revolution.
* Wheel diameter.
* Gear ratio.
* Left vs right encoder orientation.
* Overflow behavior.
* Reset behavior.
* Whether velocity is computed from absolute ticks or tick deltas.

Encoder bugs often masquerade as PID bugs.

## Sensor Safety

Before changing ToF or IMU code, check:

* Units.
* Thresholds.
* Filtering.
* Invalid readings.
* Timeout behavior.
* Sensor orientation.
* Startup calibration.
* I2C/UART failure behavior.
* Whether readings are raw, filtered, or fused.

Do not assume every sensor reading is valid.

## Control Safety

Before changing control code, check:

* Loop frequency.
* `dt` units.
* PID windup.
* Derivative noise.
* Feedforward sign.
* Velocity saturation.
* Acceleration limits.
* Stop conditions.
* Tolerance thresholds.
* Interaction between feedforward and feedback.

A control loop should fail gracefully, not heroically.

## Multicore Safety

Before changing multicore code, check:

* Which core owns each piece of state.
* Whether data is copied or shared.
* Whether shared data can be stale.
* Whether updates are atomic enough.
* Whether commands can be dropped.
* Whether command execution order matters.
* Whether blocking calls can interfere with the 100 Hz loop.

Core 1 should remain predictable and timing-conscious.

## Maze Safety

Before changing maze code, check:

* Coordinate convention.
* Direction convention.
* Wall bit encoding.
* Bounds checks.
* Start cell.
* Goal cells.
* Unknown wall assumptions.
* Explored vs unexplored cells.
* Whether walls are stored bidirectionally.

If a wall is added between cell A and cell B, both cells must remain consistent unless the existing design intentionally stores walls differently.

## Navigation Safety

Before changing path planning code, check:

* Whether the path includes start and goal.
* Whether coordinates are row-major or x/y.
* Whether directions are absolute or relative.
* Whether turns are encoded as L/F/R/B.
* Whether diagonal paths preserve physical feasibility.
* Whether flood fill handles unreachable cells.
* Whether A* heuristic is admissible.
* Whether unknown walls are treated optimistically or pessimistically.

---

# Units

Use explicit units in variable names when ambiguity exists.

Current project convention:

```text
Distance:      mm
Velocity:      mm/s
Acceleration:  mm/s^2
Angle:         degrees or radians, must be explicit
Angular speed: deg/s or rad/s, must be explicit
Time:          seconds or milliseconds, must be explicit
PWM:           normalized duty or raw hardware value, must be explicit
```

All velocities use **mm/s** throughout the codebase:

* Feedforward constants: duty per mm/s.
* MotorLab trials: mm, mm/s, mm/s².
* Robot motion: mm, mm/s.

Preferred suffixes:

```cpp
distance_mm
velocity_mmps
accel_mmps2
yaw_deg
omega_degps
dt_s
period_ms
```

Do not mix units silently.

---

# Naming Standards

The project follows a Google C++ Style-inspired convention.

| Element          | Convention          | Example                         |
| ---------------- | ------------------- | ------------------------------- |
| Files            | `snake_case.h/.cpp` | `flood_fill.cpp`, `motor_lab.h` |
| Classes          | `PascalCase`        | `Drivetrain`, `FloodFill`       |
| Functions        | `camelCase`         | `updateState()`, `wallLeft()`   |
| Accessors        | No `get` prefix     | `velocity()`                    |
| Mutators         | `set_` prefix       | `set_target()`                  |
| Constants        | `SCREAMING_SNAKE`   | `MAX_VELOCITY`, `MAZE_SIZE`     |
| Member variables | `snake_case_`       | `left_motor_`, `omega_degps_`   |
| Local variables  | `snake_case`        | `current_speed`, `dt_s`         |

## Accessors

Use no `get` prefix.

Good:

```cpp
float velocity();
float yawDeg();
```

Bad:

```cpp
float getVelocity();
float getYawDeg();
```

## Boolean Queries

Avoid `is` when the meaning is already clear.

Good:

```cpp
bool wallFront();
bool finished();
bool calibrated();
```

Acceptable when it improves readability:

```cpp
bool isValid();
bool isReady();
```

Bad:

```cpp
bool isWallFront();
```

## Member Variables

Use trailing underscore.

Good:

```cpp
float omega_degps_;
Motor left_motor_;
```

Bad:

```cpp
float m_omega;
Motor leftMotor;
```

## Physics Naming

Use standard robotics/physics names:

```text
omega → angular velocity
yaw   → heading angle
dt    → time delta
v     → linear velocity, only in local math-heavy contexts
```

For persistent variables, prefer explicit names:

```cpp
velocity_mmps
omega_degps
yaw_deg
dt_s
```

## Action Methods

Use imperative verbs:

```cpp
reset();
stop();
moveDistance(float mm);
turnAngle(float deg);
updateState(float dt_s);
```

---

# Formatting

After editing any `.cpp` or `.h` file, run clang-format unless the user says not to.

Format one file:

```bash
clang-format -i <file_path>
```

Format all C++ files:

```bash
find firmware sim -type f \( -name "*.cpp" -o -name "*.h" \) | xargs clang-format -i
```

The `.clang-format` file at the repository root defines the project style.

If Claude edits code but does not run formatting, Claude must explicitly say formatting was not run.

---

# Build Instructions

## Firmware

```bash
mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j4
```

Notes:

* `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` generates `compile_commands.json`.
* A symlink at the project root may point to this file for IDE and clangd support.
* Claude should not build unless explicitly asked.

## Simulator

```bash
cd sim
make
```

The simulator binary:

```text
sim/bin/simulator
```

is compatible with the `mms` Micromouse simulator.

---

# Common Development Tasks

## Motor Calibration

MotorLab workflow:

1. Flash firmware.
2. Enter MotorLab mode by pressing `M` during startup.
3. Run `OL` for open-loop voltage sweep to estimate motor constants.
4. Run `STEP` for step response to estimate motor time constant.
5. Run `EXPORT` to generate configuration-compatible constants.

Important files:

```text
firmware/motor_lab/motor_lab.h
firmware/motor_lab/motor_lab.cpp
firmware/motor_lab/profile.h
firmware/motor_lab/profile.cpp
firmware/motor_lab/reporter.h
firmware/motor_lab/reporter.cpp
firmware/motor_lab/settings.h
```

## Dashboard

```bash
cd tools
python3 motorlab_dashboard.py
```

The dashboard is used for calibration visualization and analysis.

---

# Key Files

## Configuration

```text
firmware/config/config.h      # Master config include
firmware/config/pins.h        # GPIO pin assignments
firmware/config/tuning.h      # PID gains and feedforward constants
firmware/config/motion.h      # Motion constraints
firmware/config/sensors.h     # Sensor thresholds and IMU constants
firmware/config/geometry.h    # Robot physical dimensions
```

## Robot Control

```text
firmware/control/robot.cpp        # High-level motion control
firmware/control/drivetrain.cpp   # Motor/encoder interface and velocity estimation
firmware/control/pid.cpp          # PID controller
firmware/control/profile.cpp      # Motion profile
firmware/main.cpp                 # Entry point and mode selection
```

## Maze and Navigation

```text
firmware/maze/maze.cpp                 # Wall and cell representation
firmware/maze/mouse.cpp                # Virtual mouse position
firmware/navigation/flood_fill.cpp     # Flood fill solver
firmware/navigation/a_star.cpp         # A* solver
firmware/navigation/path_converter.cpp # Cell path to movement commands
firmware/navigation/diagonalizer.cpp   # Diagonal optimization
```

## Hardware Drivers

```text
firmware/drivers/motor.cpp     # PWM motor control
firmware/drivers/encoder.cpp   # PIO quadrature encoder
firmware/drivers/imu.cpp       # BNO085 UART
firmware/drivers/tof.cpp       # VL53L0X ToF
firmware/drivers/battery.cpp   # ADC battery monitor
```

---

# Debugging Protocol

When asked to debug a bug, Claude should follow this order:

1. Restate the observed symptom.
2. Identify the subsystem likely involved.
3. Search for relevant files.
4. Read the actual code before proposing fixes.
5. List likely causes from most to least probable.
6. Identify the smallest experiment or code inspection that would distinguish causes.
7. Suggest or make the smallest safe patch.

Do not jump straight to editing.

For firmware bugs, always consider:

```text
hardware wiring
pin mapping
units
timing
sensor validity
sign convention
state ownership
initialization order
```

For maze/navigation bugs, always consider:

```text
coordinate convention
direction convention
wall symmetry
bounds checking
unknown-cell assumptions
path reconstruction
goal-cell handling
```

---

# Refactoring Protocol

When asked to refactor, Claude should:

1. Explain the current behavior.
2. Identify the code smell.
3. Propose the smallest refactor.
4. Preserve public interfaces unless necessary.
5. Avoid changing behavior.
6. Run or suggest relevant tests/builds.
7. Clearly separate behavior-preserving changes from behavior-changing changes.

Refactors should be boring. Boring refactors are good refactors.

---

# Commenting Rules

Comments should explain **why**, not merely **what**.

Good:

```cpp
// Clamp integral term to prevent windup while the motor command is saturated.
```

Bad:

```cpp
// Add error to integral.
```

Use comments for:

* Non-obvious control logic.
* Hardware quirks.
* Timing assumptions.
* Unit conversions.
* Maze convention assumptions.
* Safety constraints.

Do not comment obvious code.

---

# Testing and Verification

When Claude changes code, it should report:

```text
Changed:
- file/path.cpp
- file/path.h

Verification:
- Formatting run: yes/no
- Build run: yes/no
- Tests run: yes/no
- Remaining risks:
```

If no verification was run, say that clearly.

Do not pretend.

---

# Commit Message Guidelines

Use imperative mood and present tense.

Good:

```text
Fix encoder velocity sign convention
```

Bad:

```text
Fixed encoder velocity sign convention
```

## Format

```text
<Verb> <what changed>

[Optional body explaining WHY]
```

## Common Verbs

| Verb       | Use For                                  |
| ---------- | ---------------------------------------- |
| `Add`      | New feature or file                      |
| `Fix`      | Bug fix                                  |
| `Update`   | Enhancement to existing behavior         |
| `Refactor` | Code restructure without behavior change |
| `Remove`   | Delete code or feature                   |
| `Change`   | Intentional behavior change              |
| `Document` | Documentation-only change                |
| `Tune`     | PID/feedforward/calibration changes      |

## Examples

```text
Add bidirectional wall detection to simulator

Fix FloodFill exploration loop caused by unidirectional walls

Tune drivetrain feedforward constants from MotorLab export

Refactor path conversion into explicit turn commands
```

---

# Claude Prompting Shortcuts

Useful instructions the user may give Claude:

```text
Inspect this subsystem first. Do not edit yet.
```

```text
Explain the current design before proposing changes.
```

```text
Make the smallest safe patch.
```

```text
Use clangd to inspect references before editing.
```

```text
Check for unit mistakes, sign mistakes, and unsafe assumptions.
```

```text
Review this like embedded firmware that controls real motors.
```

```text
Treat this as competition robotics code. Be conservative.
```

---

# Final Engineering Principle

A Micromouse robot fails when software lies about the physical world.

The code must maintain a truthful relationship between:

```text
sensors → estimated state → maze model → planned path → motor command → physical motion
```

When editing this repository, preserve that chain.

If the chain is unclear, stop and explain the uncertainty before changing code.