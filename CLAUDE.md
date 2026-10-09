# Jurababa 2025-2026 Micromouse

Firmware, an `mms`-compatible simulator, and calibration tools for **Jurababa**,
a Raspberry Pi Pico micromouse built for IEEE maze competitions. Treat it as a
real embedded robotics system: noisy sensors, imperfect motors, battery sag,
wheel slip, and a hard real-time budget.

## The chain

Every change keeps this chain truthful:

```text
sensors → estimated state → maze model → planned path → motor command → physical motion
```

A micromouse fails when software lies about the physical world. When a change's
effect on the chain is unclear, say so before editing.

## Prime directive

Make the smallest correct change that improves the system.

Before editing, read the files involved and state the current design and the
assumptions you are relying on. Then patch the smallest region that fixes the
problem, preserving the existing architecture and public interfaces. A refactor
happens when the user asks for one.

Search the repo before assuming how something works. Invent no APIs, files, or
conventions — check first.

Name risky design choices directly, in a sentence or two, then carry on with the
work.

Use precise units in every explanation.

## Ask before changing

Get explicit approval for these. Each one silently breaks the chain everywhere
else in the system:

- pin mappings (`firmware/config/pins.h`)
- motor direction polarity
- maze coordinate conventions
- wall encoding conventions

## Layers

Dependencies point one way. Higher layers depend on lower ones; a lower layer
never reaches up.

```text
app/ → control/, maze/, navigation/
control/ → drivers/, config/
navigation/ → maze/
maze/ → config/
drivers/ → config/
```

What each layer is allowed to know:

| Layer | Knows about | Stays out of |
|---|---|---|
| `config/` | constants, compile-time configuration | logic, algorithms, state, functions |
| `drivers/` | GPIO, PWM, ADC, I2C/UART, PIO, raw hardware units | cells, flood fill, strategy |
| `control/` | velocity, acceleration, PID, profiles, pose, kinematics | maze solving |
| `maze/` | cells, directions, walls, coordinates, mouse pose | motors, sensors |
| `navigation/` | flood fill, A*, goals, cell paths, L/F/R conversion | physical sensors |
| `app/` | wiring the above together, CLI, Bluetooth, multicore | — |

Run `ls firmware/<layer>` for the current file list; it is authoritative and
this document is not.

## Operating modes

Selected at startup: **press `M` within 3 seconds for DriverLab, otherwise
Normal Mode starts automatically.** Keep that behaviour.

**Normal Mode** is the competition mode, across both cores:

- Core 0 — path planning, maze algorithms, Bluetooth, high-level decisions.
- Core 1 — motor control, sensor updates, the control loop at `LOOP_FREQUENCY_HZ` (500 Hz).

**DriverLab Mode** is a single-core calibration and characterisation tool:
open-loop voltage sweeps, step response, PID tuning, and export of
`config.h`/`tuning.h`-compatible constants.

DriverLab is standalone. It owns its own time-based profiles
(`DriverLabProfile`), its own `PID` instances, and its trial state machines
(OL, STEP, MOVE, TURN, TURN-OL, TURN-STEP), driven by one `tick()` per 500 Hz
tick from `main.cpp`. It uses `Motor`, `Encoder`, `IMU`, `ToF` and `Battery`
directly — **the HAL drivers are the only seam** between DriverLab and the
Robot/Motion stack. Keep `Motion` and `Drivetrain` out of DriverLab.

Calibration order: `OL` (open-loop voltage sweep, for the motor constants) →
`STEP` (step response, for the motor time constant) → `EXPORT` (emit the
constants). Running them out of order gives numbers the later steps contradict.

Trials are cooperative state machines: a trial records its parameters, sets
motor voltages or arms a profile, and returns. The next tick advances it one
step. That is what holds the loop period at exactly `1 / LOOP_FREQUENCY_HZ`.

## Units

Carry units in the name whenever the bare quantity is ambiguous.

| Quantity | Unit | Suffix |
|---|---|---|
| distance | mm | `distance_mm` |
| velocity | mm/s | `velocity_mmps` |
| acceleration | mm/s² | `accel_mmps2` |
| angle | deg or rad — state which | `yaw_deg` |
| angular speed | deg/s or rad/s — state which | `omega_degps` |
| time | s or ms — state which | `dt_s`, `period_ms` |
| PWM | normalised duty or raw — state which | — |

Velocities are **mm/s** throughout: feedforward constants are duty per mm/s,
and DriverLab trials and Motion both speak mm, mm/s, mm/s².

## Naming

Google C++ Style, with these specifics:

| Element | Convention | Example |
|---|---|---|
| files | `snake_case.h/.cpp` | `flood_fill.cpp` |
| classes | `PascalCase` | `Drivetrain` |
| functions | `camelCase` | `updateState()` |
| accessors | no prefix | `velocity()`, `yawDeg()` |
| mutators | `set_` prefix | `set_target()` |
| constants | `SCREAMING_SNAKE` | `MAZE_SIZE` |
| members | `snake_case_` | `omega_degps_` |
| locals | `snake_case` | `dt_s` |

Name boolean queries for the thing itself: `wallFront()`, `finished()`,
`calibrated()`. Keep `is` only where it genuinely reads better (`isValid()`).

Action methods take imperative verbs: `reset()`, `stop()`, `moveDistance()`,
`turnAngle()`, `updateState()`.

Use the standard physics names — `omega` for angular velocity, `yaw` for
heading, `dt` for time delta, `v` for linear velocity in local math — and the
explicit suffixed form for anything persistent.

## Comments

Comment the **why**: non-obvious control logic, hardware quirks, timing
assumptions, unit conversions, maze convention assumptions, safety constraints.

```cpp
// Clamp the integral term to prevent windup while the motor command saturates.
```

Code that speaks for itself needs no comment.

## Formatting

After editing any `.cpp` or `.h`, run clang-format:

```bash
clang-format -i <file_path>
```

`.clang-format` at the repo root defines the style. If you skip formatting, say
so explicitly.

## Building

**The user builds and flashes.** Suggest commands; run them only when asked.

Firmware:

```bash
mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j4
```

Simulator: `cd sim && make`, producing `sim/bin/simulator`.

## Reporting your work

Close every code change with:

```text
Changed:
- file/path.cpp

Verification:
- Formatting run: yes/no
- Build run: yes/no
- Tests run: yes/no
- Remaining risks:
```

State what you ran and what you did not. A claim of "built and tested" means
the command appears in the transcript.

## Debugging

Restate the symptom, name the subsystem, search for the files, and read the code
before proposing a fix. Then list likely causes most-probable first, identify
the smallest experiment or inspection that separates them, and only then patch.

Per-subsystem pre-flight checks — motors, encoders, sensors, control loop and
`dt`, multicore state ownership, maze conventions, navigation assumptions — are
in **[`docs/subsystem-checklists.md`](docs/subsystem-checklists.md)**. Read the
section for the subsystem you are about to change.

## Commits

Imperative mood, present tense: `Fix encoder velocity sign convention`.

```text
<Verb> <what changed>

[optional body explaining WHY]
```

Verbs: `Add`, `Fix`, `Update`, `Refactor`, `Remove`, `Change`, `Document`,
`Tune`.

## Tooling

`.mcp.json` configures **context7** (library/framework docs — Pico SDK, PyQt6)
and **sequential-thinking** (step-by-step reasoning for hard debugging).

The calibration dashboard is `cd tools && python3 driverlab_dashboard.py`.
