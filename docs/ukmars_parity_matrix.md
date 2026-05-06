# UKMARS Parity Matrix

This file is the working checklist for keeping Jurababa aligned with UKMARS Maze Runner Core
without losing the intentional Jurababa upgrades.

## Preserved Jurababa Upgrades

- Tuned PID/feedforward constants and wheel geometry.
- A*/LFR/diagonalizer planning.
- ToF sensors and asymmetric left/right ToF calibration.
- IMU yaw/omega rotation feedback.
- DriverLab as a standalone calibration tool.
- Bluetooth console reliability fixes.
- CLI `PATH`, `STYLE`, `STAGE`, working `W`, and explored-only fast runs.

## Architecture

| UKMARS | Jurababa Current | Target | Action |
|---|---|---|---|
| `Mouse` physical robot/search facade | `Mouse` | `Mouse` | Renamed/adapted |
| Virtual maze position inside `Mouse` | `MazeMouse` | `MazeMouse` | Renamed |
| `Motion` | `Motion` | `Motion` | Renamed/adapted |
| `Maze` | `Maze` | `Maze` with mask/wall-state semantics | Adapt carefully |
| `Sensors` | `FirmwareMouse` + `ToF` + `Motion` caches | ToF sensor service + UKMARS wall flags/steering | Adapted |
| `Reporter` | `Log` + CLI prints | Reporter-style compact reports | Compact action logs added |
| `DriverLab` | `DriverLab` | `DriverLab` | Keep separate |

## Mouse / Search

| UKMARS Feature | Jurababa Current | Target | Action |
|---|---|---|---|
| `State` | `State` plus `MovementStyle` | `State` plus style | Added |
| `m_handStart` | Physical `Mouse` start logic | Physical facade-owned hand-start state | Added |
| `init()` | `setUp`, CLI reset | UKMARS-style init adapted to virtual mouse | Added |
| `set_heading()` | virtual direction methods | Delegate to virtual mouse | Added |
| `search_maze()` | `STAGE`/`SEARCH` | UKMARS workflow using A*/LFR `search_to` | Added |
| `search_to()` | Present | UKMARS physical flow, A*/LFR decision | Adapted |
| `update_map()` | Present | update unknown walls only | Adapted |
| `move_ahead()` | Present | `adjust_forward_position(-CELL_SIZE_MM)` | Verified/kept |
| `turn_left/right/back()` | Present | UKMARS stop/adjust/turn/return | Verified/kept |
| `turn_to_face()` | Present | UKMARS heading helper | Added |
| `panic()` | Present | motor-safe stop + visible LED recovery cue | Added |
| `run_to()` | Present | Explored-only fast-run wrapper | Added as Jurababa upgrade |
| `follow_to()` | Present | Left-wall follow setup helper | Added |
| `wander_to()` | Present | Random wander setup helper | Added |
| `randomHeading()` / `getRandomBool()` | Present | UKMARS wander helpers | Added |
| `test_SS90E()` | Present | Named smooth-turn setup test | Added |

## Motion / Profile

| UKMARS Feature | Jurababa Current | Target | Action |
|---|---|---|---|
| `reset_drive_system()` | Present | Same | Verified/kept |
| `disable_drive()` | Present | Safe output disable/stop | Added |
| `wait_until_position()` | `Mouse` helper | Motion helper too | Added |
| `wait_until_distance()` | Present | UKMARS helper | Added |
| Profile 5 mm braking trick | Present | Keep | Verified/kept |
| UKMARS PD + feedforward mixer | Present | Keep tuned constants | Verified/kept |
| Encoder-derived rotation | Not used | IMU yaw delta | Intentional difference |

## Maze / Planner

| UKMARS Feature | Jurababa Current | Target | Action |
|---|---|---|---|
| `WallState` | `EXIT/WALL/UNKNOWN/VIRTUAL` | `EXIT/WALL/UNKNOWN/VIRTUAL` | Added |
| `MazeMask` | `MASK_OPEN/MASK_CLOSED` | `MASK_OPEN/MASK_CLOSED` | Added |
| `update_wall_state()` | exact-name wrapper | update unknown only | Added |
| `set_wall_state()` | exact-name wrapper | unconditional wall-state write | Added |
| A*/LFR/diagonalizer | Proven | Primary planner | Keep |

## Sensors / ToF Steering

| UKMARS Feature | Jurababa Current | Target | Action |
|---|---|---|---|
| `see_left/front/right` | exact-name wrappers | cached wall flags from ToF service | Adapted |
| `STEER_NORMAL` | runtime mode | runtime mode, disabled by default | Added |
| `STEER_LEFT_WALL` | runtime mode | diagnostic/future mode | Added |
| `STEER_RIGHT_WALL` | runtime mode | diagnostic/future mode | Added |
| `STEERING_OFF` | explicit default mode | explicit default mode | Added/default |
| Cross-track error | diagnostics | UKMARS-shaped, asymmetric ToF normalized | Adapted |

## Reporter / CLI

| UKMARS Feature | Jurababa Current | Target | Action |
|---|---|---|---|
| `log_action_status()` | compact action block | compact action block | Added |
| `print_wall_sensors()` | ToF/steering diagnostic | raw mm + wall flags + CTE | Added |
| `report_profile()` | profile diagnostic | position/velocity/angle/omega report | Added |
| `conf_log_front_sensor()` | setup diagnostic | front ToF vs profile position | Added |
| `conf_sensor_spin_calibrate()` | setup diagnostic | 360-degree ToF/IMU spin report | Added |
| `conf_edge_detection()` | setup diagnostic | side ToF wall-edge position report | Added |
| `print_maze(PLAIN)` | `W` works | preserve | Keep |
| `print_maze(COSTS)` | real cost view | real cost view | Added |
| `print_maze(DIRS)` | real direction view | real direction view | Added |
| `COMP` | removed | remove from normal CLI | Deleted |
