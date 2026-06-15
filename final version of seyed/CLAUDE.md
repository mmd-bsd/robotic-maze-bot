# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project

**SEYED** — a line-following maze robot. The robot drives along a 90° grid maze, searches for a target (big black area), proves it found the fastest route, returns to start, then races the fastest path. The fastest path minimises **time** (not distance) under an acceleration model — the robot stops at every turn and accelerates on clear straights.

**Three parts:**
1. **Python/Tkinter simulator** (`final version of seyed/simulator/`) — active
2. **C maze-solving library** (`robot codes/`) — active C port for STM32 firmware
3. **STM32 firmware** (`New Start/code/Core/`) — runs on STM32G031G8Ux (8KB RAM, 64KB flash)

**Canonical docs** (read these first in any session):
- [`ARCHITECTURE.md`](./ARCHITECTURE.md) — layout, rules, conventions, glossary
- [`ALGORITHMS.md`](./ALGORITHMS.md) — every algorithm in detail
- [`CHANGELOG.md`](./CHANGELOG.md) — worklog + hard-won lessons; **update it whenever you change code**

---

## How to run

```powershell
python "final version of seyed/simulator/maze solver/maze_solver.py"
python "final version of seyed/simulator/maze generator/maze_generator.py"
```

**Dependencies:** Python 3.x with tkinter (stdlib). Pillow (`pip install pillow`) optional — anti-aliased circles fall back gracefully.

---

## Key rules

1. **No cheating during search** — the robot must not use the target's position until `target_found` is True.
2. **All bounds/pruning use TIME**, not distance (acceleration model — `v_max` matters).
3. **Work only in `final version of seyed/simulator/`.** `New Start/` and `old docs/` are read-only history.
4. **Shell is PowerShell on Windows** — chain with `;`, quote paths with spaces.
5. **After any code change**, append an entry to `CHANGELOG.md`. Keep `ARCHITECTURE.md`/`ALGORITHMS.md` in sync.
6. **Units are real-world:** 1 world unit = 1 cm. Speeds cm/s, accel cm/s².
7. **Prefer standard library**, single-file scripts. No `networkx`/`matplotlib` in the final-version simulator.
8. **Dark theme** — reuse the color constants at the top of each file.

---

## Headless testing pattern

```python
import importlib, tkinter as tk
root = tk.Tk(); root.withdraw()
SolverApp = importlib.import_module("maze solver.maze_solver").SolverApp
app = SolverApp(root)
while app.phase != "DONE":
    app._frame(dt)
```

This runs the full mission without `mainloop()` — no display needed. Call `sys.stdout.reconfigure(encoding='utf-8')` before printing unicode.

---

## C code reference map

**Primary C code:** `robot codes/` — the portable C maze solver + STM32 HAL bridge.
See `robot codes/STATUS.md` for full file listing and build commands.

For the STM32 integration, these are the key reference files (read-only, do not modify):

| What | Path | Status |
|---|---|---|
| Working robot firmware | `New Start/code/Core/Src/main.c` | Left-hand-rule explorer, basic string-path, `loop_start` FSM |
| STM sample with maze_gbf integrated | `New Start/Simulator/py-code/maze solving/maze_gbf/stm-sample-code/main.c` | Shows `USE_MAZE_GBF` integration via `maze_hal.h` |
| HAL bridge | `New Start/Simulator/py-code/maze solving/maze_gbf/stm-sample-code/maze_hal.h` | Translates `MazeDirection` → firmware `cross`/motion primitives |
| C maze library (types) | `New Start/Simulator/py-code/maze solving/maze_gbf/inc/maze_types.h` | `MazeNode`, `MazeEdge`, `MazeGraph`, `MazeRobot`, `MazeSensors` |
| C maze library (graph) | `New Start/Simulator/py-code/maze solving/maze_gbf/inc/maze_graph.h` | BFS shortest-path, neighbor queries on known map |
| C maze library (robot) | `New Start/Simulator/py-code/maze solving/maze_gbf/inc/maze_robot.h` | Frontier detection, direction calculation, path management |
| C maze library (main) | `New Start/Simulator/py-code/maze solving/maze_gbf/src/maze_gbf.c` | 3-priority GBF: unexplored→frontier→return home |
| C test (standalone) | `New Start/Simulator/c-code/C-test-4.c` | Standalone C test with Dijkstra, command generation, full maze |
| **New C solver** (types) | `robot codes/inc/maze_types.h` | `MazeNode`, `MazeEdge`, `MazeGraph`, `MazeRobot`, `MazeSensors` |
| **New C solver** (config) | `robot codes/inc/maze_config.h` | Memory limits, motion-model params, compile-time validation |
| **New C solver** (graph) | `robot codes/inc/maze_graph.h` | Binary-heap Dijkstra by real distance on known map |
| **New C solver** (robot) | `robot codes/inc/maze_robot.h` | Heading, F/L/R/B commands, frontiers, branch ranking |
| **New C solver** (explore) | `robot codes/inc/maze_explore.h` | P1→P2→P3 target-agnostic exploration |
| **New C solver** (proof) | `robot codes/inc/maze_proof.h` | Time-based admissible lower bound, frontier pruning |
| **New C solver** (fastrun) | `robot codes/inc/maze_fastrun.h` | Trapezoidal profile, stop-graph Dijkstra, time-optimal path |
| **New C solver** (FSM) | `robot codes/inc/maze_solver.h` | EXPLORE→RETURN_HOME→FAST_RUN→DONE |
| **New C solver** (HAL) | `robot codes/inc/maze_hal.h` | STM32 HAL bridge — sensors, position, `maze_hal_tick()` |

**Hardware:** STM32G031G8Ux (Cortex-M0+, 8KB RAM, 64KB flash), 18 IR sensors, LSM6DS3TR IMU, GTD servo motors, Bluetooth UART debug.
