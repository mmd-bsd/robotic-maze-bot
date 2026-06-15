# BUILD GUIDE -- SEYED C Maze Solver

**Location:** `final version of seyed/robot codes/`

---

## How to run a maze

Design a maze in the simulator, save it as `.json`, then:

```powershell
# Test any maze in one command:
python scripts/run_maze.py ../simulator/mazes/sample_maze.json
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_maze.py ../simulator/mazes/your_maze.json
```

What it does automatically:
1. Reads the `.json`
2. Generates `test/_maze_data.h` (temp)
3. Compiles `run_maze.c` + all solver sources
4. Runs the `.exe` and prints results
5. Cleans up `_maze_data.h`

Example output:

```
=== Maze Run: sample_maze4.json ===
  Nodes: 37, Edges: 37
  Start: (-20,-20)  Target: (40,-120)

-- Results --
  Steps:             97
  Commands:          97
  Target found:      yes
  Phase:             DONE
  Proof state:       FULL (mapped everything)
  Fast path nodes:   23
  Fast path time:    12.30 s
  Shortest distance: 240 cm

  Command stream (full mission): FFFRRLLRRLLRLFFFFFL...FFFFFFFRFFFFFRFFFFF
  Fast-run command stream:          FLRFFFFFFFRFFFFFRFFFFF
```

**No C code editing per maze.** Just point at a `.json` file.

---

## How to rebuild everything after code changes

```powershell
.\scripts\build_all.ps1
```

This builds and runs all unit tests (21 tests total):
- `test_graph.exe` (6 tests) -- nodes, edges, Dijkstra
- `test_robot.exe` (7 tests) -- heading, commands, frontiers
- `integration_test.exe` (8 tests) -- full mission on sample_maze

(Also builds `test_hal_compile.exe` for STM32 HAL verification, but doesn't auto-run it.)

---

## File layout

```
robot codes/
├── scripts/                     # Automation tools
│   ├── run_maze.py                Feed any .json → build → run → result
│   └── build_all.ps1              Rebuild + run all unit tests
├── inc/                         # Headers (8 files)
│   ├── maze_types.h               Structs, enums, MazeCommand
│   ├── maze_config.h              Memory limits, motion params
│   ├── maze_graph.h               Node/edge CRUD, Dijkstra
│   ├── maze_robot.h               Heading, F/L/R/B, frontiers
│   ├── maze_explore.h             P1→P2→P3 exploration
│   ├── maze_proof.h               Time-based early-stop proof
│   ├── maze_fastrun.h             Trapezoidal velocity, stop-graph
│   ├── maze_solver.h              TOP-LEVEL FSM
│   └── maze_hal.h                 STM32 hardware bridge
├── src/                         # Sources (6 files)
│   ├── maze_graph.c
│   ├── maze_robot.c
│   ├── maze_explore.c
│   ├── maze_proof.c
│   ├── maze_fastrun.c
│   └── maze_solver.c
├── test/                        # Test programs (5 files)
│   ├── run_maze.c                 Generic runner (reads _maze_data.h)
│   ├── test_graph.c               Unit: graph module (6 tests)
│   ├── test_robot.c               Unit: robot module (7 tests)
│   ├── integration_test.c         Full mission on sample_maze (8 tests)
│   └── test_hal_compile.c         HAL bridge smoke test (10 tests)
└── build/                       # Output .exe files (gitignored)
    ├── run_maze.exe
    ├── test_graph.exe
    ├── test_robot.exe
    ├── integration_test.exe
    └── test_hal_compile.exe
```

---

## Manual build commands (rarely needed)

### Test 1 -- Graph module

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o build/test_graph.exe
./build/test_graph.exe
```

### Test 2 -- Robot module

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe
./build/test_robot.exe
```

### Test 3 -- Integration (sample_maze)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o build/integration_test.exe
./build/integration_test.exe
```

### Test 4 -- HAL bridge

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_hal_compile.c -lm -o build/test_hal_compile.exe
./build/test_hal_compile.exe
```

---

## Pipeline (who calls who)

```
Your STM32 firmware (main.c)
         │
         ▼
    maze_hal.h          ◄-- hardware bridge (sensors, position, commands)
         │
         ▼
    maze_solver.c       ◄-- TOP-LEVEL: EXPLORE → RETURN_HOME → FAST_RUN → DONE
         │
    ┌────┼────┬─────────┐
    ▼    ▼    ▼         ▼
 robot  graph explore  proof  fastrun   ◄-- workers called by the solver
```

- **`maze_solver.c`** is the main brain -- it runs the state machine and delegates.
- **`maze_graph.c`** + **`maze_robot.c`** are pure logic (no hardware dependency).
- **`maze_explore.c`** + **`maze_proof.c`** + **`maze_fastrun.c`** are the three mission stages.
- **`maze_hal.h`** is header-only -- it reads STM32 globals and feeds the solver.

---

## Dependencies between modules

```
maze_types.h     ← everyone includes this
maze_config.h    ← everyone includes this

maze_graph.c     ← no dependencies (pure)
maze_robot.c     ← depends on maze_graph
maze_explore.c   ← depends on maze_graph, maze_robot, maze_proof
maze_proof.c     ← depends on maze_graph
maze_fastrun.c   ← depends on maze_graph
maze_solver.c    ← depends on ALL of the above
maze_hal.h       ← depends on maze_solver (bridges to hardware)
```

---

## On the STM32 (Keil / arm-none-eabi)

**Sources** (6 `.c` files): all from `src/`

**Include paths**: add `inc/`

**In `main.c`**:

```c
#define USE_MAZE_SOLVER
#include "maze_hal.h"

// At boot:
maze_hal_init();

// At every intersection:
MazeCommand cmd = maze_hal_tick();
cross = maze_cmd_to_cross(cmd);
// ... firmware's existing motion dispatch handles the rest
```

---

## Quick reference

| Command | What it does |
|---|---|
| `python scripts/run_maze.py <file.json>` | Test any maze (one command) |
| `.\scripts\build_all.ps1` | Rebuild + run all unit tests |
| `gcc --version` | Verify GCC (MSYS2 MinGW) |

**Flags:** `-std=c11 -Wall -Wextra -pedantic` for all; `-lm` for math (fast-run floats); `-Werror` for HAL test (must be zero-warning). Output goes to `build/` (gitignored). `test/_maze_data.h` is auto-generated and gitignored.
