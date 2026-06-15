# BUILD GUIDE — SEYED C Maze Solver

**Location:** `final version of seyed/robot codes/`

---

## Prerequisites

- **GCC** 8+ (MSYS2 MinGW recommended)
  ```powershell
  # Add to PATH (once per PowerShell session)
  $env:PATH = "C:\msys64\mingw64\bin;$env:PATH"

  # Verify
  gcc --version
  ```
- All commands are run **from the `robot codes/` folder**:
  ```powershell
  cd "F:\Robotic fle 2022\SEYED\final version of seyed\robot codes"
  ```

---

## File layout

```
robot codes/
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
│   ├── test_graph.c               Unit: graph module (6 tests)
│   ├── test_robot.c               Unit: robot module (7 tests)
│   ├── integration_test.c         Full mission on 27-node maze (8 tests)
│   ├── test_maze4.c               Full mission on 37-node maze (8 tests)
│   └── test_hal_compile.c         HAL bridge smoke test (10 tests)
└── build/                       # Output .exe files (gitignored)
    ├── test_graph.exe
    ├── test_robot.exe
    ├── integration_test.exe
    ├── test_maze4.exe
    └── test_hal_compile.exe
```

---

## Build & run commands

### Run all tests (one command)

```powershell
# Build everything
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o build/test_graph.exe; gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe; gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o build/integration_test.exe; gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_maze4.c -lm -o build/test_maze4.exe; gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_hal_compile.c -lm -o build/test_hal_compile.exe; echo "=== ALL BUILDS DONE ==="

# Run all
./build/test_graph.exe; ./build/test_robot.exe; ./build/integration_test.exe; ./build/test_maze4.exe; ./build/test_hal_compile.exe
```

---

### Test 1 — Graph module (6 tests)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o build/test_graph.exe
./build/test_graph.exe
```

Verifies: node add/find, edge add/explore, neighbors, Dijkstra shortest-path by real distance.

---

### Test 2 — Robot module (7 tests)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe
./build/test_robot.exe
```

Verifies: heading tracking, F/L/R/B command generation, frontier detection, branch ranking (turn-minimizing), path following.

---

### Test 3 — Full mission integration (8 tests)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o build/integration_test.exe
./build/integration_test.exe
```

Verifies the **complete pipeline** on the 27-node sample maze with incremental discovery:

```
=== Integration Test - Full Mission on sample_maze ===
  Init solver at start (0,0)                           PASS
  Reveal start node and its edges                      PASS
  Full mission loop                                    (53 steps, 53 commands) PASS
  Mission completed all phases                         PASS
  Command log non-empty                                (53 cmds) PASS
  Fast path computed                                   (len=8, time=6.50s) PASS
  Proof state terminal                                 (FULL) PASS
  Start→target path exists                             (140 cm) PASS

  Command stream (full mission): FFFFFBLLBRFLLRFRFRFRFBRBRLLRLLRFLRRRRLLLBLRBRBFFFLRLF
  Fast-run command stream:          FFFLRLF

8 tests, 0 failed
```

This is the most important test — it proves the algorithm works before putting it on the robot.

---

### Test 4 — HAL bridge (10 tests)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_hal_compile.c -lm -o build/test_hal_compile.exe
./build/test_hal_compile.exe
```

Verifies: `maze_hal_init()` / `maze_hal_tick()` with stubbed firmware globals, branch discovery from sensors, target detection, `maze_cmd_to_cross()` mappings, direct motor control.

---

### Test 5 — Custom maze: sample_maze4 (8 tests)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_maze4.c -lm -o build/test_maze4.exe
./build/test_maze4.exe
```

Verifies the **complete pipeline** on `sample_maze4.json` (37 nodes, 37 edges, start at (-20,-20), target at (40,-120)):

```
=== Integration Test - sample_maze4 (37 nodes) ===
  Init solver at start (-20,-20)                       PASS
  Reveal start node and its edges                      PASS
  Full mission loop                                    (97 steps, 97 commands) PASS
  Mission completed all phases                         PASS
  Command log non-empty                                (97 cmds) PASS
  Fast path computed                                   (len=23, time=12.30s) PASS
  Proof state terminal                                 (FULL) PASS
  Start->target path exists                            (240 cm) PASS

  Command stream (full mission): FFFRRLLRRLLRLFFFFFLFFFFFLFFFFFFFLBRFFFRFBFRFFFRFFFFFRFFFRFBFRFRLRRLLRRLLFFRRLRFFFFFFFRFFFFFRFFFFF
  Fast-run command stream:          FLRFFFFFFFRFFFFFRFFFFF

8 tests, 0 failed
```

---

## Pipeline (who calls who)

```
Your STM32 firmware (main.c)
         │
         ▼
    maze_hal.h          ◄── hardware bridge (sensors, position, commands)
         │
         ▼
    maze_solver.c       ◄── TOP-LEVEL: EXPLORE → RETURN_HOME → FAST_RUN → DONE
         │
    ┌────┼────┬─────────┐
    ▼    ▼    ▼         ▼
 robot  graph explore  proof  fastrun   ◄── workers called by the solver
```

- **`maze_solver.c`** is the main brain — it runs the state machine and delegates to the others.
- **`maze_graph.c`** + **`maze_robot.c`** are pure logic (no hardware dependency).
- **`maze_explore.c`** + **`maze_proof.c`** + **`maze_fastrun.c`** are the three mission stages.
- **`maze_hal.h`** is header-only — it reads STM32 globals (`nav`, `head`, `s[]`, `node[][]`) and feeds them into the solver. No `.c` file needed.

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

Add these to your Keil project:

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

| Test binary | Tests | What it checks |
|---|---|---|
| `build/test_graph.exe` | 6 | Nodes, edges, Dijkstra |
| `build/test_robot.exe` | 7 | Heading, commands, frontiers |
| `build/integration_test.exe` | 8 | **Full mission** on 27-node maze |
| `build/test_maze4.exe` | 8 | **Full mission** on 37-node maze (sample_maze4) |
| `build/test_hal_compile.exe` | 10 | HAL bridge with stubbed firmware |

**Flags:** `-std=c11 -Wall -Wextra -pedantic` for all; `-lm` for math (fast-run floats); `-Werror` for HAL test (must be zero-warning).
