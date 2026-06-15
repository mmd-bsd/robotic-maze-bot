# C Port Status — SEYED Maze Solver

**Goal:** Port the Python `maze_solver.py` algorithm to C for STM32G031G8Ux (Cortex-M0+, 8KB RAM, 64KB flash).

**Location:** `final version of seyed/robot codes/`

**Status: ALL CORE MODULES + HAL BRIDGE COMPLETE — 31 tests pass, zero warnings.**

---

## ✅ DONE — Steps 1–9

### Step 1 — Types & Config
- `inc/maze_types.h` — All structs, enums, error codes
- `inc/maze_config.h` — Fixed-point motion model, memory limits, debug

### Step 2 — Graph Module
- `inc/maze_graph.h`, `src/maze_graph.c` — Binary heap Dijkstra on known map, node/edge CRUD, integer sqrt
- `test/test_graph.c` — **6 tests pass**

### Step 3 — Robot Module
- `inc/maze_robot.h`, `src/maze_robot.c` — Heading tracking, F/L/R/B command generation (cross-product, ALGORITHMS.md §5), frontier detection, branch ranking (turn-minimizing)
- `test/test_robot.c` — **7 tests pass**

### Step 4 — Exploration Module
- `inc/maze_explore.h`, `src/maze_explore.c` — P1→P2→P3 priority system, target-agnostic, hooks into proof module for frontier filtering

### Step 5 — Proof Module
- `inc/maze_proof.h`, `src/maze_proof.c` — Time-based admissible lower bound, frontier filtering, proof state determination (PROVEN / FULL / DISABLED)

### Step 6 — Fast-Run Module
- `inc/maze_fastrun.h`, `src/maze_fastrun.c` — Trapezoidal velocity profile (`run_time`, `speed_at`), time-optimal path (stop-graph Dijkstra), fast-run plan builder

### Step 7 — Top-Level Solver FSM
- `inc/maze_solver.h`, `src/maze_solver.c` — EXPLORE→RETURN_HOME→FAST_RUN→DONE state machine, position update (node discovery)

### Step 8 — Integration Test
- `test/integration_test.c` — Full mission on 27-node sample maze with incremental discovery. **8 tests pass**, 53 steps, command stream: `FFFFFBLLBRFLLRFRFRFRFBRBRLLRLLRFLRRRRLLLBLRBRBFFFLRLF`

### Step 9 — STM32 HAL Bridge
- `inc/maze_hal.h` — Sensor reading (head-aware), position from firmware, `maze_hal_tick()` / `maze_hal_init()`, direct motor control fallback, BLT debug output
- `test/test_hal_compile.c` — **10 tests pass** — compile+smoke with stub firmware globals, 3-node L-maze mission
- **Solver fix:** `maze_solver_update_position()` now processes sensor data for ALL nodes (not just new ones) via `_discover_branches()` — creates placeholder nodes + unexplored edges for detected open paths. Fixes start-node branch discovery on real hardware.

---

## Full test suite results

```
test_graph:          6 tests, 0 failed
test_robot:          7 tests, 0 failed
integration_test:    8 tests, 0 failed
test_hal_compile:   10 tests, 0 failed
─────────────────────────────
TOTAL:              31 tests, 0 failed, 0 warnings
```

---

## 🔲 REMAINING

### Step 10 — STM32 Cross-Compile & Integration
Cross-compile with `arm-none-eabi-gcc`, integrate into `New Start/code/` as a `USE_MAZE_SOLVER` feature alongside existing firmware motion primitives.  The HAL bridge (`maze_hal.h`) is ready; this step is about toolchain setup, linking, and on-target testing.

---

## File listing

```
robot codes/
├── inc/
│   ├── maze_types.h       ✅
│   ├── maze_config.h      ✅
│   ├── maze_graph.h       ✅
│   ├── maze_robot.h       ✅
│   ├── maze_explore.h     ✅
│   ├── maze_proof.h       ✅
│   ├── maze_fastrun.h     ✅
│   ├── maze_solver.h      ✅
│   └── maze_hal.h         ✅  (new — Step 9)
├── src/
│   ├── maze_graph.c       ✅
│   ├── maze_robot.c       ✅
│   ├── maze_explore.c     ✅
│   ├── maze_proof.c       ✅
│   ├── maze_fastrun.c     ✅
│   └── maze_solver.c      ✅  (updated — branch discovery fix)
├── test/
│   ├── test_graph.c       ✅
│   ├── test_robot.c       ✅
│   ├── integration_test.c ✅
│   └── test_hal_compile.c ✅  (new — Step 9)
├── PLAN_HAL.md            📋  (implementation plan)
├── STATUS.md              (this file)
```

## Build commands

```powershell
# Build and run all tests
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o test/test_graph.exe
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o test/test_robot.exe
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o test/integration_test.exe
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_hal_compile.c -lm -o test/test_hal_compile.exe

./test/test_graph.exe && ./test/test_robot.exe && ./test/integration_test.exe && ./test/test_hal_compile.exe
```

## Design decisions (don't change without asking)

- All arrays **statically sized** — no `malloc` on target
- Motion model params are **fixed-point ×100** input, but time math uses `float` (proof/fast-run called infrequently; can optimize to integer later)
- `MazeHeading` matches firmware `nav` (0=N, 1=W, 2=S, 3=E)
- `MazeCommand` = 'F'/'L'/'R'/'B' chars; `maze_cmd_to_cross()` converts to firmware `cross` (0/1/2/4)
- Dijkstra on **known map only** (explored edges), by **real distance** (cm), not hop count
- Y is up (matches Python simulator and firmware convention)
- Time-optimal path uses **stop-graph Dijkstra** — from each node, walk straight in each direction over explored edges; every reached node is a candidate with time = run_time(total_distance)
- **Sensor branch discovery** in `maze_solver_update_position()` now runs for ALL nodes (not just new ones), creating placeholder neighbor nodes at 20 cm offset for each detected open path.  Back is skipped (the return edge is created when the robot physically drives between nodes).  Heading=NONE defaults to NORTH for the first sensor reading.
- **HAL bridge is header-only** (`static inline`), owns `MazeGraph` + `MazeRobot` as static globals.  Firmware globals are declared `extern` (same pattern as existing `maze_hal.h`).  Sensors are head-aware (front bank vs rear bank).

## Key reference files

| File | Why |
|---|---|
| `../ARCHITECTURE.md` | Project rules, conventions, glossary |
| `../ALGORITHMS.md` | Every algorithm in detail (§1-§7) |
| `../../New Start/code/Core/Src/main.c` | Working robot firmware (read-only reference) |
| `../../New Start/Simulator/py-code/maze solving/maze_gbf/stm-sample-code/maze_hal.h` | Existing HAL bridge pattern (read-only reference) |
| `inc/maze_hal.h` | New HAL bridge for the current solver (this project) |
