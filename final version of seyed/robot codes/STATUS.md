# C Port Status — SEYED Maze Solver

**Goal:** Port the Python `maze_solver.py` algorithm to C for STM32G031G8Ux (Cortex-M0+, 8KB RAM, 64KB flash).

**Location:** `final version of seyed/robot codes/`

**Status: ALL CORE MODULES + HAL BRIDGE + DECISION CORE COMPLETE — 31 tests pass, zero warnings, plus the brain host test at 5/5 on five mazes.**

---

## ✅ DONE — Steps 1–10

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

### Step 10 — Decision Core
- `inc/brain.h` — the contract. `BrainIn` in (4 relative exits + target flag + `dist_cm`), one move out (`'F'/'L'/'R'/'B'`), then `brain_home_path()` / `brain_fast_path()` on `BRAIN_DONE`. **The robot owns everything physical; the brain never reads a sensor, motor, encoder or compass.**
- `src/brain.c` — dead reckoning (snap `dist_cm` to whole `MAZE_CELL_CM` cells), the command-granularity reduction, and the two plan strings. Home and fast routes still come from `maze_graph_shortest_path()` / `maze_fastrun_build_plan()` — not reimplemented.
- `test/brain_host.c` + `scripts/run_brain.py` — drives the brain from a **model of the robot**: it stops only where the firmware's junction test would fire and reports only what sensors can see. Unlike `run_maze.c`/`integration_test.c` it never calls `reveal_node()`, so the sensor-driven discovery path (and `PLACEHOLDER_DIST_CM`) is actually exercised.
- **Result: 5/5 checks on 5 mazes.** On `real_field` the brain drives through 11 cells without a report, and its fast-path time (6.50 s) **equals the time-optimal cost of the full maze** — a maze it was never allowed to see. That is the whole `proven_optimal` claim.
- Four design bugs found by the harness, all silently fatal — see the 2026-09-23 CHANGELOG entry. The most important: **a command is one STOP, not one graph edge**, and `MazeRobot.visited_nodes` is not a valid stop-point test.
- **Not yet wired into the firmware.** RAM is still 1808 B short (see `measure_solver_ram.sh`).

---

## Full test suite results

```
test_graph:          6 tests, 0 failed
test_robot:          7 tests, 0 failed
integration_test:    8 tests, 0 failed
test_hal_compile:   10 tests, 0 failed
-----------------------------
TOTAL:              31 tests, 0 failed, 0 warnings

brain_host:        5/5 checks on 5 mazes (real_field, sample_maze 1-4)
                   run via: python scripts/run_brain.py <maze.json>
                   NOT part of build_all.ps1 -- it needs a maze .json to
                   generate _maze_data.h, so run_brain.py drives it.
```

## Maze runner

```powershell
python scripts/run_maze.py ../simulator/mazes/sample_maze.json
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_brain.py ../simulator/mazes/real_field.json   # decision core (5/5 checks)
```

No C editing needed.  Takes any maze `.json` from the simulator.

---

## 🔲 REMAINING

### Step 11 — STM32 Integration

**Status: build/link DONE, RAM-blocked, on-target testing pending.**

The firmware project is `../firmware/` (copy of `New Start/code/t2/`), Keil project
`firmware/MDK-ARM/Source.uvprojx`. The solver sources are referenced **in place**
from `src/` with `inc/` on the include path, so the algorithm keeps one source of
truth shared with `run_maze.py`.

It **compiles and links with Keil's own ARM Compiler 5** — no IDE required:

```bash
bash scripts/build_firmware.sh          # -> build/firmware/seyed.hex
bash scripts/measure_solver_ram.sh      # -> does it fit in 8 KB?
```

Measured on the real toolchain:

| Build | Flash | RAM |
|---|---|---|
| Firmware as-is (solver dormant) | 34880 / 65536 (53%) | 7632 / 8192 (**93%**) |
| Firmware with solver **activated** | — | **does not link — `L6407E`, 1808 bytes too big** |

**Progress:** the solver config is now sized to the real field (64 nodes / 112
edges, was 80/160), which cut the shortfall **3208 → 1808 B**. Note that 1536 B
of the 7632 is the startup file's `Stack_Size` (1024) + `Heap_Size` (512), so
the *variables* only account for 6096 B.

**Remaining blocker.** The arrays cannot simply be deleted the way an earlier
plan assumed: `maze_hal.h`'s integration contract depends on `link[path_c][0]`
(rule 2, the solver's grid snap) and reads `node[path_c][0..1]` (position). The
genuinely removable legacy pieces are the path *strings* — `path_discoverd`,
`path_discoverd_s`, `result_1`, `result_2`, `path_back` ≈ **800 B**.

Two candidate directions (deliberately deferred — see CHANGELOG.md):

- **grow `node[]` and free elsewhere** — required for correctness regardless:
  `node[50]` is indexed by the command count (~70 on this field), which is what
  caused the overflow fixed in M0. Keeps the current HAL contract.
- **drop the legacy dead-reckoning entirely** — let the solver's own graph be
  the position source. Frees ~3400 B and removes the overflow at the root, but
  it changes the `maze_hal.h` contract, so it is M2 work.

Then: wire the call sites in `main.c`.  **The chosen seam is `brain_step()`
(`inc/brain.h`), not `maze_hal_tick()`** — the brain is a pure decision function
and the firmware keeps ownership of all motion and sensing, which is how the
bring-up mode was specified (print each decision over Bluetooth, pause ~5 s, then
execute).  The call site is inside `if (roatating == 0)`, replacing the whole
junction-*or*-dead-end block, **not** just the `if (right_poss || left_poss)`
branch.  Finally, calibrate the encoder.

---

### Milestone M0 — firmware builds and fits ⚠️ partially done

- ✅ **Config sized to the field** (`inc/maze_config.h`) — shortfall 3208 → 1808 B.
- ✅ **`node[]` / `link[]` overflow fixed** in `firmware/Core/Src/main.c` — this was
  a **pre-existing memory-corruption bug**, live in the M1 build: `node[50]`
  indexed by the command count wrote 504 B past the end, into `uwTick` and
  `hi2c2` (the IMU's I2C handle). Guards are unconditional (the bug is in the
  legacy firmware too) and cost +44 B flash / +0 B RAM. See CHANGELOG.md.
- ✅ **`build_firmware.sh` map fix** — it had been printing a `seyed.map` path
  that AC5 never creates (`--map` takes no argument; the map goes to stdout).
- 🔲 **Still 1808 B short of linking the solver** — the fork above.

---

### Milestone M1 — bench telemetry ✅ built, awaiting robot time

M1 exists because three numbers the solver needs cannot be derived from any file
in this repo — they only exist on the physical robot:

1. **counts per 20 cm cell**, which calibrates the entire map
2. **which sensors actually fire** at a junction, which validates `SENSORS.md`
3. **how long the branch detector stays live**, which is the decision's timing budget

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh   # -> build/firmware/seyed.hex
python scripts/parse_telemetry.py capture.txt    # -> the measurements
```

It **does not change how the robot drives** — the left-hand rule is untouched —
so it flashes today, before any of the solver RAM work above is done. That is
why M1 was deliberately pulled ahead of the rest of M0.

| Build | Flash | RAM |
|---|---|---|
| Firmware as-is (solver dormant) | 34880 / 65536 (53%) | 7632 / 8192 (93%) |
| Firmware + telemetry | 35580 / 65536 (54%) | 7704 / 8192 (**94%**) |

Telemetry costs **+72 B RAM / +700 B flash** over the plain build above, and with
the macro undefined it compiles to exactly the plain build's sizes
(Code=34232, ZI=6208).

**Flash the M1 hex only from a build that includes the `node[]` bounds guards
(2026-09-23 or later).** Earlier M1 hex had the overflow described under M0 below:
on a mission longer than 49 commands it wrote past `node[50]` into `uwTick` and
`hi2c2`, so a clean telemetry capture could not have been trusted. The guards add
+20 B flash to this build and no RAM.

**Status: built, compiles with the real ARMCC 5, links, wire format verified on
the host.** The *measurements themselves* need the robot; nothing in this repo
can produce them. See `BUILD_GUIDE.md` for the line format.

---

## File listing

```
robot codes/
├── scripts/
│   ├── run_maze.py            ✅  (Feed any .json → build → run → result)
│   ├── build_all.ps1           ✅  (Rebuild + run all unit tests)
│   ├── build_firmware.sh       ✅  (Build + link the STM32 firmware, ARMCC 5)
│   ├── measure_solver_ram.sh   ✅  (Measure the 8 KB RAM budget)
│   ├── field_to_maze.py        ✅  (Real field image → maze .json + overlay)
│   ├── parse_telemetry.py      ✅  (M1 capture → counts/cell, sensors, windows)
│   └── run_brain.py            ✅  (Brain host test — robot-model driver)
├── inc/
│   ├── maze_types.h            ✅
│   ├── maze_config.h           ✅
│   ├── maze_graph.h            ✅
│   ├── maze_robot.h            ✅
│   ├── maze_explore.h          ✅
│   ├── maze_proof.h            ✅
│   ├── maze_fastrun.h          ✅
│   ├── maze_solver.h           ✅
│   ├── maze_hal.h              ✅
│   └── brain.h                 ✅  (Decision-core contract)
├── src/
│   ├── maze_graph.c            ✅
│   ├── maze_robot.c            ✅
│   ├── maze_explore.c          ✅
│   ├── maze_proof.c            ✅
│   ├── maze_fastrun.c          ✅
│   ├── maze_solver.c           ✅
│   └── brain.c                 ✅  (Junction report → move; plans on DONE)
├── test/
│   ├── run_maze.c              ✅  (Generic runner — reads _maze_data.h)
│   ├── test_graph.c            ✅
│   ├── test_robot.c            ✅
│   ├── integration_test.c      ✅
│   ├── test_hal_compile.c      ✅
│   └── brain_host.c            ✅  (5/5 on 5 mazes — robot model, not a replayer)
├── SENSORS.md                  📋  Sensor map: s[i] ↔ silkscreen ↔ MUX ↔ role
├── BUILD_GUIDE.md              📋
├── PLAN_HAL.md                 📋
├── STATUS.md                   (this file)
```

## Build commands

All commands run from `robot codes/`.  Output goes to `build/` (gitignored).

See **[BUILD_GUIDE.md](./BUILD_GUIDE.md)** for the full guide.

### Quick: test any maze .json
```powershell
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_brain.py ../simulator/mazes/real_field.json   # decision core (5/5 checks)
```

### Quick: rebuild all after code changes
```powershell
.\scripts\build_all.ps1
```

### Manual (rarely needed)
```powershell
# Add GCC to PATH (once per session)
$env:PATH = "C:\msys64\mingw64\bin;$env:PATH"

gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o build/test_graph.exe
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o build/integration_test.exe
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/test_hal_compile.c -lm -o build/test_hal_compile.exe
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
