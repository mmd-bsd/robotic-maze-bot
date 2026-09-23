# C Port Status — SEYED Maze Solver

**Goal:** Port the Python `maze_solver.py` algorithm to C for STM32G031G8Ux (Cortex-M0+, 8KB RAM, 64KB flash).

**Location:** `final version of seyed/robot codes/`

**Status: THE BRAIN DRIVES THE ROBOT. The legacy left-hand-rule explorer has been
deleted outright from the firmware, the decision core is wired in, and the whole
thing fits in RAM with 1360 bytes to spare. 21 unit tests pass, zero warnings,
plus the brain host test at 7/7 on five mazes. On-target bring-up is the only
thing left.**

**The RAM blocker is gone.** The firmware used to carry a *second, complete* maze
implementation — the legacy explorer's own `link[100][4]` + `node[50][6]` map and
its command strings — and two navigation stacks do not share 8 KB. Deleting it and
putting the brain in its place came out **800 bytes smaller than the old firmware
was while doing nothing**:

| Build | Flash | RAM |
|---|---|---|
| Legacy firmware, solver dormant (the old baseline) | 34880 / 65536 | 7632 / 8192 (560 free) |
| **Brain-driven firmware (now)** | **40820 / 65536 (62%)** | **6832 / 8192 (1360 free)** |
| Brain + M1 telemetry (the supervised build) | 41792 / 65536 (63%) | 6904 / 8192 (1288 free) |

There is no `USE_MAZE_SOLVER` switch any more: the brain *is* the decision maker,
unconditionally. See `../CHANGELOG.md`, 2026-09-23.

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

### Step 9 — STM32 HAL Bridge — ❌ RETIRED (2026-09-23)
- `inc/maze_hal.h` and `test/test_hal_compile.c` are **deleted**, along with
  `PLAN_HAL.md`. The HAL seam assumed the solver would drive the robot through
  `maze_hal_tick()` while the legacy explorer kept its own map. That is not the
  design that shipped: the brain is a *pure decision function* and the firmware
  keeps all motion and sensing, so the seam is `brain_step()` inside `main.c`,
  not a HAL. Keeping both would have meant two integration points and two RAM
  budgets for one job.
- The solver fix that came out of it is kept: `maze_solver_update_position()` runs
  the sensor-driven discovery for ALL nodes (not just new ones) via
  `_discover_branches()`, creating placeholder nodes at `PLACEHOLDER_DIST_CM`.

### Step 10 — Decision Core
- `inc/brain.h` — the contract. `BrainIn` in (4 relative exits + target flag + `dist_cm`), one move out (`'F'/'L'/'R'/'B'`), then `brain_home_path()` / `brain_fast_path()` on `BRAIN_DONE`. **The robot owns everything physical; the brain never reads a sensor, motor, encoder or compass.**
- `src/brain.c` — dead reckoning (snap `dist_cm` to whole `MAZE_CELL_CM` cells), the command-granularity reduction, and the two plan strings. Home and fast routes still come from `maze_graph_shortest_path()` / `maze_fastrun_build_plan()` — not reimplemented.
- `test/brain_host.c` + `scripts/run_brain.py` — drives the brain from a **model of the robot**: it stops only where the firmware's junction test would fire and reports only what sensors can see. Unlike `run_maze.c`/`integration_test.c` it never calls `reveal_node()`, so the sensor-driven discovery path (and `PLACEHOLDER_DIST_CM`) is actually exercised.
- **Result: 7/7 checks on 5 mazes.** On `real_field` the brain drives through 11 cells without a report, and its fast-path time (6.50 s) **equals the time-optimal cost of the full maze** — a maze it was never allowed to see. That is the whole `proven_optimal` claim.
- Four design bugs found by the harness, all silently fatal — see the 2026-09-23 CHANGELOG entry. The most important: **a command is one STOP, not one graph edge**, and `MazeRobot.visited_nodes` is not a valid stop-point test.

### Step 11 — Firmware integration — ✅ wired, 🔲 awaiting on-target bring-up
- The legacy explorer is **deleted** from `firmware/Core/Src/main.c`: `link[][]`,
  `node[][]`, `local_cross[]`, `fill_map()`, `short_path()`, `replaceWord()`
  (which held the firmware's only `malloc`), `back_home()`, `path_discoverd[]`,
  `result_0/1/2`. About 3.2 KB freed.
- The seam is `brain_report()`: it builds a `BrainIn` from the globals the
  firmware already had, calls `brain_step()`, and returns the move — which then
  goes through the *unchanged* legacy `cross` dispatch to the motors. `brain_init()`
  runs on the KEY1 press (edge-detected, so holding the button cannot wipe the map).
- `set_plan()` copies the brain's two plans into `path_back` / `path_discoverd_s`
  and appends the legacy `'D'` end sentinel, which stages 4 and 6 depend on twice.
- `replay_dispatch()` is now the single command executor for both replay stages.
  It adds **`'F'`** (the brain's straight) and **`'B'`** — the legacy stages had no
  `'B'` case at all, and the real field's fast path is `BFFLRLF`, so the robot
  would have stalled mid-mission.
- **Two ordering bugs in the legacy replay stages, both fixed** — see the CHANGELOG
  entry; they are not obvious and they are not cosmetic.
- **`brain.c` was missing from the Keil project.** `Source.uvprojx`'s `MazeSolver`
  group listed the six solver sources but not `brain.c`, so the IDE build — the
  way the firmware is actually flashed — would have failed to link with
  `Undefined symbol brain_step`, even though `build_firmware.sh` succeeded (the
  script adds `brain` by name). Fixed in `.uvprojx` and `.uvoptx`.
- With `USE_MAZE_TELEMETRY` defined, every junction is printed over Bluetooth
  followed by a **5 s pause before the move is made**, so an operator can stop a
  bad run before the robot commits. The motors are explicitly stopped and the stop
  pushed to the servos first, because the pause blocks the superloop that would
  otherwise re-issue the drive command.

---

## Full test suite results

```
test_graph:          6 tests, 0 failed
test_robot:          7 tests, 0 failed
integration_test:    8 tests, 0 failed
brain:               compile-only, -Werror, 0 warnings
-----------------------------
TOTAL:              21 tests, 0 failed, 0 warnings

brain_host:        7/7 checks on 5 mazes (real_field, sample_maze 1-4)
                   run via: python scripts/run_brain.py <maze.json>
                   NOT part of build_all.ps1 -- it needs a maze .json to
                   generate _maze_data.h, so run_brain.py drives it.
                   build_all.ps1 compiles brain.c alone (-Werror) so the
                   warning guarantee still covers it.
```

**31 → 21 is not lost coverage.** The 10 tests that went were `test_hal_compile`'s,
and they tested `maze_hal.h` — a seam that no longer exists. They are in git
history if the HAL approach is ever revived.

## Maze runner

```powershell
python scripts/run_maze.py ../simulator/mazes/sample_maze.json
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_brain.py ../simulator/mazes/real_field.json   # decision core (7/7 checks)
```

No C editing needed.  Takes any maze `.json` from the simulator.

---

## 🔲 REMAINING

### Bring-up on the robot — the only step left

Everything builds, links, fits and passes on the host. What no test in this repo
can do is check the brain's *model* against the physical robot: `brain_host.c`
drives the brain from a model of the robot, so a wrong model is invisible to it.

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh   # the SUPERVISED build
bash scripts/measure_solver_ram.sh               # the RAM budget, per object
python scripts/parse_telemetry.py capture.txt    # read the Bluetooth log
```

Flash the telemetry hex, press KEY1, and read §2 and §6 of the report. The three
things to look at, in order of how much they matter:

1. **`in.front` at a corner.** `(s[3]||s[4]||s[5]||s[6])` is the firmware's own
   proxy and the ONE input never verified against hardware. At a corner the
   robot's own incoming arm may still be under the centre group, so it would read
   "forward open" where there is no forward — and because `Forward()` is the line
   follower, not "drive straight", the robot would steer round the corner while
   the brain believed it went straight. `parse_telemetry.py` prints
   `?? CORNER CANDIDATE` for exactly this. **If it fires, the fix is that one
   expression in `brain_report()`** — not in the brain.
2. **Drift.** `dist_cm` should snap to whole 20 cm cells with a small residue that
   does not grow. A climbing drift means the counts-per-cell constant is wrong and
   the time-optimal planner is reasoning over a distorted map.
3. **That the executed move is the printed one,** and that the 5 s pause holds.

Then calibrate the encoder and re-run M1's three measurements.

---

### Milestone M0 — firmware builds and fits ✅ DONE

- ✅ **Config sized to the field** (`inc/maze_config.h`) — shortfall 3208 → 1808 B.
- ✅ **`node[]` / `link[]` overflow fixed** in `firmware/Core/Src/main.c` — this was
  a **pre-existing memory-corruption bug**, live in the M1 build: `node[50]`
  indexed by the command count wrote 504 B past the end, into `uwTick` and
  `hi2c2` (the IMU's I2C handle). Moot now (both arrays are deleted), but the
  finding stands in CHANGELOG.md.
- ✅ **`build_firmware.sh` map fix** — it had been printing a `seyed.map` path
  that AC5 never creates (`--map` takes no argument; the map goes to stdout).
- ✅ **The RAM blocker is gone** — the legacy map is deleted and the brain fits
  with 1360 B to spare. **`measure_solver_ram.sh` now reports FITS.**
- ✅ **`build_firmware.sh` size reporting fixed** — the `fromelf` parse matched a
  `(uncompressed)` row that this build never emits, so `TOTAL_RAM` came out **0**
  and the fit check silently passed. It now reads the linker's own totals and
  **exits 1 if it cannot**.

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

The telemetry build is now also **the supervised build**: `USE_TELEMETRY`
additionally arms the 5 s bring-up pause before each junction move. It costs
+72 B RAM / +~1 KB flash over the plain build, and with the macro undefined the
path costs exactly nothing.

| Build | Flash | RAM |
|---|---|---|
| Brain-driven (plain) | 40820 / 65536 (62%) | 6832 / 8192 (1360 free) |
| Brain-driven + telemetry | 41792 / 65536 (63%) | 6904 / 8192 (1288 free) |

**The `J` line gained three fields** (`target`, `dist_cm`, `node`) and a new
**`B` line** was added for the bring-up decision, so captures from a build
before 2026-09-23 will not parse. `parse_telemetry.py` reports unparsed lines
rather than silently skipping them.

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
│   └── brain.h                 ✅  (Decision-core contract — the seam)
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
│   └── brain_host.c            ✅  (7/7 on 5 mazes — robot model, not a replayer)
├── SENSORS.md                  📋  Sensor map: s[i] ↔ silkscreen ↔ MUX ↔ role
├── BUILD_GUIDE.md              📋
├── STATUS.md                   (this file)
```

## Build commands

All commands run from `robot codes/`.  Output goes to `build/` (gitignored).

See **[BUILD_GUIDE.md](./BUILD_GUIDE.md)** for the full guide.

### Quick: test any maze .json
```powershell
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_brain.py ../simulator/mazes/real_field.json   # decision core (7/7 checks)
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
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc -c src/brain.c -o build/brain.o   # compile-only; its test needs a maze header
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
- **The seam is `brain_step()`, not a HAL.** The brain is a pure decision function — `BrainIn` in, one `'F'/'L'/'R'/'B'` out — and the firmware keeps all sensing and all motion. It never reads a sensor, motor, encoder or compass, and it holds no firmware pointer. `maze_hal.h` proposed the opposite (the solver driving the robot through `maze_hal_tick()` while the legacy explorer kept its own map) and is deleted; two integration points for one job meant two RAM budgets. See `inc/brain.h`.
- **The brain owns the map, the firmware owns the robot.** `maze_hal.h`'s HAL used to declare the firmware's `link[][]` / `node[][]` as `extern` so the solver could read them. Those arrays are gone; nothing in the library reaches into the firmware now.

## Key reference files

| File | Why |
|---|---|
| `../ARCHITECTURE.md` | Project rules, conventions, glossary |
| `../ALGORITHMS.md` | Every algorithm in detail (§1-§7) |
| `../../New Start/code/Core/Src/main.c` | Working robot firmware (read-only reference) |
| `../../New Start/Simulator/py-code/maze solving/maze_gbf/stm-sample-code/main.c` | An earlier `USE_MAZE_GBF` integration via a `maze_hal.h` (read-only history — superseded by `inc/brain.h`) |
| `inc/brain.h` | **The integration contract.** Read this before touching `firmware/Core/Src/main.c` |
| `../../firmware/Core/Src/main.c` | The port target: `brain_report()` builds the `BrainIn`, `replay_dispatch()` executes the plans |
