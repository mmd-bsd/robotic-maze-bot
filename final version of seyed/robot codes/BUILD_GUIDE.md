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

# Same, but drive the decision core (brain.c) instead of the raw solver.
# Models the real robot: stops only where the firmware's junction detector
# would, reports only what the sensors can see, then replays both plans on
# the true maze.  This is the one that validates the on-robot behaviour.
python scripts/run_brain.py ../simulator/mazes/real_field.json
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

## Build the STM32 firmware (no Keil needed)

The firmware project lives at `../firmware/` and builds with Keil's own ARM
Compiler 5, which is installed at `C:\Keil_v5\ARM\ARMCC\bin`. You do **not** need
to open the Keil IDE to check that it compiles, links, and fits:

```bash
# Firmware as it is today (left-hand rule):
bash scripts/build_firmware.sh

# Same, with USE_MAZE_SOLVER defined:
USE_SOLVER=1 bash scripts/build_firmware.sh

# M1 bench build: same left-hand firmware + sensor/encoder telemetry.
# This is the build you flash to calibrate the robot (see below).
USE_TELEMETRY=1 bash scripts/build_firmware.sh
```

Output goes to `build/firmware/` (gitignored): `.o` objects, `seyed.axf`,
**`seyed.hex`** (flashable), the link map, and the scatter file used.

It prints Keil's own size line and a budget check:

```
 Program Size: Code=34232 RO-data=612 RW-data=1424 ZI-data=6208
 FLASH :  34880 /  65536 bytes  (53% used, 30656 free)
 RAM   :   7632 /   8192 bytes  (93% used, 560 free)
```

### Does it still fit in RAM?

**RAM is the binding constraint on this project.** The firmware alone already uses
93% of the 8 KB, and the solver does not yet link alongside the legacy path/map
arrays. Measure before assuming:

```bash
bash scripts/measure_solver_ram.sh
```

It builds the firmware twice — once as-is, once with a generated patched copy of
`main.c` that activates the solver — and reports the real numbers:

```
 baseline RAM (solver dormant) :  7632 bytes
 shortfall to make it link     :  1808 bytes
```

(That shortfall was 3208 B before the solver's config was sized to the real
8x7 field; see `inc/maze_config.h` and CHANGELOG.md.)

Note that 1536 B of the 7632 is the startup file's `Stack_Size` (1024) +
`Heap_Size` (512) reservation, and both are counted in the Keil size line — so
the firmware's actual *variables* account for 6096 B. That matters because the
solver's heaviest stack frame (`maze_time_optimal_path()`, five
`MAZE_MAX_NODES`-sized arrays) is 704 B at 64 nodes against a 1024 B stack.

`Core/Src/main.c` is never modified; the patched copy is generated fresh each run
into `build/probe/`.

**Toolchain note:** this project uses **ARM Compiler 5 (`uAC6=0`), not armclang**.
The solver contains no C11-only constructs (verified under
`gcc -std=c99 -Wall -Wextra -pedantic -Werror`), but check AC5 specifically before
assuming a new construct is fine.

---

---

## M1 bench telemetry (encoder calibration)

Before the solver can map anything, three numbers have to come off the real
robot. `USE_TELEMETRY=1` produces a build that measures all three, and it does
**not** change how the robot drives — the left-hand rule is untouched, so this
build is safe to flash before any of the solver RAM work is done.

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh
# flash build/firmware/seyed.hex, capture the Bluetooth output to capture.txt:
python scripts/parse_telemetry.py capture.txt
```

| What it adds | Cost |
|---|---|
| `tlm_ms` 1 ms stamp, 64 B pending line, 8 ms divider | **+72 B RAM**, +700 B flash |
| RAM with telemetry on | 7704 / 8192 (**94%**) — fits today |
| RAM with it off | 7632 / 8192 (93%) — compiles to the plain build's sizes |

> **Flash this build only from 2026-09-23 or later.** Earlier telemetry hex
> carried a `node[]` overflow that corrupted `uwTick` and the IMU's `hi2c2`
> handle on any mission longer than 49 commands, so a capture from it could not
> be trusted. See the `node[]` entry in CHANGELOG.md.

It writes three line types to the Bluetooth link the firmware already uses, as
ASCII, at most one line per 8 ms slot:

```
S,<ms>,<front>,<rear>,<e0>,<e1>,<L>,<R>,<cross>   125 Hz, while driving
J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>
Z,<ms>,<state>                                    target zone enter/leave
```

`<front>` is `s[0..9]` as a hex bitmask and `<rear>` is `s[10..17]` — two
separate fields so the banks can never be confused. `<raw>` is the
**unconverted** summed encoder count for the link, which is the whole point:
the firmware divides it by `2.467*2` and then adds an empirical offset, and the
telemetry lets you check the divisor and the offsets separately instead of
inferring them from a final number.

Two constraints are worth knowing before trusting the output:

- **One transmit per slot.** `BLT_SendData()` restarts the TX DMA
  (`Hardware.c:135`), so a second call before the first has drained truncates
  it. The firmware therefore queues a junction line and sends *either* that
  *or* a stream sample, never both.
- **8 ms sampling against a ~20 ms branch window** gives only 2-3 samples across
  a junction. That is enough to say *which* sensors fire; it is not enough to
  measure the window to the millisecond, and `parse_telemetry.py` prints that
  caveat rather than hiding it. If a precise window is ever needed the fix is an
  on-chip ring buffer dumped after the run, not a faster radio link.

`parse_telemetry.py` reports the four things M1 exists to produce: the measured
counts-per-cell (which is what actually calibrates the map), which sensors fire
at each junction (which validates `SENSORS.md`), the branch-window samples, and
the target-zone timing.

---

## File layout

```
robot codes/
├── scripts/                     # Automation tools
│   ├── run_maze.py                Feed any .json → build → run → result
│   ├── build_all.ps1              Rebuild + run all unit tests
│   ├── build_firmware.sh          Build + link the STM32 firmware (ARMCC 5)
│   ├── measure_solver_ram.sh      Measure the 8 KB RAM budget
│   ├── field_to_maze.py           Real field image → maze .json + overlay check
│   └── parse_telemetry.py         M1 capture → counts/cell, sensors, windows
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

## On the STM32 (Keil / ARMCC 5)

Already wired up in `../firmware/MDK-ARM/Source.uvprojx` (group `MazeSolver`,
plus `../../robot codes/inc` on the include path). To reproduce by hand:

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
| `python scripts/run_brain.py <file.json>` | Test the decision core against a simulated robot |
| `.\scripts\build_all.ps1` | Rebuild + run all unit tests |
| `bash scripts/build_firmware.sh` | Build + link the STM32 firmware → `seyed.hex` |
| `USE_TELEMETRY=1 bash scripts/build_firmware.sh` | M1 bench build: +sensor/encoder telemetry |
| `python scripts/parse_telemetry.py capture.txt` | Turn an M1 capture into measurements |
| `bash scripts/measure_solver_ram.sh` | Check the 8 KB RAM budget |
| `gcc --version` | Verify GCC (MSYS2 MinGW) |

**Flags:** `-std=c11 -Wall -Wextra -pedantic` for all; `-lm` for math (fast-run floats); `-Werror` for HAL test (must be zero-warning). Output goes to `build/` (gitignored). `test/_maze_data.h` is auto-generated and gitignored.
