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

This builds 4 targets and runs 3 of them (21 tests total):
- `test_graph.exe` (6 tests) -- nodes, edges, Dijkstra
- `test_robot.exe` (7 tests) -- heading, commands, frontiers
- `integration_test.exe` (8 tests) -- full mission on sample_maze
- `brain.o` -- **compile-only** with `-Werror`. It cannot be linked here because
  `brain_host.c`, its only test, needs a generated maze header; but the object
  compile needs no maze data and keeps `brain.c` under the same zero-warning
  rule as the rest of the library.

`brain_host.c` is driven by `python scripts/run_brain.py <maze.json>` instead —
that script generates the header first. It reports **7/7 checks on 5 mazes**.

---

## Build the STM32 firmware (no Keil needed)

The firmware project lives at `../firmware/` and builds with Keil's own ARM
Compiler 5, which is installed at `C:\Keil_v5\ARM\ARMCC\bin`. You do **not** need
to open the Keil IDE to check that it compiles, links, and fits:

```bash
# The brain-driven firmware (what flashes today):
bash scripts/build_firmware.sh

# Supervised build: the same firmware plus telemetry AND a 5 s pause before
# every junction move.  This is the bring-up build -- see below.
USE_TELEMETRY=1 bash scripts/build_firmware.sh
```

There is no `USE_SOLVER` switch any more and no legacy fallback: the decision
core *is* the decision maker, unconditionally. The legacy left-hand-rule
explorer was deleted outright on 2026-09-23 (it still exists in git history).

Output goes to `build/firmware/` (gitignored): `.o` objects, `seyed.axf`,
**`seyed.hex`** (flashable), the link map, and the scatter file used.

It prints Keil's own size line and a budget check:

```
 Program Size: Code=39748 RO-data=... RW-data=... ZI-data=5872
 FLASH :  40820 /  65536 bytes  (62% used, 24716 free)
 RAM   :   6832 /   8192 bytes  (83% used, 1360 free)
```

### Does it still fit in RAM?

**RAM is the binding constraint on this project**, and it was the reason the
legacy explorer had to go: the firmware used to carry a *second, complete* maze
map, and two navigation stacks do not share 8 KB. Measure, don't assume:

```bash
bash scripts/measure_solver_ram.sh
```

It builds the firmware once and reports the totals from the linker, then the
**per-object RAM breakdown, largest first**:

```
== totals ==
 FLASH :  40820 /  65536 bytes  (62% used)
 RAM   :   6832 /   8192 bytes  (83% used, 1360 free)
 ...
 FITS -- 1360 bytes of RAM headroom
```

It **exits 1** if the build overflows, and also if it cannot read the linker's
totals at all — a silent zero there would make the fit check pass vacuously.

Note that part of the RAM total is the startup file's `Stack_Size` (1024) +
`Heap_Size` (512) reservation, and both are counted in the Keil size line. That
matters because the solver's heaviest stack frame (`maze_time_optimal_path()`,
five `MAZE_MAX_NODES`-sized arrays) is 704 B at 64 nodes against a 1024 B stack.

| Build | Flash | RAM |
|---|---|---|
| Legacy firmware, solver dormant (the old baseline) | 34880 / 65536 (53%) | 7632 / 8192 (93%) |
| **Brain-driven (now)** | **40820 / 65536 (62%)** | **6832 / 8192 (1360 free)** |
| Brain-driven + telemetry | 41792 / 65536 (63%) | 6904 / 8192 (1288 free) |

The brain-driven build is **800 bytes smaller than the old firmware was while
doing nothing.**

**Toolchain note:** this project uses **ARM Compiler 5 (`uAC6=0`), not armclang**.
The solver contains no C11-only constructs (verified under
`gcc -std=c99 -Wall -Wextra -pedantic -Werror`), but check AC5 specifically before
assuming a new construct is fine.

---

---

## M1 bench telemetry — and the supervised bring-up build

Three numbers the brain needs cannot be derived from any file in this repo; they
exist only on the physical robot:

1. **counts per 20 cm cell**, which calibrates the entire map
2. **which sensors actually fire** at a junction, which validates `SENSORS.md`
3. **how long the branch detector stays live**, which is the decision's timing budget

`USE_TELEMETRY=1` produces a build that measures all three — and it is also the
**supervised build**: it arms a 5 s pause between printing a decision and making
it. Press KEY1, and the robot stops at each junction for 5 s with the decision on
the Bluetooth link, so a bad decision can be caught before the robot commits. The
motors are explicitly stopped and the stop pushed to the servos before the pause,
because the pause blocks the superloop that would otherwise re-issue the drive
command.

It changes **what** the robot decides in no way at all — telemetry and the pause
are both guarded by the macro.

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh
# flash build/firmware/seyed.hex, capture the Bluetooth output to capture.txt:
python scripts/parse_telemetry.py capture.txt
```

| What it adds | Cost |
|---|---|
| `tlm_ms` 1 ms stamp, 64 B pending line, 8 ms divider | **+72 B RAM**, +~1 KB flash |
| RAM with telemetry on | 6904 / 8192 (1288 free) |
| RAM with it off | 6832 / 8192 (1360 free) |

It writes four line types to the Bluetooth link the firmware already uses, as
ASCII, at most one line per 8 ms slot:

```
S,<ms>,<front>,<rear>,<e0>,<e1>,<L>,<R>,<cross>   125 Hz, while driving
J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>,<target>,<dist>,<node>
Z,<ms>,<state>                                    target zone enter/leave
B,<ms>,<node>,<x>,<y>,<drift>,<move>              bring-up decision (SS.6)
```

`<front>` is `s[0..9]` as a hex bitmask and `<rear>` is `s[10..17]` — two
separate fields so the banks can never be confused. `<raw>` is the
**unconverted** summed encoder count for the link, which is the whole point:
the firmware divides it by `2.467*2` and then adds an empirical offset, and the
telemetry lets you check the divisor and the offsets separately instead of
inferring them from a final number.

> **The `J` line gained three fields and the `B` line is new (2026-09-23).**
> Captures made before that date will not parse — `parse_telemetry.py` now
> reports unparsed lines rather than silently skipping them, so an old capture
> shows up as noise instead of as a wrong measurement.

`<target>` is the target-zone flag, `<dist>` the distance the brain was told
(cells × 20 cm), and `<node>` the brain's own node id — the three things that
turn a capture into a check of the brain's *view* rather than just of the sensors.
The `B` line is the bring-up decision itself: the brain's dead-reckoned position,
its accumulated `drift`, and the move it chose — printed **before** the 5 s pause.

Two constraints are worth knowing before trusting the output:

- **One transmit per slot.** `BLT_SendData()` restarts the TX DMA
  (`Hardware.c:135`), so a second call before the first has drained truncates
  it. The firmware therefore queues a junction line and sends *either* that
  *or* a stream sample, never both. The `B` line is sent immediately before the
  5 s pause, where nothing else is competing for the link.
- **8 ms sampling against a ~20 ms branch window** gives only 2-3 samples across
  a junction. That is enough to say *which* sensors fire; it is not enough to
  measure the window to the millisecond, and `parse_telemetry.py` prints that
  caveat rather than hiding it. If a precise window is ever needed the fix is an
  on-chip ring buffer dumped after the run, not a faster radio link.

`parse_telemetry.py` reports: the measured counts-per-cell (which is what
actually calibrates the map), which sensors fire at each junction (which
validates `SENSORS.md`), the branch-window samples, the target-zone timing, the
brain's own view (`dist_cm` in cells, and a total-drift check), and the bring-up
decisions. It prints **`?? CORNER CANDIDATE`** when a junction has exactly one
lateral exit and a forward one — the case where `in.front` may be lying about
there being a forward path.

---

## File layout

```
robot codes/
├── scripts/                     # Automation tools
│   ├── run_maze.py                Feed any .json → build → run → result
│   ├── run_brain.py               Same, but drives the decision core (7/7 checks)
│   ├── build_all.ps1              Rebuild + run all unit tests
│   ├── build_firmware.sh          Build + link the STM32 firmware (ARMCC 5)
│   ├── measure_solver_ram.sh      Measure the 8 KB RAM budget, per object
│   ├── field_to_maze.py           Real field image → maze .json + overlay check
│   └── parse_telemetry.py         M1 capture → counts/cell, sensors, decisions
├── inc/                         # Headers (9 files)
│   ├── maze_types.h               Structs, enums, MazeCommand
│   ├── maze_config.h              Memory limits, motion params
│   ├── maze_graph.h               Node/edge CRUD, Dijkstra
│   ├── maze_robot.h               Heading, F/L/R/B, frontiers
│   ├── maze_explore.h             P1→P2→P3 exploration
│   ├── maze_proof.h               Time-based early-stop proof
│   ├── maze_fastrun.h             Trapezoidal velocity, stop-graph
│   ├── maze_solver.h              TOP-LEVEL FSM
│   └── brain.h                    THE SEAM: BrainIn in, one move out
├── src/                         # Sources (7 files)
│   ├── maze_graph.c
│   ├── maze_robot.c
│   ├── maze_explore.c
│   ├── maze_proof.c
│   ├── maze_fastrun.c
│   ├── maze_solver.c
│   └── brain.c                    Junction report → move; the two plans on DONE
├── test/                        # Test programs (5 files)
│   ├── run_maze.c                 Generic runner (reads _maze_data.h)
│   ├── test_graph.c               Unit: graph module (6 tests)
│   ├── test_robot.c               Unit: robot module (7 tests)
│   ├── integration_test.c         Full mission on sample_maze (8 tests)
│   └── brain_host.c               Decision core vs a robot model (7/7, 5 mazes)
└── build/                       # Output .exe files (gitignored)
    ├── run_maze.exe
    ├── test_graph.exe
    ├── test_robot.exe
    ├── integration_test.exe
    └── brain.o
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

### Test 4 -- Decision core (compile-only)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc -c src/brain.c -o build/brain.o
python scripts/run_brain.py ../simulator/mazes/real_field.json   # the real run
```

---

## Pipeline (who calls who)

```
Your STM32 firmware (main.c)
         │
         │  brain_report(): builds a BrainIn from the globals the firmware
         │  already had, calls brain_step(), gets one move back
         ▼
      brain.h            ◄-- THE SEAM.  4 relative exits + target + dist_cm in,
         │                   'F'/'L'/'R'/'B' or BRAIN_DONE out
         ▼
      brain.c            ◄-- dead reckoning + the two plan strings
         │
         ▼
    maze_solver.c        ◄-- TOP-LEVEL: EXPLORE → RETURN_HOME → FAST_RUN → DONE
         │
    ┌────┼────┬─────────┐
    ▼    ▼    ▼         ▼
 robot  graph explore  proof  fastrun   ◄-- workers called by the solver
```

- **`brain.c`** is what the firmware talks to. The robot owns everything
  physical; the brain owns the map.
- **`maze_solver.c`** runs the state machine and delegates.
- **`maze_graph.c`** + **`maze_robot.c`** are pure logic (no hardware dependency).
- **`maze_explore.c`** + **`maze_proof.c`** + **`maze_fastrun.c`** are the three mission stages.
- There is **no HAL**. `brain.h` never reads a sensor, motor, encoder or compass,
  and holds no firmware pointer — that is the whole point of the seam.

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
brain.c          ← depends on maze_solver (drives it; knows nothing of hardware)
```

---

## On the STM32 (Keil / ARMCC 5)

Already wired up in `../firmware/MDK-ARM/Source.uvprojx` (group `MazeSolver`,
plus `../../robot codes/inc` on the include path). To reproduce by hand:

**Sources** (7 `.c` files): all from `src/`

**Include paths**: add `inc/`

**In `main.c`**:

```c
#include "brain.h"

// On the KEY1 press (edge-detected, so holding the button cannot wipe the map):
brain_init();

// At every junction:
BrainIn in;
in.left = left_poss; in.right = right_poss;
in.front = (s[3] || s[4] || s[5] || s[6]);
in.back = 1; in.target = OnEndZoon; in.dist_cm = <the link just closed, cm>;

char move = brain_step(&in);      /* 'F'/'L'/'R'/'B', or BRAIN_DONE */
// ... the firmware's existing cross dispatch handles the motion
```

On `BRAIN_DONE`, `brain_home_path()` and `brain_fast_path()` supply the two
replay strings. `main.c`'s **`set_plan()`** copies them and appends the legacy
`'D'` end sentinel that the replay stages depend on, and **`replay_dispatch()`**
is the single command executor for both stages.

---

## Quick reference

| Command | What it does |
|---|---|
| `python scripts/run_maze.py <file.json>` | Test any maze (one command) |
| `python scripts/run_brain.py <file.json>` | Test the decision core against a simulated robot |
| `.\scripts\build_all.ps1` | Rebuild + run all unit tests |
| `bash scripts/build_firmware.sh` | Build + link the STM32 firmware → `seyed.hex` |
| `USE_TELEMETRY=1 bash scripts/build_firmware.sh` | **The supervised bring-up build:** +telemetry, +5 s pause per junction |
| `python scripts/parse_telemetry.py capture.txt` | Turn a capture into measurements + brain decisions |
| `bash scripts/measure_solver_ram.sh` | Check the 8 KB RAM budget (per object, exits 1 on overflow) |
| `python scripts/field_to_maze.py <field.png>` | Real field image → maze `.json` |
| `gcc --version` | Verify GCC (MSYS2 MinGW) |

**Flags:** `-std=c11 -Wall -Wextra -pedantic` for all; `-lm` for math (fast-run floats); `-Werror` on the `brain.c` compile (must be zero-warning). Output goes to `build/` (gitignored). `test/_maze_data.h` is auto-generated and gitignored.
