# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> This root file is the canonical one. The older partial copy at
> `final version of seyed/CLAUDE.md` was deleted on 2026-09-23 (it carried the
> wrong shell and a wrong project scope) — this is the only one now.

---

## Project

**SEYED** — a line-following maze robot. The robot drives along a 90° grid maze, searches for a target (big black area), proves it found the fastest route, returns to start, then races the fastest path. The fastest path minimises **time** (not distance) under an acceleration model — the robot stops at every turn and accelerates on clear straights.

**Three parts:**
1. **Python/Tkinter simulator** (`final version of seyed/simulator/`) — active
2. **C maze-solving library** (`final version of seyed/robot codes/`) — active C port, host-testable
3. **STM32 firmware** (`final version of seyed/firmware/`) — runs on STM32G031G8Ux (Cortex-M0+, 8KB RAM, 64KB flash)

**Canonical docs** (read these first in any session):
- [`final version of seyed/ARCHITECTURE.md`](./final%20version%20of%20seyed/ARCHITECTURE.md) — layout, rules, conventions, glossary; §5 is the hard-won rules list
- [`final version of seyed/ALGORITHMS.md`](./final%20version%20of%20seyed/ALGORITHMS.md) — every algorithm in detail (§0 mental model … §7 complexity)
- [`final version of seyed/CHANGELOG.md`](./final%20version%20of%20seyed/CHANGELOG.md) — worklog + hard-won lessons; **update it whenever you change code**
- [`final version of seyed/robot codes/BUILD_GUIDE.md`](./final%20version%20of%20seyed/robot%20codes/BUILD_GUIDE.md) — full C build reference
- [`final version of seyed/robot codes/STATUS.md`](./final%20version%20of%20seyed/robot%20codes/STATUS.md) — module status + **"Design decisions (don't change without asking)"**

---

## How to run

### Simulator (Python)

```bash
python "final version of seyed/simulator/maze solver/maze_solver.py"
python "final version of seyed/simulator/maze generator/maze_generator.py"
```

**Dependencies:** Python 3.x with tkinter (stdlib). Pillow (`pip install pillow`) optional — anti-aliased circles fall back gracefully. Both apps are single-file; the solver auto-loads `simulator/mazes/sample_maze.json` on startup if present.

### C Solver (testing)

```bash
cd "final version of seyed/robot codes"

# Test any maze .json from the simulator (generates a temp header, compiles, runs, cleans up):
python scripts/run_maze.py ../simulator/mazes/sample_maze.json

# Rebuild + run all unit tests after code changes:
powershell -NoProfile -ExecutionPolicy Bypass -File ./scripts/build_all.ps1
```

**Toolchain:** verified working here — GCC 14.2.0 (MSYS2 MinGW, `/c/msys64/mingw64/bin/gcc`) and Python 3.14 (`/c/Python314/python`). If `gcc` is not found, add it: `export PATH="/c/msys64/mingw64/bin:$PATH"`.

**Which tests actually run:** `build_all.ps1` builds 4 targets and runs 3 — `test_graph` (6) + `test_robot` (7) + `integration_test` (8) = **21 tests**. The fourth is `brain.o`, a **compile-only** `-Werror` build: it cannot be linked there because `brain_host.c` (its only test) needs a generated maze header, and `scripts/run_brain.py` drives that instead — **7/7 checks on 5 mazes**.

All C flags: `-std=c11 -Wall -Wextra -pedantic`, plus `-lm` (fast-run float math) and `-Werror` for the `brain.c` compile (must stay zero-warning). Output goes to `build/`; `test/_maze_data.h` is generated per-run — both gitignored.

### Firmware (STM32, ARMCC 5)

```bash
cd "final version of seyed/robot codes"
bash scripts/build_firmware.sh                  # build + link, prints the Keil size line
USE_TELEMETRY=1 bash scripts/build_firmware.sh  # the supervised bring-up build
bash scripts/measure_solver_ram.sh              # the 8 KB RAM budget, per object
```

Uses Keil's own **ARM Compiler 5** (`uAC6=0`, `C:\Keil_v5\ARM\ARMCC\bin`) — **not** armclang. The brain-driven build is **40820 / 65536 flash, 6832 / 8192 RAM (1360 B free)**; run `measure_solver_ram.sh` before claiming anything fits.

### Shell note

Claude Code's Bash tool runs **Git Bash (POSIX sh)**, not PowerShell — even though the repo docs and CHANGELOG talk about PowerShell. In this tool use bash syntax: `export PATH=...` (not `$env:PATH`), quote paths with spaces. `.ps1` scripts must be invoked via `powershell -NoProfile -ExecutionPolicy Bypass -File <script>`. The `;` chaining and path quoting advice from the docs applies to both.

---

## Key rules

1. **No cheating during search** — the robot must not use the target's position until `target_found` is True.
2. **All bounds/pruning use TIME**, not distance (acceleration model — `v_max` matters).
3. **Work only in `final version of seyed/`.** `New Start/` and `old docs/` are read-only history. (This root `CLAUDE.md` is the one exception — it documents the whole repo.)
4. **After any code change**, append an entry to `final version of seyed/CHANGELOG.md`. Keep `ARCHITECTURE.md`/`ALGORITHMS.md` in sync.
5. **Units are real-world:** 1 world unit = 1 cm. Speeds cm/s, accel cm/s².
6. **Prefer standard library**, single-file scripts. No `networkx`/`matplotlib` in the final-version simulator.
7. **Dark theme** — reuse the color constants at the top of each file.
8. **Console encoding is cp1252**: printing unicode (`✓`, `→`) crashes scripts. Call `sys.stdout.reconfigure(encoding='utf-8')` first.

---

## Architecture overview

### Simulator (Python/Tkinter)

Two standalone apps sharing a JSON maze format. `maze_solver.py` is ~1750 lines, `maze_generator.py` ~600.

- **`maze_generator.py`** — Click-to-build maze editor. Edges are forced horizontal/vertical (90° maze). Save/load JSON, set start/target, zoom/pan.
- **`maze_solver.py`** — Full mission simulation. Phases are **`EXPLORE → FAST_RUN → DONE`** — return-to-start is *not* a separate phase in the Python solver, it's the tail of EXPLORE. Features: time-optimal pathfinding, acceleration-aware motion, early-stop proof, command logging, speedometer, playback control.

**Maze JSON format:**
```json
{
  "grid_size": 20,
  "nodes": { "0": [0, 0], "1": [0, 20] },
  "edges": [[0, 1]],
  "start": 0,
  "target": 1
}
```
`nodes` maps string id → `[x, y]` world coords (**Y is up**). Edges are axis-aligned only. Node ids are assigned on save sorted by position, so they are stable for a given drawing. Four sample mazes ship in `simulator/mazes/` (`sample_maze` … `sample_maze4`).

### C Solver Library (`final version of seyed/robot codes/`)

Portable C maze solver for STM32G031G8Ux — 9 headers, 7 sources, 21 tests, zero warnings.

```
robot codes/
├── inc/          # 9 headers (types, config, 6 modules, brain.h)
├── src/          # 7 implementations (graph, robot, explore, proof, fastrun, solver, brain)
├── test/         # 4 test programs + run_maze.c generic runner
├── scripts/      # run_maze.py, run_brain.py, build_all.ps1, build_firmware.sh, ...
└── build/        # compiled output (gitignored)
```

**Module pipeline:** `brain.h` (the seam — `BrainIn` in, one move out; no hardware access, no `extern` firmware globals) → `brain.c` (dead reckoning + the two plan strings) → `maze_solver.c` (top-level FSM, **`EXPLORE → RETURN_HOME → FAST_RUN → DONE`** — note this *is* 4 phases here, unlike the Python) → the workers `maze_graph.c`, `maze_robot.c`, `maze_explore.c`, `maze_proof.c`, `maze_fastrun.c`. `maze_graph.c` and `maze_robot.c` are pure logic with no hardware dependency.

**There is no HAL.** The old `maze_hal.h` assumed the solver would drive the robot through `maze_hal_tick()` while the legacy explorer kept its own map — two integration points and two RAM budgets for one job. It was deleted on 2026-09-23 along with the legacy explorer. The firmware keeps all sensing and motion; the brain is a pure decision function.

**`maze_config.h` is the single tuning point** — memory limits, motion params, algorithm thresholds, with compile-time `#error` validation. Don't scatter new constants into the modules.

**C constraints that bind any change:**
- All arrays are **statically sized, no `malloc`** (`MAZE_MAX_NODES=64`, `MAZE_MAX_EDGES=112`). Adding a struct field is a RAM-budget decision — the whole firmware has 1360 B free. Note `brain.c` keeps its own `s_home[193]` / `s_fast[193]`, which `main.c` then copies into `path_back` / `path_discoverd_s`; pointing them at caller-supplied buffers would free 386 B if the margin ever gets tight.
- Motion params are **fixed-point ×100** (`MAZE_V_MAX_FP`, `MAZE_ACCEL_FP`), but time math uses `float` (proof/fast-run run infrequently).
- `MAZE_V_MAX_FP` must stay in sync with the value the early-stop proof uses, or the proof becomes inadmissible.
- `MazeHeading` deliberately matches firmware `nav` (0=N, 1=W, 2=S, 3=E). `MazeCommand` is the char `'F'/'L'/'R'/'B'`; `maze_cmd_to_cross()` maps it to firmware `cross` (0/1/2/4). Don't "fix" these to a cleaner ordering — they mirror hardware.
- The fast-run path uses **stop-graph Dijkstra**: from each node walk straight in each direction over explored edges; every reached node is a candidate with `time = run_time(total_distance)`.

---

## Algorithm: frontier-based exploration

The robot explores using three priorities:

1. **Priority 1** — Explore an unexplored branch from the current node (target-agnostic turn preference: straight → left → right → reverse)
2. **Priority 2** — Drive to the nearest *useful* frontier (visited node with unexplored edges), measured by **real travel distance** via Dijkstra on the known map (not hop count)
3. **Priority 3** — Return to start (no useful frontiers remain)

**Early-stop proof:** After finding the target, frontiers are pruned using a **time-based admissible bound**: `LB_time(f) = (dist_known(start→f) + ‖f→target‖) / v_max`. A frontier is only useful if `LB_time(f) < best_known_TIME`. This ensures a longer-but-straighter undiscovered path is never wrongly pruned.

The fastest path is computed **on the discovered map only** — never the full maze — so early-stopping stays honest.

**See `ALGORITHMS.md` §1-§2 for the full algorithm, §5 for command generation, §6 for the acceleration model.**

---

## Headless testing pattern (simulator)

`"maze solver"` has a space, so it is **not importable as a package** — `importlib.import_module("maze solver.maze_solver")` raises `ModuleNotFoundError`. Add the directory to `sys.path` and import the module by its bare name (verified working):

```python
import sys, importlib, tkinter as tk
sys.stdout.reconfigure(encoding='utf-8')   # console is cp1252
sys.path.insert(0, "maze solver")
m = importlib.import_module("maze_solver")
root = tk.Tk(); root.withdraw()
app = m.SolverApp(root)
frames = 0
while app.phase != "DONE" and frames < 60000:   # always bound the loop
    app._frame(0.033)                           # dt in sim-seconds
    frames += 1
```

Runs the full mission without `mainloop()` — no display needed. `sample_maze.json` completes in ~1100 frames. `app.phase` goes `EXPLORE → FAST_RUN → DONE`.

---

## STM32 integration (C solver)

**The brain is already wired in** — `final version of seyed/firmware/Core/Src/main.c`. There is no `USE_MAZE_SOLVER` switch and no legacy fallback; the decision core *is* the decision maker.

```c
#include "brain.h"

brain_init();                 // on the KEY1 press, edge-detected

BrainIn in;                   // built at each junction from the globals the
in.left = left_poss;          // firmware already had
in.right = right_poss;
in.front = (s[3] || s[4] || s[5] || s[6]);
in.back = 1; in.target = OnEndZoon; in.dist_cm = <the link just closed, cm>;

char move = brain_step(&in);  // 'F'/'L'/'R'/'B', or BRAIN_DONE
// ... the firmware's unchanged `cross` dispatch handles the motion
```

On `BRAIN_DONE`, `brain_home_path()` and `brain_fast_path()` supply the two replay strings. `set_plan()` copies them into `path_back` / `path_discoverd_s` and appends the legacy `'D'` end sentinel the replay stages depend on; `replay_dispatch()` is the single executor for both stages.

**Watch out for two things the legacy stages got wrong** (both fixed 2026-09-23, both would have driven the robot somewhere the plan did not intend):
- **A command is one STOP, not one graph edge.** The brain's `'L'` means *turn in place now, then drive until you would stop*. The legacy stage drove forward first and dispatched `path[cc-1]` at the *next* junction.
- **The home path is not a retrace.** It is the shortest path on the discovered graph and may leave by a different branch, so the robot does **not** flip its heading. `brain.h` says the home path assumes the robot's *current* heading and the fast path assumes the heading after the home run — the two strings are one continuous stream.

**Reference files (read-only):**
- `New Start/code/Core/Src/main.c` — the earlier left-hand-rule firmware (history; the legacy explorer came from here)
- `New Start/Simulator/py-code/maze solving/maze_gbf/stm-sample-code/main.c` — an earlier `USE_MAZE_GBF` integration via a `maze_hal.h` (history)
- `final version of seyed/robot codes/inc/brain.h` — **the contract.** Read this before touching `main.c`.

**Remaining work:** on-target bring-up only. Nothing in the repo can check the brain's *model* against the physical robot (`brain_host.c` drives the brain from a model of the robot, so a wrong model is invisible to it). Flash `USE_TELEMETRY=1`, press KEY1, and read §2 and §6 of `parse_telemetry.py`'s report.

---

## Common commands reference

| Command | Purpose |
|---|---|
| `python "final version of seyed/simulator/maze solver/maze_solver.py"` | Run solver GUI |
| `python "final version of seyed/simulator/maze generator/maze_generator.py"` | Run maze generator |
| `cd "final version of seyed/robot codes" && python scripts/run_maze.py ../simulator/mazes/<file>.json` | Test any maze with the C solver |
| `cd "final version of seyed/robot codes" && powershell -NoProfile -ExecutionPolicy Bypass -File ./scripts/build_all.ps1` | Rebuild + run C tests (21) |
| `cd "final version of seyed/robot codes" && python scripts/run_brain.py ../simulator/mazes/real_field.json` | Decision core vs a robot model (7/7) |
| `cd "final version of seyed/robot codes" && bash scripts/build_firmware.sh` | Build + link the STM32 firmware (ARMCC 5) |
| `cd "final version of seyed/robot codes" && bash scripts/measure_solver_ram.sh` | Check the 8 KB RAM budget |
| `gcc --version` | Verify GCC (MSYS2 MinGW) |

---

## Glossary

- **Node** — Maze intersection (has world position)
- **Edge** — Traversable line segment between adjacent nodes (axis-aligned only)
- **Known map** — Subset of nodes/edges the robot has actually driven
- **Frontier** — A visited node with at least one unexplored incident edge (drawn as yellow stubs)
- **Heading** — Robot's facing as unit vector (N/E/S/W)
- **Command** — `F` forward, `L` turn left, `R` turn right, `B` U-turn. **One command is one STOP, not one graph edge**: it means "turn as told, then drive until you reach somewhere you would stop", so it can carry the robot several cells.
- **Stop point** — A junction the robot actually comes to rest at (lateral exit, or dead end). A node with only a forward exit is driven *through* without a report.
- **proven_optimal** — Exploration stopped early with incomplete map (proof by time bound)
- **fully_explored** — Explored every edge before finishing
