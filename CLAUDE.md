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

### Live capture (bring-up instrument)

```bash
cd "final version of seyed/robot codes"
python scripts/bt_monitor.py                  # GUI: pick the COM port, connect
python scripts/bt_monitor.py --replay test/fixtures/capture_agree.txt --headless
```

Opens the Bluetooth COM port and, as the data arrives, draws the brain's believed map and checks **every decision the robot makes against the real `brain.c`** — spawned as `build/brain_oracle.exe`, not a Python re-implementation — so a disagreement is a fact about the robot or the wire, not about a second implementation. Recordings land in `build/bt_captures/` as verbatim `.txt` (readable by `parse_telemetry.py`) + `.csv` + `.json`.

**`pyserial` is the repo's only third-party dependency** (rule 6's stdlib preference): imported inside a `try` with a clear `pip install pyserial` message, needed **only** for the live serial path — `--replay`, `--demo` and `--headless` run without it.

### C Solver (testing)

```bash
cd "final version of seyed/robot codes"

# Test any maze .json from the simulator (generates a temp header, compiles, runs, cleans up):
python scripts/run_maze.py ../simulator/mazes/sample_maze.json

# Rebuild + run all unit tests after code changes:
powershell -NoProfile -ExecutionPolicy Bypass -File ./scripts/build_all.ps1
```

**Toolchain:** verified working here — GCC 14.2.0 (MSYS2 MinGW, `/c/msys64/mingw64/bin/gcc`) and Python 3.14 (`/c/Python314/python`). If `gcc` is not found, add it: `export PATH="/c/msys64/mingw64/bin:$PATH"`.

**Which tests actually run:** `build_all.ps1` builds 5 targets and runs them all — `test_graph` (6) + `test_robot` (7) + `integration_test` (8) = **21 unit tests**, plus `brain_oracle --selftest` (**5 steps — the one place `brain.c` is actually *executed***), plus `bt_monitor.py --replay` on two fixtures (**0 mismatches on the agreeing one, exactly 1 on the disagreeing one**). **It exits 1 if anything failed.** The `brain.o` target stays a **compile-only** `-Werror` build, keeping `brain.c` under the zero-warning rule; `brain_host.c` (its maze-driven test) needs a generated maze header, so `scripts/run_brain.py` drives that instead — **7/7 checks on 5 mazes**. Note that 7/7 **does not transfer to the robot**: `brain_host.c` derives its inputs from the maze's true topology, so it can produce inputs the hardware cannot (see the `in.front` finding below).

All C flags: `-std=c11 -Wall -Wextra -pedantic`, plus `-lm` (fast-run float math) and `-Werror` for the `brain.c` compile (must stay zero-warning). Output goes to `build/`; `test/_maze_data.h` is generated per-run — both gitignored.

### Firmware (STM32, ARMCC 5)

```bash
cd "final version of seyed/robot codes"
bash scripts/build_firmware.sh                  # build + link, prints the Keil size line
bash scripts/measure_solver_ram.sh              # the 8 KB RAM budget, per object
```

**The build takes no mode argument.** The three diagnostic switches —
`USE_MAZE_TELEMETRY`, `USE_MAZE_HEALTH`, `HEALTH_ONLY` — are plain `0`/`1` in
`firmware/Core/Src/main.c`'s `BUILD SWITCHES` block, and that file is the only
place they exist: no `-D` flags, and Keil's `main.c` `Define:` box is empty. Set
them, rebuild, and the banner echoes back what was built. Bring-up = telemetry `1`
+ health `1` + `HEALTH_ONLY 0`; bench = health `1` + `HEALTH_ONLY 1`.

Uses Keil's own **ARM Compiler 5** (`uAC6=0`, `C:\Keil_v5\ARM\ARMCC\bin`) — **not** armclang. Measured 2026-09-24: the plain brain-driven build is **40080 / 65536 flash, 6832 / 8192 RAM (1360 B free)**; the telemetry+health bring-up build costs 2 KB more flash and 80 B more RAM (**42124 / 6912**, 1280 B free); the health bench build is **41176 / 6848**. Run `measure_solver_ram.sh` — or rather, re-link — before claiming anything fits.

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
├── test/         # 5 test programs + run_maze.c generic runner + fixtures/
├── scripts/      # run_maze.py, run_brain.py, build_all.ps1, build_firmware.sh,
│                 # parse_telemetry.py, bt_monitor.py, ...
└── build/        # compiled output (gitignored)
```

**Module pipeline:** `brain.h` (the seam — `BrainIn` in, one move out; no hardware access, no `extern` firmware globals) → `brain.c` (dead reckoning + the two plan strings) → `maze_solver.c` (top-level FSM, **`EXPLORE → RETURN_HOME → FAST_RUN → DONE`** — note this *is* 4 phases here, unlike the Python) → the workers `maze_graph.c`, `maze_robot.c`, `maze_explore.c`, `maze_proof.c`, `maze_fastrun.c`. `maze_graph.c` and `maze_robot.c` are pure logic with no hardware dependency.

**There is no HAL.** The old `maze_hal.h` assumed the solver would drive the robot through `maze_hal_tick()` while the legacy explorer kept its own map — two integration points and two RAM budgets for one job. It was deleted on 2026-09-23 along with the legacy explorer. The firmware keeps all sensing and motion; the brain is a pure decision function.

**`maze_config.h` is the single tuning point** — memory limits, motion params, algorithm thresholds, with compile-time `#error` validation. Don't scatter new constants into the modules.

**C constraints that bind any change:**
- All arrays are **statically sized, no `malloc`** (`MAZE_MAX_NODES=64`, `MAZE_MAX_EDGES=112`). Adding a struct field is a RAM-budget decision — the whole firmware has 1360 B free (1280 B in the telemetry build). Note `brain.c` keeps its own `s_home[193]` / `s_fast[193]`, which `main.c` then copies into `path_back` / `path_discoverd_s`; pointing them at caller-supplied buffers would free 386 B if the margin ever gets tight.
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

**Remaining work:** on-target bring-up only. Nothing in the repo can check the brain's *model* against the physical robot (`brain_host.c` drives the brain from a model of the robot, so a wrong model is invisible to it). Flash the bring-up build (telemetry `1` + health `1` in `main.c`), press KEY1, and **watch it live with `scripts/bt_monitor.py`** — it feeds each junction's real `BrainIn` to `build/brain_oracle.exe` (the real `brain.c`) and flags a divergent decision at the junction, while the robot is still on the field. Then read §2 and §6 of `parse_telemetry.py`'s report for the offline measurements.

**⚠ OPEN — settle on the robot before trusting P1: `in.front` is fused to `left`/`right`.** `main.c:932` builds `in.front` as `(s[3]||s[4]||s[5]||s[6])`, and `main.c:1338-1339` builds the laterals on that **same** term, so:

```
left_poss  = s[2] && (s[3]||s[4]||s[5]||s[6])  =  s[2] && front
right_poss = s[7] && (s[3]||s[4]||s[5]||s[6])  =  s[7] && front
```

`front = 0` therefore forces `left = right = 0`. **`L=1 ⟹ F=1` is an identity, not a tendency** — the brain can never be handed "turn left, nothing ahead", which is exactly what a corner is. The stop test at `main.c:1377` closes the loop: `head_delay` is reset whenever `at_node` is true, so the persistence clause cannot fire at a node, so a node stop must come from `at_node && (left_poss||right_poss)` and therefore has `front = 1`.

**So at a corner with no straight-through lane the brain gets one of exactly two inputs, and both are wrong:**

| corner stop is… | mask | brain is told | should be |
|---|---|---|---|
| `at_node` (bits 0,9 set) | `F=1, L=1, R=0` | corner ≡ T-junction → P1 answers `'F'` | `F=0, L=1, R=0` |
| persistence (bits 0,9 clear) | `F=0, L=0, R=0` | **corner ≡ dead end** → brain answers `'B'` | `F=0, L=1, R=0` |

**The test — at a corner whose lane does not continue straight, read field 7 of the `J` line (raw mask, hex):** bits 0 **and** 9 set → it is reported as a T; bits 0/9 clear → reported as a dead end.

**Do not cite the replay fixtures as evidence** — `bt_monitor.py --replay`'s "front == (left or right) at 5 of 5 junctions" is **circular** (the masks were built from the firmware's own stop condition, so it restates the source). A fixture cannot test the question it was constructed to assume. Details in `robot codes/STATUS.md` and the 2026-09-23 CHANGELOG entry.

---

## Common commands reference

| Command | Purpose |
|---|---|
| `python "final version of seyed/simulator/maze solver/maze_solver.py"` | Run solver GUI |
| `python "final version of seyed/simulator/maze generator/maze_generator.py"` | Run maze generator |
| `cd "final version of seyed/robot codes" && python scripts/run_maze.py ../simulator/mazes/<file>.json` | Test any maze with the C solver |
| `cd "final version of seyed/robot codes" && powershell -NoProfile -ExecutionPolicy Bypass -File ./scripts/build_all.ps1` | Rebuild + run C tests (21) |
| `cd "final version of seyed/robot codes" && python scripts/run_brain.py ../simulator/mazes/real_field.json` | Decision core vs a robot model (7/7) |
| `cd "final version of seyed/robot codes" && python scripts/bt_monitor.py` | **Live Bluetooth capture + virtual-brain decision check** (needs `pip install pyserial`; opens COM9 — the bench adapter, `DEFAULT_PORT` — by itself when enumerated) |
| `cd "final version of seyed/robot codes" && python scripts/bt_monitor.py --replay test/fixtures/capture_agree.txt --headless` | Re-run a capture with no robot; exit 1 on any mismatch |
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
- **Corner** — A node where **the lane turns 90° and there is no way straight on**: exactly one lateral exit, `back` (the way you came), and **no `front`**. In `BrainIn` terms `(F=0, L=1, R=0)` or `(F=0, L=0, R=1)`. It is *not* a corner if a straight-through lane exists (that is a T or a crossing), and a dead end (`F=0, L=0, R=0`) is not one either. The distinction matters because the hardware cannot currently report it — see rule 9 in `ARCHITECTURE.md` §5.
- **Stop point** — A junction the robot actually comes to rest at (lateral exit, or dead end). A node with only a forward exit is driven *through* without a report.
- **proven_optimal** — Exploration stopped early with incomplete map (proof by time bound)
- **fully_explored** — Explored every edge before finishing
