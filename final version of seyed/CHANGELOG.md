# SEYED Maze Robot — Development History / Worklog

> **Read this after `ARCHITECTURE.md` and `ALGORITHMS.md`.** This file is the
> running history of *what was changed, why, and what we learned* — so a new
> session can get up to speed fast and avoid repeating mistakes.
>
> ## RULE FOR EVERY SESSION (including AI agents)
> **After you change the code, append/update an entry here in the same turn.**
> Keep newest at the top. Record: the date, what changed, *why*, key decisions
> and trade-offs, files touched, and anything you learned (gotchas, conventions).
> Also keep `ARCHITECTURE.md` / `ALGORITHMS.md` in sync when behaviour changes.

---

## Conventions & hard-won lessons (keep at top; update as you learn)

- **Shell is PowerShell on Windows.** Chain commands with `;` (NOT `&&`).
  `$(cat <<'EOF' …)` here-docs do **not** work — for multi-line commit messages
  write a temp file and `git commit -F msg.txt`, then delete it.
- **Repo path has spaces** (`F:\Robotic fle 2022\SEYED`). Always quote paths.
- **Console encoding is cp1252**: printing unicode like `✓` crashes test scripts.
  In quick scripts call `sys.stdout.reconfigure(encoding='utf-8')` first.
- **Headless test pattern** (no GUI display needed):
  load the module with `importlib`, `root = tk.Tk(); root.withdraw()`, build
  `SolverApp(root)`, then drive `app._frame(dt)` in a loop until `phase=="DONE"`.
  This exercises the full mission + drawing without `mainloop()`.
- **Units are real-world: 1 world unit = 1 cm** (grid spacing 20 → 20 cm).
  Speeds are cm/s, accel cm/s². Don't reintroduce abstract "units/s".
- **The fastest route is by TIME, not distance** (acceleration model). Any
  bound/pruning that compares routes MUST use time with the same `v_max`, or it
  becomes inadmissible (a longer-but-straighter route can be faster).
- **Tkinter `dash=` is in pixels**, but stub/edge *lengths* are world units —
  don't confuse the two.
- **Tk ovals aren't anti-aliased**: circles go through the Pillow cache
  (`_circle_image`). Keep `PhotoImage` refs alive (the cache holds them).
- Keep everything **single-file, standard-library** (Pillow optional). No
  `networkx` / `matplotlib` in the final-version simulator.
- Don't touch `New Start/` or `old docs/` — they are read-only history.

---

## History (newest first)

### 2026-06-15 — Increased memory limits to 100 nodes + macro-ified all struct sizes
- **MAZE_MAX_NODES 50→100, MAZE_MAX_EDGES 100→200, MAZE_MAX_PATH_LENGTH 50→100, MAZE_MAX_FRONTIERS 20→40, MAZE_COMMAND_LOG_SIZE 512→1024, MAZE_EXPLORATION_TIMEOUT 1000→2000.**
- **maze_types.h structs now use `MAZE_MAX_*` macros** instead of hardcoded `[50]`/`[100]`/`[512]`.  Added `#include "maze_config.h"` to maze_types.h so the macros are available at struct-definition time.
- **All .c files updated:** replaced every hardcoded `[50]`/`<50`/`, 50)` with the appropriate `MAZE_MAX_*` macro (graph heap, Dijkstra dist/parent/visited, path buffers, fast-run stop-graph arrays, frontier tracking).
- **sample_maze3.json now works** — 56 nodes needed this bump.  Result: 95 steps, target found, fast path 13 nodes / 9.14s, 240 cm.
- **Memory estimate:** ~6.1 KB with 100-node defaults (still safe on STM32G0's 8KB RAM).
- **21 unit tests + 3 maze runs verified** (sample_maze, sample_maze4, sample_maze3).
- Files: `maze_config.h`, `maze_types.h`, `maze_graph.c`, `maze_robot.c`, `maze_explore.c`, `maze_fastrun.c`, `maze_solver.c`, `CHANGELOG.md`.

### 2026-06-15 — Scripted maze runner: feed .json, get result (no C editing)
- **Created `scripts/run_maze.py`** -- reads any maze `.json` from the simulator,
  generates `test/_maze_data.h`, compiles + runs `run_maze.c`, prints full results
  (steps, commands, fast-run stream, time, distance), then cleans up the temp header.
  One command to test any maze: `python scripts/run_maze.py ../simulator/mazes/your_maze.json`.
- **Created `test/run_maze.c`** -- generic test harness that `#include`s the generated
  `_maze_data.h` instead of hardcoding maze data.  Same incremental-discovery logic
  as `integration_test.c`, but fed dynamically from the `.json`.
- **Created `scripts/build_all.ps1`** -- one-command rebuild + run of all unit tests
  (`test_graph`, `test_robot`, `integration_test`, `test_hal_compile`).  Use after
  editing any C source.
- **Removed `test_maze4.c`** -- replaced by `run_maze.py` + `run_maze.c` (generic).
- **Added `_maze_data.h` to `.gitignore`** -- auto-generated temp file, never committed.
- **Rewrote `BUILD_GUIDE.md`** -- simplified to reflect the new workflow.
- Files: `scripts/run_maze.py` (new), `scripts/build_all.ps1` (new),
  `test/run_maze.c` (new), `test/test_maze4.c` (deleted),
  `.gitignore`, `BUILD_GUIDE.md`, `CHANGELOG.md`.

### 2026-06-15 — Fast-run command stream in tests + sample_maze4 test
- **Added fast-run command stream** to both integration tests — after the full mission command log, the test now computes and prints the F/L/R/B commands specifically for the fast-run path using `maze_robot_compute_command()` on `fast_path_nodes[]`.
- **Created `test/test_maze4.c`** — full-mission test on `sample_maze4.json` (37 nodes, start at (-20,-20), target at (40,-120)).  97 steps, fast path 23 nodes / 12.30s, shortest distance 240 cm.  8 tests pass.
- **Updated `BUILD_GUIDE.md`** — added Test 5 entry, updated "run all" one-liner, updated reference table, updated sample output with new fast-run command line.
- Files: `test_maze4.c` (new), `test/integration_test.c`, `BUILD_GUIDE.md`, `CHANGELOG.md`.

### 2026-06-15 — Build folder clean-up + BUILD_GUIDE.md
- **Moved all `.exe` files** from `test/` to new `build/` folder (gitignored).
- **Added `*.exe` to `.gitignore`** — no compiled binaries in the repo.
- **Created `BUILD_GUIDE.md`** in `robot codes/` — full build/run commands, file layout, pipeline diagram, module dependency tree.
- **Fixed stale compile paths** in all 4 test file comments (old `-I../inc` → `-I inc`, output `-o build/...exe`).
- **Updated `STATUS.md`** build commands to use `build/` folder + point to new guide.
- Files: `BUILD_GUIDE.md` (new), `build/` (new), `.gitignore`, `STATUS.md`, `test/test_graph.c`, `test/test_robot.c`, `test/integration_test.c`, `test/test_hal_compile.c`.

### 2026-06-15 — Reorganised: C code moved to `robot codes/`
- **Moved all C solver files** from `simulator/maze solver/maze_solver_c/` to
  `robot codes/` under `final version of seyed/`.  The C solver is NOT part of
  the simulator — it's a standalone portable library for the STM32 firmware.
- **Clean separation:** `simulator/` = Python/Tkinter only.  `robot codes/` = C
  solver headers, sources, tests, HAL bridge, STATUS.md.
- Updated all documentation paths: `ARCHITECTURE.md`, `CLAUDE.md`, `CHANGELOG.md`,
  `STATUS.md`.
- Build commands unchanged (relative paths within `robot codes/` are still valid).

### 2026-06-15 — HAL bridge + sensor-based branch discovery fix (Step 9)
- **Created `maze_hal.h`** — STM32 Hardware Abstraction Layer bridging the new C
  solver to the real robot firmware.  Header-only (`static inline`), follows the
  existing `stm-sample-code/maze_hal.h` pattern.
- **API:** `maze_hal_init()` (boot) + `maze_hal_tick()` (per-intersection): reads
  sensors (head-aware — front bank s[3..6] vs rear bank s[12..15]), gets position
  from firmware's `node[path_c]`, calls `maze_solver_update_position()` + 
  `maze_solver_step()`, returns `MazeCommand` → firmware converts to `cross` via
  `maze_cmd_to_cross()`.
- **Integration:** gated behind `USE_MAZE_SOLVER` (alongside existing `USE_MAZE_GBF`).
  Firmware changes are ~10 lines per intersection block; the existing `cross`
  dispatch handles motor execution.
- **Direct motor control** fallback (`maze_hal_execute_direct`) for call sites that
  want to skip `cross` — handles `head` flip and `nav` +180° for U-turns.
- **BLT debug** (`maze_hal_print_state`, `maze_hal_print_command`) gated by
  `MAZE_DEBUG_ENABLED`.
- **Test:** `test_hal_compile.c` — 10 tests, compile+smoke with stubbed firmware
  globals, 3-node L-maze mission.
- **Solver fix:** `maze_solver_update_position()` previously only processed sensor
  data when creating a NEW node — the start node had no unexplored branches, so
  exploration returned NONE immediately.  Now branches are discovered for ALL nodes
  via `_discover_branches()`: creates placeholder neighbor nodes at 20 cm offset
  for each detected open path (left/forward/right; back is skipped).  Heading=NONE
  defaults to NORTH for the first sensor reading.  `maze_graph_add_edge()` is
  idempotent (skips duplicates), so repeated sensor readings are safe.
- Files: `inc/maze_hal.h`, `test/test_hal_compile.c`, `src/maze_solver.c` (updated),
  `STATUS.md`, `CHANGELOG.md`, `CLAUDE.md`.
- **31 tests pass, zero warnings.**

### 2026-06-15 — C port of maze solver: all core modules complete (Steps 1-8)
- **Created a complete C port** of the Python `maze_solver.py` algorithm in
  `robot codes/`, targeting STM32G031G8Ux (Cortex-M0+,
  8KB RAM, 64KB flash).  8 modules, 21 tests, zero warnings.
- **Modules built:**
  - `maze_types.h` / `maze_config.h` — types matching firmware `nav` (0=N/1=W/
    2=S/3=E), commands as F/L/R/B chars, fixed-point motion model (×100)
  - `maze_graph.h/.c` — node/edge CRUD, binary-heap Dijkstra on known map by
    real distance (cm), integer sqrt
  - `maze_robot.h/.c` — heading tracking, F/L/R/B command generation via cross-
    product (ALGORITHMS.md §5), frontier detection, turn-minimizing branch ranking
  - `maze_explore.h/.c` — P1→P2→P3 priority system, hooks proof module for
    frontier filtering
  - `maze_proof.h/.c` — time-based admissible lower bound (LB_time = (dist_known
    + straight_line) / v_max), frontier pruning, PROVEN/FULL/DISABLED states
  - `maze_fastrun.h/.c` — trapezoidal velocity profile (`run_time`, `speed_at`),
    time-optimal path (stop-graph Dijkstra), fast-run plan builder
  - `maze_solver.h/.c` — top-level EXPLORE→RETURN_HOME→FAST_RUN→DONE FSM
- **Integration test** runs full mission on 27-node sample maze with incremental
  discovery: 53 steps, command stream verified, fast path = 8 nodes / 6.50s,
  proof state = FULL.
- **Design decisions:** all arrays statically sized (no malloc), time math uses
  float for now (proof/fast-run called infrequently — can convert to integer
  later if needed), headings match firmware `nav` encoding.
- **Remaining:** STM32 HAL bridge (`maze_hal.h`) + cross-compile/integration
  into `New Start/code/`.
- Files: `robot codes/` (8 headers + 6 sources + 3 tests
  + STATUS.md), `CLAUDE.md` (updated with C reference map), `CHANGELOG.md`.

---

### 2026-06-14 — Yellow stubs: solid lines, 5 cm, one-cycle delay, edge-enter removal
- Unexplored-branch stubs are now **solid yellow lines of fixed 5 cm length**
  (replaces the previous fixed-size yellow dots). A solid line reads more clearly
  as "a branch continues this way" regardless of the edge's orientation or length,
  and 5 cm is ¼ of the standard 20 cm node spacing — long enough to be visible
  without overshooting to the next node.
- **One-cycle visual delay:** stubs appear only when the robot graphic has
  *visually arrived* at a node (tracked by new `_arrived_nodes` set), not when
  the logical state advances. This prevents stubs from popping up at the
  destination while the robot is still mid-glide.
- **Removal-on-entry:** a stub disappears only when the robot enters that
  specific edge (checked via `visited_edges`). Stubs persist across all
  previously-visited nodes — they are not cleared just because the robot left.
- Tools: `maze_solver.py` (`STUB_LEN=5.0`, `_draw_stubs`, `_arrived_nodes`),
  `ARCHITECTURE.md`, `ALGORITHMS.md`, `CHANGELOG.md`.

### 2026-06-14 — Uniform yellow stub dots (fixed size, edge-length independent)
- Unexplored-branch stubs are now **3 fixed-size dots** per branch (`STUB_DOT_R`,
  `STUB_DOT_SPACING`, `STUB_DOT_COUNT`) — identical on every edge, no scaling
  with edge length. Replaces the dashed-line stub that looked different on long vs
  short edges.
- Legend updated: unexplored stub shown as a dot.
- Files: `simulator/maze solver/maze_solver.py`.

### 2026-06-14 — Added `CHANGELOG.md` (session worklog)
- Removed the `SEYED • Final Version` subtitle and the `ROBOT MOTION MODEL`
  header label (visual clutter).
- Made the **Optimized search** toggle a small, natural-width button (new
  `compact` style on `_button`); `Fit View` sits next to `Load Maze…`; `Play /
  Step / Reset` on their own row.
- **Unexplored-branch stub** now spans `STUB_FRAC = 0.62` of the *edge* (scales
  with edge length, so it reads as "branch this way" even on long corridors)
  with **bigger dots** (`width=8`, `dash=(1,8)`). Replaces the old fixed
  `STUB_LEN`. Reason: on mazes with multi-cell edges the old 17 cm stub looked
  tiny (< half the visible gap).
- Files: `simulator/maze solver/maze_solver.py`.

### 2026-06-14 — Speedometer, speed readout fix, layout spacing
- Fixed **Speed value flickering 0 ↔ 40** during search: a line-follower drives
  through intersections without stopping, so `cur_speed` is now held at the
  constant explore speed for the whole search (drops to 0 only on pause / stop).
- Added a **car-cluster speedometer gauge** (`_build_gauge` / `_draw_gauge`):
  half-dial, green→red band that fills to the current speed, ticks/labels,
  needle, digital `cm/s`. It rescales to **Max speed** and is responsive to the
  panel width (redrawn on `<Configure>`).
- Files: `maze_solver.py`, `ARCHITECTURE.md`.

### 2026-06-14 — Optimized-search toggle, real units, playback multiplier
- **Optimized search** toggle (`Robot.use_proof`): ON may stop early once the
  fastest route is proven; OFF disables the proof and maps the **full maze**.
  Proof field shows `off (full)` when disabled.
- **Real-world units**: defaults `v_max = 100 cm/s`, `a_max = 50 cm/s²`, cautious
  explore `40 cm/s`. Sliders relabelled with cm units; status shows cm / cm/s.
- **Playback** changed from an arbitrary delay slider to a **real-time
  multiplier** (`0.25x … 8x`, `1x` = true real time). Fixed `FRAME_MS`; each
  frame advances sim-time by `FRAME_MS/1000 · playback`.
- Files: `maze_solver.py`, `ARCHITECTURE.md`, `ALGORITHMS.md`.

### 2026-06-14 — Early-stop proof made TIME-based (bug fix)
- **Bug:** the proof pruned frontiers by *distance*, so it raised `PROVEN ✓`
  even when an unexplored, longer-but-straighter route would be **faster in
  time** (fewer stops). Reported from a screenshot.
- **Fix:** prune frontier `f` only if
  `LB_time(f) = (dist_known(start→f) + ‖f→target‖) / v_max ≥ best_known_TIME`,
  where `best_known_TIME` comes from `time_optimal_path`. Dividing the distance
  lower bound by `v_max` is an admissible time lower bound, so a faster straight
  branch is never wrongly pruned. `Robot` now carries `v_max`/`a_max`, kept in
  sync with the sliders.
- Consequence: a *faster* robot prunes less (distance is "cheap" in time) — this
  is correct, not a regression.
- Files: `maze_solver.py`, `ARCHITECTURE.md` (rule 6), `ALGORITHMS.md` (§2/§4/§6).

### 2026-06-14 — Acceleration-aware fast run + multi-path visualisation
- Implemented the acceleration model that `ALGORITHMS.md` had listed as future
  work:
  - `run_time`, `speed_at`, `stop_indices`, `path_time` — trapezoidal profile,
    **v = 0 at every turn**, accelerate on straights.
  - `time_optimal_path` — minimum-**time** route over the *discovered* map via a
    "stop graph" Dijkstra (edges = maximal straight runs; state = node, because
    the robot is stopped at every turn). Expanded back to a full node path.
  - `FastRunPlan` — time-parameterised trajectory (samples `t→s`, `t→v`) for
    smooth physics-based animation and speed-gradient colouring.
  - `yen_k_shortest` — alternative routes for display.
- **Animation** rewritten to be frame-based: robot **glides** between nodes
  (constant speed while exploring; accel profile on the fast run) via a decoupled
  `anim_pos`/`anim_heading`.
- **Visualisation**: fastest route bold dark green; slower alternatives lighter
  & thinner; the driven fast-path coloured by a **speed gradient (red=fast,
  green=slow)** via `speed_color_segments` + `speed_to_color`.
- **GUI**: added Max speed / Max accel; status gained current Speed + Path time.
- **Note kept:** exploration/return/command-generation (§1, §3, §5) are
  unchanged; only the fastest-path metric changed (distance → time).
- Files: `maze_solver.py`, `ARCHITECTURE.md`, `ALGORITHMS.md`.

---

## Project state snapshot (update when it drifts)

- **Active areas:**
  - `final version of seyed/simulator/` — Python/Tkinter generator + solver
  - `final version of seyed/robot codes/` — C solver library + STM32 HAL bridge
- **Solver phases:** `EXPLORE → RETURN_HOME → FAST_RUN → DONE`.
- **Fastest route:** minimum-time, accel-aware, on the discovered map; proof is
  time-based and admissible.
- **C port:** 8 modules, 31 tests, zero warnings.  HAL bridge (`maze_hal.h`)
  ready for STM32 integration (Step 9 complete, Step 10 pending).
- **Mazes:** `simulator/mazes/*.json` (`sample_maze.json` auto-loads).
- **Run:**
  - `python "final version of seyed/simulator/maze solver/maze_solver.py"`
  - `python "final version of seyed/simulator/maze generator/maze_generator.py"`
- **Git remote:** `origin` → `github.com/mmd-bsd/robotic-maze-bot.git`, branch `main`.

## Known open items / ideas (not done yet)

- Explore speed is a constant (40 cm/s), not yet a GUI slider — expose if needed.
- Turns are modelled as a **full stop**; a finite turn speed-cap / turn-time and
  real encoder/IMU calibration of `v_max`/`a_max` are future work.
- **STM32 cross-compile + on-target integration** (Step 10): compile `robot codes/`
  with `arm-none-eabi-gcc`, link into `New Start/code/` behind `USE_MAZE_SOLVER`,
  test on hardware.
