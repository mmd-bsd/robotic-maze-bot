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

- **Active area:** `final version of seyed/simulator/` (generator + solver).
- **Solver phases:** `EXPLORE → FAST_RUN → DONE` (return-home is part of EXPLORE).
- **Fastest route:** minimum-time, accel-aware, on the discovered map; proof is
  time-based and admissible.
- **Mazes:** `simulator/mazes/*.json` (`sample_maze.json` auto-loads).
- **Run:**
  - `python "final version of seyed/simulator/maze solver/maze_solver.py"`
  - `python "final version of seyed/simulator/maze generator/maze_generator.py"`
- **Git remote:** `origin` → `github.com/mmd-bsd/robotic-maze-bot.git`, branch `main`.

## Known open items / ideas (not done yet)

- Explore speed is a constant (40 cm/s), not yet a GUI slider — expose if needed.
- Turns are modelled as a **full stop**; a finite turn speed-cap / turn-time and
  real encoder/IMU calibration of `v_max`/`a_max` are future work.
- Real-robot STM32 firmware port (command stream + proof) is still future work.
