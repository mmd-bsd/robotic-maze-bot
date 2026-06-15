# SEYED Maze Robot — Project Architecture & Rules

> Read this first in any new session. It explains what the project is, how the
> repository is laid out, how the simulator works, and the conventions/rules
> that have been established so far. Algorithm details live in
> [`ALGORITHMS.md`](./ALGORITHMS.md); the running history of changes, decisions,
> and gotchas lives in [`CHANGELOG.md`](./CHANGELOG.md) — **skim it too, and add
> an entry there whenever you change the code.**

---

## 1. What the project is

A **line-following maze robot**. The robot drives along lines that form a grid
maze. All intersections are **90° crossings** (no diagonals, no curves at
junctions). The mission, in order:

1. **Search / explore** the maze starting from the **start** point. The robot
   does **not** know where the target is at the start.
2. **Find the target** — a *big black area* on the field (modelled as a single
   target node).
3. **Prove it has found the fastest route** — either by discovering every path
   to the target, or by proving no undiscovered path could be shorter.
4. **Return to the start** point.
5. **Drive the fastest path** from start to target.

The fastest path accounts for the robot's **acceleration / variable speed**
(speed is not constant): the robot drives the **first search at a constant,
cautious speed**, then on the **fast run** it accelerates on clear straights and
**comes to a stop (v = 0) at every turn**. The fastest route therefore minimises
**time**, not distance — a longer-but-straighter route can beat a shorter twisty
one. See `ALGORITHMS.md` → §6. The exploration **early-stop proof is time-based
too** (same `v_max`), so it never prunes a straight branch that could be faster.

The project has **two parts**:

1. **Simulator** (Python) — design mazes and visualize/verify the algorithm.
2. **Real robot code** (STM32 / C) — the firmware that runs on hardware (future
   "final version"; earlier iterations exist under `New Start/` and `old docs/`).

---

## 2. Repository layout

```
SEYED/
├── final version of seyed/         <-- ACTIVE development area ("final version")
│   ├── simulator/
│   │   ├── maze generator/
│   │   │   └── maze_generator.py    Tkinter GUI to build mazes by clicking
│   │   ├── maze solver/
│   │   │   └── maze_solver.py       Tkinter GUI to simulate the robot
│   │   └── mazes/
│   │       ├── sample_maze.json     27-node demo maze (ported from sim_V12)
│   │       └── sample_maze2.json    larger demo with a far branch (tests pruning)
│   ├── robot codes/                  C maze solver port for STM32 firmware
│   │   ├── inc/                      headers (types, config, modules, HAL)
│   │   ├── src/                      implementations (graph, robot, FSM, …)
│   │   ├── test/                     unit + integration tests (31 tests)
│   │   └── STATUS.md                 module status, build commands, design notes
│   ├── ARCHITECTURE.md              <-- this file
│   ├── ALGORITHMS.md                algorithm + command-generation reference
│   └── CHANGELOG.md                 running worklog/history (keep it updated)
│
├── New Start/                       Previous iteration (reference, do not break)
│   ├── Simulator/
│   │   ├── py-code/
│   │   │   ├── maze generator/      maze_editor_V2.py, maze_editor_V3.py
│   │   │   └── maze solving/        sim_V2 … sim_V12_GBF_stable.py, maze_gbf/ (C port)
│   │   └── ALGORITHM_DESCRIPTION.md old step-by-step walkthrough of V12
│   ├── code/                        STM32 firmware (Core/, MDK-ARM, etc.)
│   └── field/
│
└── old docs/                        Oldest experiments: t1..t13 STM32 HAL projects
```

**Where to work:** all new simulator work goes in
`final version of seyed/simulator/`. The `New Start/` and `old docs/` trees are
**history/reference** — read them for context, but do not modify them unless
explicitly asked.

---

## 3. The simulator

Two standalone Tkinter apps that share a simple **JSON maze format**. The
generator saves a `.json`; the solver loads the same `.json`. No copy-pasting of
code between them.

### 3.1 Maze JSON format

```json
{
  "grid_size": 20,
  "nodes": { "0": [0, 0], "1": [0, 20], "22": [-60, 80] },
  "edges": [ [0, 1], [1, 22] ],
  "start": 0,
  "target": 22
}
```

- `nodes`: map of **string node id → `[x, y]` world coordinates** (Y is up).
- `edges`: list of `[id, id]` pairs; every edge is axis-aligned (horizontal or
  vertical) because the maze is 90°-only.
- `start` / `target`: node ids. Both are required by the solver.
- Node ids are assigned by the generator on save (sorted by position), so they
  are stable for a given drawing.

### 3.2 `maze_generator.py`

- **Build by clicking:** click an intersection to select it (yellow); click a
  second to draw an edge. Click **on** an existing edge to split it at that
  point and select the new node. Click the same point again to deselect.
- Edges are forced **horizontal/vertical** (90° maze). Non-axis-aligned attempts
  are rejected with a status message.
- **Set Start** (`S`, green circle) / **Set Target** (`T`, black circle = the
  big black area) / **Delete node** (`D`). Start ≠ Target enforced.
- **Save / Load** JSON (`Ctrl+S` / `Ctrl+O`), default folder `simulator/mazes/`.
- View: mouse-wheel **zoom** (around cursor), middle/right-drag **pan**, **Fit
  View**, auto-fit on first open and on load.

### 3.3 `maze_solver.py`

- **Loads** a maze JSON (auto-loads `sample_maze.json` on startup if present).
- Animates the **full mission** with phases: `EXPLORE → FAST_RUN → DONE`
  (return-to-start is part of EXPLORE).
- Controls: **Load**, **Play/Pause**, **Step**, **Reset**, an **Optimized
  search** toggle, a **Playback** multiplier (`0.25x … 8x`, where **1x = true
  real time**), **Max speed** + **Max accel** sliders (the robot motion model),
  **Fit View**, plus zoom/pan.
- **Units are real-world:** 1 world unit = **1 cm** (grid spacing 20 → 20 cm).
  Defaults: max speed **100 cm/s**, max accel **50 cm/s²**, cautious explore
  speed **40 cm/s**. Speeds/distances in the status card are cm/s and cm.
- **Optimized search** ON lets the robot stop early once the fastest route is
  proven (§2); OFF disables the proof so it always maps the **full maze**.
- The robot **glides** smoothly between nodes: constant speed while exploring,
  and the trapezoidal accelerate/cruise/decelerate profile on the fast run.
- **Status card:** Phase, Steps, At node, Last cmd, Target found, Travelled
  distance, **Proof** (the early-stop flag), current **Speed**, and **Path
  time** (the fastest route's drive time).
- **Speedometer:** a car-cluster-style half-dial (needle + green→red band +
  digital cm/s) tracks the robot's live speed; it rescales to the **Max speed**
  setting. During search the speed is the constant explore speed; during the
  fast run it follows the acceleration profile.
- **Command log:** every move as `F/L/R/B` with the from→to node and heading —
  these are the commands the real robot will execute.
- Implementation is **pure standard library** for the graph algorithms (BFS /
  Dijkstra are hand-written). It does **not** use `networkx`.

### 3.4 Visual legend (solver)

| Element | Meaning |
|---|---|
| Grey thin line | Unexplored maze edge (not yet driven) |
| Blue thick line | Visited / explored edge |
| Yellow solid line (5 cm) | Unexplored branch seen from a visited node (frontier) |
| Bold dark-green line | The **fastest** (minimum-time) route |
| Lighter/thinner green lines | **Slower** alternative routes (lighter = slower) |
| Red→green gradient on the fast path | Robot **speed** as it drives (red = fast, green = slow/at turns) |
| Green circle | Start |
| Black circle + red ring | Target (the black area) |
| Red circle + arrow | Robot (arrow = heading) |

---

## 4. Dependencies & how to run

- **Python 3.x** with **tkinter** (standard library).
- **Pillow** (`pip install pillow`) — used to render **anti-aliased circles**.
  If Pillow is missing, the apps still run but fall back to plain (aliased)
  Tkinter ovals.
- The final-version simulator does **not** need `networkx` or `matplotlib`
  (those are only used by the older `New Start/` sims).

```bash
python "final version of seyed/simulator/maze generator/maze_generator.py"
python "final version of seyed/simulator/maze solver/maze_solver.py"
```

---

## 5. Rules & conventions (learned so far)

These are hard-won; keep them in mind before changing the simulator.

### Algorithm correctness (most important)
1. **No cheating during the first search.** The robot does **not** know the
   target's location until it physically reaches it. **No exploration decision
   may use the target's coordinates** before `target_found` is true. (A previous
   version sorted branches by distance-to-target and "beelined" to the target —
   that was a bug.)
2. **After** the target is found, using its position is allowed (for the
   early-stop proof).
3. **Local exploration choice is target-agnostic:** prefer **straight → left →
   right → reverse** (fewest turns for a line follower), with a fixed compass
   tie-break so behaviour is deterministic.
4. **Frontier selection uses real travel distance** (Dijkstra on the *known*
   map), not hop count.
5. **The fastest path is computed on the DISCOVERED map only** — never on the
   full maze — so early-stopping stays honest (the robot must not "use" edges it
   never drove).
6. **Early-stop pruning is admissible and TIME-based:** a frontier `f` is only
   worth visiting if `(dist(start→f) + straight_line(f→target)) / v_max <
   best_known_TIME`. Dividing the distance lower bound by `v_max` is a valid
   lower bound on time, so pruning never discards a route that could actually be
   *faster* — including a clear straight that beats a twisty known path. The
   bound MUST use the same `v_max` as the fast run. See `ALGORITHMS.md` §2.
7. Flags: `proven_optimal` = stopped early **with branches still unexplored**;
   `fully_explored` = genuinely explored everything. They are mutually
   exclusive.

### Maze rules
8. Mazes are **90° only** — all edges horizontal or vertical. The generator
   enforces this.
9. Coordinates are world units; **Y is up**. `grid_size` defaults to 20.

### GUI / graphics
10. **Dark theme.** Colors are defined as constants at the top of each file
    (`BG`, `PANEL_BG`, `CARD_BG`, `CANVAS_BG`, `FG`, `ACCENT`, `ACCENT2`, …).
    Reuse them; don't hard-code new colors ad hoc.
11. **Tkinter ovals are NOT anti-aliased.** Render circles via **Pillow
    supersampling** (`_circle_image`, drawn at 4× then LANCZOS-downscaled,
    cached by `(fill, outline, r, ow)`). Keep PhotoImage references alive
    (the cache holds them).
12. **Windows DPI:** call `_enable_hidpi()` (SetProcessDpiAwareness) before
    creating the Tk root, and set `tk scaling`, or everything renders blurry on
    high-DPI screens.
13. The **target is a circle** (black fill + red ring), not a square.
14. The side **panel is resizable** (a `PanedWindow` sash) **and scrollable**
    (inner Canvas + Scrollbar + mouse-wheel) so it works on small screens.
    Keep panel content in fixed heights (e.g. command log) so the scroll region
    is correct.
15. **Playback is a real-time multiplier** (`0.25x … 8x`, snapped), not a raw
    delay: at `1x` the animation runs in **true real time** (the frame interval
    is fixed at `FRAME_MS`; each frame advances sim-time by
    `FRAME_MS/1000 · playback`). This is separate from the robot's physical
    **Max speed / Max accel** (cm, cm/s, cm/s²) — never conflate the two.

### Project / workflow
16. Prefer **self-contained files** (the project values single-file scripts) and
    **standard library only** (Pillow optional). Don't add heavy deps casually.
17. The shell here is **PowerShell on Windows** — chain commands with `;`, not
    `&&`. Quote paths with spaces (the repo path has spaces).
18. Don't break `New Start/` or `old docs/`; treat them as read-only history.
19. When unsure about a product decision (data format, scope, UX), ask — the
    user prefers being consulted on meaningful trade-offs.
20. **Always update [`CHANGELOG.md`](./CHANGELOG.md)** in the same turn you
    change the code: append a dated entry (newest first) with what/why/decisions
    and any new lessons. Keep `ARCHITECTURE.md`/`ALGORITHMS.md` in sync too.

---

## 6. Glossary

- **Node** — a maze intersection (has a world position).
- **Edge** — a traversable line segment between two adjacent nodes.
- **Known map** — the subset of nodes/edges the robot has actually driven
  (`visited_nodes`, `visited_edges`).
- **Frontier** — a *visited* node that still has at least one *unexplored*
  incident edge (drawn as solid yellow lines, 5 cm long, along the branch
  direction). Stubs appear only after the robot has visually arrived at the node
  and disappear when the robot enters that specific edge.
- **Heading** — the robot's current facing as a unit vector (N/E/S/W).
- **Command** — `F` forward, `L` turn left, `R` turn right, `B` U-turn (reverse).
- **proven_optimal** — exploration stopped early because no undiscovered path
  could beat the best known one (map intentionally incomplete).
- **fully_explored** — exploration discovered every edge before finishing.

---

## 7. Roadmap / open items

- **Acceleration-aware fastest path** (variable speed, stop at turns) — **done**
  in the simulator (`time_optimal_path`, `FastRunPlan`), and the exploration
  early-stop **proof is time-based** too (admissible with `v_max`), so a faster
  straight branch is never falsely pruned; see `ALGORITHMS.md` §2/§6.
  Open: calibrate `v_max` / `a_max` / per-turn caps from real encoder/IMU data
  (turns are currently modelled as a full stop).
- **Real-robot firmware** in the final version (port the command stream +
  exploration/proof logic to STM32; earlier C work lives in
  `New Start/Simulator/py-code/maze solving/maze_gbf/` and `New Start/code/`).
