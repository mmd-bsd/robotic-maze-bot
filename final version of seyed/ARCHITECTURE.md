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

The project has **three parts**:

1. **Simulator** (Python) — design mazes and visualize/verify the algorithm.
2. **C maze-solving library** (`robot codes/`) — a portable, host-testable port of
   the same algorithm, plus the decision core (`brain.c`).
3. **STM32 firmware** (`firmware/`) — the robot. It owns all sensing and motion;
   it calls `brain_step()` at each junction and replays the two plans the brain
   returns. Earlier iterations exist under `New Start/` and `old docs/`
   (read-only history).

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
│   │   ├── inc/                      headers (types, config, modules, brain)
│   │   ├── src/                      implementations (graph, robot, FSM, brain, …)
│   │   ├── test/                     unit + integration tests (21) + brain_oracle
│   │   │   └── fixtures/             bt_monitor replay captures (agree/disagree,
│   │   │                             health ok/fault — synthetic, see below)
│   │   ├── scripts/                  build_all.ps1, run_maze.py, run_brain.py,
│   │   │                             parse_telemetry.py, bt_monitor.py,
│   │   │                             make_health_fixtures.py, …
│   │   └── STATUS.md                 module status, build commands, design notes
│   ├── firmware/                     STM32G031 firmware (the robot)
│   │   ├── Core/Src/main.c           superloop, motion, and the brain seam
│   │   └── MDK-ARM/                  Keil project (ARM Compiler 5)
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

**C library + firmware** (from `robot codes/`): GCC 14.2 (MSYS2 MinGW) and Keil's
**ARM Compiler 5** — see `robot codes/BUILD_GUIDE.md`.

```bash
powershell -NoProfile -ExecutionPolicy Bypass -File ./scripts/build_all.ps1
bash scripts/build_firmware.sh
```

**Two compile-time diagnostics, never both in one state.** They are mutually
exclusive by *when* they run, not by macro: the mission stream (`S`/`J`/`Z`/`B`)
only fires while driving, the health stream (`H`/`T`) only while stopped, and one
predicate — `TLM_DRIVING()` — is what separates them.

| Flag | Build | Streams |
|---|---|---|
| *(none)* | normal | nothing; the build that races |
| `USE_MAZE_HEALTH` | `USE_HEALTH=1` | `H`/`T` while stopped — keys, 18 IR, gyro |
| `USE_MAZE_HEALTH` + `HEALTH_ONLY` | `USE_HEALTH=1` | the same, and the robot **cannot start a mission** — the pre-KEY1 loop never exits, so `loop_start` stays 0 for the whole session. This is the bench build; KEY2 = IR calibration, KEY3 = gyro, KEY1 = re-send the banner |
| `USE_MAZE_TELEMETRY` (+ health) | `USE_TELEMETRY=1` | `S`/`J`/`Z`/`B` while driving **and** the health stream while stopped, plus the 5 s per-junction pause (the supervised bring-up build — and this one must be able to run, so it never defines `HEALTH_ONLY`) |

Both streams are read by the same two tools, which go through one parser
(`scripts/parse_telemetry.py`) so they cannot drift: `bt_monitor.py` for live
capture and a virtual-brain decision check, `parse_telemetry.py` for the offline
report. `bt_monitor.py --health` switches the same pipeline to the bench verdict.
In the GUI the picture and the cards are **two independent switches** (`Canvas:
diag | path draw`, `Cards: health | mission`), both following the capture's line
kinds until clicked; `--health` pins both. Since 2026-09-24; they were one
button, which made "health cards up beside the map" unaskable.

The robot also **announces itself**: the `Hi ,mmdi` banner goes out on reset, and
under `USE_MAZE_HEALTH` the USART RX interrupt makes any received byte produce the
same banner, so the monitor app can prove the radio works in *both* directions
rather than inferring it from a stream that has not started yet. That banner is the
only line on the wire that can be a reply.

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

### Brain / firmware seam
8. **The seam is `brain_step()` — a pure decision function.** `BrainIn` in, one
   `'F'/'L'/'R'/'B'` out, the two plan strings on `BRAIN_DONE`. The brain never
   reads a sensor, motor, encoder or compass and holds no firmware pointer; the
   firmware keeps all sensing and all motion. There is no HAL (two integration
   points for one job meant two RAM budgets).
9. **An input built from the same expression as another input is not independent
   of it — and the coupling can make a state unreachable.** The live case:
   `main.c:932` builds `in.front` as `(s[3]||s[4]||s[5]||s[6])`, and
   `main.c:1338-1339` builds the laterals on that *same* term, so
   `left_poss = s[2] && front` and `right_poss = s[7] && front`. Hence **`L=1 ⟹
   F=1`, identically**: `front = 0` forces both laterals to 0, and the input
   `(F=0, L=1)` — a corner — **is unreachable on this hardware whatever the
   sensors do**. Add the stop test at `main.c:1377` (`head_delay` is reset at a
   node, so a node stop must come from `at_node && (left_poss||right_poss)`) and
   a corner must reach the brain as *either* `(F=1,L=1)` — corner read as a
   T-junction — *or* `(F=0,L=0)` — corner read as a **dead end**. Both wrong; only
   the mask's bits 0/9 say which (see `STATUS.md`).
   **When adding a `BrainIn` field, check what it is already implied by, and
   whether the implication makes some real-world state unrepresentable.**
10. **A model-driven host test cannot validate a hardware-derived input, and a
    fixture cannot test the question it was built to assume.** `brain_host.c`
    derives front/left/right from `true_neighbor()` — the maze's real topology —
    so it *can* produce `(F=0, L=1)` (which the hardware cannot) and never
    produces the hardware's corner state; its 7/7 passes on an input the robot
    cannot generate. And `bt_monitor.py`'s replay fixtures are **synthetic**: the
    capture's masks were built from the firmware's own stop condition (`centre
    live AND >=1 lateral`), so replaying them and reporting "front is 1 at 5 of 5
    junctions" only restates the identity in item 9. Both are false-confidence
    traps. Only a real capture tests this.
11. **A command is one STOP, not one graph edge** — "turn as told, then drive
    until you reach somewhere you would stop", so it can carry the robot several
    cells. Consequently the home path is **not a retrace** (it is the shortest
    path on the discovered graph and may leave by a different branch), and the
    home and fast path strings are one continuous stream: the home path assumes
    the current heading, the fast path the heading after the home run.

### Maze rules
12. Mazes are **90° only** — all edges horizontal or vertical. The generator
    enforces this.
13. Coordinates are world units; **Y is up**. `grid_size` defaults to 20.

### GUI / graphics
14. **Dark theme.** Colors are defined as constants at the top of each file
    (`BG`, `PANEL_BG`, `CARD_BG`, `CANVAS_BG`, `FG`, `ACCENT`, `ACCENT2`, …).
    Reuse them; don't hard-code new colors ad hoc.
15. **Tkinter ovals are NOT anti-aliased.** Render circles via **Pillow
    supersampling** (`_circle_image`, drawn at 4× then LANCZOS-downscaled,
    cached by `(fill, outline, r, ow)`). Keep PhotoImage references alive
    (the cache holds them).
16. **Windows DPI:** call `_enable_hidpi()` (SetProcessDpiAwareness) before
    creating the Tk root, and set `tk scaling`, or everything renders blurry on
    high-DPI screens.
17. The **target is a circle** (black fill + red ring), not a square.
18. The side **panel is resizable** (a `PanedWindow` sash) **and scrollable**
    (inner Canvas + Scrollbar + mouse-wheel) so it works on small screens.
    Keep panel content in fixed heights (e.g. command log) so the scroll region
    is correct.
19. **Playback is a real-time multiplier** (`0.25x … 8x`, snapped), not a raw
    delay: at `1x` the animation runs in **true real time** (the frame interval
    is fixed at `FRAME_MS`; each frame advances sim-time by
    `FRAME_MS/1000 · playback`). This is separate from the robot's physical
    **Max speed / Max accel** (cm, cm/s, cm/s²) — never conflate the two.

### Project / workflow
20. Prefer **self-contained files** (the project values single-file scripts) and
    **standard library only** (Pillow optional). Don't add heavy deps casually.
21. The shell here is **PowerShell on Windows** — chain commands with `;`, not
    `&&`. Quote paths with spaces (the repo path has spaces).
22. Don't break `New Start/` or `old docs/`; treat them as read-only history.
23. When unsure about a product decision (data format, scope, UX), ask — the
    user prefers being consulted on meaningful trade-offs.
24. **Always update [`CHANGELOG.md`](./CHANGELOG.md)** in the same turn you
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
- **Corner** — a node where the lane turns 90° **and there is no lane straight
  on**: one lateral exit, `back`, and no `front` — `(F=0, L=1, R=0)` or
  `(F=0, L=0, R=1)`. A straight-through lane makes it a T or a crossing, not a
  corner; `(F=0, L=0, R=0)` is a dead end. Worth naming because the hardware
  cannot currently report this state at all (rule 9).
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
- **Real-robot firmware** — the C solver library in `robot codes/` implements the
  full exploration + proof + fast-run algorithm, and the decision core (`brain.c`)
  is **already wired into `firmware/Core/Src/main.c`**: the brain builds the two
  plan strings, and the firmware's own replay stages drive them.  The legacy
  left-hand-rule explorer was deleted outright (2026-09-23), which is what freed
  the RAM — see `robot codes/STATUS.md`.  Remaining: on-target bring-up and
  encoder calibration.  The `New Start/` tree holds earlier C prototypes
  (read-only reference).
  For bring-up there are now two instruments: the **M1 telemetry** stream for
  what the robot *decided* while driving, and the **health build**
  (`USE_MAZE_HEALTH`) for what the hardware *reads* while stopped.  The health
  view is the static test for the open `in.front` corner defect (rule 9 below):
  park the robot on a corner by hand and read the centre group and the gate off
  the screen, with no drive and no `J` line needed.  (The pad board is the canvas's
  `diag` picture; `--health` pins it.)  Neither can validate the
  brain's *model* of the robot — only the robot can, which is why both exist.
