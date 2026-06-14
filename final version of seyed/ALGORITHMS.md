# SEYED Maze Robot — Algorithms

> Companion to [`ARCHITECTURE.md`](./ARCHITECTURE.md). This document explains, in
> detail, every algorithm used by `simulator/maze solver/maze_solver.py`: how the
> robot explores, how it proves it has the fastest route, how it returns home,
> how the fastest path is computed, and how robot commands (`F/L/R/B`) are
> generated. It ends with the acceleration-aware timing model that drives the
> (minimum-time) fast run.

---

## 0. Mental model

The maze is an **undirected graph** `G = (V, E)`:

- `V` = intersections (nodes), each with a 2-D position `pos[v] = (x, y)`.
- `E` = traversable line segments. Every edge is **axis-aligned** (90° maze).
- `start` and `target` are nodes. The robot begins at `start` facing nothing
  (heading unknown). It learns `G` only by driving along edges.

Two graphs matter:

- **True maze** `G` — the full graph (the simulator knows it; the *real* robot
  obviously does not).
- **Known map** — only what the robot has driven so far: `visited_nodes` and
  `visited_edges`. **All decisions are made on the known map** (plus the target
  position, but only after the target is found).

The robot's mission runs as a small state machine:

```
EXPLORE  ──(no useful branch left)──►  return home  ──►  FAST_RUN  ──►  DONE
```

---

## 1. Exploration (frontier-based, target-agnostic)

### 1.1 Definitions

- **Unexplored neighbour** of node `u`: a neighbour `v` in `G` such that edge
  `(u, v)` is not yet in `visited_edges`. (In the real robot this is "a line
  leaves this junction in that direction, but I haven't driven it yet".)
- **Frontier**: a *visited* node that still has at least one unexplored
  neighbour. Frontiers are the boundary between known and unknown, drawn as
  **yellow dashed stubs**.

### 1.2 The step rule

Each call to `Robot.step_explore()` makes **one move**, choosing by priority:

**Priority 1 — explore a branch right here.**
If the current node has an unexplored neighbour, drive down one of them. The
choice is **target-agnostic** (the robot doesn't know where the target is yet):

```
rank each candidate branch by the turn it requires, preferring
    straight (F)  <  left (L)  <  right (R)  <  reverse (B)
ties broken by a fixed compass order (N, E, S, W)
```

This mimics a real line-follower: keep going straight when possible, minimise
turns, behave deterministically. (Implemented in `_branch_priority`.)

**Priority 2 — drive to the nearest *useful* frontier.**
If the current node has no unexplored branch, travel (over the known map) to the
closest frontier, measured by **real travel distance** via Dijkstra — not by hop
count. "Useful" matters only after the target is found (see §2); before that,
every frontier is useful.

**Priority 3 — finish and go home.**
If no useful frontier remains, the robot is done exploring. It drives back to
`start` over the known map (one step per `step_explore` call) and, once there,
sets `exploration_complete`.

> **Why explore past the target?** Finding the target does *not* end
> exploration. The robot must be able to *prove* the fastest route, which means
> either mapping everything or proving the rest cannot help (§2).

### 1.3 Comparison with the old `sim_V12_GBF_stable.py`

`sim_V12` used the same Priority 1/2/3 shape but had two weaknesses that this
version fixes:

| | sim_V12 | This version |
|---|---|---|
| Priority 1 tie-break | first neighbour in adjacency list | straight → left → right → reverse (turn-minimising) |
| "Nearest" frontier | **hop count** (BFS) | **real distance** (Dijkstra) |
| Stop condition | always map the **entire** maze | stop as soon as the fastest route is **proven** (§2) |
| Uses target during search | named "GBF" but never used a heuristic | **never** uses target until found (by design) |

---

## 2. The "fastest route proven" early-stop (branch & bound)

This is the key optimisation. It lets the robot stop **before** fully mapping the
maze, the moment it can *prove* no undiscovered path could be faster.

> **The bound is by TIME, not distance.** Because the fast run accelerates and
> stops at turns (§6), a longer-but-*straighter* undiscovered path (fewer stops)
> can be faster than a shorter twisty known one. A distance bound would wrongly
> prune such a path and raise `PROVEN ✓` too early — exactly the bug this section
> now avoids. The bound must therefore use the *same metric the fast run does*.

### 2.1 When it may run

Only **after the target has been physically reached** (`target_found == True`).
Before that the robot does not know the target location and may not use it.

### 2.2 The bound

Let `best_time = ` the minimum-**time** `start → target` route on the **known
map** (`time_optimal_path`, §6.2). Any *undiscovered* `start → target` path must,
at some point, leave the known map — and the first node where it does so is some
**frontier `f`**. Since the robot can never travel faster than `v_max`, that
path's time is at least:

```
LB_time(f) = ( dist_known(start → f)  +  ‖ pos[f] − pos[target] ‖ ) / v_max
               └ known shortest distance ┘    └ straight-line lower bound ┘
```

- `dist_known(start → f)` lower-bounds the length of the path's *known* prefix
  (it ends at `f`, using known edges); the straight line lower-bounds the unknown
  suffix `f → target`. Dividing the total length by `v_max` lower-bounds its time
  (you cannot cover a distance in less time than at top speed). It is therefore
  **admissible** — it never *over*-estimates the true time.

### 2.3 The pruning rule

A frontier `f` is **useful** iff:

```
LB_time(f) < best_time
```

- If `LB_time(f) ≥ best_time`, *no* path that first enters the unknown at `f` can
  beat the best known route in time, so `f` is **pruned** — the robot never
  drives to it.
- When **no useful frontier remains**, the best known route is provably optimal:
  the robot stops exploring and goes home. This sets **`proven_optimal`** (if
  there were still pruned/unexplored frontiers) or **`fully_explored`** (if it
  had genuinely explored everything).

This is **per-frontier** pruning. A common mistake (an earlier version of this
code) is a *global* check that only stops when *every* frontier is hopeless —
that still lets the robot wander down individual hopeless branches (e.g. a
corridor heading away from the target on the far side). Per-frontier pruning
skips those branches entirely.

> Note: a faster robot (larger `v_max`) makes distance "cheaper" in time, so
> `LB_time` is smaller and **fewer** frontiers are pruned — the robot rightly
> insists on checking more straights before it can claim optimality.

### 2.4 Why it's safe (admissibility)

Because `LB_time(f)` is a lower bound on time, pruning `f` can only remove paths
whose true time is `≥ LB_time(f) ≥ best_time`. Such paths cannot be strictly
faster than the best one already known, so discarding them never changes the
optimum. The returned fastest route is therefore **provably time-optimal**.

### 2.5 GUI flag

The solver's **Proof** field reflects this state:

- `searching…` — target not found yet.
- `checking…` — target found, still some useful frontier to verify.
- `PROVEN ✓` — stopped early; fastest route proven (by **time**) with an
  **incomplete** map.
- `full map` — had to explore everything to be sure.
- `off (full)` — the **Optimized search** toggle is OFF, so the proof is
  disabled and the robot maps the whole maze regardless.

---

## 3. Returning home

When exploration finishes (no useful frontier), the robot navigates back to
`start` along the **known map**, one edge per step, using the Dijkstra
predecessor tree from the current node. Each step still emits a command, so the
"return" is part of the animated command stream.

---

## 4. Fastest path (metric: TIME, acceleration-aware)

Once home, the fastest route is computed on the **DISCOVERED map**
(`robot._known_adj()`) by **minimum drive TIME** under the motion model of §6
(`time_optimal_path`). Computing it on the *discovered* map (not the true maze)
is essential: in the early-stop case the robot has not seen every edge, and it
must not "use" an edge it never drove.

The robot then drives this path in the **FAST RUN** phase, animated with a real
trapezoidal velocity profile (`FastRunPlan`): it glides smoothly, accelerating
on clear straights and slowing to a stop at every turn. The traversed portion is
coloured by **speed** (red = fast, green = slow). Alternative slower routes
(found with Yen's K-shortest-by-distance) are drawn lighter so the fastest one
stands out. The phase ends in **DONE**.

> The exploration early-stop proof (§2) bounds by **time** with the same
> `v_max`, so the returned route is provably time-optimal over the *whole* maze,
> not just the discovered part — whether it stopped early (`proven_optimal`) or
> mapped everything (`fully_explored`).

---

## 5. Robot command generation (`F / L / R / B`)

The robot tracks a **heading** as a unit vector:

```
NORTH = (0, 1)   SOUTH = (0, -1)   EAST = (1, 0)   WEST = (-1, 0)   (Y is up)
```

For a move from node `a` to node `b`, the direction is the sign of the position
delta. The command is derived from the previous heading and the new heading:

```
turn_command(prev, new):
    prev is None            -> "F"   (first move, no turn yet)
    new == prev             -> "F"   (straight)
    new == -prev            -> "B"   (U-turn / reverse)
    cross(prev, new) > 0    -> "L"   (left)
    cross(prev, new) < 0    -> "R"   (right)

cross(p, n) = p.x * n.y - p.y * n.x
```

With **Y up**, a left turn from NORTH `(0,1)` goes WEST `(-1,0)`:
`cross = 0·0 − 1·(−1) = +1 > 0 → L` ✔. A right turn NORTH→EAST gives
`cross = −1 < 0 → R` ✔.

Every move (explore, backtrack, return-home, and fast-run) appends a record
`{from, to, cmd, heading}` to the command log. This stream is exactly what the
real robot firmware will consume.

---

## 6. Acceleration-aware motion model (fastest path = time)

The robot's speed is **not constant**: it accelerates on straights and **stops
for turns**. So the fastest path minimises **time**, not distance, and a longer
but straighter route can beat a shorter twisty one. This is implemented in the
simulator (`run_time`, `speed_at`, `time_optimal_path`, `FastRunPlan`).

### 6.1 Velocity profile per straight run

A **straight run** is a maximal sequence of collinear edges between two stops
(the robot is at `v = 0` at both ends, because it stops at every turn — and at
the start and the target). For a run of length `L` with `entry = exit = 0`, a
**trapezoidal** profile is used: accelerate at `a_max` up to `v_max`, optionally
cruise, then decelerate to 0. If `L` is too short to reach `v_max` the profile is
a symmetric triangle. The closed-form time is (`run_time`):

```
d_acc = v_max² / (2·a_max)                       # distance to reach v_max
if 2·d_acc ≤ L:  t = 2·(v_max/a_max) + (L − 2·d_acc)/v_max     # trapezoid
else:            t = 2·√(a_max·L) / a_max                       # triangle
```

and the speed at arc-distance `s` into the run is (`speed_at`):

```
v(s) = min( v_max, √(2·a_max·s), √(2·a_max·(L−s)) )
```

This `v(s)` is exactly what drives the **red→green speed gradient** on the fast
path, and `FastRunPlan` integrates it into a time→(position, speed) trajectory so
the robot animates with real physics.

### 6.2 Minimum-time search (`time_optimal_path`)

Because the robot is stopped (`v = 0`) at every turn, a turn node fully
"resets" its state — so the search state is just the **node**. We run Dijkstra on
a *stop graph* whose edges are straight runs of **any** length between turn
points, each edge costing `run_time(length)`:

```
from node u, for each compass direction d:
    walk straight in d over the discovered map; every node w reached
    is a candidate run-end with cost run_time(dist(u→w))
```

Driving straight through an intersection (no stop) is captured naturally: the
spanning run edge from the previous turn to the next turn exists, so the optimum
never pays for a needless stop. The result is expanded back into a full node list
(including straight-through nodes) for animation and command generation.

### 6.3 Units, alternative routes & GUI

**Units are real-world:** 1 world unit = **1 cm** (grid spacing 20 → 20 cm), so
`v_max` is cm/s and `a_max` is cm/s². Defaults: `v_max = 100`, `a_max = 50`,
cautious explore speed `40`. `yen_k_shortest` (Yen's algorithm, by distance)
surfaces a handful of other routes; each is timed with `path_time` and drawn
**lighter the slower it is**, so the bold dark-green fastest route stands out.
`v_max` / `a_max` are **Max speed** / **Max accel** sliders (re-planning the fast
run live and kept in sync with the §2 proof). An **Optimized search** toggle
disables the early-stop proof when the user wants a guaranteed full-map search.
**Playback** is a separate real-time multiplier (`1x` = real time), independent
of the robot's physical speed.

### 6.4 Open work

- Model turns as a finite speed cap / turn time instead of a full stop, and
  **calibrate** `v_max`, `a_max`, and per-turn caps from encoder/IMU data.

The exploration proof (§2) already shares this model's `v_max` to stay admissible
for the time metric; the rest of §1 and §3 (frontier choice, return) are
unchanged.

---

## 7. Complexity (per simulated step)

Mazes are small, so this is comfortably fast:

- Priority 1: `O(deg)`.
- Priority 2 / proof / home: one or two Dijkstra sweeps on the known map,
  `O(E log V)` each.
- Whole mission: bounded by the number of moves (each edge driven at most a few
  times) × per-step Dijkstra. Negligible for hand-built mazes.
