# SEYED Maze Robot — Algorithms

> Companion to [`ARCHITECTURE.md`](./ARCHITECTURE.md). This document explains, in
> detail, every algorithm used by `simulator/maze solver/maze_solver.py`: how the
> robot explores, how it proves it has the fastest route, how it returns home,
> how the fastest path is computed, and how robot commands (`F/L/R/B`) are
> generated. It ends with the planned acceleration-aware timing model.

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
maze, the moment it can *prove* no undiscovered path could be shorter.

### 2.1 When it may run

Only **after the target has been physically reached** (`target_found == True`).
Before that the robot does not know the target location and may not use it.

### 2.2 The bound

Let `best_known = ` shortest distance `start → target` on the **known map**
(Dijkstra). Any *undiscovered* `start → target` path must, at some point, leave
the known map — and the first node where it does so is some **frontier `f`**.
Therefore its length is at least:

```
LB(f) = dist_known(start → f)  +  ‖ pos[f] − pos[target] ‖
                                   └─ straight-line (Euclidean) distance ─┘
```

- `dist_known(start → f)` — shortest known distance to reach `f`.
- The straight line is an **admissible lower bound**: in any maze a real path is
  never shorter than the straight-line distance, so this never *over*-estimates.

### 2.3 The pruning rule

A frontier `f` is **useful** iff:

```
LB(f) < best_known
```

- If `LB(f) ≥ best_known`, *no* path that first enters the unknown at `f` can
  beat the best known path, so `f` is **pruned** — the robot never drives to it.
- When **no useful frontier remains**, the best known path is provably optimal:
  the robot stops exploring and goes home. This sets **`proven_optimal`** (if
  there were still pruned/unexplored frontiers) or **`fully_explored`** (if it
  had genuinely explored everything).

This is **per-frontier** pruning. A common mistake (an earlier version of this
code) is a *global* check that only stops when *every* frontier is hopeless —
that still lets the robot wander down individual hopeless branches (e.g. a
corridor heading away from the target on the far side). Per-frontier pruning
skips those branches entirely.

### 2.4 Why it's safe (admissibility)

Because `LB(f)` is a lower bound, pruning `f` can only remove paths whose true
length is `≥ LB(f) ≥ best_known`. Such paths cannot be strictly shorter than the
best one already known, so discarding them never changes the optimum. The
returned fastest path is therefore still **provably optimal**.

### 2.5 GUI flag

The solver's **Proof** field reflects this state:

- `searching…` — target not found yet.
- `checking…` — target found, still some useful frontier to verify.
- `PROVEN ✓` — stopped early; fastest route proven with an **incomplete** map.
- `full map` — had to explore everything to be sure.

---

## 3. Returning home

When exploration finishes (no useful frontier), the robot navigates back to
`start` along the **known map**, one edge per step, using the Dijkstra
predecessor tree from the current node. Each step still emits a command, so the
"return" is part of the animated command stream.

---

## 4. Fastest path (current metric: distance)

Once home, the fastest path is computed with **Dijkstra on the DISCOVERED map**
(`robot._known_adj()`), with each edge weighted by its **Euclidean length**:

```
weight(u, v) = ‖ pos[u] − pos[v] ‖
```

Computing it on the *discovered* map (not the true maze) is essential: in the
early-stop case the robot has not seen every edge, and it must not "use" an edge
it never drove. Because of the §2 proof, the discovered-map optimum equals the
true optimum.

The robot then drives this path in the **FAST RUN** phase (animated, with
commands), ending in **DONE**.

> The metric here is **distance**. The acceleration-aware **time** metric is
> future work — see §6.

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

## 6. Future work — acceleration-aware fastest path

The real robot's speed is **not constant**: it accelerates on straights and must
**slow down for turns**. So the truly fastest path minimises **time**, not
distance, and a longer but straighter route can beat a shorter twisty one.

Planned model (for the final real-robot version):

1. **Segment the path** into straight runs separated by turns (L/R/B).
2. **Turn speed limits:** entering/leaving a turn caps the speed (e.g. `B` ≈
   stop-and-turn, `L`/`R` a moderate cap, straight = full speed). A U-turn costs
   the most.
3. **Trapezoidal / S-curve velocity profile** per straight run: accelerate at
   `a_max` from the entry speed, optionally cruise at `v_max`, decelerate to the
   required exit speed before the next turn. Time per run is the area/duration of
   that profile (closed-form given length, entry/exit speed, `a_max`, `v_max`).
4. **Edge/path cost = total time** (sum of run times + turn times), and run a
   shortest-*time* search (Dijkstra/DP) over the discovered graph. Because the
   cost of a node depends on entry **and** exit direction (the turn there), the
   state should be `(node, incoming_heading)` rather than just `node`.
5. **Calibrate** `a_max`, `v_max`, and per-turn speed caps from the real robot's
   encoder/IMU data.

When this lands, only §4's metric changes (distance → time); §1–§3 (exploration,
proof, return) stay the same.

---

## 7. Complexity (per simulated step)

Mazes are small, so this is comfortably fast:

- Priority 1: `O(deg)`.
- Priority 2 / proof / home: one or two Dijkstra sweeps on the known map,
  `O(E log V)` each.
- Whole mission: bounded by the number of moves (each edge driven at most a few
  times) × per-step Dijkstra. Negligible for hand-built mazes.
