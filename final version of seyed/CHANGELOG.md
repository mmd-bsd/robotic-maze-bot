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

- **The agent shell is Git Bash (POSIX sh), not PowerShell.** This file used to
  claim otherwise; it was wrong and cost time. Use `export PATH=…`, `$(…)`,
  `&&`, and here-docs. `.ps1` scripts still need
  `powershell -NoProfile -ExecutionPolicy Bypass -File <script>`.
  (The root `CLAUDE.md` is canonical; `final version of seyed/CLAUDE.md` still
  carries the old PowerShell claim and is stale.)
- **Repo path has spaces** (`F:\Robotic fle 2022\SEYED`). Always quote paths.
- **Console encoding is cp1252**: printing unicode like `✓` crashes test scripts.
  In quick scripts call `sys.stdout.reconfigure(encoding='utf-8')` first.
- **Headless test pattern** (no GUI display needed):
  load the module with `importlib`, `root = tk.Tk(); root.withdraw()`, build
  `SolverApp(root)`, then drive `app._frame(dt)` in a loop until `phase=="DONE"`.
  This exercises the full mission + drawing without `mainloop()`.
- **Units are real-world: 1 world unit = 1 cm** (grid spacing 20 → 20 cm).
  Speeds are cm/s, accel cm/s². Don't reintroduce abstract "units/s".
- **A command is one STOP, not one graph edge.** The solver emits a command per
  edge; the robot executes a command per place it comes to rest. The graph
  contains nodes the robot drives through without stopping (the 20 cm provisional
  node, and the lattice point a self-healed edge passes through). Anything that
  turns a node path into robot commands **must reduce it to stop points first**,
  or every command after the first is aimed at a node the robot has already
  passed. `brain.c:_emit_path()` shows the pattern; `MazeRobot.visited_nodes` is
  **not** a valid stop-point test (the solver marks nodes it merely decided to
  drive to).
- **`MAZE_CELL_CM` (`maze_config.h`) is the single source of truth for the lattice
  pitch.** Snapping `dist_cm` to whole cells is a feature, not a tolerated
  approximation: junctions are only on lattice points, so it absorbs encoder
  error rather than accumulating it — and the early-stop proof is geometric, so
  it needs coordinates that really are on the lattice. Expose the residual
  (`drift_cm`) rather than hiding it.
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
- **The firmware can be built without Keil**: `bash scripts/build_firmware.sh`
  drives the real ARMCC 5 (`C:\Keil_v5\ARM\ARMCC\bin`) and prints the Keil
  `Program Size:` line. Use it instead of asking for a build. The Keil project is
  AC5 (`uAC6=0`), **not** armclang — verify with AC5 before assuming a compile works.
- **RAM is the binding constraint on this project, always check it.** The firmware
  alone already uses 93% of the 8 KB (7632/8192), of which 1536 B is the
  `Stack_Size` (1024) + `Heap_Size` (512) reservation from the startup file — and
  both are counted in the Keil size line. The solver still does **not** link:
  measured shortfall **1808 B** after the config was sized to the field (was
  3208 B). Run `bash scripts/measure_solver_ram.sh` before claiming anything fits.
- **An index is not bounded by the array it writes.** The firmware had two
  writers indexing `node[50]` by the *command count* (~70 on the real field) and
  ran 504 B off the end, into `uwTick` and `hi2c2`. When you find an array indexed
  by a counter, check what bounds that counter — nothing in the C type system
  will. Same class: `link[100]` (indexed the same way, and `GTD` follows it).
  `armlink --symbols` gives the real layout, so see what an overflow would
  actually hit instead of assuming it is benign padding.
- **Prefer measuring to estimating for anything size-related.** The exact
  symbol sizes came from `armlink --symbols`, not from adding up declarations;
  the first attempt at that sum was wrong by 60 B.
- Keil's output dir in this project is called **`Source`**, not `Objects`.
- Bash arrays (`"${arr[@]}"`), never space-joined strings, for paths in this repo.

---

## History (newest first)

### 2026-09-23 — The two junction-input questions are answered: inherit the gate, use centre-on-line for `front`

The two inputs `BrainIn` could not be derived from source alone are now decided
by the user, and `inc/brain.h` records them as settled rather than open.

| input | source | decision |
|---|---|---|
| junction gate | `s[0] && s[9]`, main.c:1372/1378 | **inherit as written** — anded, as the source reads |
| `front` | centre group `s[3]`…`s[6]`, main.c:1380 | **`in.front = (s[3]\|\|s[4]\|\|s[5]\|\|s[6])`** |

So the junction declaration the integration writes is the firmware's own
structure, not a replacement for it:

```c
(at node)  = s[0] && s[9];
(junction) = (at node) && (left_poss || right_poss || dead_end);
in.front   = (s[3] || s[4] || s[5] || s[6]);
```

**`front` is still worth measuring.** The proxy is unambiguous at a crossing and
at a T, but at a **corner** it may read "forward open" where there is no forward —
if the incoming arm is still under the centre group when the robot stops. That
matters because `Forward()` is the *line follower*, not "drive straight": a robot
told `'F'` at a corner steers round it while the brain believes it went straight,
and the dead reckoning diverges from there.

The check is one M1 capture at a known corner — the `J` line carries the raw
`front` mask as hex, so the answer is readable rather than argued. **Not settled
by more geometry**: the previous day's attempt to derive sensor behaviour from a
diagram produced a confidently wrong conclusion (see the entry below), and that
is the lesson being applied. If the corner does lie, the fix is that one
expression at the integration site — the brain is unaffected.

No code change; `brain_host` still 5/5 on all five mazes.

---

### 2026-09-23 — Reading the real junction logic: the move mapping is 1:1, and the gate is a position gate

> **CORRECTION (same day, later).** The section below headed "Not good: the junction
> gate must be dropped" was **wrong and is withdrawn.** It assumed `s[0]`/`s[9]` sit at
> the outer extremes of the front row (±45.9 mm apart, a 91.8 mm span). The user
> corrected this: both sit **in the middle of the robot, on the axis of rotation**. They
> are an **arrival** test ("my rotation axis is over the node — decide now"), not a
> "black on both sides" test, and they are satisfied at **every** node type — crossing,
> T and corner alike. So the gate is *exactly* what the brain wants and can be
> inherited as-is; a junction is declared on
> `(s[0] && s[9]) && (left_poss || right_poss || dead-end)`. The withdrawn table, the
> "91.8 mm" arithmetic, and the claim that the legacy explorer "turns only at 4-ways"
> all rest on the bad premise. The *good* half of the entry — the 1:1 move mapping and
> `path_append()` as the call point — is unaffected and stands. See SENSORS.md §2 for
> the corrected geometry.

Read `firmware/Core/Src/main.c:1353-1525` (the real discovery block) against the
brain, on the user's instruction to feed the brain from it. One result stands, one
was **withdrawn the same day** after the user corrected the sensor geometry — read
the correction banner below before the second half.

#### Good: the moves map 1:1, so the integration is nearly free

The firmware already has exactly the four actions the brain emits, selected by
`cross`, and the dispatch is already written:

| brain | `cross` | action | main.c |
|---|---|---|---|
| `'F'` | 0 | `Forward()` / `Forward_r()` | 1362 |
| `'L'` | 1 | `turn_left()` / `turn_left_r()` | 1367 |
| `'R'` | 2 | `turn_right()` / `turn_right_r()` | 1372 |
| `'B'` | 4 | `head` flip + `nav+=2` | 1389 |

So the integration replaces only the *choice* of `cross`. **Nothing between the
decision and the motors changes.**

`path_append()` (`main.c:939`) is the call point: it is the single choke point all
eight junction decisions pass through, and it is where the link length is recorded.
The brain is invoked just before the decision, because it needs `dist_cm` to place
the node — the same number `path_append()` computes from the encoder delta since the
`on_link==0 → ResetEncoder` at the top of the link (1359-1364), run through the
`/(2.467*2)` and the per-command corrections at 951-972. That arithmetic should be
factored into one helper called from both places; the corrections (+85/+95 after a
turn, +25/+30 after a straight, +104 after a 'B') are empirical and must not drift.

`in.left`/`in.right` come straight from `left_poss`/`right_poss` (1366-1367),
`in.target` from `OnEndZoon` (1320). Both are read **before** the motion primitive,
which clears them (602-603 etc.) — the rule SENSORS.md already records.

#### ~~Not good: the junction gate must be dropped, not inherited~~ — WITHDRAWN

> **Everything from here to "The one thing left unresolved" is withdrawn.** It was
> derived by placing `s[0]`/`s[9]` at the outer extremes of the front row. They are
> not there — they are on the rotation axis (see the correction banner above). The
> block is a clean left-hand rule and the gate is a position/arrival test that fires
> at every node type, so it *can* be inherited. Kept only so the reasoning is not
> silently lost; do not act on it.

The firmware only ever chooses a turn when **both outer front sensors see line**:

```c
if (left_poss && (s[0] && s[9]))                        { cross=1; ... }   /* 1372 */
else if (left_poss==0 && right_poss==1 && s[9] && s[0]) { ... }            /* 1378 */
```

`left_poss`/`right_poss` are set for any lateral (1366-1367); the `s[0] && s[9]` gate
is a second, separate condition. At the 10.2 mm pitch that is a **91.8 mm** span
(SENSORS.md §3), so a branch on **one side only** cannot satisfy it.

Sampling the bar at the node centreline, with a 20 mm line, the black under the row
is:

| node type | black at the bar | `left_poss` | `right_poss` | gate |
|---|---|---|---|---|
| straight N+S | `\|x\| <= 10` | 0 | 0 | no |
| corner S+W | `x <= 10` | 1 | 0 | **no** |
| T S+N+W | `x <= 10` | 1 | 0 | **no** |
| T S+N+E | `x >= -10` | 0 | 1 | **no** |
| 4-way | everything | 1 | 1 | **yes** |

So during discovery the gate passes **only at a 4-way**, and the only other report is
`'B'` at a dead end — `'S'` needs `left_poss==0` and `'R'` needs the centre group
*off* the line, neither of which happens at a 4-way. **The legacy explorer turns only
at 4-ways, goes straight at every T, and records nothing at either.**

That is not a bug: it is a coherent left-hand-rule walk, and the branch it declines
to take is one the rule had already decided against. But it is **the wrong junction
test for the brain**, which needs every real junction reported or it will never learn
a one-sided branch and will claim `fully_explored` with edges missing. The brain must
declare a junction on `left_poss || right_poss || dead-end`. The gate is part of the
left-hand-rule logic being *replaced*, not part of reading the sensors.

> **^ Withdrawn** (see the banner). The gate is a position gate on the rotation axis,
> fires at every node type, and is inherited unchanged. The junction test the brain
> uses is `(s[0] && s[9]) && (left_poss || right_poss || dead-end)` — which is the
> firmware's existing structure, not a replacement for it. One sub-question is still
> open: whether the pair is **ANDed** (as the source reads) or **ORed** (the user is
> unsure what the hardware does). The M1 capture settles it.

#### The one thing left unresolved: `front`

`centre-on-line` (`s[3]||s[4]||s[5]||s[6]`, the firmware's own proxy at 1380) is
correct at a crossing and at a T, and **wrong at a corner**: the corner's own arm is
under the centre, so the proxy reads "forward open" where there is no forward. This
matters because `Forward()` is not "drive straight" — it is the *line follower*, so a
robot told 'F' at a corner will steer round the corner while the brain believes it
went straight, and the dead reckoning diverges.

Derived from geometry, not observed — so it wants measuring. The M1 telemetry build
already logs exactly what settles both questions:

```
J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>
```

`<front>`/`<rear>` are the raw sensor masks as hex, so the black pattern at every
real junction is recoverable, and `<L>`,`<R>`,`<cross>` say what the legacy logic
concluded from it. One capture turns both open questions from reading into
measurement.

#### Also fixed

- **SENSORS.md's citations were stale.** It cited `main.c:1202-1214` for the junction
  decision — that region is now commented-out I2C scan code; the decision is at
  1366-1407. The `turn_left()` clears moved `571-572` → `602-603` and the rest
  likewise. Corrected.
- **`nav` is updated only by explicit turns** (`turn_left` 614, `turn_right` 686,
  U-turn 1434 — `Forward()` touches it not at all), and `node[]` is accumulated along
  `nav` (1550-1553). So any turn taken without an explicit primitive leaves `nav`
  stale. Recorded in SENSORS.md; it does **not** affect the solver, which keeps its
  own heading and updates it from the moves it issued.
- `inc/brain.h` gained a "HOW TO FEED IT FROM THE FIRMWARE" section: the mapping
  table, the exact expressions, the call point, and the two open questions.

No code changed. All five brain host tests still 5/5.

---

### 2026-09-23 — The decision core (`brain.c`): the solver behind a junction-report interface

**The problem this solves.** The solver library assumes it *drives* the robot: it
emits one command per graph edge and expects the caller to be a virtual robot
that teleports along the path it plans. On the real field that is the wrong
seam. Line following, junction detection, target detection and the encoder
already work in the firmware, and they are the hard-won parts. What SEYED
actually needs from this codebase is the **decision**, not the driving.

`inc/brain.h` + `src/brain.c` are that seam, and nothing more:

```
  robot                                     brain
    |  drives to the next junction           |
    |---- BrainIn{exits, target, dist_cm} -->|
    |<------- 'F' | 'L' | 'R' | 'B' ---------|
    |<------- BRAIN_DONE --------------------|
    |  brain_home_path() -> "BR"             |
    |  brain_fast_path() -> "BFFLRLF"        |
```

`BrainIn` is six fields: four relative exits, a target flag, and a driven
distance. The brain never reads a sensor, a motor, an encoder, or a compass.
"Left"/"right" are always relative to the robot's own facing, so the caller does
no direction arithmetic and never needs to know which way is north — the brain
defines its frame's north as wherever the robot points at boot.

**`brain.c` is a deliberate adapter, not a reimplementation.** Every algorithm
in it already existed and already passed its tests: home is
`maze_graph_shortest_path()`, the fast run is `maze_fastrun_build_plan()`, the
search is `maze_explore_step()`. The brain adds only the three things those
modules do not do — dead reckoning, the command-granularity reduction, and the
two command strings.

#### `MAZE_CELL_CM` — one lattice pitch, two consumers

Added to `maze_config.h` as the single source of truth for the cell size, and
`PLACEHOLDER_DIST_CM` in `maze_solver.c` now derives from it instead of repeating
`20`. `brain.c` uses it to snap `dist_cm` to whole cells.

Rounding to the nearest cell is **the point, not an approximation we tolerate**: a
junction is only ever on a lattice point, so snapping removes encoder error
instead of accumulating it, and the early-stop proof is geometric and needs node
coordinates to actually *be* on the lattice. The residual is exposed as
`BrainStatus.drift_cm` so a real calibration fault stays visible to the operator
rather than being silently absorbed forever.

The provisional-node concern turns out to be self-healing: `maze_hal`-style
`maze_solver_update_position()` adds an edge from the node the robot was at to the
node it *actually* arrived at, with length from coordinates. A 40 cm run through
an unstopped passthrough becomes `A —20— P —20— B` where `P` is the real lattice
node. **Zero new graph code was needed.**

#### Four design bugs, all found by the harness rather than by reasoning

This is the entry worth reading. Each of these produced a plan that *looked*
plausible and was silently unexecutable.

**1. One command is one STOP, not one graph EDGE.** The solver emits a command per
edge. The robot executes a command per place it comes to rest. The graph contains
nodes the robot drives straight through — the 20 cm provisional node, and the
lattice point a self-healed edge passes through — so a plan of `n` edges can need
far fewer than `n` commands. The home route planned as `BFR` (3 commands, ending
at a passthrough the robot would never stop at) when the executable plan is `BR`.
Every command after the first was aimed at a node the robot had already passed.

`_emit_path()` reduces a node path to its stop points first. **This is safe
precisely because a node the robot does not stop at is provably a straight
passthrough, hence collinear** — if it had a lateral branch the firmware's side
sensors would have fired and the robot *would* have stopped. Dropping it leaves a
straight leg of the same shape and length.

**2. `visited_nodes` is not a stop-point test.** The first fix used
`robot.visited_nodes[]` to identify stop points. Wrong: `maze_robot_move_to()`
sets `visited_nodes[next_id] = true` for a node the solver has merely *decided to
drive to*, including the provisional nodes. That reintroduced the very nodes the
reduction exists to drop — `BR` came out as `BFR`. The brain now keeps its own
`s_stopped[]`, written only from a real report.

**3. Calling `maze_solver_step()` corrupts the pose.** It signals "exploration is
over" by *transitioning* — computing the route home and immediately starting to
drive it, which calls `maze_robot_move_to()` and rewrites `heading`,
`current_node`, `current_coord` and the visited flags. Everything the plan
generation needs to read. The home route came out planned from the wrong pose and
the first turn was silently wrong. Fixed by calling `maze_explore_step()` directly
and treating `MAZE_CMD_NONE` as the end — with a guard for a pending path, because
`NONE == BRAIN_DONE == 0`.

**4. The first move was forced FORWARD.** `maze_explore.c:119` derives
`is_first` from `heading == MAZE_HEADING_NONE`, and
`maze_robot_compute_command()` answers `is_first` with an unconditional FORWARD —
it assumes a robot that has not moved yet already points the right way, which in
the solver's own virtual drive it does. A real robot does not. On
`sample_maze3.json`, whose start node's only exit is lateral, the brain said `F`
and drove into a wall. `brain_init()` now sets `heading = MAZE_NORTH` so the first
turn is computed like any other. **Not fixed in `maze_robot.c`**: there `is_first`
is a deliberate contract for the virtual drive and the existing tests rely on it.

Bug 4 is the argument for testing more than the field maze. `real_field.json`
happens to have a start node with a forward exit, so it passed while three of the
four bugs were still live.

#### `test/brain_host.c` + `scripts/run_brain.py` — a robot, not a replayer

The existing runners (`run_maze.c`, `integration_test.c`) both "drive" by calling
`reveal_node()`, which reads the TRUE maze and hands the solver every neighbouring
edge at its true coordinate. That skips the sensor-driven discovery path entirely
— including `_discover_branch()` and its placeholder — which is exactly the path
the real robot exercises.

`brain_host.c` models a robot instead:

- it stops **only where the firmware's junction test would fire** (a side sensor
  on the line, or a dead end), mirroring `main.c:1202-1214`; a pure straight
  passthrough is driven through **without a report**, so the brain must
  reconstruct it from `dist_cm` — the case that would break a naive
  implementation;
- it reports only what sensors can see: exits relative to its own facing, a target
  flag, a driven distance. Never a coordinate, a node id, or a compass direction.

Then it checks the result **without asking the brain to grade its own work**:
both plans are replayed on the true maze from the true pose (home must land on
start, fast on target, every step on a real edge), and the fast path's time is
compared against the time-optimal cost over the **entire** true maze — the maze
the brain was never allowed to see.

#### Result

| maze | reports | cells driven through | home | fast | fast time | full-maze optimum |
|---|---|---|---|---|---|---|
| `real_field` | 47 | 11 | `BR` | `BFFLRLF` | 6.50 s | 6.50 s |
| `sample_maze` | 24 | 3 | `BL` | `BFLRLF` | 6.50 s | 6.50 s |
| `sample_maze2` | 24 | 3 | `BL` | `BFLRLF` | 6.50 s | 6.50 s |
| `sample_maze3` | 39 | 41 | `LL` | `BRLLR` | 9.14 s | 9.14 s |
| `sample_maze4` | 26 | 34 | `RRLRRLLRRLLF` | `BLRFRRF` | 12.30 s | 12.30 s |

**5/5 checks on all five mazes.** On the real field the brain drives through 11
cells without a report, finds the target, and its fast-path time **equals the
time-optimal cost of the full maze** — a maze it was never allowed to see. That
equality is the whole claim `proven_optimal` makes, and it is what validates the
`dist_cm` + lattice-snapping + self-healing design and the time-based early-stop
proof together.

The other 21 tests are unaffected (`test_graph` 6, `test_robot` 7,
`integration_test` 8) and `test_hal_compile.exe` still passes its 10.
`run_brain.py` treats gcc warnings as failure; the new code compiles clean under
`-std=c11 -Wall -Wextra -pedantic`.

#### RAM

The brain's statics are **2446 B**: `MazeGraph` 1288 + `MazeRobot` 688 +
`s_stopped` 64 + the two command strings 386 (sized from
`MAZE_COMMAND_LOG_SIZE`, not `MAZE_MAX_PATH_LENGTH` — a path is a list of NODES
but a plan is a list of COMMANDS, and a mission makes ~2.6 commands per node;
sizing by path length would silently truncate). `_finish()`'s two path buffers
are 256 B of stack.

That is *more* than the solver the HAL bridge instantiates, and the firmware is
already at 7632 / 8192 B before either. `measure_solver_ram.sh` still reports
**DOES NOT FIT — 1808 bytes short**. Integration remains RAM-blocked until the
legacy `node[]`/`link[]`/path arrays are retired.

**Files:** `inc/brain.h` (new), `src/brain.c` (new), `test/brain_host.c` (new),
`scripts/run_brain.py` (new), `inc/maze_config.h`, `src/maze_solver.c`.

---
### 2026-09-23 — M0 memory work: config sized to the field, and a real `node[]` overflow fixed

Three changes, in ascending order of importance. The third one is the headline:
it is a **pre-existing memory-corruption bug in the working firmware**, not a
consequence of the solver port.

#### 1. `maze_config.h` sized to the actual field (not the simulator's worst case)

The field is a fixed **8 x 7 = 56 lattice points**; `real_field.json` uses 38 of
them with 41 edges. The solver was configured for 80 nodes / 160 edges.

| define | was | now | why |
|---|---|---|---|
| `MAZE_MAX_NODES` | 80 | **64** | 56 lattice points + ~14% for the placeholder nodes the solver sprouts at detected branches |
| `MAZE_MAX_EDGES` | 160 | **112** | a fully-connected 8x7 grid has 8x6 + 7x7 = 97 edges |
| `MAZE_MAX_PATH_LENGTH` | 80 | **64** | a shortest path never revisits a node, so it tracks NODES |
| `MAZE_COMMAND_LOG_SIZE` | 256 | **192** | measured: real field = 70 commands, worst sim maze = 97 |
| `MAZE_MAX_FRONTIERS` | 24 | **24 (unchanged)** | see below — deliberately NOT shrunk |

Measured with `bash scripts/measure_solver_ram.sh`:

```
shortfall to make it link:  3208 B  ->  1808 B      (1400 B better)
```

**Why this is not cosmetic — the stack.** `maze_time_optimal_path()` in
`maze_fastrun.c` keeps five `MAZE_MAX_NODES`-sized arrays live at once
(`best_time` float, `prev_turn`, `settled`, `turn_path`, plus `expanded`). At 80
nodes that is a **single 880-byte stack frame** against the 1024-byte
`Stack_Size` in `startup_stm32g031xx.s`. A stack overflow on Cortex-M0+ is
**silent corruption, not a link error** — the link would have succeeded and the
robot would have misbehaved inexplicably on the field. At 64 nodes the same
frame is 704 B. `maze_graph_dijkstra`'s `MinHeap` (also a stack local, 8 B/node)
drops 640 -> 512 B with it.

**`MAZE_MAX_FRONTIERS` was left at 24 on purpose.** Shrinking it to 16 saves
32 B. But its only writer, `maze_robot_find_frontiers()`
(`src/maze_robot.c:175`), **truncates silently** — no error, no count flag — and
the truncated list is what `maze_explore.c` hands to the early-stop proof. A
frontier dropped by truncation can make the proof conclude that nothing useful
remains and stop the search early, returning a **non-optimal route while
reporting `proven_optimal`**. That is the exact failure the proof exists to
prevent, and it would be silent. Not worth 32 bytes. **Do not "optimize" this
later without fixing the truncation first.**

#### 2. `build_firmware.sh` advertised a map file that never existed

The script printed `Map: .../build/firmware/seyed.map`, but AC5's `--map` takes
**no argument** and creates **no file** — it prints the memory map to *stdout*,
and `--map=<file>` is rejected (`L3916U: Unexpected argument for option
'--map'`). The script now persists stdout to that path, so the printed path is
true (and true in the link-failure message too). Also fixed a stale mode string
that still said "50 Hz" for the M1 telemetry dump (it is 125 Hz).

#### 3. `node[]` overflowed the array — up to 504 bytes past the end

**This is the important one, and it is not caused by the solver port.**

`int node[50][6]` is indexed by the **running command count** (`path_c`), not by
the array size. Two live writers, neither with a bounds check:

| writer | index | where |
|---|---|---|
| `fill_map()` | `node[path_c + 1]` | called from `path_append()` before `path_c++` |
| `loop_start == 2` (reach end zone) | `node[i + 1]`, `i < strlen(path_discoverd)+1` | node dump |

A full mission on the real field issues **~70 commands**, so these write
`node[51]`..`node[71]`. Read the actual layout out of the linker map
(`armlink --symbols`) rather than guessing:

```
node[]   0x200000CC .. 0x2000057C   (1200 B, 50 rows)
 0x2000057C  IR_ReadCounter     <-- first victim, from the 50th command
 0x2000057E  adcv
 0x20000590  uwTick             <-- the HAL 1 ms tick
 0x20000594  SystemCoreClock
 0x200005AC  hi2c2              <-- the IMU's I2C handle
 0x20000600  BLT_TX_Buffer
```

So an over-long mission silently corrupted the HAL tick and, further out, the
IMU bus. Classic "robot behaves inexplicably". It was **live in the M1 telemetry
build** — i.e. in the hex that was about to be flashed.

`link[100][4]` has the identical failure mode (also indexed by `path_c`), and
whatever follows it is worse: **`GTD`, the servo structs holding
`IncrementalEncoder`** — the encoder values every length measurement depends on.

**Fix — bounds guards, chosen over a redesign (see the decision note below):**

- `main.c:134-135` — `NODE_ROWS` / `LINK_ROWS` derived from the arrays with
  `sizeof`, so they cannot drift if either array is resized.
- `main.c:851` — `fill_map()` returns early when `path_c + 1 >= NODE_ROWS`.
- `main.c:1548` — the dump loop gained `&& i + 1 < NODE_ROWS`.
- `main.c:945` — `if(on_link==1)` became `if(on_link==1 && path_c < LINK_ROWS)`.

Verified on the host (`build/guardtest/t.c`, a replica of both writers):
unguarded reaches `node[70]` / `node[71]` — **504 bytes past the end**; guarded
stops at `node[49]`, **0 bytes past**. Boundary predicates asserted at each step.

**The guards are unconditional, not behind `#ifdef USE_MAZE_SOLVER`** — the bug
is in the legacy firmware too, so gating the fix on the solver build would leave
it live. Cost: **+44 B flash / +0 B RAM** (plain build 34836 -> 34880). The one
compiler warning is unchanged (`dir_string` set but unused, pre-existing).

**Consequence, recorded deliberately:** the legacy map now stops at `node[49]`.
Missions longer than that still drive correctly — the guard only skips
bookkeeping — but `node[path_c][0..1]` goes stale for `path_c >= 49`, and
`maze_hal.h` reads exactly that for its position feed. **So the solver needs its
own position source before it can take over (M2).** Fixing memory safety first
was deliberate: a wrong map is debuggable, a corrupted HAL tick is not.

#### Two corrections to earlier claims in this project

Recorded because both were stated confidently and both were wrong:

1. **"Retiring `link[]` + `node[]` frees 3412 B."** Wrong — they cannot simply be
   deleted. `maze_hal.h`'s integration contract (rule 2) depends on
   `link[path_c][0]` for the solver's grid snap, and its design notes read
   `node[path_c][0..1]` for position. Only the legacy *path strings*
   (`path_discoverd`, `path_discoverd_s`, `result_1`, `result_2`, `path_back`,
   ~800 B) are genuinely removable.
2. **The 3208 B shortfall was attributed to the legacy arrays alone.** The
   measured figure was right, the attribution was incomplete: the solver's own
   config was over-sized for this field, which is why fix #1 removed 1400 B
   before any legacy code was touched.

#### Where M0 stands now

Still **1808 B short** of linking the solver. The remaining options were
presented as a fork and the **"bound the writes first"** option was chosen — stop
the corruption, leave behaviour otherwise identical, and defer the RAM
architecture (grow `node[]` to cover the field vs. drop the legacy dead-reckoning
entirely and let the solver's graph be the position source). The latter is the
cleaner root fix and frees ~3400 B, but it changes the `maze_hal.h` contract, so
it is M2 work, not M0.

**Files touched:** `robot codes/inc/maze_config.h`, `robot codes/scripts/build_firmware.sh`,
`firmware/Core/Src/main.c`, this file, `robot codes/STATUS.md`, `robot codes/BUILD_GUIDE.md`.

**Regression state:** 31/31 solver tests pass (6 graph + 7 robot + 8 integration +
10 HAL); all five mazes pass including the 56-node boundary case
(`sample_maze3`); firmware links at 34880/7632 (plain) and 35580/7704 (telemetry).

### 2026-09-23 — `final version of seyed` is now its own repo (pushed, private)

Split out on request so the active work has a standalone remote.

- **Remote:** `https://github.com/mmd-bsd/seyed` — **private**, default branch `main`
- **First commit:** `5c9bfe9`, 61 files / 17026 insertions (377 files on disk)
- **Branch naming:** `gh repo create` defaults to `master`; renamed to `main` to
  match the parent repo, then the stale remote branch was deleted.

**The parent repo still tracks these same files, on purpose.** "Keep both
tracked" was chosen over untracking or a submodule.

This entry first claimed that the parent would now record a **gitlink** instead
of the files. **That was wrong**, and it is worth recording that it was, because
"embedded repo → gitlink" is the intuitive answer. Checked rather than assumed
with `git add -n "final version of seyed"` in the parent: it stages the
individual files, and prints no embedded-repository warning. The reason is that
those paths are *already in the parent's index* — git keeps tracking them one by
one. A gitlink would only appear if they were first untracked
(`git rm -r --cached "final version of seyed"`), which is exactly the change that
was declined.

So the real consequence is **drift, not a broken index**: two independent copies
of the same files now live in two repos, and committing to one does not update
the other. `robot codes/{BUILD_GUIDE,STATUS,SENSORS}.md`, `CHANGELOG.md` and
`firmware/` have all already been edited on both sides this session.

**What is not in the new repo**, by design — it reuses the parent's `.gitignore`
verbatim (its patterns are all `**/`-anchored, so a repo rooted at
`final version of seyed` interprets them identically):

| Excluded | Why | Size |
|---|---|---|
| `firmware/Drivers/` | STM32 CMSIS + HAL vendor code | 20 MB |
| `robot codes/build/` | compiled test binaries | 3.8 MB |

The index stores LF throughout (`core.autocrlf=true` only affects the Windows
checkout) — verified with `git cat-file` on `build_firmware.sh`, so the shell
scripts stay runnable on any platform.

### 2026-09-23 — M1 bench telemetry: the firmware can now measure the robot

**Goal:** the three numbers the solver port is blocked on cannot be derived from
anything in this repo — they only exist on the physical robot:

1. **counts per 20 cm cell** — calibrates the whole map
2. **which sensors actually fire** at a junction — validates `SENSORS.md`
3. **how long the branch detector stays live** — the decision's timing budget

**What was built:** `USE_TELEMETRY=1 bash scripts/build_firmware.sh` produces a
build that streams all three over the existing Bluetooth link, plus
`scripts/parse_telemetry.py` to turn a capture into numbers.

**Why this and not the solver wiring:** the telemetry build **does not change how
the robot drives** — the left-hand rule is untouched — and it fits today:

| Build | Flash | RAM |
|---|---|---|
| firmware as-is | 34836 (53%) | 7632 (93%) |
| firmware + telemetry | 35560 (54%) | 7704 (**94%**) |

+72 B RAM, +36 B flash. With the macro undefined the build is **byte-identical**
to the documented baseline (Code=34188, ZI=6208 — verified, not assumed). So M1
flashes *before* the solver's RAM retirement, which is the rest of M0. That
ordering was deliberate: M1 needs no solver code, and its output is what makes
M2 trustworthy.

#### Design decisions worth keeping

- **`path_append()` is the single hook.** It is the one function every junction
  decision passes through — four call sites in the front-bank block, four in the
  rear-bank block, plus the `'D'` the target latches. Instrumenting the decision
  sites instead would have meant eight hooks and eight chances to miss one.
- **`<raw>` is the unconverted encoder count, emitted beside `<cm>`.** The
  firmware computes `raw/(2.467*2) + offset` with five different empirical
  offsets. Emitting both lets the divisor and the offsets be checked
  *separately* instead of inferred from one final number. That is the whole point
  of M1.
- **Junction lines are queued, not sent.** `BLT_SendData()` restarts the TX DMA
  (`Hardware.c:135`), so a second call before the first has drained silently
  truncates it. With a stream running, junction records would have been eaten
  some of the time — and those are the records that matter. Events park in a
  64 B buffer and the 8 ms slot sends *either* the event *or* a stream sample.
- **Stream at 125 Hz, not the 50 Hz `Task20Ms` cadence.** The branch window is
  ~20 ms, so a 20 ms sample period could not resolve it at all. 8 ms gives 2-3
  samples across a window at ~40% of a 115200 link.

#### What was learned

**8 ms sampling still cannot measure a 20 ms window properly.** It resolves
*which* sensors fire and roughly how many samples the window spans — an upper
bound, not a measurement. For a true duration the right instrument is an on-chip
ring buffer dumped after the run, not a faster radio link. `parse_telemetry.py`
prints this caveat in its own output rather than papering over it, and M2 should
not treat those spans as precise.

**The two-bank split is confirmed structurally.** `path_append()` being the
single choke point for both banks is independent evidence for the 10/8 split in
`SENSORS.md`: each bank has exactly four decision paths, which is what a
symmetric ±2.5-pitch detector pair per bank implies.

**An encoder cross-check is now possible.** `2.467 counts/mm/wheel` becomes
4.934 counts/cm summed, i.e. **98.68 counts per 20 cm cell**. The parser
estimates the true quantum from the raw counts alone and prints the firmware's
assumption next to it, so the first real capture immediately shows whether 2.467
is right — and every link whose cell count is not near a whole number is one
where an offset is absorbing an error instead of describing it.

#### Also

- `parse_telemetry.py` was validated on a synthetic capture in the verified wire
  format, degenerate inputs included. The *format* is verified; the *physics* is
  not — only the robot can confirm the numbers.
- Fixed this file's stale shell note (it claimed PowerShell). See Conventions.
- Corrected the sensor-role table in the entry below: `s[1]`/`s[8]` are **target
  detectors**, not merely "not branch detectors", and the banks are **10/8**,
  not 9/9. `SENSORS.md` was corrected earlier in the session; this file was
  still carrying the old wording.

**Files:** `firmware/Core/Src/main.c`, `robot codes/scripts/build_firmware.sh`,
`robot codes/scripts/parse_telemetry.py` (new), `robot codes/BUILD_GUIDE.md`,
`robot codes/STATUS.md`.

---

### 2026-09-23 — Sensor map nailed down, and the real test field is now a runnable maze

**Goal:** the two images the user supplied — sensor numbering on the PCB, and the
maze the robot will actually be tested on. Both turned out to need more than a look.

#### 1. The sensor map is now authoritative (`robot codes/SENSORS.md`, new)

Until now the roles of `s[0..17]` were *derived from the line-following code* —
nobody had written down which physical sensor each index is. That is exactly the
kind of thing that is silently wrong for months, so it is now pinned to source:

```
s[i]  ≡  silkscreen  S<i>     for every i in 1..8 and 10..17
```

The firmware index and the PCB silkscreen are **the same number**. That falls out
of `Hardware.c`'s `Read_IRSensors()`, whose MUX-counter assignment (counters 0-7 →
S5, S4, S3, S6, S7, S1, S8, S2) looks scrambled but exists precisely to make the
`S` numbers come out in order. Only `s[0]`/`s[9]` break the pattern — they are the
two pads wired straight to ADC pins instead of through the MUX.

Roles, from the junction decision at `main.c:1366-1407` (cited as 1202-1214 when
this entry was written — that region is now commented-out I2C scan code) and the
target test at `main.c:1167`:

| index | front / rear | lateral | role |
|---|---|---|---|
| 2 / 11 | | ±2.5 pitch | **left** branch detector |
| 3,4,5,6 / 12,13,14,15 | | ±1.5, ±0.5 | on-line centre group |
| 7 / 16 | | +2.5 pitch | **right** branch detector |
| 0 / 9 | | **on the axis** | ~~−4.5 / +4.5~~ — **corrected: both sit in the middle of the robot, on the rotation axis** (user-measured). They are the arrival/position pair (`s[0] && s[9]` = "my axis is over the node"), not outer extremes. See SENSORS.md §2 and the 2026-09-23 correction banner. |
| 1,8 / 10,17 | | ∓3.5 pitch | **target detectors** (half the all-black row test) — not branch detectors |

**The banks are not the same size** — front is `s[0]`…`s[9]` (10) and rear is
`s[10]`…`s[17]` (8), not 9/9. The MUX explains the count (one 8-channel MUX serves
`s[1]`…`s[8]` and `s[10]`…`s[17]`; `s[0]`/`s[9]` are the two left over and got
dedicated ADC pins), and the membership is forced by symmetry: front centre sits at
4.5 and rear at 13.5, and both rows put their branch detectors ±2.5 pitches from
their own centre. Under a 9/9 split the front centre would be 4.0 and `s[3]`…`s[6]`
would be lopsided about it.

**Consequence for `maze_hal.h`:** none — and that is worth recording so nobody
"fixes" it later. The HAL never indexes `s[]` for branch decisions; it reads the
firmware's latched `left_poss`/`right_poss` (`maze_hal.h:148-151`), so it inherits
the firmware's branch logic exactly. Its only raw indices are the centre groups
`s[3..6]`/`s[12..15]`, which are right under either split. The rule to keep: do not
add raw index reads for branch detection — wiring `s[0]`, `s[1]`, `s[8]` or `s[9]`
in as branch detectors would sprout phantom edges at every node.

`s[1]`/`s[8]` are also why the target test needs a **71.4 mm**-wide black area; the
real disc is 128 mm, so there is 28 mm of margin each side.

What is *not* recoverable from any file in the repo is the physical geometry, so that
was measured on the real board. **Geometry is now closed — nothing there blocks M2:**

| Quantity | Value | Source |
|---|---|---|
| `s[2]` → `s[7]` (active array) | 51 mm over 5 gaps → **pitch 10.2 mm** | measured |
| active array `s[2]`…`s[7]` | **51 mm**, branch detectors at **±25.5 mm** | derived |
| centre group `s[3]`…`s[6]` | 30.6 mm | derived |
| **robot centre → front sensor line** | **57 mm** | measured |

Cross-checked against the field: a 20 mm track and a 200 mm cell mean the 51 mm
active array is 2.5 line-widths — 2-3 sensors hold the line at once and the array
sits comfortably inside a cell. The geometry does not fight the solver.

The **57 mm lead** is the number with consequences. It cancels out of link lengths
(both ends of a junction-to-junction encoder read are taken at the same bar
position — which is why the lengths already look right), but it is the physical
origin of the `+25/+30/+85/+95/+104` offsets in `path_append()`, and it is the turn
budget in the fast run: the robot commits to a turn with the bar at the node and
finishes it over the next 57 mm. At 100 cm/s that is 57 ms of travel per junction —
so the sensor detection window for a branch is ~20 ms, and the whole junction
decision rests on latching `left_poss`/`right_poss` on one good sample. It does, and
the existing left-hand firmware relies on exactly that.

One measurement still does not reconcile: `s[1]` → `s[17]` = 76 mm crosses banks
(`s[17]` is rear), and the arithmetic that fits it best puts the two rows only 26 mm
apart, which would make front/rear nearly redundant. **This is not blocking** — the
HAL only needs to know *which* bank is leading, not how far apart they are. Recorded
in `SENSORS.md` §3 for when it matters.

#### 2. The real field runs in both solvers (`simulator/mazes/real_field.json`, new)

`robot and field data/field.png` is the actual test maze. Reading it out:

- It is a **line maze** — the black lines are the *tracks*, not walls. Proof: the
  target disc (radius ≈ 20 px) is centred on a grid **intersection**, not a cell
  centre. A filled cell would have looked like a square at a cell centre.
- The image is a **flat graphic export** (7 KB PNG for 582×458 — no photographic
  noise, no perspective), so measured positions are exact rather than estimates.
- Tracks are 6-7 px wide on a **62.5 px grid**; the field is a 20 cm grid, so
  **3.125 px/cm** — which makes the tracks **≈ 2.0 cm wide** and the cell 20 cm,
  matching the grid size assumed everywhere else in this project.
- The thin 1-2 px rectangle round the edge is **not a track** — it is the edge of
  the white field sheet against the grey page. Worth stating because the red start
  arrow points *along* it, which reads as "start on the border" until you check the
  line width.

Lattice is **8 columns × 7 rows**; start is the bottom of the long centre vertical
(red arrow), target is the disc at column 1 / row 2.

**`scripts/field_to_maze.py` was rewritten.** The first version tried to extract the
network from the image automatically and got it wrong (disc radius 204 px, 9 of ~20
segments found). Automatic extraction is the wrong shape of solution here: the
drawing is hand-made, so most tracks land on the lattice but individual elements are
off by 20-30 px, and *any* parser either silently snaps them or silently emits a
non-grid maze. So the lattice is transcribed once by hand into a `SEGMENTS` table and
the script's job is to expand it, write the JSON, and **draw the reconstruction back
over the original image** so a human can check it in one glance. Verified both
solvers on the result:

| | C solver | Python simulator |
|---|---|---|
| nodes / edges | 38 / 41 | 38 / 41 |
| steps | 70 | 69 |
| fast-path time | 6.50 s | 6.51 s |
| proof state | FULL | full map |
| fast path | 140 cm | — |

(The 1-step gap is the same counting difference already present on the sample mazes;
it is the last command tick, not a disagreement about the route.)

#### 3. Two things about the field still need the user

Both are flagged in code (`field_to_maze.py`) rather than guessed at:

1. **The image looks cropped at the top.** No horizontal track exists above row 0,
   yet verticals at columns 1, 3 and 4 run off the top edge — column 4's by a full
   ~30 px, half a cell. Either there is another row above the crop, or those stubs
   are deliberate. The current graph models the *visible* field only.
2. **The right-hand box does not snap to the lattice.** Everything else in the
   drawing lands on the 62.5 px grid to within a couple of px; this one box sits at
   x 415-467 where a 1-cell box would be 438-500. Its vertical extent is exactly
   right (rows 2-4), so it is a one-cell-wide, two-cell-tall loop that was drawn
   ~23 px too far left. Transcribed as columns 6-7; the `--overlay` output shows the
   mismatch plainly.

**Files:** `robot codes/SENSORS.md` (new), `robot codes/scripts/field_to_maze.py`
(rewritten), `simulator/mazes/real_field.json` (new), `robot codes/build/imgcrop/*`
(scratch, gitignored).

---

### 2026-09-23 — M0: firmware project for the solver + a command-line build that proves it fits

**Goal:** get the tested C solver (`robot codes/`) running on the real robot instead
of the left-hand rule. M0 was "make it build and fit". **It does not fit yet — and
that is now a measured fact, not an estimate.**

**New: the firmware can be built from the command line.** Keil's ARM Compiler 5 is
installed at `C:\Keil_v5\ARM\ARMCC\bin`, so the real toolchain can be driven
without the Keil IDE:
- `bash scripts/build_firmware.sh` — compiles + links the whole firmware for
  STM32G031G8Ux, prints the Keil `Program Size:` line, writes `build/firmware/seyed.hex`
- `USE_SOLVER=1 bash scripts/build_firmware.sh` — same with `USE_MAZE_SOLVER` defined
- `bash scripts/measure_solver_ram.sh` — answers "does it still fit in 8 KB?"

**Measured facts (ARMCC 5, `-O3`, C99, Cortex-M0+):**

| Build | Flash | RAM |
|---|---|---|
| Firmware as-is (solver present but dormant) | 34836 / 65536 (53%) | **7632 / 8192 (93%)** |
| Firmware with solver **activated** | — | **does not link: `L6407E`, 0xc88 = 3208 bytes too big** |

- The solver needs **2624 bytes** of static RAM (measured via `fromelf -z` on a
  probe object). The existing firmware is already at 93% RAM, so activating the
  solver overflows by 3208 bytes.
- **Compiling the solver in costs nothing until it is called.** With
  `USE_MAZE_SOLVER` defined but `main.c` not yet including `maze_hal.h`, the build
  is byte-identical to the baseline (34836 / 7632) — `armlink --remove` drops the
  whole unused library. So wiring it into the Keil project is provably free, and
  only the activation step is RAM-critical.
- Flash is *not* a problem: 53% used, ~30 KB free.

**Fixed the solver's memory limits** (`robot codes/inc/maze_config.h`): 100→**80**
nodes, 200→**160** edges, path buffers 100→**80**, frontier list 40→**24**,
command log 1024→**256**. Cuts solver RAM from ~3990 to **2624 bytes** and still
covers the largest simulator maze (56 nodes) with 43% headroom. Also **replaced the
config's size table, which was wrong** — it claimed 10-byte nodes/edges and omitted
`MazeRobot`'s arrays entirely, understating the real cost by ~2 KB. Real ARM32 sizes
are now documented there.

**New firmware project: `final version of seyed/firmware/`** — a copy of
`New Start/code/t2/` (the real, Keil-buildable firmware). `New Start/` stays
read-only per the project rule. Keil references the solver sources **in place** at
`../../robot codes/src/*.c` with include path `../../robot codes/inc`, so the
algorithm keeps a single source of truth shared with `run_maze.py`.

**`robot codes/inc/maze_hal.h` reconciled against the real firmware:**
- **Real bug fixed:** it declared `extern int BLT_SendData(int len);` but the
  firmware defines `void BLT_SendData(int length)` (`Inc/Hardware.h:28`). Two TUs
  disagreeing on a return type compiles and links fine — it is undefined behaviour
  that would never surface as an error. Now `void`.
- **Every `extern` verified** against `firmware/Core/Src/main.c`; the line-number
  comments all pointed at the stale `code/Core` copy and are corrected.
- **The documented integration snippet was wrong**: it placed the tick inside
  `if (right_poss || left_poss)`, which **misses dead ends** — a dead end is a
  frontier decision, so the map would come out wrong. Corrected to the real call
  site: inside `if (roatating == 0)`, replacing the whole junction-*or*-dead-end
  block (there are two, `main.c:1206-1243` for `head==0` and `main.c:1289-1326`
  for `head==1`).
- Documented *why* the tick must sit exactly there: `left_poss`/`right_poss` are
  latched and then **cleared by `turn_left()`/`turn_right()` themselves**
  (`main.c:571-572, 643-644`), so they are only valid in the instant before the
  motion primitive runs.

**ARMCC 5 compatibility verified:** all 6 solver sources compile warning-free under
the real ARMCC 5 in C99 mode (this project is `uAC6=0` — AC5, not armclang). They
also compile clean under `gcc -std=c99 -Wall -Wextra -pedantic -Werror`, and contain
no VLAs or C11-only constructs. **31 tests still pass.**

**Gotchas learned this session:**
- **Do not glob `Drivers/STM32G0xx_HAL_Driver/Src/*.c`** — that directory also holds
  `*_template.c` files that belong to no target and fail to compile. Build exactly
  the 18 files `Source.uvprojx` lists.
- **`armcc`/`armlink` word-split on the spaces in this repo's paths.** Keep sources
  and flags in bash **arrays** expanded as `"${arr[@]}"`; never build a
  space-joined string and re-split it.
- `fromelf --text -z` output is **CRLF** and its whole-image row is
  `(uncompressed)` — there is no "Grand Totals" line (that one is armlink's).
- Keil's output dir in this project is named **`Source`**, not `Objects`, so the
  existing `.gitignore` rules did not cover it; **30 MB of build artifacts** would
  have been committed. Added `**/MDK-ARM/Source/`.
- Pre-existing warning in the original firmware: `main.c:1378` `dir_string` set but
  never used. Not introduced here; left alone.

**Next (M1/M2), in order:**
1. Retire the legacy path/map arrays under `#ifndef USE_MAZE_SOLVER`
   (`link[100][4]`=1600 B, `node[50][6]`=1200 B, the four path strings=800 B →
   **3600 B**), plus shrink `BLT_RX_Buffer`/`GTD_RX_Buffer` 500→64 (**872 B**).
   Total **4472 B** freed against a 3208 B shortfall → ~1.3 KB headroom.
2. Re-run `scripts/measure_solver_ram.sh` to confirm the fit before flashing.
3. Then wire the tick call sites (M2).

---

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
  - `final version of seyed/firmware/` — **the STM32 firmware** (copy of `New Start/code/t2/`);
    Keil project at `firmware/MDK-ARM/Source.uvprojx`, solver sources referenced in
    place from `robot codes/`
- **Solver phases:** `EXPLORE → RETURN_HOME → FAST_RUN → DONE`.
- **Fastest route:** minimum-time, accel-aware, on the discovered map; proof is
  time-based and admissible.
- **C port:** 8 modules, 31 tests, zero warnings.  HAL bridge (`maze_hal.h`)
  reconciled against the real firmware.  Integration is **M0-complete but
  RAM-blocked**: the solver does not fit until the legacy path/map arrays are
  retired (see the 2026-09-23 entry).
- **Decision core (`brain.c`) — host-validated.** `brain.h`/`brain.c` expose the
  solver as a pure decision function over junction reports (`BrainIn`) → one move
  (`'F'/'L'/'R'/'B'`), then two command strings when done.  The robot owns
  everything physical; the brain never reads a sensor, motor, encoder or compass.
  **5/5 checks on all five mazes**, including `real_field`, where the brain's
  fast-path time equals the time-optimal cost of the full maze it was never
  allowed to see.  Test: `python scripts/run_brain.py ../simulator/mazes/<f>.json`.
  Not yet wired into the firmware.
- **Firmware build:** `bash scripts/build_firmware.sh` (no Keil needed);
  RAM check: `bash scripts/measure_solver_ram.sh`.
- **Mazes:** `simulator/mazes/*.json` (`sample_maze.json` auto-loads).
- **Run:**
  - `python "final version of seyed/simulator/maze solver/maze_solver.py"`
  - `python "final version of seyed/simulator/maze generator/maze_generator.py"`
- **Git remote:** `origin` → `github.com/mmd-bsd/robotic-maze-bot.git`, branch `main`.

## Known open items / ideas (not done yet)

- Explore speed is a constant (40 cm/s), not yet a GUI slider — expose if needed.
- Turns are modelled as a **full stop**; a finite turn speed-cap / turn-time and
  real encoder/IMU calibration of `v_max`/`a_max` are future work.
- **STM32 on-target integration**: the build/link step is DONE (ARMCC 5, from the
  command line). What remains is (a) retiring the legacy path/map arrays to free
  the **1808 B** the link is short by (current figure from
  `measure_solver_ram.sh`), (b) wiring `brain_step()` into `main.c`'s junction
  handler, then (c) encoder calibration + on-hardware testing.
- **Bring-up mode chosen:** the brain runs in firmware and prints each decision
  over Bluetooth, waits ~5 s, then executes. Supervised, one node at a time —
  see `New Start/` for the firmware's existing Bluetooth UART debug path.
- **Two firmware questions, both now decided — the M1 capture only checks the
  second** (see the 2026-09-23 "Reading the real junction logic" entry, and
  `inc/brain.h` for the integration notes):
  1. ~~Is the junction gate `s[0] && s[9]` an AND or an OR?~~ **DECIDED
     (2026-09-23): inherit it AS WRITTEN — `&&`, per main.c:1372/1378.**
  2. ~~`front` = `centre-on-line` may be wrong at a corner.~~ **DECIDED
     (2026-09-23): use it — `in.front = s[3]||s[4]||s[5]||s[6]`.** The residual
     corner doubt is settled by measurement, not geometry: one M1 capture at a
     known corner reads the raw mask and confirms or refutes it. If it lies, the
     fix is that one expression in the integration, not in the brain.
  The `J` telemetry lines carry the raw `front`/`rear` sensor masks as hex plus
  `L`,`R`,`cross`, so one capture turns the remaining doubt into a measurement.
- **The brain has never run on hardware.** Everything above is host-validated
  against a *model* of the firmware's junction detector (`brain_host.c` mirrors
  the front-bank block at `main.c:1366-1407`). The first on-robot run is what tests
  that model.
