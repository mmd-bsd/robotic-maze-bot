# C Port Status — SEYED Maze Solver

**Goal:** Port the Python `maze_solver.py` algorithm to C for STM32G031G8Ux (Cortex-M0+, 8KB RAM, 64KB flash).

**Location:** `final version of seyed/robot codes/`

**Status: THE BRAIN DRIVES THE ROBOT. The legacy left-hand-rule explorer has been
deleted outright from the firmware, the decision core is wired in, and the whole
thing fits in RAM with 1360 bytes to spare. 21 unit tests pass, zero warnings,
plus the brain host test at 7/7 on five mazes. On-target bring-up is the only
thing left.**

**The RAM blocker is gone.** The firmware used to carry a *second, complete* maze
implementation — the legacy explorer's own `link[100][4]` + `node[50][6]` map and
its command strings — and two navigation stacks do not share 8 KB. Deleting it and
putting the brain in its place came out **800 bytes smaller than the old firmware
was while doing nothing**:

| Build | Flash | RAM |
|---|---|---|
| Legacy firmware, solver dormant (the old baseline) | 34880 / 65536 | 7632 / 8192 (560 free) |
| **Brain-driven firmware (now)** | **40820 / 65536 (62%)** | **6832 / 8192 (1360 free)** |
| Brain + M1 telemetry (the supervised build) | 41792 / 65536 (63%) | 6904 / 8192 (1288 free) |

There is no `USE_MAZE_SOLVER` switch any more: the brain *is* the decision maker,
unconditionally. See `../CHANGELOG.md`, 2026-09-23.

---

## ✅ DONE — Steps 1–10

### Step 1 — Types & Config
- `inc/maze_types.h` — All structs, enums, error codes
- `inc/maze_config.h` — Fixed-point motion model, memory limits, debug

### Step 2 — Graph Module
- `inc/maze_graph.h`, `src/maze_graph.c` — Binary heap Dijkstra on known map, node/edge CRUD, integer sqrt
- `test/test_graph.c` — **6 tests pass**

### Step 3 — Robot Module
- `inc/maze_robot.h`, `src/maze_robot.c` — Heading tracking, F/L/R/B command generation (cross-product, ALGORITHMS.md §5), frontier detection, branch ranking (turn-minimizing)
- `test/test_robot.c` — **7 tests pass**

### Step 4 — Exploration Module
- `inc/maze_explore.h`, `src/maze_explore.c` — P1→P2→P3 priority system, target-agnostic, hooks into proof module for frontier filtering

### Step 5 — Proof Module
- `inc/maze_proof.h`, `src/maze_proof.c` — Time-based admissible lower bound, frontier filtering, proof state determination (PROVEN / FULL / DISABLED)

### Step 6 — Fast-Run Module
- `inc/maze_fastrun.h`, `src/maze_fastrun.c` — Trapezoidal velocity profile (`run_time`, `speed_at`), time-optimal path (stop-graph Dijkstra), fast-run plan builder

### Step 7 — Top-Level Solver FSM
- `inc/maze_solver.h`, `src/maze_solver.c` — EXPLORE→RETURN_HOME→FAST_RUN→DONE state machine, position update (node discovery)

### Step 8 — Integration Test
- `test/integration_test.c` — Full mission on 27-node sample maze with incremental discovery. **8 tests pass**, 53 steps, command stream: `FFFFFBLLBRFLLRFRFRFRFBRBRLLRLLRFLRRRRLLLBLRBRBFFFLRLF`

### Step 9 — STM32 HAL Bridge — ❌ RETIRED (2026-09-23)
- `inc/maze_hal.h` and `test/test_hal_compile.c` are **deleted**, along with
  `PLAN_HAL.md`. The HAL seam assumed the solver would drive the robot through
  `maze_hal_tick()` while the legacy explorer kept its own map. That is not the
  design that shipped: the brain is a *pure decision function* and the firmware
  keeps all motion and sensing, so the seam is `brain_step()` inside `main.c`,
  not a HAL. Keeping both would have meant two integration points and two RAM
  budgets for one job.
- The solver fix that came out of it is kept: `maze_solver_update_position()` runs
  the sensor-driven discovery for ALL nodes (not just new ones) via
  `_discover_branches()`, creating placeholder nodes at `PLACEHOLDER_DIST_CM`.

### Step 10 — Decision Core
- `inc/brain.h` — the contract. `BrainIn` in (4 relative exits + target flag + `dist_cm`), one move out (`'F'/'L'/'R'/'B'`), then `brain_home_path()` / `brain_fast_path()` on `BRAIN_DONE`. **The robot owns everything physical; the brain never reads a sensor, motor, encoder or compass.**
- `src/brain.c` — dead reckoning (snap `dist_cm` to whole `MAZE_CELL_CM` cells), the command-granularity reduction, and the two plan strings. Home and fast routes still come from `maze_graph_shortest_path()` / `maze_fastrun_build_plan()` — not reimplemented.
- `test/brain_host.c` + `scripts/run_brain.py` — drives the brain from a **model of the robot**: it stops only where the firmware's junction test would fire and reports only what sensors can see. Unlike `run_maze.c`/`integration_test.c` it never calls `reveal_node()`, so the sensor-driven discovery path (and `PLACEHOLDER_DIST_CM`) is actually exercised.
- **Result: 7/7 checks on 5 mazes.** On `real_field` the brain drives through 11 cells without a report, and its fast-path time (6.50 s) **equals the time-optimal cost of the full maze** — a maze it was never allowed to see. That is the whole `proven_optimal` claim.
- Four design bugs found by the harness, all silently fatal — see the 2026-09-23 CHANGELOG entry. The most important: **a command is one STOP, not one graph edge**, and `MazeRobot.visited_nodes` is not a valid stop-point test.

### Step 11 — Firmware integration — ✅ wired, 🔲 awaiting on-target bring-up
- The legacy explorer is **deleted** from `firmware/Core/Src/main.c`: `link[][]`,
  `node[][]`, `local_cross[]`, `fill_map()`, `short_path()`, `replaceWord()`
  (which held the firmware's only `malloc`), `back_home()`, `path_discoverd[]`,
  `result_0/1/2`. About 3.2 KB freed.
- The seam is `brain_report()`: it builds a `BrainIn` from the globals the
  firmware already had, calls `brain_step()`, and returns the move — which then
  goes through the *unchanged* legacy `cross` dispatch to the motors. `brain_init()`
  runs on the KEY1 press (edge-detected, so holding the button cannot wipe the map).
- `set_plan()` copies the brain's two plans into `path_back` / `path_discoverd_s`
  and appends the legacy `'D'` end sentinel, which stages 4 and 6 depend on twice.
- `replay_dispatch()` is now the single command executor for both replay stages.
  It adds **`'F'`** (the brain's straight) and **`'B'`** — the legacy stages had no
  `'B'` case at all, and the real field's fast path is `BFFLRLF`, so the robot
  would have stalled mid-mission.
- **Two ordering bugs in the legacy replay stages, both fixed** — see the CHANGELOG
  entry; they are not obvious and they are not cosmetic.
- **`brain.c` was missing from the Keil project.** `Source.uvprojx`'s `MazeSolver`
  group listed the six solver sources but not `brain.c`, so the IDE build — the
  way the firmware is actually flashed — would have failed to link with
  `Undefined symbol brain_step`, even though `build_firmware.sh` succeeded (the
  script adds `brain` by name). Fixed in `.uvprojx` and `.uvoptx`.
- With `USE_MAZE_TELEMETRY` defined, every junction is printed over Bluetooth
  followed by a **5 s pause before the move is made**, so an operator can stop a
  bad run before the robot commits. The motors are explicitly stopped and the stop
  pushed to the servos first, because the pause blocks the superloop that would
  otherwise re-issue the drive command.

---

## Full test suite results

```
test_graph:          6 tests, 0 failed
test_robot:          7 tests, 0 failed
integration_test:    8 tests, 0 failed
brain:               compile-only, -Werror, 0 warnings
brain_oracle:        5 steps, 0 failed  (--selftest)   <- brain.c EXECUTED
bt_monitor replay:   agree -> 0 mismatches, exit 0
                     disagree -> 1 mismatch, exit 1
                     health_ok -> 0 health FAIL, exit 0
                     health_fault -> 1 health FAIL, exit 1
-----------------------------
TOTAL:              21 unit tests + 1 oracle selftest + 2 decision replays
                    + 2 health replays, 0 failed, 0 warnings

brain_host:        7/7 checks on 5 mazes (real_field, sample_maze 1-4)
                   run via: python scripts/run_brain.py <maze.json>
                   NOT part of build_all.ps1 -- it needs a maze .json to
                   generate _maze_data.h, so run_brain.py drives it.
                   build_all.ps1 compiles brain.c alone (-Werror) so the
                   warning guarantee still covers it.
```

`build_all.ps1` now **checks exit codes** on the run phase and exits 1 itself if
anything failed. Before 2026-09-23 it printed the tests' output and reported DONE
regardless — a failing test was visible in the scroll but did not fail the build.

**31 → 21 is not lost coverage.** The 10 tests that went were `test_hal_compile`'s,
and they tested `maze_hal.h` — a seam that no longer exists. They are in git
history if the HAL approach is ever revived.

## Maze runner

```powershell
python scripts/run_maze.py ../simulator/mazes/sample_maze.json
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_brain.py ../simulator/mazes/real_field.json   # decision core (7/7 checks)
```

No C editing needed.  Takes any maze `.json` from the simulator.

---

## 🔲 REMAINING

### Bring-up on the robot — the only step left

Everything builds, links, fits and passes on the host. What no test in this repo
can do is check the brain's *model* against the physical robot: `brain_host.c`
drives the brain from a model of the robot, so a wrong model is invisible to it.

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh   # the SUPERVISED build
bash scripts/measure_solver_ram.sh               # the RAM budget, per object
python scripts/bt_monitor.py                     # LIVE: watch it and check it
python scripts/parse_telemetry.py capture.txt    # offline report, after the run
```

**Watch it live first.** `bt_monitor.py` opens the COM port, draws the brain's
believed map and position as the data arrives, and — the point of it — feeds the
exact `BrainIn` each junction produced to `build/brain_oracle.exe`, which is
**the real `brain.c`**, and compares its move with the one the robot made. Every
junction gets a green tick or a red X *while the robot is still on the field*, so
a bad decision is caught at the junction instead of in a report afterwards. It
also records `build/bt_captures/capture_<ts>.txt` verbatim, which is exactly
`parse_telemetry.py`'s input, so the offline pass below still works on it.

`python scripts/bt_monitor.py --replay <capture.txt> --headless` re-runs any
capture without a robot and exits 1 on a mismatch.

Flash the telemetry hex, press KEY1, and read §2 and §6 of the report. The three
things to look at, in order of how much they matter:

0. **`in.front` cannot be independent of `left`/`right` — `L=1 ⟹ F=1`, by
   algebra. OPEN only in *which* wrong input a corner produces.**

   `main.c:932` builds `in.front` as `(s[3]||s[4]||s[5]||s[6])`, and
   `main.c:1338-1339` builds the laterals on that **same** term:

   ```
   left_poss  = s[2] && (s[3]||s[4]||s[5]||s[6])  =  s[2] && front
   right_poss = s[7] && (s[3]||s[4]||s[5]||s[6])  =  s[7] && front
   ```

   So `front` = 0 forces `left_poss` = `right_poss` = 0. **The brain can never be
   handed "turn left, nothing ahead"** — that input is unreachable on this
   hardware, whatever the sensors do. That is not a tendency or a measurement; it
   is what the two assignments say.

   The stop test at `main.c:1377` closes the loop. `head_delay` is reset whenever
   `at_node` is true:

   ```c
   centre_dark = (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0);   /* all WHITE */
   if (centre_dark && !at_node) head_delay++; else head_delay = 0;
   ```

   so the persistence clause **cannot fire at a node** — a node stop must come
   from `at_node && (left_poss||right_poss)`, which forces `front` = 1.

   **Therefore, at a corner with no straight-through lane, the brain receives one
   of exactly two inputs, and both are wrong:**

   | corner stop is… | mask shows | brain is told | should be |
   |---|---|---|---|
   | `at_node` (bits 0,9 set) | `F=1, L=1, R=0` | "straight or left" — corner ≡ T-junction | `F=0, L=1, R=0` |
   | persistence (bits 0,9 clear) | `F=0, L=0, R=0` | **"dead end — reverse"** — corner ≡ dead end | `F=0, L=1, R=0` |

   There is no third option, because `F=0` drags `L` and `R` to 0 with it. Which
   of the two you actually get is the one thing left to measure.

   **THE TEST — at a corner whose lane does not continue straight, read field 7
   of the `J` line (the raw front mask, hex):**
   - bits 0 **and** 9 set → `at_node` stop → corner is reported as a T-junction.
   - bits 0/9 clear → persistence stop → corner is reported as a **dead end**.
     (This is the more alarming of the two: the brain answers `'B'`.)

   **Two things NOT to cite as evidence.** (a) `bt_monitor.py --replay
   test/fixtures/capture_agree.txt`'s "front == (left or right) at 5 of 5
   junctions" is **circular** — those masks were built *from* the firmware's stop
   condition, so it restates the source rather than testing it; a fixture cannot
   test the question it was constructed to assume. (b) **`brain_host`'s 7/7 does
   not transfer** — it derives front/left/right from `true_neighbor()`, the maze's
   real topology, so it *can* produce `(F=0, L=1)`, precisely the input this
   hardware cannot.

   **Why the obvious fix does not work.** `s[3..6]` *do* measure black under the
   front-centre row, and at a corner that black is real — it is the junction the
   robot is standing on. The problem is not that `front` is unmeasured, it is that
   `front` is **fused** to `L`/`R`: any reading you get from `s[3..6]` arrives at
   the brain *simultaneously* as `L`/`R`, so it cannot separate them. And a thin
   lane and a junction blob both read black, so one instant of four centre sensors
   cannot tell "lane continues" from "I am on a blob" either. Separating them
   needs evidence over *distance* — the persistence trick `head_delay` already
   uses, mirrored — and that is a firmware decision, not a patch.

   **A static test now exists (M1.5, 2026-09-23), and it needs no drive.** The
   `J`-line test above requires a corner stop in a real run. The health build
   instead lets you put the robot on a corner **by hand**, on the bench, and read
   the panel: the `DERIVED` block prints `front = s[3..6]`, `L = S2 && front`,
   `R = S7 && front`, `at_node = S0 && S9` from the raw ADC, so you can see
   directly whether a hand-placed corner produces `(F=1, L=1, R=0)` or
   `(F=0, L=0, R=0)`. It also settles the gate's AND-vs-OR question — a pose where
   exactly one of `S0`/`S9` is over black is trivially arrangeable when nothing is
   moving. Both are facts about the robot; neither is a fixture, so neither is
   circular.
1. **Drift.** `dist_cm` should snap to whole 20 cm cells with a small residue that
   does not grow. A climbing drift means the counts-per-cell constant is wrong and
   the time-optimal planner is reasoning over a distorted map.
2. **That the executed move is the printed one,** and that the 5 s pause holds.
   The live monitor's *confirm-in-the-wrong-place* flag is this check: it marks a
   move whose predicted arrival node is not the node the next `B`/`J` reports.
3. **That the oracle agrees on every junction.** A `DECISION MISMATCH` with a
   clean derivation means the wire lost or duplicated a junction (a BLE drop does
   exactly this — check the unparsed counter, and treat the rest of the run as
   suspect rather than chasing ghosts); a `DERIVATION MISMATCH` means the wiring
   and the telemetry are not the same expression.

Then calibrate the encoder and re-run M1's three measurements.

---

### Milestone M0 — firmware builds and fits ✅ DONE

- ✅ **Config sized to the field** (`inc/maze_config.h`) — shortfall 3208 → 1808 B.
- ✅ **`node[]` / `link[]` overflow fixed** in `firmware/Core/Src/main.c` — this was
  a **pre-existing memory-corruption bug**, live in the M1 build: `node[50]`
  indexed by the command count wrote 504 B past the end, into `uwTick` and
  `hi2c2` (the IMU's I2C handle). Moot now (both arrays are deleted), but the
  finding stands in CHANGELOG.md.
- ✅ **`build_firmware.sh` map fix** — it had been printing a `seyed.map` path
  that AC5 never creates (`--map` takes no argument; the map goes to stdout).
- ✅ **The RAM blocker is gone** — the legacy map is deleted and the brain fits
  with 1360 B to spare. **`measure_solver_ram.sh` now reports FITS.**
- ✅ **`build_firmware.sh` size reporting fixed** — the `fromelf` parse matched a
  `(uncompressed)` row that this build never emits, so `TOTAL_RAM` came out **0**
  and the fit check silently passed. It now reads the linker's own totals and
  **exits 1 if it cannot**.

---

### Milestone M1 — bench telemetry ✅ built, awaiting robot time

M1 exists because three numbers the solver needs cannot be derived from any file
in this repo — they only exist on the physical robot:

1. **counts per 20 cm cell**, which calibrates the entire map
2. **which sensors actually fire** at a junction, which validates `SENSORS.md`
3. **how long the branch detector stays live**, which is the decision's timing budget

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh   # -> build/firmware/seyed.hex
python scripts/parse_telemetry.py capture.txt    # -> the measurements
```

The telemetry build is now also **the supervised build**: `USE_TELEMETRY`
additionally arms the 5 s bring-up pause before each junction move. It costs
+72 B RAM / +~1 KB flash over the plain build, and with the macro undefined the
path costs exactly nothing.

| Build | Flash | RAM |
|---|---|---|
| Brain-driven (plain) | 40820 / 65536 (62%) | 6832 / 8192 (1360 free) |
| + health **bench** (`USE_HEALTH=1`, adds `HEALTH_ONLY`) | 41560 / 65536 (63%) | 6848 / 8192 (1344 free) |
| + telemetry, which includes health, and can still run | 42264 / 65536 (64%) | 6912 / 8192 (1280 free) |

**The `J` line gained three fields** (`target`, `dist_cm`, `node`) and a new
**`B` line** was added for the bring-up decision, so captures from a build
before 2026-09-23 will not parse. `parse_telemetry.py` reports unparsed lines
rather than silently skipping them.

**Status: built, compiles with the real ARMCC 5, links, wire format verified on
the host.** The *measurements themselves* need the robot; nothing in this repo
can produce them. See `BUILD_GUIDE.md` for the line format.

---

### Milestone M1.5 — bench health check ✅ built, awaiting robot time

M1 answers *what the robot decided* while driving. It does not answer *what the
hardware reads* while stopped — the M1 gate is `loop_start != 0`, so a parked or
pre-KEY1 robot is silent, and `brain_host.c` drives the brain from a **model**, so
a wrong model is invisible to it. M1.5 is the second question.

```bash
USE_HEALTH=1    bash scripts/build_firmware.sh   # BENCH: also defines HEALTH_ONLY
USE_TELEMETRY=1 bash scripts/build_firmware.sh   # both -- the one to flash to run
python scripts/bt_monitor.py                     # health panel is automatic
python scripts/bt_monitor.py --replay cap.txt --headless --health   # exit 1 on FAIL
```

Two lines, sent **only while stopped** (`loop_start == 0 || loop_start >= 7`):

```
H,<ms>,<keys>,<loop>,<head>,<gz>,<za>,<a0>,...,<a17>      20 Hz
T,<ms>,<what>,<v0>,...,<v17>       what=mid|min|max       ~0.35 s, cycling
```

`H` is 25 fields, `T` is 21. `TLM_DRIVING()` is the single predicate that keeps
the health sender and the mission sender from ever calling `BLT_SendData()` in one
pass; a queued `J`/`Z` event always wins its slot.

**`USE_HEALTH=1` is the bench build, and it cannot run a mission.** It also defines
`HEALTH_ONLY`, which makes the pre-KEY1 boot loop never exit. KEY2 drives the IR
calibration, KEY3 the gyro one, and KEY1 only re-sends the banner. A bench session
therefore leaves `loop_start` at **0 forever** — and that is the point: it is the
proof `HEALTH_ONLY` took. Anything else means the mission started.
`USE_TELEMETRY=1` deliberately does **not** define it; that build has to be able to
drive.

**The robot announces itself.** On reset the unconditional `Hi ,mmdi` banner at
`main.c:1313` goes out ~300 ms after power-up — usually before anyone has
connected, which is why the IRQ now answers too: under `USE_MAZE_HEALTH` *any*
received byte sets `health_hello`, `health_tick()` replies with the same banner, and
`SerialSource.send()` gives the app its first-ever TX path (probe on connect + a
**Ping robot** button). This is the only line on the wire that can be a *reply*, so
it is the only proof the link works in both directions — a silent bar proves
nothing, because the robot streams whether or not anyone is listening.

**The app now keeps a TERMINAL** (bottom strip): banner, unparsed lines, key edges,
threshold changes, plan dumps, link open/close/errors. The 20 Hz stream is filtered
out of it deliberately — it would bury the banner within a second. A **STATE** card
shows `loop_start` and its name (`parse_telemetry.LOOP_STATES`, shared with the
offline report).

**The view draws the BOARD, not a sensor bar.** The pads are a **ring** round the
perimeter: the front row `S2…S7` with the side pads `S1`/`S8` set back behind them,
`s[0]`/`s[9]` **side by side on the rotation axis** at the robot's centre, then the
mirror image at the back with the rear bank's indices running the other way
(`S17…S10` left to right). One definition: `parse_telemetry.PAD_CELL` (a grid cell
per pad) — the canvas scales it, the text report prints it. The grid is **portrait**
(29 rows × 17 columns ≈ the board's 570 × 335 mm), because a board drawn wider than
it is tall is a robot that does not exist and it hides where the axis pads sit; the
canvas draws the board outline, the dashed centre line and the pads, and the pad's
name and ADC are the only words on it — the derived summary, the role legend and
the caveats live on the cards beside it, not under the picture. That replaces a
two-row bar that had `S0`/`S9` at the outer ends of a ten-wide front row, which a
first pass at a top view suggested and the board itself contradicts; SENSORS.md §2
had it right all along.

**Deliberately not sent: the `s[]` mask.** On the bench `s[]` is still all-zero
(the hysteresis block lives in the superloop; the one-shot init runs only after
KEY1), so it would be a fresh third derivation, not the firmware's own value. The
app derives the bit from `adc` vs `mid` and labels it as derived.

**It also fixed a dead gesture.** `KEY3`'s gyro calibration in the boot loop only
set `GyroCalF`, and the sole consumer of that flag is `Calculate_Z_Angle()`, which
ran only from the superloop — so it did nothing until KEY1, which then started a
fresh calibration anyway. `health_tick()` calls it in the boot loop.

**Status: built, links, the whole verdict path asserted both ways.** The two
fixtures are **synthetic** (`scripts/make_health_fixtures.py`) and test the tools,
not the robot — a fixture cannot answer "are the sensors healthy". That question
is what the robot is for.

---

## File listing

```
robot codes/
├── scripts/
│   ├── run_maze.py            ✅  (Feed any .json → build → run → result)
│   ├── build_all.ps1           ✅  (Rebuild + run all unit tests)
│   ├── build_firmware.sh       ✅  (Build + link the STM32 firmware, ARMCC 5)
│   ├── measure_solver_ram.sh   ✅  (Measure the 8 KB RAM budget)
│   ├── field_to_maze.py        ✅  (Real field image → maze .json + overlay)
│   ├── parse_telemetry.py      ✅  (M1 capture → counts/cell, sensors, windows)
│   ├── bt_monitor.py           ✅  (LIVE Bluetooth capture + virtual-brain check)
│   ├── make_health_fixtures.py ✅  (Regenerate the two synthetic health captures)
│   └── run_brain.py            ✅  (Brain host test — robot-model driver)
├── inc/
│   ├── maze_types.h            ✅
│   ├── maze_config.h           ✅
│   ├── maze_graph.h            ✅
│   ├── maze_robot.h            ✅
│   ├── maze_explore.h          ✅
│   ├── maze_proof.h            ✅
│   ├── maze_fastrun.h          ✅
│   ├── maze_solver.h           ✅
│   └── brain.h                 ✅  (Decision-core contract — the seam)
├── src/
│   ├── maze_graph.c            ✅
│   ├── maze_robot.c            ✅
│   ├── maze_explore.c          ✅
│   ├── maze_proof.c            ✅
│   ├── maze_fastrun.c          ✅
│   ├── maze_solver.c           ✅
│   └── brain.c                 ✅  (Junction report → move; plans on DONE)
├── test/
│   ├── run_maze.c              ✅  (Generic runner — reads _maze_data.h)
│   ├── test_graph.c            ✅
│   ├── test_robot.c            ✅
│   ├── integration_test.c      ✅
│   ├── brain_oracle.c          ✅  (brain.c as a stdin/stdout filter — the
│   │                                virtual brain. --selftest; no maze needed)
│   ├── fixtures/
│   │   ├── capture_agree.txt    ✅  (replay fixture — 0 mismatches)
│   │   ├── capture_disagree.txt ✅  (one field altered — exactly 1 mismatch)
│   │   ├── capture_health_ok.txt    ✅  (SYNTHETIC — 0 health FAIL)
│   │   └── capture_health_fault.txt ✅  (SYNTHETIC — 1 health FAIL, exit 1)
│   └── brain_host.c            ✅  (7/7 on 5 mazes — robot model, not a replayer)
├── SENSORS.md                  📋  Sensor map: s[i] ↔ silkscreen ↔ MUX ↔ role
├── BUILD_GUIDE.md              📋
├── STATUS.md                   (this file)
```

## Build commands

All commands run from `robot codes/`.  Output goes to `build/` (gitignored).

See **[BUILD_GUIDE.md](./BUILD_GUIDE.md)** for the full guide.

### Quick: test any maze .json
```powershell
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_brain.py ../simulator/mazes/real_field.json   # decision core (7/7 checks)
```

### Quick: rebuild all after code changes
```powershell
.\scripts\build_all.ps1
```
Now also builds `brain_oracle`, runs its `--selftest`, and runs the two
`bt_monitor.py` replay checks. **Exits 1 if anything failed.**

### Live capture (the bring-up instrument)
```powershell
python scripts/bt_monitor.py                                  # GUI: pick COM port, connect
python scripts/bt_monitor.py --replay test/fixtures/capture_agree.txt --headless
python scripts/bt_monitor.py --demo                           # GUI smoke test, no hardware
```
Needs `pip install pyserial` for the serial path only — `--replay` and `--demo`
work without it. Records raw `.txt` + `.csv` + `.json` into `build/bt_captures/`.

Add `--health` for the **bench health view/report** instead of the mission check:
keys, all 18 raw `IR_ADC[]`, `IR_mid/min/max` and the gyro, drawn in the physical
bar layout. Exits 1 on any FAIL, **2** if the capture contains no `H` lines (so
"nothing to check" cannot read as a pass).

### Manual (rarely needed)
```powershell
# Add GCC to PATH (once per session)
$env:PATH = "C:\msys64\mingw64\bin;$env:PATH"

gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o build/test_graph.exe
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o build/integration_test.exe
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc -c src/brain.c -o build/brain.o   # compile-only; its test needs a maze header

# brain_oracle -- the virtual brain. Links and RUNS brain.c, needs no maze data.
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c \
    src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c src/brain.c \
    test/brain_oracle.c -lm -o build/brain_oracle.exe
build/brain_oracle.exe --selftest     # 5 steps; exit 1 on any mismatch
build/brain_oracle.exe --probe        # print the same steps, unchecked
```

## Design decisions (don't change without asking)

- All arrays **statically sized** — no `malloc` on target
- Motion model params are **fixed-point ×100** input, but time math uses `float` (proof/fast-run called infrequently; can optimize to integer later)
- `MazeHeading` matches firmware `nav` (0=N, 1=W, 2=S, 3=E)
- `MazeCommand` = 'F'/'L'/'R'/'B' chars; `maze_cmd_to_cross()` converts to firmware `cross` (0/1/2/4)
- Dijkstra on **known map only** (explored edges), by **real distance** (cm), not hop count
- Y is up (matches Python simulator and firmware convention)
- Time-optimal path uses **stop-graph Dijkstra** — from each node, walk straight in each direction over explored edges; every reached node is a candidate with time = run_time(total_distance)
- **Sensor branch discovery** in `maze_solver_update_position()` now runs for ALL nodes (not just new ones), creating placeholder neighbor nodes at 20 cm offset for each detected open path.  Back is skipped (the return edge is created when the robot physically drives between nodes).  Heading=NONE defaults to NORTH for the first sensor reading.
- **The seam is `brain_step()`, not a HAL.** The brain is a pure decision function — `BrainIn` in, one `'F'/'L'/'R'/'B'` out — and the firmware keeps all sensing and all motion. It never reads a sensor, motor, encoder or compass, and it holds no firmware pointer. `maze_hal.h` proposed the opposite (the solver driving the robot through `maze_hal_tick()` while the legacy explorer kept its own map) and is deleted; two integration points for one job meant two RAM budgets. See `inc/brain.h`.
- **The brain owns the map, the firmware owns the robot.** `maze_hal.h`'s HAL used to declare the firmware's `link[][]` / `node[][]` as `extern` so the solver could read them. Those arrays are gone; nothing in the library reaches into the firmware now.
- **The health stream carries raw `IR_ADC[]`, never the firmware's `s[]` mask.** On the bench `s[]` is all-zero (the hysteresis block lives in the superloop; the one-shot init runs only after KEY1), so streaming it would publish a *third* derivation rather than the firmware's own value — and a reader would take it for the firmware's opinion. The app derives the bit from `adc` vs `mid` and labels it derived; the `S`/`J` lines stay the authority on what the firmware believed.
- **No float `printf` in the firmware, ever** — one `%.1f` pulls 1-2 KB of the 64 KB flash for the float formatter. Scaled integers on the wire (×10 for deg/s and deg), divided once at parse time. This is why the health stream's gyro fields are integers.
- **One `BLT_SendData()` per superloop pass.** The call restarts the TX DMA, so a second one before the first drains truncates the first — silently. This is why the health lines are short, why `T` is three ~106-byte lines rather than one 290-byte line, and why the health sender is gated on `!TLM_EVENT_PENDING()` so a queued `J`/`Z` always wins its slot. It is also why `health_mute_until` stands the stream off for 40 ms after `calibr_ir()` prints its own `IR_mid` dump.
- **`HEALTH_ONLY`'s boot loop uses a `static volatile _Bool bench_leave`, not a bare `for(;;)`.** With a provably-infinite loop ARMCC emits `#128-D: loop is not reachable`, everything after the loop is eliminated, and flash silently drops from ~41.5 KB to 39492 B because the whole mission got thrown away — the warning was the only sign. A `#pragma` would hide the fact that the code after the loop is still wanted; a volatile the compiler cannot fold keeps it alive *and* keeps the zero-warning rule. Do not "simplify" this to `for(;;)`.
- **The app's TERMINAL is an event log, never a raw dump of the wire.** At 20 lines/s the health stream would push the `Hi ,mmdi` banner — the one line that can be a *reply*, and so the only proof the link works both ways — off the top within a second. `H`/`S` are therefore filtered out of it and drawn in the bar/panel instead. `_log_line()` matches on `Pipeline.feed`'s tag, not `parse_line`'s verdict, because a non-record line is re-tagged on the way through (the banner arrives as `CHATTER`).
- **`parse_telemetry.PAD_CELL` is the ONE definition of the sensor geometry — a grid cell per pad — and both the canvas and the text report are drawn from it.** Do not add a second layout, and do not go back to "rows": the pads are a **ring** (`S0`/`S9` on the rotation axis at the robot's centre, `S1`/`S8` set back behind the front row, the rear bank running `S17…S10`), and any row-shaped model puts `S0`, `S1`, `S8`, `S9`, `S10` or `S17` in the wrong place. The grid is **portrait** (`BOARD_ROWS`/`BOARD_COLS` = 29/17 = 1.71, the board's own 570/335) — the drawn rectangle has to have the robot's shape, so a wide grid is a bug, not a style. `TARGET_ROW`/`REAR_TARGET_ROW` stay in index order and are unaffected — the target test is order-blind. Read `PAD_CELL`'s comment before changing any of it.
- **The health canvas carries no words of its own beyond each pad's name and ADC.** The derived summary, the role words, `loop_start` and the "a green strip is not a verdict" caveat all live on the right-hand cards (`DERIVED`, `HEALTH`, `STATE`), and they were removed from under the board once they were shown to be duplicating them — a picture with a page of footnotes under it is a picture nobody reads. Do not add a caption back to the canvas: put it on a card.
- **The right panel scrolls, and the THRESHOLDS table scrolls inside it.** The column of cards requests ~780 px against the ~580 the 800 px window leaves it (toolbar 39 + TERMINAL 148), and the card that falls off the bottom is CHECKS — the verdict. So the cards are on a scrolling canvas, and the 19-row thresholds table has its own scrollbar with a fixed 10-line height rather than growing. `_panel_wheel` routes the wheel by what is under the pointer (a `Text` scrolls itself; anything else inside the right panel scrolls the panel); it is explicit because Tk's own Text binding would scroll the table *and* the panel under it on one notch. Any card fed from the 30 ms tick must use `_set_scroll_text`, which rewrites only on a real change and carries the scroll position over — re-inserting identical text every frame fights the operator's scrollbar.

## Key reference files

| File | Why |
|---|---|
| `../ARCHITECTURE.md` | Project rules, conventions, glossary |
| `../ALGORITHMS.md` | Every algorithm in detail (§1-§7) |
| `../../New Start/code/Core/Src/main.c` | Working robot firmware (read-only reference) |
| `../../New Start/Simulator/py-code/maze solving/maze_gbf/stm-sample-code/main.c` | An earlier `USE_MAZE_GBF` integration via a `maze_hal.h` (read-only history — superseded by `inc/brain.h`) |
| `inc/brain.h` | **The integration contract.** Read this before touching `firmware/Core/Src/main.c` |
| `../../firmware/Core/Src/main.c` | The port target: `brain_report()` builds the `BrainIn`, `replay_dispatch()` executes the plans |
