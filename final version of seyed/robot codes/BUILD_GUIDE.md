# BUILD GUIDE -- SEYED C Maze Solver

**Location:** `final version of seyed/robot codes/`

---

## How to run a maze

Design a maze in the simulator, save it as `.json`, then:

```powershell
# Test any maze in one command:
python scripts/run_maze.py ../simulator/mazes/sample_maze.json
python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
python scripts/run_maze.py ../simulator/mazes/your_maze.json

# Same, but drive the decision core (brain.c) instead of the raw solver.
# Models the real robot: stops only where the firmware's junction detector
# would, reports only what the sensors can see, then replays both plans on
# the true maze.  This is the one that validates the on-robot behaviour.
python scripts/run_brain.py ../simulator/mazes/real_field.json
```

What it does automatically:
1. Reads the `.json`
2. Generates `test/_maze_data.h` (temp)
3. Compiles `run_maze.c` + all solver sources
4. Runs the `.exe` and prints results
5. Cleans up `_maze_data.h`

Example output:

```
=== Maze Run: sample_maze4.json ===
  Nodes: 37, Edges: 37
  Start: (-20,-20)  Target: (40,-120)

-- Results --
  Steps:             97
  Commands:          97
  Target found:      yes
  Phase:             DONE
  Proof state:       FULL (mapped everything)
  Fast path nodes:   23
  Fast path time:    12.30 s
  Shortest distance: 240 cm

  Command stream (full mission): FFFRRLLRRLLRLFFFFFL...FFFFFFFRFFFFFRFFFFF
  Fast-run command stream:          FLRFFFFFFFRFFFFFRFFFFF
```

**No C code editing per maze.** Just point at a `.json` file.

---

## How to rebuild everything after code changes

```powershell
.\scripts\build_all.ps1
```

This builds 5 targets and runs them all: **21 unit tests + 1 oracle self-test +
2 decision replays + 2 health replays**, and **it exits 1 if anything failed.**

- `test_graph.exe` (6 tests) -- nodes, edges, Dijkstra
- `test_robot.exe` (7 tests) -- heading, commands, frontiers
- `integration_test.exe` (8 tests) -- full mission on sample_maze
- `brain.o` -- **compile-only** with `-Werror`, keeping `brain.c` under the same
  zero-warning rule as the rest of the library (`brain_host.c` needs a generated
  maze header, so it cannot be linked in this script — see below).
- `brain_oracle.exe` -- **the virtual brain, linked and RUN.** `brain.c` as a
  stdin/stdout filter: one `BrainIn` per line in, one move out. It needs no maze
  header (the brain discovers its map from `dist_cm` alone), so this is the one
  place in the suite where `brain.c` is actually *executed*. `--selftest` feeds a
  known five-junction sequence and checks the moves; `--probe` prints the same
  sequence unchecked.
- two `bt_monitor.py --replay` checks; see **Live capture** below.

`brain_host.c` is driven by `python scripts/run_brain.py <maze.json>` instead —
that script generates the header first. It reports **7/7 checks on 5 mazes**.

> **`brain_host`'s 7/7 does not transfer to the robot.** It derives front/left/right
> from `true_neighbor()` — the maze's real topology — so it can produce inputs the
> hardware cannot, and never produces the hardware's corner state. See the
> `in.front` finding in `STATUS.md` §"Bring-up on the robot".

---

## Live capture — `bt_monitor.py`

The bring-up instrument. It opens the Bluetooth COM port, draws the brain's
believed map and position as the data arrives, and checks every decision the robot
makes against **the real `brain.c`** (via `brain_oracle.exe`) rather than a Python
re-implementation — so a disagreement is a fact about the robot or the wire, not
about a second implementation.

```powershell
python scripts/bt_monitor.py                    # GUI: pick the COM port, connect
python scripts/bt_monitor.py --demo             # GUI smoke test, no hardware
python scripts/bt_monitor.py --replay test/fixtures/capture_agree.txt --headless
python scripts/bt_monitor.py --record           # also write build/bt_captures/
```

| Mode | What it does |
|---|---|
| (default) | GUI + live serial |
| `--port COM7 --baud 115200` | skip the port picker |
| `--replay <capture.txt>` | run the identical pipeline from a file, no serial |
| `--headless` | no tkinter; prints a verdict; **exit 1 on any mismatch** |
| `--health` | the bench health view/report instead of the mission check |
| `--demo` | synthetic data into the GUI, no hardware |
| `--no-oracle` | plot only, skip the decision check |

The **TERMINAL** strip along the bottom of the window is an *event log*, not a raw
dump: the banner, every unparsed line, key edges, threshold changes, plan dumps and
link open/close/errors. The 20 Hz `H`/`S` stream deliberately does not appear there
— at that rate it would push the banner off the top within a second, and the banner
is the line the pane exists for. The stream is drawn in the bar/panel above instead.
**Ping robot** re-sends the probe; the app also sends it automatically on connect.

With a `USE_TELEMETRY=1` build the view follows the data: `H`/`T` lines put the
app on the **health panel**, `S`/`J` lines put it back on the mission map, and a
toolbar button overrides either way. The live health panel is documented in
[Health check — `USE_HEALTH=1`](#health-check--use_health1) below, together with
the bench procedure and the `H`/`T` field tables.

**Dependency:** `pip install pyserial`, needed for the serial path **only** — the
import is guarded with a clear message, and `--replay` / `--demo` work without it.
This is the repo's only third-party dependency.

**Recording** goes to `build/bt_captures/` (gitignored), per-line flushed so a
runaway robot does not lose the evidence:

- `capture_<ts>.txt` — the wire **verbatim**, which is exactly
  `parse_telemetry.py`'s input format, so the existing offline report still reads
  any capture this tool makes
- `capture_<ts>.csv` — one row per line, fixed superset schema, blanks where N/A
- `capture_<ts>.json` — the oracle's per-junction verdicts and the two final plans

**Regenerating the fixtures.** `test/fixtures/capture_agree.txt` and
`capture_disagree.txt` are small hand-derived captures, committed so the replay
checks in `build_all.ps1` have something to assert. They were generated from the
oracle on 2026-09-23, so agreement is guaranteed *by construction* rather than
assumed. To make more: build the oracle, feed it a `BrainIn` per line, and use
`--probe` output as the robot's move column:

```powershell
build/brain_oracle.exe --probe     # the moves, unchecked
```

`capture_disagree.txt` is `capture_agree.txt` with **exactly one field changed**
(junction 3's `ch` → `B`), and the suite asserts exactly one mismatch at that
junction — a checker that only ever passes proves nothing.

The two **health** fixtures, `capture_health_ok.txt` and
`capture_health_fault.txt`, are generated the same way but are **synthetic
throughout** — `python scripts/make_health_fixtures.py` writes both from one
schedule with only the planted faults varying, so a diff between them is a diff
between the faults. They test the tools; they cannot test a sensor.

---

## Build the STM32 firmware (no Keil needed)

The firmware project lives at `../firmware/` and builds with Keil's own ARM
Compiler 5, which is installed at `C:\Keil_v5\ARM\ARMCC\bin`. You do **not** need
to open the Keil IDE to check that it compiles, links, and fits:

```bash
# The brain-driven firmware (what flashes today):
bash scripts/build_firmware.sh

# Supervised build: the same firmware plus telemetry AND a 5 s pause before
# every junction move.  This is the bring-up build -- see below.
USE_TELEMETRY=1 bash scripts/build_firmware.sh
```

There is no `USE_SOLVER` switch any more and no legacy fallback: the decision
core *is* the decision maker, unconditionally. The legacy left-hand-rule
explorer was deleted outright on 2026-09-23 (it still exists in git history).

Output goes to `build/firmware/` (gitignored): `.o` objects, `seyed.axf`,
**`seyed.hex`** (flashable), the link map, and the scatter file used.

It prints Keil's own size line and a budget check:

```
 Program Size: Code=39748 RO-data=... RW-data=... ZI-data=5872
 FLASH :  40820 /  65536 bytes  (62% used, 24716 free)
 RAM   :   6832 /   8192 bytes  (83% used, 1360 free)
```

### Does it still fit in RAM?

**RAM is the binding constraint on this project**, and it was the reason the
legacy explorer had to go: the firmware used to carry a *second, complete* maze
map, and two navigation stacks do not share 8 KB. Measure, don't assume:

```bash
bash scripts/measure_solver_ram.sh
```

It builds the firmware once and reports the totals from the linker, then the
**per-object RAM breakdown, largest first**:

```
== totals ==
 FLASH :  40820 /  65536 bytes  (62% used)
 RAM   :   6832 /   8192 bytes  (83% used, 1360 free)
 ...
 FITS -- 1360 bytes of RAM headroom
```

It **exits 1** if the build overflows, and also if it cannot read the linker's
totals at all — a silent zero there would make the fit check pass vacuously.

Note that part of the RAM total is the startup file's `Stack_Size` (1024) +
`Heap_Size` (512) reservation, and both are counted in the Keil size line. That
matters because the solver's heaviest stack frame (`maze_time_optimal_path()`,
five `MAZE_MAX_NODES`-sized arrays) is 704 B at 64 nodes against a 1024 B stack.

| Build | Flash | RAM |
|---|---|---|
| Legacy firmware, solver dormant (the old baseline) | 34880 / 65536 (53%) | 7632 / 8192 (93%) |
| **Brain-driven (now)** | **40820 / 65536 (62%)** | **6832 / 8192 (1360 free)** |
| Brain-driven + telemetry | 41792 / 65536 (63%) | 6904 / 8192 (1288 free) |

The brain-driven build is **800 bytes smaller than the old firmware was while
doing nothing.**

**Toolchain note:** this project uses **ARM Compiler 5 (`uAC6=0`), not armclang**.
The solver contains no C11-only constructs (verified under
`gcc -std=c99 -Wall -Wextra -pedantic -Werror`), but check AC5 specifically before
assuming a new construct is fine.

---

---

## M1 bench telemetry — and the supervised bring-up build

Three numbers the brain needs cannot be derived from any file in this repo; they
exist only on the physical robot:

1. **counts per 20 cm cell**, which calibrates the entire map
2. **which sensors actually fire** at a junction, which validates `SENSORS.md`
3. **how long the branch detector stays live**, which is the decision's timing budget

`USE_TELEMETRY=1` produces a build that measures all three — and it is also the
**supervised build**: it arms a 5 s pause between printing a decision and making
it. Press KEY1, and the robot stops at each junction for 5 s with the decision on
the Bluetooth link, so a bad decision can be caught before the robot commits. The
motors are explicitly stopped and the stop pushed to the servos before the pause,
because the pause blocks the superloop that would otherwise re-issue the drive
command.

It changes **what** the robot decides in no way at all — telemetry and the pause
are both guarded by the macro.

```bash
USE_TELEMETRY=1 bash scripts/build_firmware.sh
# flash build/firmware/seyed.hex, capture the Bluetooth output to capture.txt:
python scripts/parse_telemetry.py capture.txt
```

| What it adds | Cost |
|---|---|
| `tlm_ms` 1 ms stamp, 64 B pending line, 8 ms divider | **+72 B RAM**, +~1 KB flash |
| RAM with telemetry on | 6904 / 8192 (1288 free) |
| RAM with it off | 6832 / 8192 (1360 free) |

It writes four line types to the Bluetooth link the firmware already uses, as
ASCII, at most one line per 8 ms slot:

```
S,<ms>,<front>,<rear>,<e0>,<e1>,<L>,<R>,<cross>   125 Hz, while driving
J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>,<target>,<dist>,<node>
Z,<ms>,<state>                                    target zone enter/leave
B,<ms>,<node>,<x>,<y>,<drift>,<move>              bring-up decision (SS.6)
```

`<front>` is `s[0..9]` as a hex bitmask and `<rear>` is `s[10..17]` — two
separate fields so the banks can never be confused. `<raw>` is the
**unconverted** summed encoder count for the link, which is the whole point:
the firmware divides it by `2.467*2` and then adds an empirical offset, and the
telemetry lets you check the divisor and the offsets separately instead of
inferring them from a final number.

> **The `J` line gained three fields and the `B` line is new (2026-09-23).**
> Captures made before that date will not parse — `parse_telemetry.py` now
> reports unparsed lines rather than silently skipping them, so an old capture
> shows up as noise instead of as a wrong measurement.

`<target>` is the target-zone flag, `<dist>` the distance the brain was told
(cells × 20 cm), and `<node>` the brain's own node id — the three things that
turn a capture into a check of the brain's *view* rather than just of the sensors.
The `B` line is the bring-up decision itself: the brain's dead-reckoned position,
its accumulated `drift`, and the move it chose — printed **before** the 5 s pause.

Two constraints are worth knowing before trusting the output:

- **One transmit per slot.** `BLT_SendData()` restarts the TX DMA
  (`Hardware.c:135`), so a second call before the first has drained truncates
  it. The firmware therefore queues a junction line and sends *either* that
  *or* a stream sample, never both. The `B` line is sent immediately before the
  5 s pause, where nothing else is competing for the link.
- **8 ms sampling against a ~20 ms branch window** gives only 2-3 samples across
  a junction. That is enough to say *which* sensors fire; it is not enough to
  measure the window to the millisecond, and `parse_telemetry.py` prints that
  caveat rather than hiding it. If a precise window is ever needed the fix is an
  on-chip ring buffer dumped after the run, not a faster radio link.

`parse_telemetry.py` reports: the measured counts-per-cell (which is what
actually calibrates the map), which sensors fire at each junction (which
validates `SENSORS.md`), the branch-window samples, the target-zone timing, the
brain's own view (`dist_cm` in cells, and a total-drift check), and the bring-up
decisions. It prints **`?? CORNER CANDIDATE`** when a junction has exactly one
lateral exit and a forward one — the case where `in.front` may be lying about
there being a forward path.

---

## Health check — `USE_HEALTH=1`

The **bench** instrument. M1 telemetry only streams while a mission is running
(`loop_start != 0`), so a robot standing still — before KEY1, or parked after a
run — is silent. That is exactly the state in which you want to look at the
hardware, and it is the state the health build answers for:

```bash
USE_HEALTH=1 bash scripts/build_firmware.sh       # BENCH ONLY -- no mission
USE_TELEMETRY=1 bash scripts/build_firmware.sh    # both -- the one to flash to run
```

**The two differ by one macro.** `USE_HEALTH=1` also defines `HEALTH_ONLY`: the
pre-KEY1 boot loop never exits, so the robot **cannot** start a mission whatever
is pressed. That is the bench build — sit the robot down in diagnostics and leave
it there. `USE_TELEMETRY=1` adds the health stream *without* `HEALTH_ONLY`, because
that is the bring-up build and it has to be able to drive.

`USE_TELEMETRY=1` implies the health stream as well, so a single flash covers both
jobs. The two never contend for the link: the health send is gated on the robot
being **stopped** (`loop_start == 0 || loop_start >= 7`) and on
`TLM_EVENT_PENDING()` being false, so a queued junction line always wins its slot.
`TLM_DRIVING()` is the one predicate that separates them.

In Keil the defines go in **Options for Target → C/C++ → Define**, which already
reads `USE_HAL_DRIVER,STM32G031xx`. Add `USE_MAZE_HEALTH,HEALTH_ONLY` for the bench
build, or `USE_MAZE_TELEMETRY,USE_MAZE_HEALTH` to be able to run. (The **Asm** tab's
`Define` box is a different, empty box — that one is not it.)

```bash
python scripts/bt_monitor.py                 # the health panel is automatic
python scripts/parse_telemetry.py capture.txt   # or the offline report
```

### What it adds

Two line types, sent only while stopped:

```
H,<ms>,<keys>,<loop>,<head>,<gz>,<za>,<b0>,...,<b17>        20 Hz
T,<ms>,mid,<v0>,...,<v17>                        ONCE, at calibration
```

| Field | Meaning |
|---|---|
| `<keys>` | hex bitmask, live: **1=KEY1** (PB5), **2=KEY2** (PC15), **4=KEY3** (PC14) |
| `<loop>` | `loop_start`, so the app knows which state the robot is in. Named by `parse_telemetry.LOOP_STATES` (0 = BOOT/BENCH, 1 = EXPLORE, … 8 = DONE) and shown on the health panel's STATE card. **In the bench build this is 0 and stays 0** — anything else means the mission started, i.e. `HEALTH_ONLY` did not take |
| `<head>` | which bank leads; 0 = the front row, which is the one on the bench |
| `<gz>` | `Gyro_Z` in **deg/s ×10**, as an integer — **scaled at the send site** by `ANGULAR_RATE_SENSITIVITY_2000DPS`, because `Gyro_Z` itself is raw LSB (0.070 dps/LSB) |
| `<za>` | `Z_Angle` in **deg ×10**, as an integer — no scale needed, it is already degrees |
| `b0..b17` | **1 = pad over BLACK, 0 = white** — computed in the firmware at send time |
| `<v0..v17>` | `IR_mid[]`, sent **once** (see below) |

`H` is **25 fields** (tag + 6 scalars + 18 channels) and `T` is **21** (tag + ms +
`what` + 18). Both counts are asserted by `parse_telemetry.py`; a mismatch is
reported as a BAD line rather than guessed at.

`<gz>`/`<za>` are integers because the firmware links **no float printf at all**
— one `%.1f` would pull in the float formatter for 1-2 KB of the 64 KB flash. ×10
buys 0.1° resolution for free, and the parser divides by 10 once, at parse time,
so no consumer has to remember to.

**The bits, not the raw `IR_ADC[]`.** 18 numbers of 3-4 digits per line was ~40
bytes of the 20 Hz stream spent on values nothing reads. What the bench wants is
which pads are over black, and the firmware has the answer already: the H line
carries `b[i] = (IR_ADC[i] <= IR_mid[i] - 500)`, its own entry-into-black test.
The trade is stated plainly because it is real: **the line carries the decision,
not the evidence for it**, so the app can no longer re-derive the bit its own
way and compare. What survives is every fault that shows as a pad *not
behaving* (stuck black, stuck white, never tested); what is gone is anything
needing the magnitude of a reading — including the old "every channel reads 0,
so the emitters are unpowered" FAIL, which is now an INFO naming the one-second
test that separates a dead emitter bar from a robot parked on white field.

**Deliberately not sent: the `s[]` mask.** On the bench `s[]` is still all-zero —
the hysteresis block that fills it lives in the superloop and the one-shot init
runs only after KEY1 — so a mask here would publish a *third* derivation rather
than the firmware's own value. The bit above is computed at send time instead;
the `S`/`J` lines stay the authority on what the *firmware* believed.

**Why `T` is sent once.** A threshold only changes when a calibration writes it,
so `T` goes out the moment `calibr_ir()` finishes and never otherwise. The old
scheme cycled mid/min/max through the stream forever — a third of the bench
bandwidth restating a number that had not moved — and with the raw ADC gone
there is nothing left for min/max to be compared against. One line also keeps
well inside the `BLT_SendData()` DMA restart window (`Hardware.c:135`); `min`/`max`
are still *accepted* by the parser for old captures, but no build sends them.

| Build | Flash | RAM |
|---|---|---|
| plain | 40820 / 65536 | 6832 / 8192 (1360 free) |
| `USE_HEALTH=1` (bench: `+HEALTH_ONLY`) | 41560 / 65536 | 6848 / 8192 (1344 free) |
| `USE_TELEMETRY=1` (health included, can run) | 42264 / 65536 | 6912 / 8192 (1280 free) |

### The bench procedure

1. Flash the **bench** build (`USE_HEALTH=1`, i.e. `USE_MAZE_HEALTH` +
   `HEALTH_ONLY`), or the `USE_TELEMETRY=1` build if you also want to run. Power on
   without pressing KEY1. The robot sits in the boot loop and streams `H`/`T`.
2. Connect `bt_monitor.py`. Two things should happen together: the view switches to
   the health panel by itself, and **`Hi ,mmdi` appears in the TERMINAL** — the app
   sends a probe byte the moment it connects and the robot answers with its banner.
   The banner is the only line on the wire that can be a *reply*, so it is the only
   proof the radio works in both directions. If it does not arrive, press **Ping
   robot**; if it still does not, the port or the pairing is wrong and the silent
   bar means nothing either way.
3. **`loop_start`** — the STATE card at the top of the health panel reads it and
   names it. On the bench it is `0  BOOT / BENCH` and stays there. **That is the
   check that `HEALTH_ONLY` took**: anything other than 0 means the mission started.
4. **Keys** — press each of KEY1/KEY2/KEY3. Each indicator lights on press and
   the edge log records the down time and how long it was held. A button that
   never lights is open; one that lights and stays lit past `KEY_STUCK_MS` is
   shorted, and the log is what distinguishes a stuck button from a bounce. In the
   bench build the two calibrations live on the keys: **KEY2 = the IR calibration**
   (it prints its own 18-value `IR_mid` dump, which lands in the TERMINAL) and
   **KEY3 = the gyro one**. KEY1 only re-sends the banner here.
5. **Sensors** — slide a card or a finger under the bar. Every pad it passes
   should track high→low→high with the raw ADC printed in the cell. A pad pinned
   at `0` or `4095` for the whole capture is called out as a FAIL: a pad stuck
   **low** reads BLACK (a lane the brain can see and the robot cannot drive
   down), one stuck at the rail reads WHITE forever (a missing branch detector).
6. **Coordinates** — `GYRO` reads `Gyro_Z` as a rate. A robot standing still
   should read ≈0 deg/s; a large offset is a bias, not motion, and **KEY3** is
   the gesture that calibrates it. Before KEY1, `Z_Angle` is always uncalibrated.
7. **The layout on screen** — it is the **board**, not a sensor bar, and it is drawn
   long and narrow like the board itself. `S0` and `S9` straddle the dashed centre
   line (`S0`/`S9` sit on the rotation axis at the robot's centre), `S1` is set back
   under `S2`, `S8` under `S7`, and the rear bank runs `S17..S10` left to right.
   Each pad carries its own name and ADC; that is all the drawing says — the
   derived summary and the caveats are on the cards beside it. Park the robot on a
   known corner and read the DERIVED card against the map — that is the check the
   app cannot make for you (`SENSORS.md` §3, which lists the three bench questions
   this settles).
8. **The cards scroll** — the column is taller than the window, so the wheel over
   any card scrolls the panel (CHECKS, the verdict, is at the bottom). The
   **THRESHOLDS** table has its own scrollbar: all 18 pads are in it, 10 rows at a
   time, and the wheel over it scrolls the table rather than the panel.
9. Either stop here (bench build), or — if you flashed the `USE_TELEMETRY=1` build —
   press **KEY1**: the stream stops, the mission starts, and the tools switch back
   to the mission view on the first `S`/`J` line.

> **The one thing this fixes on the way past.** The `KEY3` "calibrate gyro"
> gesture in the boot loop was **dead** before this change: it only set
> `GyroCalF`, and the only thing that services that flag is `Calculate_Z_Angle()`,
> which ran solely from the superloop. So the buzz-and-hold did nothing until
> KEY1 — at which point the boot sequence started a fresh calibration anyway.
> `health_tick()` runs `Calculate_Z_Angle()` in the boot loop, so KEY3 now works
> where it is offered. Scoped to the health build, so normal-boot behaviour is
> untouched.

### Headless

```bash
python scripts/bt_monitor.py --replay capture.txt --headless --health
```

Feeds the capture to the same `HealthModel` and prints the **identical health
section** the offline tool does — both go through `pt.health_report()`, so they
cannot drift; only the wrapper differs (`parse_telemetry.py` adds its mission
summary counts above, `bt_monitor.py` adds its exit line below). It exits **1** on
any FAIL, **2** if the capture contains no `H` lines at all — a distinct code so
"nothing to check" can never read as a pass. `build_all.ps1` asserts both
directions against the two synthetic fixtures: `capture_health_ok.txt` → 0,
`capture_health_fault.txt` → 1.

> Those two fixtures are **parser and UI tests with invented values**
> (`scripts/make_health_fixtures.py` builds them from a toy model of the bar).
> They say the tools agree on the wire format and reach the right verdict. They
> say **nothing** about any physical sensor — that is the whole point of the
> bench check, and a synthesised file cannot answer it.

---

## File layout

```
robot codes/
├── scripts/                     # Automation tools
│   ├── run_maze.py                Feed any .json → build → run → result
│   ├── run_brain.py               Same, but drives the decision core (7/7 checks)
│   ├── build_all.ps1              Rebuild + run ALL tests (exits 1 on failure)
│   ├── build_firmware.sh          Build + link the STM32 firmware (ARMCC 5)
│   ├── measure_solver_ram.sh      Measure the 8 KB RAM budget, per object
│   ├── field_to_maze.py           Real field image → maze .json + overlay check
│   ├── parse_telemetry.py         M1 capture → counts/cell, sensors, decisions
│   ├── bt_monitor.py              LIVE Bluetooth capture + virtual-brain check
│   └── make_health_fixtures.py    Regenerate the two synthetic health captures
├── inc/                         # Headers (9 files)
│   ├── maze_types.h               Structs, enums, MazeCommand
│   ├── maze_config.h              Memory limits, motion params
│   ├── maze_graph.h               Node/edge CRUD, Dijkstra
│   ├── maze_robot.h               Heading, F/L/R/B, frontiers
│   ├── maze_explore.h             P1→P2→P3 exploration
│   ├── maze_proof.h               Time-based early-stop proof
│   ├── maze_fastrun.h             Trapezoidal velocity, stop-graph
│   ├── maze_solver.h              TOP-LEVEL FSM
│   └── brain.h                    THE SEAM: BrainIn in, one move out
├── src/                         # Sources (7 files)
│   ├── maze_graph.c
│   ├── maze_robot.c
│   ├── maze_explore.c
│   ├── maze_proof.c
│   ├── maze_fastrun.c
│   ├── maze_solver.c
│   └── brain.c                    Junction report → move; the two plans on DONE
├── test/                        # Test programs (6 files)
│   ├── run_maze.c                 Generic runner (reads _maze_data.h)
│   ├── test_graph.c               Unit: graph module (6 tests)
│   ├── test_robot.c               Unit: robot module (7 tests)
│   ├── integration_test.c         Full mission on sample_maze (8 tests)
│   ├── brain_oracle.c             Decision core, linked AND RUN (--selftest)
│   ├── fixtures/                  Replay captures: agree (0) / disagree (1)
│   └── brain_host.c               Decision core vs a robot model (7/7, 5 mazes)
└── build/                       # Output (gitignored)
    ├── run_maze.exe
    ├── test_graph.exe
    ├── test_robot.exe
    ├── integration_test.exe
    ├── brain.o
    ├── brain_oracle.exe
    └── bt_captures/             Captures written by bt_monitor.py
```

---

## Manual build commands (rarely needed)

### Test 1 -- Graph module

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c test/test_graph.c -o build/test_graph.exe
./build/test_graph.exe
```

### Test 2 -- Robot module

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe
./build/test_robot.exe
```

### Test 3 -- Integration (sample_maze)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c test/integration_test.c -lm -o build/integration_test.exe
./build/integration_test.exe
```

### Test 4 -- Decision core (compile-only, zero-warning check)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc -c src/brain.c -o build/brain.o
python scripts/run_brain.py ../simulator/mazes/real_field.json   # the real run
```

### Test 5 -- Decision core (linked and RUN -- the virtual brain)

```powershell
gcc -std=c11 -Wall -Wextra -pedantic -Werror -I inc src/maze_graph.c src/maze_robot.c \
    src/maze_explore.c src/maze_proof.c src/maze_fastrun.c src/maze_solver.c src/brain.c \
    test/brain_oracle.c -lm -o build/brain_oracle.exe
./build/brain_oracle.exe --selftest     # 5 steps, exit 1 on any mismatch
./build/brain_oracle.exe --probe        # the same steps, unchecked
./build/brain_oracle.exe                # filter mode: BrainIn per stdin line
```

Filter mode is what `bt_monitor.py` drives. The protocol is strictly one reply per
request — `INIT` → `OK init`, then `L R F B TARGET DIST` → `S <fields...>`, and
`QUIT`:

```
$ printf 'INIT\n0 0 1 1 0 0\n0 1 0 1 0 20\n' | ./build/brain_oracle.exe
OK init
S F 0 0 0 0 1 0 0 0 0 0.0000
S R 0 1 0 20 2 1 0 0 0 0.0000
```

Note the `S` fields carry the brain's dead-reckoned `x`/`y` in **its own frame**,
plus `reports`, `cells`, `drift_cm`, `target_found`, `finished` and
`fast_time_s`; on `finished` it also prints the `HOME` and `FAST` plan strings.

Two things that bite if you touch this file: **`brain.c`'s state is static, so one
process is one mission** (a second replay needs a fresh process or an `INIT`), and
**stdout must be flushed per line** — MinGW does not honour `_IOLBF` on a pipe, so
without the explicit `fflush` every response sits in the C library's buffer and the
host deadlocks waiting for an answer that arrived (it is a deadlock, not a slow
run; this cost an afternoon).

---

## Pipeline (who calls who)

```
Your STM32 firmware (main.c)
         │
         │  brain_report(): builds a BrainIn from the globals the firmware
         │  already had, calls brain_step(), gets one move back
         ▼
      brain.h            ◄-- THE SEAM.  4 relative exits + target + dist_cm in,
         │                   'F'/'L'/'R'/'B' or BRAIN_DONE out
         ▼
      brain.c            ◄-- dead reckoning + the two plan strings
         │
         ▼
    maze_solver.c        ◄-- TOP-LEVEL: EXPLORE → RETURN_HOME → FAST_RUN → DONE
         │
    ┌────┼────┬─────────┐
    ▼    ▼    ▼         ▼
 robot  graph explore  proof  fastrun   ◄-- workers called by the solver
```

- **`brain.c`** is what the firmware talks to. The robot owns everything
  physical; the brain owns the map.
- **`maze_solver.c`** runs the state machine and delegates.
- **`maze_graph.c`** + **`maze_robot.c`** are pure logic (no hardware dependency).
- **`maze_explore.c`** + **`maze_proof.c`** + **`maze_fastrun.c`** are the three mission stages.
- There is **no HAL**. `brain.h` never reads a sensor, motor, encoder or compass,
  and holds no firmware pointer — that is the whole point of the seam.

---

## Dependencies between modules

```
maze_types.h     ← everyone includes this
maze_config.h    ← everyone includes this

maze_graph.c     ← no dependencies (pure)
maze_robot.c     ← depends on maze_graph
maze_explore.c   ← depends on maze_graph, maze_robot, maze_proof
maze_proof.c     ← depends on maze_graph
maze_fastrun.c   ← depends on maze_graph
maze_solver.c    ← depends on ALL of the above
brain.c          ← depends on maze_solver (drives it; knows nothing of hardware)
```

---

## On the STM32 (Keil / ARMCC 5)

Already wired up in `../firmware/MDK-ARM/Source.uvprojx` (group `MazeSolver`,
plus `../../robot codes/inc` on the include path). To reproduce by hand:

**Sources** (7 `.c` files): all from `src/`

**Include paths**: add `inc/`

**In `main.c`**:

```c
#include "brain.h"

// On the KEY1 press (edge-detected, so holding the button cannot wipe the map):
brain_init();

// At every junction:
BrainIn in;
in.left = left_poss; in.right = right_poss;
in.front = (s[3] || s[4] || s[5] || s[6]);
in.back = 1; in.target = OnEndZoon; in.dist_cm = <the link just closed, cm>;

char move = brain_step(&in);      /* 'F'/'L'/'R'/'B', or BRAIN_DONE */
// ... the firmware's existing cross dispatch handles the motion
```

On `BRAIN_DONE`, `brain_home_path()` and `brain_fast_path()` supply the two
replay strings. `main.c`'s **`set_plan()`** copies them and appends the legacy
`'D'` end sentinel that the replay stages depend on, and **`replay_dispatch()`**
is the single command executor for both stages.

---

## Quick reference

| Command | What it does |
|---|---|
| `python scripts/run_maze.py <file.json>` | Test any maze (one command) |
| `python scripts/run_brain.py <file.json>` | Test the decision core against a simulated robot |
| `.\scripts\build_all.ps1` | Rebuild + run all unit tests |
| `bash scripts/build_firmware.sh` | Build + link the STM32 firmware → `seyed.hex` |
| `USE_TELEMETRY=1 bash scripts/build_firmware.sh` | **The supervised bring-up build:** +telemetry, +health stream, +5 s pause per junction |
| `USE_HEALTH=1 bash scripts/build_firmware.sh` | **Bench only:** health stream, `HEALTH_ONLY` — the robot cannot start a mission. KEY2 = IR cal, KEY3 = gyro cal, KEY1 = re-send the banner |
| `python scripts/parse_telemetry.py capture.txt` | Turn a capture into measurements + brain decisions |
| `python scripts/bt_monitor.py` | **LIVE capture + virtual-brain decision check (GUI)** |
| `python scripts/bt_monitor.py --replay <cap> --headless` | Re-run a capture with no robot; exit 1 on mismatch |
| `python scripts/bt_monitor.py --replay <cap> --headless --health` | Health verdict, no robot; exit 1 on FAIL, 2 if no `H` lines |
| `./build/brain_oracle.exe --selftest` | The virtual brain's own 5-step self-test |
| `bash scripts/measure_solver_ram.sh` | Check the 8 KB RAM budget (per object, exits 1 on overflow) |
| `python scripts/field_to_maze.py <field.png>` | Real field image → maze `.json` |
| `pip install pyserial` | Only needed for `bt_monitor.py`'s live serial path |
| `gcc --version` | Verify GCC (MSYS2 MinGW) |

**Flags:** `-std=c11 -Wall -Wextra -pedantic` for all; `-lm` for math (fast-run floats); `-Werror` on the `brain.c` compile and on `brain_oracle` (both must be zero-warning). Output goes to `build/` (gitignored). `test/_maze_data.h` is auto-generated and gitignored.
