# SEYED IR sensor map — authoritative reference

**Why this file exists.** `maze_hal.h` reads the firmware's `s[]` array, so the solver
inherits whatever roles the line-following firmware already gives those indices. Those
roles were derived, not documented. This file pins the mapping down from source and
records exactly which numbers are *proven* and which still need a tape measure.

**Sources (both authoritative, both read-only):**
- `firmware/Core/Src/main.c:1134` — `s[i]` is produced from `IR_ADC[i]`, 1:1
- `firmware/Core/Src/Hardware.c` — `Read_IRSensors()`, the ADC/MUX assignment
- `firmware/Core/Src/main.c:1366-1407` — the junction decision, which fixes the roles
  (an earlier revision of this file said `1202-1214`; that region is now
  commented-out I2C scan code, the block has shifted)

---

## 1. The chain: silkscreen → MUX → `IR_ADC[]` → `s[]`

`Read_IRSensors()` fills 16 of the 18 channels through an 8-state analog MUX
(`IR_ReadCounter` 0…7, selected by `MUX_A/B/C`), plus two channels wired straight to
ADC pins. Written out:

| MUX ch | front bank | rear bank | silkscreen |
|---|---|---|---|
| — (direct ADC) | `IR_ADC[0]` = `adcv[2]` | `IR_ADC[9]` = `adcv[0]` | (the two inner pads) |
| 5 | `IR_ADC[1]` | `IR_ADC[10]` | **S1** / **S10** |
| 7 | `IR_ADC[2]` | `IR_ADC[11]` | **S2** / **S11** |
| 2 | `IR_ADC[3]` | `IR_ADC[12]` | **S3** / **S12** |
| 1 | `IR_ADC[4]` | `IR_ADC[13]` | **S4** / **S13** |
| 0 | `IR_ADC[5]` | `IR_ADC[14]` | **S5** / **S14** |
| 3 | `IR_ADC[6]` | `IR_ADC[15]` | **S6** / **S15** |
| 4 | `IR_ADC[7]` | `IR_ADC[16]` | **S7** / **S16** |
| 6 | `IR_ADC[8]` | `IR_ADC[17]` | **S8** / **S17** |

**The useful consequence, and the thing to remember:**

> `s[i]` ≡ silkscreen `S<i>` for every i in 1…8 and 10…17.

The firmware index and the PCB silkscreen are the same number. So you can read a
sensor number off the board and index `s[]` with it directly — no lookup table needed
on the bench. Only `s[0]` and `s[9]` break the pattern: they are the two direct-ADC
pads and carry the silkscreen of their bank instead.

The MUX ordering looks scrambled but is not: counters 0-7 are assigned so that the
`S` numbers come out in order, which is why S1 is on channel 5 and S2 on channel 7.

**Banks — and they are not the same size.** The **front** bank is `s[0]`…`s[9]`, ten
sensors; the **rear** bank is `s[10]`…`s[17]`, eight. `head` (0 = driving forward,
1 = driving reversed) selects which bank the navigation code reads — that is how the
robot drives "backwards" without spinning. ("Bank", not "row": neither is a straight
line of ten — see the physical layout below.)

*How the 10/8 split was established, because 9/9 is the obvious guess and it is
wrong.* The MUX assignment explains the *count* — one 8-channel MUX serves 8 front
(`s[1]`…`s[8]`) plus 8 rear (`s[10]`…`s[17]`), and `s[0]`/`s[9]` are the two left over
that got wired straight to ADC pins. The *membership* is then forced by symmetry in
the firmware's own index choices:

| | front | rear |
|---|---|---|
| centre group | `s[3]`,`s[4]`,`s[5]`,`s[6]` | `s[12]`,`s[13]`,`s[14]`,`s[15]` |
| implied row centre | **4.5** | **13.5** |
| branch detectors | `s[2]`, `s[7]` | `s[11]`, `s[16]` |
| distance from centre | −2.5, +2.5 pitches | −2.5, +2.5 pitches |

Both banks put their branch detectors exactly ±2.5 pitches from their own centre, and
both centre groups straddle it evenly. That is only possible if the front bank spans
`0…9` and the rear spans `10…17`. Under a 9/9 split the front bank's centre would sit
at 4.0, leaving `s[3]`…`s[6]` lopsided about it. `s[0]` and `s[9]`, the two long
outliers, have no rear counterpart, which is exactly why they are the two with
dedicated ADC pins.

> **⚠ The `+9` mirror is an *index* relation, and the board does not obviously
> honour it.** It is normally stated as "`s[2]` and `s[11]` are at the same lateral
> offset". Read off the board, the rear bank's silkscreen runs the **opposite** way
> round to the front's, which puts `s[11]` at the rear row's **right** end — the
> opposite lateral offset to `s[2]`. So either the rear bank is mounted mirrored on
> the board or the firmware's rear left/right is swapped. That is §3's item 3, and it
> is a question about the robot rather than about this arithmetic — the index
> identity `s[k] ↔ s[k+9]` still holds, it is the *left/right* meaning that is in
> doubt.

### Physical layout — the pads are a RING, and the index runs round it

**Read off the board itself, 2026-09-23**: `specifiction/sens num order.jpg` (the
top view with the pads numbered) and `New Start/robot sensore/sensores.png` (the
bare board). Nothing in the firmware says any of this; §1's `+9` relation is about
the firmware's *index* arithmetic, not about the board.

The pads are **not two straight rows**. They ring the perimeter, and the index runs
**clockwise round that ring starting at the robot's centre**:

```
                     FRONT EDGE OF THE ROBOT
         S1                                        S8       side pads, set back
           S2    S3    S4    S5    S6    S7                (s[1] is under s[2],
                                                             s[8] is under s[7])
                     S0          S9
   [===== left motor =====][===== right motor =====]
        ^ the axis of rotation runs between the motors, at the robot's centre
         S17                                      S10       side pads, set back
           S16   S15   S14   S13   S12   S11
                     REAR EDGE OF THE ROBOT
```

Three things follow, and the first is the one that matters:

- **`s[0]` and `s[9]` sit side by side ON THE AXIS OF ROTATION**, at the robot's
  centre — not at the ends of a row. That is what makes `s[0] && s[9]` mean *the
  axis is over a node*.
- **`s[1]` and `s[8]` are the side pads**, set back behind the front row: `s[1]`
  under `s[2]`, `s[8]` under `s[7]`. So the eight target pads `s[1]`…`s[8]` form a
  **U**, not a line — which is how a 128 mm disc blacks all eight at once.
- The **rear bank reads `S17…S10` left to right** — the reverse of the front.

> **This is the geometry §1's arithmetic already assumes, once "the row" is read as
> the U's outer arc.** The arc `s[1]`…`s[8]` is symmetric about index **4.5**, which
> puts the branch detectors `s[2]`/`s[7]` exactly −2.5/+2.5 pitches out; the rear
> arc `s[10]`…`s[17]` is symmetric about **13.5** the same way. The two extra front
> pads `s[0]`/`s[9]` sit *inside* the ring, on the axis, and have no rear
> counterpart — which is exactly why they are the two wired straight to ADC pins
> rather than onto the MUX. Nothing in §1 changes.

**In code the geometry has ONE definition**: `parse_telemetry.PAD_CELL`, a
(column, row) cell per pad plus `BOARD_COLS`/`BOARD_ROWS`. The live canvas draws
from it and the text report prints it, so the two cannot drift.
`TARGET_ROW`/`REAR_TARGET_ROW` stay in index order because the target test is
order-blind (the whole U has to be black at once).

The grid is **portrait**, because the robot is: the board is about 335 mm across
and 570 mm long, and `BOARD_ROWS`/`BOARD_COLS` = 29/17 = 1.71 is that same
shape. A grid wider than it is tall draws a robot that does not exist, and it
hides the thing the picture is for — that `S0`/`S9` sit at *mid-length*, on the
axis, a long way back from the front row. The canvas draws the board outline,
the dashed centre line and the pads; the pad's own name and ADC are inside the
cell, and **nothing else is written on it**, because the derived summary, the
role words and every caveat are on the cards beside it.

> **Consequence for the rear bank, and an open question.** The rear bank's indices
> run the opposite way round on the board, so the `+9` mirror lands `s[11]` — the
> firmware's rear **left** branch detector — at the rear row's **right** end, under
> the front row's right side, and `s[16]` at its left. Either the rear bank is
> mounted mirrored on the board, or the firmware's rear left/right is swapped. Both
> are plausible; neither is established. Park the robot on a corner it should turn
> **left** from and read whether `S11` or `S16` is the pad over the black lane.
> That is a fact about the robot, and it is the third thing the §3 bench check can
> settle.

> **Superseded reading, for the record.** A first pass at a different top view
> concluded that `S0`/`S9` were the outer ends of a ten-wide front row. That was
> wrong — the board says otherwise, and it contradicted §2's earlier measurement,
> which was right. Both the code and this file now draw the ring above.

### Polarity — which way round the numbers go

**This was not written down anywhere before 2026-09-23**, and it inverts the meaning
of both rails and of the calibration names. From the live path at `main.c:1960-1967`:

```c
if      (IR_ADC[i] >= IR_mid[i] + 50)  s_current[i] = 0;   /* white */
else if (IR_ADC[i] <= IR_mid[i] - 50)  s_current[i] = 1;   /* black */
```

> **Low ADC = BLACK. High ADC = WHITE.** A pad over the reflective field reads
> **high**; over the line or the target disc it reads **low**.

The pad is a reflective sensor, so this is the expected direction (less light back
→ lower reading), but the `IR_min`/`IR_max` *names* suggest the opposite, so:

- **`IR_max` is the WHITE end**, `IR_min` the **BLACK** end — the reverse of how
  they read. `calibr_ir()` (`main.c:519-569`) spins `MotorB(40,-40)` for 185
  iterations tracking each channel's extremes, then
  `IR_mid[i] = (IR_max[i] + IR_min[i]) * 0.5`.
- A pad pinned at **0** reads BLACK for ever — a phantom lane, or a whole phantom
  target row. That is the dangerous direction.
- A pad pinned at **4095** reads WHITE for ever — a dead phototransistor, i.e. a
  missing branch detector.

**Two different bands, deliberately.** The live path uses a symmetric ±50 and then
debounces: `s[]` only flips after `s_counter[i] > OffToOnTrsh` (5) / `> OnToOffTrsh`
(5) consecutive disagreeing samples (`main.c:96-97`, `main.c:1971-1986`). The
one-shot init just after KEY1 uses a **wider, asymmetric** band (`main.c:1344-1351`):

```c
if      (IR_ADC[i] >= IR_mid[i]+100)  s[i]=0;
else if (IR_ADC[i] <= IR_mid[i]-500)  s[i]=1;      /* no else: keeps the old value */
```

Black is the harder call to make (needs −500) because at boot `IR_mid[]` holds the
power-on defaults (`main.c:90`: `{1400,1200,1400,…}`) — **not** anything the robot
measured — so a loose black threshold would invent lanes out of the field's glare.
Outside the two bands the bit is *latched*, not freshly decided; that is why a
reading sitting within ±100 of `mid` is reported as inconclusive rather than as a
state.

---

## 2. Roles — what each index is used for

Taken from the junction decision at `main.c:1366-1407`, which is the only place that
gives the sensors meaning. The same pattern repeats for the rear bank when
`head == 1`.

| Index | Position | Front | Rear | Role |
|---|---|---|---|---|
| — | **on the rotation axis, robot centre** | `s[0]` | — | centre-of-node detector; paired with `s[9]` |
| — | **on the rotation axis, robot centre** | `s[9]` | — | centre-of-node detector; paired with `s[0]` |

> **CORRECTION — an earlier revision of this file placed `s[0]`/`s[9]` at ±4.5 pitch,
> i.e. the outer extremes of the front row. That was wrong.** Measured on the board
> by the user: both sit **in the middle of the robot, on the axis of rotation**. They
> are the two direct-ADC pads (`adcv[2]` and `adcv[0]`), which is why they are wired
> separately from the MUX and why they break the `i ≡ S<i>` numbering.
>
> **CONFIRMED off the board, 2026-09-23.** `specifiction/sens num order.jpg` shows
> the pads directly: `S0` and `S9` are the two pads either side of the motors'
> centre line, with `S1` set back behind `S2` and `S8` behind `S7`. So the
> correction above stands and §1's physical-layout section is drawn from it. (A
> first pass at a different top view wrongly took `S0`/`S9` for the outer ends of a
> ten-wide front row; that reading is withdrawn, and it was flagged here as OPEN
> for a few hours.)
>
> That makes `s[0] && s[9]` a **position gate, not a "black on both sides" gate**: it
> fires when the robot's rotation axis is over the node, i.e. **"you have arrived,
> decide now."** It is satisfied at every node type — crossing, T, corner — not only
> at a 4-way. Everything below that was derived by placing them at the extremes is
> withdrawn; see the note under §2.
>
> **Open:** whether the pair is ANDed or ORed is not yet established. `&&` is what
> the source reads at `main.c:1372`/`1378`, but `cross` also has stale-value
> behaviour that could mask an OR. The `J` telemetry lines settle it — they carry the
> raw `front` mask, so the two bits can be read directly against `cross`.
| a | −3.5 pitch | `s[1]` | `s[10]` | **target detector** (part of the all-black row test) |
| b | −2.5 pitch | `s[2]` | `s[11]` | **left branch detector** |
| c | −1.5 pitch | `s[3]` | `s[12]` | on-line, centre group |
| c | −0.5 pitch | `s[4]` | `s[13]` | on-line, centre group |
| c | +0.5 pitch | `s[5]` | `s[14]` | on-line, centre group |
| c | +1.5 pitch | `s[6]` | `s[15]` | on-line, centre group |
| b | +2.5 pitch | `s[7]` | `s[16]` | **right branch detector** |
| a | +3.5 pitch | `s[8]` | `s[17]` | **target detector** (part of the all-black row test) |

`head == 1` should be read as the same table with the front column swapped for the
rear one — the roles are identical, only the indices shift by 9.

**Target detection.** The "big black area" is recognised by the whole inner row going
black at once (`main.c:1167`):

```c
if ((s[1] && s[2] && s[3] && s[4] && s[5] && s[6] && s[7] && s[8])  ||
    (s[10] && s[11] && s[12] && s[13] && s[14] && s[15] && s[16] && s[17])) {
    end_zone_timer++;
} else { end_zone_timer = 0; }
```

Target = that held for more than 2 loop iterations. Two things follow:

1. **`s[1]` and `s[8]` are not unused** — an earlier revision of this file said they
   were. They are not *branch* detectors, but they are half the target test.
2. The target must be at least as wide as the `s[1]`…`s[8]` span — **71.4 mm** at the
   measured pitch. The disc on the real field measures **128 mm** across, so it
   covers the row with 28 mm to spare on each side. That is a comfortable margin and
   a good sign; a target closer to 71 mm would make detection knife-edge.

Note the commented-out variant at `main.c:1163` tests
`s[0] && s[1] && s[2] && s[7] && s[8] && s[9]` — symmetric only under the 10-sensor
front row, which is further confirmation of the split above.

The decision itself:

```c
if (s[2] && (s[3] || s[4] || s[5] || s[6]))  left_poss  = 1;   /* a branch opens left  */
if (s[7] && (s[3] || s[4] || s[5] || s[6]))  right_poss = 1;   /* a branch opens right */
```

The `&& (s[3] || s[4] || s[5] || s[6])` guard is the important half: a side sensor only
counts as a branch **while the robot is still on the line**. That is what stops the
robot from calling every bit of stray black on the field a junction.

Dead end is the mirror image — centre group all off, held for `head_delay >= 50` ticks:

```c
else if (s[3]==0 && s[4]==0 && s[5]==0 && s[6]==0 && head_delay >= 50) { cross = 4; }
```

### The `s[0] && s[9]` gate — the "you are at the node" signal

The decision block is gated on the pair `s[0] && s[9]`, which sit on the robot's
rotation axis (see the correction in §2). So the gate means **"the rotation axis is
over the node"** — it is the firmware's *arrival* test, not a test of which branches
exist. `left_poss`/`right_poss` say what the branches are; the gate says when to act
on them.

Read that way the block is a clean left-hand rule, and the fall-through is the
"not there yet" case rather than an unhandled one:

| at the node, gate true | `left_poss` | `right_poss` | centre | result |
|---|---|---|---|---|
| crossing | 1 | 1 | on | `'L'` — left, priority 1 |
| T, branch left | 1 | 0 | on | `'L'` |
| T, branch right | 0 | 1 | on | `'S'` — left blocked, straight is next |
| corner | 1 | 0 | on | `'L'` |
| dead end | 0 | 0 | off | `'B'` — after `head_delay >= 50` |
| *not at the node* | — | — | — | `Forward()`, nothing recorded |

An earlier revision of this file concluded from pitch arithmetic that the gate could
only pass at a 4-way, and that the legacy explorer silently skipped one-sided
branches. **That conclusion is withdrawn** — it rested entirely on the wrong positions
for `s[0]`/`s[9]`. With the pair on the rotation axis there is no such restriction.

**And it has a consequence for the legacy map.**

---

## 3. Watching the roles live — the bench check

Everything above is derived from source. The way to check it against the actual
robot is the **health build** (`BUILD_GUIDE.md` §"Health check"), which streams all
18 raw `IR_ADC[]` values, the three keys and the gyro over Bluetooth **while the
robot is stopped**, and `bt_monitor.py` draws them in the **top view** in the
physical layout §1 establishes: the ring, with `S0`/`S9` straddling the dashed
centre line and the rear bank running `S17…S10` left to right. Both the panel and
the text report are drawn from `parse_telemetry.PAD_CELL`, so the screen cannot
disagree with §1.

```bash
USE_HEALTH=1 bash scripts/build_firmware.sh      # bench build: no mission at all
python scripts/bt_monitor.py                     # health panel is automatic while idle
```

The panel's **STATE** card shows `loop_start` and names it; on the bench build it
reads `0 BOOT / BENCH` and stays there, which is also the proof that `HEALTH_ONLY`
took (anything else means the mission started — press nothing, and if it moves,
that is the finding). `Hi ,mmdi` should appear in the app's **TERMINAL** the moment
you connect: the app probes, the robot answers. That banner is the only line on the
wire that can be a *reply*, so it is the only proof the link works both ways.

Each cell shows its raw ADC and the black/white state derived from `adc` vs
`IR_mid`, coloured by **role**, so the §2 table can be read straight off the screen.
The pad's name and ADC are the **only** words on the drawing — the derived summary,
the role words and every caveat are on the cards beside it, so the picture is not
competing with its own footnotes. (The cards scroll: the wheel over a card moves
the panel, and the THRESHOLDS table — one row per pad, 18 of them — has a
scrollbar of its own.) That makes the three things this file cannot
settle from a desk answerable in one minute on the bench:

1. **The `s[0] && s[9]` gate, AND or OR** (the open question in §2). Park the robot
   on a junction **by hand** and read the two gate cells. The `DERIVED` card prints
   `at_node = S0 && S9` from the ADC, so a pose where exactly one of
   the pair is over black is immediately visible — and the answer is a fact about
   the robot rather than about the source.
2. **The roles themselves.** Move a card under the bar and watch which cells track
   it. A pad that tracks white and black perfectly can still be the **wrong pad**.
3. **The rear bank's left/right**, given its indices run reversed on the board
   (§1). Park on a corner the robot should turn **left** from and see whether `S11`
   or `S16` is over the black lane. This is the one item on this list that the
   board read has *raised* rather than settled.

> **A green strip is not a verdict.** The panel shows what the pads *read*, which
> is exactly what a synthesised fixture cannot test. The two committed health
> fixtures are parser and UI tests with invented values
> (`scripts/make_health_fixtures.py`); passing them says nothing about any sensor
> on the board.

The `J` telemetry lines remain the authority on what the *firmware* believed at a
junction (they carry the raw `front` mask, `raw`, `cross` and the brain's move),
so the bench check and an M1 capture answer different questions and are worth
running together — the health view for the statics, the `J` lines for the dynamics.