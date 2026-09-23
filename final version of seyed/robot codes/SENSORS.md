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

**Banks — and they are not the same size.** The **front** row is `s[0]`…`s[9]`, ten
sensors; the **rear** row is `s[10]`…`s[17]`, eight. `head` (0 = driving forward,
1 = driving reversed) selects which row the navigation code reads — that is how the
robot drives "backwards" without spinning.

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

Both rows put their branch detectors exactly ±2.5 pitches from their own centre, and
both centre groups straddle it evenly. That is only possible if the front row spans
`0…9` and the rear spans `10…17`. Under a 9/9 split the front row's centre would sit
at 4.0, leaving `s[3]`…`s[6]` lopsided about it. The rows are also centred *on each
other* — `s[2]` and `s[11]` are at the same lateral offset — which is what makes the
`+9` mirror (`s[k]` → `s[k+9]`) valid. `s[0]` and `s[9]`, the two long outliers, have
no rear counterpart, which is exactly why they are the two with dedicated ADC pins.

---

## 2. Roles — what each index is used for

Taken from the junction decision at `main.c:1366-1407`, which is the only place that
gives the sensors meaning. The same pattern repeats for the rear bank when
`head == 1`.

| Index | Lateral offset | Front | Rear | Role |
|---|---|---|---|---|
| — | −4.5 pitch | `s[0]` | — | outer extreme; used only paired with `s[9]` |
| — | +4.5 pitch | `s[9]` | — | outer extreme; used only paired with `s[0]` |
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

### The `s[0] && s[9]` gate — and what it costs

The decision block is gated on the two outer front sensors, the outermost pair:
**±45.9 mm** at the 10.2 mm pitch, so a **91.8 mm** span.

```c
if (right_poss || left_poss)
{
    if (left_poss && (s[0] && s[9]))                     { cross=1; path_append('L'); }  /* 1372 */
    else if (left_poss==0 && right_poss==1 && s[9] && s[0]) { ... }                      /* 1378 */
}
```

A **one-sided** branch cannot satisfy `s[0] && s[9]`: a branch to the left puts black
left of the bar, not at +45.9 mm. So at a T-junction, and at a corner, the gate is
false, no `cross` is chosen, and control falls through to `Forward()` **with no
`path_append()`** — that branch is never recorded. `cross` works by staying stale at
0, which is why the `Forward()` at 1412 reads as "no decision" as well as "straight".

That is *correct for the left-hand rule*: at a T with forward open, going straight is
the left-hand-rule answer and the gate is what stops the robot turning. It is not a
bug in the legacy explorer. It **is** the wrong junction test for the brain, which
needs every real junction reported or it will never learn that branch — see the
integration notes in `inc/brain.h`.

**The implication, worked out from the pitch geometry.** Sampling the bar at the
node centreline, the black under `s[0]`..`s[9]` is, per node type (robot travelling
north, line 20 mm wide, offsets from SENSORS.md §3):

| node type | black at the bar | `left_poss` | `right_poss` | `s[0]&&s[9]` |
|---|---|---|---|---|
| straight `N+S` | `\|x\| <= 10` | 0 | 0 | no |
| corner `S+W` | `x <= 10` | 1 | 0 | **no** |
| T `S+N+W` | `x <= 10` | 1 | 0 | **no** |
| T `S+N+E` | `x >= -10` | 0 | 1 | **no** |
| 4-way | everything | 1 | 1 | **yes** |

So during discovery the gate passes **only at a 4-way**, where `left_poss &&
(s[0]&&s[9])` gives `'L'` — and the only other report is `'B'` at a dead end.
`'S'` needs `left_poss==0`, and `'R'` needs the centre group *off* the line; neither
can happen at a 4-way. **The legacy explorer therefore turns only at 4-ways, goes
straight at every T, and records nothing at either.** That is a coherent
left-hand-rule walk — the branch it declines to take is one the rule has already
decided against — and it is why the recorded map is a walk, not a map.

This is derived from the geometry, not observed. It is checkable in one capture:
count the `J` lines and compare `cross` against the `front` mask on each.

**And it has a consequence for the legacy map.** `nav` — the firmware's heading, and
the axis `node[]` is accumulated along — is updated **only by explicit turns**:

| primitive | `nav` | line |
|---|---|---|
| `turn_left()` / `turn_left_r()` | `nav++` | 614 / 652 |
| `turn_right()` / `turn_right_r()` | `nav--` | 686 / 723 |
| U-turn (`cross==4`) | `nav+=2` | 1434 / 1520 |
| `Forward()` / `Forward_r()` | **untouched** | 613 / 651 |

(`nav` is 0=N, 1=W, 2=S, 3=E — `main.c:80` — identical to `MazeHeading`.)

So a turn the robot takes **without** passing the gate — line-following round a
corner — leaves `nav` stale, and `node[i+1] = node[i] ± link[i][0]` (`main.c:1550-1553`)
accumulates that error for the rest of the mission.

Worth measuring rather than asserting: every `J` line carries `nav`, so one capture
shows directly whether `nav` advances by 1 per real 90° turn. Either way it does **not**
affect the solver — the brain keeps its own heading and updates it from the moves it
issued, never from `nav`.

### What this means for the solver

The HAL is already structurally safe here, which is worth recording so nobody
"fixes" it: it does **not** index the sensor array itself for branch decisions. It
reads the firmware's latched `left_poss` / `right_poss` flags
(`maze_hal.h:148-151`), so it inherits the firmware's branch logic exactly and cannot
disagree with it about which sensor is which. The only raw indices it touches are the
centre groups `s[3]`…`s[6]` and `s[12]`…`s[15]` (`maze_hal.h:141-144`), which are
correct under either bank split — so the 10/8 correction above changes no code.

**Do not add raw index reads for branch detection.** `s[0]`, `s[1]`, `s[8]` and
`s[9]` are not branch detectors — feeding them to the graph would sprout phantom
edges at every node. This is also the reason for the "read sensors *before* the
motion primitive" rule in the integration contract: `turn_left()` clears
`left_poss`/`right_poss` itself (`main.c:602-603`, `640-641`, `674-675`, `710-711`),
so a read taken after the call sees zeros.

---

## 3. Measured geometry

Supplied by the user, measured on the real board:

| Measurement | Value | Implies |
|---|---|---|
| `s[2]` → `s[7]` centre-to-centre | **51 mm** | active array is 6 sensors on **5 gaps** → **pitch = 10.2 mm** |
| `s[1]` → `s[17]` centre-to-centre | **76 mm** | see the open question below — `s[17]` is in the *other* bank |

Derived from the 10.2 mm pitch, for the front bank (the rear bank mirrors it at +9):

| Span | Sensors | Width |
|---|---|---|
| centre group `s[3]`…`s[6]` | 4 | 30.6 mm |
| **active array `s[2]`…`s[7]`** | 6 | **51.0 mm** |
| outer pair `s[1]`…`s[8]` | 8 | 71.4 mm |

So the branch detectors sit **±25.5 mm** either side of the array centreline, and the
rocker is ±25.5 mm — a branch is only *seen* while the bar is within roughly half a
line-width of the node. That is the number that sets how much the robot may be
off-centre when it makes a junction decision.

**Sanity check against the field** (§4): the track is 20 mm wide and the cell is
200 mm. An active array of 51 mm is 2.5 line-widths — wide enough that 2-3 sensors
hold the line at once, narrow enough to fit well inside a cell. This geometry is
consistent; nothing here fights the solver.

### Open question — the second measurement does not fit

`s[17]` is in the *rear* bank, so `s[1]` → `s[17]` is not a same-row span. Checking it
against the 10.2 mm pitch:

- If it were really `s[1]` → `s[8]` (a typo for the number I asked for), 76 mm over
  7 gaps gives pitch **10.86 mm** — **6% off** the 10.2 mm from the first
  measurement. Bigger than caliper error, so it probably is not this.
- If the two rows run parallel and `s[1]`→`s[17]` is the **diagonal** across them,
  then `76² = 71.4² + d²` → the rows are **26 mm apart**. Very close — at that
  separation "front bank" and "rear bank" would barely differ.
- If the rows are numbered in opposite directions, `s[17]` sits behind `s[1]` and
  76 mm is the **row separation** directly. But the firmware mirrors the banks at
  +9 (`s[2]`→`s[11]`, `s[7]`→`s[16]`), which only works if both rows are numbered
  the same way — so this reading contradicts the code.

The middle reading is the one the arithmetic supports, but it needs confirming before
anything depends on it.

### Sensor lead: 57 mm from the robot centre to the front sensor line

The bar looks **57 mm ahead of the robot's centre** (assumed to be at/near the drive
axle — see below). At `v_max` = 100 cm/s the robot covers that in **57 ms**.

Where this does and does not matter:

- **It cancels out of link lengths.** Every junction is detected at the same bar
  position, so an encoder reading taken junction-to-junction measures exactly the
  travel between two junctions — the 57 mm is on both ends. This is why the link
  lengths the firmware reports already look right.
- **It does not cancel out of the maneuver offsets.** `path_append()` adds
  `+25/+30/+85/+95/+104` depending on the previous move (`main.c:847`). Those
  differences exist *because* of this lead: turning takes more or less arc than
  going straight, and the bar ends up a different distance from the axle each time.
  The 57 mm is the physical quantity behind those empirical numbers.
- **It shifts the first node and the target hit.** The start node is where the robot
  is *placed*, which is a bar position; the target is detected when the bar is over
  the disc (`end_zone_timer > 2`), 57 mm before the axle arrives.
- **It is the turn budget in the fast run.** The robot commits to a turn with the bar
  at the node and completes it over the following 57 mm of travel. A fast run that
  penalises each turn is therefore right to — the turn is physically "free" distance
  but not free time.

**Assumption to check:** 57 mm was measured from the *robot centre*. If the drive
axle is offset from the centre, the number that matters is axle-to-bar. For this
chassis they should be within a few mm; worth a glance.

---

## 4. Field measurements taken from the image

From `robot and field data/field.png` (582 × 458, a flat graphic export — a 7 KB PNG,
so no photographic distortion; the line positions are exact, not estimates):

- Track width: **6-7 px**
- Grid pitch: **62.5 px**, and the field is a 20 cm grid → **≈ 3.125 px/cm**
- Therefore the tracks are **≈ 2.0 cm wide** and the cell is **20 cm** — which matches
  the grid size already assumed everywhere else in this project
- Target ("big black area") is a filled disc of radius ≈ 20 px ≈ **6.4 cm**, centred
  exactly on a grid **intersection** — confirming the field is a *line* maze, where
  the black lines are the tracks and the disc marks a node on them
