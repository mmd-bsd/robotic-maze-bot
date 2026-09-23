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