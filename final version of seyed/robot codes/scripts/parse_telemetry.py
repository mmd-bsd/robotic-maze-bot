#!/usr/bin/env python3
"""parse_telemetry.py -- turn an M1 telemetry capture into bench measurements.

WHAT THIS IS FOR
The M1 firmware build (`USE_TELEMETRY=1 bash scripts/build_firmware.sh`) streams
three line types over the Bluetooth link at 115200 baud.  Capture that text to a
file, then run this.  It answers the three questions the solver port is blocked
on, none of which can be answered from a desk:

  1. ENCODER CALIBRATION -- how many counts is one 20 cm cell, really?  The
     firmware assumes 2.467 counts/mm per wheel, i.e. 4.934 counts/cm on the
     summed encoder.  If that is wrong, every link length is wrong, and with it
     the whole map.  This estimates the true quantum from the raw counts.
  2. WHICH SENSORS FIRE AT A JUNCTION -- validates SENSORS.md.  If the branch
     detector is not the sensor we think it is, the graph gets phantom edges.
  3. THE BRANCH WINDOW -- how long the branch detector is live as the bar
     crosses the node.  This is the robot's timing budget for the decision.

LINE FORMAT (see the block comment in firmware/Core/Src/main.c)
  S,<ms>,<front>,<rear>,<e0>,<e1>,<L>,<R>,<cross>      125 Hz, while driving
  J,<ms>,<ch>,<nav>,<head>,<raw>,<cm>,<front>,<rear>,<L>,<R>,<cross>,<target>,<dist>,<node>
  B,<ms>,<node>,<x>,<y>,<drift>,<move>                 bring-up pause only
  Z,<ms>,<state>                                       target zone enter/leave
  H,<ms>,<keys>,<loop>,<head>,<gz>,<za>,<a0>,...,<a17>     20 Hz, health build
  T,<ms>,<what>,<v0>,...,<v17>        what=mid|min|max    ~0.35 s, health build

  <front> = s[0..9] as hex, bit i = s[i]
  <rear>  = s[10..17] as hex, bit i = s[10+i]
  <raw>   = raw summed-encoder counts for the link, BEFORE the /(2.467*2)
  <cm>    = the firmware's own converted+offset length, for comparison
  <ch>    = the move the BRAIN chose: F/L/R/B, or D when it finished
  <target>= 1 if the target-zone latch (OnEndZoon) was set at this junction
  <dist>  = the distance in cm handed to the brain for this link
  <node>  = the brain's node id for this junction (0..N-1, or 65535 = invalid)

  THE H AND T LINES ARE A DIFFERENT BUILD (USE_MAZE_HEALTH, see main.c).
  They stream whenever the robot is STANDING STILL -- before KEY1 and after a
  run -- which is the one state the S/J/Z stream above never covers, so the two
  sets never appear at once.  Where S/J/Z say what the firmware BELIEVED, H/T
  are the raw bench evidence:

  <keys>  = hex bitmask, live: 1 = KEY1 (PB5), 2 = KEY2 (PC15), 4 = KEY3 (PC14)
  <loop>  = loop_start, so the reader can say which state the robot is in
  <head>  = which bank leads (0 = the front row, which is what the bench is)
  <gz>    = Gyro_Z  in deg/s x 10   } integers on purpose: this firmware links
  <za>    = Z_Angle in deg   x 10   } no float printf at all, and one %.1f
                                      would pull the formatter in for 1-2 KB
  <a*>    = IR_ADC[0..17], the raw sensor array
  <v*>    = IR_mid / IR_min / IR_max -- the calibration evidence, three arrays
            on three separate lines rather than one 54-value line, because
            BLT_SendData() restarts the TX DMA and a long line is a long window
            in which an event would truncate it

  There is deliberately NO s[]/front/rear mask on an H line.  On the bench s[]
  is still all zero -- the hysteresis block that fills it lives in the superloop
  and its one-shot init runs only after KEY1 -- so a mask here would be a third
  derivation of the same thing rather than the firmware's own value.  Derive it
  from <a*> vs <v*> instead (HealthModel does exactly that), and read the S/J
  lines for what the firmware actually believed while it was driving.

  NOTE ON <ch>: this used to be the legacy explorer's own decision, and it was
  logged AFTER the move was chosen.  It is now the brain's move, so the J line
  is the record of what the brain was told and what it answered -- which is what
  the bring-up check is for.  The three trailing fields were added with it; a
  capture from an older build will not parse.

A CAVEAT THIS SCRIPT PRINTS RATHER THAN HIDES
The stream is 125 Hz, so sensor masks are sampled every 8 ms.  The branch window
is only ~20 ms wide, so the window is resolved to roughly 2-3 samples -- enough
to say *which* sensors fire and *whether* the window is a couple of samples or
much longer, not enough to measure it to the millisecond.  If a precise window
is ever needed, the fix is an on-chip ring buffer dumped after the run, not a
faster radio link.

Usage:
  python scripts/parse_telemetry.py capture.txt
"""

import argparse
import re
import sys
import textwrap

# Console is cp1252 on this machine; unicode in output crashes it (CLAUDE.md #8).
sys.stdout.reconfigure(encoding="utf-8")

# From SENSORS.md §3/§4 and maze_config.h -- kept as named constants so the
# report states where its expectations come from instead of hard-coding numbers.
DISC_WIDTH_MM = 128.0    # target disc diameter, measured on the real field
V_MAX_CM_S = 100.0       # MAZE_V_MAX_FP / 100, the fault-speed bound

# SENSORS.md §2: silkscreen names and roles, indexed the same way the firmware
# indexes s[].  Front is s[0..9], rear is s[10..17] (a 10/8 split, not 9/9).
FRONT_ROLE = {
    0: ("S0", "outer pad (only ever used paired with S9)"),
    1: ("S1", "TARGET detector"),
    2: ("S2", "LEFT branch detector"),
    3: ("S3", "centre"),
    4: ("S4", "centre"),
    5: ("S5", "centre"),
    6: ("S6", "centre"),
    7: ("S7", "RIGHT branch detector"),
    8: ("S8", "TARGET detector"),
    9: ("S9", "outer pad (only ever used paired with S0)"),
}
REAR_ROLE = {
    10: ("S10", "TARGET detector"),
    11: ("S11", "LEFT branch detector"),
    12: ("S12", "centre"),
    13: ("S13", "centre"),
    14: ("S14", "centre"),
    15: ("S15", "centre"),
    16: ("S16", "RIGHT branch detector"),
    17: ("S17", "TARGET detector"),
}


def sensor_name(i):
    """Index -> silkscreen name.  `s[i]` IS `S<i>` for 1..8 and 10..17
    (SENSORS.md §1); only 0 and 9 break the pattern, and they are named."""
    return (FRONT_ROLE.get(i) or REAR_ROLE.get(i))[0]


def sensor_role(i):
    return (FRONT_ROLE.get(i) or REAR_ROLE.get(i))[1]


# ------------------------------------------------------- the health build
#
# Keys, as the firmware packs them into an H line's <keys> field
# (main.c health_keys(); pins from firmware/Core/Inc/Hardware.h).
KEY_BITS = ((1, "KEY1", "PB5"), (2, "KEY2", "PC15"), (4, "KEY3", "PC14"))

# Sensor geometry, all from SENSORS.md §1/§2.  Kept as named sets rather than
# magic numbers so the health view and the health report cannot disagree about
# which pads are "the centre group" -- a disagreement there would silently
# change the meaning of the front flag the brain is handed.
CENTRE_PADS = (3, 4, 5, 6)           # s[3..6] == in.front
LEFT_PAD, RIGHT_PAD = 2, 7           # the front branch detectors
TARGET_PADS = (1, 8)                 # half of the all-black row test
GATE_PADS = (0, 9)                   # on the rotation axis: "you are at a node"
# The whole inner row that has to go black at once to latch the target
# (main.c:1266-1267): the target pads plus the centre group plus the branch
# detectors, i.e. s[1]..s[8].
TARGET_ROW = (1, 2, 3, 4, 5, 6, 7, 8)
# The rear bank mirrors the front by +9 (SENSORS.md §2), with the same roles.
REAR_CENTRE_PADS = tuple(i + 9 for i in CENTRE_PADS)
REAR_LEFT_PAD, REAR_RIGHT_PAD = LEFT_PAD + 9, RIGHT_PAD + 9
REAR_TARGET_ROW = tuple(i + 9 for i in TARGET_ROW)

# ---- WHERE EACH PAD SITS ON THE ROBOT (the one definition of the geometry) ----
#
# Read off the board itself on 2026-09-23 -- `specifiction/sens num order.jpg`
# (the top view with the pads numbered) and `New Start/robot sensore/sensores.png`
# (the bare board) -- and confirmed by the operator.  NOTHING in the firmware
# says any of this: SENSORS.md's `+9` relation is firmware index arithmetic, not
# board geometry.
#
# The pads are NOT two straight rows.  They form a ring round the perimeter, and
# the index runs clockwise round that ring, starting at the robot's centre:
#
#      front edge of the robot
#        S1                                  S8      <- SIDE pads, set back
#          S2  S3  S4  S5  S6  S7                    <- the front row (six)
#                     S0    S9                       <- ON THE AXIS OF ROTATION
#        [=== left motor ===][=== right motor ===]    (the axis runs between them,
#                                                     at the robot's centre)
#        S17                                 S10      <- SIDE pads, set back
#          S16 S15 S14 S13 S12 S11                    <- the rear row (six)
#      rear edge of the robot
#
# Three things follow that the old two-row drawing had wrong:
#
#   * `s[0]` and `s[9]` sit side by side ON the rotation axis at the robot's
#     centre -- not at the outer ends of a row.  That is what makes
#     `s[0] && s[9]` mean "the axis is over a node", and it is the reading
#     SENSORS.md §2 recorded, which a wrong top-view reading contradicted.
#   * `s[1]` and `s[8]` are the SIDE pads: `s[1]` under `s[2]`, `s[8]` under
#     `s[7]`.  So the eight target pads `s[1..8]` make a U, not a line -- which
#     is how a 128 mm disc blacks all eight at once.
#   * the rear bank reads `S17 .. S10` left to right, the OPPOSITE way round to
#     the front (its index order also runs clockwise on the board; the rear half
#     of the ring starts at the right-hand side pad).
#
# A cell is (column, row) on a BOARD_COLS x BOARD_ROWS grid: column 0 is the
# robot's left edge, row 0 the front.  The live canvas scales the same numbers,
# which is why this is a grid and not a pair of floats.
#
# THE GRID IS PORTRAIT, because the ROBOT is: the board measures about 335 mm
# across and 570 mm long, so the drawn rectangle has to be taller than it is
# wide -- 29/17 = 1.71 against the board's 1.70, i.e. the same shape.  A grid
# that is wider than it is tall draws a robot that does not exist.  The step
# between adjacent columns (two cells, so two pads never touch) and between
# adjacent rows is what sets the ring's proportions; the pads are drawn a bit
# larger than a cell, which is the one schematic liberty taken, so that the pad
# name and its ADC both fit inside.
BOARD_COLS, BOARD_ROWS = 17, 29
PAD_CELL = {
    # ---- front bank: a U round the front half, plus the two pads on the axis.
    # The axis is the grid's centre line, column 8.5; every column here is
    # mirrored by one summing with it to 16, rows in the same way with 28.
    1: (2, 9),     # left SIDE pad -- outboard of s[2], set back behind it
    2: (3, 5), 3: (5, 5), 4: (7, 5), 5: (9, 5), 6: (11, 5), 7: (13, 5),
    8: (14, 9),    # right SIDE pad -- outboard of s[7], set back behind it
    0: (6, 14),    # ON THE AXIS OF ROTATION, just left of it, at mid-length
    9: (10, 14),   # ON THE AXIS OF ROTATION, just right of it
    # ---- rear bank: the same U, no axis pads, index order reversed
    17: (2, 19),   # left SIDE pad
    16: (3, 23), 15: (5, 23), 14: (7, 23), 13: (9, 23), 12: (11, 23),
    11: (13, 23),
    10: (14, 19),  # right SIDE pad
}

# Consequence worth knowing, and it is a finding about the ROBOT rather than
# about the drawing: because the rear bank's indices run the other way round,
# the `+9` mirror lands `s[11]` -- the firmware's rear LEFT branch detector --
# on the rear row's RIGHT end, under the front row's right side.  Either the rear
# bank is mounted mirrored or the firmware's rear left/right is swapped.  Park
# the robot on a corner it should turn left from and see whether `S11` or `S16`
# is the pad over the black lane.  See SENSORS.md §3.

# loop_start, named.  Zero is the pre-KEY1 boot loop -- the bench state -- and
# 1..8 are the mission.  The health build only streams in the stopped states
# (0, 7, 8), so a capture that shows anything else came from a driving build.
LOOP_STATES = {
    0: "BOOT / BENCH  (pre-KEY1, stopped)",
    1: "EXPLORE  (the brain is discovering)",
    2: "RETIRED  (the map dump used to live here)",
    3: "EXPLORE done  (waiting for KEY3)",
    4: "HOME PATH  (replaying the return plan)",
    5: "HOME done  (waiting for KEY3)",
    6: "FAST RUN  (replaying the fast plan)",
    7: "FAST RUN done  (waiting for KEY3)",
    8: "DONE  (stopped)",
}


def loop_state(n):
    """The name of a loop_start value.  Unknown values are passed through --
    a state this table has not heard of is worth noticing, not hiding."""
    return LOOP_STATES.get(n, "unknown state %s" % n)

RAIL = 4095          # 12-bit ADC full scale
BAND = 50            # the firmware's hysteresis half-width (main.c:1775-1779)
INIT_HI, INIT_LO = 100, 500   # the wider one-shot init band (main.c:1183-1184)
KEY_STUCK_MS = 8000  # a button held this long is stuck, not pressed
# A standing robot's gyro rate.  Every H line is a STOPPED state by
# construction (the firmware only streams health when it is not driving), so
# any rate above this is bias, not motion -- i.e. it has not been calibrated.
GYRO_STILL_DPS = 5.0


def sensor_short_role(i):
    """A column-sized role for the tables, derived from the pad groups above.

    FRONT_ROLE / REAR_ROLE hold SENSORS.md §2's wording and stay authoritative;
    this exists only so a fixed-width column does not cut "outer pad (only ever
    used paired with S9)" off in the middle of a word.  It is built from the
    same named sets the checks use, so the two cannot drift apart.
    """
    if i in GATE_PADS:
        return "node gate"
    if i in TARGET_PADS or i in (10, 17):
        return "target detector"
    if i in (LEFT_PAD, REAR_LEFT_PAD):
        return "LEFT branch det."
    if i in (RIGHT_PAD, REAR_RIGHT_PAD):
        return "RIGHT branch det."
    return "centre"


def decode_front(mask):
    """Front hex mask -> [(index, name, role)] for every bit that is set."""
    return [(i, FRONT_ROLE[i][0], FRONT_ROLE[i][1])
            for i in range(10) if mask >> i & 1]


def decode_rear(mask):
    """Rear hex mask -> [(index, name, role)]; bit i means s[10+i]."""
    return [(10 + i, REAR_ROLE[10 + i][0], REAR_ROLE[10 + i][1])
            for i in range(8) if mask >> i & 1]


def fmt_sensors(bits, width):
    """Render a mask as a fixed-width row so columns line up across lines."""
    on = {b[0] for b in bits}
    return "".join("X" if i in on else "." for i in range(width))


def parse_line(line, lineno=0):
    """Parse ONE wire line -> (kind, record).

      ("S"|"J"|"B"|"Z", dict)   a recognised line
      ("BAD", (lineno, line))   telemetry-SHAPED but broken -- the alarm
      ("BANNER", (lineno, line)) not telemetry at all -- benign, never dropped
      (None, None)              blank or a '#' comment

    BAD and BANNER are deliberately separate, and `bt_monitor.py` splits them
    the same way so both tools' "unparsed" counter means the same thing.  The
    firmware prints `Hi ,mmdi` as a boot banner on every run; calling that
    "unparsed" would train the operator to ignore the counter that exists to
    reveal a dropped junction.  Neither kind is ever silently dropped.

    THIS FUNCTION IS THE WIRE FORMAT.  `parse()` below and the live monitor
    (`bt_monitor.py --replay` / the GUI) both go through it, so a change to the
    field order cannot make one of them right and the other quietly wrong --
    which is the failure this project has already been bitten by once (a RAM
    check that parsed a row the build never emitted and so passed vacuously).
    """
    line = line.strip()
    if not line or line.startswith("#"):
        # A '#' line is a comment, not a damaged record.  The firmware never
        # emits one; the test fixtures and hand-annotated captures do, and
        # calling those "unparsed" would cry wolf on every run.
        return None, None
    f = line.split(",")
    try:
        if f[0] == "S" and len(f) == 9:
            return "S", {
                "ms": int(f[1]), "front": int(f[2], 16),
                "rear": int(f[3], 16), "e0": int(f[4]), "e1": int(f[5]),
                "L": int(f[6]), "R": int(f[7]), "cross": int(f[8]),
            }
        if f[0] == "J" and len(f) == 15:
            return "J", {
                "ms": int(f[1]), "ch": f[2], "nav": int(f[3]),
                "head": int(f[4]), "raw": int(f[5]), "cm": int(f[6]),
                "front": int(f[7], 16), "rear": int(f[8], 16),
                "L": int(f[9]), "R": int(f[10]), "cross": int(f[11]),
                # Fields 12-14 are the brain's.  `target` is the OnEndZoon
                # latch, `dist_cm` is the distance the brain was handed for
                # this link (already in cm, after the off-by-10-mm
                # simplification described in main.c), and `node` is the node
                # id the brain settled on.
                "target": int(f[12]), "dist_cm": int(f[13]),
                "node": int(f[14]),
            }
        if f[0] == "B" and len(f) == 7:
            # The bring-up line: printed BEFORE the 5 s supervised pause and
            # flushed ahead of it, so it is the record of what the brain
            # decided and where it thought it was when it decided.
            return "B", {
                "ms": int(f[1]), "node": int(f[2]), "x": int(f[3]),
                "y": int(f[4]), "drift": int(f[5]), "move": f[6],
            }
        if f[0] == "Z" and len(f) == 3:
            return "Z", {"ms": int(f[1]), "state": int(f[2])}
        # ---- the health build (USE_MAZE_HEALTH, idle only) ----
        # 25 fields: the tag, six scalars, and the eighteen channels -- the same
        # count as the firmware's `sprintf(p, "H,%lu,%X,%d,%d,%d,%d", ...)` plus
        # its 18-value loop.  (An earlier draft of this parser said 26 and would
        # have called every real H line BAD; make_health_fixtures.py refuses to
        # write a fixture that does not round-trip, which is what caught it.)
        if f[0] == "H" and len(f) == 25:
            return "H", {
                "ms": int(f[1]), "keys": int(f[2], 16), "loop": int(f[3]),
                "head": int(f[4]),
                # Stored as the wire's integer tenths; scaled here so every
                # consumer sees deg/s and deg and none of them re-does the /10.
                "gz": int(f[5]) / 10.0, "za": int(f[6]) / 10.0,
                "adc": [int(x) for x in f[7:25]],
            }
        if f[0] == "T" and len(f) == 21 and f[2] in ("mid", "min", "max"):
            return "T", {"ms": int(f[1]), "what": f[2],
                         "val": [int(x) for x in f[3:21]]}
    except (ValueError, IndexError):
        pass
    # Two different failures, and conflating them cries wolf.  A line that is
    # NOT telemetry-shaped is the firmware's boot banner (`Hi ,mmdi`, main.c)
    # or other human chatter -- benign, expected on every real capture.  A line
    # that IS shaped like telemetry (`J,...`) but does not parse is the alarming
    # kind: a dropped or truncated record.  `bt_monitor.py` splits them the same
    # way, so both tools' "unparsed" counter means the same thing.
    if f[0] in ("S", "J", "B", "Z", "H", "T"):
        return "BAD", (lineno, line)
    return "BANNER", (lineno, line)


def iter_parsed(lines, first_lineno=1):
    """Yield (lineno, kind, record) for every non-blank line of `lines`.

    A live consumer feeds this one line at a time; `parse()` feeds it a file.
    """
    for lineno, line in enumerate(lines, first_lineno):
        kind, rec = parse_line(line, lineno)
        if kind is None:
            continue
        yield lineno, kind, rec


def parse(path):
    """Capture file -> (lines_by_kind, bad, banner).

    `lines_by_kind` holds one list per line kind present -- S, J, B, Z from the
    mission build and H, T from the health build, in that key order (missing
    kinds are simply absent, so `by_kind.get("J", [])` is the way to ask).

    `bad` is the alarming set: telemetry-shaped lines that did not parse, i.e.
    evidence the wire lost or truncated something.  `banner` is benign chatter
    (the firmware's `Hi ,mmdi` boot banner).  Keep them apart -- see parse_line.
    """
    buckets = {"S": [], "J": [], "Z": [], "B": [], "H": [], "T": []}
    bad = []
    banner = []
    with open(path, "r", errors="replace") as fh:
        for lineno, kind, rec in iter_parsed(fh):
            if kind == "BAD":
                bad.append(rec)
            elif kind == "BANNER":
                banner.append(rec)
            else:
                buckets[kind].append(rec)
    return {k: v for k, v in buckets.items() if v}, bad, banner


# ==========================================================================
# THE HEALTH MODEL -- one definition of "healthy", shared by both tools
# ==========================================================================

class SensorStat:
    """Running per-channel statistics for one capture or one live session."""

    __slots__ = ("n", "lo", "hi", "last", "changes", "run_lo", "run_hi",
                 "max_run_lo", "max_run_hi", "in_band")

    def __init__(self):
        self.n = 0                  # samples seen
        self.lo = None              # smallest value seen
        self.hi = None              # largest value seen
        self.last = None
        self.changes = 0            # how many times the value moved at all
        self.run_lo = 0             # current run of samples pinned at 0
        self.run_hi = 0             # current run of samples at the 12-bit rail
        self.max_run_lo = 0
        self.max_run_hi = 0
        self.in_band = 0            # samples inside the firmware's dead band

    @property
    def span(self):
        return 0 if self.lo is None else self.hi - self.lo

    def feed(self, v, mid):
        self.n += 1
        self.lo = v if self.lo is None else min(self.lo, v)
        self.hi = v if self.hi is None else max(self.hi, v)
        if self.last is not None and v != self.last:
            self.changes += 1
        self.last = v

        self.run_lo = self.run_lo + 1 if v == 0 else 0
        self.run_hi = self.run_hi + 1 if v >= RAIL else 0
        self.max_run_lo = max(self.max_run_lo, self.run_lo)
        self.max_run_hi = max(self.max_run_hi, self.run_hi)

        # `mid` is 0 until a T line has arrived.  Nothing valid has a threshold
        # of 0, so that is a safe "not known yet" and not a magic sentinel.
        if mid and abs(v - mid) < BAND:
            self.in_band += 1


class HealthModel:
    """Live health state built from H and T lines.

    THE CHECKS LIVE HERE AND NOWHERE ELSE, for the same reason the BAD/BANNER
    split lives in `parse_line()`: the live view and the offline report must not
    be able to reach different conclusions about the same capture.

    WHAT THIS CAN AND CANNOT SHOW.  Every number comes off the wire, so a
    finding is a fact about the robot or the link.  What it cannot do is prove
    a channel is POINTING at the right thing -- a pad that tracks white and
    black perfectly is still just a pad that tracks white and black.  The roles
    are SENSORS.md's, and confirming them against the actual board is the
    operator's job, which is what the board view exists for.
    """

    def __init__(self):
        self.stats = [SensorStat() for _ in range(18)]
        self.h = 0                  # H lines seen
        self.t = 0                  # T lines seen
        self.thr = {}               # "mid"|"min"|"max" -> [18 ints]
        self.last = None            # the last H record
        self.keys = 0               # current key bitmask
        self.first_ms = None
        self.last_ms = None
        self.key_edges = []         # every transition, so bounce is visible
        self.key_down_ms = {}       # bit -> the ms it went down
        self.key_longest = {bit: 0 for bit, _, _ in KEY_BITS}
        self.gz_max = None          # largest |Gyro_Z| seen, deg/s

    # ------------------------------------------------------------- ingestion
    def feed(self, kind, rec):
        if kind == "T":
            self.t += 1
            self.thr[rec["what"]] = rec["val"]
            return
        if kind != "H":
            return

        self.h += 1
        self.last = rec
        self.last_ms = rec["ms"]
        if self.first_ms is None:
            self.first_ms = rec["ms"]
        gz = abs(rec["gz"])
        self.gz_max = gz if self.gz_max is None else max(self.gz_max, gz)

        mid = self.thr.get("mid")
        for i, v in enumerate(rec["adc"]):
            self.stats[i].feed(v, mid[i] if mid else 0)

        self._keys(rec["ms"], rec["keys"])

    def _keys(self, ms, keys):
        """Track edges, so a bouncing or stuck button shows up as a fact.

        A press is logged when it goes down AND when it comes up, with the hold
        time on the release.  Bounce therefore shows as a burst of edges too
        short to be a human press, which is exactly the shape of the defect a
        bench check is looking for.
        """
        changed = keys ^ self.keys
        for bit, name, _pin in KEY_BITS:
            if not (changed & bit):
                continue
            if keys & bit:
                self.key_down_ms[bit] = ms
                self.key_edges.append({"name": name, "ms": ms, "down": True,
                                       "held_ms": 0})
            else:
                t0 = self.key_down_ms.pop(bit, None)
                held = (ms - t0) if t0 is not None else 0
                self.key_longest[bit] = max(self.key_longest[bit], held)
                self.key_edges.append({"name": name, "ms": ms, "down": False,
                                       "held_ms": held})
        self.keys = keys

    def duration_ms(self):
        if self.first_ms is None or self.last_ms is None:
            return 0
        return self.last_ms - self.first_ms

    # ------------------------------------------------------------- derivation
    def logic(self, i):
        """Channel i: 1 = BLACK, 0 = white, None = inside the dead band.

        The firmware has two thresholds, mid-50 (to go black) and mid+50 (to go
        white), and BETWEEN them the bit is latched rather than decided
        (main.c:1775-1779).  So for a single sample in the band the only honest
        answer is "unknown", and saying 0 or 1 there would invent a reading the
        firmware never made.
        """
        if self.last is None or "mid" not in self.thr:
            return None
        v = self.last["adc"][i]
        m = self.thr["mid"][i]
        if v <= m - BAND:
            return 1
        if v >= m + BAND:
            return 0
        return None

    @staticmethod
    def _any(bits, idxs):
        vals = [bits[i] for i in idxs]
        if 1 in vals:
            return 1
        return None if None in vals else 0

    @staticmethod
    def _all(bits, idxs):
        vals = [bits[i] for i in idxs]
        if 0 in vals:
            return 0
        return None if None in vals else 1

    def derived(self):
        """What BrainIn the firmware would build from the pose on the bench.

        The SAME arithmetic as main.c's brain_report() and as derive_brainin()
        below, applied to the raw ADC instead of a latched mask:
          front = any of s[3..6]   left = s[2] && front   right = s[7] && front
        `at_node` is s[0] && s[9], the position gate on the rotation axis, and
        it is printed because a gate pad that never goes black is the single
        most useful thing a bench check can reveal.
        """
        if self.last is None:
            return None
        head = self.last["head"]
        centre = REAR_CENTRE_PADS if head else CENTRE_PADS
        lp, rp = ((REAR_LEFT_PAD, REAR_RIGHT_PAD) if head
                  else (LEFT_PAD, RIGHT_PAD))
        row = REAR_TARGET_ROW if head else TARGET_ROW

        bits = [self.logic(i) for i in range(18)]
        front = self._any(bits, centre)
        left = None if front is None else (bits[lp] and front)
        right = None if front is None else (bits[rp] and front)
        return {
            "head": head, "front": front, "left": left, "right": right,
            "at_node": self._all(bits, GATE_PADS),
            "target_row": self._all(bits, row),
            "logic": bits,
        }

    # --------------------------------------------------------------- findings
    def rows(self):
        """One dict per channel, for the report and the live table."""
        mid = self.thr.get("mid") or [0] * 18
        lo = self.thr.get("min") or [0] * 18
        hi = self.thr.get("max") or [0] * 18
        out = []
        for i, st in enumerate(self.stats):
            out.append({
                "i": i, "name": sensor_name(i),
                # `role` is SENSORS.md's wording; `short` is the same thing in
                # a column.  Both are here so no caller has to re-derive either.
                "role": sensor_role(i), "short": sensor_short_role(i),
                # `now` is the newest sample; lo/hi are the capture's extremes,
                # so a channel can have a wide range and still sit still now.
                "now": None if self.last is None else self.last["adc"][i],
                "lo": st.lo, "hi": st.hi, "span": st.span,
                "changes": st.changes, "n": st.n,
                "mid": mid[i], "cal_lo": lo[i], "cal_hi": hi[i],
                "cal_span": hi[i] - lo[i],
                "in_band": st.in_band, "logic": self.logic(i),
            })
        return out

    def findings(self):
        """-> [(severity, text)], FAIL first.  Severity: FAIL | WARN | INFO.

        POLARITY, because every threshold below depends on it and it is not
        guessable: the firmware sets s[i]=1 when IR_ADC[i] <= IR_mid[i]-50 and
        s[i]=0 when IR_ADC[i] >= IR_mid[i]+50 (main.c:1960-1967), and s=1 is
        BLACK (the lane, and the target).  So a LOW ADC is black and a HIGH ADC
        is white -- the pad reads high on the reflective field and low on the
        absorptive line.  It follows that in the calibration IR_max is the
        WHITE reading and IR_min the BLACK one, which is the opposite way round
        from how the names read.
        """
        if not self.h:
            return [("INFO", "no H lines in this capture -- not a health build "
                             "(needs USE_MAZE_HEALTH; see BUILD_GUIDE.md)")]
        fails, warns, infos = [], [], []
        n = self.h
        rows = self.rows()
        mid_known = "mid" in self.thr
        all_zero = all(r["hi"] == 0 for r in rows)

        # ---- the array as a whole: the loudest and most likely bench fault
        if all_zero:
            fails.append("every channel reads 0 for the whole capture -- the IR "
                         "emitters are unpowered or the mux is stuck, not 18 "
                         "dead sensors (IR_PWR is PA8, MUX is PA15/PB3/PB4)")
        if not mid_known:
            warns.append("no T line yet -- no thresholds on the wire, so nothing "
                         "can be compared against them")

        # ---- the calibration itself
        if mid_known:
            if not any(r["cal_hi"] for r in rows):
                warns.append("IR_min/IR_max are all zero: the thresholds are the "
                             "power-on defaults, NOT a calibration -- run one "
                             "(KEY2 during a run) before trusting any black/white")
            else:
                flat = [r["name"] for r in rows if r["cal_span"] < 200]
                if flat:
                    warns.append("calibration window under 200 counts on %s -- "
                                 "those pads never saw both white and black"
                                 % ", ".join(flat))

        # ---- could the one-shot init ever call BLACK on these pads?
        #
        # There are TWO band definitions and they are not the same.  The live
        # path uses +/-50 (main.c:1960-1967); the one-shot init that runs just
        # after KEY1 uses a wider, ASYMMETRIC band (main.c:1350-1351): black
        # needs adc <= mid-500, white needs adc >= mid+100.  A pad that never
        # gets 500 under its own mid in a capture was simply never held over
        # black, and the honest thing to report is that one fact -- NOT a WARN
        # per pad, because "was this pad over black just now" is not knowable
        # from the wire and the pads that are legitimately over white would
        # drown the real faults.  Reporting it at all is the point: it is the
        # difference between "the detection path passed" and "the detection
        # path was never exercised".
        if mid_known and not all_zero:
            usable = [r for r in rows if r["mid"] and r["lo"] is not None]
            no_black = [r for r in usable if r["lo"] > r["mid"] - INIT_LO]
            if no_black:
                who = ("all 18 pads" if len(no_black) == len(usable)
                       else ", ".join(r["name"] for r in no_black))
                infos.append("never held over black in this capture: %s.  The "
                             "one-shot init needs adc <= mid-%d to set a pad's "
                             "s[] bit to black (main.c:1350-1351), so black "
                             "detection is only exercised on the pads that did "
                             "dip -- park the robot on the line to test the rest."
                             % (who, INIT_LO))

        # ---- per channel
        # Whether ANY channel is moving decides how to read a channel that is
        # not.  On a perfectly still robot every channel holds still and that
        # is not a fault; on a moving one, a channel that alone never budges is
        # the interesting one.  Same observation, two very different weights.
        moving = any(r["changes"] for r in rows)
        for r in rows:
            if all_zero:
                break                          # already FAILed as an array
            if r["changes"] == 0 and r["hi"] == 0:
                # Stuck at 0 means stuck on BLACK.  That is the dangerous
                # direction: the brain sees a lane, or a whole target row, that
                # is not there.  A shorted pad and a pad parked over the target
                # disc look identical, which is why the severity depends on
                # whether anything else moved.
                (fails if moving else infos).append(
                    "%s (%s) sat at 0 for the whole capture%s -- a pad stuck low "
                    "reads BLACK, i.e. a lane the brain can see and the robot "
                    "cannot drive down"
                    % (r["name"], r["short"],
                       "" if moving else " (if the robot is parked on the target "
                       "pad, an all-black row is correct -- check the others)"))
            elif r["changes"] == 0 and r["hi"] >= RAIL:
                fails.append("%s (%s) is pinned at the %d rail the whole capture "
                             "-- saturated, or a dead phototransistor: it reads "
                             "WHITE whatever it is over"
                             % (r["name"], r["short"], RAIL))
            elif r["changes"] == 0:
                (warns if moving else infos).append(
                    "%s (%s) never changed (%d samples at %d)%s"
                    % (r["name"], r["short"], r["n"], r["hi"],
                       "" if moving else " -- the robot may simply be still"))
            if mid_known and r["in_band"] * 2 > n:
                infos.append("%s (%s) sat inside the +/-%d dead band for %d%% of "
                             "the capture -- its s[] bit is LATCHED there, not "
                             "freshly decided" % (r["name"], r["short"], BAND,
                                                  100 * r["in_band"] // n))

        # ---- the pads whose role makes them worth calling out
        d = self.derived()
        if d and d["at_node"] == 0:
            infos.append("the s[0]&&s[9] node gate reads OPEN right now -- move "
                         "the robot so its rotation axis is over a node and this "
                         "should read CLOSED (both pads on black)")

        # ---- the gyro: the one sensor whose fault shows as a NUMBER, not a bit
        #
        # This is a real WARN and not the cry-wolf kind: every H line is a
        # stopped state by construction, so a rate here is bias rather than
        # motion, and the fix is a thing the operator can do and then watch the
        # warning disappear.  A freshly powered robot is EXPECTED to trip this
        # until KEY3 (boot loop) or the run's own calibration zeroes it, which
        # makes it the health stream's most immediately actionable line.
        if self.gz_max is not None and self.gz_max > GYRO_STILL_DPS:
            warns.append("the gyro reads up to %.1f deg/s while the robot is "
                         "standing still (a resting robot should sit near 0), so "
                         "it is carrying a bias, not turning: calibrate it -- "
                         "KEY3 in the boot loop, or the run's own calibration "
                         "after KEY1" % self.gz_max)

        # ---- the buttons
        for bit, name, pin in KEY_BITS:
            longest = self.key_longest[bit]
            if longest >= KEY_STUCK_MS:
                fails.append("%s (%s) was held for %.1f s -- longer than any "
                             "human press; suspect a shorted or stuck button"
                             % (name, pin, longest / 1000.0))
            if not any(e["name"] == name for e in self.key_edges):
                infos.append("%s (%s) never changed in this capture"
                             % (name, pin))
        for e in self.key_edges:
            if e["down"] and 0 < e["held_ms"] < 20:
                infos.append("%s released after only %d ms at t=%d ms -- "
                             "possible contact bounce"
                             % (e["name"], e["held_ms"], e["ms"]))

        # The three lists hold bare text -- the severity IS which list a finding
        # landed in -- so the tag goes on here, once, rather than on every
        # append.  FAILs come out first because that is the order an operator
        # reads them in.
        return ([("FAIL", t) for t in fails]
                + [("WARN", t) for t in warns]
                + [("INFO", t) for t in infos])

    def worst(self):
        """The severities present, so a caller can decide an exit code."""
        sev = [s for s, _ in self.findings()]
        return "FAIL" if "FAIL" in sev else "WARN" if "WARN" in sev else "OK"


def estimate_cell(raws, lo=40.0, hi=260.0, step=0.25, top=5):
    """Find the counts-per-cell quantum that best explains the raw counts.

    Node-to-node travel is a whole number of 20 cm cells, so every link's raw
    count should sit near k * counts_per_cell for integer k.  We score each
    candidate by how far the raws sit from the nearest multiple, and return the
    best few.  This is the honest way round: it does not assume the firmware's
    2.467 counts/mm, it measures whatever the hardware actually does.
    """
    if not raws:
        return []
    scored = []
    k = lo
    while k <= hi:
        err = 0.0
        for r in raws:
            m = round(r / k)
            if m < 1:
                m = 1
            err += abs(r - m * k)
        scored.append((err / (len(raws) * k), k))   # normalise: relative error
        k += step
    scored.sort()
    return scored[:top]


# ==========================================================================
# THE HEALTH SECTION -- the offline half of the health build
# ==========================================================================

def build_health_model(by_kind):
    """Feed a fresh HealthModel every H/T line of a parsed capture.

    T lines go in FIRST, and that is a deliberate difference from the live app.
    Offline we want every H sample judged against the thresholds the build
    actually ended up with; feeding them in wire order would leave the first
    second of a capture with no thresholds at all and report those samples as
    "unknown" -- true to the wire, useless as a bench measurement.  The report
    says so on the page instead of hiding it.
    """
    model = HealthModel()
    for rec in by_kind.get("T", []):
        model.feed("T", rec)
    for rec in by_kind.get("H", []):
        model.feed("H", rec)
    return model


def _wrap(text, indent, width=70):
    """Wrap one finding under a hanging indent.

    The findings are written as whole sentences with the reasoning included --
    "S7 is pinned at the rail, so it is dead or unlit and saturated" is the
    useful form, and truncating it to fit a column would delete the part that
    tells the operator what to do next.
    """
    lines = textwrap.wrap(text, width=width) or [""]
    pad = " " * indent
    return [pad + lines[0]] + [pad + l for l in lines[1:]]


def _health_bar(model):
    """The pads in the positions they really occupy on the board, as text.

    This is a picture of the BOARD, not of a row of sensors: the geometry is
    PAD_CELL, the single definition (read it for the shape, and for why the
    shape matters).  The numbers behind each mark are in the SENSORS table
    below; this exists so the SHAPE is visible, because every role in this file
    depends on it -- `s[0] && s[9]` only means "the axis is over a node" if s[0]
    and s[9] really are on the axis.

    `#` = black, `.` = white, `?` = inside the firmware's dead band, where s[]
    is latched rather than freshly decided.

    This is what makes the open node-gate question answerable from a capture:
    park the robot on a corner by hand and the gate and the centre group can be
    read straight off this, with no drive and no J line needed.
    """
    d = model.derived()
    if d is None:
        return []

    def glyph(i):
        lg = d["logic"][i]
        return "?" if lg is None else ("#" if lg else ".")

    def grid(getter):
        """One line per board row, each pad dropped into its own column."""
        lines = []
        for r in range(BOARD_ROWS):
            cells = ["   "] * BOARD_COLS
            for i in range(18):
                col, row = PAD_CELL[i]
                if row == r:
                    cells[col] = "%-3s" % getter(i)
            lines.append("   |" + "".join(cells) + "|")
        return lines

    names, marks = grid(sensor_name), grid(glyph)
    out = ["   TOP VIEW, drawn long and narrow like the board itself.  FRONT IS",
           "   AT THE TOP; the axis of rotation runs down the middle of the board.",
           "   `#` black   `.` white   `?` inside the firmware's dead band."]
    blank = 0
    for r in range(BOARD_ROWS):
        occupied = [i for i in range(18) if PAD_CELL[i][1] == r]
        if not occupied:
            # The gaps between the groups are part of the picture -- the axis
            # pads really are a long way back from the front row -- so ONE blank
            # line is kept per gap.  One, not all of them: the grid is portrait
            # and mostly margin, and 24 blank lines of margin would bury the
            # shape they are meant to show.
            blank += 1
            if blank == 1:
                out.append(names[r])
            continue
        blank = 0
        out.append(names[r])
        out.append(marks[r])

    out.append("")
    out.append("   The eight TARGET pads are a U, not a line: the front bank")
    out.append("   minus its two axis pads (s[1..8]), or the whole rear bank")
    out.append("   (s[10..17], which has no axis pads) -- that is why a %d mm"
               % DISC_WIDTH_MM)
    out.append("   disc blacks all eight at once.")
    out.append("   s[0] and s[9] are the node gate and sit ON the axis, so both")
    out.append("   black means the axis is over a node.")
    out.append("   Which bank LEADS on the bench is head=%d: %s."
               % (d["head"], "rear" if d["head"] else "front"))
    return out


def health_report(by_kind, bad):
    """Print the bench health section.  -> exit code: 1 if anything FAILed.

    Driven by the same HealthModel that `bt_monitor.py --health` and the live
    GUI use, so this section and the app cannot reach different conclusions
    about one capture -- the discipline the BAD/BANNER split already follows.
    """
    model = build_health_model(by_kind)
    bar = "-" * 74
    print()
    print(bar)
    print(" H. BENCH HEALTH CHECK   (USE_MAZE_HEALTH build, robot standing still)")
    print(bar)

    dur = model.duration_ms()
    print("  %d H samples, %d T lines, %.1f s of wire time"
          % (model.h, model.t, dur / 1000.0))
    if dur > 0 and model.h > 1:
        print("  effective rate: %.1f Hz  (the firmware paces it at 20 Hz)"
              % (1000.0 * (model.h - 1) / dur))
    print("  thresholds on the wire: %s"
          % (", ".join(sorted(model.thr)) if model.thr
             else "NONE -- no T line in this capture"))
    if bad:
        print("  %d telemetry-shaped line(s) did not parse -- the wire dropped"
              % len(bad))
        print("  something, so read every number below as possibly stale.")

    print("""
  WHAT THIS IS.  H/T lines exist only in a USE_MAZE_HEALTH build, and the
  firmware only sends them while the robot is STANDING STILL -- before KEY1
  and after a run.  So this is a bench reading, not a mission record: there
  is no map, no node and no decision in it.  Thresholds from T were applied
  to EVERY H sample here (see build_health_model); the live app cannot do
  that, so its first second may read "unknown" where this does not.""")

    # ------------------------------------------------------------ state, gyro
    if model.last is not None:
        print("\n  STATE     loop_start=%d   head=%d   keys=%X"
              % (model.last["loop"], model.last["head"], model.last["keys"]))
        print("            = %s" % loop_state(model.last["loop"]))
        print("            loop_start 0 is the boot loop (before KEY1); 7 and up")
        print("            are after a run.  Those two are the only places the")
        print("            firmware streams health, so both are stopped states.")
        print("  GYRO      Gyro_Z %.1f deg/s, Z_Angle %.1f deg"
              % (model.last["gz"], model.last["za"]))
        print("            The angle is relative to wherever the gyro was last")
        print("            zeroed, and the run re-zeros it -- so its absolute")
        print("            value means little here.  The RATE is the evidence:")
        print("            standing still, it should read near 0 deg/s.")

    # ------------------------------------------------------------------ keys
    print("\n  KEYS")
    if model.last is None:
        print("    (no H line in this capture)")
    else:
        print("    " + "   ".join("%s %-4s %s"
                                  % (name, pin,
                                     "DOWN" if model.last["keys"] & bit else "up")
                                  for bit, name, pin in KEY_BITS))
        print("    longest press: %s"
              % ", ".join("%s %.2f s" % (name, model.key_longest[bit] / 1000.0)
                          for bit, name, _pin in KEY_BITS))
        if model.key_edges:
            for e in model.key_edges[-8:]:
                print("    t=%-8d %-5s %s%s"
                      % (e["ms"], e["name"], "down" if e["down"] else "up  ",
                         "" if e["down"] else "   held %d ms" % e["held_ms"]))
            if len(model.key_edges) > 8:
                print("    ... %d transitions in all (see the edge log above)"
                      % len(model.key_edges))
        else:
            print("    no key transition in this capture -- press each one and")
            print("    it should appear here with a hold time")

    # --------------------------------------------------------------- sensors
    if model.last is not None:
        print("\n  PADS ON THE BOARD   (see PAD_CELL for the geometry)")
        print("  POLARITY (read off main.c:1960-1967, not assumed): s[i]=1 when")
        print("  adc <= mid-50 and s[i]=0 when adc >= mid+50, and s=1 is BLACK.")
        print("  So LOW adc is black, HIGH adc is white -- which means in the")
        print("  calibration IR_max is the WHITE reading and IR_min the BLACK one.")
        for line in _health_bar(model):
            print(line)

    print("\n  SENSORS   (now vs mid; `lo`/`hi` are this capture's extremes)")
    print("    %-5s %-17s %6s %6s %6s %6s %6s  %s"
          % ("pad", "role", "now", "lo", "hi", "mid", "span", "now reads"))
    for r in model.rows():
        lg = r["logic"]
        mark = "?" if lg is None else ("BLACK" if lg else "white")
        print("    %-5s %-17s %6s %6s %6s %6s %6s  %s"
              % (r["name"], r["short"],
                 "-" if r["now"] is None else r["now"],
                 "-" if r["lo"] is None else r["lo"],
                 "-" if r["hi"] is None else r["hi"],
                 r["mid"] or "-", r["span"], mark))

    if model.thr:
        print("\n  THRESHOLDS   IR_mid / IR_min / IR_max as the build holds them")
        print("    min is the BLACK end and max is the WHITE end (see the")
        print("    polarity note above); span is the contrast the calibration saw.")
        print("    %-5s %6s %6s %6s %6s  %s"
              % ("pad", "mid", "min=BLK", "max=WHT", "span", "note"))
        for r in model.rows():
            note = ""
            if not r["cal_hi"]:
                note = "no calibration -- power-on default"
            elif r["cal_span"] < 200:
                note = "narrow window: never saw both white and black"
            print("    %-5s %6s %6s %6s %6s  %s"
                  % (r["name"], r["mid"] or "-", r["cal_lo"], r["cal_hi"],
                     r["cal_span"], note))

    # --------------------------------------------------------------- derived
    d = model.derived()
    if d is not None:
        def tri(v):
            return "?" if v is None else ("1 (black)" if v else "0 (white)")

        print("\n  DERIVED   what BrainIn the firmware would build at this pose")
        print("    front   = any of s[3..6]      -> %s" % tri(d["front"]))
        print("    left    = s[2] && front       -> %s" % tri(d["left"]))
        print("    right   = s[7] && front       -> %s" % tri(d["right"]))
        print("    at_node = s[0] && s[9]        -> %s" % tri(d["at_node"]))
        print("    target  = all of s[1..8]      -> %s" % tri(d["target_row"]))
        print("    '?' means inside the +/-%d dead band, where s[] is LATCHED and"
              % BAND)
        print("    not freshly decided.  These are derived from IR_ADC[] vs")
        print("    IR_mid[] -- NOT the firmware's own latched s[].  The S/J lines")
        print("    are the authority on what it actually believed while driving.")

    # -------------------------------------------------------------- findings
    findings = model.findings()
    counts = {sev: sum(1 for s, _ in findings if s == sev)
              for sev in ("FAIL", "WARN", "INFO")}
    print("\n  CHECKS")
    if not findings:
        print("    none -- nothing on this wire looks unhealthy")
    for sev, text in findings:
        # Hanging indent at the column the text starts in (4 + "[FAIL] " = 11),
        # so a wrapped finding stays visually under its own tag.
        lines = _wrap(text, 11, 70)
        print("    [%-4s] %s" % (sev, lines[0].lstrip()))
        for line in lines[1:]:
            print(line)

    worst = model.worst()
    print("\n  HEALTH VERDICT: %s   (%d FAIL, %d WARN, %d INFO)"
          % (worst, counts["FAIL"], counts["WARN"], counts["INFO"]))
    print("  What this does NOT show: that any pad points where SENSORS.md says.")
    print("  A channel can track white and black perfectly and still be the")
    print("  wrong channel -- park the robot on a known node and read the board")
    print("  above against the map.  That is the check, and only a human can")
    print("  do it.")
    return 1 if worst == "FAIL" else 0


def report(args):
    by_kind, bad, banner = parse(args.path)
    s_lines = by_kind.get("S", [])
    j_lines = by_kind.get("J", [])
    z_lines = by_kind.get("Z", [])
    b_lines = by_kind.get("B", [])
    h_lines = by_kind.get("H", [])
    thr_lines = by_kind.get("T", [])
    print("=" * 74)
    print(" M1 telemetry report -- %s" % args.path)
    print("=" * 74)
    print("  S samples : %d" % len(s_lines))
    print("  J events  : %d" % len(j_lines))
    print("  Z events  : %d" % len(z_lines))
    print("  B events  : %d  (bring-up decisions, supervised build only)"
          % len(b_lines))
    if h_lines or thr_lines:
        # The health build.  Worth saying out loud which build this is, because
        # an H line only ever exists in the idle states -- so a capture with
        # both H and J lines is two DIFFERENT sessions on one link, not one run.
        print("  H samples : %d  (health build, robot standing still)"
              % len(h_lines))
        print("  T lines   : %d  (thresholds: %s)"
              % (len(thr_lines),
                 ", ".join(sorted({t["what"] for t in thr_lines})) or "-"))
    if banner:
        # Benign: the firmware's `Hi ,mmdi` boot banner and human chatter.
        # Counted, not alarming -- a non-zero BANNER is normal on every capture.
        print("  banner    : %d line(s), ignored (first: %r)"
              % (len(banner), banner[0][1][:60]))
    if bad:
        print("  UNPARSED  : %d TELEMETRY-SHAPED line(s) did not parse"
              % len(bad))
        print("              (first: %r)" % (bad[0][1][:60],))
        print("              -> a dropped/truncated record, or a build whose")
        print("                 field count differs.  Treat the run as suspect.")
    if not s_lines and not j_lines and not h_lines:
        print("\n  Nothing recognisable.  Is this a USE_TELEMETRY or")
        print("  USE_HEALTH capture?")
        return 1

    # -------------------------------------------------------------- health
    #
    # An H line only exists while the robot is standing still, so a capture
    # holding both H and J lines is a bench session followed by a run on one
    # link, not one thing.  The health section therefore prints first and says
    # which it is, and the mission sections below then cover the run if there
    # was one -- the two halves are never mixed into one table.
    health_rc = 0
    if h_lines:
        health_rc = health_report(by_kind, bad)
    if not (s_lines or j_lines):
        return health_rc

    # ---------------------------------------------------------------- links
    print("\n" + "-" * 74)
    print(" 1. ENCODER CALIBRATION")
    print("-" * 74)
    # 'S' is the legacy straight, 'F' the brain's; both are real links.
    moves = [j for j in j_lines if j["ch"] in "SLRFB"]
    if not moves:
        print("  no move events -- did the robot drive?")
    else:
        print("  per-link raw counts vs the firmware's own conversion.")
        print("  (raw/cm is only meaningful if the link really was `cm` long --")
        print("   that is the thing under test, which is why the next block")
        print("   estimates the cell size from `raw` alone.)")
        print("  %-7s %-4s %-5s %7s %6s %9s" %
              ("t(ms)", "ch", "nav", "raw", "cm", "raw/cm"))
        for j in moves:
            ratio = (j["raw"] / j["cm"]) if j["cm"] else float("nan")
            print("  %-7d %-4s %-5d %7d %6d %9.3f" %
                  (j["ms"], j["ch"], j["nav"], j["raw"], j["cm"], ratio))

        raws = [j["raw"] for j in moves]
        print("\n  raw counts seen: min=%d max=%d" % (min(raws), max(raws)))
        print("  firmware assumes 2.467 counts/mm/wheel -> 4.934 counts/cm on the")
        print("  summed encoder -> %.2f counts per 20 cm cell." % (4.934 * 20))

        cands = estimate_cell(raws, top=5)
        if cands:
            print("\n  measured candidates for one 20 cm cell (relative fit error):")
            for err, k in cands:
                print("     %7.2f counts/cell  = %6.3f counts/cm  (err %.3f)"
                      % (k, k / 20.0, err))
            best = cands[0][1]
            print("\n  best fit: %.2f counts/cell" % best)
            print("  multiples implied by each link:")
            for j in moves:
                n = j["raw"] / best
                print("     t=%-6d %s  raw=%-6d -> %.2f cells  (firmware said %d cm"
                      " = %.2f cells)" % (j["ms"], j["ch"], j["raw"], n,
                                          j["cm"], j["cm"] / 20.0))
            print("\n  READ THIS AS: every link whose cell count is not close to a")
            print("  whole number is one where the firmware's +25/+30/+85/+95/+104")
            print("  offsets are absorbing the error instead of describing it.")

    # ------------------------------------------------------------ junctions
    print("\n" + "-" * 74)
    print(" 2. WHICH SENSORS FIRE AT A JUNCTION")
    print("-" * 74)
    if not j_lines:
        print("  no junction events recorded")
    else:
        print("  front row: s[0..9] -> .S0 S1 S2 S3 S4 S5 S6 S7 S8 S9")
        print("  rear  row: s[10..17] -> bit i = s[10+i]")
        for j in j_lines:
            fr, rr = decode_front(j["front"]), decode_rear(j["rear"])
            print("\n  t=%d ms  move=%s  nav=%d head=%d  L=%d R=%d cross=%d"
                  % (j["ms"], j["ch"], j["nav"], j["head"], j["L"], j["R"],
                     j["cross"]))
            print("    brain: dist=%d cm  node=%d  target=%d"
                  % (j["dist_cm"], j["node"], j["target"]))
            # FRONT is the one input the firmware never computed as such, and
            # the only one still unverified against hardware: it is the centre
            # group, which at a CORNER may still be sitting on the robot's own
            # incoming arm and so read "forward open" where there is no forward.
            # A corner is exactly a junction with one lateral and no front, so
            # print the derived flag beside the raw mask and let the capture say.
            centre = (j["front"] >> 3) & 0xF          # s[3..6]
            if j["head"] == 0:
                front_open = 1 if centre else 0
                laterals = j["L"] + j["R"]
                print("    front(s[3..6]=%X) -> brain front=%d   laterals=%d"
                      % (centre, front_open, laterals))
                if laterals == 1 and front_open:
                    print("    ?? CORNER CANDIDATE: one lateral and front=1.  If")
                    print("       the robot is turning a corner here rather than")
                    print("       driving on, in.front is lying -- see brain.h.")
            print("    front %s   %s" % (fmt_sensors(fr, 10),
                                         " ".join(b[1] for b in fr) or "-"))
            print("    rear  %s   %s" % (fmt_sensors(rr, 8),
                                         " ".join(b[1] for b in rr) or "-"))
            # Only the leading bank's branch detectors are meaningful: the
            # firmware reads the front row when head==0, the rear row when
            # head==1 (SENSORS.md §2).  Check what fired against what the move
            # implies -- a mismatch here is the phantom-edge bug in the making.
            if j["head"] == 0:
                bank, ldet, rdet = "front", "S2", "S7"
            else:
                bank, ldet, rdet = "rear", "S11", "S16"
            fired = [b[1] for b in (fr if j["head"] == 0 else rr)]
            branches = [n for n in fired if n in (ldet, rdet)]

            # A target detector firing away from the target is the false-positive
            # path: main.c needs the whole inner row at once, so a lone S1/S8 is
            # stray black or a drifting IR_mid[]. Worth a line every time.
            stray = [n for n in fired if n in ("S1", "S8", "S10", "S17")]
            if stray and j["ch"] != "D":
                print("    .. note: %s live on a non-target crossing -- stray"
                      % ",".join(stray))
                print("             black, or IR_mid[] drifting on those sensors")

            if j["ch"] == "L" and ldet not in fired:
                print("    >> CHECK  move=L but %s (the %s L detector) did not fire"
                      % (ldet, bank))
            elif j["ch"] == "R" and rdet not in fired:
                print("    >> CHECK  move=R but %s (the %s R detector) did not fire"
                      % (rdet, bank))
            elif j["ch"] in ("S", "F") and branches:
                # 'S' is the legacy name for this move, 'F' the brain's; both
                # mean "carry straight on", and neither should have a lateral
                # branch detector live beside it.
                print("    >> CHECK  move=%s but a branch detector fired: %s"
                      % (j["ch"], ",".join(branches)))
            elif j["ch"] == "B":
                # A reversal is chosen because nothing else is left, so no
                # detector is expected to be live and none of the checks above
                # apply.  What matters is that the brain asked for it at all.
                print("    .. move=B (reverse) -- no branch expectation")
            elif j["ch"] == "D":
                # the target is a solid disc: the whole inner row goes black at
                # once (main.c's end-zone test), so both "branch" sensors firing
                # is expected here, not a phantom edge
                want = ("S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8")
                if j["head"] == 0 and not all(w in fired for w in want):
                    miss = [w for w in want if w not in fired]
                    print("    >> CHECK  target reached but %s did not fire -- the"
                          % ",".join(miss))
                    print("              disc may be narrower than the 71.4 mm row")
                else:
                    print("    .. full inner row live -- target detection armed")
            elif branches:
                print("    .. %s bank: %s live -- consistent with move=%s"
                      % (bank, ",".join(branches), j["ch"]))

    # -------------------------------------------------------- branch window
    print("\n" + "-" * 74)
    print(" 3. BRANCH WINDOW")
    print("-" * 74)
    if not s_lines:
        print("  no stream samples -- only junction events were captured")
    else:
        # A window is a contiguous run of samples where a branch flag is live.
        runs, cur = [], None
        for s in s_lines:
            live = s["L"] or s["R"]
            if live and cur is None:
                cur = {"start": s["ms"], "n": 0, "flags": set(), "masks": []}
            if cur is not None:
                if live:
                    cur["n"] += 1
                    if s["L"]:
                        cur["flags"].add("L")
                    if s["R"]:
                        cur["flags"].add("R")
                    cur["masks"].append((s["front"], s["rear"]))
                else:
                    cur["end"] = s["ms"]
                    runs.append(cur)
                    cur = None
        if cur is not None:
            cur["end"] = s_lines[-1]["ms"]
            runs.append(cur)

        if not runs:
            print("  left_poss/right_poss were never asserted in the stream.")
            print("  That is itself a finding: either the run never met a branch,")
            print("  or the junctions are being caught by the J events alone.")
        else:
            print("  %d branch windows seen.  Sampling resolution is 8 ms.\n"
                  % len(runs))
            for r in runs:
                dur = r["end"] - r["start"]
                print("    t=%-6d  %d sample(s)  span=%-4d ms  flags=%s"
                      % (r["start"], r["n"], dur,
                         "+".join(sorted(r["flags"]))))
                for fm, rm in r["masks"]:
                    print("        front %s  rear %s"
                          % (fmt_sensors(decode_front(fm), 10),
                             fmt_sensors(decode_rear(rm), 8)))
            spans = [r["end"] - r["start"] for r in runs]
            print("\n  window span: min=%d ms  max=%d ms  (samples: min=%d max=%d)"
                  % (min(spans), max(spans),
                     min(r["n"] for r in runs), max(r["n"] for r in runs)))
            print("  Reminder: an 8 ms sample period on a ~20 ms window gives you")
            print("  2-3 samples.  Treat the spans as upper bounds, not measurements.")

    # ------------------------------------------------------------- target
    if z_lines:
        print("\n" + "-" * 74)
        print(" 4. TARGET ZONE")
        print("-" * 74)
        for z in z_lines:
            print("  t=%d ms  zone %s" % (z["ms"],
                                          "ENTERED" if z["state"] else "left"))
        dwell = DISC_WIDTH_MM / 10.0 / V_MAX_CM_S * 1000.0
        print("  The disc is %.0f mm across, so at v_max = %.0f cm/s the pads are"
              % (DISC_WIDTH_MM, V_MAX_CM_S))
        print("  over it for about %.0f ms.  `end_zone_timer > 2` needs the"
              % dwell)
        print("  all-black row held across 3 loop iterations to latch, so an")
        print("  ENTER->left gap far short of %.0f ms means the target was" % dwell)
        print("  crossed too fast to be detected.")

    # ------------------------------------------------------- brain at work
    if j_lines:
        print("\n" + "-" * 74)
        print(" 5. THE BRAIN'S VIEW")
        print("-" * 74)
        print("  Drift is |dist_cm - whole cells|: the brain snaps every link to")
        print("  the lattice, so this is the residue it threw away.  It should")
        print("  stay small and NOT grow along the run; a value that climbs is")
        print("  the counts-per-cell constant being wrong, which the stop-graph")
        print("  planner would then be reasoning over a distorted map with.")
        print()
        print("  %-8s %-4s %8s %8s %8s" %
              ("t(ms)", "move", "dist_cm", "cells", "drift"))
        total_drift = 0
        for j in j_lines:
            cells = int((j["dist_cm"] + 10) / 20)
            drift = abs(j["dist_cm"] - cells * 20)
            total_drift += drift
            print("  %-8d %-4s %8d %8d %8d"
                  % (j["ms"], j["ch"], j["dist_cm"], cells, drift))
        print("\n  total drift absorbed: %d cm over %d reports"
              % (total_drift, len(j_lines)))
        if total_drift > 0.5 * len(j_lines) * 20:
            print("  >> CHECK  mean drift is over 10 cm per link -- that is half a")
            print("            cell.  Runs of links rounding the WRONG way would")
            print("            put nodes one cell off; suspect the encoder")
            print("            constant before suspecting the brain.")

    if b_lines:
        print("\n" + "-" * 74)
        print(" 6. BRING-UP DECISIONS (supervised build)")
        print("-" * 74)
        print("  One line per junction, printed BEFORE the 5 s pause: what the")
        print("  brain decided and where it thought it was standing when it")
        print("  decided.  Read them against the robot's actual position.")
        print()
        print("  %-8s %-6s %-12s %-8s %-6s" %
              ("t(ms)", "node", "position", "drift", "move"))
        for b in b_lines:
            print("  %-8d %-6d (%4d,%4d) %-8d %-6s"
                  % (b["ms"], b["node"], b["x"], b["y"], b["drift"], b["move"]))
        revs = [b for b in b_lines if b["move"] == "B"]
        if revs:
            print("\n  %d reversal(s) asked for.  Before this change the replay"
                  % len(revs))
            print("  stages had no 'B' case at all and the robot would have")
            print("  stalled at the junction that produced one.")

    print("\n" + "=" * 74)
    # A health FAIL is a fault whether or not the same link also carried a run,
    # so it propagates here too -- a mixed capture whose sensors are broken must
    # not exit 0 just because its mission half parsed.  A pure mission capture
    # leaves health_rc at 0, so its behaviour is unchanged.
    if health_rc:
        print("  NOTE: the health section above FAILED, so this report exits 1")
        print("        even though the mission sections parsed.")
    return health_rc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", help="captured telemetry text file")
    sys.exit(report(ap.parse_args()))


if __name__ == "__main__":
    main()
