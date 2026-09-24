#!/usr/bin/env python3
"""bt_monitor.py -- live Bluetooth capture, plot and decision check for SEYED.

WHAT THIS IS FOR
The brain has never run on the robot.  Every check in this repo drives it from a
*model* of the robot (`brain_host.c`), so a wrong model is invisible to them.
This is the instrument for the real thing: it opens the Bluetooth link, plots
the mission as it happens, and -- the point of the exercise -- feeds the input
each junction really produced to a virtual brain and compares the decisions.

THE VIRTUAL BRAIN IS THE REAL BRAIN
Not a re-implementation.  `build/brain_oracle.exe` links `src/brain.c` itself
and answers one move per `BrainIn`, so a disagreement is a finding about the
robot or the wire, never about the checker.  See `test/brain_oracle.c`.

WHAT IT CANNOT DO
Prove the brain's model of the robot is right.  Agreement means the wire carried
the run faithfully; it does not mean the run was correct.  The one place those
come apart is `in.front`, which is what the front audit is for.

READ THIS BEFORE TRUSTING in.front
`in.front` is `s[3]||s[4]||s[5]||s[6]` (firmware main.c:932) and the lateral
flags are `s[2] && centre` / `s[7] && centre` (main.c:1338-1339).  They share the
centre term, so at any stop reached by the lateral branch `front` is 1 by
construction: it is not an independent "forward is open" measurement.  A corner
(one lateral, no forward) and a T-junction (one lateral, forward exists) present
the brain with the SAME input, and P1's straight-first preference answers 'F' to
both.  `brain_host.c` derives front from the maze's true topology instead, so it
produces the one state the hardware cannot (F=0,L=1) and never produces the state
the hardware does at a corner -- which is why 7/7 on the host does not transfer.

WHAT THE FRONT AUDIT DOES AND DOES NOT SHOW (on by default)
It does NOT prove any decision wrong, and must not be read as if it did.  The
only evidence on the wire that is independent of the brain's own dead reckoning
is the sensor mask itself, so the audit reports two things and claims nothing
more:

  (a) how often `front == (left or right)` held on this capture;
  (b) the shortlist of junctions where the brain drove STRAIGHT on a
      one-lateral input.  Those are the places where a corner and a T-junction
      present the same input, so they are the ones to check by hand against the
      real maze.

  **CAVEAT THAT MATTERS: on a SYNTHETIC capture (a fixture) number (a) is true by
  construction and proves nothing.**  The fixtures' masks were built from the
  firmware's own stop test -- `centre live AND >=1 lateral` -- so replaying one
  and observing that the centre group is live at every junction restates the C
  source rather than measuring anything.  It is evidence ONLY on a real capture.
  And even then, the field that decides whether there is a defect is bits 0 and 9
  of the mask (`at_node` = `s[0] && s[9]`) at a corner with no straight-through
  lane: both set means `in.front` was 1 with nothing ahead (the defect); clear
  means `in.front` was 0 there and there is no defect.  Do not infer the defect
  from (a) -- the asymmetry lives in `at_node`, not in this ratio.

`nav` cannot settle it either: main.c only ever moves `nav` from a COMMANDED
turn (main.c:1458, 1884-1885), so it mirrors the command rather than the robot's
physical rotation, and the B lines come from the same dead reckoning the audit
would be testing.  Settling `front` needs the robot and a maze.

LINE FORMAT, SHARED WITH parse_telemetry.py
Both tools parse through `parse_telemetry.parse_line()`, so they cannot drift
apart about field order -- or about what counts as an alarming line, since the
BAD/BANNER split is that function's verdict too.  The raw capture this writes is
byte-for-byte the input `parse_telemetry.py` already takes; run it on any capture
this makes.  Note that the firmware's `Hi ,mmdi` boot banner appears on every
real capture and is BANNER (benign), not UNPARSED (the alarm).

Usage:
  python scripts/bt_monitor.py                      # GUI, opens COM9 if it is there
  python scripts/bt_monitor.py --port COM7          # GUI, connect at once to another
  python scripts/bt_monitor.py --replay cap.txt     # GUI, replay a capture
  python scripts/bt_monitor.py --replay cap.txt --headless
  python scripts/bt_monitor.py --demo               # GUI smoke test, no hardware

COM9 is this bench's adapter (`DEFAULT_PORT`): it is pre-selected, and opened at
start-up, whenever a port named COM9 is enumerated, so a bring-up does not begin
with a dropdown.  The match is on the name only -- see that constant -- and the
guard is against absence: no COM9 enumerated means nothing is selected, nothing
is opened, and the picker behaves exactly as it did before.

Needs `pip install pyserial` (the repo's only third-party dependency) for live
capture.  --replay, --demo and --headless need nothing but the stdlib.
"""

import argparse
import csv
import json
import os
import queue
import subprocess
import sys
import threading
import time

# Console is cp1252 on this machine; unicode in output crashes it (CLAUDE.md #8).
sys.stdout.reconfigure(encoding="utf-8")
sys.stderr.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT_CODES = os.path.dirname(HERE)
DEFAULT_ORACLE = os.path.join(ROBOT_CODES, "build", "brain_oracle.exe")
FIXTURES = os.path.join(ROBOT_CODES, "test", "fixtures")
CAPTURE_DIR = os.path.join(ROBOT_CODES, "build", "bt_captures")

# The bench adapter.  Windows hands out COM numbers per adapter, and this one is
# COM9 on this machine -- so the monitor can open the robot's link itself instead
# of making the operator pick the same entry out of a dropdown on every bring-up.
# The name is matched; the DEVICE is not identified (no USB VID/PID or
# description test), so this is a one-machine convenience rather than a claim
# about what COM9 is.  The only thing guarded is absence: no COM9 enumerated
# means nothing is pre-selected and nothing is opened.  See `_preferred_port`.
DEFAULT_PORT = "COM9"

sys.path.insert(0, HERE)
import parse_telemetry as pt                    # noqa: E402  (path set above)

try:
    import serial
    import serial.tools.list_ports
    HAVE_SERIAL = True
except ImportError:                              # pragma: no cover
    HAVE_SERIAL = False

# --------------------------------------------------------------- the frame
#
# The brain's own dead-reckoned frame (brain.h: "place the robot at the origin
# of its own frame ... the initial physical facing is taken to be the frame's
# north").  Latched to the 20 cm lattice.  This is NOT the maze's world frame,
# so nothing here is compared against real_field.json.
CELL_CM = 20.0
# MazeHeading order, which mirrors the firmware's nav: 0=N 1=W 2=S 3=E.
HEADING_NAME = {0: "N", 1: "W", 2: "S", 3: "E"}
HEADING_VEC = {0: (0, 1), 1: (-1, 0), 2: (0, -1), 3: (1, 0)}
# The firmware's own turn arithmetic, exactly as brain_host.c uses it.
TURN_DELTA = {"F": 0, "S": 0, "L": 1, "R": 3, "B": 2}

# --------------------------------------------------- sensor mask bit meanings
#
# tlm_front() packs s[0..9] into bits 0..9, so bit i is s[i].  Roles from
# SENSORS.md, via parse_telemetry.FRONT_ROLE.
CENTRE_MASK = 0x078        # s[3..6]  -- the centre group, = in.front
LEFT_MASK = 0x004          # s[2]     -- LEFT branch detector
RIGHT_MASK = 0x080         # s[7]     -- RIGHT branch detector
TARGET_MASK = 0x102        # s[1], s[8] -- TARGET detectors

# Palette, reused from simulator/maze solver/maze_solver.py so the bring-up view
# reads like the tool the algorithm was designed in (CLAUDE.md #7).
BG = "#12141c"
PANEL_BG = "#171a24"
CARD_BG = "#1d2130"
CANVAS_BG = "#0d0f15"
FG = "#e6e8f0"
FG_MUTED = "#878da3"
ACCENT = "#5b8cff"
GRID_COLOR = "#1a1e2c"
VISITED_EDGE = "#5b8cff"
STUB_COLOR = "#ffd23f"
FASTEST_PATH = "#06d26a"
TRAIL_COLOR = "#3b4470"
NODE_COLOR = "#aab2d6"
NODE_OUTLINE = "#0d0f15"
START_COLOR = "#3ddc84"
TARGET_RING = "#ff5d6c"
ROBOT_COLOR = "#ff5d6c"
OK_COLOR = "#2ee6a6"
BAD_COLOR = "#ff5d6c"
WARN_COLOR = "#ffd23f"
FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 10, "bold")
FONT_MONO = ("Consolas", 10)

PHASE_NAME = {0: "EXPLORE", 1: "RETURN_HOME", 2: "FAST_RUN", 3: "DONE"}

# The canvas switch's two settings, spelled for the button.  The internal names
# are short because they are what `_draw` branches on; the labels are what the
# operator reads, and `path draw` is the one that needs saying (a bare "path"
# reads like a file path on a row that also has a port picker).
CANVAS_NAME = {"diag": "diag", "path": "path draw"}

# The one byte the app ever says to the robot.  Value is arbitrary: the firmware's
# USART1 IRQ flags ANY received byte when USE_MAZE_HEALTH is 1, so the payload only
# has to be something the operator can recognise in a wiring trace.  Kept as
# `?` because it is also the byte a terminal user would type by hand to test.
PROBE_BYTE = b"?"


# ==========================================================================
# DERIVATION -- what the brain was told, and what the sensors say
# ==========================================================================

def derive_brainin(j):
    """A J record -> (the BrainIn the firmware built, what the mask implies).

    `brainin` is what we hand the oracle.  `derived` is what the raw sensor mask
    independently says.  The firmware computes left/right FROM the mask, so
    re-computing them from the mask must reproduce the reported L/R -- and a
    mis-wire cannot hide behind that, because it is the same arithmetic.
    """
    mask = j["front"]
    centre = 1 if (mask & CENTRE_MASK) else 0
    d_left = 1 if ((mask & LEFT_MASK) and centre) else 0
    d_right = 1 if ((mask & RIGHT_MASK) and centre) else 0

    brainin = {
        "front": centre,                      # in.front, main.c:932
        "left": j["L"],                       # in.left,  from left_poss
        "right": j["R"],
        "back": 1,                            # in.back,  a constant (main.c:933)
        "target": j["target"],
        "dist_cm": j["dist_cm"],
    }
    derived = {"front": centre, "left": d_left, "right": d_right}
    return brainin, derived


def classify(mask, derived):
    """The flags one junction earns, from the mask alone."""
    notes = []
    laterals = derived["left"] + derived["right"]
    if derived["front"] and laterals == 1:
        # THE ambiguous state, and the reason this tool exists.  front=1 with
        # exactly one lateral is what a corner produces -- and a corner must NOT
        # be driven straight.  It is also what a T-junction with a forward exit
        # produces, and there straight is right.  The brain cannot tell them
        # apart, because in.front is implied by the same centre group that let
        # the lateral fire.
        notes.append("CORNER-SUSPECT")
    elif derived["front"] and laterals == 2:
        notes.append("CROSSROAD")
    elif not derived["front"] and not laterals:
        notes.append("DEAD-END")
    if not derived["front"] and laterals:
        # Unreachable in a single instant -- both need the centre group -- so it
        # means a LATCHED lateral outliving the centre group.  Worth seeing: it
        # is the only way the brain ever gets a real "one lateral, no forward",
        # and it arrives by timing accident rather than by design.
        notes.append("STALE-LATERAL")
    if (mask & TARGET_MASK) and not (mask & CENTRE_MASK):
        notes.append("stray-target-bit")
    return notes


# ==========================================================================
# THE ORACLE -- the real brain, driven over a pipe
# ==========================================================================

class OracleError(Exception):
    pass


class Oracle:
    """`build/brain_oracle.exe` as a one-line-in / one-line-out service."""

    def __init__(self, exe):
        # CreateProcess does not reliably resolve a relative path with forward
        # slashes on Windows -- and this is launched from Git Bash, where those
        # are the norm.  Absolutise before handing it over.
        exe = os.path.abspath(exe)
        if not os.path.exists(exe):
            raise OracleError(
                "brain_oracle not built: %s\n"
                "  build it with:  powershell -NoProfile -ExecutionPolicy "
                "Bypass -File ./scripts/build_all.ps1" % exe)
        self.exe = exe
        self.proc = subprocess.Popen(
            [exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1)
        self.proc.stdin.write("INIT\n")
        self.proc.stdin.flush()
        if not self._wait_for("OK").startswith("OK"):
            raise OracleError("oracle did not answer INIT")

    def _wait_for(self, prefix):
        for _ in range(64):
            line = self.proc.stdout.readline()
            if not line:
                raise OracleError("oracle died (no output)")
            line = line.rstrip("\r\n")
            if line.startswith(prefix) or line.startswith("E "):
                return line
        return ""

    def step(self, brainin):
        """One BrainIn -> (move, status dict, plans)."""
        self.proc.stdin.write("%d %d %d %d %d %d\n" % (
            brainin["left"], brainin["right"], brainin["front"],
            brainin["back"], brainin["target"], brainin["dist_cm"]))
        self.proc.stdin.flush()

        line = self._wait_for("S ")
        if not line.startswith("S "):
            raise OracleError("no S response: %r" % line)
        f = line.split()
        st = {
            "phase": int(f[2]), "node": int(f[3]), "x": int(f[4]),
            "y": int(f[5]), "reports": int(f[6]), "cells": int(f[7]),
            "drift_cm": int(f[8]), "target_found": int(f[9]),
            "finished": int(f[10]), "fast_time_s": float(f[11]),
        }
        plans = {}
        if st["finished"]:
            for key in ("HOME", "FAST"):
                pl = self.proc.stdout.readline().rstrip("\r\n")
                if not pl.startswith(key):
                    raise OracleError("expected %s, got %r" % (key, pl))
                plans[key] = pl.split(" ", 1)[1].strip() if " " in pl else ""
        return f[1], st, plans

    def close(self):
        try:
            self.proc.stdin.write("QUIT\n")
            self.proc.stdin.flush()
            self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass


# ==========================================================================
# MISSION -- the believed map, in the brain's own frame
# ==========================================================================

class Mission:
    """The brain's believed world, reconstructed from the wire.

    Position is tracked from `dist_cm` and the frame heading, and the B lines are
    then used to CHECK it.  Both come from the same dead reckoning, so agreement
    is weak evidence -- but a disagreement means the brain and the firmware are
    not stepping together, which is worth knowing before reading anything else.

    The believed map is built from laterals, the back edge, and the edges the
    robot actually drove.  `in.front` is deliberately NOT used: it is the thing
    under audit, and building the map from it would make the audit a tautology.
    """

    def __init__(self, oracle=None):
        self.oracle = oracle
        self.nodes = {}          # id -> (x, y)
        self.edges = set()       # frozenset({id_a, id_b}) confirmed by driving
        self.exits = {}          # id -> set of headings with a known exit
        self.order = []          # ids in order first seen
        self.heading = 0         # frame heading; brain_init says start is north
        self.node = None         # the node the robot is standing on
        self.pos = (0.0, 0.0)
        self.target_node = None
        self.target_found = False
        self.junctions = []
        self.plans = {}
        self.plan_lines = []
        self.b_disagreements = []
        self.counters = {
            "lines": 0, "chatter": 0, "unparsed": 0, "s": 0, "z": 0,
            "decision_mismatch": 0, "derivation_mismatch": 0,
            "position_mismatch": 0, "corner_suspect": 0, "dead_end": 0,
            "stale_lateral": 0, "plan_mismatch": 0,
            # The degeneracy, measured rather than argued: how many junctions
            # had front == (left or right), and how many of those were driven
            # straight.  See the module docstring.
            "front_degenerate": 0, "ambiguous_straight": 0,
        }

    # -------------------------------------------------------------- internals
    def _register(self, node, xy):
        """Register a node; a second, different position for one id is a fault.

        The same node id in two places means a junction was lost or duplicated
        upstream: the brain's map and the robot's path have parted company.
        """
        if node in self.nodes:
            if self.nodes[node] != xy:
                self.counters["position_mismatch"] += 1
                return False
            return True
        self.nodes[node] = xy
        self.order.append(node)
        return True

    def _exit(self, node, hdg):
        self.exits.setdefault(node, set()).add(hdg)

    # ------------------------------------------------------------------ lines
    def feed(self, kind, rec):
        if kind == "S":
            self.counters["s"] += 1
            return None
        if kind == "Z":
            self.counters["z"] += 1
            return None
        if kind == "J":
            return self.junction(rec)
        if kind == "B":
            self._bringup(rec)
        return None

    def junction(self, j):
        """The heart: derive, advance the map, ask the oracle, compare."""
        brainin, derived = derive_brainin(j)
        notes = classify(j["front"], derived)

        deriv_ok = (derived["left"] == j["L"] and derived["right"] == j["R"])
        if not deriv_ok:
            self.counters["derivation_mismatch"] += 1
            notes.append("DERIVATION-MISMATCH")

        # The heading the robot ARRIVED on.  Everything about this junction is
        # expressed relative to it, and the turn happens after the report.
        heading_in = self.heading

        # (1) Advance by the link just driven.  On the first report the robot
        # has driven from the start node, which is the origin of the frame --
        # so dist_cm>0 here means the origin is behind us and it is a node too.
        cells = int(round(j["dist_cm"] / CELL_CM))
        if cells:
            dx, dy = HEADING_VEC[heading_in]
            self.pos = (self.pos[0] + dx * cells * CELL_CM,
                        self.pos[1] + dy * cells * CELL_CM)
            if self.node is None:
                # The start node: never reported, because the robot only speaks
                # at a junction, but it is in the brain's map and on the plot.
                self._register(0, (0.0, 0.0))
                self._exit(0, heading_in)
                self._exit(0, (heading_in + 2) & 3)
            else:
                self._exit(self.node, heading_in)
            self.edges.add(frozenset((0 if self.node is None else self.node,
                                      j["node"])))
            self._exit(j["node"], (heading_in + 2) & 3)

        arrived_ok = self._register(j["node"], self.pos)
        self.node = j["node"]

        # The laterals, in the frame heading the robot arrived on.
        if derived["left"]:
            self._exit(j["node"], (heading_in + 1) & 3)
        if derived["right"]:
            self._exit(j["node"], (heading_in + 3) & 3)

        if j["target"]:
            self.target_found = True
            self.target_node = j["node"]

        if "CORNER-SUSPECT" in notes:
            self.counters["corner_suspect"] += 1
        if "DEAD-END" in notes:
            self.counters["dead_end"] += 1
        if "STALE-LATERAL" in notes:
            self.counters["stale_lateral"] += 1
        # The whole degeneracy, in one comparison on one line's own fields.
        # If this holds everywhere, in.front carried no information the
        # laterals did not already carry -- measured on the real robot.
        if derived["front"] == (1 if (derived["left"] or derived["right"]) else 0):
            self.counters["front_degenerate"] += 1
        if (derived["front"] and (derived["left"] + derived["right"]) == 1
                and j["ch"] == "F"):
            self.counters["ambiguous_straight"] += 1

        # (2) The decision check.
        oracle_move, st, err = None, None, None
        if self.oracle is not None:
            try:
                oracle_move, st, plans = self.oracle.step(brainin)
                if plans:
                    self.plans = plans
            except OracleError as exc:
                err = str(exc)
                self.oracle = None
        if oracle_move is not None and oracle_move != j["ch"]:
            self.counters["decision_mismatch"] += 1
            notes.append("DECISION-MISMATCH")

        # (3) Turn.  The robot then drives on under the new heading.
        self.heading = (heading_in + TURN_DELTA.get(j["ch"], 0)) & 3

        jd = dict(j)
        jd.update({"n": len(self.junctions) + 1, "derived": derived,
                   "notes": notes, "deriv_ok": deriv_ok, "arrived_ok": arrived_ok,
                   "oracle_move": oracle_move, "oracle": st,
                   "oracle_error": err, "heading_in": heading_in,
                   "pos": self.pos, "brainin": brainin})
        self.junctions.append(jd)
        return jd

    def _bringup(self, b):
        """A B line: the brain's own snapshot, to be checked, not obeyed."""
        xy = (float(b["x"]), float(b["y"]))
        if b["node"] in self.nodes and self.nodes[b["node"]] != xy:
            self.b_disagreements.append(
                {"node": b["node"], "believed": self.nodes[b["node"]],
                 "reported": xy, "ms": b["ms"]})

    def plan_line(self, text):
        """A bare command string the firmware dumps on KEY3 (main.c:1481)."""
        s = text.strip()
        if not s or not all(c in "FLRBSD" for c in s):
            return None
        self.plan_lines.append(s)
        robot = s[:-1] if s.endswith("D") else s     # set_plan appends 'D'
        want = self.plans.get("HOME")
        if want is not None and robot != want:
            self.counters["plan_mismatch"] += 1
            return ("PLAN-MISMATCH", robot, want)
        return ("PLAN", robot, want)

    # ----------------------------------------------------------------- audit
    def ambiguity_report(self):
        """Junctions where the brain drove straight on a one-lateral input.

        NOT a list of errors.  A one-lateral input is what a corner and a
        T-junction both produce, and the brain cannot tell them apart -- so this
        is the shortlist of decisions that need a human and the real maze, which
        is exactly what STATUS.md's first bring-up question asks for.
        """
        out = []
        for j in self.junctions:
            d = j["derived"]
            if d["front"] and (d["left"] + d["right"]) == 1 and j["ch"] == "F":
                out.append({"n": j["n"], "node": j["node"], "ms": j["ms"],
                            "mask": j["front"], "left": d["left"],
                            "right": d["right"], "heading_in": j["heading_in"],
                            "dist_cm": j["dist_cm"]})
        return out

    def _count(self, kind, rec):
        """Bucket a non-record line.  The BAD/BANNER split is parse_line's -- one
        definition, shared with parse_telemetry.py -- so the two tools' unparsed
        counters cannot come to mean different things."""
        self.counters["lines"] += 1
        if kind == "BAD":
            # Telemetry-shaped and did not parse: the alarming kind, and the
            # one a dropped junction shows up as.
            self.counters["unparsed"] += 1
            return "BAD"
        if kind == "BANNER":
            self.counters["chatter"] += 1          # the boot banner, etc.
            return "CHATTER"
        return kind

    def verdict(self):
        c = self.counters
        return {"junctions": len(self.junctions),
                "verdict": "FAIL" if c["decision_mismatch"] else "PASS",
                **c}


# ==========================================================================
# RECORDING
# ==========================================================================

CSV_COLUMNS = [
    "host_ms", "robot_ms", "type",
    "ch", "nav", "head", "raw", "cm", "front", "rear", "L", "R", "cross",
    "target", "dist_cm", "node", "e0", "e1", "state", "x", "y", "drift", "move",
    "d_front", "d_left", "d_right", "oracle_move", "oracle_phase",
    "match", "notes",
    # Appended, never interleaved: a capture CSV is read by column NAME, so
    # adding at the end cannot shift a column an existing reader relies on.
    # The health build's own two lines land here.
    "keys", "loop", "bits", "what", "val",
]


def health_row(kind, rec):
    """One CSV row's worth of kwargs for an H or T line.

    The eighteen channels go in as ONE field rather than as eighteen columns:
    the mission columns are irrelevant to a health build, and duplicating the
    table for two line types that never appear together would make both harder
    to read.  Semicolons, not commas, so the CSV stays one row per wire line and
    a naive split on ',' still gives sane columns.

    `bits` carries 0/1 per pad, as the firmware sent them; the T line's `val` is
    the one time IR_mid itself is on the wire.
    """
    if kind == "H":
        return {"type": "H", "robot_ms": rec["ms"], "keys": "%X" % rec["keys"],
                "loop": rec["loop"], "head": rec["head"], "gz": rec["gz"],
                "za": rec["za"],
                "bits": ";".join(str(v) for v in rec["bits"])}
    return {"type": "T", "robot_ms": rec["ms"], "what": rec["what"],
            "val": ";".join(str(v) for v in rec["val"])}


class Recorder:
    """Three files: the raw log, a flat CSV, and the run's verdict as JSON.

    The raw log is written verbatim because it is exactly what
    `parse_telemetry.py` reads -- so any capture this makes can be analysed with
    the tool that already exists.  Everything is flushed per line: a robot that
    runs away must not run away with the evidence.
    """

    def __init__(self, directory=CAPTURE_DIR, stamp=None):
        os.makedirs(directory, exist_ok=True)
        stamp = stamp or time.strftime("%Y%m%d-%H%M%S")
        self.base = os.path.join(directory, "capture_%s" % stamp)
        self.txt = open(self.base + ".txt", "w", encoding="utf-8", newline="")
        self.csvf = open(self.base + ".csv", "w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.csvf, fieldnames=CSV_COLUMNS,
                                     extrasaction="ignore")
        self.writer.writeheader()
        self.t0 = time.monotonic()
        self.rows = 0

    def host_ms(self):
        return int((time.monotonic() - self.t0) * 1000)

    def raw(self, line):
        self.txt.write(line + "\n")
        self.txt.flush()

    def row(self, **kw):
        kw.setdefault("host_ms", self.host_ms())
        for c in CSV_COLUMNS:
            kw.setdefault(c, "")
        self.writer.writerow(kw)
        self.csvf.flush()
        self.rows += 1

    def write_json(self, payload):
        with open(self.base + ".json", "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, default=str)

    def close(self):
        self.txt.close()
        self.csvf.close()


# ==========================================================================
# LIVE SOURCE
# ==========================================================================

class SerialSource(threading.Thread):
    """pyserial readline() -> a queue.  No tkinter call ever happens here."""

    def __init__(self, port, baud, out):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.out = out
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:              # port yanked mid-run
                self.out.put(("error", str(exc)))
                return
            if raw:
                self.out.put(("line", raw.decode("ascii", "replace").strip()))

    def send(self, data):
        """Put a byte on the link.  The one thing the app says to the robot.

        Used for the liveness probe: the firmware answers any received byte with
        its `Hi ,mmdi` banner, which is how the operator knows the radio is up
        rather than inferring it from a stream that has not started yet."""
        try:
            self.ser.write(data if isinstance(data, bytes) else data.encode())
            return True
        except Exception as exc:
            self.out.put(("error", str(exc)))
            return False

    def close(self):
        self.stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


def list_ports():
    if not HAVE_SERIAL:
        return []
    return ["%s - %s" % (p.device, p.description)
            for p in serial.tools.list_ports.comports()]


def port_device(label):
    return label.split(" - ", 1)[0].strip()


# ==========================================================================
# THE PIPELINE, SHARED BY GUI AND HEADLESS
# ==========================================================================

class Pipeline:
    """Lines in, mission state out, optionally recorded.  No UI in here."""

    def __init__(self, oracle_exe=None, record=False):
        self.mission = None
        self.oracle_error = None
        self.oracle = None
        if oracle_exe:
            try:
                self.oracle = Oracle(oracle_exe)
            except OracleError as exc:
                self.oracle_error = str(exc)
        self.mission = Mission(self.oracle)
        # The bench health model.  Fed ALWAYS, in every mode, not only when the
        # health view is up: an H line means the firmware is in a stopped state,
        # the checks are free, and a fault that appears while nobody is looking
        # at that panel is exactly the fault that gets missed.  It is also what
        # --health reports, so the live view and the exit code cannot disagree.
        self.health = pt.HealthModel()
        # The same H/T records, kept as parse() would have bucketed them, so
        # --health can hand them to the SAME formatter parse_telemetry.py uses.
        # That is what makes `bt_monitor --replay f --health` and
        # `parse_telemetry f` print an identical health section: not two
        # formatters kept in step by hand, but one.
        self.health_by_kind = {"H": [], "T": []}
        self.bad_lines = []
        self.rec = Recorder() if record else None

    def feed(self, line):
        """Feed one raw wire line. Returns ('J', junction) or (kind, None)."""
        m = self.mission
        if self.rec:
            self.rec.raw(line)

        kind, rec = pt.parse_line(line, 0)
        if kind is None:
            return None, None

        if kind in ("BAD", "BANNER"):
            # A command-string dump on KEY3 is not a telemetry line; try it as
            # a plan before writing it off as noise.
            pl = m.plan_line(rec[1])
            if pl:
                if self.rec:
                    self.rec.row(type=pl[0], notes=str(pl[1:]))
                return pl[0], pl
            tag = m._count(kind, rec)
            if self.rec:
                self.rec.row(type=tag, notes=rec[1][:80])
            return tag, None

        m.counters["lines"] += 1
        if kind == "BAD":
            self.bad_lines.append(rec)

        if kind in ("H", "T"):
            # The health build.  It has no map and no decision in it, so it goes
            # to the health model and nowhere near the mission state machine --
            # an H line must not advance the robot's believed position.
            self.health.feed(kind, rec)
            self.health_by_kind[kind].append(rec)
            if self.rec:
                self.rec.row(**health_row(kind, rec))
            return kind, rec

        result = m.feed(kind, rec)

        if self.rec:
            if kind == "J" and result is not None:
                self.rec.row(type="J", robot_ms=rec["ms"], **rec,
                             d_front=result["derived"]["front"],
                             d_left=result["derived"]["left"],
                             d_right=result["derived"]["right"],
                             oracle_move=result["oracle_move"] or "",
                             oracle_phase=(result["oracle"] or {}).get("phase", ""),
                             match=("" if result["oracle_move"] is None
                                    else int(result["oracle_move"] == rec["ch"])),
                             notes=" ".join(result["notes"]))
            else:
                self.rec.row(type=kind, **rec)
        return kind, result

    def health_verdict(self):
        return self.health.worst()

    def health_report(self):
        """Print the bench health section.  -> exit code.

        Delegates to `parse_telemetry.health_report()` on purpose.  The health
        checks and their wording live in one place, so the live tool and the
        batch tool cannot drift -- the same reasoning that makes parse_line()
        the single definition of the wire format.
        """
        return pt.health_report(self.health_by_kind, self.bad_lines)

    def summary(self):
        return self.mission.verdict()

    def shutdown(self):
        if self.rec:
            self.rec.write_json({"verdict": self.mission.verdict(),
                                 "plans": self.mission.plans,
                                 "ambiguous_straights":
                                     self.mission.ambiguity_report(),
                                 "b_disagreements":
                                     self.mission.b_disagreements})
            self.rec.close()
            self.rec = None
        if self.oracle is not None:
            self.oracle.close()
            self.oracle = None
            self.mission.oracle = None


# ==========================================================================
# HEADLESS
# ==========================================================================

def run_headless_health(pipe, lines):
    """--health: the bench check, assertable from build_all.ps1.

    Exit 1 iff a finding is FAIL, which is the same verdict the GUI colours and
    parse_telemetry.py prints -- all three go through HealthModel.findings().
    Exit 2 is reserved for "this capture has nothing to check", which is a
    different thing from "the robot is unhealthy" and must not read as a pass.
    """
    for line in lines:
        pipe.feed(line)

    if not pipe.health.h:
        print("bt_monitor --health: no H lines in this capture -- nothing to check.")
        print("  H/T lines exist only when USE_MAZE_HEALTH is 1 in main.c")
        print("  'Health check') and only while the robot stands still.  A")
        print("  mission capture has none, so this is almost certainly the wrong")
        print("  file, and it must not look like a pass.")
        return 2

    rc = pipe.health_report()
    base = pipe.rec.base if pipe.rec else None
    pipe.shutdown()
    if base:
        print("\n recorded: %s.txt / .csv / .json" % base)
    print(" bt_monitor --health exit %d (%s)"
          % (rc, "FAIL present" if rc else "no FAIL"))
    return rc


def run_headless(args):
    lines = None
    if args.replay:
        with open(args.replay, "r", encoding="utf-8", errors="replace") as fh:
            lines = [l.rstrip("\r\n") for l in fh]
    elif not sys.stdin.isatty():
        lines = [l.rstrip("\r\n") for l in sys.stdin]
    else:
        print("bt_monitor --headless needs --replay <file> or a pipe on stdin")
        return 2

    pipe = Pipeline(args.oracle, record=args.record)
    # --health does not need the oracle at all: it checks the SENSORS, and there
    # is no decision in an H line to check against.  Requiring the oracle there
    # would make the health check fail on a machine that has not built it, and
    # then the two checks in build_all.ps1 would be coupled for no reason.
    if pipe.oracle is None and not (args.no_oracle or args.health):
        print("bt_monitor: %s" % (pipe.oracle_error or "no oracle"))
        return 2

    if args.health:
        return run_headless_health(pipe, lines)

    print("  %-4s %-8s %-10s %-8s %-6s %-6s %s"
          % ("#", "t(ms)", "mask->FLR", "dir", "robot", "oracle", "notes"))
    for line in lines:
        kind, res = pipe.feed(line)
        if kind == "J" and res is not None:
            d = res["derived"]
            print("  %-4d %-8d %03X->%d%d%d  %-8s %-6s %-6s %s"
                  % (res["n"], res["ms"], res["front"], d["front"], d["left"],
                     d["right"], HEADING_NAME[res["heading_in"]], res["ch"],
                     res["oracle_move"] or "-", " ".join(res["notes"]) or "ok"))

    m = pipe.mission
    c = m.counters
    print("\n" + "=" * 74)
    print(" junctions checked : %d" % len(m.junctions))
    print(" DECISION MISMATCH : %d" % c["decision_mismatch"])
    print(" derivation        : %d" % c["derivation_mismatch"])
    print(" position          : %d" % c["position_mismatch"])
    print(" plan mismatch     : %d" % c["plan_mismatch"])
    print(" CORNER-SUSPECT    : %d  (front=1 with one lateral -- ambiguous)"
          % c["corner_suspect"])
    print(" dead ends         : %d" % c["dead_end"])
    print(" stale lateral     : %d" % c["stale_lateral"])
    print(" unparsed lines    : %d  (chatter: %d)"
          % (c["unparsed"], c["chatter"]))
    if m.b_disagreements:
        print(" B-line position disagreements: %d" % len(m.b_disagreements))

    if args.front_audit and m.junctions:
        n = len(m.junctions)
        d = c["front_degenerate"]
        print("\n FRONT AUDIT -- what this number can and cannot tell you")
        print("  in.front == (in.left or in.right) at %d of %d junctions." % (d, n))
        if d == n:
            print("  in.front carried no information the laterals did not already")
            print("  carry -- at a ONE-LATERAL junction that makes a corner and a")
            print("  T-junction the same input, so the brain cannot tell them apart.")
            print("  BUT SEE THE CAVEAT: on a SYNTHETIC capture this is true by")
            print("  construction (the masks were built from the firmware's own")
            print("  stop test), so it proves nothing.  It is evidence only on a")
            print("  REAL capture -- and even then the thing that decides whether")
            print("  there is a defect is bits 0 and 9 of the mask (at_node =")
            print("  s[0]&&s[9]) at a corner with no straight-through lane:")
            print("  both set -> in.front was 1 with nothing ahead (a defect);")
            print("  clear -> in.front was 0 -> no defect.")
        straights = m.ambiguity_report()
        print("\n  %d junction(s) drove STRAIGHT on a one-lateral input."
              % len(straights))
        if straights:
            print("  Check each against the real maze: at a corner the correct")
            print("  move was a turn, and nothing on the wire could say so.")
            for b in straights:
                print("   #%-3d node %-4d t=%-8d mask=%03X L%d R%d heading_in=%s"
                      % (b["n"], b["node"], b["ms"], b["mask"], b["left"],
                         b["right"], HEADING_NAME[b["heading_in"]]))
    print("=" * 74)

    base = pipe.rec.base if pipe.rec else None
    pipe.shutdown()
    if base:
        print(" recorded: %s.txt / .csv / .json" % base)
        print("           the .txt is exactly parse_telemetry.py's input, e.g.")
        print("           python scripts/parse_telemetry.py %s.txt" % base)
    return 1 if c["decision_mismatch"] else 0


# ==========================================================================
# GUI
# ==========================================================================

class MonitorApp:
    # How many lines the TERMINAL keeps.  Big enough for a whole bench session's
    # worth of key edges and calibrations, small enough that the Text widget is
    # never the reason a frame takes long -- and trimming is what stops an
    # all-nighter's worth of unparsed lines from growing without bound.
    TERM_LINES = 400

    def __init__(self, root, tk, ttk, args):
        self.root, self.tk = root, tk
        self.args = args
        self.pipe = Pipeline(args.oracle, record=args.record)
        self.queue = queue.Queue()
        self.source = None
        self.playing = False
        self.replay_lines = []
        self.replay_i = 0
        self.last_j = None
        self.last_s = None
        self._port_manual = False   # set once the operator picks a port by hand
        self._thr_sig = {}      # last-seen T values, so only CHANGES are logged
        self._prev_keys = None  # last key bitmask, so only EDGES are logged

        self._build(tk, ttk)
        if args.demo:
            self._load(self._demo_lines())
            self.playing = True
        elif args.replay:
            with open(args.replay, "r", encoding="utf-8", errors="replace") as fh:
                self._load([l.rstrip("\r\n") for l in fh])
            self.playing = True
        self.refresh_ports()
        if args.port:
            # An explicit --port is an instruction, not a preference: open it
            # even if enumeration did not see it, and fail loudly if it will not
            # open.  It also wins over the COM9 default.
            self.connect(port_device(args.port))
        else:
            self._autoconnect()
        self.root.after(30, self._tick)

    # -------------------------------------------------------------- UI build
    def _build(self, tk, ttk):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TCombobox", fieldbackground=CARD_BG,
                        background=CARD_BG, foreground=FG)

        bar = tk.Frame(self.root, bg=PANEL_BG)
        bar.pack(side="top", fill="x")
        tk.Label(bar, text="Port", bg=PANEL_BG, fg=FG_MUTED,
                 font=FONT).pack(side="left", padx=(10, 4), pady=8)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=36,
                                     state="readonly")
        self.port_box.pack(side="left", pady=8)
        self.port_box.bind("<<ComboboxSelected>>", self._port_picked)
        tk.Button(bar, text="Refresh", command=self.refresh_ports, bg=CARD_BG,
                  fg=FG, font=FONT, relief="flat").pack(side="left", padx=4)
        tk.Label(bar, text="Baud", bg=PANEL_BG, fg=FG_MUTED,
                 font=FONT).pack(side="left", padx=(12, 4))
        self.baud_var = tk.StringVar(value=str(self.args.baud))
        tk.Entry(bar, textvariable=self.baud_var, width=8, bg=CARD_BG, fg=FG,
                 insertbackground=FG, font=FONT_MONO,
                 relief="flat").pack(side="left")
        self.btn_conn = tk.Button(bar, text="Connect", command=self._toggle,
                                  bg=ACCENT, fg="#0d0f15", font=FONT_BOLD,
                                  relief="flat", width=11)
        self.btn_conn.pack(side="left", padx=8)
        self.rec_var = tk.BooleanVar(value=bool(self.args.record))
        tk.Checkbutton(bar, text="Record", variable=self.rec_var, bg=PANEL_BG,
                       fg=FG, selectcolor=CARD_BG, activebackground=PANEL_BG,
                       activeforeground=FG, font=FONT).pack(side="left", padx=6)
        # TWO SWITCHES, INDEPENDENT.  CANVAS is WHAT IS DRAWN (the pad board vs
        # the believed map and trail); CARDS is WHICH COLUMN OF READOUTS sits
        # beside it (the mission cards vs the bench health cards).  They used to
        # be one button, which meant wanting the health cards up while watching
        # where the robot thinks it is -- the normal thing to want on the bench
        # -- was impossible.  See set_canvas/set_cards.
        self.btn_canvas = tk.Button(bar, text="Canvas: path draw",
                                    command=self._toggle_canvas, bg=CARD_BG,
                                    fg=FG, font=FONT, relief="flat", width=16)
        self.btn_canvas.pack(side="left", padx=6)
        self.btn_cards = tk.Button(bar, text="Cards: mission",
                                   command=self._toggle_cards, bg=CARD_BG,
                                   fg=FG, font=FONT, relief="flat", width=16)
        self.btn_cards.pack(side="left", padx=6)
        # Packed from the RIGHT so the second button cannot squeeze the link
        # status off the bar on a narrow window -- "link error: ..." is the most
        # valuable thing on this row and it is the one pack would clip first.
        self.lbl_conn = tk.Label(bar, text="not connected", bg=PANEL_BG,
                                 fg=FG_MUTED, font=FONT)
        self.lbl_conn.pack(side="right", padx=10)

        # ---- THE TERMINAL.  Deliberately an EVENT log, not a raw dump of the
        # wire: the health stream runs at 20 lines a second, so a true raw
        # terminal would push the `Hi ,mmdi` banner -- the one line that proves
        # the link -- off the top within a second of arriving, which is exactly
        # the opposite of what it is wanted for.  So this shows the banner,
        # every unparsed line, key edges, the calibrations and link changes, and
        # the 20 Hz stream is drawn in the bar above instead.  Nothing is hidden
        # by leaving it out; it is just in the other pane.
        #
        # Packed BEFORE the body: pack hands out space in call order, so a
        # fill="x" strip added afterwards would get whatever the expanding body
        # left over -- i.e. nothing.
        term = tk.Frame(self.root, bg=PANEL_BG)
        term.pack(side="bottom", fill="x")
        head = tk.Frame(term, bg=PANEL_BG)
        head.pack(fill="x")
        self.lbl_term = tk.Label(head, text="TERMINAL   (link events -- the 20 Hz "
                                            "stream is in the bar above)",
                                 bg=PANEL_BG, fg=ACCENT, font=FONT_BOLD)
        self.lbl_term.pack(side="left", padx=8, pady=(4, 0))
        tk.Button(head, text="Ping robot", command=self._ping, bg=CARD_BG, fg=FG,
                  font=FONT, relief="flat").pack(side="right", padx=8, pady=3)
        tk.Button(head, text="Clear", command=self._clear_term, bg=CARD_BG, fg=FG,
                  font=FONT, relief="flat").pack(side="right", padx=2, pady=3)
        self.term = tk.Text(term, height=7, bg=CANVAS_BG, fg=FG,
                            font=("Consolas", 9), relief="flat", wrap="none",
                            highlightthickness=0, insertbackground=FG)
        self.term.pack(fill="x", padx=8, pady=(2, 8))
        self.term.tag_configure("banner", foreground=OK_COLOR)
        self.term.tag_configure("bad", foreground=BAD_COLOR)
        self.term.tag_configure("warn", foreground=WARN_COLOR)
        self.term.tag_configure("muted", foreground=FG_MUTED)
        self.term.configure(state="disabled")
        self._log("bt_monitor started.  %s opens by itself when the adapter is "
                  "there." % DEFAULT_PORT, "muted")
        self._log("Otherwise pick a port and Connect -- the probe byte asks the "
                  "robot to identify itself.", "muted")

        body = tk.Frame(self.root, bg=BG)
        body.pack(side="top", fill="both", expand=True)
        self.canvas = tk.Canvas(body, bg=CANVAS_BG, highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True, padx=(8, 4), pady=8)

        right = tk.Frame(body, bg=PANEL_BG, width=440)
        right.pack(side="right", fill="y", padx=(4, 8), pady=8)
        right.pack_propagate(False)
        self.right = right

        # THE CARDS SCROLL.  The column is taller than the window: the toolbar
        # and the TERMINAL strip take about 190 px of the 800, so the panel is
        # left with ~580, and the six health cards request ~780 on a capture with
        # findings.  The card that fell off the bottom was CHECKS -- the verdict,
        # the one thing the bench build exists to produce.  So the cards live on
        # a canvas that scrolls; each card keeps its shape and nothing is
        # unreachable.  See _scroll_card for the one card with its own scrollbar
        # on top of this (THRESHOLDS, 19 rows of table).
        self.panel_scroll = tk.Scrollbar(right, orient="vertical", bg=CARD_BG,
                                         troughcolor=PANEL_BG, bd=0,
                                         highlightthickness=0, relief="flat",
                                         activebackground=ACCENT)
        self.panel_canvas = tk.Canvas(right, bg=PANEL_BG, highlightthickness=0,
                                      yscrollcommand=self.panel_scroll.set)
        self.panel_scroll.configure(command=self.panel_canvas.yview)
        self.panel_scroll.pack(side="right", fill="y")
        self.panel_canvas.pack(side="left", fill="both", expand=True)
        holder = tk.Frame(self.panel_canvas, bg=PANEL_BG)
        self._panel_win = self.panel_canvas.create_window((0, 0), window=holder,
                                                          anchor="nw")
        # The inner frame cannot know the canvas width, and the canvas cannot
        # know the frame height, so each is told by the other's <Configure>.
        holder.bind("<Configure>", lambda e: self.panel_canvas.configure(
            scrollregion=self.panel_canvas.bbox("all")))
        self.panel_canvas.bind("<Configure>", lambda e: self.panel_canvas
                               .itemconfigure(self._panel_win, width=e.width))
        self.root.bind_all("<MouseWheel>", self._panel_wheel, add="+")

        # Two panels, only ever one packed at a time.  They are built once and
        # swapped with pack/pack_forget rather than rebuilt per frame: a tick
        # runs every 30 ms and tearing down a hundred widgets at that rate would
        # be felt.
        self.mission_panel = tk.Frame(holder, bg=PANEL_BG)
        self.health_panel = tk.Frame(holder, bg=PANEL_BG)
        self._build_mission_panel(tk)
        self._build_health_panel(tk)
        # `--health` is the operator already saying which build this is, so it
        # pins BOTH switches and switches the auto-follow off.  Without it both
        # start their benign default and follow the first telemetry line.
        self.set_cards("health" if self.args.health else "mission", auto=not
                       self.args.health)
        self.set_canvas("diag" if self.args.health else "path", auto=not
                        self.args.health)

    def _build_mission_panel(self, tk):
        self.status_labels = {}
        card = self._card(self.mission_panel, "MISSION", tk)
        for key in ("phase", "node", "position", "heading", "drift", "reports",
                    "cells", "target_found", "fast_time", "lines", "unparsed",
                    "mismatches", "corner"):
            row = tk.Frame(card, bg=CARD_BG)
            row.pack(fill="x")
            tk.Label(row, text=key, bg=CARD_BG, fg=FG_MUTED, font=FONT,
                     width=13, anchor="w").pack(side="left")
            lbl = tk.Label(row, text="-", bg=CARD_BG, fg=FG, font=FONT_MONO,
                           anchor="w")
            lbl.pack(side="left")
            self.status_labels[key] = lbl

        self.brainin_text = self._text_card(self.mission_panel,
                                            "LAST JUNCTION -> BrainIn", FG, tk)
        self.verdict_text = self._text_card(self.mission_panel, "DECISION",
                                            FG, tk)
        self.sensor_lbls = [self._text_card(self.mission_panel,
                                            "SENSOR BAR  (S lines, 125 Hz)"
                                            if i == 0 else None, FG_MUTED, tk,
                                            mono=("Consolas", 9))
                            for i in range(2)]
        self.plans_text = self._text_card(self.mission_panel, "PLANS (oracle)",
                                          FASTEST_PATH, tk)

    def _build_health_panel(self, tk):
        """The bench cards.  Same information the --health report prints, so
        the operator on the bench and the operator reading a file are looking
        at the same verdict rather than two opinions."""
        # STATE first: on the bench the single most useful number on the wire is
        # loop_start, because everything else is read in the context of it --
        # "is this robot stopped, and is it stopped where I think it is".
        card = self._card(self.health_panel, "STATE  (loop_start)", tk)
        self.state_label = tk.Label(card, text="-", bg=CARD_BG, fg=FG,
                                    font=("Consolas", 11, "bold"), anchor="w",
                                    justify="left")
        self.state_label.pack(fill="x", padx=6, pady=(4, 0))
        self.state_note = tk.Label(card, text="-", bg=CARD_BG, fg=FG_MUTED,
                                   font=("Consolas", 8), anchor="w",
                                   justify="left", wraplength=380)
        self.state_note.pack(fill="x", padx=6, pady=(0, 4))

        card = self._card(self.health_panel, "KEYS", tk)
        row = tk.Frame(card, bg=CARD_BG)
        row.pack(fill="x")
        self.key_lbls = []
        for bit, name, pin in pt.KEY_BITS:
            lbl = tk.Label(row, text="%s\n%s" % (name, pin), bg=CARD_BG,
                           fg=FG_MUTED, font=FONT_BOLD, width=13, pady=6)
            lbl.pack(side="left", padx=2)
            self.key_lbls.append((bit, lbl))
        # The edge log is what makes BOUNCE and a STUCK button visible: a press
        # that never comes up, or a burst of edges too short to be a finger.
        self.key_log = tk.Label(card, text="no key changes yet", bg=CARD_BG,
                                fg=FG_MUTED, font=("Consolas", 9),
                                justify="left", anchor="w")
        self.key_log.pack(fill="x", padx=6, pady=2)

        self.derived_text = self._text_card(self.health_panel,
                                            "DERIVED  (the firmware's own bits)",
                                            FG, tk, mono=("Consolas", 10))
        self.gyro_text = self._text_card(self.health_panel, "GYRO", FG, tk)
        self.thr_text = self._scroll_card(self.health_panel,
                                          "PADS  (IR_mid from the T line, and "
                                          "the bit each one sent)",
                                          FG_MUTED, tk, mono=("Consolas", 9))

        # CHECKS is the one card whose contents change shape, so its widgets are
        # rebuilt only when the finding list actually changes -- colour-coding
        # one line per finding is the whole point of it.
        tk.Label(self.health_panel, text="CHECKS", bg=PANEL_BG, fg=ACCENT,
                 font=FONT_BOLD, anchor="w").pack(fill="x", padx=8, pady=(8, 2))
        self.checks_card = tk.Frame(self.health_panel, bg=CARD_BG)
        self.checks_card.pack(fill="x", padx=8, pady=(0, 4))
        self._checks_sig = None

    def set_canvas(self, mode, auto=False):
        """Which picture the canvas draws: "diag" (the 18-pad board) or "path"
        (the believed map, the trail and the robot's heading)."""
        self.canvas_mode = mode
        self.canvas_auto = auto
        self.btn_canvas.configure(text="Canvas: %s%s"
                                  % (CANVAS_NAME[mode], " (auto)" if auto else ""))

    def _toggle_canvas(self):
        # An explicit click means the operator has decided; stop second-guessing
        # them on the next line that arrives.
        self.set_canvas("path" if self.canvas_mode == "diag" else "diag",
                        auto=False)

    def set_cards(self, cards, auto=False):
        """Which column of cards is beside the canvas: the mission readouts or
        the bench health readouts.  Named `cards` rather than `view` because it
        stopped meaning "the view" the moment the canvas got its own switch."""
        self.card_view = cards
        self.card_auto = auto
        self.mission_panel.pack_forget()
        self.health_panel.pack_forget()
        (self.health_panel if cards == "health"
         else self.mission_panel).pack(fill="both", expand=True)
        self.btn_cards.configure(text="Cards: %s%s"
                                 % (cards, " (auto)" if auto else ""))

    def _toggle_cards(self):
        self.set_cards("mission" if self.card_view == "health" else "health",
                       auto=False)

    # Which build this capture is, said in the two switches' own vocabularies.
    # H/T lines only exist in a health build, J/S/B/Z only while driving, so
    # they name both the useful card column and the useful picture.
    AUTO_VIEW = {"H": "health", "T": "health",
                 "J": "mission", "S": "mission", "B": "mission", "Z": "mission"}
    AUTO_CANVAS = {"health": "diag", "mission": "path"}

    def _auto_view(self, kind):
        """Follow the build, until the operator says otherwise -- per switch.

        A capture is one build or the other, so the first real line tells us
        which cards and which picture are useful, and guessing wrong costs
        nothing but a click.

        Only telemetry moves either switch.  The `Hi ,mmdi` boot banner arrives
        FIRST on every real capture, health or not, so switching on anything
        that parses would make a health capture flash the mission panel before
        settling -- a flicker that reads as a bug.

        The two switches follow INDEPENDENTLY: clicking one stops its own auto
        follow and leaves the other still tracking, so an operator who pins the
        canvas to `path draw` on a health capture keeps the pad board off the
        canvas from then on but still gets the health cards as they arrive.
        """
        want = self.AUTO_VIEW.get(kind)
        if want is None:
            return
        if self.card_auto and want != self.card_view:
            self.set_cards(want, auto=True)
        canvas = self.AUTO_CANVAS[want]
        if self.canvas_auto and canvas != self.canvas_mode:
            self.set_canvas(canvas, auto=True)

    def _card(self, parent, title, tk):
        tk.Label(parent, text=title, bg=PANEL_BG, fg=ACCENT, font=FONT_BOLD,
                 anchor="w").pack(fill="x", padx=8, pady=(8, 2))
        card = tk.Frame(parent, bg=CARD_BG)
        card.pack(fill="x", padx=8, pady=(0, 4))
        return card

    def _text_card(self, parent, title, colour, tk, mono=FONT_MONO, wrap=0):
        if title:
            tk.Label(parent, text=title, bg=PANEL_BG, fg=ACCENT, font=FONT_BOLD,
                     anchor="w").pack(fill="x", padx=8, pady=(8, 2))
        card = tk.Frame(parent, bg=CARD_BG)
        card.pack(fill="x", padx=8, pady=(0, 4))
        lbl = tk.Label(card, text="-", bg=CARD_BG, fg=colour, font=mono,
                       justify="left", anchor="w",
                       wraplength=wrap or 0)
        lbl.pack(fill="x", padx=6, pady=2)
        return lbl

    def _scroll_card(self, parent, title, colour, tk, mono=FONT_MONO, height=10):
        """A card whose text scrolls in place, rather than growing the panel.

        The thresholds table is one line per pad -- 19 of them -- and the right
        panel is a single column of cards in an 800 px window.  Left to grow, it
        pushes CHECKS off the bottom and clips the last pads, which are exactly
        the rows an operator scanning for a dead sensor has not reached yet.  So
        this card has a fixed height and its own scrollbar: everything stays
        reachable, and the live cards above it (STATE, KEYS, GYRO) stay where
        they were instead of the whole panel scrolling to reach a table.
        """
        tk.Label(parent, text=title, bg=PANEL_BG, fg=ACCENT, font=FONT_BOLD,
                 anchor="w").pack(fill="x", padx=8, pady=(8, 2))
        card = tk.Frame(parent, bg=CARD_BG)
        card.pack(fill="x", padx=8, pady=(0, 4))
        box = tk.Text(card, height=height, bg=CARD_BG, fg=colour, font=mono,
                      relief="flat", bd=0, highlightthickness=0, wrap="word",
                      padx=6, pady=2, state="disabled", takefocus=1,
                      insertwidth=0, selectbackground=CARD_BG,
                      selectforeground=colour)
        bar = tk.Scrollbar(card, orient="vertical", command=box.yview,
                           bg=CARD_BG, troughcolor=PANEL_BG, bd=0,
                           highlightthickness=0, relief="flat",
                           activebackground=ACCENT)
        box.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        box.pack(side="left", fill="both", expand=True)
        return box

    def _set_scroll_text(self, box, text, colour):
        """Rewrite a scrolling card, but ONLY when its text really changed.

        This is called from the 30 ms tick.  Re-inserting the same 19 lines every
        frame would fight the operator's scrollbar and reset the view under their
        hand.  T lines arrive about three times a second and stop changing once
        the calibration has settled, so the guard costs nothing and it is what
        makes the card usable at all.  The scroll position is carried across an
        update for the same reason.
        """
        if text == box.get("1.0", "end-1c") and colour == box.cget("fg"):
            return
        at = box.yview()[0]
        box.configure(state="normal")
        box.delete("1.0", "end")
        box.insert("1.0", text)
        box.configure(state="disabled", fg=colour)
        box.yview_moveto(at)

    def _panel_wheel(self, e):
        """The wheel scrolls whatever is under the pointer.

        Two scroll regions meet here: the card column, and the THRESHOLDS table
        inside it, plus the TERMINAL strip which is a Text of its own.  A Text
        scrolls itself (it is the innermost thing the pointer can be over), and
        anything else inside the right-hand panel scrolls the panel.  The
        handler is explicit rather than left to Tk's own wheel bindings, which
        would scroll BOTH the table and the panel under it on one notch.
        """
        w = e.widget
        if isinstance(w, self.tk.Text):
            w.yview_scroll(-3 if e.delta > 0 else 3, "units")
            return "break"
        while w is not None:
            if w is self.right:
                self.panel_canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")
                return "break"
            w = getattr(w, "master", None)
        return None

    # -------------------------------------------------------------- terminal
    def _log(self, text, tag=None):
        """Append one timestamped line to the TERMINAL.

        The Text widget stays `disabled` so a stray click cannot put a caret in
        it and make the log editable; the state is flipped only around this
        insert.  Trimming from the top is what keeps an unattended session from
        growing the widget forever.
        """
        self.term.configure(state="normal")
        self.term.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text),
                         (tag,) if tag else ())
        over = int(self.term.index("end-1c").split(".")[0]) - self.TERM_LINES
        if over > 0:
            self.term.delete("1.0", "%d.0" % (over + 1))
        self.term.see("end")
        self.term.configure(state="disabled")

    def _clear_term(self):
        self.term.configure(state="normal")
        self.term.delete("1.0", "end")
        self.term.configure(state="disabled")

    def _ping(self):
        """Ask the robot to identify itself.

        This is the whole point of the terminal: `Hi ,mmdi` arriving here proves
        the radio is up in BOTH directions.  A silent health bar proves nothing
        on its own -- the robot streams whether or not anything is listening, so
        a mis-paired port looks identical to a robot that is merely holding
        still.  A banner can only be a reply.
        """
        if self.source is None:
            self._log("no link -- connect first, then ping", "warn")
            return
        if self.source.send(PROBE_BYTE):
            self._log("-> probe %r sent, waiting for the banner"
                      % PROBE_BYTE.decode(), "muted")

    # ------------------------------------------------------------- connection
    def refresh_ports(self):
        ports = list_ports()
        self.port_box["values"] = (
            ports or ["(no ports found)"] if HAVE_SERIAL
            else ["(pyserial not installed -- pip install pyserial)"])
        if not ports:
            return
        # The preference is re-applied on EVERY refresh, not just the first one.
        # Windows only creates the Bluetooth SPP port once the link is paired --
        # which is usually AFTER this app was started -- so Refresh is the moment
        # the bench port appears and the selection has to move onto it.  A port
        # the operator picked by hand outranks the preference and is never
        # stolen back; see _port_picked.
        if not self._port_manual:
            self.port_var.set(self._preferred_port(ports))

    def _port_picked(self, _event=None):
        """The operator chose a port by hand, so Refresh leaves it alone from now
        on -- including if they chose a different one than `DEFAULT_PORT`."""
        self._port_manual = True

    def _preferred_port(self, ports):
        """`DEFAULT_PORT` if a port by that name is enumerated, else the first.

        Matched against the ENUMERATED labels rather than trusted as a constant,
        so an unplugged adapter is never pre-selected.  The match is on the port
        NAME only: a different device that happens to be COM9 is selected -- and
        auto-connected, see `_autoconnect` -- exactly as if it were the adapter.
        That is the accepted trade for a no-click bring-up; identifying the
        adapter proper would take a VID/PID test, and this file has no business
        knowing which USB chip is on the bench."""
        for label in ports:
            if port_device(label).upper() == DEFAULT_PORT:
                return label
        return ports[0]

    def _autoconnect(self):
        """Open the bench port at start-up -- the no-click bring-up.

        Silent, not an error, when the adapter is not there: the GUI is also
        started to read a capture or to look at a past one, and a red
        `open failed` on a monitor that was never going to see a robot is noise.
        Not attempted on --replay/--demo at all, where there is no robot and
        opening a COM port would be opening a device for no reason."""
        if self.args.replay or self.args.demo:
            return
        if (self.port_var.get()
                and port_device(self.port_var.get()).upper() == DEFAULT_PORT):
            self.connect(DEFAULT_PORT)

    def _toggle(self):
        if self.source:
            self.disconnect()
        else:
            self.connect(port_device(self.port_var.get()))

    def connect(self, port):
        if not HAVE_SERIAL:
            self.lbl_conn.configure(text="pyserial missing: pip install pyserial",
                                    fg=BAD_COLOR)
            return
        try:
            self.source = SerialSource(port, int(self.baud_var.get()), self.queue)
            self.source.start()
        except Exception as exc:
            self.lbl_conn.configure(text="open failed: %s" % exc, fg=BAD_COLOR)
            self.source = None
            return
        self.btn_conn.configure(text="Disconnect")
        self.lbl_conn.configure(text="connected %s" % port, fg=OK_COLOR)
        self._log("opened %s @ %s" % (port, self.baud_var.get()), "muted")
        # Ask straight away rather than waiting for the operator to press Ping:
        # "every time we connect, the robot says Hi" is the point of the banner,
        # and the connect click is the moment that has to be proved.
        self._ping()
        if self.rec_var.get():
            self.pipe.rec = self.pipe.rec or Recorder()
            self.lbl_conn.configure(
                text="recording -> %s.txt" % os.path.basename(self.pipe.rec.base),
                fg=WARN_COLOR)
            self._log("recording -> %s.txt"
                      % os.path.basename(self.pipe.rec.base), "muted")

    def disconnect(self):
        if self.source:
            self.source.close()
            self.source = None
            self._log("link closed", "muted")
        self.btn_conn.configure(text="Connect")
        self.lbl_conn.configure(text="not connected", fg=FG_MUTED)

    # ---------------------------------------------------------------- replay
    def _load(self, lines):
        self.replay_lines = lines
        self.replay_i = 0

    def _demo_lines(self):
        path = os.path.join(FIXTURES, "capture_agree.txt")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                return [l.rstrip("\r\n") for l in fh
                        if l.strip() and not l.startswith("#")]
        return []

    # ------------------------------------------------------------------ tick
    def _tick(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "line":
                    self._feed(payload)
                elif kind == "error":
                    self.lbl_conn.configure(text="link error: %s" % payload,
                                            fg=BAD_COLOR)
                    self._log("link error: %s" % payload, "bad")
                    self.disconnect()
        except queue.Empty:
            pass

        if self.playing and self.replay_i < len(self.replay_lines):
            self._feed(self.replay_lines[self.replay_i])
            self.replay_i += 1
            if self.replay_i >= len(self.replay_lines):
                self.playing = False

        self.canvas.delete("all")
        self._draw()
        self._update_panels()
        self.root.after(30, self._tick)

    def _feed(self, line):
        kind, res = self.pipe.feed(line)
        if kind == "J" and res is not None:
            self.last_j = res
        elif kind == "S":
            self.last_s = res
        self._auto_view(kind)
        self._log_line(kind, line, res)

    def _log_line(self, kind, line, res):
        """What goes in the terminal, and what deliberately does not.

        The 20 Hz stream is the whole point of the bar above and would bury
        everything else here within a second, so H lines do not appear as
        lines -- except when a KEY changes, which is a fact about the hardware
        and worth its own entry.  T lines are logged only when they change what
        the thresholds ARE: three of them cycle every second and they are
        identical until somebody presses KEY2.

        `kind` is NOT `parse_line`'s verdict -- `Pipeline.feed` re-tags a
        non-record line on the way through, so the handshake arrives here as
        CHATTER.  (`_count` is the one that decides that split.)"""
        if kind == "CHATTER":
            # Three things land here, and they are worth telling apart: the
            # `Hi ,mmdi` handshake (the whole point of the pane -- green), the
            # boot banner at power-on, and any T line that failed to parse.
            text = line.strip()
            if not text:
                return
            if "Hi" in text and "mmd" in text:
                self._log("robot says: %s   <- LINK IS UP" % text, "banner")
            else:
                self._log("robot: %s" % text, "muted")
        elif kind == "BAD":
            self._log("unparsed: %s" % line.strip(), "bad")
        elif kind in ("PLAN", "PLAN-MISMATCH") and res is not None:
            # KEY3 dumps the firmware's own stored plan string.  Comparing it
            # against the oracle's is the check that the copy into
            # path_back/path_discoverd_s did not corrupt it, so a mismatch is
            # the loudest thing this pane can say.
            bad = kind == "PLAN-MISMATCH"
            self._log("%s: %s%s"
                      % ("PLAN MISMATCH" if bad else "plan dump",
                         res[1],
                         "" if bad else "   (matches the oracle)"),
                      "bad" if bad else "muted")
        elif kind == "H" and res is not None:
            keys = res["keys"]
            if self._prev_keys is not None and keys != self._prev_keys:
                for bit, name, _pin in pt.KEY_BITS:
                    if (keys & bit) == (self._prev_keys & bit):
                        continue
                    self._log("  %s %s   (t=%d ms)"
                              % (name, "DOWN" if keys & bit else "up  ",
                                 res["ms"]),
                              "warn" if keys & bit else "muted")
            self._prev_keys = keys
        elif kind == "T" and res is not None:
            sig = tuple(res["val"])
            if self._thr_sig.get(res["what"]) != sig:
                first = res["what"] not in self._thr_sig
                self._thr_sig[res["what"]] = sig
                self._log("thresholds %-3s %s  %s..."
                          % (res["what"],
                             "arrived" if first else "CHANGED to",
                             ",".join(str(v) for v in res["val"][:6])),
                          "muted" if first else "warn")
        elif kind == "Z":
            self._log("target zone: %s" % line.strip(), "warn")

    # ------------------------------------------------------------------ draw
    def _transform(self):
        """Fit the believed map to the canvas.  Brain frame: Y is up."""
        w = self.canvas.winfo_width() or 700
        h = self.canvas.winfo_height() or 700
        pts = list(self.pipe.mission.nodes.values()) or [(0.0, 0.0)]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        pad = 60.0
        span_x = max(1.0, max(xs) - min(xs)) + 2 * CELL_CM
        span_y = max(1.0, max(ys) - min(ys)) + 2 * CELL_CM
        scale = min((w - 2 * pad) / span_x, (h - 2 * pad) / span_y)
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0
        return scale, w / 2.0 - cx * scale, h / 2.0 + cy * scale

    def _w2c(self, x, y, tr):
        scale, ox, oy = tr
        return ox + x * scale, oy - y * scale

    def _draw(self):
        # The canvas mode says WHAT is drawn; which cards are up is set_cards's
        # business and does not reach this function.  That is the whole point of
        # the split -- the health cards can sit beside the map and trail.
        if self.canvas_mode == "diag":
            self._draw_health()
            return
        tk = self.tk
        m = self.pipe.mission
        tr = self._transform()

        if not self.args.no_lattice:
            self._draw_lattice(tr)

        for e in m.edges:
            a, b = tuple(e)
            if a in m.nodes and b in m.nodes:
                self._line(m.nodes[a], m.nodes[b], tr, VISITED_EDGE, 2)

        for node, hs in m.exits.items():
            if node not in m.nodes:
                continue
            x, y = m.nodes[node]
            for hdg in hs:
                dx, dy = HEADING_VEC[hdg]
                self._line((x, y), (x + dx * 0.6 * CELL_CM, y + dy * 0.6 * CELL_CM),
                           tr, STUB_COLOR, 2)

        pts = [m.nodes[j["node"]] for j in m.junctions if j["node"] in m.nodes]
        for a, b in zip(pts, pts[1:]):
            self._line(a, b, tr, TRAIL_COLOR, 7)

        for node, (x, y) in m.nodes.items():
            cx, cy = self._w2c(x, y, tr)
            colour = NODE_COLOR
            if node == m.target_node:
                colour = TARGET_RING
            elif node == 0:
                colour = START_COLOR
            self.canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill=colour,
                                    outline=NODE_OUTLINE)
            self.canvas.create_text(cx + 9, cy - 9, text=str(node), anchor="w",
                                    fill=FG_MUTED, font=("Consolas", 8))

        for j in m.junctions:
            if j["node"] not in m.nodes:
                continue
            cx, cy = self._w2c(*m.nodes[j["node"]], tr)
            if j["oracle_move"] is None:
                tag, colour = "?", WARN_COLOR
            elif j["oracle_move"] == j["ch"]:
                tag, colour = "v", OK_COLOR
            else:
                tag, colour = "X", BAD_COLOR
            self.canvas.create_text(cx, cy, text=tag, fill=colour,
                                    font=("Consolas", 12, "bold"))

        cx, cy = self._w2c(*m.pos, tr)
        dx, dy = HEADING_VEC[m.heading]
        s = 11
        px, py = -dy * s * 0.6, dx * s * 0.6
        self.canvas.create_polygon(
            cx + dx * s, cy - dy * s,
            cx - dx * s * 0.7 + px, cy + dy * s * 0.7 - py,
            cx - dx * s * 0.7 - px, cy + dy * s * 0.7 + py,
            fill=ROBOT_COLOR, outline=NODE_OUTLINE)

        self.canvas.create_text(14, 12, anchor="w", fill=FG_MUTED,
                                font=("Consolas", 9),
                                text="brain frame: origin = start node, "
                                     "up = the robot's initial facing")
        if m.oracle is None:
            # The plot alone proves nothing about correctness; say so, loudly.
            self.canvas.create_text(14, 30, anchor="w", fill=WARN_COLOR,
                                    font=("Consolas", 9),
                                    text="NO ORACLE -- decisions are NOT being "
                                         "checked (%s)"
                                         % (self.pipe.oracle_error or "not built"))

    def _line(self, a, b, tr, colour, width):
        x1, y1 = self._w2c(a[0], a[1], tr)
        x2, y2 = self._w2c(b[0], b[1], tr)
        self.canvas.create_line(x1, y1, x2, y2, fill=colour, width=width,
                                capstyle="round")

    def _draw_lattice(self, tr):
        """A faint 20 cm lattice: the brain snaps to it, so it is the real grid."""
        scale, ox, oy = tr
        w = self.canvas.winfo_width() or 700
        h = self.canvas.winfo_height() or 700
        step = CELL_CM * scale
        if step < 6:
            return
        x = ox % step
        while x < w:
            self.canvas.create_line(x, 0, x, h, fill=GRID_COLOR)
            x += step
        y = oy % step
        while y < h:
            self.canvas.create_line(0, y, w, y, fill=GRID_COLOR)
            y += step

    # ---------------------------------------------------------------- panels
    # ------------------------------------------------------- the health canvas
    #
    # WHAT THIS PICTURE IS FOR.  It is the bench answer to a question the repo
    # cannot answer from a desk: which pads are actually over black, right now,
    # and what would the firmware make of it.  Park the robot on a corner by
    # hand and the centre group and the node gate can be read straight off the
    # screen -- which is the static test for the open `in.front` question
    # (root CLAUDE.md), with no drive and no J line needed.
    #
    # WHAT IT IS NOT.  The roles drawn here are SENSORS.md's, and a pad that
    # tracks white and black perfectly can still be the WRONG pad.  Confirming
    # that needs the robot on the field.  The colour is the bit the firmware
    # itself decided when it sent the line (`IR_ADC[i] <= IR_mid[i]-500`, see
    # main.c's health_send), not a host-side opinion about a raw reading -- the
    # raw reading is not on the wire any more.  The S/J lines remain the
    # authority on what it believed while driving.
    # THE KEY to the strips under the pads.  Nothing on the canvas spells these
    # out any more: the strip colour IS the role, and this tuple is the order
    # they are drawn in -- the same four roles, and the same colours, as the
    # HEALTH card's per-pad rows, which is where the words are.
    ROLE_TAG = ("centre (in.front)", "branch detector", "target detector",
                "node gate (s[0]&&s[9])")
    ROLE_COLOUR = (ACCENT, WARN_COLOR, TARGET_RING, OK_COLOR)

    def _draw_health(self):
        c = self.canvas
        model = self.pipe.health
        w = c.winfo_width() or 700
        if model.last is None:
            c.create_text(w / 2.0, 60, anchor="center", fill=FG_MUTED, font=FONT,
                          text="waiting for the first H line -- the health build "
                               "streams only while the robot is standing still")
            return

        d = model.derived()

        # THE BOARD, drawn as a board: long and narrow, front at the top, every
        # pad at its pt.PAD_CELL position.  Read that table's comment for the
        # shape and for why the shape matters.  The cells are square and sized to
        # fit BOTH the pane's width and its height, so the picture keeps the
        # robot's proportions instead of being stretched to fill the canvas --
        # which is what a row-of-sensors drawing could not do.
        #
        # NOTHING is written in the margins.  Everything that used to be (the
        # target-U caption, the derived summary, the role legend, the caveat) is
        # on the right-hand cards already, or is a duplicate of one.
        # winfo_* is 1 for a widget that has not been laid out yet (the first
        # pass after start-up), and `or` does not catch that, so test it.
        cw = c.winfo_width() if c.winfo_width() > 200 else 700
        chh = c.winfo_height() if c.winfo_height() > 200 else 560
        cell = min((cw - 36.0) / pt.BOARD_COLS, (chh - 28.0) / pt.BOARD_ROWS)
        bw, bh = pt.BOARD_COLS * cell, pt.BOARD_ROWS * cell
        x0, y0 = (cw - bw) / 2.0, 14.0
        # S0 sits just left of the centre line and S9 just right of it, so the
        # rotation axis IS the board's centre line, and that line is drawn.
        axis_x = x0 + pt.BOARD_COLS / 2.0 * cell
        # The pads are a bit larger than their cell -- the one schematic liberty
        # taken in this drawing, so the name and the bit both fit inside.  They
        # are sized from the cell rather than fixed, so they grow with the pane
        # and never collide: the closest two pads are two cells apart, and 1.6
        # cells of pad leaves a visible gap between them.
        pw, ph = cell * 1.6, cell * 1.5

        c.create_rectangle(x0, y0, x0 + bw, y0 + bh, outline="#3a4058", width=2)
        c.create_line(axis_x, y0, axis_x, y0 + bh, fill="#8f97b0", dash=(3, 3))

        def role_tag(i):
            short = pt.sensor_short_role(i)
            return {"centre": 0, "LEFT branch det.": 1, "RIGHT branch det.": 1,
                    "target detector": 2, "node gate": 3}[short]

        def pad_box(i):
            """The pixel box of pad i, and its centre."""
            col, r = pt.PAD_CELL[i]
            cx = x0 + (col + 0.5) * cell
            cy = y0 + (r + 0.5) * cell
            return cx - pw / 2.0, cy - ph / 2.0, cx + pw / 2.0, cy + ph / 2.0

        # The target test, where it applies: the whole U of the LEADING bank has
        # to go black at once (main.c:1266-1267) -- s[1..8] in front, s[10..17]
        # at the back.  A dashed box rather than a caption: the eight pads are
        # either together at the front of the ring or together at the back.
        box = [pad_box(i) for i in (pt.TARGET_ROW if not d["head"]
                                    else pt.REAR_TARGET_ROW)]
        c.create_rectangle(min(b[0] for b in box) - 5, min(b[1] for b in box) - 5,
                           max(b[2] for b in box) + 5, max(b[3] for b in box) + 7,
                           outline=TARGET_RING, dash=(4, 3), width=1)

        for i in range(18):
            x1, y1, x2, y2 = pad_box(i)
            lg = d["logic"][i]
            if lg is None:
                fill, outline, tcol = "#2a2f45", WARN_COLOR, WARN_COLOR
            elif lg:
                # BLACK: near-black fill, so the picture matches the paper.
                fill, outline, tcol = "#0a0b10", "#4a5170", FG
            else:
                fill, outline, tcol = "#c9cee0", "#eef1fa", "#12141c"
            c.create_rectangle(x1, y1, x2, y2, fill=fill, outline=outline,
                               width=2)
            c.create_text((x1 + x2) / 2.0, y1 + ph * 0.28,
                          text=pt.sensor_name(i), fill=tcol,
                          font=("Consolas", 7, "bold"))
            # THE BIT ITSELF, 0 or 1 -- the same number the robot sent, so the
            # board and the wire cannot be read two ways.  A pad shows 1 when it
            # is over black.  (This spot used to hold a raw ADC count; do not put
            # another number here -- 0/1 per sensor is the whole point.)
            c.create_text((x1 + x2) / 2.0, y1 + ph * 0.72,
                          text="?" if lg is None else str(lg),
                          fill=tcol, font=("Consolas", 7, "bold"))
            # A strip under each pad carries its ROLE.  Drawn as a strip rather
            # than as a box around the pad so two overlapping roles (the centre
            # group is also inside the target U) do not fight for the outline.
            c.create_rectangle(x1, y2 + 1, x2, y2 + 4,
                               fill=self.ROLE_COLOUR[role_tag(i)], outline="")

        # That is the whole drawing.  Nothing is written under the board -- not
        # the derived summary, not the role legend, not the loop_start line, and
        # not the "a green strip does not mean the sensors are right" caveat.
        # Every one of them is on a card to the right (`_update_health_panel` and
        # `DERIVED`), and a second copy under the picture was competing with the
        # picture for the same pane.  The pads' own name and ADC stay inside
        # their cells: without those the drawing would say nothing at all.
        #
        # The strips under the cells are the ROLE, and they are the one thing
        # here that needs its key: the key is `ROLE_TAG`/`ROLE_COLOUR` above, and
        # the same four names are spelled out on the HEALTH card's checks.

    # ------------------------------------------------------------- the panels
    def _update_health_panel(self):
        model = self.pipe.health
        last = model.last
        if last is None:
            return

        for bit, lbl in self.key_lbls:
            down = bool(last["keys"] & bit)
            lbl.configure(bg=ACCENT if down else CARD_BG,
                          fg="#0d0f15" if down else FG_MUTED)

        # loop_start, spelled out.  A live H stream only ever arrives in a
        # stopped state, so a state this panel does not expect is worth saying
        # out loud rather than printing a bare number over.
        state = last["loop"]
        stopped = state == 0 or state >= 7
        self.state_label.configure(
            text="%d   %s" % (state, pt.loop_state(state)),
            fg=FG if stopped else WARN_COLOR)
        self.state_note.configure(
            text="head=%d (%s bank leads)   keys=%X%s"
                 % (last["head"], "front" if not last["head"] else "rear",
                    last["keys"],
                    "" if stopped else
                    "\n>> the health stream only exists in a STOPPED state "
                    "(0, or 7 and up) -- seeing %d means either a driving build "
                    "or a state table that is out of date" % state))

        if model.key_edges:
            lines = []
            for e in model.key_edges[-5:]:
                lines.append("t=%-7d %-5s %s%s"
                             % (e["ms"], e["name"],
                                "down" if e["down"] else "up  ",
                                "" if e["down"] else "  held %d ms" % e["held_ms"]))
            self.key_log.configure(text="\n".join(lines), fg=FG)
        else:
            self.key_log.configure(text="no key changes yet -- press one",
                                   fg=FG_MUTED)

        d = model.derived()

        def tri(v):
            return "?" if v is None else str(v)
        if d:
            self.derived_text.configure(
                text="F%s  L%s  R%s   at_node=%s  target=%s\n"
                     "built from the bits the firmware sent, so this IS its\n"
                     "answer -- the S/J lines are the authority while driving"
                     % (tri(d["front"]), tri(d["left"]), tri(d["right"]),
                        tri(d["at_node"]), tri(d["target_row"])),
                fg=FG)
        gyro_ok = model.gz_max is None or model.gz_max <= pt.GYRO_STILL_DPS
        self.gyro_text.configure(
            text="Gyro_Z %6.1f deg/s   (peak |.| %s)\nZ_Angle %5.1f deg\n%s"
                 % (last["gz"], "-" if model.gz_max is None
                    else "%.1f" % model.gz_max, last["za"],
                    "resting: looks calibrated" if gyro_ok else
                    ">> A BIAS, NOT MOTION -- press KEY3 to calibrate"),
            fg=FG if gyro_ok else WARN_COLOR)

        # TWO COLUMNS PER PAD, and no prose: the threshold the bit was decided
        # against, then the bit.  `IR_mid` is per-PAD on purpose -- it comes from
        # the T line the calibration sends once, and it is the only other number
        # the robot puts on the wire, so it belongs beside the pad it belongs to.
        # There is deliberately no `note` column: a word column next to a sensor
        # made the table unreadable, and every verdict it could have carried is
        # already spelled out on the CHECKS card below.
        if model.h:
            known = "mid" in model.thr
            rows = ["  pad     IR_mid   bit"]
            for r in model.rows():
                rows.append("  %-5s %s   %s"
                            % (r["name"],
                               ("%6d" % r["mid"]) if known else "     -",
                               r["now"]))
            self._set_scroll_text(self.thr_text, "\n".join(rows),
                                  FG_MUTED if known else WARN_COLOR)
        else:
            self._set_scroll_text(self.thr_text,
                                  "no H line yet -- no pad bits",
                                  WARN_COLOR)

        findings = model.findings()
        sig = tuple(findings)
        if sig != self._checks_sig:
            # Rebuild only on change: this runs from a 30 ms tick.
            self._checks_sig = sig
            for child in self.checks_card.winfo_children():
                child.destroy()
            colours = {"FAIL": BAD_COLOR, "WARN": WARN_COLOR, "INFO": FG_MUTED}
            if not findings:
                findings = [("INFO", "none -- nothing on this wire looks "
                                     "unhealthy")]
            for sev, text in findings:
                self.tk.Label(self.checks_card, text="[%s] %s" % (sev, text),
                              bg=CARD_BG, fg=colours[sev], font=("Consolas", 8),
                              justify="left", anchor="w", wraplength=380).pack(
                                  fill="x", padx=4, pady=1)

    def _update_panels(self):
        if self.card_view == "health":
            self._update_health_panel()
            return
        m = self.pipe.mission
        c = m.counters
        j = self.last_j
        st = (j or {}).get("oracle") or {}
        vals = {
            "phase": PHASE_NAME.get(st.get("phase"), "-"),
            "node": str(m.node) if m.node is not None else "-",
            "position": "(%.0f, %.0f) cm" % m.pos,
            "heading": HEADING_NAME[m.heading],
            "drift": "%s cm" % st.get("drift_cm", "-"),
            "reports": str(st.get("reports", "-")),
            "cells": str(st.get("cells", "-")),
            "target_found": "yes" if m.target_found else "no",
            "fast_time": ("%.3f s" % st["fast_time_s"]) if st.get("fast_time_s")
            else "-",
            "lines": str(c["lines"] + c["chatter"] + c["unparsed"]),
            "unparsed": str(c["unparsed"]),
            "mismatches": str(c["decision_mismatch"]),
            "corner": str(c["corner_suspect"]),
        }
        for key, lbl in self.status_labels.items():
            alert = (key == "mismatches" and c["decision_mismatch"])
            lbl.configure(text=vals.get(key, "-"), fg=BAD_COLOR if alert else FG)

        if j:
            d = j["derived"]
            self.brainin_text.configure(
                text="F%d L%d R%d  back=1  T%d   dist=%d cm\n"
                     "mask %03X -> F%d L%d R%d      firmware said L%d R%d  %s"
                % (d["front"], d["left"], d["right"], j["target"], j["dist_cm"],
                   j["front"], d["front"], d["left"], d["right"], j["L"], j["R"],
                   "OK" if j["deriv_ok"] else ">> DERIVATION MISMATCH"))
            ok = j["oracle_move"] is None or j["oracle_move"] == j["ch"]
            self.verdict_text.configure(
                text="robot %s   oracle %s   %s\n%s"
                     % (j["ch"], j["oracle_move"] or "n/a",
                        "AGREE" if ok else ">> MISMATCH",
                        " ".join(j["notes"]) or "-"),
                fg=OK_COLOR if ok else BAD_COLOR)

        if self.last_s:
            fr = pt.decode_front(self.last_s["front"])
            rr = pt.decode_rear(self.last_s["rear"])
            self.sensor_lbls[0].configure(
                text="front %s  %s" % (pt.fmt_sensors(fr, 10),
                                       " ".join(b[1] for b in fr) or "-"))
            self.sensor_lbls[1].configure(
                text="rear  %s  %s" % (pt.fmt_sensors(rr, 8),
                                       " ".join(b[1] for b in rr) or "-"))

        if m.plans:
            self.plans_text.configure(text="home %s\nfast %s"
                                      % (m.plans.get("HOME", ""),
                                         m.plans.get("FAST", "")))
        elif self.replay_lines or self.source:
            self.plans_text.configure(text="(not finished yet)")

    def shutdown(self):
        self.disconnect()
        self.pipe.shutdown()


def run_gui(args):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title("SEYED -- Bluetooth mission monitor")
    root.configure(bg=BG)
    root.geometry("1200x800")
    app = MonitorApp(root, tk, ttk, args)
    try:
        root.mainloop()
    finally:
        app.shutdown()
    return 0


# ==========================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="COM port to open (skips the picker; default: "
                                   "%s, opened at start-up when present)"
                                   % DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay", help="replay a captured text file, no hardware")
    ap.add_argument("--demo", action="store_true",
                    help="GUI smoke test on a committed capture")
    ap.add_argument("--headless", action="store_true",
                    help="no GUI; exit 1 on any decision mismatch")
    ap.add_argument("--health", action="store_true",
                    help="bench health check only (needs an H/T capture, i.e. a "
                         "build with USE_MAZE_HEALTH 1 in main.c); exit 1 on any "
                         "FAIL, 2 if the capture has no H lines at all")
    ap.add_argument("--oracle", default=DEFAULT_ORACLE,
                    help="path to brain_oracle.exe")
    ap.add_argument("--no-oracle", action="store_true",
                    help="do not require the oracle")
    ap.add_argument("--no-front-audit", dest="front_audit", action="store_false",
                    default=True)
    ap.add_argument("--no-lattice", action="store_true")
    ap.add_argument("--record", action="store_true",
                    help="record to build/bt_captures/")
    args = ap.parse_args()

    if args.headless:
        return run_headless(args)
    return run_gui(args)


if __name__ == "__main__":
    sys.exit(main())
