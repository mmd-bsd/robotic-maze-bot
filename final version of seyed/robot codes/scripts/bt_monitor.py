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
  python scripts/bt_monitor.py                      # GUI, pick a port
  python scripts/bt_monitor.py --port COM7          # GUI, connect at once
  python scripts/bt_monitor.py --replay cap.txt     # GUI, replay a capture
  python scripts/bt_monitor.py --replay cap.txt --headless
  python scripts/bt_monitor.py --demo               # GUI smoke test, no hardware

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
]


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
    if pipe.oracle is None and not args.no_oracle:
        print("bt_monitor: %s" % (pipe.oracle_error or "no oracle"))
        return 2

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
            self.connect(port_device(args.port))
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
        self.lbl_conn = tk.Label(bar, text="not connected", bg=PANEL_BG,
                                 fg=FG_MUTED, font=FONT)
        self.lbl_conn.pack(side="left", padx=10)

        body = tk.Frame(self.root, bg=BG)
        body.pack(side="top", fill="both", expand=True)
        self.canvas = tk.Canvas(body, bg=CANVAS_BG, highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True, padx=(8, 4), pady=8)

        right = tk.Frame(body, bg=PANEL_BG, width=440)
        right.pack(side="right", fill="y", padx=(4, 8), pady=8)
        right.pack_propagate(False)

        self.status_labels = {}
        card = self._card(right, "MISSION", tk)
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

        self.brainin_text = self._text_card(right, "LAST JUNCTION -> BrainIn",
                                            FG, tk)
        self.verdict_text = self._text_card(right, "DECISION", FG, tk)
        self.sensor_lbls = [self._text_card(right, "SENSOR BAR  (S lines, 125 Hz)"
                                            if i == 0 else None, FG_MUTED, tk,
                                            mono=("Consolas", 9))
                            for i in range(2)]
        self.plans_text = self._text_card(right, "PLANS (oracle)",
                                          FASTEST_PATH, tk)

    def _card(self, parent, title, tk):
        tk.Label(parent, text=title, bg=PANEL_BG, fg=ACCENT, font=FONT_BOLD,
                 anchor="w").pack(fill="x", padx=8, pady=(8, 2))
        card = tk.Frame(parent, bg=CARD_BG)
        card.pack(fill="x", padx=8, pady=(0, 4))
        return card

    def _text_card(self, parent, title, colour, tk, mono=FONT_MONO):
        if title:
            tk.Label(parent, text=title, bg=PANEL_BG, fg=ACCENT, font=FONT_BOLD,
                     anchor="w").pack(fill="x", padx=8, pady=(8, 2))
        card = tk.Frame(parent, bg=CARD_BG)
        card.pack(fill="x", padx=8, pady=(0, 4))
        lbl = tk.Label(card, text="-", bg=CARD_BG, fg=colour, font=mono,
                       justify="left", anchor="w")
        lbl.pack(fill="x", padx=6, pady=2)
        return lbl

    # ------------------------------------------------------------- connection
    def refresh_ports(self):
        ports = list_ports()
        if not HAVE_SERIAL:
            ports = ["(pyserial not installed -- pip install pyserial)"]
        self.port_box["values"] = ports or ["(no ports found)"]
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

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
        if self.rec_var.get():
            self.pipe.rec = self.pipe.rec or Recorder()
            self.lbl_conn.configure(
                text="recording -> %s.txt" % os.path.basename(self.pipe.rec.base),
                fg=WARN_COLOR)

    def disconnect(self):
        if self.source:
            self.source.close()
            self.source = None
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
    def _update_panels(self):
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
    ap.add_argument("--port", help="COM port to open (skips the picker)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--replay", help="replay a captured text file, no hardware")
    ap.add_argument("--demo", action="store_true",
                    help="GUI smoke test on a committed capture")
    ap.add_argument("--headless", action="store_true",
                    help="no GUI; exit 1 on any decision mismatch")
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
