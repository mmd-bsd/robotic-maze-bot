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

  <front> = s[0..9] as hex, bit i = s[i]
  <rear>  = s[10..17] as hex, bit i = s[10+i]
  <raw>   = raw summed-encoder counts for the link, BEFORE the /(2.467*2)
  <cm>    = the firmware's own converted+offset length, for comparison
  <ch>    = the move the BRAIN chose: F/L/R/B, or D when it finished
  <target>= 1 if the target-zone latch (OnEndZoon) was set at this junction
  <dist>  = the distance in cm handed to the brain for this link
  <node>  = the brain's node id for this junction (0..N-1, or 65535 = invalid)

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


def parse(path):
    s_lines, j_lines, z_lines, b_lines, bad = [], [], [], [], []
    with open(path, "r", errors="replace") as fh:
        for lineno, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            f = line.split(",")
            try:
                if f[0] == "S" and len(f) == 9:
                    s_lines.append({
                        "ms": int(f[1]), "front": int(f[2], 16),
                        "rear": int(f[3], 16), "e0": int(f[4]), "e1": int(f[5]),
                        "L": int(f[6]), "R": int(f[7]), "cross": int(f[8]),
                    })
                elif f[0] == "J" and len(f) == 15:
                    j_lines.append({
                        "ms": int(f[1]), "ch": f[2], "nav": int(f[3]),
                        "head": int(f[4]), "raw": int(f[5]), "cm": int(f[6]),
                        "front": int(f[7], 16), "rear": int(f[8], 16),
                        "L": int(f[9]), "R": int(f[10]), "cross": int(f[11]),
                        # Fields 12-14 are the brain's.  `target` is the
                        # OnEndZoon latch, `dist_cm` is the distance the brain
                        # was handed for this link (already in cm, after the
                        # off-by-10-mm simplification described in main.c), and
                        # `node` is the node id the brain settled on.
                        "target": int(f[12]), "dist_cm": int(f[13]),
                        "node": int(f[14]),
                    })
                elif f[0] == "B" and len(f) == 7:
                    # The bring-up line: printed BEFORE the 5 s supervised
                    # pause and flushed ahead of it, so it is the record of
                    # what the brain decided and where it thought it was when
                    # it decided.
                    b_lines.append({
                        "ms": int(f[1]), "node": int(f[2]), "x": int(f[3]),
                        "y": int(f[4]), "drift": int(f[5]), "move": f[6],
                    })
                elif f[0] == "Z" and len(f) == 3:
                    z_lines.append({"ms": int(f[1]), "state": int(f[2])})
                else:
                    bad.append((lineno, line))
            except (ValueError, IndexError):
                bad.append((lineno, line))
    return s_lines, j_lines, z_lines, b_lines, bad


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


def report(args):
    s_lines, j_lines, z_lines, b_lines, bad = parse(args.path)
    print("=" * 74)
    print(" M1 telemetry report -- %s" % args.path)
    print("=" * 74)
    print("  S samples : %d" % len(s_lines))
    print("  J events  : %d" % len(j_lines))
    print("  Z events  : %d" % len(z_lines))
    print("  B events  : %d  (bring-up decisions, supervised build only)"
          % len(b_lines))
    if bad:
        print("  UNPARSED  : %d lines (first: %r)" % (len(bad), bad[0][1][:60]))
        print("              -> wrong firmware build, or a truncated capture?")
    if not s_lines and not j_lines:
        print("\n  Nothing recognisable.  Is this a USE_TELEMETRY capture?")
        return 1

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
        print("  The disc is %.0f mm across, so at v_max = %.0f cm/s the bar is"
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
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", help="captured telemetry text file")
    sys.exit(report(ap.parse_args()))


if __name__ == "__main__":
    main()
