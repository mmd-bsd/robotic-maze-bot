#!/usr/bin/env python3
"""make_health_fixtures.py -- write the two synthetic health captures.

WHAT THESE ARE
  test/fixtures/capture_health_ok.txt      a healthy bench session   -> exit 0
  test/fixtures/capture_health_fault.txt   the same, two faults      -> exit 1

Both are PARSER AND UI TESTS WITH INVENTED VALUES.  Every ADC count in them
comes from the toy model in this file, not from a robot.  Passing them says the
tools agree on the wire format and reach the verdict they should; it says
NOTHING about any physical sensor.  This is the same honesty rule
capture_agree.txt carries -- a fixture cannot test the question it was
constructed to assume -- and it applies with more force here, because "are the
sensors healthy" is exactly the question a synthesised file cannot answer.

WHY GENERATED RATHER THAN HAND-WRITTEN
So the deliberate fault is reviewable in one place (the SPECS table below) and
the files can be regenerated when the wire format changes, instead of being
hand-edited into a state nobody can justify.

THE FAULTS ARE THE ONLY DIFFERENCE between the two files: one emit() writes
both, from the same schedule, with `pinned_rail` and `thresholds` varying.  A
diff between the fixtures is therefore a diff between the faults, and nothing
else -- which is what makes the pair a real two-sided test rather than two
unrelated files.

Usage:
  python scripts/make_health_fixtures.py
"""

import os
import sys

# Console is cp1252 on this machine; unicode in output crashes it (CLAUDE.md #8).
sys.stdout.reconfigure(encoding="utf-8")

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE_DIR = os.path.normpath(os.path.join(HERE, os.pardir, "test", "fixtures"))

# ==========================================================================
# The model of the sensor bar
# ==========================================================================
#
# POLARITY, and everything below depends on it: the firmware sets s[i]=1
# ("black") when IR_ADC[i] <= IR_mid[i]-50, and s[i]=0 ("white") when
# IR_ADC[i] >= IR_mid[i]+50 (main.c:1960-1967).  So a pad reads HIGH over the
# reflective field and LOW over the line -- and therefore in the calibration
# IR_max is the WHITE end and IR_min the BLACK end, the opposite way round from
# how those names read.
WHITE = 3180          # a pad over the reflective field (high)
BLACK = 165           # a pad over the line or the target disc (low)
JITTER = 12           # sample-to-sample wobble, so `changes` is not trivially 0

# The schedule the two captures share: a card slid under the bar from the left
# half to the right half, so the centre group, both branch detectors and the
# target pads all see both surfaces.  (t0, t1, pads over black), with t1 = None
# meaning "to the end of the capture" -- the LAST phase is the one still in
# force at the final sample, and the final sample is what the report's SENSOR
# BAR and DERIVED blocks draw, so it has to be the interesting pose.
SCENE = (
    (0,    700,  ()),
    (700,  1300, (1, 2, 3, 4)),        # under the LEFT half of the inner row
    (1300, None, (5, 6, 7, 8)),        # slid across to the RIGHT half
)

# (down_ms, up_ms, bit) -- KEY1 and KEY2 overlap on purpose: the <keys> field is
# a bitmask, and two buttons down at once is the case a naive reader gets wrong.
KEYS = ((600, 1000, 1), (850, 1050, 2), (1500, 1750, 4))

PERIOD_MS = 50        # 20 Hz, the rate the firmware paces the H line at
END_MS = 2000
T_EVERY = 7           # every 7th slot is a T line, as health_send() does

# main.c:90, literally: the power-on defaults.  A robot that has never been
# calibrated holds THESE in IR_mid[], with IR_min/IR_max still zero.
POWER_ON_MID = (1400, 1200, 1400, 1400, 1400, 1400, 1400, 1400, 1400,
                1400, 1400, 1400, 1400, 1400, 1400, 1400, 1400, 1400)

# ==========================================================================
# The two captures
# ==========================================================================
SPECS = (
    {
        "file": "capture_health_ok.txt",
        "what": "a healthy bench session, captured AFTER a run",
        "verdict": "expects exit 0 -- no FAIL",
        "loop": 7,               # 7 = stopped after the fast run, so the
        "pinned_rail": (),       #   calibration below is one that really ran
        "thresholds": "calibrated",
        "gz_tenths": (3, 7),     # a resting gyro: 0.3 - 0.7 deg/s
        "notes": (
            "Keys cycle (KEY1 and KEY2 overlap, to exercise the bitmask), the",
            "card slides from the left half of the inner row to the right half,",
            "and every pad was calibrated against both surfaces.",
        ),
    },
    {
        "file": "capture_health_fault.txt",
        "what": "the SAME session with two faults planted",
        "verdict": "expects exit 1 -- FAIL, naming S7 and the missing calibration",
        "loop": 0,               # 0 = the boot loop, before KEY1 -- which is
        "pinned_rail": (7,),     #   exactly when nothing is calibrated yet
        "thresholds": "uncalibrated",
        "gz_tenths": (120, 142),  # an uncalibrated gyro: ~12-14 deg/s of bias
        "notes": (
            "FAULT 1 -- S7 (the right branch detector) is held at the 4095 rail",
            "for the whole capture: it never moves, so it is saturated or dead,",
            "and it would read WHITE whatever it is over.  That is the branch",
            "detector the graph needs to see a right-hand lane, so it is a",
            "missing-edge fault, not a cosmetic one.",
            "",
            "FAULT 2 -- IR_min/IR_max are still zero, i.e. no calibration has",
            "run, so IR_mid is the power-on default from main.c:90 rather than",
            "anything this robot measured.  Every black/white call in the file",
            "is therefore against a guessed threshold.",
            "",
            "The gyro bias (~13 deg/s standing still) is a CONSEQUENCE of the",
            "same power-on state, not a third planted fault: it is what an",
            "uncalibrated LSM6DS3TR reads, and it is the warning that tells the",
            "operator to press KEY3.",
        ),
    },
)


class LCG:
    """A tiny explicit generator, deliberately not `random`.

    The fixtures must regenerate byte-identically on any Python and any
    platform, so they cannot depend on a library whose stream is only stable by
    convention.  This is the same reasoning the rest of the repo applies to
    preferring the standard library over a dependency: own the whole thing.
    """

    def __init__(self, seed):
        self.s = seed & 0x7FFFFFFF

    def span(self, half):
        """A pseudo-random offset in [-half, +half]."""
        self.s = (1103515245 * self.s + 12345) & 0x7FFFFFFF
        return (self.s % (2 * half + 1)) - half


def over_black(i, ms):
    for t0, t1, pads in SCENE:
        if ms >= t0 and (t1 is None or ms < t1):
            return i in pads
    return False


def calibrated_ends():
    """Per-pad (black_end, white_end) from a calibration that saw both.

    Spread across the bar rather than identical, so the file exercises the
    per-channel columns instead of eighteen copies of one number -- a real bar
    has eighteen different LEDs and eighteen different phototransistors.
    """
    return [(140 + 3 * i, 3120 + 7 * i) for i in range(18)]


def build(spec):
    """-> the file as a list of lines."""
    rng = LCG(0x5E7ED)                    # fixed: same bytes every run
    cal = calibrated_ends()
    if spec["thresholds"] == "calibrated":
        mid = [((b + w) // 2) for b, w in cal]          # (IR_max+IR_min)/2
        lo = [b for b, _w in cal]                       # IR_min = the BLACK end
        hi = [w for _b, w in cal]                       # IR_max = the WHITE end
        # A pad sits just inside the range its calibration found, not exactly
        # on the ends: the ends are the extremes of a spin, not the resting
        # reading, and a fixture that put them on the end would hide an
        # off-by-a-little bug in the comparison.
        base_b = [b + 20 for b, _w in cal]
        base_w = [w - 40 for _b, w in cal]
    else:
        mid = list(POWER_ON_MID)                        # never calibrated
        lo = [0] * 18
        hi = [0] * 18
        base_b = [BLACK + 5 * (i % 3) for i in range(18)]
        base_w = [WHITE + 6 * (i % 5) for i in range(18)]

    keys_at = {}
    for t0, t1, bit in KEYS:
        for ms in range(t0, t1, PERIOD_MS):
            keys_at[ms] = keys_at.get(ms, 0) | bit

    gz0, gz1 = spec["gz_tenths"]
    times = list(range(0, END_MS + 1, PERIOD_MS))

    out = []
    slot = 0
    for k, ms in enumerate(times):
        keys = keys_at.get(ms, 0)
        # KEY3 is the one that starts a calibration, so the gyro settles after
        # it in the healthy file.  Belt and braces: it also makes the fixture
        # wire-consistent with the warning the model raises.
        gz = gz0 + (gz1 - gz0) * k // max(1, len(times) - 1)
        za = (gz0 * ms) // 1000
        adc = []
        for i in range(18):
            if i in spec["pinned_rail"]:
                adc.append(4095)                        # FAULT: never jitters
                continue
            v = base_b[i] if over_black(i, ms) else base_w[i]
            adc.append(max(0, min(4095, v + rng.span(JITTER))))
        out.append("H,%d,%X,%d,%d,%d,%d,%s"
                   % (ms, keys, spec["loop"], 0, gz, za,
                      ",".join(str(v) for v in adc)))

        if k % T_EVERY == T_EVERY - 1:
            which = (slot // T_EVERY) % 3
            name, vals = (("mid", mid), ("min", lo),
                          ("max", hi))[which]
            out.append("T,%d,%s,%s" % (ms, name, ",".join(str(v) for v in vals)))
        slot += 1

    return out


def header(spec, generated_by):
    """The '#' block, which parse_line() treats as a comment, not a record."""
    lines = [
        "# Synthetic health capture for TESTING THE TOOLS -- not a robot.",
        "#",
        "# %s: %s." % (spec["file"], spec["what"]),
        "# %s" % spec["verdict"],
        "#",
        "# INVENTED VALUES.  Every ADC count, threshold and gyro reading below",
        "# was produced by scripts/make_health_fixtures.py from a toy model of",
        "# the bar -- it is not a recording of anything.  It tests that",
        "# parse_telemetry.py and bt_monitor.py agree on the H/T wire format and",
        "# reach the verdict they should.  It is NOT evidence about any physical",
        "# sensor, and passing it says nothing about hardware.  The sensor that",
        "# matters can only be checked by watching the real robot.",
        "#",
    ]
    lines += ["# " + n if n else "#" for n in spec["notes"]]
    lines += [
        "#",
        "# Generated %s by scripts/make_health_fixtures.py -- REGENERATE rather"
        % generated_by,
        "# than editing by hand, so the faults stay reviewable in one place.",
        "#",
        "Hi ,mmdi",
    ]
    return lines


def main():
    # Import the parser rather than re-deriving the format here: if the wire
    # format changes, feeding the generated file back through parse_line() is
    # what catches the fixtures going stale, and this script is the only place
    # that can do it without a human remembering to.
    sys.path.insert(0, HERE)
    import parse_telemetry as pt

    today = "2026-09-23"
    written = []
    for spec in SPECS:
        body = build(spec)
        lines = header(spec, today) + body

        # Self-check, on the way out.  A fixture that does not parse is worse
        # than no fixture: build_all.ps1 would report the tool as broken and the
        # real H line as suspect.  Exactly one BANNER is expected -- the
        # firmware's `Hi ,mmdi` boot banner, which is benign and deliberate.
        kinds = {}
        for n, line in enumerate(lines, 1):
            kind, rec = pt.parse_line(line, n)
            kinds[kind] = kinds.get(kind, 0) + 1
        if kinds.get("BAD") or kinds.get("BANNER", 0) != 1:
            print("  REFUSING to write %s: it does not parse cleanly (%s)"
                  % (spec["file"], kinds))
            return 1

        model = pt.build_health_model({"H": [], "T": []})
        for line in lines:
            kind, rec = pt.parse_line(line)
            if kind in ("H", "T"):
                model.feed(kind, rec)
        worst = model.worst()
        want = "FAIL" if spec["pinned_rail"] else "OK"
        if worst != want:
            print("  REFUSING to write %s: verdict is %s, expected %s"
                  % (spec["file"], worst, want))
            for sev, text in model.findings():
                print("      [%s] %s" % (sev, text))
            return 1

        path = os.path.join(FIXTURE_DIR, spec["file"])
        with open(path, "w", encoding="ascii", newline="\n") as fh:
            fh.write("\n".join(lines) + "\n")
        written.append((spec, path, kinds, model))

    for spec, path, kinds, model in written:
        rel = os.path.relpath(path, os.path.dirname(FIXTURE_DIR))
        print("  wrote %s  (%d H, %d T, verdict %s)"
              % (rel, kinds.get("H", 0), kinds.get("T", 0), model.worst()))
        for sev, text in model.findings():
            print("      [%-4s] %s" % (sev, text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
