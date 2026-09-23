#!/usr/bin/env python3
"""field_to_maze.py -- turn the REAL test field into a simulator maze .json.

A SEYED field is a *line* maze: the black lines are the tracks the robot drives
along, not walls.  The output graph is therefore the line network itself --
nodes are junctions / corners / dead ends, edges are straight runs.

WHY THIS SCRIPT IS NOT A PURE IMAGE PARSER
------------------------------------------
The shipped field image is hand-drawn, so automatic line extraction is
unreliable: most tracks land on the 20 cm lattice but individual elements are
off by 20-30 px (see the box near the right edge).  A parser that "snaps to the
lattice" will silently move those, and a parser that does not will emit a
non-grid maze.  Either way the failure is quiet, which is the worst kind.

So the lattice is transcribed once, by hand, into SEGMENTS below, and the
script's job is to (a) expand it into a unit-edge graph, (b) write the .json,
and (c) draw the reconstruction back over the original image so a human can see
immediately whether it matches.  Check (c) every time the field changes.

Usage:
  python scripts/field_to_maze.py <field.png> [-o ../simulator/mazes/real_field.json]
                                  [--overlay build/imgcrop/field_check.png]

The same .json then runs in both the simulator and the C solver:
  python scripts/run_maze.py ../simulator/mazes/real_field.json
"""

import argparse
import json
import os
import sys

import numpy as np
from PIL import Image

# --------------------------------------------------------------- the lattice
#
# Measured from the image (see SENSORS.md §4): tracks are 6-7 px wide and the
# grid pitch is 62.5 px, and the field is a 20 cm grid -> 3.125 px/cm.
#
#   columns  c0..c7  at image x = 63, 125.5, 188, 250.5, 313, 375.5, 438, 500.5
#   rows     r0..r6  at image y = 32, 94.5, 157, 219, 281, 343.5, 406
#
# r0 is the TOP of the image, so world y runs the other way (Y is up).

PX_PER_CM = 3.125
CELL_CM = 20.0
COL_X0, ROW_Y0 = 63.0, 32.0
N_COLS, N_ROWS = 8, 7

COL_PX = [COL_X0 + 62.5 * i for i in range(N_COLS)]
ROW_PX = [ROW_Y0 + 62.5 * i for i in range(N_ROWS)]


def node_xy(c, r):
    """Lattice (col, row) -> world cm.  Image row 0 is the top, so y is flipped."""
    return (c * CELL_CM, (N_ROWS - 1 - r) * CELL_CM)


def node_px(c, r):
    return (COL_PX[c], ROW_PX[r])


def node_id(c, r):
    return r * N_COLS + c


# ------------------------------------------------------- transcribed network
#
# Straight runs of track, in lattice coordinates.  A run covers every lattice
# point it passes through, so ("H", 1, 2, 5) means a horizontal track on row 1
# from column 2 to column 5 -- three edges, through nodes (2,1), (3,1), (4,1).

H_RUNS = [
    (0, 1, 2),   # top row: short stub right of c1
    (0, 5, 6),   # top row: dead end at c6
    (1, 2, 5),
    (2, 0, 3),   # carries the target disc at c1
    (2, 6, 7),   # top of the right-hand box
    (3, 0, 2),
    (3, 3, 6),   # runs right and meets the box's left edge
    (4, 0, 3),
    (4, 4, 5),
    (4, 6, 7),   # bottom of the right-hand box
    (5, 1, 4),
]

V_RUNS = [
    (0, 2, 4),
    (1, 0, 2),
    (2, 1, 2),
    (2, 3, 4),
    (3, 2, 4),
    (4, 0, 6),   # the spine: a clear 6-cell straight from top to bottom
    (5, 0, 1),
    (6, 2, 4),   # left edge of the right-hand box
    (7, 2, 4),   # right edge of the right-hand box
]

# NOTE -- known open question, not a transcription:
# the verticals at c1, c3 and c4 run off the TOP EDGE of the image (y = 0,
# a full ~30 px above row 0), while no horizontal track exists there.  So
# either the field image is cropped and there is another row above, or those
# three stubs are deliberate.  Until that is settled this graph is the
# *visible* field only.

SOLID_DISC_PX = (123.0, 155.0, 20.0)   # target: centroid x, y, radius
START = (4, 6)                          # bottom of the spine (red arrow
TARGET = (1, 2)                         # points here); disc sits at c1/r2
START_HEADING_EAST = True               # arrow points east -- see report


def build_graph():
    """Expand the runs into unit edges.  Returns (coords, edges) keyed by id."""
    edges = set()

    def link(a, b):
        edges.add((a, b) if a < b else (b, a))

    for r, c0, c1 in H_RUNS:
        for c in range(c0, c1):
            link(node_id(c, r), node_id(c + 1, r))
    for c, r0, r1 in V_RUNS:
        for r in range(r0, r1):
            link(node_id(c, r), node_id(c, r + 1))

    used = {n for e in edges for n in e}
    coords = {n: node_xy(n % N_COLS, n // N_COLS) for n in used}
    return coords, sorted(edges)


def draw_overlay(path, out):
    """Redraw the reconstructed network on top of the original field image."""
    im = Image.open(path).convert("RGB")
    a = np.array(im)

    def dot(x, y, col, k=3):
        xi, yi = int(round(x)), int(round(y))
        a[max(0, yi - k):yi + k + 1, max(0, xi - k):xi + k + 1] = col

    for r, c0, c1 in H_RUNS:
        for c in range(c0, c1 + 1):
            dot(*node_px(c, r), (0, 160, 255))
        y = ROW_PX[r]
        a[int(y) - 1:int(y) + 2, int(COL_PX[c0]):int(COL_PX[c1]) + 1] = (0, 160, 255)
    for c, r0, r1 in V_RUNS:
        for r in range(r0, r1 + 1):
            dot(*node_px(c, r), (0, 160, 255))
        x = COL_PX[c]
        a[int(ROW_PX[r0]):int(ROW_PX[r1]) + 1, int(x) - 1:int(x) + 2] = (0, 160, 255)

    # the transcribed lattice lines, so off-lattice drawing is visible
    for x in COL_PX:
        a[:, int(x)] = np.minimum(a[:, int(x)], (170, 170, 170))
    for y in ROW_PX:
        a[int(y), :] = np.minimum(a[int(y), :], (170, 170, 170))

    cx, cy, rad = SOLID_DISC_PX
    dot(cx, cy, (0, 200, 0), k=int(rad))
    dot(*node_px(*START), (255, 0, 0), k=5)
    dot(*node_px(*TARGET), (0, 200, 0), k=5)

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    Image.fromarray(a).resize((a.shape[1] * 2, a.shape[0] * 2),
                              Image.NEAREST).save(out)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("-o", "--out",
                    default=os.path.join("..", "simulator", "mazes", "real_field.json"))
    ap.add_argument("--overlay", default=os.path.join("build", "imgcrop",
                                                      "field_check.png"))
    args = ap.parse_args()

    coords, edges = build_graph()
    print(f"nodes : {len(coords)}")
    print(f"edges : {len(edges)}")

    ids = sorted(coords)
    remap = {n: i for i, n in enumerate(ids)}
    data = {
        "grid_size": CELL_CM,
        "nodes": {str(remap[n]): list(coords[n]) for n in ids},
        "edges": [[remap[a], remap[b]] for a, b in edges],
        "start": remap[node_id(*START)],
        "target": remap[node_id(*TARGET)],
    }

    # sanity: would the solver be able to reach the target at all?
    adj = {i: [] for i in ids}
    for a, b in edges:
        adj[a].append(b)
        adj[b].append(a)
    seen, stack = {data["start"]}, [data["start"]]
    while stack:
        for m in adj[stack.pop()]:
            if m not in seen:
                seen.add(m)
                stack.append(m)
    deg = {i: len(adj[i]) for i in ids}
    print(f"reachable from start : {len(seen)}/{len(ids)}")
    print(f"start degree={deg[data['start']]}  target degree={deg[data['target']]}")
    print(f"degree-1 (dead ends)  : {sum(1 for d in deg.values() if d == 1)}")

    with open(args.out, "w") as f:
        json.dump(data, f, indent=1)
    print(f"wrote : {args.out}")

    print(f"check : {draw_overlay(args.image, args.overlay)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
