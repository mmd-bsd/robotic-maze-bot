"""
Maze Solver (Final Version - SEYED)
===================================

Loads a maze built by the Maze Generator (.json) and animates the robot's full
mission, step by step, in a modern dark-mode UI.

Phases
------
  1. EXPLORE   - hybrid frontier exploration (see "Algorithm" below). After the
                 target is physically found, a branch-and-bound proof checks each
                 step whether any UNDISCOVERED path could still be shorter; the
                 moment that becomes impossible the robot stops early (even with
                 an incomplete map). The GUI "Proof" flag shows this state.
  2. RETURN    - when no unexplored paths remain, the robot drives back to start
                 (this is part of the exploration logic).
  3. FAST RUN  - the fastest (minimum-TIME) start -> target path is computed and
                 the robot drives it with a real acceleration profile: constant
                 cautious speed while exploring, but on the fast run it speeds up
                 on clear straights and stops (v = 0) at every turn. Alternative
                 slower routes are shown lighter; the driven path is coloured by
                 speed (red = fast, green = slow).

For every move the solver also produces a robot command:
      F = forward, L = turn left, R = turn right, B = U-turn (go back)
from a tracked heading (N/E/S/W) - exactly what the real robot needs.

Algorithm (improved vs. sim_V12_GBF_stable)
-------------------------------------------
sim_V12 used a frontier search with two weaknesses:
  * the "nearest" frontier was chosen by hop count (BFS), not real travel cost;
  * when several unexplored paths existed it just took the first in the list,
    even though the file was named "Greedy Best-First".

This version keeps V12's guarantee of complete mapping (required so the fastest
path is provably optimal) but improves the decisions WITHOUT cheating - during
the first search the robot does NOT know where the target is, so no decision may
use the target's location:
  * Priority 1 - among unexplored branches at the current node, prefer to keep
    going straight, then left, then right, then reverse (fewest turns for a
    line-following robot). This is target-agnostic.
  * Priority 2 - choose the nearest frontier by real travel DISTANCE (Dijkstra
    on the KNOWN map only), not hop count, reusing one Dijkstra sweep per step.
The target is discovered only when the robot physically reaches it. The robot's
total travelled distance is tracked and shown.

The "fastest path" is the minimum-TIME route under an acceleration model (stop at
turns, accelerate on straights), computed on the discovered map; v_max and a_max
are adjustable in the GUI. The exploration early-stop proof is TIME-based with the
same v_max, so it never prunes a straight branch that could be faster (see
ALGORITHMS.md sec. 2).

Requires only the Python standard library (tkinter); Pillow optional for
anti-aliased circles.
"""

import colorsys
import heapq
import json
import math
import os
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import filedialog, messagebox

try:
    from PIL import Image, ImageDraw, ImageTk
    _HAS_PIL = True
except Exception:  # noqa: BLE001
    _HAS_PIL = False

# ----------------------------- Configuration ------------------------------- #
NODE_RADIUS = 7
STUB_LEN = 5.0

# --- Motion model (acceleration-aware fast run) ---------------------------- #
# REAL-WORLD UNITS: 1 world unit == 1 cm (the maze grid spacing is 20 -> 20 cm).
# Speeds are therefore cm/s and accelerations cm/s^2.
# The explore phase drives at a cautious CONSTANT speed (unknown territory).
# The fast run accelerates on clear straights and STOPS (v = 0) at every turn,
# following a trapezoidal velocity profile bounded by v_max and a_max.
DEFAULT_VMAX = 100.0        # robot top speed during the fast run (cm/s)
DEFAULT_ACCEL = 50.0        # robot max (de)acceleration during the fast run (cm/s^2)
EXPLORE_SPEED = 40.0        # cautious constant speed used while searching (cm/s)
FRAME_MS = 30               # real milliseconds between animation frames (~33 fps)
TRAJ_DT = 0.012             # sampling step for the precomputed fast-run trajectory
GRADIENT_SEG = 4.0          # world-units (cm) per coloured sub-segment on the path
N_ALT_PATHS = 6             # how many alternative (slower) paths to show

# Playback presets: simulated-time / real-time ratio. 1x == true real time.
PLAYBACK_VALUES = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]
DEFAULT_PLAYBACK_INDEX = 2  # -> 1x

# ------------------------------ Dark theme --------------------------------- #
BG = "#12141c"
PANEL_BG = "#171a24"
CARD_BG = "#1d2130"
CANVAS_BG = "#0d0f15"
FG = "#e6e8f0"
FG_MUTED = "#878da3"
ACCENT = "#5b8cff"
ACCENT2 = "#2ee6a6"
GRID_COLOR = "#1a1e2c"
AXIS_COLOR = "#2b3146"
UNEXPLORED_EDGE = "#5b6488"
VISITED_EDGE = "#5b8cff"
OPTIMAL_EDGE = "#2ee6a6"
# Fastest path: bold + dark green.  Alternative (slower) paths: lighter greens.
FASTEST_PATH = "#06d26a"
FASTEST_PATH_DARK = "#0a8f47"
ALT_PATH_DARK = "#2f9c63"     # lightest-shade anchor for the *fastest* alternative
ALT_PATH_LIGHT = "#cdeed9"    # lightest-shade anchor for the *slowest* alternative
STUB_COLOR = "#ffd23f"
NODE_COLOR = "#aab2d6"
NODE_OUTLINE = "#0d0f15"
START_COLOR = "#3ddc84"
TARGET_FILL = "#0a0a0a"
TARGET_RING = "#ff5d6c"
ROBOT_COLOR = "#ff5d6c"

FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 10, "bold")
FONT_TITLE = ("Segoe UI", 15, "bold")
FONT_MONO = ("Consolas", 10)
FONT_MONO_S = ("Consolas", 9)

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MAZES_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "mazes"))

NORTH, SOUTH, EAST, WEST = (0, 1), (0, -1), (1, 0), (-1, 0)
INF = float("inf")

# Exploration preference: straight is best, then left, then right, then U-turn.
TURN_RANK = {"F": 0, "L": 1, "R": 2, "B": 3}
DIR_INDEX = {NORTH: 0, EAST: 1, SOUTH: 2, WEST: 3}


# =========================================================================== #
#  Graph (pure standard library)
# =========================================================================== #
class Maze:
    def __init__(self, data):
        self.grid_size = data.get("grid_size", 20)
        self.pos = {int(k): tuple(v) for k, v in data["nodes"].items()}
        self.adj = {n: set() for n in self.pos}
        for a, b in data["edges"]:
            self.adj[a].add(b)
            self.adj[b].add(a)
        self.start = data.get("start")
        self.target = data.get("target")
        if self.start is None or self.target is None:
            raise ValueError("Maze must define both a start and a target.")

    def neighbors(self, node):
        return self.adj.get(node, set())

    def dist(self, a, b):
        (x1, y1), (x2, y2) = self.pos[a], self.pos[b]
        return math.hypot(x2 - x1, y2 - y1)


def dijkstra_fields(maze, adj, src):
    """Single-source shortest distances (by geometry) over adjacency `adj`."""
    dist = {src: 0.0}
    prev = {src: None}
    pq = [(0.0, src)]
    while pq:
        d, cur = heapq.heappop(pq)
        if d > dist.get(cur, INF):
            continue
        for nb in adj.get(cur, ()):
            nd = d + maze.dist(cur, nb)
            if nd < dist.get(nb, INF):
                dist[nb] = nd
                prev[nb] = cur
                heapq.heappush(pq, (nd, nb))
    return dist, prev


def path_from_prev(prev, dst):
    if dst not in prev:
        return None
    path, cur = [], dst
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    return path[::-1]


def dijkstra_path(maze, src, dst):
    _, prev = dijkstra_fields(maze, maze.adj, src)
    return path_from_prev(prev, dst)


def heading_name(h):
    return {NORTH: "N", SOUTH: "S", EAST: "E", WEST: "W"}.get(h, "?")


def turn_command(prev_h, new_h):
    if prev_h is None or new_h == prev_h:
        return "F"
    if new_h[0] == -prev_h[0] and new_h[1] == -prev_h[1]:
        return "B"
    cross = prev_h[0] * new_h[1] - prev_h[1] * new_h[0]
    return "L" if cross > 0 else "R"


def direction(maze, a, b):
    """Unit-ish axis direction (sign of the delta) from node a to node b."""
    (x1, y1), (x2, y2) = maze.pos[a], maze.pos[b]
    return ((x2 > x1) - (x2 < x1), (y2 > y1) - (y2 < y1))


# =========================================================================== #
#  Motion model: trapezoidal velocity profile (v_max / a_max, 0 at every turn)
# =========================================================================== #
def run_time(length, vmax, accel):
    """Time to drive a straight run of `length`, starting and ending at v = 0.

    Trapezoidal profile: accelerate at `accel` up to (at most) `vmax`, optionally
    cruise, then decelerate to 0. If the run is too short to reach `vmax` the
    profile is a symmetric triangle.
    """
    if length <= 0:
        return 0.0
    d_acc = vmax * vmax / (2.0 * accel)           # distance to reach vmax
    if 2.0 * d_acc <= length:                     # trapezoid (reaches vmax)
        t_ramp = vmax / accel
        cruise = (length - 2.0 * d_acc) / vmax
        return 2.0 * t_ramp + cruise
    v_peak = math.sqrt(accel * length)            # triangle (never reaches vmax)
    return 2.0 * v_peak / accel


def speed_at(s, length, vmax, accel):
    """Speed at arc-distance `s` into a run of `length` (entry/exit speed = 0)."""
    if length <= 0:
        return 0.0
    s = max(0.0, min(length, s))
    v_acc = math.sqrt(2.0 * accel * s)
    v_dec = math.sqrt(2.0 * accel * (length - s))
    return min(vmax, v_acc, v_dec)


def stop_indices(maze, path):
    """Indices in `path` where the robot must be at v = 0.

    Those are the two endpoints plus every interior node where the heading
    changes (an L/R/B turn). Straight-through nodes (F) are *not* stops.
    """
    if not path:
        return set()
    stops = {0, len(path) - 1}
    for i in range(1, len(path) - 1):
        d_in = direction(maze, path[i - 1], path[i])
        d_out = direction(maze, path[i], path[i + 1])
        if d_in != d_out:
            stops.add(i)
    return stops


def path_time(maze, path, vmax, accel):
    """Total drive time for a full node `path` under the motion model."""
    if not path or len(path) < 2:
        return 0.0
    stops = stop_indices(maze, path)
    total = 0.0
    run_len = 0.0
    for i in range(1, len(path)):
        run_len += maze.dist(path[i - 1], path[i])
        if i in stops:                # end of a straight run -> account for it
            total += run_time(run_len, vmax, accel)
            run_len = 0.0
    return total


# =========================================================================== #
#  Path search on the DISCOVERED map
# =========================================================================== #
def _straight_chain(maze, adj, u, d):
    """Ordered nodes reachable from u by driving straight in direction d."""
    chain = []
    cur = u
    while True:
        nxt = None
        for nb in adj.get(cur, ()):
            if direction(maze, cur, nb) == d:
                nxt = nb
                break
        if nxt is None:
            break
        chain.append(nxt)
        cur = nxt
    return chain


def time_optimal_path(maze, adj, start, target, vmax, accel):
    """Fastest (minimum-TIME) start->target path over the discovered graph.

    The robot stops (v = 0) at every turn and accelerates on straights, so a
    longer-but-straighter route can beat a shorter twisty one. We search a
    "stop graph" whose edges are maximal straight runs (any length) between
    nodes where the robot turns; each edge costs `run_time(length)`. Because the
    robot is stopped at every node of this graph, the state is just the node and
    a plain Dijkstra is exact.
    """
    if start == target:
        return [start], 0.0
    best = {start: 0.0}
    prev = {start: None}
    pq = [(0.0, start)]
    while pq:
        t, u = heapq.heappop(pq)
        if t > best.get(u, INF):
            continue
        if u == target:
            break
        for d in (NORTH, EAST, SOUTH, WEST):
            run_len = 0.0
            prev_node = u
            for w in _straight_chain(maze, adj, u, d):
                run_len += maze.dist(prev_node, w)
                prev_node = w
                nt = t + run_time(run_len, vmax, accel)
                if nt < best.get(w, INF) - 1e-12:
                    best[w] = nt
                    prev[w] = u
                    heapq.heappush(pq, (nt, w))
    if target not in prev:
        return None, INF
    # Reconstruct the turn-point path, then expand runs into every node.
    turns = path_from_prev(prev, target)
    full = [turns[0]]
    for a, b in zip(turns, turns[1:]):
        d = direction(maze, a, b)
        cur = a
        while cur != b:
            nb = next(n for n in adj.get(cur, ())
                      if direction(maze, cur, n) == d)
            full.append(nb)
            cur = nb
    return full, best.get(target, INF)


def _dijkstra_constrained(maze, adj, src, dst, banned_nodes, banned_edges):
    dist = {src: 0.0}
    prev = {src: None}
    pq = [(0.0, src)]
    while pq:
        d, cur = heapq.heappop(pq)
        if d > dist.get(cur, INF):
            continue
        if cur == dst:
            break
        for nb in adj.get(cur, ()):
            if nb in banned_nodes:
                continue
            if tuple(sorted((cur, nb))) in banned_edges:
                continue
            nd = d + maze.dist(cur, nb)
            if nd < dist.get(nb, INF):
                dist[nb] = nd
                prev[nb] = cur
                heapq.heappush(pq, (nd, nb))
    path = path_from_prev(prev, dst)
    if path is None:
        return None, INF
    return path, dist.get(dst, INF)


def yen_k_shortest(maze, adj, src, dst, k):
    """K shortest loop-less paths by DISTANCE (Yen's algorithm).

    Used only to surface a few *alternative* routes for display; the headline
    "fastest" route is chosen by TIME (`time_optimal_path`).
    """
    first, _ = _dijkstra_constrained(maze, adj, src, dst, set(), set())
    if first is None:
        return []
    A = [first]
    B = []
    while len(A) < k:
        prev_path = A[-1]
        for i in range(len(prev_path) - 1):
            spur = prev_path[i]
            root = prev_path[:i + 1]
            banned_edges = set()
            for p in A:
                if len(p) > i and p[:i + 1] == root:
                    banned_edges.add(tuple(sorted((p[i], p[i + 1]))))
            banned_nodes = set(root[:-1])
            spur_path, _ = _dijkstra_constrained(
                maze, adj, spur, dst, banned_nodes, banned_edges)
            if spur_path is None:
                continue
            cand = root[:-1] + spur_path
            if cand not in A and cand not in [b[1] for b in B]:
                cost = sum(maze.dist(a, b) for a, b in zip(cand, cand[1:]))
                heapq.heappush(B, (cost, cand))
        if not B:
            break
        A.append(heapq.heappop(B)[1])
    return A


# =========================================================================== #
#  Colour helpers (speed gradient + green path shades)
# =========================================================================== #
def _hex(r, g, b):
    return "#%02x%02x%02x" % (int(r * 255), int(g * 255), int(b * 255))


def speed_to_color(v, vmax):
    """Map a speed to a colour: slow = green, fast = red (smooth gradient)."""
    t = 0.0 if vmax <= 0 else max(0.0, min(1.0, v / vmax))
    hue = (1.0 - t) * (120.0 / 360.0)   # 120deg (green) -> 0deg (red)
    r, g, b = colorsys.hsv_to_rgb(hue, 0.92, 0.98)
    return _hex(r, g, b)


def _lerp_hex(c1, c2, t):
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def alt_path_color(rank, n):
    """Lighter green the slower the alternative (rank 0 = fastest alternative)."""
    if n <= 1:
        return ALT_PATH_DARK
    return _lerp_hex(ALT_PATH_DARK, ALT_PATH_LIGHT, rank / (n - 1))


# =========================================================================== #
#  Fast-run plan: time-parameterised trajectory along the fastest path
# =========================================================================== #
class FastRunPlan:
    """Pre-computes position/speed as a function of time along a node path."""

    def __init__(self, maze, path, vmax, accel):
        self.maze = maze
        self.path = path
        self.vmax = vmax
        self.accel = accel
        # Per-node cumulative arc length.
        self.node_arc = [0.0]
        for a, b in zip(path, path[1:]):
            self.node_arc.append(self.node_arc[-1] + maze.dist(a, b))
        self.total_len = self.node_arc[-1]
        # Straight runs (arc-length intervals between stops).
        stops = sorted(stop_indices(maze, path))
        self.runs = []      # (s_start, s_end)
        for a, b in zip(stops, stops[1:]):
            self.runs.append((self.node_arc[a], self.node_arc[b]))
        # Sample time -> arc length, time -> speed.
        self.t_samples = [0.0]
        self.s_samples = [0.0]
        self.v_samples = [0.0]
        t = 0.0
        for s0, s1 in self.runs:
            length = s1 - s0
            rt = run_time(length, vmax, accel)
            n = max(2, int(math.ceil(rt / TRAJ_DT)))
            for j in range(1, n + 1):
                tau = rt * j / n
                s = self._arc_in_run(tau, length)
                t += rt / n
                self.t_samples.append(t)
                self.s_samples.append(s0 + s)
                self.v_samples.append(speed_at(s, length, vmax, accel))
        self.total_time = self.t_samples[-1]

    def _arc_in_run(self, tau, length):
        """Arc distance covered `tau` seconds into a run of `length`."""
        a, vmax = self.accel, self.vmax
        d_acc = vmax * vmax / (2.0 * a)
        if 2.0 * d_acc <= length:                     # trapezoid
            t_ramp = vmax / a
            cruise_t = (length - 2.0 * d_acc) / vmax
            if tau <= t_ramp:
                return 0.5 * a * tau * tau
            if tau <= t_ramp + cruise_t:
                return d_acc + vmax * (tau - t_ramp)
            td = tau - t_ramp - cruise_t
            return d_acc + (length - 2.0 * d_acc) + vmax * td - 0.5 * a * td * td
        v_peak = math.sqrt(a * length)                # triangle
        t_ramp = v_peak / a
        if tau <= t_ramp:
            return 0.5 * a * tau * tau
        td = tau - t_ramp
        return length / 2.0 + v_peak * td - 0.5 * a * td * td

    def sample(self, t):
        """Return (world_pos, heading, speed, arc_s) at sim time `t`."""
        t = max(0.0, min(self.total_time, t))
        # Locate the time bracket.
        lo, hi = 0, len(self.t_samples) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.t_samples[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        t0, t1 = self.t_samples[i - 1], self.t_samples[i]
        frac = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        s = self.s_samples[i - 1] + frac * (self.s_samples[i] - self.s_samples[i - 1])
        v = self.v_samples[i - 1] + frac * (self.v_samples[i] - self.v_samples[i - 1])
        return (*self.pos_at_arc(s), self._heading_at_arc(s), v, s)

    def time_at_arc(self, s):
        """Sim time at which the robot reaches arc length `s`."""
        s = max(0.0, min(self.total_len, s))
        lo, hi = 0, len(self.s_samples) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.s_samples[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        s0, s1 = self.s_samples[i - 1], self.s_samples[i]
        frac = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        return self.t_samples[i - 1] + frac * (self.t_samples[i] - self.t_samples[i - 1])

    def pos_at_arc(self, s):
        """Interpolated world position at arc length `s` along the path."""
        s = max(0.0, min(self.total_len, s))
        lo, hi = 0, len(self.node_arc) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.node_arc[mid] < s:
                lo = mid + 1
            else:
                hi = mid
        i = max(1, lo)
        a0, a1 = self.node_arc[i - 1], self.node_arc[i]
        frac = 0.0 if a1 <= a0 else (s - a0) / (a1 - a0)
        (x0, y0) = self.maze.pos[self.path[i - 1]]
        (x1, y1) = self.maze.pos[self.path[i]]
        return (x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac)

    def _heading_at_arc(self, s):
        i = 1
        while i < len(self.node_arc) - 1 and self.node_arc[i] < s - 1e-9:
            i += 1
        return direction(self.maze, self.path[i - 1], self.path[i])

    def speed_color_segments(self, up_to_arc):
        """Small coloured sub-segments (world coords) up to arc `up_to_arc`.

        Each segment's colour encodes the robot's speed there (green->red).
        """
        segs = []
        s = 0.0
        while s < min(up_to_arc, self.total_len) - 1e-9:
            s_next = min(s + GRADIENT_SEG, up_to_arc, self.total_len)
            mid = 0.5 * (s + s_next)
            v = self._speed_at_arc(mid)
            p0 = self.pos_at_arc(s)
            p1 = self.pos_at_arc(s_next)
            segs.append((p0, p1, speed_to_color(v, self.vmax)))
            s = s_next
        return segs

    def _speed_at_arc(self, s):
        for s0, s1 in self.runs:
            if s0 - 1e-9 <= s <= s1 + 1e-9:
                return speed_at(s - s0, s1 - s0, self.vmax, self.accel)
        return 0.0


# =========================================================================== #
#  Robot - improved hybrid frontier exploration
# =========================================================================== #
class Robot:
    def __init__(self, maze, vmax=DEFAULT_VMAX, accel=DEFAULT_ACCEL,
                 use_proof=True):
        self.maze = maze
        self.current = maze.start
        self.visited_nodes = {maze.start}
        self.visited_edges = set()
        self.heading = None
        self.exploration_complete = False
        self.target_found = False
        self.total_distance = 0.0
        # Motion model used by the TIME-based early-stop proof (kept in sync with
        # the GUI sliders). The proof must use the same metric the fast run does.
        self.vmax = vmax
        self.accel = accel
        # When False, the early-stop proof is disabled: the robot ALWAYS maps the
        # full maze (the "optimized search" toggle in the GUI).
        self.use_proof = use_proof
        # How exploration ended:
        self.fully_explored = False     # every path was discovered
        self.proven_optimal = False     # stopped early - proven no faster path

    def _known_adj(self):
        adj = {n: set() for n in self.visited_nodes}
        for a, b in self.visited_edges:
            adj[a].add(b)
            adj[b].add(a)
        return adj

    def unexplored_neighbors(self, node):
        return [nb for nb in self.maze.neighbors(node)
                if tuple(sorted((node, nb))) not in self.visited_edges]

    def frontiers(self):
        return {n for n in self.visited_nodes if self.unexplored_neighbors(n)}

    def _direction(self, a, b):
        (x1, y1), (x2, y2) = self.maze.pos[a], self.maze.pos[b]
        return ((x2 > x1) - (x2 < x1), (y2 > y1) - (y2 < y1))

    def _branch_priority(self, neighbor):
        """Target-agnostic ranking: straight < left < right < reverse.

        Ties (and the very first move, when heading is unknown) fall back to a
        fixed compass order so behaviour is deterministic - never uses the
        target's location.
        """
        new_h = self._direction(self.current, neighbor)
        cmd = turn_command(self.heading, new_h)
        return (TURN_RANK[cmd], DIR_INDEX.get(new_h, 9))

    def _go(self, nxt):
        new_h = self._direction(self.current, nxt)
        cmd = turn_command(self.heading, new_h)
        self.total_distance += self.maze.dist(self.current, nxt)
        self.visited_edges.add(tuple(sorted((self.current, nxt))))
        self.visited_nodes.add(nxt)
        rec = {"from": self.current, "to": nxt, "cmd": cmd,
               "heading": heading_name(new_h)}
        self.current = nxt
        self.heading = new_h
        if nxt == self.maze.target:
            self.target_found = True
        return rec

    def _useful_frontiers(self, fr, dist_start, best_time):
        """Frontiers that could still lead to a path FASTER (in time) than the
        best known one.

        The fastest route accounts for acceleration (stop at turns, speed up on
        straights), so the proof must bound by TIME, not distance - otherwise a
        longer-but-straighter undiscovered path (fewer stops) could be wrongly
        pruned. Any path that first leaves the known map at frontier f has time
        at least:
            LB_time(f) = ( dist(start->f) + straight_line(f->target) ) / v_max
        because the robot can never travel faster than v_max, and `dist(start->f)`
        (known map) and the Euclidean `f->target` are admissible lower bounds on
        the two portions' lengths. If LB_time(f) >= best known time, no path
        through f can beat it, so f is PRUNED. This is per-frontier branch &
        bound, and - being a true lower bound - it never discards a route that
        could actually be faster (e.g. a clear straight on the far side).
        """
        useful = set()
        for f in fr:
            reach = dist_start.get(f, INF) + self.maze.dist(f, self.maze.target)
            lb_time = reach / self.vmax if self.vmax > 0 else INF
            if lb_time < best_time - 1e-9:
                useful.add(f)
        return useful

    def step_explore(self):
        if self.exploration_complete:
            return None, "Exploration already complete."

        target_known = self.target_found
        fr = self.frontiers()

        # Once the target is known, keep only frontiers that could still beat
        # the best path found so far - measured by TIME (acceleration-aware), to
        # match the fast run. Before that we cannot prune (we do not know where
        # the target is), so every frontier counts. With the proof disabled
        # ("optimized search" off) we never prune and map the full maze.
        if target_known and self.use_proof:
            kadj = self._known_adj()
            dist_start, _ = dijkstra_fields(self.maze, kadj, self.maze.start)
            _, best_time = time_optimal_path(
                self.maze, kadj, self.maze.start, self.maze.target,
                self.vmax, self.accel)
            useful = self._useful_frontiers(fr, dist_start, best_time)
        else:
            useful = fr

        # Nothing left worth exploring -> finish and head home.
        if not useful:
            if target_known and fr:
                self.proven_optimal = True   # pruned remaining branches
            else:
                self.fully_explored = True   # genuinely explored everything
            if self.current == self.maze.start:
                self.exploration_complete = True
                msg = ("Proven fastest path - map not fully explored."
                       if self.proven_optimal else
                       "Exploration complete - all paths mapped.")
                return None, msg + " Home."
            adj = self._known_adj()
            _, prev = dijkstra_fields(self.maze, adj, self.current)
            path = path_from_prev(prev, self.maze.start)
            if path and len(path) > 1:
                tag = ("Fastest path proven" if self.proven_optimal
                       else "Mapping done")
                return self._go(path[1]), tag + " - returning home."
            self.exploration_complete = True
            return None, "Home."

        # Priority 1: explore a branch at the current node - but only if this
        # node can still possibly improve the path (target-agnostic before the
        # target is found, since then every branch matters).
        here = self.unexplored_neighbors(self.current)
        if here and (not target_known or self.current in useful):
            here.sort(key=self._branch_priority)
            note = ("Explore (prefer straight, no target knowledge)."
                    if not target_known else
                    "Explore (branch may still beat best path).")
            return self._go(here[0]), note

        # Priority 2: drive to the nearest USEFUL frontier (by real distance).
        adj = self._known_adj()
        dist, prev = dijkstra_fields(self.maze, adj, self.current)
        nearest = min(useful, key=lambda f: dist.get(f, INF))
        path = path_from_prev(prev, nearest)
        if not path or len(path) < 2:
            self.exploration_complete = True
            return None, "No reachable useful frontier."
        return self._go(path[1]), "Drive to nearest useful frontier."


# =========================================================================== #
#  Application
# =========================================================================== #
class SolverApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Maze Solver - SEYED")
        self.root.configure(bg=BG)

        # --- Slider style: white thumb/handle ---
        _s = ttk.Style()
        _s.theme_use('clam')
        _s.configure('white.Horizontal.TScale',
                     background='white',
                     troughcolor=CARD_BG,
                     sliderthickness=18,
                     sliderrelief='flat')
        _s.map('white.Horizontal.TScale',
               background=[('active', '#d0d0d0'), ('!active', 'white')])

        self.maze = None
        self.robot = None
        self.phase = "IDLE"
        self.step_count = 0
        self.playing = False
        self.playback = PLAYBACK_VALUES[DEFAULT_PLAYBACK_INDEX]   # sim/real ratio

        # Motion-model parameters (editable in the GUI), in real-world units.
        self.vmax = DEFAULT_VMAX        # cm/s
        self.accel = DEFAULT_ACCEL      # cm/s^2

        # "Optimized search": when True the robot may stop early once the fastest
        # route is proven; when False it always maps the full maze.
        self.optimize = True

        # Fastest (by TIME) path + alternatives, plus the animated trajectory.
        self.fastest_path = None
        self.alt_paths = []            # list of (path, time) ranked slow->fast
        self.run_plan = None           # FastRunPlan for the fastest path
        self.run_time = 0.0            # sim-seconds elapsed in the fast run
        self.cur_speed = 0.0           # robot's current speed (for the readout)

        # Decoupled visual robot pose (so motion can glide between nodes).
        self.anim_pos = None           # world (x, y)
        self.anim_heading = None
        self._glide = None             # active explore glide, or None
        self._fast_arc = 0.0           # arc length covered on the fast path
        self._arrived_nodes = set()    # nodes the robot graphic has visually reached

        self.scale = 2.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self._pan_anchor = None
        self._after_id = None
        self._needs_fit = True
        self._img_cache = {}     # cached anti-aliased circle images

        self._build_ui()
        self.root.update_idletasks()

        sample = os.path.join(MAZES_DIR, "sample_maze.json")
        if os.path.isfile(sample):
            self.load_maze(sample)

    # --------------------------------------------------------------------- #
    def _build_ui(self):
        # Bottom status bar first so it always keeps its space.
        self.status = tk.StringVar(value="Load a maze to begin.")
        tk.Label(self.root, textvariable=self.status, anchor="w", bg=CARD_BG,
                 fg=FG_MUTED, padx=10, pady=5, font=FONT).pack(
            side=tk.BOTTOM, fill=tk.X)

        # Resizable split: drag the divider to widen/narrow the panel.
        main = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=BG,
                              sashwidth=7, sashrelief=tk.FLAT, bd=0,
                              sashpad=0, opaqueresize=True)
        main.pack(fill=tk.BOTH, expand=True)

        canvas_holder = tk.Frame(main, bg=BG)
        self.canvas = tk.Canvas(canvas_holder, bg=CANVAS_BG,
                                highlightthickness=0, bd=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=(10, 0), pady=10)
        self.canvas.bind("<MouseWheel>", self.on_zoom)
        self.canvas.bind("<Button-4>", self.on_zoom)
        self.canvas.bind("<Button-5>", self.on_zoom)
        self.canvas.bind("<ButtonPress-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan_move)
        self.canvas.bind("<ButtonPress-3>", self.on_pan_start)
        self.canvas.bind("<B3-Motion>", self.on_pan_move)
        self.canvas.bind("<Configure>", self._on_configure)

        panel = tk.Frame(main, bg=PANEL_BG)
        main.add(canvas_holder, stretch="always", minsize=380)
        main.add(panel, minsize=250, width=300)

        # Scrollable panel: shows a scrollbar when the window is too short.
        self._pcanvas = tk.Canvas(panel, bg=PANEL_BG, highlightthickness=0, bd=0)
        pscroll = tk.Scrollbar(panel, orient=tk.VERTICAL,
                               command=self._pcanvas.yview)
        self._pcanvas.configure(yscrollcommand=pscroll.set)
        pscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self._pcanvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        pad_window = tk.Frame(self._pcanvas, bg=PANEL_BG)
        self._pad_win = self._pcanvas.create_window((0, 0), window=pad_window,
                                                    anchor="nw")
        pad_window.bind(
            "<Configure>",
            lambda e: self._pcanvas.configure(scrollregion=self._pcanvas.bbox("all")))
        self._pcanvas.bind(
            "<Configure>",
            lambda e: self._pcanvas.itemconfig(self._pad_win, width=e.width))
        pad = tk.Frame(pad_window, bg=PANEL_BG)
        pad.pack(fill=tk.X, padx=14, pady=14)

        tk.Label(pad, text="Maze Solver", font=FONT_TITLE, bg=PANEL_BG,
                 fg=FG).pack(anchor="w")
        # tk.Label(pad, text="SEYED \u2022 Final Version", font=("Segoe UI", 9),
        #          bg=PANEL_BG, fg=ACCENT).pack(anchor="w", pady=(0, 8))

        toprow = tk.Frame(pad, bg=PANEL_BG)
        toprow.pack(fill=tk.X)
        self._button(toprow, "Load Maze\u2026", self.choose_maze, side=tk.LEFT,
                     expand=True)
        self._button(toprow, "Fit View",
                     lambda: (self.fit_view(), self.redraw()), side=tk.LEFT)

        ctl = tk.Frame(pad, bg=PANEL_BG)
        ctl.pack(fill=tk.X, pady=(8, 0))
        self.play_btn = self._button(ctl, "\u25B6  Play", self.toggle_play,
                                     side=tk.LEFT, expand=True, accent=True)
        self._button(ctl, "Step", self.single_step, side=tk.LEFT, expand=True)
        self._button(ctl, "Reset", self.reset, side=tk.LEFT, expand=True)

        # Optimized-search toggle: ON = stop early once proven; OFF = full map.
        self.optimize_btn = self._button(
            pad, "Optimized search:  ON", self._toggle_optimize)
        self._style_toggle(self.optimize_btn, self.optimize)

        # --- Playback speed: a real-time multiplier (1x = true real time) --- #
        prow = tk.Frame(pad, bg=PANEL_BG)
        prow.pack(fill=tk.X, pady=(12, 0))
        tk.Label(prow, text="PLAYBACK", font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG_MUTED).pack(side=tk.LEFT)
        self.var_playback = tk.StringVar(value=self._fmt_mult(self.playback))
        tk.Label(prow, textvariable=self.var_playback, font=("Segoe UI", 9, "bold"),
                 bg=PANEL_BG, fg=ACCENT).pack(side=tk.RIGHT)
        self.speed = ttk.Scale(pad, from_=0, to=len(PLAYBACK_VALUES) - 1,
                               orient=tk.HORIZONTAL, command=self._on_playback,
                               style='white.Horizontal.TScale')
        self.speed.set(DEFAULT_PLAYBACK_INDEX)
        self.speed.pack(fill=tk.X)

        # # --- Robot motion model (real-world units; affects the fast run) --- #
        # tk.Label(pad, text="ROBOT MOTION MODEL", font=("Segoe UI", 8, "bold"),
        #          bg=PANEL_BG, fg=ACCENT2).pack(anchor="w", pady=(12, 0))
        self.var_vmax = tk.StringVar(value=f"{int(DEFAULT_VMAX)} cm/s")
        vh = tk.Frame(pad, bg=PANEL_BG)
        vh.pack(fill=tk.X, pady=(4, 0))
        tk.Label(vh, text="Max speed", font=("Segoe UI", 8), bg=PANEL_BG,
                 fg=FG_MUTED).pack(side=tk.LEFT)
        tk.Label(vh, textvariable=self.var_vmax, font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG).pack(side=tk.RIGHT)
        self.vmax_scale = ttk.Scale(pad, from_=10, to=200, orient=tk.HORIZONTAL,
                                    command=self._on_vmax,
                                    style='white.Horizontal.TScale')
        self.vmax_scale.set(int(DEFAULT_VMAX))
        self.vmax_scale.pack(fill=tk.X)
        self.var_accel = tk.StringVar(value=f"{int(DEFAULT_ACCEL)} cm/s\u00b2")
        ah = tk.Frame(pad, bg=PANEL_BG)
        ah.pack(fill=tk.X, pady=(4, 0))
        tk.Label(ah, text="Max accel", font=("Segoe UI", 8), bg=PANEL_BG,
                 fg=FG_MUTED).pack(side=tk.LEFT)
        tk.Label(ah, textvariable=self.var_accel, font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG).pack(side=tk.RIGHT)
        self.accel_scale = ttk.Scale(pad, from_=10, to=200, orient=tk.HORIZONTAL,
                                     command=self._on_accel,
                                     style='white.Horizontal.TScale')
        self.accel_scale.set(int(DEFAULT_ACCEL))
        self.accel_scale.pack(fill=tk.X)

        # Speedometer gauge (car-cluster style).
        self._build_gauge(pad)

        # Status card
        info = tk.Frame(pad, bg=CARD_BG)
        info.pack(fill=tk.X, pady=(12, 0))
        inner = tk.Frame(info, bg=CARD_BG)
        inner.pack(fill=tk.X, padx=12, pady=10)
        self.var_phase = tk.StringVar(value="-")
        self.var_step = tk.StringVar(value="0")
        self.var_node = tk.StringVar(value="-")
        self.var_cmd = tk.StringVar(value="-")
        self.var_found = tk.StringVar(value="No")
        self.var_dist = tk.StringVar(value="0")
        self.var_proof = tk.StringVar(value="-")
        self.var_speed = tk.StringVar(value="0")
        self.var_ptime = tk.StringVar(value="-")
        rows = [("Phase", self.var_phase), ("Steps", self.var_step),
                ("At node", self.var_node), ("Last cmd", self.var_cmd),
                ("Target found", self.var_found), ("Travelled", self.var_dist),
                ("Proof", self.var_proof), ("Speed", self.var_speed),
                ("Path time", self.var_ptime)]
        self._value_labels = {}
        for label, var in rows:
            r = tk.Frame(inner, bg=CARD_BG)
            r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=label, width=13, anchor="w", bg=CARD_BG,
                     fg=FG_MUTED, font=FONT).pack(side=tk.LEFT)
            lbl = tk.Label(r, textvariable=var, anchor="w", bg=CARD_BG, fg=FG,
                           font=FONT_MONO)
            lbl.pack(side=tk.LEFT)
            self._value_labels[label] = lbl

        tk.Label(pad, text="COMMAND LOG", font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG_MUTED).pack(anchor="w", pady=(12, 4))
        logframe = tk.Frame(pad, bg=CARD_BG)
        logframe.pack(fill=tk.X)
        scroll = tk.Scrollbar(logframe)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log = tk.Listbox(logframe, font=FONT_MONO_S, height=8,
                              yscrollcommand=scroll.set, bg=CARD_BG, fg=FG,
                              bd=0, highlightthickness=0,
                              selectbackground=ACCENT, activestyle="none")
        self.log.pack(side=tk.LEFT, fill=tk.X, expand=True)
        scroll.config(command=self.log.yview)

        leg = tk.Frame(pad, bg=PANEL_BG)
        leg.pack(fill=tk.X, pady=(8, 0))
        for color, text, shape in [(VISITED_EDGE, "Visited path", "line"),
                                   (STUB_COLOR, "Unexplored stub", "line"),
                                   (FASTEST_PATH, "Fastest path", "line"),
                                   (ALT_PATH_LIGHT, "Slower paths", "line"),
                                   (None, "Fast run: slow\u2192fast", "grad"),
                                   (ROBOT_COLOR, "Robot", "dot"),
                                   (TARGET_FILL, "Target", "target")]:
            r = tk.Frame(leg, bg=PANEL_BG)
            r.pack(fill=tk.X, pady=1)
            c = tk.Canvas(r, width=18, height=14, bg=PANEL_BG, highlightthickness=0)
            if shape == "line":
                c.create_line(2, 7, 16, 7, fill=color, width=4)
            elif shape == "grad":
                for j in range(7):
                    seg = speed_to_color(j / 6.0 * self.vmax, self.vmax)
                    c.create_line(2 + j * 2, 7, 4 + j * 2, 7, fill=seg, width=4)
            elif shape == "target":
                c.create_oval(4, 1, 14, 11, fill=TARGET_FILL, outline=TARGET_RING,
                              width=2)
            else:
                c.create_oval(4, 1, 14, 11, fill=color, outline="")
            c.pack(side=tk.LEFT)
            tk.Label(r, text=text, bg=PANEL_BG, fg=FG, font=FONT).pack(
                side=tk.LEFT, padx=6)

        # Enable mouse-wheel scrolling anywhere over the panel.
        self._bind_panel_wheel(self._pcanvas)

    def _bind_panel_wheel(self, widget):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            widget.bind(seq, self._panel_wheel)
        for child in widget.winfo_children():
            self._bind_panel_wheel(child)

    def _panel_wheel(self, event):
        if getattr(event, "num", None) == 5 or getattr(event, "delta", 0) < 0:
            self._pcanvas.yview_scroll(1, "units")
        elif getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0:
            self._pcanvas.yview_scroll(-1, "units")
        return "break"

    def _button(self, parent, text, cmd, side=tk.TOP, expand=False, accent=False,
                compact=False):
        bg = ACCENT if accent else "#262c3d"
        active = "#7aa0ff" if accent else "#343c54"
        fg = "#0d0f15" if accent else FG
        btn = tk.Button(parent, text=text, command=cmd, font=FONT_BOLD if accent else FONT,
                        bg=bg, fg=fg, activebackground=active, activeforeground=fg,
                        relief=tk.FLAT, bd=0, padx=10, pady=4 if compact else 7,
                        cursor="hand2", highlightthickness=0)
        if compact:
            btn.pack(anchor="e", pady=3)         # small, right-aligned
        elif side == tk.TOP:
            btn.pack(fill=tk.X, pady=3)
        else:
            btn.pack(side=side, fill=tk.X, expand=expand, padx=2)
        btn.bind("<Enter>", lambda e: btn.config(bg=active))
        btn.bind("<Leave>", lambda e: btn.config(bg=bg))
        btn._base_bg = bg
        return btn

    # --------------------------------------------------------------------- #
    #  Speedometer gauge (car-cluster style)
    # --------------------------------------------------------------------- #
    def _build_gauge(self, parent):
        tk.Label(parent, text="SPEEDOMETER", font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG_MUTED).pack(anchor="w", pady=(12, 2))
        holder = tk.Frame(parent, bg=CARD_BG)
        holder.pack(fill=tk.X)
        self.gauge = tk.Canvas(holder, height=150, bg=CARD_BG,
                               highlightthickness=0, bd=0)
        self.gauge.pack(fill=tk.X, padx=8, pady=8)
        self.gauge.bind("<Configure>", lambda e: self._draw_gauge())
        self._draw_gauge()

    def _draw_gauge(self):
        g = getattr(self, "gauge", None)
        if g is None:
            return
        g.delete("all")
        w = g.winfo_width()
        if w <= 1:
            w = 240                       # not laid out yet; use a sensible default
        h = int(g.cget("height"))
        cx, cy = w / 2.0, h - 30.0
        R = min(w / 2.0 - 24, h - 44)
        vmax = max(1.0, float(self.vmax))
        v = max(0.0, min(vmax, float(self.cur_speed)))
        bbox = (cx - R, cy - R, cx + R, cy + R)

        # Faint full-sweep backing arc.
        g.create_arc(bbox, start=0, extent=180, style=tk.ARC,
                     outline="#2a3043", width=12)
        # Coloured speed band (green -> yellow -> red), filled up to current v.
        N = 48
        frac_now = v / vmax
        for k in range(N):
            f0, f1 = k / N, (k + 1) / N
            start = 180.0 * (1.0 - f1)
            mid = 0.5 * (f0 + f1)
            lit = mid <= frac_now
            color = speed_to_color(mid * vmax, vmax) if lit else "#333a4f"
            g.create_arc(bbox, start=start, extent=180.0 / N + 0.7, style=tk.ARC,
                         outline=color, width=12)

        # Ticks + speed labels around the dial.
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            ang = math.radians(180.0 * (1.0 - t))
            ca, sa = math.cos(ang), math.sin(ang)
            g.create_line(cx + (R - 13) * ca, cy - (R - 13) * sa,
                          cx + (R - 3) * ca, cy - (R - 3) * sa,
                          fill=FG_MUTED, width=2)
            g.create_text(cx + (R - 26) * ca, cy - (R - 26) * sa,
                          text=f"{int(round(t * vmax))}", fill=FG_MUTED,
                          font=("Segoe UI", 7))

        # Needle + hub.
        ang = math.radians(180.0 * (1.0 - frac_now))
        g.create_line(cx, cy, cx + (R - 16) * math.cos(ang),
                      cy - (R - 16) * math.sin(ang),
                      fill=ROBOT_COLOR, width=3, capstyle=tk.ROUND)
        g.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill="#222838",
                      outline=FG_MUTED, width=1)

        # Digital readout below the pivot.
        g.create_text(cx, cy + 16, text=f"{v:.0f} cm/s", fill=FG,
                      font=("Segoe UI", 12, "bold"))

    # --------------------------------------------------------------------- #
    #  Loading
    # --------------------------------------------------------------------- #
    def choose_maze(self):
        path = filedialog.askopenfilename(
            title="Load Maze",
            initialdir=MAZES_DIR if os.path.isdir(MAZES_DIR) else _THIS_DIR,
            filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")])
        if path:
            self.load_maze(path)

    def load_maze(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.maze = Maze(data)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Load failed", f"Could not load maze:\n{exc}")
            return
        self.reset()
        self._needs_fit = True
        self.fit_view()
        self.redraw()
        self.status.set(f"Loaded {os.path.basename(path)}  "
                        f"({len(self.maze.pos)} nodes). Press Play.")

    # --------------------------------------------------------------------- #
    #  Control
    # --------------------------------------------------------------------- #
    def reset(self):
        self._cancel_loop()
        self.playing = False
        self._set_play_label()
        if self.maze is None:
            return
        self.robot = Robot(self.maze, self.vmax, self.accel, self.optimize)
        self.phase = "EXPLORE"
        self.step_count = 0
        self.fastest_path = None
        self.alt_paths = []
        self.run_plan = None
        self.run_time = 0.0
        self.cur_speed = 0.0
        self._fast_arc = 0.0
        self._fast_node_i = 0
        self._fast_heading = None
        self._last_fast_cmd = "-"
        self._glide = None
        self.anim_pos = self.maze.pos[self.maze.start]
        self.anim_heading = None
        self._arrived_nodes = {self.maze.start}
        self.var_ptime.set("-")
        self.log.delete(0, tk.END)
        self._update_status_vars("-")
        self.redraw()

    def _set_play_label(self):
        if hasattr(self, "play_btn"):
            txt = "\u275A\u275A Pause" if self.playing else "\u25B6  Play"
            self.play_btn.config(text=txt)

    def toggle_play(self):
        if self.maze is None:
            return
        self.playing = not self.playing
        self._set_play_label()
        if self.playing:
            self._loop()
        else:
            # Paused -> the robot is stopped, so the gauge should read 0.
            self.cur_speed = 0.0
            self._update_status_vars(self.var_cmd.get())
            self.redraw()

    def _loop(self):
        if not self.playing:
            return
        # Advance simulated time by (real frame seconds * playback). 1x is real
        # time; the frame interval is fixed so higher playback = larger sim step.
        more = self._frame(FRAME_MS / 1000.0 * self.playback)
        self.redraw()
        if more:
            self._after_id = self.root.after(FRAME_MS, self._loop)
        else:
            self.playing = False
            self._set_play_label()

    def single_step(self):
        if self.maze is None:
            return
        self.playing = False
        self._set_play_label()
        self._step_once()
        self.redraw()

    # --- one animation frame (advances simulated time by `dt`) ------------- #
    def _frame(self, dt):
        if self.phase == "EXPLORE":
            return self._frame_explore(dt)
        if self.phase == "FAST_RUN":
            return self._frame_fast(dt)
        return False

    def _frame_explore(self, dt):
        # Start a new glide when the previous one finished.
        if self._glide is None:
            if self.robot.exploration_complete:
                self._begin_fast_run()
                return True
            rec, info = self.robot.step_explore()
            if rec is None:
                if self.robot.exploration_complete:
                    self._begin_fast_run()
                    return True
                self.status.set(info or "Exploration stopped.")
                return False
            self.step_count += 1
            self._log_record("EXP", rec)
            self.status.set(info)
            a, b = rec["from"], rec["to"]
            length = self.maze.dist(a, b)
            espeed = min(EXPLORE_SPEED, self.vmax)   # never exceed the cap
            self._glide = {"a": a, "b": b, "len": length, "el": 0.0,
                           "dur": max(1e-6, length / espeed),
                           "speed": espeed, "cmd": rec["cmd"]}
            self.anim_heading = direction(self.maze, a, b)
        # Advance the active glide at CONSTANT speed (cautious search pace).
        g = self._glide
        g["el"] += dt
        frac = min(1.0, g["el"] / g["dur"])
        ax, ay = self.maze.pos[g["a"]]
        bx, by = self.maze.pos[g["b"]]
        self.anim_pos = (ax + (bx - ax) * frac, ay + (by - ay) * frac)
        # Constant search speed: the robot drives intersections without stopping,
        # so keep showing the cruise speed (don't blip to 0 at each node arrival).
        self.cur_speed = g["speed"]
        if frac >= 1.0:
            self.anim_pos = (bx, by)
            self._glide = None
            self._arrived_nodes.add(g["b"])
        self._update_status_vars(g["cmd"])
        return True

    def _frame_fast(self, dt):
        if not self.run_plan:
            self._finish_fast()
            return False
        self.run_time += dt
        x, y, heading, v, arc = self.run_plan.sample(self.run_time)
        self.anim_pos = (x, y)
        self.anim_heading = heading
        self.cur_speed = v
        self._fast_arc = arc
        self._log_fast_crossings(arc)
        if self.run_time >= self.run_plan.total_time - 1e-9:
            self._fast_arc = self.run_plan.total_len
            self._log_fast_crossings(self.run_plan.total_len)
            self.anim_pos = self.maze.pos[self.fastest_path[-1]]
            self._finish_fast()
            return False
        self._update_status_vars(self._last_fast_cmd)
        return True

    def _log_fast_crossings(self, arc):
        """Log each fast-run edge as the robot's arc position passes its node."""
        path = self.fastest_path
        while self._fast_node_i < len(path) - 1 and \
                arc >= self.run_plan.node_arc[self._fast_node_i + 1] - 1e-9:
            i = self._fast_node_i
            a, b = path[i], path[i + 1]
            new_h = direction(self.maze, a, b)
            cmd = turn_command(self._fast_heading, new_h)
            self._fast_heading = new_h
            self._last_fast_cmd = cmd
            self.step_count += 1
            self._log_record("RUN", {"from": a, "to": b, "cmd": cmd,
                                     "heading": heading_name(new_h)})
            self._fast_node_i += 1

    def _step_once(self):
        """The Step button: advance one whole logical move (snap, no glide)."""
        if self.phase == "EXPLORE":
            if self._glide is not None:
                g = self._glide
                self.anim_pos = self.maze.pos[g["b"]]
                self._arrived_nodes.add(g["b"])
                self._glide = None
                self._update_status_vars(g["cmd"])
            elif self.robot.exploration_complete:
                self._begin_fast_run()
            else:
                rec, info = self.robot.step_explore()
                if rec is None:
                    if self.robot.exploration_complete:
                        self._begin_fast_run()
                    else:
                        self.status.set(info or "Exploration stopped.")
                    return
                self.step_count += 1
                self._log_record("EXP", rec)
                self.status.set(info)
                self.anim_pos = self.maze.pos[rec["to"]]
                self.anim_heading = direction(self.maze, rec["from"], rec["to"])
                self.cur_speed = 0.0
                self._arrived_nodes.add(rec["to"])
                self._update_status_vars(rec["cmd"])
                if self.robot.exploration_complete:
                    self._begin_fast_run()
        elif self.phase == "FAST_RUN":
            if not self.run_plan or self._fast_node_i >= len(self.fastest_path) - 1:
                self._finish_fast()
                return
            target_arc = self.run_plan.node_arc[self._fast_node_i + 1]
            self.run_time = self.run_plan.time_at_arc(target_arc)
            self._frame_fast(0.0)

    def _begin_fast_run(self):
        # Use only the map the robot actually discovered (honest for early stop).
        adj = self.robot._known_adj()
        self.phase = "FAST_RUN"
        self.run_time = 0.0
        self._fast_arc = 0.0
        self._fast_node_i = 0
        self._fast_heading = None
        self._last_fast_cmd = "-"
        self._glide = None
        self.var_phase.set("FAST RUN")
        self._update_proof_flag()

        # Fastest route is the minimum-TIME path (accel-aware); alternatives are
        # other routes shown lighter so the fastest stands out.
        self.fastest_path, best_t = time_optimal_path(
            self.maze, adj, self.maze.start, self.maze.target,
            self.vmax, self.accel)
        self._compute_alt_paths(adj)
        if self.fastest_path and len(self.fastest_path) > 1:
            self.run_plan = FastRunPlan(self.maze, self.fastest_path,
                                        self.vmax, self.accel)
            self.anim_pos = self.maze.pos[self.maze.start]
            self.anim_heading = None
            self.cur_speed = 0.0
            dist = sum(self.maze.dist(a, b)
                       for a, b in zip(self.fastest_path, self.fastest_path[1:]))
            self.var_ptime.set(f"{best_t:.2f}s")
            how = ("proven early (map NOT fully explored)"
                   if self.robot.proven_optimal else "after full mapping")
            self.status.set(f"Fastest path {how}: {self.fastest_path} "
                            f"(time {best_t:.2f}s, dist {dist:.0f}; "
                            f"travelled {self.robot.total_distance:.0f}).")
        else:
            self.run_plan = None
            self.status.set("Exploration done, but no path to target found!")

    def _compute_alt_paths(self, adj):
        """Alternative routes (Yen K-shortest by distance), ranked by TIME."""
        self.alt_paths = []
        cands = yen_k_shortest(self.maze, adj, self.maze.start,
                               self.maze.target, N_ALT_PATHS)
        seen = set()
        ranked = []
        for p in cands:
            key = tuple(p)
            if key in seen or p == self.fastest_path:
                continue
            seen.add(key)
            ranked.append((p, path_time(self.maze, p, self.vmax, self.accel)))
        ranked.sort(key=lambda pt: pt[1])      # fastest alternative first
        self.alt_paths = ranked

    def _rebuild_fast_run(self):
        """Recompute the fast-run plan after a motion-parameter change."""
        if self.phase not in ("FAST_RUN", "DONE") or self.robot is None:
            return
        adj = self.robot._known_adj()
        keep_arc = self._fast_arc
        self.fastest_path, best_t = time_optimal_path(
            self.maze, adj, self.maze.start, self.maze.target,
            self.vmax, self.accel)
        self._compute_alt_paths(adj)
        if not (self.fastest_path and len(self.fastest_path) > 1):
            self.run_plan = None
            return
        self.run_plan = FastRunPlan(self.maze, self.fastest_path,
                                    self.vmax, self.accel)
        self.var_ptime.set(f"{best_t:.2f}s")
        if self.phase == "DONE":
            self._fast_arc = self.run_plan.total_len
            self.run_time = self.run_plan.total_time
        else:
            # Continue from roughly the same point along the path.
            keep_arc = min(keep_arc, self.run_plan.total_len)
            self._fast_arc = keep_arc
            self.run_time = self.run_plan.time_at_arc(keep_arc)
            x, y, heading, v, arc = self.run_plan.sample(self.run_time)
            self.anim_pos = (x, y)
            self.anim_heading = heading
            self.cur_speed = v
        self.redraw()

    def _finish_fast(self):
        self.phase = "DONE"
        self.cur_speed = 0.0
        self.var_phase.set("DONE")
        self._update_status_vars(self._last_fast_cmd)
        self.status.set("Fast run complete - robot reached the target.")

    def _log_record(self, tag, rec):
        self.log.insert(tk.END, f"{tag} {rec['cmd']}  {rec['from']:>3} -> "
                                f"{rec['to']:<3}  ({rec['heading']})")
        self.log.see(tk.END)

    def _update_status_vars(self, last_cmd):
        names = {"EXPLORE": "EXPLORING", "FAST_RUN": "FAST RUN", "DONE": "DONE",
                 "IDLE": "-"}
        self.var_phase.set(names.get(self.phase, self.phase))
        self.var_step.set(str(self.step_count))
        if self.phase in ("FAST_RUN", "DONE") and self.fastest_path:
            i = min(self._fast_node_i, len(self.fastest_path) - 1)
            cur = self.fastest_path[i]
        else:
            cur = self.robot.current if self.robot else "-"
        self.var_node.set(str(cur))
        self.var_cmd.set(last_cmd)
        self.var_found.set("Yes" if (self.robot and self.robot.target_found) else "No")
        self.var_dist.set(f"{self.robot.total_distance:.0f} cm" if self.robot else "0 cm")
        self.var_speed.set(f"{self.cur_speed:.0f} cm/s")
        self._update_proof_flag()

    def _update_proof_flag(self):
        """The GUI flag the user asked for: shows whether we proved the fastest
        path early (partial map) or had to fully explore."""
        text, color = "-", FG
        if self.robot:
            if self.robot.proven_optimal:
                text, color = "PROVEN \u2713", ACCENT2
            elif self.robot.fully_explored:
                text, color = "full map", FG
            elif not self.optimize:
                text, color = "off (full)", FG_MUTED
            elif not self.robot.target_found:
                text, color = "searching\u2026", STUB_COLOR
            else:
                text, color = "checking\u2026", STUB_COLOR
        self.var_proof.set(text)
        if hasattr(self, "_value_labels"):
            self._value_labels["Proof"].config(fg=color)

    @staticmethod
    def _fmt_mult(m):
        return (f"{m:g}x")

    def _on_playback(self, val):
        # Discrete real-time multiplier (0.25x .. 8x); 1x is true real time.
        idx = max(0, min(len(PLAYBACK_VALUES) - 1, int(round(float(val)))))
        self.playback = PLAYBACK_VALUES[idx]
        if hasattr(self, "var_playback"):
            self.var_playback.set(self._fmt_mult(self.playback))

    def _on_vmax(self, val):
        self.vmax = max(1.0, float(val))
        self.var_vmax.set(f"{self.vmax:.0f} cm/s")
        if self.robot is not None:
            self.robot.vmax = self.vmax     # keep the TIME-based proof in sync
        self._rebuild_fast_run()
        self._draw_gauge()                  # rescale the dial to the new v_max

    def _on_accel(self, val):
        self.accel = max(1.0, float(val))
        self.var_accel.set(f"{self.accel:.0f} cm/s\u00b2")
        if self.robot is not None:
            self.robot.accel = self.accel
        self._rebuild_fast_run()

    def _style_toggle(self, btn, on):
        """Colour a toggle button green when ON, muted when OFF."""
        bg = ACCENT2 if on else "#262c3d"
        active = "#52f0bf" if on else "#343c54"
        fg = "#0d0f15" if on else FG
        btn.config(bg=bg, fg=fg, activebackground=active, activeforeground=fg)
        btn._base_bg = bg
        # Rebind hover so it reverts to the correct (state-dependent) colours.
        btn.bind("<Enter>", lambda e: btn.config(bg=active))
        btn.bind("<Leave>", lambda e: btn.config(bg=bg))

    def _toggle_optimize(self):
        self.optimize = not self.optimize
        self.optimize_btn.config(
            text=f"Optimized search:  {'ON' if self.optimize else 'OFF'}")
        self._style_toggle(self.optimize_btn, self.optimize)
        if self.robot is not None:
            self.robot.use_proof = self.optimize
        self.status.set(
            "Optimized search ON - may stop early once the fastest route is "
            "proven." if self.optimize else
            "Optimized search OFF - the robot will map the full maze.")
        self._update_proof_flag()

    def _cancel_loop(self):
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
            self._after_id = None

    # --------------------------------------------------------------------- #
    #  Transforms
    # --------------------------------------------------------------------- #
    def _on_configure(self, event=None):
        if self._needs_fit and self.canvas.winfo_width() > 1 and self.maze:
            self.fit_view()
            self._needs_fit = False
        self.redraw()

    def world_to_canvas(self, wx, wy):
        return self.offset_x + wx * self.scale, self.offset_y - wy * self.scale

    def canvas_to_world(self, cx, cy):
        return (cx - self.offset_x) / self.scale, (self.offset_y - cy) / self.scale

    def fit_view(self):
        w = self.canvas.winfo_width() or 830
        h = self.canvas.winfo_height() or 730
        if not self.maze or not self.maze.pos:
            self.scale = 2.0
            self.offset_x, self.offset_y = w / 2, h / 2
            return
        xs = [p[0] for p in self.maze.pos.values()]
        ys = [p[1] for p in self.maze.pos.values()]
        span_x = max(max(xs) - min(xs), self.maze.grid_size)
        span_y = max(max(ys) - min(ys), self.maze.grid_size)
        margin = 80
        self.scale = max(0.3, min((w - 2 * margin) / span_x,
                                  (h - 2 * margin) / span_y, 14))
        cx = (min(xs) + max(xs)) / 2
        cy = (min(ys) + max(ys)) / 2
        self.offset_x = w / 2 - cx * self.scale
        self.offset_y = h / 2 + cy * self.scale

    def on_zoom(self, event):
        factor = 1.1 if (getattr(event, "num", None) == 4 or
                         getattr(event, "delta", 0) > 0) else 1 / 1.1
        wx, wy = self.canvas_to_world(event.x, event.y)
        self.scale = max(0.2, min(self.scale * factor, 30))
        self.offset_x = event.x - wx * self.scale
        self.offset_y = event.y + wy * self.scale
        self.redraw()

    def on_pan_start(self, event):
        self._pan_anchor = (event.x, event.y, self.offset_x, self.offset_y)

    def on_pan_move(self, event):
        if not self._pan_anchor:
            return
        ax, ay, ox, oy = self._pan_anchor
        self.offset_x = ox + (event.x - ax)
        self.offset_y = oy + (event.y - ay)
        self.redraw()

    # --------------------------------------------------------------------- #
    #  Drawing
    # --------------------------------------------------------------------- #
    def redraw(self):
        self._draw_gauge()
        self.canvas.delete("all")
        self._draw_grid()
        if not self.maze:
            return
        self._draw_edges()
        self._draw_stubs()
        self._draw_paths()
        self._draw_nodes()
        self._draw_robot()

    def _draw_grid(self):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        if w <= 1 or h <= 1:
            return
        step = self.maze.grid_size if self.maze else 20
        wx0, wy1 = self.canvas_to_world(0, 0)
        wx1, wy0 = self.canvas_to_world(w, h)
        while (wx1 - wx0) / step > 120:
            step *= 2
        x = int(wx0 // step) * step
        while x <= wx1:
            cx, _ = self.world_to_canvas(x, 0)
            self.canvas.create_line(cx, 0, cx, h,
                                    fill=AXIS_COLOR if x == 0 else GRID_COLOR)
            x += step
        y = int(wy0 // step) * step
        while y <= wy1:
            _, cy = self.world_to_canvas(0, y)
            self.canvas.create_line(0, cy, w, cy,
                                    fill=AXIS_COLOR if y == 0 else GRID_COLOR)
            y += step

    def _draw_edges(self):
        visited = self.robot.visited_edges if self.robot else set()
        for a in self.maze.pos:
            for b in self.maze.neighbors(a):
                if a < b:
                    x1, y1 = self.world_to_canvas(*self.maze.pos[a])
                    x2, y2 = self.world_to_canvas(*self.maze.pos[b])
                    if (a, b) in visited:
                        self.canvas.create_line(x1, y1, x2, y2,
                                                fill=VISITED_EDGE, width=7,
                                                capstyle=tk.ROUND)
                    else:
                        self.canvas.create_line(x1, y1, x2, y2,
                                                fill=UNEXPLORED_EDGE, width=4,
                                                capstyle=tk.ROUND)

    def _draw_stubs(self):
        if not self.robot:
            return
        for u in self._arrived_nodes:
            x1, y1 = self.maze.pos[u]
            for v in self.maze.neighbors(u):
                if tuple(sorted((u, v))) in self.robot.visited_edges:
                    continue
                x2, y2 = self.maze.pos[v]
                dx, dy = x2 - x1, y2 - y1
                mag = math.hypot(dx, dy)
                if mag == 0:
                    continue
                ex, ey = x1 + STUB_LEN * dx / mag, y1 + STUB_LEN * dy / mag
                cx1, cy1 = self.world_to_canvas(x1, y1)
                cx2, cy2 = self.world_to_canvas(ex, ey)
                self.canvas.create_line(cx1, cy1, cx2, cy2, fill=STUB_COLOR,
                                        width=6, capstyle=tk.ROUND)

    def _draw_paths(self):
        if self.phase not in ("FAST_RUN", "DONE"):
            return
        # 1) Alternative (slower) routes: the slower, the lighter & thinner.
        n = len(self.alt_paths)
        for rank, (path, _t) in enumerate(self.alt_paths):
            color = alt_path_color(rank, n)
            width = max(3, 7 - rank)
            for a, b in zip(path, path[1:]):
                x1, y1 = self.world_to_canvas(*self.maze.pos[a])
                x2, y2 = self.world_to_canvas(*self.maze.pos[b])
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width,
                                        capstyle=tk.ROUND)
        # 2) The fastest route: bold, dark green (drawn on top of alternatives).
        if self.fastest_path and len(self.fastest_path) > 1:
            for a, b in zip(self.fastest_path, self.fastest_path[1:]):
                x1, y1 = self.world_to_canvas(*self.maze.pos[a])
                x2, y2 = self.world_to_canvas(*self.maze.pos[b])
                self.canvas.create_line(x1, y1, x2, y2, fill=FASTEST_PATH_DARK,
                                        width=11, capstyle=tk.ROUND)
        # 3) Speed gradient on the part the robot has already driven (red=fast).
        if self.run_plan and self._fast_arc > 0:
            for p0, p1, color in self.run_plan.speed_color_segments(self._fast_arc):
                x1, y1 = self.world_to_canvas(*p0)
                x2, y2 = self.world_to_canvas(*p1)
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=13,
                                        capstyle=tk.ROUND)

    def _draw_nodes(self):
        for p in self.maze.pos.values():
            self._disc(p, NODE_COLOR, NODE_RADIUS, outline=NODE_OUTLINE, ow=2)
        sp = self.maze.pos[self.maze.start]
        self._ring(sp, START_COLOR, NODE_RADIUS + 4, 2)
        self._disc(sp, START_COLOR, NODE_RADIUS + 3, outline=CANVAS_BG, ow=2)
        self._target(self.maze.pos[self.maze.target])

    def _draw_robot(self):
        if not self.robot:
            return
        if self.anim_pos is not None:
            pos, heading = self.anim_pos, self.anim_heading
        else:
            pos, heading = self.maze.pos[self.robot.current], self.robot.heading
        x, y = self.world_to_canvas(*pos)
        r = NODE_RADIUS + 4
        self._ring(pos, ROBOT_COLOR, r + 3, 2)
        self._disc(pos, ROBOT_COLOR, r, outline="#ffffff", ow=2)
        if heading:
            hx, hy = heading
            self.canvas.create_line(x, y, x + hx * (r + 10), y - hy * (r + 10),
                                    fill="white", width=3, arrow=tk.LAST)

    # --- anti-aliased circle primitives (Pillow-backed, with fallback) ---
    def _target(self, pos):
        r = NODE_RADIUS + 6
        self._ring(pos, TARGET_RING, r + 4, 2)
        self._disc(pos, TARGET_FILL, r, outline=TARGET_RING, ow=3)

    @staticmethod
    def _hex_to_rgba(color):
        h = color.lstrip("#")
        return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), 255)

    def _circle_image(self, fill, outline, r, ow):
        """Return a cached, supersampled (anti-aliased) circle PhotoImage."""
        key = (fill, outline, int(r), int(ow))
        cached = self._img_cache.get(key)
        if cached is not None:
            return cached
        ss = 4
        pad = ow + 2
        diam = 2 * r
        big = (diam + 2 * pad) * ss
        img = Image.new("RGBA", (big, big), (0, 0, 0, 0))
        d = ImageDraw.Draw(img)
        box = [pad * ss, pad * ss, big - pad * ss - 1, big - pad * ss - 1]
        d.ellipse(box,
                  fill=self._hex_to_rgba(fill) if fill else None,
                  outline=self._hex_to_rgba(outline) if outline else None,
                  width=max(1, ow * ss) if outline else 0)
        small = big // ss
        img = img.resize((small, small), Image.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        self._img_cache[key] = photo
        return photo

    def _disc(self, pos, fill, r, outline="", ow=0):
        x, y = self.world_to_canvas(*pos)
        if _HAS_PIL:
            self.canvas.create_image(x, y, image=self._circle_image(fill, outline, r, ow))
        else:
            self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=fill,
                                    outline=outline, width=max(1, ow))

    def _ring(self, pos, color, r, ow=2):
        x, y = self.world_to_canvas(*pos)
        if _HAS_PIL:
            self.canvas.create_image(x, y, image=self._circle_image(None, color, r, ow))
        else:
            self.canvas.create_oval(x - r, y - r, x + r, y + r, outline=color,
                                    width=ow)


def _enable_hidpi():
    """Make the app DPI-aware on Windows so circles/edges render crisply."""
    import sys
    if sys.platform != "win32":
        return
    import ctypes
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)   # system DPI aware
    except Exception:  # noqa: BLE001
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:  # noqa: BLE001
            pass


def main():
    _enable_hidpi()
    root = tk.Tk()
    try:
        root.tk.call("tk", "scaling", root.winfo_fpixels("1i") / 72.0)
    except Exception:  # noqa: BLE001
        pass
    root.geometry("1200x800")
    SolverApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
