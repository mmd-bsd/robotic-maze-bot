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
  3. FAST RUN  - the optimal (shortest-distance) start -> target path is computed
                 and the robot drives it, animated.

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

NOTE: the "fastest path" uses shortest geometric distance (Dijkstra). The
acceleration / variable-speed timing model is planned for the final robot
version and is intentionally left out here.

Requires only the Python standard library (tkinter).
"""

import heapq
import json
import math
import os
import tkinter as tk
from tkinter import filedialog, messagebox

try:
    from PIL import Image, ImageDraw, ImageTk
    _HAS_PIL = True
except Exception:  # noqa: BLE001
    _HAS_PIL = False

# ----------------------------- Configuration ------------------------------- #
NODE_RADIUS = 7
STUB_LEN = 17.0
DEFAULT_DELAY_MS = 280

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


# =========================================================================== #
#  Robot - improved hybrid frontier exploration
# =========================================================================== #
class Robot:
    def __init__(self, maze):
        self.maze = maze
        self.current = maze.start
        self.visited_nodes = {maze.start}
        self.visited_edges = set()
        self.heading = None
        self.exploration_complete = False
        self.target_found = False
        self.total_distance = 0.0
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

    def _useful_frontiers(self, fr, dist_start, best_known):
        """Frontiers that could still lead to a path shorter than best_known.

        A path that first leaves the known map at frontier f has length at least
            dist(start -> f)  +  straight_line(f -> target)
        (the straight line is an admissible lower bound for any maze). If that
        bound is already >= the best known path, frontier f can be PRUNED - the
        robot need never explore it. This is per-frontier branch-and-bound, so
        branches that are provably too far (e.g. on the far side of the target)
        are skipped entirely.
        """
        useful = set()
        for f in fr:
            lb = dist_start.get(f, INF) + self.maze.dist(f, self.maze.target)
            if lb < best_known - 1e-9:
                useful.add(f)
        return useful

    def step_explore(self):
        if self.exploration_complete:
            return None, "Exploration already complete."

        target_known = self.target_found
        fr = self.frontiers()

        # Once the target is known, keep only frontiers that could still beat
        # the best path found so far. Before that we cannot prune (we do not
        # know where the target is), so every frontier counts.
        if target_known:
            kadj = self._known_adj()
            dist_start, _ = dijkstra_fields(self.maze, kadj, self.maze.start)
            best_known = dist_start.get(self.maze.target, INF)
            useful = self._useful_frontiers(fr, dist_start, best_known)
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

        self.maze = None
        self.robot = None
        self.phase = "IDLE"
        self.step_count = 0
        self.playing = False
        self.delay = DEFAULT_DELAY_MS

        self.optimal_path = None
        self.fast_index = 0
        self.fast_current = None
        self.fast_heading = None
        self.fast_traversed = []

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
        tk.Label(pad, text="SEYED \u2022 Final Version", font=("Segoe UI", 9),
                 bg=PANEL_BG, fg=ACCENT).pack(anchor="w", pady=(0, 8))

        self._button(pad, "Load Maze\u2026", self.choose_maze)

        ctl = tk.Frame(pad, bg=PANEL_BG)
        ctl.pack(fill=tk.X, pady=(8, 0))
        self.play_btn = self._button(ctl, "\u25B6  Play", self.toggle_play,
                                     side=tk.LEFT, expand=True, accent=True)
        self._button(ctl, "Step", self.single_step, side=tk.LEFT, expand=True)
        self._button(ctl, "Reset", self.reset, side=tk.LEFT, expand=True)

        tk.Label(pad, text="SPEED  (slow \u2192 fast)", font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG_MUTED).pack(anchor="w", pady=(12, 0))
        self.speed = tk.Scale(pad, from_=1, to=100, orient=tk.HORIZONTAL,
                              command=self._on_speed, bg=PANEL_BG, fg=FG,
                              troughcolor=CARD_BG, highlightthickness=0,
                              bd=0, activebackground=ACCENT, sliderrelief=tk.FLAT)
        self.speed.set(65)
        self.speed.pack(fill=tk.X)

        self._button(pad, "Fit View", lambda: (self.fit_view(), self.redraw()))

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
        rows = [("Phase", self.var_phase), ("Steps", self.var_step),
                ("At node", self.var_node), ("Last cmd", self.var_cmd),
                ("Target found", self.var_found), ("Travelled", self.var_dist),
                ("Proof", self.var_proof)]
        self._value_labels = {}
        for label, var in rows:
            r = tk.Frame(inner, bg=CARD_BG)
            r.pack(fill=tk.X, pady=1)
            tk.Label(r, text=label, width=12, anchor="w", bg=CARD_BG,
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
                                   (OPTIMAL_EDGE, "Fastest path", "line"),
                                   (ROBOT_COLOR, "Robot", "dot"),
                                   (TARGET_FILL, "Target", "target")]:
            r = tk.Frame(leg, bg=PANEL_BG)
            r.pack(fill=tk.X, pady=1)
            c = tk.Canvas(r, width=18, height=14, bg=PANEL_BG, highlightthickness=0)
            if shape == "line":
                c.create_line(2, 7, 16, 7, fill=color, width=4)
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

    def _button(self, parent, text, cmd, side=tk.TOP, expand=False, accent=False):
        bg = ACCENT if accent else "#262c3d"
        active = "#7aa0ff" if accent else "#343c54"
        fg = "#0d0f15" if accent else FG
        btn = tk.Button(parent, text=text, command=cmd, font=FONT_BOLD if accent else FONT,
                        bg=bg, fg=fg, activebackground=active, activeforeground=fg,
                        relief=tk.FLAT, bd=0, padx=10, pady=7, cursor="hand2",
                        highlightthickness=0)
        if side == tk.TOP:
            btn.pack(fill=tk.X, pady=3)
        else:
            btn.pack(side=side, fill=tk.X, expand=expand, padx=2)
        btn.bind("<Enter>", lambda e: btn.config(bg=active))
        btn.bind("<Leave>", lambda e: btn.config(bg=bg))
        btn._base_bg = bg
        return btn

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
        self.robot = Robot(self.maze)
        self.phase = "EXPLORE"
        self.step_count = 0
        self.optimal_path = None
        self.fast_index = 0
        self.fast_current = None
        self.fast_heading = None
        self.fast_traversed = []
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

    def _loop(self):
        if not self.playing:
            return
        more = self.advance()
        self.redraw()
        if more:
            self._after_id = self.root.after(self.delay, self._loop)
        else:
            self.playing = False
            self._set_play_label()

    def single_step(self):
        if self.maze is None:
            return
        self.playing = False
        self._set_play_label()
        self.advance()
        self.redraw()

    def advance(self):
        if self.phase == "EXPLORE":
            return self._advance_explore()
        if self.phase == "FAST_RUN":
            return self._advance_fast()
        return False

    def _advance_explore(self):
        rec, info = self.robot.step_explore()
        if rec:
            self.step_count += 1
            self._log_record("EXP", rec)
            self._update_status_vars(rec["cmd"])
            self.status.set(info)
        if self.robot.exploration_complete:
            self._begin_fast_run()
        return True

    def _begin_fast_run(self):
        # Use only the map the robot actually discovered (honest for early stop).
        adj = self.robot._known_adj()
        _, prev = dijkstra_fields(self.maze, adj, self.maze.start)
        self.optimal_path = path_from_prev(prev, self.maze.target)
        self.phase = "FAST_RUN"
        self.fast_index = 0
        self.fast_current = self.maze.start
        self.fast_heading = None
        self.fast_traversed = []
        self.var_phase.set("FAST RUN")
        self._update_proof_flag()
        if self.optimal_path:
            total = sum(self.maze.dist(a, b)
                        for a, b in zip(self.optimal_path, self.optimal_path[1:]))
            how = ("proven early (map NOT fully explored)"
                   if self.robot.proven_optimal else "after full mapping")
            self.status.set(f"Fastest path {how}: {self.optimal_path} "
                            f"(dist {total:.0f}; travelled {self.robot.total_distance:.0f}).")
        else:
            self.status.set("Exploration done, but no path to target found!")

    def _advance_fast(self):
        if not self.optimal_path or self.fast_index >= len(self.optimal_path) - 1:
            self._finish_fast()
            return False
        a = self.optimal_path[self.fast_index]
        b = self.optimal_path[self.fast_index + 1]
        new_h = self.robot._direction(a, b)
        cmd = turn_command(self.fast_heading, new_h)
        self.fast_heading = new_h
        self.fast_current = b
        self.fast_traversed.append(tuple(sorted((a, b))))
        self.fast_index += 1
        self.step_count += 1
        self._log_record("RUN", {"from": a, "to": b, "cmd": cmd,
                                 "heading": heading_name(new_h)})
        self._update_status_vars(cmd)
        if self.fast_index >= len(self.optimal_path) - 1:
            self._finish_fast()
            return False
        return True

    def _finish_fast(self):
        self.phase = "DONE"
        self.var_phase.set("DONE")
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
        if self.phase in ("FAST_RUN", "DONE") and self.fast_current is not None:
            cur = self.fast_current
        else:
            cur = self.robot.current if self.robot else "-"
        self.var_node.set(str(cur))
        self.var_cmd.set(last_cmd)
        self.var_found.set("Yes" if (self.robot and self.robot.target_found) else "No")
        self.var_dist.set(f"{self.robot.total_distance:.0f}" if self.robot else "0")
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
            elif not self.robot.target_found:
                text, color = "searching\u2026", STUB_COLOR
            else:
                text, color = "checking\u2026", STUB_COLOR
        self.var_proof.set(text)
        if hasattr(self, "_value_labels"):
            self._value_labels["Proof"].config(fg=color)

    def _on_speed(self, val):
        # Higher slider value = faster (smaller delay between steps).
        v = float(val)
        self.delay = int(round(800 - (v - 1) * 800 / 99))

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
        self.canvas.delete("all")
        self._draw_grid()
        if not self.maze:
            return
        self._draw_edges()
        self._draw_stubs()
        self._draw_optimal()
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
        for u in self.robot.visited_nodes:
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
                                        width=6, dash=(5, 3), capstyle=tk.ROUND)

    def _draw_optimal(self):
        if self.phase not in ("FAST_RUN", "DONE") or not self.optimal_path:
            return
        for a, b in zip(self.optimal_path, self.optimal_path[1:]):
            x1, y1 = self.world_to_canvas(*self.maze.pos[a])
            x2, y2 = self.world_to_canvas(*self.maze.pos[b])
            done = tuple(sorted((a, b))) in self.fast_traversed
            self.canvas.create_line(x1, y1, x2, y2, fill=OPTIMAL_EDGE,
                                    width=14 if done else 8, capstyle=tk.ROUND)

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
        if self.phase in ("FAST_RUN", "DONE") and self.fast_current is not None:
            node, heading = self.fast_current, self.fast_heading
        else:
            node, heading = self.robot.current, self.robot.heading
        pos = self.maze.pos[node]
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
