"""
Maze Generator (Final Version - SEYED)
======================================

A comfortable, modern (dark-mode) Tkinter GUI for building line-mazes by clicking.

Workflow (same pattern as the old maze_editor_V3, but as a full app):
  - Left-click an empty grid intersection  -> selects it (yellow).
  - Left-click a second intersection        -> draws an edge (only horizontal
                                                or vertical, because the robot
                                                only follows 90-degree cross lines).
  - Left-click ON an existing edge           -> splits that edge at the clicked
                                                point and selects the new node.
  - Left-click the same point again          -> deselects it.

Set special points (select a node first, then click the button or press the key):
  - [S] Set Start  (green circle)
  - [T] Set Target (black circle - the big black area on the real field)
  - [D] Delete the selected node and all its edges

Navigation:
  - Mouse wheel        -> zoom in / out (around the cursor)
  - Middle/Right drag  -> pan the view
  - "Fit View" button  -> recenter everything

Saving / Loading:
  - Save Maze...  -> writes a .json file (this is what the solver loads).
  - Load Maze...  -> reopen a .json file to keep editing.

Requires only the Python standard library (tkinter).
"""

import json
import os
import tkinter as tk
from tkinter import filedialog, messagebox

try:
    from PIL import Image, ImageDraw, ImageTk
    _HAS_PIL = True
except Exception:  # noqa: BLE001
    _HAS_PIL = False

# ----------------------------- Configuration ------------------------------- #
GRID_SIZE = 20
DEFAULT_SCALE = 2.0
NODE_RADIUS = 6
HIT_RADIUS = 12

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
EDGE_COLOR = "#7c83a0"
NODE_COLOR = "#aab2d6"
NODE_OUTLINE = "#0d0f15"
START_COLOR = "#3ddc84"
TARGET_FILL = "#0a0a0a"
TARGET_RING = "#ff5d6c"
SELECT_COLOR = "#ffd54a"
SELECT_OUTLINE = "#ff9f1c"
BTN_BG = "#262c3d"
BTN_ACTIVE = "#343c54"

FONT = ("Segoe UI", 10)
FONT_BOLD = ("Segoe UI", 10, "bold")
FONT_TITLE = ("Segoe UI", 15, "bold")
FONT_MONO = ("Consolas", 10)

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MAZES_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "mazes"))


def is_point_on_segment(point, end1, end2):
    px, py = point
    x1, y1 = end1
    x2, y2 = end2
    return (min(x1, x2) <= px <= max(x1, x2) and
            min(y1, y2) <= py <= max(y1, y2))


class MazeGenerator:
    def __init__(self, root):
        self.root = root
        self.root.title("Maze Generator - SEYED")
        self.root.configure(bg=BG)

        self.edges = set()
        self.start_pos = None
        self.target_pos = None
        self.selected_pos = None

        self.scale = DEFAULT_SCALE
        self.offset_x = 0.0
        self.offset_y = 0.0
        self._pan_anchor = None
        self._needs_fit = True
        self._img_cache = {}

        self._build_ui()
        self.root.update_idletasks()
        self.fit_view()
        self.redraw()

    # --------------------------------------------------------------------- #
    #  UI
    # --------------------------------------------------------------------- #
    def _build_ui(self):
        self.status = tk.StringVar(value="Click an intersection to begin.")
        tk.Label(self.root, textvariable=self.status, anchor="w",
                 bg=CARD_BG, fg=FG_MUTED, padx=10, pady=5, font=FONT).pack(
            side=tk.BOTTOM, fill=tk.X)

        main = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=BG,
                              sashwidth=7, sashrelief=tk.FLAT, bd=0, sashpad=0,
                              opaqueresize=True)
        main.pack(fill=tk.BOTH, expand=True)

        canvas_holder = tk.Frame(main, bg=BG)
        self.canvas = tk.Canvas(canvas_holder, bg=CANVAS_BG,
                                highlightthickness=0, bd=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=(10, 0), pady=10)
        self.canvas.bind("<Button-1>", self.on_left_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<MouseWheel>", self.on_zoom)
        self.canvas.bind("<Button-4>", self.on_zoom)
        self.canvas.bind("<Button-5>", self.on_zoom)
        self.canvas.bind("<ButtonPress-2>", self.on_pan_start)
        self.canvas.bind("<B2-Motion>", self.on_pan_move)
        self.canvas.bind("<ButtonPress-3>", self.on_pan_start)
        self.canvas.bind("<B3-Motion>", self.on_pan_move)
        self.canvas.bind("<Configure>", self._on_configure)

        self.root.bind("s", lambda e: self.set_start())
        self.root.bind("t", lambda e: self.set_target())
        self.root.bind("d", lambda e: self.delete_selected())
        self.root.bind("<Delete>", lambda e: self.delete_selected())
        self.root.bind("<Control-s>", lambda e: self.save_maze())
        self.root.bind("<Control-o>", lambda e: self.load_maze())

        panel = tk.Frame(main, bg=PANEL_BG)
        main.add(canvas_holder, stretch="always", minsize=380)
        main.add(panel, minsize=230, width=270)

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

        tk.Label(pad, text="Maze Generator", font=FONT_TITLE, bg=PANEL_BG,
                 fg=FG).pack(anchor="w")
        tk.Label(pad, text="SEYED \u2022 Final Version", font=("Segoe UI", 9),
                 bg=PANEL_BG, fg=ACCENT).pack(anchor="w", pady=(0, 8))

        self._section(pad, "Special points")
        self._button(pad, "Set Start", self.set_start, "S")
        self._button(pad, "Set Target", self.set_target, "T")
        self._button(pad, "Delete node", self.delete_selected, "D")

        self._section(pad, "Map")
        self._button(pad, "Clear All", self.clear_all)
        self._button(pad, "Fit View", lambda: (self.fit_view(), self.redraw()))

        self._section(pad, "File")
        self._button(pad, "Save Maze\u2026", self.save_maze, "Ctrl+S")
        self._button(pad, "Load Maze\u2026", self.load_maze, "Ctrl+O")

        self._section(pad, "Legend")
        self._legend(pad, START_COLOR, "Start", "circle")
        self._legend(pad, TARGET_FILL, "Target (black area)", "target")
        self._legend(pad, NODE_COLOR, "Node / junction", "circle")
        self._legend(pad, SELECT_COLOR, "Selected", "circle")

        self.coord_var = tk.StringVar(value="")
        tk.Label(pad, textvariable=self.coord_var, bg=PANEL_BG, fg=FG_MUTED,
                 font=FONT_MONO).pack(anchor="w", pady=(12, 0))

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

    def _section(self, parent, text):
        tk.Label(parent, text=text.upper(), font=("Segoe UI", 8, "bold"),
                 bg=PANEL_BG, fg=FG_MUTED).pack(anchor="w", pady=(14, 6))

    def _button(self, parent, text, cmd, hint=None):
        label = text if not hint else f"{text}"
        btn = tk.Button(parent, text=label, command=cmd, font=FONT,
                        bg=BTN_BG, fg=FG, activebackground=BTN_ACTIVE,
                        activeforeground=FG, relief=tk.FLAT, bd=0, anchor="w",
                        padx=12, pady=7, cursor="hand2", highlightthickness=0)
        btn.pack(fill=tk.X, pady=3)
        btn.bind("<Enter>", lambda e: btn.config(bg=BTN_ACTIVE))
        btn.bind("<Leave>", lambda e: btn.config(bg=BTN_BG))
        if hint:
            tk.Label(btn, text=hint, font=("Segoe UI", 8), bg=BTN_BG,
                     fg=FG_MUTED).place(relx=1.0, rely=0.5, anchor="e", x=-8)
        return btn

    def _legend(self, parent, color, text, shape):
        row = tk.Frame(parent, bg=PANEL_BG)
        row.pack(fill=tk.X, anchor="w", pady=2)
        c = tk.Canvas(row, width=18, height=18, bg=PANEL_BG, highlightthickness=0)
        if shape == "target":
            c.create_oval(3, 3, 15, 15, fill=TARGET_FILL, outline=TARGET_RING,
                          width=2)
        else:
            c.create_oval(3, 3, 15, 15, fill=color, outline="")
        c.pack(side=tk.LEFT)
        tk.Label(row, text=text, bg=PANEL_BG, fg=FG, font=FONT).pack(
            side=tk.LEFT, padx=6)

    # --------------------------------------------------------------------- #
    #  Transforms
    # --------------------------------------------------------------------- #
    def _on_configure(self, event=None):
        if self._needs_fit and self.canvas.winfo_width() > 1:
            self.fit_view()
            self._needs_fit = False
        self.redraw()

    def world_to_canvas(self, wx, wy):
        return self.offset_x + wx * self.scale, self.offset_y - wy * self.scale

    def canvas_to_world(self, cx, cy):
        return (cx - self.offset_x) / self.scale, (self.offset_y - cy) / self.scale

    def snap(self, wx, wy):
        return (int(round(wx / GRID_SIZE) * GRID_SIZE),
                int(round(wy / GRID_SIZE) * GRID_SIZE))

    def fit_view(self):
        w = self.canvas.winfo_width() or 830
        h = self.canvas.winfo_height() or 730
        positions = self._all_positions()
        if not positions:
            self.scale = DEFAULT_SCALE
            self.offset_x, self.offset_y = w / 2, h / 2
            return
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        span_x = max(max(xs) - min(xs), GRID_SIZE)
        span_y = max(max(ys) - min(ys), GRID_SIZE)
        margin = 70
        self.scale = max(0.2, min((w - 2 * margin) / span_x,
                                  (h - 2 * margin) / span_y, 12))
        cx = (min(xs) + max(xs)) / 2
        cy = (min(ys) + max(ys)) / 2
        self.offset_x = w / 2 - cx * self.scale
        self.offset_y = h / 2 + cy * self.scale

    # --------------------------------------------------------------------- #
    #  Events
    # --------------------------------------------------------------------- #
    def on_mouse_move(self, event):
        sx, sy = self.snap(*self.canvas_to_world(event.x, event.y))
        self.coord_var.set(f"cursor  ({sx}, {sy})")

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

    def on_left_click(self, event):
        clicked = self.snap(*self.canvas_to_world(event.x, event.y))
        scx, scy = self.world_to_canvas(*clicked)
        tol = max(HIT_RADIUS, GRID_SIZE * self.scale / 2 + 4)
        if abs(scx - event.x) > tol or abs(scy - event.y) > tol:
            return

        edge = self._find_edge_for_split(clicked)
        if edge:
            p1, p2 = edge
            self.edges.discard(edge)
            self.edges.add(tuple(sorted((p1, clicked))))
            self.edges.add(tuple(sorted((clicked, p2))))
            self.selected_pos = clicked
            self.set_status(f"Split edge at {clicked}.")
            self.redraw()
            return

        if self.selected_pos is None:
            self.selected_pos = clicked
            self.set_status(f"Selected {clicked}. Click another point to draw an edge.")
        elif self.selected_pos == clicked:
            self.selected_pos = None
            self.set_status("Deselected.")
        else:
            self._try_create_edge(self.selected_pos, clicked)
            self.selected_pos = None
        self.redraw()

    # --------------------------------------------------------------------- #
    #  Editing
    # --------------------------------------------------------------------- #
    def _try_create_edge(self, p1, p2):
        if p1[0] != p2[0] and p1[1] != p2[1]:
            self.set_status("Edge must be horizontal or vertical (90-deg maze).")
            return
        edge = tuple(sorted((p1, p2)))
        if edge in self.edges:
            self.set_status("Edge already exists.")
            return
        self.edges.add(edge)
        self.set_status(f"Edge created {p1} - {p2}.")

    def _find_edge_for_split(self, clicked):
        for (p1, p2) in self.edges:
            if clicked in (p1, p2):
                continue
            dx1, dy1 = clicked[0] - p1[0], clicked[1] - p1[1]
            dx2, dy2 = p2[0] - p1[0], p2[1] - p1[1]
            if (dx1 * dy2 - dy1 * dx2) == 0 and is_point_on_segment(clicked, p1, p2):
                return (p1, p2)
        return None

    def set_start(self):
        if not self.selected_pos:
            self.set_status("Select a node first, then Set Start.")
            return
        if self.selected_pos == self.target_pos:
            self.set_status("That point is the target. Pick another point.")
            return
        self.start_pos = self.selected_pos
        self.selected_pos = None
        self.set_status(f"Start set at {self.start_pos}.")
        self.redraw()

    def set_target(self):
        if not self.selected_pos:
            self.set_status("Select a node first, then Set Target.")
            return
        if self.selected_pos == self.start_pos:
            self.set_status("That point is the start. Pick another point.")
            return
        self.target_pos = self.selected_pos
        self.selected_pos = None
        self.set_status(f"Target set at {self.target_pos}.")
        self.redraw()

    def delete_selected(self):
        if not self.selected_pos:
            self.set_status("Select a node first, then Delete.")
            return
        p = self.selected_pos
        removed = {e for e in self.edges if p in e}
        self.edges.difference_update(removed)
        if self.start_pos == p:
            self.start_pos = None
        if self.target_pos == p:
            self.target_pos = None
        self.selected_pos = None
        self.set_status(f"Deleted node {p} and {len(removed)} edge(s).")
        self.redraw()

    def clear_all(self):
        if not (self.edges or self.start_pos or self.target_pos):
            return
        if not messagebox.askyesno("Clear All", "Delete the entire maze?"):
            return
        self.edges.clear()
        self.start_pos = self.target_pos = self.selected_pos = None
        self.set_status("Cleared.")
        self.redraw()

    # --------------------------------------------------------------------- #
    #  Save / Load
    # --------------------------------------------------------------------- #
    def _all_positions(self):
        positions = set()
        if self.start_pos:
            positions.add(self.start_pos)
        if self.target_pos:
            positions.add(self.target_pos)
        for p1, p2 in self.edges:
            positions.add(p1)
            positions.add(p2)
        return positions

    def build_maze_dict(self):
        positions = sorted(self._all_positions())
        pos_to_id = {pos: i for i, pos in enumerate(positions)}
        return {
            "grid_size": GRID_SIZE,
            "nodes": {str(i): list(pos) for pos, i in pos_to_id.items()},
            "edges": [[pos_to_id[p1], pos_to_id[p2]] for (p1, p2) in self.edges],
            "start": pos_to_id.get(self.start_pos),
            "target": pos_to_id.get(self.target_pos),
        }

    def save_maze(self):
        if not self.edges:
            self.set_status("Nothing to save - draw some edges first.")
            return
        os.makedirs(MAZES_DIR, exist_ok=True)
        path = filedialog.asksaveasfilename(
            title="Save Maze", initialdir=MAZES_DIR, defaultextension=".json",
            filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        data = self.build_maze_dict()
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        warn = ""
        if data["start"] is None:
            warn += " (no START set!)"
        if data["target"] is None:
            warn += " (no TARGET set!)"
        self.set_status(f"Saved {os.path.basename(path)}.{warn}")

    def load_maze(self):
        path = filedialog.askopenfilename(
            title="Load Maze",
            initialdir=MAZES_DIR if os.path.isdir(MAZES_DIR) else _THIS_DIR,
            filetypes=[("Maze JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            nodes = {int(k): tuple(v) for k, v in data["nodes"].items()}
            self.edges = {tuple(sorted((nodes[a], nodes[b]))) for a, b in data["edges"]}
            self.start_pos = tuple(nodes[data["start"]]) if data.get("start") is not None else None
            self.target_pos = tuple(nodes[data["target"]]) if data.get("target") is not None else None
            self.selected_pos = None
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Load failed", f"Could not load maze:\n{exc}")
            return
        self._needs_fit = True
        self.fit_view()
        self.redraw()
        self.set_status(f"Loaded {os.path.basename(path)}.")

    # --------------------------------------------------------------------- #
    #  Drawing
    # --------------------------------------------------------------------- #
    def set_status(self, text):
        self.status.set(text)

    def redraw(self):
        self.canvas.delete("all")
        self._draw_grid()
        self._draw_edges()
        self._draw_nodes()

    def _draw_grid(self):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        if w <= 1 or h <= 1:
            return
        wx0, wy1 = self.canvas_to_world(0, 0)
        wx1, wy0 = self.canvas_to_world(w, h)
        step = GRID_SIZE
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
        for (p1, p2) in self.edges:
            x1, y1 = self.world_to_canvas(*p1)
            x2, y2 = self.world_to_canvas(*p2)
            self.canvas.create_line(x1, y1, x2, y2, fill=EDGE_COLOR, width=7,
                                    capstyle=tk.ROUND)

    def _draw_nodes(self):
        for pos in self._all_positions():
            self._disc(pos, NODE_COLOR, NODE_RADIUS, outline=NODE_OUTLINE, ow=2)
        if self.start_pos:
            self._ring(self.start_pos, START_COLOR, NODE_RADIUS + 4, 2)
            self._disc(self.start_pos, START_COLOR, NODE_RADIUS + 3,
                       outline=CANVAS_BG, ow=2)
        if self.target_pos:
            self._target(self.target_pos)
        if self.selected_pos:
            self._ring(self.selected_pos, SELECT_OUTLINE, NODE_RADIUS + 5, 2)
            self._disc(self.selected_pos, SELECT_COLOR, NODE_RADIUS + 3,
                       outline=SELECT_OUTLINE, ow=2)

    def _target(self, pos):
        r = NODE_RADIUS + 6
        self._ring(pos, TARGET_RING, r + 4, 2)
        self._disc(pos, TARGET_FILL, r, outline=TARGET_RING, ow=3)

    @staticmethod
    def _hex_to_rgba(color):
        h = color.lstrip("#")
        return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), 255)

    def _circle_image(self, fill, outline, r, ow):
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
        img = img.resize((big // ss, big // ss), Image.LANCZOS)
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
    root.geometry("1180x800")
    MazeGenerator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
