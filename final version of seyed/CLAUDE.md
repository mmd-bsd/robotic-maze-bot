# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project

**SEYED** — a line-following maze robot. The robot drives along a 90° grid maze, searches for a target (big black area), proves it found the fastest route, returns to start, then races the fastest path. The fastest path minimises **time** (not distance) under an acceleration model — the robot stops at every turn and accelerates on clear straights.

**Two parts:** (1) Python/Tkinter simulator (active), (2) STM32/C firmware (future).

**Canonical docs** (read these first in any session):
- [`ARCHITECTURE.md`](./ARCHITECTURE.md) — layout, rules, conventions, glossary
- [`ALGORITHMS.md`](./ALGORITHMS.md) — every algorithm in detail
- [`CHANGELOG.md`](./CHANGELOG.md) — worklog + hard-won lessons; **update it whenever you change code**

---

## How to run

```powershell
python "final version of seyed/simulator/maze solver/maze_solver.py"
python "final version of seyed/simulator/maze generator/maze_generator.py"
```

**Dependencies:** Python 3.x with tkinter (stdlib). Pillow (`pip install pillow`) optional — anti-aliased circles fall back gracefully.

---

## Key rules

1. **No cheating during search** — the robot must not use the target's position until `target_found` is True.
2. **All bounds/pruning use TIME**, not distance (acceleration model — `v_max` matters).
3. **Work only in `final version of seyed/simulator/`.** `New Start/` and `old docs/` are read-only history.
4. **Shell is PowerShell on Windows** — chain with `;`, quote paths with spaces.
5. **After any code change**, append an entry to `CHANGELOG.md`. Keep `ARCHITECTURE.md`/`ALGORITHMS.md` in sync.
6. **Units are real-world:** 1 world unit = 1 cm. Speeds cm/s, accel cm/s².
7. **Prefer standard library**, single-file scripts. No `networkx`/`matplotlib` in the final-version simulator.
8. **Dark theme** — reuse the color constants at the top of each file.

---

## Headless testing pattern

```python
import importlib, tkinter as tk
root = tk.Tk(); root.withdraw()
SolverApp = importlib.import_module("maze solver.maze_solver").SolverApp
app = SolverApp(root)
while app.phase != "DONE":
    app._frame(dt)
```

This runs the full mission without `mainloop()` — no display needed. Call `sys.stdout.reconfigure(encoding='utf-8')` before printing unicode.
