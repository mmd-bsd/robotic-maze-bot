#!/usr/bin/env python3
"""
run_brain.py -- Run the decision core (brain.c) against a maze .json.

Usage:
    python scripts/run_brain.py ../simulator/mazes/real_field.json
    python scripts/run_brain.py ../simulator/mazes/sample_maze3.json

What it does:
    1. Reads the maze .json file
    2. Generates test/_maze_data.h   (reusing run_maze.py's generator)
    3. Compiles brain_host.c + brain.c + all solver sources against it
    4. Runs the .exe -- a simulated robot that stops only where the real
       firmware's junction detector would, and reports only what its sensors
       could see
    5. Cleans up the generated header

How this differs from run_maze.py: run_maze.py drives the solver by revealing
the TRUE maze to it (reveal_node()), which skips the sensor-driven discovery
path.  This one drives the brain the way the robot will: exits, a target flag,
and a distance.  The brain is never told a coordinate or a compass heading.

Exit code is 0 only if every check in brain_host.c passed.
"""
import subprocess
import sys
from pathlib import Path

# run_maze.py holds the JSON -> C header generator and the project paths; reuse
# them rather than keeping a second copy in step.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_maze import (                                          # noqa: E402
    generate_header, cleanup, die, DATA_H, BUILD_DIR, INC_DIR, SRC_DIR, TEST_DIR,
)

EXE = BUILD_DIR / "brain_host.exe"

SOURCES = [
    SRC_DIR / "maze_graph.c",
    SRC_DIR / "maze_robot.c",
    SRC_DIR / "maze_explore.c",
    SRC_DIR / "maze_proof.c",
    SRC_DIR / "maze_fastrun.c",
    SRC_DIR / "maze_solver.c",
    SRC_DIR / "brain.c",
    TEST_DIR / "brain_host.c",
]

# Same flags as every other target in the project: the brain must be as clean as
# the modules it borrows.
CFLAGS = [
    "-std=c11", "-Wall", "-Wextra", "-pedantic",
    f"-I{INC_DIR}",
    f"-I{TEST_DIR}",
    "-lm",
]


def main():
    if len(sys.argv) < 2:
        print(f"Usage:  python {Path(__file__).name} <maze.json>")
        print("Example: python scripts/run_brain.py "
              "../simulator/mazes/real_field.json")
        sys.exit(1)

    json_path = Path(sys.argv[1]).resolve()
    print("=== Brain Runner ===")
    print(f"  Maze:  {json_path}")

    n_nodes, n_edges = generate_header(json_path)
    print(f"  Header: {DATA_H}  ({n_nodes} nodes, {n_edges} edges)")

    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    cmd = ["gcc"] + CFLAGS + [str(s) for s in SOURCES] + ["-o", str(EXE)]
    print(f"  [compile] {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        cleanup()
        die(f"compilation failed (exit code {result.returncode})")
    if result.stderr.strip():
        # gcc warnings are a failure here too -- the project is zero-warning
        print(result.stderr, file=sys.stderr)
        print("  (compiler emitted warnings -- treat as failure)", file=sys.stderr)
        cleanup()
        sys.exit(1)

    print(f"  [run] {EXE}")
    print()
    rc = subprocess.run([str(EXE)]).returncode

    cleanup()
    sys.exit(rc)


if __name__ == "__main__":
    main()
