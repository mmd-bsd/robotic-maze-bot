# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Robot maze simulation project implementing various graph-based exploration algorithms. The project simulates a robot navigating through a maze represented as a graph, with visualization of the exploration process and optimal path calculation.

## Project Structure

```
Simulator/
├── py-code/
│   ├── maze solving/        # Main algorithm implementations
│   ├── maze generator/      # Maze creation/editing tools
│   └── test path/          # Integration tests
├── ALGORITHM_DESCRIPTION.md # Detailed algorithm walkthrough
└── .gitignore
```

## Running Simulations

Navigate to `py-code/maze solving/` and run any simulation file:

```bash
cd py-code/maze\ solving/
python sim_V12_GBF_stable.py    # Recommended stable version
```

All simulation files are self-contained and can be run directly with Python 3.x.

## Dependencies

Install required packages:
```bash
pip install networkx matplotlib numpy
```

## Architecture

### Maze Representation
- Mazes are represented as NetworkX graphs with spatial coordinates
- Each node has a position stored in `G.nodes[node]['pos']` as (x, y) tuple
- Edges represent traversable paths between nodes
- START_NODE and TARGET_NODE constants define the navigation goal

### Algorithm Implementations

**Stable Version (Recommended):**
- `sim_V12_GBF_stable.py` - Greedy Best-First with frontier-based exploration

**Other Variants:**
- `sim_V2_LeftHandRuleToEnd.py` - Left-Hand Rule (wall-following)
- `sim_V10_PureDFS_noLHR_notOptimized.py` - Pure DFS exploration
- `sim_V4_LHR_DFS_not_optimized.py` - Hybrid Left-Hand + DFS
- `sim_V11_GreedyBestFirst_v1.py` - Earlier GBF version

### Common Simulation Pattern

All simulation files follow this structure:
1. **Maze Definition** - `nodes_positions`, `edges_list`, START_NODE, TARGET_NODE
2. **Graph Creation** - NetworkX graph with positional data
3. **Visualization** - `draw_maze()` function using matplotlib
4. **Robot Class** - Encapsulates exploration logic with `move()` method
5. **Main Loop** - Iterative exploration with `plt.pause()` for animation

### Frontier-Based Exploration Algorithm

The primary algorithm (V11, V12) uses a priority system:

1. **Priority 1**: Explore unexplored neighbors from current node
2. **Priority 2**: Find nearest frontier (visited node with unexplored neighbors)
3. **Priority 3**: Return to start when all frontiers exhausted

Key concepts:
- **Known Map**: Subset of graph explored (tracked in `robot.visited_nodes`, `robot.visited_edges`)
- **Frontier**: Visited node with at least one unexplored adjacent edge
- **Orange stubs**: Visual indicators of unexplored paths from visited nodes

### Animation Control

Adjust `PRINT_SPEED` constant (in seconds) to control animation speed:
```python
PRINT_SPEED = 0.3  # Default: 300ms per step
PRINT_SPEED = 0    # Fast: No delay
PRINT_SPEED = 1.0  # Slow: 1 second per step
```

## Creating Custom Mazes

Use the maze editor tools:
```bash
cd py-code/maze\ generator/
python maze_editor_V3.py
```

Controls:
- **Left-click**: Add nodes and edges
- **Key press 's'**: Set start node
- **Key press 't'**: Set target node
- **Key press 'e'**: Export maze to console (copy-paste into simulation file)

## Visualization Legend

- **Green node**: Start position
- **Black node**: Target position
- **Red node**: Current robot position
- **Light blue nodes**: All maze nodes
- **Grey edges**: Unexplored paths
- **Blue edges**: Visited/Explored paths
- **Orange dashed stubs**: Unexplored paths visible from visited nodes
- **Green wide path**: Final optimal path (calculated after exploration)

## Key Design Decisions

**Why multiple simulation versions?**
- Each file represents a different algorithmic approach or optimization iteration
- Version numbers track evolution from simple to sophisticated algorithms
- V12 is the most stable and well-documented version

**Why continue exploration after finding target?**
- Algorithm performs complete mapping before calculating optimal path
- Target discovery sets flag but doesn't terminate exploration
- Optimal path calculated using Dijkstra's algorithm on full graph after exploration

**Why NetworkX?**
- Provides efficient graph algorithms (shortest path, BFS)
- Easy graph manipulation and querying
- Built-in visualization support (though we use matplotlib directly)

## Common Modifications

**Change maze structure:**
- Modify `nodes_positions` and `edges_list` in any simulation file
- Maintain consistency: ensure all edges reference valid nodes

**Adjust exploration behavior:**
- Modify `Robot.move()` method priority logic
- Change `get_unexplored_neighbors()` to implement custom heuristics

**Create new algorithm variant:**
- Copy `sim_V12_GBF_stable.py` as template
- Modify `Robot.move()` to implement different strategy
- Update title in `draw_maze()` call

## Testing

Integration tests located in `py-code/test path/`. These validate the complete exploration and pathfinding pipeline.
