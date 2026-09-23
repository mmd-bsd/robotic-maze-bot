/**
 * @file maze_config.h
 * @brief Compile-time configuration for the SEYED maze solver (C port).
 *
 * Every tunable — memory limits, motion-model parameters, algorithm thresholds,
 * and platform-specific defines — lives here so it can be adjusted without
 * touching the algorithm source.
 *
 * All physical values are in real-world units: 1 unit = 1 cm.
 * Speeds are cm/s, accelerations are cm/s².
 */

#ifndef MAZE_CONFIG_H
#define MAZE_CONFIG_H

#include <stdint.h>

/*============================================================================
 * MEMORY CONSTRAINTS (target: STM32G031G8Ux, 8KB RAM, 64KB flash)
 *
 * The solver owns TWO static globals (in maze_hal.h): one MazeGraph and one
 * MazeRobot.  Everything else it needs is stack scratch.  Real ARM32 sizes with
 * the defaults below:
 *
 *     Component                       Count        Bytes
 *     ─────────────────────────────   ──────────   ─────
 *     MazeGraph.nodes  (6 B each)     64            384
 *     MazeGraph.edges  (8 B each)     112           896
 *     MazeGraph scalars                4             8
 *     MazeRobot.visited_nodes          64            64
 *     MazeRobot.explored_edges         112          112
 *     MazeRobot.command_log            192          192
 *     MazeRobot.path_buffer  (2 B)     64           128
 *     MazeRobot.fast_path_nodes (2 B)  64           128
 *     MazeRobot scalars/pointers       —             44
 *     ─────────────────────────────────────────────────────
 *     STATIC TOTAL                                1956  (~1.9 KB)
 *
 *     Peak stack scratch (NOT counted above — it is per-call):
 *     maze_time_optimal_path()  5 arrays x 64       704   <-- the worst frame
 *     MinHeap inside maze_graph_dijkstra  64 x 8    512
 *     frontier arrays in maze_explore     2 x 24 x 2 96
 *     path/rev/expanded scratch (4 x 64 x 2)        512
 *     call chain + locals                 —        ~300
 *============================================================================*/

/*----------------------------------------------------------------------------
 * WHY THESE NUMBERS — the field is 8 x 7
 *
 * The real board is a fixed 8 x 7 grid = 56 lattice points, and
 * simulator/mazes/real_field.json uses 38 of them with 41 edges.  The limits
 * below are sized to that board, not to the simulator's worst-case maze.
 *
 * It is tempting to treat this as cosmetic.  It is not, for two reasons:
 *
 *  1. STATIC RAM.  Every limit here is multiplied by a small constant, so the
 *     whole config costs 1956 B instead of 2628 B — 672 B that the 8 KB part
 *     does not have.  The firmware alone was already at 93% before the solver.
 *
 *  2. PEAK STACK, which is the real reason.  maze_time_optimal_path() keeps
 *     five MAZE_MAX_NODES-sized arrays live at once — at 80 nodes that is a
 *     single 880-byte stack frame against a 1024-byte reservation.  A stack
 *     overflow on Cortex-M0+ is silent corruption, not a clean error, so it
 *     would not show up until the robot behaved inexplicably on the field.
 *     At 64 nodes the same frame is 704 B.
 *
 * Sizing the arrays to the physical board is legitimate and is what makes the
 * fit safe.  Hard-coding the route would not be: see rule 1 in CLAUDE.md — the
 * robot must not use the target's position before target_found, and a baked-in
 * route also fails if the field is laid out differently on competition day.
 *----------------------------------------------------------------------------*/

/** Maximum maze intersections the robot can discover.
 *  64 = the 56 lattice points of the 8 x 7 board, plus ~14% for the placeholder
 *  nodes the solver sprouts at 20 cm offsets for each detected open branch
 *  (a spurious sensor reading can put one off-lattice).  80 gave 43% headroom
 *  over the same board and cost 96 B of static + 96 B of peak stack — margin
 *  the 8 KB part cannot pay for. */
#ifndef MAZE_MAX_NODES
#define MAZE_MAX_NODES          64
#endif

/** Maximum traversable line segments between nodes.
 *  A fully-connected 8 x 7 grid has 8x6 + 7x7 = 97 edges, so 112 covers the
 *  whole board with room for a few spurious ones.  The real field uses 41. */
#ifndef MAZE_MAX_EDGES
#define MAZE_MAX_EDGES          112
#endif

/** Maximum path length (nodes) for navigation buffers.
 *  A shortest path never visits a node twice, so this tracks MAZE_MAX_NODES.
 *  Also drives FOUR stack arrays (maze_graph rev[], maze_explore path[],
 *  maze_fastrun expanded[]/path_nodes[]), so it is not a struct-only knob. */
#ifndef MAZE_MAX_PATH_LENGTH
#define MAZE_MAX_PATH_LENGTH    64
#endif

/** Maximum number of frontier nodes to track concurrently.
 *  This array is a STACK local, so keep it small.
 *
 *  DELIBERATELY LEFT AT 24 — do not shrink this to save the 32 B.  The only
 *  writer, maze_robot_find_frontiers(), truncates SILENTLY at this limit
 *  (no error, no count flag), and the truncated list is what maze_explore.c
 *  hands to the early-stop proof.  A frontier dropped by truncation can make
 *  the proof conclude that nothing useful remains and stop the search early —
 *  i.e. return a non-optimal route while claiming optimal_proven.  That is the
 *  exact failure the proof exists to prevent, and it would be silent.  32 B is
 *  not worth it. */
#ifndef MAZE_MAX_FRONTIERS
#define MAZE_MAX_FRONTIERS      24
#endif

/** Maximum trajectory points for the fast-run acceleration profile.
 *  Largest number of trajectory points the fast-run planner may emit. */
#ifndef MAZE_MAX_TRAJECTORY
#define MAZE_MAX_TRAJECTORY     80
#endif

/** Command-log buffer size (F/L/R/B characters).
 *  Measured, not guessed: a full mission on the real field (38 nodes) is 70
 *  commands, and the worst simulator maze tried (sample_maze4, 37 nodes) is 97
 *  — i.e. up to ~2.6 commands per node.  192 covers 64 nodes at that worst
 *  observed ratio (168) with room to spare.  Overflow is SAFE but silent:
 *  maze_robot_log_command() just stops recording
 *  (src/maze_robot.c:335), so a truncated log loses the mission record without
 *  affecting the route. */
#ifndef MAZE_COMMAND_LOG_SIZE
#define MAZE_COMMAND_LOG_SIZE   192
#endif

/*============================================================================
 * MOTION MODEL (acceleration-aware fast run)
 *
 * These MUST match the values used in the exploration early-stop proof, or
 * the proof becomes inadmissible.  The Python simulator defaults are:
 *
 *     DEFAULT_VMAX   = 100.0 cm/s
 *     DEFAULT_ACCEL  =  50.0 cm/s²
 *     EXPLORE_SPEED  =  40.0 cm/s
 *
 * The robot accelerates on clear straights and comes to a full stop (v = 0)
 * at every turn (turns are modelled as a stop — see ARCHITECTURE.md §6).
 *
 * All speeds/accels use FIXED-POINT with 2 decimal places so the Cortex-M0+
 * (no FPU) can compute run_time() and speed_at() with integer math.
 *============================================================================*/

/** Robot top speed during the fast run (cm/s * 100, i.e. 10000 = 100.00 cm/s). */
#ifndef MAZE_V_MAX_FP
#define MAZE_V_MAX_FP           10000   /* 100.00 cm/s */
#endif

/** Robot maximum acceleration / deceleration (cm/s² * 100). */
#ifndef MAZE_ACCEL_FP
#define MAZE_ACCEL_FP           5000    /* 50.00 cm/s² */
#endif

/** Constant cautious speed used during exploration (cm/s * 100). */
#ifndef MAZE_EXPLORE_SPEED_FP
#define MAZE_EXPLORE_SPEED_FP   4000    /* 40.00 cm/s */
#endif

/** Fixed-point scale factor (precision: 2 decimal places). */
#define MAZE_FP_SCALE           100

/*============================================================================
 * ALGORITHM THRESHOLDS
 *============================================================================*/

/** Coordinate matching tolerance (cm).  Two coordinates within this distance
 *  are considered the same node.  Should be smaller than half the minimum
 *  edge length to avoid false merges. */
#ifndef MAZE_COORD_TOLERANCE_CM
#define MAZE_COORD_TOLERANCE_CM 3
#endif

/** Maximum exploration steps before a safety timeout (prevents infinite
 *  loops if something goes wrong with sensor detection).  For a 100-node
 *  maze, 2000 steps is generous (allows backtracking). */
#ifndef MAZE_EXPLORATION_TIMEOUT
#define MAZE_EXPLORATION_TIMEOUT 2000
#endif

/** Delay (in sensor read cycles, ~2ms each) before the robot decides
 *  the straight path has genuinely ended (dead-end detection).  The
 *  firmware uses head_delay >= 50 (100ms); same here. */
#ifndef MAZE_DEAD_END_DELAY
#define MAZE_DEAD_END_DELAY     50
#endif

/** Minimum edge length to record (cm).  Edges shorter than this are
 *  probably sensor noise / false intersections. */
#ifndef MAZE_MIN_EDGE_LENGTH_CM
#define MAZE_MIN_EDGE_LENGTH_CM 5
#endif

/** Physical pitch of the maze lattice (cm) — the distance between two adjacent
 *  lattice intersections on the real field.  1 world unit = 1 cm.
 *
 *  SINGLE SOURCE OF TRUTH for the cell size.  Two consumers:
 *
 *   - `maze_solver.c` uses it as PLACEHOLDER_DIST_CM, the offset at which it
 *     sprouts a provisional neighbour when a sensor reading reports an open
 *     branch.  If a real link is LONGER than this, the provisional node is not
 *     where the robot ends up — but that is not a bug and needs no correction.
 *     A node the robot drives through without stopping is by definition a
 *     straight passthrough (no side branch fired, so no junction was detected),
 *     so it is collinear, and `maze_solver_update_position()` adds the edge from
 *     the provisional node to the node the robot is ACTUALLY at, computing its
 *     length from coordinates.  The graph therefore grows the intermediate
 *     lattice point itself and stays geometrically exact.
 *   - `brain.c` uses it to snap the reported `dist_cm` to whole cells before
 *     dead-reckoning.  Snapping absorbs encoder error: without it, drift would
 *     walk the robot's coordinate off the lattice, and the early-stop proof —
 *     which is geometric — would start bounding the wrong distances.
 *
 *  Measured on the real field image: 62.5 px pitch at 3.125 px/cm = 20.0 cm,
 *  and all 41 edges of simulator/mazes/real_field.json are exactly 20.0 cm. */
#ifndef MAZE_CELL_CM
#define MAZE_CELL_CM            20
#endif

/*============================================================================
 * DEBUG & DIAGNOSTICS
 *============================================================================*/

/** Set to 1 to enable debug_print() output via UART / semihosting.
 *  Set to 0 for production builds (saves flash & UART bandwidth). */
#ifndef MAZE_DEBUG_ENABLED
#define MAZE_DEBUG_ENABLED      0
#endif

#if MAZE_DEBUG_ENABLED
    /** Override this macro in your platform HAL to route debug messages. */
    #ifndef MAZE_DEBUG_PRINT
    #define MAZE_DEBUG_PRINT(fmt, ...) \
        do { /* default: no-op — define in platform HAL */ } while (0)
    #endif
#else
    #define MAZE_DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

/*============================================================================
 * COMPILE-TIME VALIDATION
 *============================================================================*/

#if MAZE_MAX_NODES > 255
#error "MAZE_MAX_NODES must fit in uint8_t indices — max 255"
#endif

#if MAZE_MAX_EDGES > 500
#error "MAZE_MAX_EDGES too large for adjacency precomputation buffer"
#endif

#if MAZE_V_MAX_FP <= 0 || MAZE_ACCEL_FP <= 0
#error "Motion-model parameters must be positive"
#endif

#if MAZE_V_MAX_FP / MAZE_ACCEL_FP > 10000
#error "v_max / a_max ratio is unreasonably large — check units (cm/s, cm/s²)"
#endif

#endif /* MAZE_CONFIG_H */
