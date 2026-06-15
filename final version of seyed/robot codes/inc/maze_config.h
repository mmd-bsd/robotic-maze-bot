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
 * The worst-case memory footprint for the maze solver is ~3.0 KB with the
 * defaults below, leaving ~5.0 KB for the firmware's own globals, stack, and
 * HAL buffers.
 *
 *     Component               Default    Bytes
 *     ─────────────────────   ────────   ─────
 *     Nodes (50 × 10 B)       MAX_NODES     500
 *     Edges (100 × 10 B)      MAX_EDGES    1000
 *     Adjacency (50 × 4 × 2)  —             400
 *     Dijkstra arrays          —             400
 *     Path buffers (50 × 2B)  —             100
 *     Command log              —             512
 *     Trajectory points        —             200
 *     ─────────────────────────────────────────
 *     TOTAL                                ~3112
 *============================================================================*/

/** Maximum maze intersections the robot can discover. */
#ifndef MAZE_MAX_NODES
#define MAZE_MAX_NODES          50
#endif

/** Maximum traversable line segments between nodes. */
#ifndef MAZE_MAX_EDGES
#define MAZE_MAX_EDGES          100
#endif

/** Maximum path length (nodes) for navigation buffers. */
#ifndef MAZE_MAX_PATH_LENGTH
#define MAZE_MAX_PATH_LENGTH    50
#endif

/** Maximum number of frontier nodes to track concurrently. */
#ifndef MAZE_MAX_FRONTIERS
#define MAZE_MAX_FRONTIERS      20
#endif

/** Maximum trajectory points for the fast-run acceleration profile. */
#ifndef MAZE_MAX_TRAJECTORY
#define MAZE_MAX_TRAJECTORY     200
#endif

/** Command-log buffer size (F/L/R/B characters). */
#ifndef MAZE_COMMAND_LOG_SIZE
#define MAZE_COMMAND_LOG_SIZE   512
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
 *  loops if something goes wrong with sensor detection).  For a 50-node
 *  maze, 1000 steps is generous (allows backtracking). */
#ifndef MAZE_EXPLORATION_TIMEOUT
#define MAZE_EXPLORATION_TIMEOUT 1000
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
