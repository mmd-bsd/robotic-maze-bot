#ifndef MAZE_CONFIG_H
#define MAZE_CONFIG_H

/**
 * @file maze_config.h
 * @brief Hardware and algorithm configuration parameters
 *
 * This file contains all tunable parameters for the maze navigation
 * system including memory constraints, algorithm tuning, and
 * hardware-specific settings.
 */

/*============================================================================
 * MEMORY CONSTRAINTS
 *============================================================================*/

/**
 * @brief Maximum number of nodes in the maze
 *
 * This limits the size of the maze that can be explored.
 * Adjust based on available RAM. For STM32 with >8KB RAM,
 * 50 nodes is reasonable (approximately 350 bytes for node storage).
 *
 * Memory usage per node: ~7 bytes
 * Total for 50 nodes: ~350 bytes
 */
#define MAZE_MAX_NODES          50

/**
 * @brief Maximum number of edges in the maze
 *
 * In a typical maze, each node connects to 2-4 neighbors.
 * For 50 nodes with average 3 connections, ~75 edges needed.
 *
 * Memory usage per edge: ~5 bytes
 * Total for 100 edges: ~500 bytes
 */
#define MAZE_MAX_EDGES          100

/**
 * @brief Maximum path length for navigation
 *
 * Maximum number of nodes in a path during navigation.
 * Used for temporary path buffers.
 *
 * Memory usage: ~100 bytes (50 × 2 bytes)
 */
#define MAZE_MAX_PATH_LENGTH    50

/*============================================================================
 * ALGORITHM TUNING PARAMETERS
 *============================================================================*/

/**
 * @brief Coordinate matching tolerance
 *
 * Tolerance for matching coordinates when detecting if robot
 * has returned to a previously visited node.
 *
 * Units: Same as coordinate system (typically millimeters)
 * Value: 2 units (2mm tolerance for localization noise)
 */
#define MAZE_COORD_TOLERANCE    2

/**
 * @brief Maximum exploration steps timeout
 *
 * Safety limit to prevent infinite exploration loops.
 * If exploration exceeds this many steps, it will be terminated.
 *
 * For a 50-node maze, 500 steps is generous (allows for backtracking).
 */
#define MAZE_EXPLORATION_TIMEOUT 500

/**
 * @brief Maximum number of frontiers to track
 *
 * Limits the size of the frontier array during exploration.
 * In practice, only a few frontiers exist at any time.
 */
#define MAZE_MAX_FRONTIERS      20

/*============================================================================
 * HARDWARE-SPECIFIC CONFIGURATION
 *============================================================================*/

/**
 * @brief Sensor update debounce delay
 *
 * Delay between sensor reads to debounce and stabilize readings.
 *
 * Units: milliseconds
 */
#define MAZE_SENSOR_UPDATE_DELAY_MS   10

/**
 * @brief Motor movement timeout
 *
 * Maximum time to wait for a movement to complete before
 * checking sensors again.
 *
 * Units: milliseconds
 */
#define MAZE_MOVEMENT_TIMEOUT_MS      5000

/**
 * @brief Debug output enable flag
 *
 * Set to 1 to enable debug output via UART or other interface.
 * Set to 0 to disable debug output for production/release builds.
 */
#define MAZE_DEBUG_ENABLED      1

#if MAZE_DEBUG_ENABLED
/**
 * @brief Debug print macro
 *
 * Override this to implement platform-specific debug output.
 * Default implementation does nothing (define in your platform code).
 */
#ifndef MAZE_DEBUG_PRINT
#define MAZE_DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

#else
#define MAZE_DEBUG_PRINT(fmt, ...) do {} while(0)
#endif

/*============================================================================
 * VALIDATION CONFIGURATION
 *============================================================================*/

/**
 * @brief Enable runtime assertions
 *
 * Set to 1 to enable runtime assertion checks for debugging.
 * Set to 0 for production to reduce code size and improve performance.
 */
#define MAZE_ENABLE_ASSERTS     1

#if MAZE_ENABLE_ASSERTS
/**
 * @brief Assertion macro
 *
 * Checks condition and calls error handler if false.
 * Define MAZE_ASSERT_HANDLER() to implement custom error handling.
 */
#ifndef MAZE_ASSERT
#define MAZE_ASSERT(cond) \
    do { \
        if (!(cond)) { \
            MAZE_DEBUG_PRINT("ASSERTION FAILED: %s:%d\n", __FILE__, __LINE__); \
            MAZE_ASSERT_HANDLER(); \
        } \
    } while(0)
#endif

#ifndef MAZE_ASSERT_HANDLER
#define MAZE_ASSERT_HANDLER() do {} while(0)
#endif

#else
#define MAZE_ASSERT(cond) do {} while(0)
#endif

/*============================================================================
 * MEMORY USAGE CALCULATION
 *============================================================================*/

/*
 * Total Memory Usage Calculation (for MAZE_MAX_NODES=50, MAZE_MAX_EDGES=100):
 *
 * Static Memory:
 * - Nodes array:        50 × 7 bytes  =  350 bytes
 * - Edges array:       100 × 5 bytes  =  500 bytes
 * - Visited nodes:     50 × 1 byte    =   50 bytes
 * - Robot state:                      =  ~200 bytes
 * - Adjacency lists:                  =  ~300 bytes (estimated)
 *
 * Dynamic Memory (temporary):
 * - Path buffers:       50 × 2 bytes  =  100 bytes
 * - Frontier arrays:    20 × 2 bytes  =   40 bytes
 *
 * TOTAL: ~1.5 KB (well within 8KB limit, leaving ~6.5KB for other code)
 */

#endif // MAZE_CONFIG_H
