/**
 * @file maze_types.h
 * @brief Core type definitions for the SEYED maze navigation system (C port).
 *
 * Maps 1:1 to the Python simulator (maze_solver.py) data model, adapted for
 * embedded static allocation on STM32G0 (Cortex-M0+, 8KB RAM).
 *
 * All units are real-world: 1 world unit = 1 cm.
 * Y is up (matches the Python simulator coordinate system).
 */

#ifndef MAZE_TYPES_H
#define MAZE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * COMPASS HEADING
 *
 * Matches the firmware's `nav` variable:
 *   0 = NORTH (+Y), 1 = WEST (-X), 2 = SOUTH (-Y), 3 = EAST (+X)
 *
 * The command-generation module computes F/L/R/B from heading deltas using
 * a cross-product-like lookup (exactly as in ALGORITHMS.md §5).
 *============================================================================*/

typedef enum {
    MAZE_NORTH = 0,   /**< +Y  (0, +1) */
    MAZE_WEST  = 1,   /**< -X  (-1, 0) */
    MAZE_SOUTH = 2,   /**< -Y  (0, -1) */
    MAZE_EAST  = 3,   /**< +X  (+1, 0) */
    MAZE_HEADING_NONE = 0xFF  /**< Sentinel: heading not yet set (first move) */
} MazeHeading;

/*============================================================================
 * MOVEMENT COMMANDS
 *
 * These are the F / L / R / B commands the real robot executes.
 * They match the firmware's `cross` encoding:
 *   0 = forward, 1 = left, 2 = right, 4 = U-turn (back)
 *
 * The Python simulator generates these from heading deltas; the C library
 * does the same (see maze_robot.h).
 *============================================================================*/

typedef enum {
    MAZE_CMD_FORWARD = 'F',   /**< Straight ahead                     */
    MAZE_CMD_LEFT    = 'L',   /**< Turn left + move                   */
    MAZE_CMD_RIGHT   = 'R',   /**< Turn right + move                  */
    MAZE_CMD_BACK    = 'B',   /**< U-turn / reverse                   */
    MAZE_CMD_NONE    = '\0'   /**< Sentinel / no command              */
} MazeCommand;

/** Convert MazeCommand to firmware `cross` value (0/1/2/4). */
static inline uint8_t maze_cmd_to_cross(MazeCommand cmd) {
    switch (cmd) {
        case MAZE_CMD_FORWARD: return 0;
        case MAZE_CMD_LEFT:    return 1;
        case MAZE_CMD_RIGHT:   return 2;
        case MAZE_CMD_BACK:    return 4;
        default:               return 0xFF;
    }
}

/*============================================================================
 * MISSION PHASES
 *
 * Mirrors the Python solver's state machine:
 *   EXPLORE → RETURN_HOME → FAST_RUN → DONE
 *============================================================================*/

typedef enum {
    MAZE_PHASE_EXPLORE     = 0,   /**< Searching for the target           */
    MAZE_PHASE_RETURN_HOME = 1,   /**< Driving back to start node         */
    MAZE_PHASE_FAST_RUN    = 2,   /**< Driving the time-optimal path      */
    MAZE_PHASE_DONE         = 3   /**< Mission complete                   */
} MazePhase;

/*============================================================================
 * PROOF STATE
 *
 * Tracks whether the fastest route is proven (matches the GUI's Proof flag).
 *============================================================================*/

typedef enum {
    MAZE_PROOF_SEARCHING  = 0,   /**< Target not yet found                        */
    MAZE_PROOF_CHECKING   = 1,   /**< Target found; still verifying frontiers     */
    MAZE_PROOF_PROVEN     = 2,   /**< Stopped early — fastest route is optimal     */
    MAZE_PROOF_FULL       = 3,   /**< Explored everything before finishing         */
    MAZE_PROOF_DISABLED   = 4    /**< Optimized-search toggle is OFF               */
} MazeProofState;

/*============================================================================
 * 2-D COORDINATE
 *============================================================================*/

typedef struct {
    int16_t x;            /**< X coordinate (horizontal), world units = cm */
    int16_t y;            /**< Y coordinate (vertical, Y is up), cm        */
} MazeCoord;

/*============================================================================
 * GRAPH: NODE
 *
 * A maze intersection.  node[0] = start, node[target] = target.
 * The `connections[]` bitmask records which cardinal directions have an
 * edge from this node (bit 0=N, 1=W, 2=S, 3=E), as discovered by the robot.
 *============================================================================*/

typedef struct {
    MazeCoord coord;          /**< World position (cm)                         */
    uint8_t  connections;     /**< Bits 0..3 = N,W,S,E known to exist          */
    bool     visited;         /**< Robot has physically been to this node      */
} MazeNode;

/*============================================================================
 * GRAPH: EDGE
 *
 * A traversable line segment between two adjacent nodes.  The edge stores
 * its physical length (cm) and exploration status.
 *============================================================================*/

typedef struct {
    uint16_t node_a;          /**< First node index  (a < b by convention)     */
    uint16_t node_b;          /**< Second node index                           */
    uint16_t length_cm;       /**< Physical length in cm (from encoder ticks)  */
    uint8_t  heading;         /**< Cardinal direction a→b (MazeHeading 0..3)   */
    bool     explored;        /**< Robot has driven across this edge           */
} MazeEdge;

/*============================================================================
 * GRAPH STRUCTURE
 *
 * All arrays are statically sized by MAZE_MAX_NODES / MAZE_MAX_EDGES.
 * No dynamic allocation on the target MCU.
 *============================================================================*/

typedef struct {
    MazeNode  nodes[50];      /**< Node array  (max from maze_config.h)        */
    MazeEdge  edges[100];     /**< Edge array                                  */
    uint16_t  node_count;     /**< Current number of nodes in known map        */
    uint16_t  edge_count;     /**< Current number of edges in known map        */
    uint16_t  start_node;     /**< Index of the start node                     */
    uint16_t  target_node;    /**< Index of the target node (set when found)   */
} MazeGraph;

/*============================================================================
 * SENSOR READING
 *
 * Populated from the firmware's IR sensor array and end-zone detection.
 * The `target_reached` flag uses the 2-tick debounced end_zone_timer.
 *============================================================================*/

typedef struct {
    bool can_go_forward;      /**< Straight path continues past junction       */
    bool can_go_left;         /**< Left branch exists at this junction         */
    bool can_go_right;        /**< Right branch exists at this junction        */
    bool can_go_back;         /**< Reverse direction is always possible        */
    bool target_reached;      /**< All sensors detect black (target/end zone)  */
} MazeSensors;

/*============================================================================
 * FAST-RUN PLAN SEGMENT
 *
 * A precomputed sample point along the time-optimal path for animation /
 * speed-gradient control.  Each segment covers one straight run (maximal
 * collinear sequence) between two stops.
 *============================================================================*/

typedef struct {
    uint16_t segment_id;      /**< Index of this straight run                  */
    float    t_from_start;    /**< Time (s) since start of this segment        */
    float    distance_cm;     /**< Arc distance (cm) into this segment         */
    float    speed_cms;       /**< Instantaneous speed (cm/s) at this point    */
} MazeTrajectoryPoint;

/*============================================================================
 * ROBOT STATE
 *
 * The complete state of the robot during a mission.  All fields are
 * statically owned; no heap pointers except for fast-run trajectory.
 *============================================================================*/

typedef struct {
    /* --- Position & heading --- */
    uint16_t    current_node;   /**< Node index where robot is now             */
    MazeCoord   current_coord;  /**< Current world coordinates (cm)            */
    MazeHeading heading;        /**< Current facing (N/E/S/W)                  */

    /* --- Mission state --- */
    MazePhase       phase;         /**< Current mission phase                  */
    MazeProofState  proof_state;   /**< Current proof status                   */
    bool            target_found;  /**< Robot has physically reached target    */
    bool            use_proof;     /**< Optimized-search toggle (early stop)   */

    /* --- Known map tracking --- */
    bool visited_nodes[50];       /**< Boolean array: node[i] visited?         */
    bool explored_edges[100];     /**< Boolean array: edge[i] explored?        */

    /* --- Command log --- */
    char    command_log[512];    /**< F/L/R/B command stream                   */
    uint16_t command_count;      /**< Number of commands emitted               */

    /* --- Navigation state (multi-step path following) --- */
    uint16_t path_buffer[50];    /**< Current Dijkstra path (node indices)     */
    uint16_t path_length;        /**< Number of nodes in path_buffer           */
    uint16_t path_index;         /**< Current step within path_buffer          */

    /* --- Fast-run plan --- */
    uint16_t fast_path_nodes[50];    /**< Node sequence of the fastest route   */
    uint16_t fast_path_length;       /**< Number of nodes in fast_path         */
    float    fast_path_time_s;       /**< Total drive time of fastest route (s)*/
    MazeTrajectoryPoint* fast_traj;  /**< Precomputed trajectory (or NULL)     */
    uint16_t fast_traj_count;        /**< Number of trajectory points          */

    /* --- Statistics --- */
    uint16_t steps_taken;        /**< Total movement steps executed            */
    float    total_distance_cm;  /**< Cumulative distance travelled (cm)       */

    /* --- Graph reference --- */
    MazeGraph* graph;            /**< Pointer to the graph the robot is using  */
} MazeRobot;

/*============================================================================
 * ERROR CODES
 *============================================================================*/

typedef enum {
    MAZE_OK                 =  0,   /**< Success                                */
    MAZE_ERR_NULL_PTR       = -1,   /**< NULL argument                          */
    MAZE_ERR_GRAPH_FULL     = -2,   /**< Cannot add more nodes/edges            */
    MAZE_ERR_NODE_NOT_FOUND = -3,   /**< Node index out of range                */
    MAZE_ERR_NO_PATH        = -4,   /**< No path exists between nodes           */
    MAZE_ERR_NOT_READY      = -5,   /**< Robot not in the expected state        */
    MAZE_ERR_INVALID_PARAM  = -6    /**< Parameter out of valid range           */
} MazeErrorCode;

/*============================================================================
 * CONSTANTS
 *============================================================================*/

/** Sentinel value for "no node" / "not found". */
#define MAZE_INVALID_NODE  0xFFFF

/** Maximum number of neighbors at a 90° intersection. */
#define MAZE_MAX_NEIGHBORS 4

#endif /* MAZE_TYPES_H */
