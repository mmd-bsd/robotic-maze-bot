#ifndef MAZE_TYPES_H
#define MAZE_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file maze_types.h
 * @brief Core type definitions for the maze navigation system
 *
 * This file defines all the data structures used throughout
 * the maze solving algorithm implementation.
 */

/**
 * @brief Direction enumeration for robot movement commands
 *
 * These are the possible directions the robot can move.
 * The motor control layer will convert these to actual motor commands.
 */
typedef enum {
    MAZE_DIR_STOP = 0,      /**< Stop movement, exploration complete */
    MAZE_DIR_FORWARD,       /**< Move forward relative to robot heading */
    MAZE_DIR_LEFT,          /**< Turn left and move forward */
    MAZE_DIR_RIGHT,         /**< Turn right and move forward */
    MAZE_DIR_BACK           /**< Move backward (reverse) */
} MazeDirection;

/**
 * @brief 2D coordinate structure
 *
 * Represents a position in the maze using 16-bit signed integers.
 * Units are typically in millimeters or centimeters depending on
 * the localization system resolution.
 */
typedef struct {
    int16_t x;              /**< X coordinate (horizontal position) */
    int16_t y;              /**< Y coordinate (vertical position) */
} MazeCoord;

/**
 * @brief Node structure representing an intersection in the maze
 *
 * Each node represents a unique intersection or decision point
 * in the maze where the robot can change direction.
 */
typedef struct {
    uint16_t id;            /**< Sequential node ID (0=start, 1, 2, 3...) */
    MazeCoord coord;        /**< Physical coordinates of this node */
    bool visited;           /**< Flag indicating if node has been visited */
} MazeNode;

/**
 * @brief Edge structure connecting two nodes
 *
 * Represents a traversable path between two intersections in the maze.
 * Edges are bidirectional (undirected graph).
 */
typedef struct {
    uint16_t node_a;        /**< First node ID (smaller ID for consistency) */
    uint16_t node_b;        /**< Second node ID (larger ID) */
    bool explored;          /**< True if robot has traversed this edge */
} MazeEdge;

/**
 * @brief Graph adjacency list entry
 *
 * Used to build adjacency lists for efficient neighbor lookup.
 * Each entry points to the next neighbor in the list.
 */
typedef struct MazeAdjListEntry {
    uint16_t neighbor_id;                   /**< ID of neighboring node */
    struct MazeAdjListEntry* next;          /**< Next neighbor in list */
} MazeAdjListEntry;

/**
 * @brief Complete graph structure representing the maze
 *
 * This structure contains all nodes, edges, and adjacency information
 * for the maze being explored. Memory is statically allocated for
 * embedded systems efficiency.
 */
typedef struct {
    MazeNode* nodes;                  /**< Dynamic array of nodes */
    MazeEdge* edges;                  /**< Dynamic array of edges */
    MazeAdjListEntry** adj_list;      /**< Adjacency lists for fast neighbor lookup */
    uint16_t node_count;              /**< Current number of nodes in graph */
    uint16_t edge_count;              /**< Current number of edges in graph */
    uint16_t max_nodes;               /**< Maximum nodes (memory allocation limit) */
    uint16_t max_edges;               /**< Maximum edges (memory allocation limit) */
} MazeGraph;

/**
 * @brief Robot state structure
 *
 * Tracks the current state of the robot including position,
 * exploration status, and reference to the graph being built.
 */
typedef struct {
    uint16_t current_node_id;         /**< ID of node where robot currently is */
    uint16_t start_node_id;           /**< ID of starting position node */
    uint16_t target_node_id;          /**< ID of target destination node (set when found) */
    MazeCoord current_coord;          /**< Current physical coordinates */

    // Known map tracking
    bool* visited_nodes;              /**< Boolean array indexed by node_id */
    uint16_t visited_edge_count;      /**< Number of edges marked as explored */

    // Exploration state
    bool exploration_complete;        /**< Flag: entire maze explored */
    bool target_found;                /**< Flag: target location has been reached */

    // Navigation state for multi-step path following
    MazeDirection cached_direction;   /**< Cached direction for current move */
    uint16_t* current_path;           /**< Path array (sequence of node IDs) */
    uint16_t path_length;             /**< Length of current path */
    uint16_t path_index;              /**< Current position in path */

    // Graph reference
    MazeGraph* graph;                 /**< Pointer to the graph structure */
} MazeRobot;

/**
 * @brief Sensor input structure
 *
 * This structure is populated by the sensor processing layer
 * before calling the navigation algorithm.
 */
typedef struct {
    bool can_go_forward;      /**< True if path forward is available */
    bool can_go_left;         /**< True if left turn is possible */
    bool can_go_right;        /**< True if right turn is possible */
    bool can_go_back;         /**< True if backward path exists */
    bool target_reached;      /**< True if robot has reached target (all sensors detect black) */
} MazeSensors;

/**
 * @brief Error codes for maze navigation functions
 */
typedef enum {
    MAZE_SUCCESS = 0,              /**< Operation completed successfully */
    MAZE_ERR_NULL_PTR = -1,        /**< Null pointer argument */
    MAZE_ERR_NO_MEMORY = -2,       /**< Memory allocation failed */
    MAZE_ERR_GRAPH_FULL = -3,      /**< Cannot add more nodes/edges */
    MAZE_ERR_NODE_NOT_FOUND = -4,  /**< Node ID does not exist */
    MAZE_ERR_NO_PATH = -5,         /**< No path exists between nodes */
    MAZE_ERR_INVALID_PARAM = -6,   /**< Invalid parameter value */
    MAZE_ERR_NOT_INITIALIZED = -7  /**< Structure not initialized */
} MazeErrorCode;

#endif // MAZE_TYPES_H
