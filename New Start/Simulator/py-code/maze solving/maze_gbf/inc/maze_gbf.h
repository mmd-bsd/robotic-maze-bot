#ifndef MAZE_GBF_H
#define MAZE_GBF_H

#include "maze_types.h"

/**
 * @file maze_gbf.h
 * @brief Main API for Greedy Best-First maze navigation
 *
 * This file declares the high-level API for the maze navigation system.
 * The main entry point is maze_gbf_get_next_move() which provides
 * navigation decisions based on sensor input and robot position.
 */

/*============================================================================
 * INITIALIZATION AND CLEANUP
 *============================================================================*/

/**
 * @brief Initialize the maze solving system
 *
 * Sets up the graph and robot structures for maze exploration.
 * Call this before any other maze_gbf functions.
 *
 * @param robot Pointer to robot structure to initialize
 * @param graph Pointer to graph structure to initialize
 * @param start_x Starting X coordinate
 * @param start_y Starting Y coordinate
 * @return MAZE_SUCCESS on success, error code on failure
 *
 * @example
 * MazeRobot robot;
 * MazeGraph graph;
 * int result = maze_gbf_init(&robot, &graph, 0, 0);
 * if (result != MAZE_SUCCESS) {
 *     // Handle error
 * }
 */
int maze_gbf_init(MazeRobot* robot,
                  MazeGraph* graph,
                  int16_t start_x,
                  int16_t start_y);

/**
 * @brief Main navigation function - get next move direction
 *
 * This is the primary API function. Call it each time the robot
 * reaches an intersection or needs a navigation decision.
 *
 * The function implements a 3-priority decision system:
 * - Priority 1: Explore unexplored paths at current location
 * - Priority 2: Navigate to nearest frontier (boundary of known space)
 * - Priority 3: Return to start when exploration is complete
 *
 * @param robot Pointer to robot structure
 * @param sensors Current sensor readings (directions available)
 * @param current_x Current robot X coordinate
 * @param current_y Current robot Y coordinate
 * @return Direction to move (STOP when exploration complete)
 *
 * @example
 * MazeSensors sensors = {
 *     .can_go_forward = true,
 *     .can_go_left = false,
 *     .can_go_right = true,
 *     .can_go_back = false,
 *     .target_reached = false
 * };
 *
 * MazeDirection dir = maze_gbf_get_next_move(&robot, &sensors, 100, 200);
 * switch (dir) {
 *     case MAZE_DIR_FORWARD:
 *         // Move forward
 *         break;
 *     case MAZE_DIR_LEFT:
 *         // Turn left
 *         break;
 *     // ... etc
 * }
 */
MazeDirection maze_gbf_get_next_move(MazeRobot* robot,
                                     const MazeSensors* sensors,
                                     int16_t current_x,
                                     int16_t current_y);

/**
 * @brief Clean up and free resources
 *
 * Frees all dynamically allocated memory used by the navigation system.
 * Call this when done with maze navigation.
 *
 * @param robot Pointer to robot structure
 * @param graph Pointer to graph structure
 *
 * @example
 * maze_gbf_cleanup(&robot, &graph);
 */
void maze_gbf_cleanup(MazeRobot* robot, MazeGraph* graph);

/*============================================================================
 * STATUS QUERIES
 *============================================================================*/

/**
 * @brief Check if exploration is complete
 *
 * Returns true when the entire maze has been explored and
 * the robot has returned to the start position.
 *
 * @param robot Pointer to robot structure
 * @return true if exploration is complete, false otherwise
 */
bool maze_gbf_is_exploration_complete(const MazeRobot* robot);

/**
 * @brief Check if target has been found
 *
 * Returns true when the robot has reached the target location
 * (all sensors detect black).
 *
 * @param robot Pointer to robot structure
 * @return true if target found, false otherwise
 */
bool maze_gbf_is_target_found(const MazeRobot* robot);

/**
 * @brief Get current robot position
 *
 * Returns the robot's current coordinates.
 *
 * @param robot Pointer to robot structure
 * @return Current coordinates
 */
MazeCoord maze_gbf_get_current_position(const MazeRobot* robot);

/**
 * @brief Get number of nodes explored
 *
 * Returns the number of unique intersections/nodes discovered.
 *
 * @param robot Pointer to robot structure
 * @return Number of nodes in the graph
 */
uint16_t maze_gbf_get_node_count(const MazeRobot* robot);

/**
 * @brief Get number of edges explored
 *
 * Returns the number of paths between nodes that have been traversed.
 *
 * @param robot Pointer to robot Pointer to robot structure
 * @return Number of edges marked as explored
 */
uint16_t maze_gbf_get_edge_count(const MazeRobot* robot);

/*============================================================================
 * ADVANCED FUNCTIONS
 *============================================================================*/

/**
 * @brief Force target node ID
 *
 * Manually set the target node ID (normally detected automatically
 * when all sensors read black).
 *
 * @param robot Pointer to robot structure
 * @param target_node_id Node ID to set as target
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_gbf_set_target_node(MazeRobot* robot, uint16_t target_node_id);

/**
 * @brief Calculate optimal path from start to target
 *
 * Computes the shortest path through the explored maze from start
 * to target. Useful for retrieving the final optimal path after
 * exploration is complete.
 *
 * @param robot Pointer to robot structure
 * @param path Output array for path (sequence of node IDs)
 * @param max_path_length Maximum path length (array size)
 * @return Path length (number of nodes), or 0 if no path exists
 */
uint16_t maze_gbf_get_optimal_path(const MazeRobot* robot,
                                    uint16_t* path,
                                    uint16_t max_path_length);

/**
 * @brief Pause/resume exploration
 *
 * Pauses or resumes the exploration process.
 * When paused, get_next_move() will return STOP.
 *
 * @param robot Pointer to robot structure
 * @param paused true to pause, false to resume
 */
void maze_gbf_set_paused(MazeRobot* robot, bool paused);

/**
 * @brief Check if exploration is paused
 *
 * Returns whether exploration is currently paused.
 *
 * @param robot Pointer to robot structure
 * @return true if paused, false otherwise
 */
bool maze_gbf_is_paused(const MazeRobot* robot);

/*============================================================================
 * STATISTICS AND DEBUGGING
 *============================================================================*/

/**
 * @brief Get exploration statistics
 *
 * Fills a structure with statistics about the exploration progress.
 *
 * @param robot Pointer to robot structure
 * @param total_nodes Output: total nodes discovered
 * @param visited_nodes Output: nodes visited
 * @param total_edges Output: total edges found
 * @param explored_edges Output: edges traversed
 * @param steps_taken Output: number of movement steps taken
 */
void maze_gbf_get_statistics(const MazeRobot* robot,
                              uint16_t* total_nodes,
                              uint16_t* visited_nodes,
                              uint16_t* total_edges,
                              uint16_t* explored_edges,
                              uint32_t* steps_taken);

/**
 * @brief Reset exploration (keep graph, reset robot state)
 *
 * Resets the robot to the starting position but keeps the
 * discovered graph structure. Useful for re-running navigation
 * on the same maze.
 *
 * @param robot Pointer to robot structure
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_gbf_reset_exploration(MazeRobot* robot);

/**
 * @brief Export graph data for visualization/analysis
 *
 * Exports the discovered graph structure in a format suitable
 * for external analysis or visualization.
 *
 * @param robot Pointer to robot structure
 * @param nodes Output: array of nodes
 * @param edges Output: array of edges
 * @param max_nodes Maximum nodes to output
 * @param max_edges Maximum edges to output
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_gbf_export_graph(const MazeRobot* robot,
                          MazeNode* nodes,
                          MazeEdge* edges,
                          uint16_t max_nodes,
                          uint16_t max_edges);

#endif // MAZE_GBF_H
