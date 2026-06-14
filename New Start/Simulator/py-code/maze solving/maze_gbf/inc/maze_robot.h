#ifndef MAZE_ROBOT_H
#define MAZE_ROBOT_H

#include "maze_types.h"

/**
 * @file maze_robot.h
 * @brief Robot state management and frontier detection API
 *
 * This file declares functions for managing robot state,
 * detecting frontiers for exploration, and calculating
 * navigation directions.
 */

/*============================================================================
 * ROBOT INITIALIZATION AND CLEANUP
 *============================================================================*/

/**
 * @brief Initialize robot structure
 *
 * Sets up the robot state with starting position and target.
 * Must be called before using other robot functions.
 *
 * @param robot Pointer to robot structure to initialize
 * @param graph Pointer to graph structure robot will use
 * @param start_node_id Starting node ID
 * @param target_node_id Target node ID (set to 0 if unknown)
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_robot_init(MazeRobot* robot,
                    MazeGraph* graph,
                    uint16_t start_node_id,
                    uint16_t target_node_id);

/**
 * @brief Clean up robot resources
 *
 * Frees any dynamically allocated memory in the robot structure.
 *
 * @param robot Pointer to robot structure to clean up
 */
void maze_robot_cleanup(MazeRobot* robot);

/**
 * @brief Reset robot to initial state
 *
 * Resets exploration flags and navigation state but keeps
 * the same graph reference.
 *
 * @param robot Pointer to robot structure to reset
 */
void maze_robot_reset(MazeRobot* robot);

/*============================================================================
 * POSITION MANAGEMENT
 *============================================================================*/

/**
 * @brief Update robot position to a new node
 *
 * Moves the robot to a new node, updates visited status,
 * and records the move in path history.
 *
 * @param robot Pointer to robot structure
 * @param next_node_id Node ID to move to
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_robot_update_position(MazeRobot* robot, uint16_t next_node_id);

/**
 * @brief Set robot current coordinates
 *
 * Updates the robot's current position coordinates.
 * Used when robot reaches a new physical location.
 *
 * @param robot Pointer to robot structure
 * @param coord New coordinates
 */
void maze_robot_set_coord(MazeRobot* robot, MazeCoord coord);

/**
 * @brief Get robot current coordinates
 *
 * Returns the robot's current position coordinates.
 *
 * @param robot Pointer to robot structure
 * @return Current coordinates
 */
MazeCoord maze_robot_get_coord(const MazeRobot* robot);

/*============================================================================
 * VISITATION TRACKING
 *============================================================================*/

/**
 * @brief Mark node as visited
 *
 * Marks a specific node as visited in the robot's tracking.
 *
 * @param robot Pointer to robot structure
 * @param node_id Node ID to mark
 */
void maze_robot_mark_node_visited(MazeRobot* robot, uint16_t node_id);

/**
 * @brief Check if node has been visited
 *
 * Returns whether a specific node has been marked as visited.
 *
 * @param robot Pointer to robot structure
 * @param node_id Node ID to check
 * @return true if visited, false otherwise
 */
bool maze_robot_is_node_visited(const MazeRobot* robot, uint16_t node_id);

/*============================================================================
 * FRONTIER DETECTION
 *============================================================================*/

/**
 * @brief Find all frontier nodes
 *
 * Frontiers are visited nodes that have at least one unexplored edge.
 * These represent the boundary between known and unknown maze areas.
 *
 * @param robot Pointer to robot structure
 * @param frontiers Output array for frontier node IDs
 * @param max_frontiers Maximum frontiers to return (array size)
 * @return Number of frontiers found
 */
uint16_t maze_robot_find_frontiers(const MazeRobot* robot,
                                   uint16_t* frontiers,
                                   uint16_t max_frontiers);

/**
 * @brief Find nearest frontier node
 *
 * Finds the frontier node with the shortest path from the robot's
 * current position using BFS distance calculation.
 *
 * @param robot Pointer to robot structure
 * @return Nearest frontier node ID, or -1 if no frontiers found
 */
int maze_robot_find_nearest_frontier(const MazeRobot* robot);

/**
 * @brief Check if current node has unexplored neighbors
 *
 * Quick check to determine if the robot's current position
 * has any unexplored paths available.
 *
 * @param robot Pointer to robot structure
 * @return true if unexplored neighbors exist, false otherwise
 */
bool maze_robot_has_unexplored_neighbors(const MazeRobot* robot);

/*============================================================================
 * DIRECTION CALCULATION
 *============================================================================*/

/**
 * @brief Calculate direction to reach a target node
 *
 * Determines which direction (FORWARD, LEFT, RIGHT, BACK) the robot
 * should move to reach the target node from its current position.
 * Direction is calculated based on coordinate differences.
 *
 * @param robot Pointer to robot structure
 * @param target_node_id ID of node to reach
 * @return Direction to move, or STOP if calculation fails
 */
MazeDirection maze_robot_get_direction_to_node(const MazeRobot* robot,
                                               uint16_t target_node_id);

/**
 * @brief Calculate direction between two coordinates
 *
 * Determines which direction to move from one coordinate to another.
 * Useful for path following without node references.
 *
 * @param from Starting coordinate
 * @param to Target coordinate
 * @return Direction to move
 */
MazeDirection maze_robot_get_direction_to_coord(MazeCoord from, MazeCoord to);

/*============================================================================
 * PATH MANAGEMENT
 *============================================================================*/

/**
 * @brief Set current navigation path
 *
 * Stores a path for the robot to follow. The robot will execute
 * this path step by step.
 *
 * @param robot Pointer to robot structure
 * @param path Array of node IDs representing the path
 * @param path_length Length of the path array
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_robot_set_path(MazeRobot* robot,
                        const uint16_t* path,
                        uint16_t path_length);

/**
 * @brief Clear current navigation path
 *
 * Clears any stored path and frees associated memory.
 *
 * @param robot Pointer to robot structure
 */
void maze_robot_clear_path(MazeRobot* robot);

/**
 * @brief Get next move from current path
 *
 * Returns the next direction to move along the current path.
 * Advances the path index.
 *
 * @param robot Pointer to robot structure
 * @return Next direction, or STOP if path is complete/invalid
 */
MazeDirection maze_robot_get_next_path_move(MazeRobot* robot);

/**
 * @brief Check if robot is currently following a path
 *
 * Returns whether the robot has an active path to follow.
 *
 * @param robot Pointer to robot structure
 * @return true if following a path, false otherwise
 */
bool maze_robot_has_path(const MazeRobot* robot);

/*============================================================================
 * EXPLORATION STATE
 *============================================================================*/

/**
 * @brief Mark exploration as complete
 *
 * Sets the exploration complete flag, indicating the entire
 * maze has been mapped.
 *
 * @param robot Pointer to robot structure
 */
void maze_robot_set_exploration_complete(MazeRobot* robot);

/**
 * @brief Check if exploration is complete
 *
 * Returns whether the robot has finished exploring the maze.
 *
 * @param robot Pointer to robot structure
 * @return true if exploration is complete, false otherwise
 */
bool maze_robot_is_exploration_complete(const MazeRobot* robot);

/**
 * @brief Mark target as found
 *
 * Sets the target found flag when the robot reaches the destination.
 *
 * @param robot Pointer to robot structure
 */
void maze_robot_set_target_found(MazeRobot* robot);

/**
 * @brief Check if target has been found
 *
 * Returns whether the robot has discovered the target location.
 *
 * @param robot Pointer to robot structure
 * @return true if target found, false otherwise
 */
bool maze_robot_is_target_found(const MazeRobot* robot);

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Get robot's current node ID
 *
 * Returns the ID of the node where the robot is currently located.
 *
 * @param robot Pointer to robot structure
 * @return Current node ID
 */
uint16_t maze_robot_get_current_node(const MazeRobot* robot);

/**
 * @brief Get robot's start node ID
 *
 * Returns the ID of the robot's starting position.
 *
 * @param robot Pointer to robot structure
 * @return Start node ID
 */
uint16_t maze_robot_get_start_node(const MazeRobot* robot);

/**
 * @brief Get robot's target node ID
 *
 * Returns the ID of the target destination.
 *
 * @param robot Pointer to robot structure
 * @return Target node ID, or 0 if not set
 */
uint16_t maze_robot_get_target_node(const MazeRobot* robot);

#endif // MAZE_ROBOT_H
