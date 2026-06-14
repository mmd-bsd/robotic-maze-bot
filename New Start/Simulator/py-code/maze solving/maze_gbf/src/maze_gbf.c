/**
 * @file maze_gbf.c
 * @brief Main Greedy Best-First navigation algorithm implementation
 *
 * This file implements the V12 Greedy Best-First frontier-based
 * exploration algorithm, converted from the Python implementation
 * in sim_V12_GBF_stable.py.
 *
 * The algorithm uses a 3-priority decision system:
 * 1. Explore unexplored neighbors at current node
 * 2. Navigate to nearest frontier node
 * 3. Return to start when exploration complete
 */

#include "maze_gbf.h"
#include "maze_robot.h"
#include "maze_graph.h"
#include "maze_utils.h"
#include "maze_config.h"
#include <stdlib.h>
#include <stdlib.h>
#include <string.h>

/*============================================================================
 * INTERNAL STATE
 *============================================================================*/

/**
 * @brief Internal paused state
 *
 * When paused, get_next_move() will return STOP regardless of
 * exploration state.
 */
static bool exploration_paused = false;

/*============================================================================
 * INITIALIZATION AND CLEANUP
 *============================================================================*/

int maze_gbf_init(MazeRobot* robot,
                  MazeGraph* graph,
                  int16_t start_x,
                  int16_t start_y) {
    if (robot == NULL || graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    // Initialize graph
    int result = maze_graph_init(graph, MAZE_MAX_NODES, MAZE_MAX_EDGES);
    if (result != MAZE_SUCCESS) {
        return result;
    }

    // Add start node
    MazeCoord start_coord = {start_x, start_y};
    int start_node_id = maze_graph_add_node(graph, start_coord);
    if (start_node_id < 0) {
        maze_graph_cleanup(graph);
        return start_node_id;
    }

    // Initialize robot
    result = maze_robot_init(robot, graph, (uint16_t)start_node_id, 0);
    if (result != MAZE_SUCCESS) {
        maze_graph_cleanup(graph);
        return result;
    }

    // Reset paused state
    exploration_paused = false;

    MAZE_DEBUG_PRINT("Maze GBF initialized at (%d, %d)\n", start_x, start_y);

    return MAZE_SUCCESS;
}

void maze_gbf_cleanup(MazeRobot* robot, MazeGraph* graph) {
    if (robot != NULL) {
        maze_robot_cleanup(robot);
    }

    if (graph != NULL) {
        maze_graph_cleanup(graph);
    }

    exploration_paused = false;
}

/*============================================================================
 * MAIN NAVIGATION FUNCTION
 *============================================================================*/

MazeDirection maze_gbf_get_next_move(MazeRobot* robot,
                                     const MazeSensors* sensors,
                                     int16_t current_x,
                                     int16_t current_y) {
    // Check if paused
    if (exploration_paused) {
        return MAZE_DIR_STOP;
    }

    // Validate inputs
    if (robot == NULL || sensors == NULL) {
        return MAZE_DIR_STOP;
    }

    // Check if following a cached path
    if (maze_robot_has_path(robot)) {
        return maze_robot_get_next_path_move(robot);
    }

    // Check if exploration is complete
    if (maze_robot_is_exploration_complete(robot)) {
        MAZE_DEBUG_PRINT("Exploration complete. Stopping.\n");
        return MAZE_DIR_STOP;
    }

    // Update robot coordinates
    MazeCoord current_coord = {current_x, current_y};
    maze_robot_set_coord(robot, current_coord);

    // Find or create node at current position
    int node_id = maze_graph_find_node_by_coord(robot->graph, current_coord);

    if (node_id == -1) {
        // New node discovered
        MAZE_DEBUG_PRINT("New node discovered at (%d, %d)\n", current_x, current_y);

        node_id = maze_graph_add_node(robot->graph, current_coord);
        if (node_id < 0) {
            MAZE_DEBUG_PRINT("ERROR: Cannot add node - graph full\n");
            return MAZE_DIR_STOP;
        }
    }

    // If we've moved to a different node, add edge and update position
    if ((uint16_t)node_id != robot->current_node_id) {
        // Add edge from previous node to this node
        int edge_result = maze_graph_add_edge(robot->graph,
                                               robot->current_node_id,
                                               (uint16_t)node_id);
        if (edge_result == MAZE_SUCCESS) {
            // Mark edge as explored since we just traversed it
            maze_graph_mark_edge_explored(robot->graph,
                                          robot->current_node_id,
                                          (uint16_t)node_id);
        }

        // Update robot position
        maze_robot_update_position(robot, (uint16_t)node_id);
    }

    // Check if target reached
    if (sensors->target_reached && !robot->target_found) {
        MAZE_DEBUG_PRINT("Target reached at node %d!\n", node_id);
        maze_robot_set_target_found(robot);
        robot->target_node_id = (uint16_t)node_id;
    }

    // PRIORITY 1: Check for unexplored paths at current node
    uint16_t unexplored[4];
    uint16_t unexplored_count = maze_graph_get_unexplored_neighbors(
        robot->graph,
        robot->current_node_id,
        unexplored,
        4
    );

    if (unexplored_count > 0) {
        // Explore the first unexplored neighbor
        uint16_t next_node = unexplored[0];

        MAZE_DEBUG_PRINT("Priority 1: Exploring unexplored neighbor %d\n", next_node);

        // Calculate and cache path to this neighbor
        uint16_t path[MAZE_MAX_PATH_LENGTH];
        uint16_t path_length = maze_graph_shortest_path(
            robot->graph,
            robot->current_node_id,
            next_node,
            path,
            MAZE_MAX_PATH_LENGTH
        );

        if (path_length > 0) {
            // Store path for following
            maze_robot_set_path(robot, path, path_length);

            // Return direction to first step
            return maze_robot_get_next_path_move(robot);
        }
    }

    // PRIORITY 2: Find nearest frontier
    uint16_t frontiers[MAZE_MAX_FRONTIERS];
    uint16_t frontier_count = maze_robot_find_frontiers(robot, frontiers, MAZE_MAX_FRONTIERS);

    if (frontier_count > 0) {
        int nearest_frontier = maze_robot_find_nearest_frontier(robot);

        if (nearest_frontier >= 0) {
            MAZE_DEBUG_PRINT("Priority 2: Moving to nearest frontier %d\n", nearest_frontier);

            // Calculate and cache path to frontier
            uint16_t path[MAZE_MAX_PATH_LENGTH];
            uint16_t path_length = maze_graph_shortest_path(
                robot->graph,
                robot->current_node_id,
                (uint16_t)nearest_frontier,
                path,
                MAZE_MAX_PATH_LENGTH
            );

            if (path_length > 0) {
                // Store path for following
                maze_robot_set_path(robot, path, path_length);

                // Return direction to first step
                return maze_robot_get_next_path_move(robot);
            }
        }
    }

    // PRIORITY 3: No frontiers left - return to start
    if (robot->current_node_id == robot->start_node_id) {
        MAZE_DEBUG_PRINT("Priority 3: Exploration complete! Returned to start.\n");
        maze_robot_set_exploration_complete(robot);
        return MAZE_DIR_STOP;
    } else {
        MAZE_DEBUG_PRINT("Priority 3: No frontiers, returning to start node %d\n",
                         robot->start_node_id);

        // Calculate path back to start
        uint16_t path[MAZE_MAX_PATH_LENGTH];
        uint16_t path_length = maze_graph_shortest_path(
            robot->graph,
            robot->current_node_id,
            robot->start_node_id,
            path,
            MAZE_MAX_PATH_LENGTH
        );

        if (path_length > 0) {
            // Store path for following
            maze_robot_set_path(robot, path, path_length);

            // Return direction to first step
            return maze_robot_get_next_path_move(robot);
        }
    }

    // Fallback: shouldn't reach here
    MAZE_DEBUG_PRINT("ERROR: No valid move found\n");
    return MAZE_DIR_STOP;
}

/*============================================================================
 * STATUS QUERIES
 *============================================================================*/

bool maze_gbf_is_exploration_complete(const MazeRobot* robot) {
    if (robot == NULL) {
        return false;
    }
    return maze_robot_is_exploration_complete(robot);
}

bool maze_gbf_is_target_found(const MazeRobot* robot) {
    if (robot == NULL) {
        return false;
    }
    return maze_robot_is_target_found(robot);
}

MazeCoord maze_gbf_get_current_position(const MazeRobot* robot) {
    if (robot == NULL) {
        MazeCoord empty = {0, 0};
        return empty;
    }
    return maze_robot_get_coord(robot);
}

uint16_t maze_gbf_get_node_count(const MazeRobot* robot) {
    if (robot == NULL || robot->graph == NULL) {
        return 0;
    }
    return maze_graph_get_node_count(robot->graph);
}

uint16_t maze_gbf_get_edge_count(const MazeRobot* robot) {
    if (robot == NULL || robot->graph == NULL) {
        return 0;
    }
    return maze_graph_get_edge_count(robot->graph);
}

/*============================================================================
 * ADVANCED FUNCTIONS
 *============================================================================*/

int maze_gbf_set_target_node(MazeRobot* robot, uint16_t target_node_id) {
    if (robot == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (target_node_id >= robot->graph->node_count) {
        return MAZE_ERR_NODE_NOT_FOUND;
    }

    robot->target_node_id = target_node_id;
    maze_robot_set_target_found(robot);

    return MAZE_SUCCESS;
}

uint16_t maze_gbf_get_optimal_path(const MazeRobot* robot,
                                    uint16_t* path,
                                    uint16_t max_path_length) {
    if (robot == NULL || path == NULL) {
        return 0;
    }

    if (robot->target_node_id == 0) {
        return 0;  // No target set
    }

    return maze_graph_shortest_path(
        robot->graph,
        robot->start_node_id,
        robot->target_node_id,
        path,
        max_path_length
    );
}

void maze_gbf_set_paused(MazeRobot* robot, bool paused) {
    (void)robot;  // Unused parameter
    exploration_paused = paused;
}

bool maze_gbf_is_paused(const MazeRobot* robot) {
    (void)robot;  // Unused parameter
    return exploration_paused;
}

/*============================================================================
 * STATISTICS AND DEBUGGING
 *============================================================================*/

void maze_gbf_get_statistics(const MazeRobot* robot,
                              uint16_t* total_nodes,
                              uint16_t* visited_nodes,
                              uint16_t* total_edges,
                              uint16_t* explored_edges,
                              uint32_t* steps_taken) {
    if (robot == NULL) {
        return;
    }

    if (total_nodes != NULL) {
        *total_nodes = maze_graph_get_node_count(robot->graph);
    }

    if (visited_nodes != NULL) {
        uint16_t count = 0;
        for (uint16_t i = 0; i < robot->graph->node_count; i++) {
            if (robot->graph->nodes[i].visited) {
                count++;
            }
        }
        *visited_nodes = count;
    }

    if (total_edges != NULL) {
        *total_edges = maze_graph_get_edge_count(robot->graph);
    }

    if (explored_edges != NULL) {
        *explored_edges = robot->visited_edge_count;
    }

    if (steps_taken != NULL) {
        // This would need to be tracked during exploration
        // For now, return 0
        *steps_taken = 0;
    }
}

int maze_gbf_reset_exploration(MazeRobot* robot) {
    if (robot == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    maze_robot_reset(robot);
    return MAZE_SUCCESS;
}

int maze_gbf_export_graph(const MazeRobot* robot,
                          MazeNode* nodes,
                          MazeEdge* edges,
                          uint16_t max_nodes,
                          uint16_t max_edges) {
    if (robot == NULL || nodes == NULL || edges == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (robot->graph == NULL) {
        return MAZE_ERR_NOT_INITIALIZED;
    }

    // Copy nodes
    uint16_t node_count = robot->graph->node_count;
    if (node_count > max_nodes) {
        node_count = max_nodes;
    }
    memcpy(nodes, robot->graph->nodes, node_count * sizeof(MazeNode));

    // Copy edges
    uint16_t edge_count = robot->graph->edge_count;
    if (edge_count > max_edges) {
        edge_count = max_edges;
    }
    memcpy(edges, robot->graph->edges, edge_count * sizeof(MazeEdge));

    return MAZE_SUCCESS;
}
