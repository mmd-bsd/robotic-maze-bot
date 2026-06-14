/**
 * @file maze_robot.c
 * @brief Robot state management and frontier detection implementation
 *
 * This file implements robot state management, frontier detection
 * for exploration, and direction calculation functions.
 */

#include "maze_robot.h"
#include "maze_graph.h"
#include "maze_config.h"
#include <stdlib.h>
#include <string.h>
#include <stdlib.h>

/*============================================================================
 * ROBOT INITIALIZATION AND CLEANUP
 *============================================================================*/

int maze_robot_init(MazeRobot* robot,
                    MazeGraph* graph,
                    uint16_t start_node_id,
                    uint16_t target_node_id) {
    if (robot == NULL || graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    // Initialize state
    robot->graph = graph;
    robot->current_node_id = start_node_id;
    robot->start_node_id = start_node_id;
    robot->target_node_id = target_node_id;

    // Get start node coordinates
    MazeNode* start_node = maze_graph_get_node(graph, start_node_id);
    if (start_node == NULL) {
        return MAZE_ERR_NODE_NOT_FOUND;
    }
    robot->current_coord = start_node->coord;

    // Allocate visited nodes array
    robot->visited_nodes = (bool*)malloc(graph->max_nodes * sizeof(bool));
    if (robot->visited_nodes == NULL) {
        return MAZE_ERR_NO_MEMORY;
    }

    // Initialize visited array (all false except start)
    memset(robot->visited_nodes, 0, graph->max_nodes * sizeof(bool));
    robot->visited_nodes[start_node_id] = true;

    // Initialize state
    robot->visited_edge_count = 0;
    robot->exploration_complete = false;
    robot->target_found = false;

    // Initialize path tracking
    robot->current_path = NULL;
    robot->path_length = 0;
    robot->path_index = 0;
    robot->cached_direction = MAZE_DIR_STOP;

    // Mark start node as visited in graph
    maze_graph_mark_node_visited(graph, start_node_id);

    MAZE_DEBUG_PRINT("Robot initialized at node %d\n", start_node_id);

    return MAZE_SUCCESS;
}

void maze_robot_cleanup(MazeRobot* robot) {
    if (robot == NULL) {
        return;
    }

    // Free visited nodes array
    if (robot->visited_nodes != NULL) {
        free(robot->visited_nodes);
        robot->visited_nodes = NULL;
    }

    // Free current path
    maze_robot_clear_path(robot);
}

void maze_robot_reset(MazeRobot* robot) {
    if (robot == NULL || robot->graph == NULL) {
        return;
    }

    // Clear visited array
    if (robot->visited_nodes != NULL) {
        memset(robot->visited_nodes, 0, robot->graph->max_nodes * sizeof(bool));
        robot->visited_nodes[robot->start_node_id] = true;
    }

    // Reset state
    robot->current_node_id = robot->start_node_id;
    robot->visited_edge_count = 0;
    robot->exploration_complete = false;
    robot->target_found = false;

    // Clear path
    maze_robot_clear_path(robot);
    robot->cached_direction = MAZE_DIR_STOP;

    // Get start coordinates
    MazeNode* start_node = maze_graph_get_node(robot->graph, robot->start_node_id);
    if (start_node != NULL) {
        robot->current_coord = start_node->coord;
    }
}

/*============================================================================
 * POSITION MANAGEMENT
 *============================================================================*/

int maze_robot_update_position(MazeRobot* robot, uint16_t next_node_id) {
    if (robot == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (next_node_id >= robot->graph->node_count) {
        return MAZE_ERR_NODE_NOT_FOUND;
    }

    // Update current node
    robot->current_node_id = next_node_id;

    // Update coordinates
    MazeNode* next_node = maze_graph_get_node(robot->graph, next_node_id);
    if (next_node != NULL) {
        robot->current_coord = next_node->coord;
    }

    // Mark as visited
    maze_robot_mark_node_visited(robot, next_node_id);
    maze_graph_mark_node_visited(robot->graph, next_node_id);

    // Mark edge as explored
    // Note: edge should have been added before calling this function
    // But we mark it explored here when we actually traverse it

    MAZE_DEBUG_PRINT("Robot moved to node %d at (%d, %d)\n",
                     next_node_id,
                     robot->current_coord.x,
                     robot->current_coord.y);

    return MAZE_SUCCESS;
}

void maze_robot_set_coord(MazeRobot* robot, MazeCoord coord) {
    if (robot == NULL) {
        return;
    }
    robot->current_coord = coord;
}

MazeCoord maze_robot_get_coord(const MazeRobot* robot) {
    if (robot == NULL) {
        MazeCoord empty = {0, 0};
        return empty;
    }
    return robot->current_coord;
}

/*============================================================================
 * VISITATION TRACKING
 *============================================================================*/

void maze_robot_mark_node_visited(MazeRobot* robot, uint16_t node_id) {
    if (robot == NULL || robot->visited_nodes == NULL) {
        return;
    }

    if (node_id < robot->graph->max_nodes) {
        robot->visited_nodes[node_id] = true;
    }
}

bool maze_robot_is_node_visited(const MazeRobot* robot, uint16_t node_id) {
    if (robot == NULL || robot->visited_nodes == NULL) {
        return false;
    }

    if (node_id >= robot->graph->max_nodes) {
        return false;
    }

    return robot->visited_nodes[node_id];
}

/*============================================================================
 * FRONTIER DETECTION
 *============================================================================*/

uint16_t maze_robot_find_frontiers(const MazeRobot* robot,
                                   uint16_t* frontiers,
                                   uint16_t max_frontiers) {
    if (robot == NULL || frontiers == NULL) {
        return 0;
    }

    uint16_t count = 0;

    // Iterate through all nodes
    for (uint16_t i = 0; i < robot->graph->node_count; i++) {
        uint16_t node_id = robot->graph->nodes[i].id;

        // Only check visited nodes (frontiers are on the boundary of known space)
        if (!maze_robot_is_node_visited(robot, node_id)) {
            continue;
        }

        // Check if node has unexplored neighbors
        uint16_t unexplored[4];
        uint16_t unexp_count = maze_graph_get_unexplored_neighbors(
            robot->graph, node_id, unexplored, 4);

        if (unexp_count > 0) {
            // This node is a frontier
            frontiers[count++] = node_id;
            if (count >= max_frontiers) {
                break;
            }
        }
    }

    return count;
}

int maze_robot_find_nearest_frontier(const MazeRobot* robot) {
    if (robot == NULL) {
        return -1;
    }

    // Get all frontiers
    uint16_t frontiers[MAZE_MAX_FRONTIERS];
    uint16_t frontier_count = maze_robot_find_frontiers(robot, frontiers, MAZE_MAX_FRONTIERS);

    if (frontier_count == 0) {
        return -1;  // No frontiers found
    }

    // Find nearest frontier using shortest path length
    int nearest_frontier = -1;
    uint16_t min_distance = UINT16_MAX;

    for (uint16_t i = 0; i < frontier_count; i++) {
        uint16_t frontier_id = frontiers[i];

        uint16_t distance = maze_graph_shortest_path_length(
            robot->graph,
            robot->current_node_id,
            frontier_id
        );

        if (distance > 0 && distance < min_distance) {
            min_distance = distance;
            nearest_frontier = (int)frontier_id;
        }
    }

    return nearest_frontier;
}

bool maze_robot_has_unexplored_neighbors(const MazeRobot* robot) {
    if (robot == NULL) {
        return false;
    }

    uint16_t unexplored[4];
    uint16_t count = maze_graph_get_unexplored_neighbors(
        robot->graph,
        robot->current_node_id,
        unexplored,
        4
    );

    return count > 0;
}

/*============================================================================
 * DIRECTION CALCULATION
 *============================================================================*/

MazeDirection maze_robot_get_direction_to_node(const MazeRobot* robot,
                                               uint16_t target_node_id) {
    if (robot == NULL) {
        return MAZE_DIR_STOP;
    }

    if (target_node_id >= robot->graph->node_count) {
        return MAZE_DIR_STOP;
    }

    MazeNode* target_node = maze_graph_get_node(robot->graph, target_node_id);
    if (target_node == NULL) {
        return MAZE_DIR_STOP;
    }

    return maze_robot_get_direction_to_coord(robot->current_coord, target_node->coord);
}

MazeDirection maze_robot_get_direction_to_coord(MazeCoord from, MazeCoord to) {
    int16_t dx = to.x - from.x;
    int16_t dy = to.y - from.y;

    // Determine primary direction based on which difference is larger
    if (abs(dx) > abs(dy)) {
        // Horizontal movement
        if (dx > 0) {
            return MAZE_DIR_RIGHT;
        } else {
            return MAZE_DIR_LEFT;
        }
    } else {
        // Vertical movement
        if (dy > 0) {
            return MAZE_DIR_FORWARD;
        } else {
            return MAZE_DIR_BACK;
        }
    }
}

/*============================================================================
 * PATH MANAGEMENT
 *============================================================================*/

int maze_robot_set_path(MazeRobot* robot,
                        const uint16_t* path,
                        uint16_t path_length) {
    if (robot == NULL || path == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    // Clear existing path
    maze_robot_clear_path(robot);

    // Allocate new path
    robot->current_path = (uint16_t*)malloc(path_length * sizeof(uint16_t));
    if (robot->current_path == NULL) {
        return MAZE_ERR_NO_MEMORY;
    }

    // Copy path
    memcpy(robot->current_path, path, path_length * sizeof(uint16_t));
    robot->path_length = path_length;
    robot->path_index = 0;

    return MAZE_SUCCESS;
}

void maze_robot_clear_path(MazeRobot* robot) {
    if (robot == NULL) {
        return;
    }

    if (robot->current_path != NULL) {
        free(robot->current_path);
        robot->current_path = NULL;
    }

    robot->path_length = 0;
    robot->path_index = 0;
    robot->cached_direction = MAZE_DIR_STOP;
}

MazeDirection maze_robot_get_next_path_move(MazeRobot* robot) {
    if (robot == NULL || robot->current_path == NULL) {
        return MAZE_DIR_STOP;
    }

    // Check if we've completed the path
    if (robot->path_index >= robot->path_length - 1) {
        maze_robot_clear_path(robot);
        return MAZE_DIR_STOP;
    }

    // Move to next node in path
    robot->path_index++;

    uint16_t next_node_id = robot->current_path[robot->path_index];
    MazeDirection direction = maze_robot_get_direction_to_node(robot, next_node_id);

    // Update position
    maze_robot_update_position(robot, next_node_id);

    // Check if path is complete
    if (robot->path_index >= robot->path_length - 1) {
        maze_robot_clear_path(robot);
    }

    return direction;
}

bool maze_robot_has_path(const MazeRobot* robot) {
    if (robot == NULL) {
        return false;
    }
    return (robot->current_path != NULL && robot->path_length > 0);
}

/*============================================================================
 * EXPLORATION STATE
 *============================================================================*/

void maze_robot_set_exploration_complete(MazeRobot* robot) {
    if (robot == NULL) {
        return;
    }
    robot->exploration_complete = true;
}

bool maze_robot_is_exploration_complete(const MazeRobot* robot) {
    if (robot == NULL) {
        return false;
    }
    return robot->exploration_complete;
}

void maze_robot_set_target_found(MazeRobot* robot) {
    if (robot == NULL) {
        return;
    }
    robot->target_found = true;
}

bool maze_robot_is_target_found(const MazeRobot* robot) {
    if (robot == NULL) {
        return false;
    }
    return robot->target_found;
}

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

uint16_t maze_robot_get_current_node(const MazeRobot* robot) {
    if (robot == NULL) {
        return 0;
    }
    return robot->current_node_id;
}

uint16_t maze_robot_get_start_node(const MazeRobot* robot) {
    if (robot == NULL) {
        return 0;
    }
    return robot->start_node_id;
}

uint16_t maze_robot_get_target_node(const MazeRobot* robot) {
    if (robot == NULL) {
        return 0;
    }
    return robot->target_node_id;
}
