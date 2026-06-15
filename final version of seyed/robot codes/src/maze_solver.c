/**
 * @file maze_solver.c
 * @brief Top-level mission FSM: EXPLORE → RETURN_HOME → FAST_RUN → DONE.
 */

#include "maze_solver.h"
#include "maze_explore.h"
#include "maze_fastrun.h"
#include "maze_proof.h"
#include <string.h>

/* ==========================================================================
 * INTERNAL HELPERS
 * ========================================================================== */

/** Placeholder distance (cm) used when creating a neighbor node for a
 *  sensor-detected branch.  The real distance is filled in when the robot
 *  physically arrives.  20 cm is the standard grid spacing. */
#define PLACEHOLDER_DIST_CM  20

/**
 * @brief  Discover a single branch direction and create a placeholder node
 *         + unexplored edge if one does not already exist.
 *
 * @param  g          Graph.
 * @param  node_id    Current node index.
 * @param  abs_dir    Absolute cardinal direction of the branch (MazeHeading).
 * @param  coord      Current node coordinates.
 */
static void _discover_branch(MazeGraph* g, uint16_t node_id,
                             MazeHeading abs_dir, MazeCoord coord) {
    /* Check whether an edge already exists in this direction */
    uint8_t bit = (uint8_t)(1 << abs_dir);
    if (g->nodes[node_id].connections & bit) return;  /* already known */

    /* Compute placeholder neighbor position */
    MazeCoord nb_coord = coord;
    switch (abs_dir) {
        case MAZE_NORTH: nb_coord.y += PLACEHOLDER_DIST_CM; break;
        case MAZE_SOUTH: nb_coord.y -= PLACEHOLDER_DIST_CM; break;
        case MAZE_EAST:  nb_coord.x += PLACEHOLDER_DIST_CM; break;
        case MAZE_WEST:  nb_coord.x -= PLACEHOLDER_DIST_CM; break;
        default: return;
    }

    /* Find or create the neighbor node */
    int nb = maze_graph_find_node(g, nb_coord);
    if (nb < 0) {
        nb = maze_graph_add_node(g, nb_coord);
        if (nb < 0) return;
    }

    /* Add an unexplored edge between current node and neighbor */
    maze_graph_add_edge(g, node_id, (uint16_t)nb);
}

/**
 * @brief  Process sensor data to discover branches at a node.
 *
 * Converts head-relative sensor flags (can_go_forward / left / right) into
 * absolute cardinal directions using the robot's current heading, then
 * creates placeholder nodes and unexplored edges for each detected branch.
 *
 * "Back" is skipped — it is always "true" from a sensor standpoint, and the
 * return edge is created when the robot actually drives between nodes.
 */
static void _discover_branches(MazeGraph* g, uint16_t node_id,
                               MazeHeading heading, MazeCoord coord,
                               const MazeSensors* sensors) {
    /* If heading is not yet set (very first sensor reading, before any move),
     * default to NORTH — the robot's physical initial placement. */
    MazeHeading h = heading;
    if (h == MAZE_HEADING_NONE) h = MAZE_NORTH;
    if ((uint8_t)h > 3) return;

    /* Look-up table: [heading][0=left 1=fwd 2=right] → absolute direction */
    static const MazeHeading branch_dir[4][3] = {
        /* NORTH */ { MAZE_WEST,  MAZE_NORTH, MAZE_EAST  },
        /* WEST  */ { MAZE_SOUTH, MAZE_WEST,  MAZE_NORTH },
        /* SOUTH */ { MAZE_EAST,  MAZE_SOUTH, MAZE_WEST  },
        /* EAST  */ { MAZE_NORTH, MAZE_EAST,  MAZE_SOUTH },
    };

    if (sensors->can_go_left)    _discover_branch(g, node_id, branch_dir[h][0], coord);
    if (sensors->can_go_forward) _discover_branch(g, node_id, branch_dir[h][1], coord);
    if (sensors->can_go_right)   _discover_branch(g, node_id, branch_dir[h][2], coord);
}

/* ==========================================================================
 * INIT
 * ========================================================================== */

int maze_solver_init(MazeGraph* graph, MazeRobot* robot, MazeCoord start) {
    int rc = maze_graph_init(graph, start);
    if (rc != MAZE_OK) return rc;

    rc = maze_robot_init(robot, graph);
    return rc;
}

/* ==========================================================================
 * POSITION UPDATE (called each time the robot has new sensor data)
 * ========================================================================== */

uint16_t maze_solver_update_position(MazeRobot* robot,
                                     MazeCoord coord,
                                     const MazeSensors* sensors) {
    if (!robot || !robot->graph) return MAZE_INVALID_NODE;

    MazeGraph* g = robot->graph;

    /* Find or create node at this position */
    int node_idx = maze_graph_find_node(g, coord);
    if (node_idx < 0) {
        node_idx = maze_graph_add_node(g, coord);
        if (node_idx < 0) return MAZE_INVALID_NODE;
    }

    uint16_t new_id = (uint16_t)node_idx;

    /* Discover branches from sensor data (runs for ALL nodes, so even the
     * start node gets its unexplored branches populated on the first sensor
     * reading). */
    _discover_branches(g, new_id, robot->heading, coord, sensors);

    /* If we moved from a previous node, add the traversed edge */
    if (new_id != robot->current_node &&
        robot->current_node != MAZE_INVALID_NODE) {
        maze_graph_add_edge(g, robot->current_node, new_id);
        maze_graph_mark_explored(g, robot->current_node, new_id);
    }

    /* Mark arrival and update position tracking */
    maze_robot_arrive(robot, new_id);
    robot->current_node  = new_id;
    robot->current_coord = coord;

    /* Check for target */
    if (sensors->target_reached && !robot->target_found) {
        maze_robot_set_target(robot, new_id);
    }

    return new_id;
}

/* ==========================================================================
 * MAIN STEP FUNCTION
 * ========================================================================== */

MazeCommand maze_solver_step(MazeRobot* robot,
                             int32_t v_max_fp,
                             int32_t a_max_fp) {
    if (!robot || !robot->graph) return MAZE_CMD_NONE;

    switch (robot->phase) {

    case MAZE_PHASE_EXPLORE: {
        MazeCommand cmd = maze_explore_step(robot, robot->use_proof,
                                            v_max_fp, a_max_fp);
        if (cmd != MAZE_CMD_NONE) return cmd;

        /* Exploration step returned NONE → check if we should go home */
        if (!maze_robot_has_path(robot)) {
            /* No more useful frontiers → switch to RETURN_HOME */
            robot->phase = MAZE_PHASE_RETURN_HOME;

            /* If we're already at start, skip to fast run */
            if (robot->current_node == robot->graph->start_node) {
                robot->phase = MAZE_PHASE_FAST_RUN;
                /* fall through to FAST_RUN setup */
            } else {
                /* Compute path home */
                uint16_t path[50];
                uint16_t plen = maze_graph_shortest_path(robot->graph,
                                                          robot->current_node,
                                                          robot->graph->start_node,
                                                          path, 50);
                if (plen >= 2) {
                    maze_robot_set_path(robot, path, plen);
                    return maze_robot_step_path(robot);
                }
                /* Can't reach start — shouldn't happen on known map */
                robot->phase = MAZE_PHASE_DONE;
                return MAZE_CMD_NONE;
            }
        }
        return MAZE_CMD_NONE;
    }

    case MAZE_PHASE_RETURN_HOME: {
        if (maze_robot_has_path(robot)) {
            MazeCommand cmd = maze_robot_step_path(robot);
            if (robot->current_node == robot->graph->start_node) {
                robot->phase = MAZE_PHASE_FAST_RUN;
            }
            return cmd;
        }
        /* Path exhausted but not at start — shouldn't happen */
        robot->phase = MAZE_PHASE_FAST_RUN;
        /* fall through */
    }
    /* fall through */
    case MAZE_PHASE_FAST_RUN: {
        /* Build the fast-run plan (first time entering this phase) */
        if (robot->fast_path_length == 0) {
            if (!maze_fastrun_build_plan(robot, v_max_fp, a_max_fp)) {
                robot->phase = MAZE_PHASE_DONE;
                return MAZE_CMD_NONE;
            }
            /* Start following the fast path */
            robot->path_index = 0;
            robot->path_length = robot->fast_path_length;
            memcpy(robot->path_buffer, robot->fast_path_nodes,
                   robot->fast_path_length * sizeof(uint16_t));
        }

        if (maze_robot_has_path(robot)) {
            MazeCommand cmd = maze_robot_step_path(robot);
            if (robot->current_node == robot->graph->target_node) {
                robot->phase = MAZE_PHASE_DONE;
            }
            return cmd;
        }
        robot->phase = MAZE_PHASE_DONE;
        return MAZE_CMD_NONE;
    }

    case MAZE_PHASE_DONE:
    default:
        return MAZE_CMD_NONE;
    }
}
