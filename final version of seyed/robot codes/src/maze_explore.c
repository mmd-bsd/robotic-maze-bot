/**
 * @file maze_explore.c
 * @brief Target-agnostic frontier exploration (ALGORITHMS.md §1).
 *
 * Implements the P1→P2→P3 priority system.  Before the target is found,
 * ALL frontiers are useful.  After the target is found, the proof module
 * filters out hopeless frontiers.
 */

#include "maze_explore.h"
#include "maze_proof.h"
#include "maze_graph.h"
#include <string.h>

/* ==========================================================================
 * P1: BEST UNEXPLORED BRANCH AT CURRENT NODE
 * ========================================================================== */

bool maze_explore_best_branch(const MazeRobot* robot, uint16_t* chosen) {
    if (!robot || !robot->graph || !chosen) return false;

    uint16_t unex[4];
    uint8_t count = maze_graph_get_unexplored_neighbors(robot->graph,
                                                         robot->current_node,
                                                         unex, 4);
    if (count == 0) return false;

    if (count == 1) {
        *chosen = unex[0];
        return true;
    }

    /* Multiple unexplored — rank by turn cost */
    uint16_t ranked[4];
    maze_robot_rank_branches(robot, unex, count, ranked);
    *chosen = ranked[0];
    return true;
}

/* ==========================================================================
 * P2: NEAREST USEFUL FRONTIER
 * ========================================================================== */

/**
 * Find the nearest useful frontier by Dijkstra distance.
 * If use_proof is enabled and target is found, hopeless frontiers are skipped.
 */
static uint16_t _nearest_useful_frontier(const MazeRobot* robot,
                                         bool use_proof,
                                         int32_t v_max_fp,
                                         int32_t a_max_fp) {
    if (!robot || !robot->graph) return MAZE_INVALID_NODE;

    /* Get all frontiers */
    uint16_t all_frontiers[MAZE_MAX_FRONTIERS];
    uint8_t fcount = maze_robot_find_frontiers(robot, all_frontiers,
                                                MAZE_MAX_FRONTIERS);
    if (fcount == 0) return MAZE_INVALID_NODE;

    /* Filter to useful ones only */
    uint16_t useful[MAZE_MAX_FRONTIERS];
    uint8_t ucount = fcount;

    if (use_proof && robot->target_found) {
        ucount = maze_proof_filter_frontiers(robot, all_frontiers, fcount,
                                             useful, MAZE_MAX_FRONTIERS,
                                             v_max_fp, a_max_fp);
    } else {
        memcpy(useful, all_frontiers, fcount * sizeof(uint16_t));
    }

    if (ucount == 0) return MAZE_INVALID_NODE;

    /* Nearest by Dijkstra distance */
    uint16_t best   = MAZE_INVALID_NODE;
    uint32_t best_d = UINT32_MAX;

    for (uint8_t i = 0; i < ucount; i++) {
        uint32_t d = maze_graph_path_distance(robot->graph,
                                              robot->current_node,
                                              useful[i]);
        if (d < best_d) {
            best_d = d;
            best   = useful[i];
        }
    }

    return best;
}

/* ==========================================================================
 * P3: EXPLORATION COMPLETE CHECK
 * ========================================================================== */

bool maze_explore_is_complete(const MazeRobot* robot) {
    if (!robot || !robot->graph) return true;

    /* Exploration is complete when no frontiers remain */
    uint16_t frontiers[MAZE_MAX_FRONTIERS];
    uint8_t fcount = maze_robot_find_frontiers(robot, frontiers,
                                                MAZE_MAX_FRONTIERS);
    return (fcount == 0);
}

/* ==========================================================================
 * MAIN STEP FUNCTION
 * ========================================================================== */

MazeCommand maze_explore_step(MazeRobot* robot,
                              bool use_proof,
                              int32_t v_max_fp,
                              int32_t a_max_fp) {
    if (!robot || !robot->graph) return MAZE_CMD_NONE;

    /* --- Priority 1: unexplored branch at current node --- */
    uint16_t branch;
    if (maze_explore_best_branch(robot, &branch)) {
        /* Drive down this unexplored edge */
        bool is_first = (robot->heading == MAZE_HEADING_NONE);
        int rc = maze_robot_move_to(robot, branch, is_first);

        if (rc != MAZE_OK) return MAZE_CMD_NONE;

        /* Return the last command logged */
        if (robot->command_count > 0)
            return (MazeCommand)robot->command_log[robot->command_count - 1];
        return MAZE_CMD_NONE;
    }

    /* --- Priority 2: nearest useful frontier --- */
    uint16_t frontier = _nearest_useful_frontier(robot, use_proof,
                                                  v_max_fp, a_max_fp);
    if (frontier != MAZE_INVALID_NODE) {
        /* Compute path to frontier */
        uint16_t path[50];
        uint16_t plen = maze_graph_shortest_path(robot->graph,
                                                  robot->current_node,
                                                  frontier, path, 50);
        if (plen >= 2) {
            /* Set up multi-step path following */
            maze_robot_set_path(robot, path, plen);
            return maze_robot_step_path(robot);
        }
    }

    /* --- Priority 3: no frontiers → exploration complete --- */
    /* Update proof state */
    uint16_t all_f[MAZE_MAX_FRONTIERS];
    uint8_t fc = maze_robot_find_frontiers(robot, all_f, MAZE_MAX_FRONTIERS);

    if (use_proof) {
        uint16_t uf[MAZE_MAX_FRONTIERS];
        uint8_t uc = fc;
        if (robot->target_found) {
            uc = maze_proof_filter_frontiers(robot, all_f, fc,
                                             uf, MAZE_MAX_FRONTIERS,
                                             v_max_fp, a_max_fp);
        }
        robot->proof_state = maze_proof_determine_state(robot, fc, uc);
    } else {
        robot->proof_state = MAZE_PROOF_DISABLED;
    }

    return MAZE_CMD_NONE;   /* caller should switch phase */
}
