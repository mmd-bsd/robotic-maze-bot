/**
 * @file maze_proof.c
 * @brief Time-based early-stop branch & bound (ALGORITHMS.md §2).
 *
 * Uses the shared fast-run module for time computations.
 */

#include "maze_proof.h"
#include "maze_fastrun.h"
#include "maze_graph.h"

static inline float fp_to_f(int32_t v) { return (float)v / 100.0f; }

/* ==========================================================================
 * PUBLIC API
 * ========================================================================== */

uint32_t maze_proof_frontier_lb(const MazeRobot* robot,
                                uint16_t frontier,
                                int32_t v_max_fp) {
    if (!robot || !robot->graph) return UINT32_MAX;
    if (!robot->target_found)    return 0;
    if (frontier >= robot->graph->node_count) return UINT32_MAX;

    float v_max = fp_to_f(v_max_fp);

    /* dist_known(start → frontier) */
    uint32_t dist_known = maze_graph_path_distance(robot->graph,
                                                    robot->graph->start_node,
                                                    frontier);
    if (dist_known == UINT32_MAX) return UINT32_MAX;

    /* straight_line(frontier → target) */
    MazeCoord fc = robot->graph->nodes[frontier].coord;
    MazeCoord tc = robot->graph->nodes[robot->graph->target_node].coord;
    uint32_t straight = maze_straight_line_cm(fc, tc);

    /* LB_time = (dist_known + straight) / v_max  →  centiseconds */
    float lb_s = (float)(dist_known + straight) / v_max;
    return (uint32_t)(lb_s * 100.0f + 0.5f);
}

uint32_t maze_proof_best_time(const MazeRobot* robot,
                              int32_t v_max_fp,
                              int32_t a_max_fp) {
    if (!robot || !robot->graph) return UINT32_MAX;
    if (!robot->target_found)    return UINT32_MAX;

    return maze_time_optimal_cost(robot->graph,
                                  robot->graph->start_node,
                                  robot->graph->target_node,
                                  v_max_fp, a_max_fp);
}

bool maze_proof_is_useful(const MazeRobot* robot,
                          uint16_t frontier,
                          int32_t v_max_fp,
                          int32_t a_max_fp) {
    if (!robot || !robot->graph) return true;
    if (!robot->target_found)    return true;

    uint32_t lb    = maze_proof_frontier_lb(robot, frontier, v_max_fp);
    uint32_t best  = maze_proof_best_time(robot, v_max_fp, a_max_fp);

    if (lb == UINT32_MAX || best == UINT32_MAX) return true;
    return lb < best;
}

uint8_t maze_proof_filter_frontiers(const MazeRobot* robot,
                                    const uint16_t* frontiers,
                                    uint8_t fcount,
                                    uint16_t* useful,
                                    uint8_t max_useful,
                                    int32_t v_max_fp,
                                    int32_t a_max_fp) {
    uint8_t count = 0;
    if (!robot || !frontiers || !useful || max_useful == 0) return 0;
    if (!robot->target_found) return fcount;  /* all pass through */

    for (uint8_t i = 0; i < fcount && count < max_useful; i++) {
        if (maze_proof_is_useful(robot, frontiers[i], v_max_fp, a_max_fp)) {
            useful[count++] = frontiers[i];
        }
    }
    return count;
}

MazeProofState maze_proof_determine_state(const MazeRobot* robot,
                                          uint8_t total_frontiers,
                                          uint8_t useful_count) {
    if (!robot->use_proof) return MAZE_PROOF_DISABLED;
    if (!robot->target_found) return MAZE_PROOF_SEARCHING;

    if (useful_count == 0) {
        if (total_frontiers == 0) return MAZE_PROOF_FULL;
        return MAZE_PROOF_PROVEN;
    }
    return MAZE_PROOF_CHECKING;
}
