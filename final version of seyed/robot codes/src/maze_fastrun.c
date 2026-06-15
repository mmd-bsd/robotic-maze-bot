/**
 * @file maze_fastrun.c
 * @brief Acceleration-aware fast run (ALGORITHMS.md §6).
 *
 * Trapezoidal velocity profile for minimum-TIME path finding and fast-run
 * trajectory precomputation.
 */

#include "maze_fastrun.h"
#include "maze_graph.h"
#include <math.h>
#include <float.h>
#include <string.h>

/* ==========================================================================
 * FIXED-POINT HELPERS
 * ========================================================================== */

static inline float fp_to_f(int32_t v) { return (float)v / 100.0f; }

/* ==========================================================================
 * TRAPEZOIDAL VELOCITY PROFILE
 * ========================================================================== */

uint32_t maze_run_time_cs(float L_cm, int32_t v_max_fp, int32_t a_max_fp) {
    if (L_cm <= 0.0f) return 0;

    float v = fp_to_f(v_max_fp);
    float a = fp_to_f(a_max_fp);

    float d_acc = (v * v) / (2.0f * a);
    float time_s;

    if (2.0f * d_acc <= L_cm) {
        /* Trapezoid: accel → cruise → decel */
        time_s = 2.0f * (v / a) + (L_cm - 2.0f * d_acc) / v;
    } else {
        /* Triangle: accel → decel (never reaches v_max) */
        time_s = 2.0f * sqrtf(a * L_cm) / a;
    }

    return (uint32_t)(time_s * 100.0f + 0.5f);
}

float maze_speed_at_cm(float s, float L_cm, int32_t v_max_fp, int32_t a_max_fp) {
    if (L_cm <= 0.0f || s < 0.0f) return 0.0f;
    if (s > L_cm) s = L_cm;

    float v = fp_to_f(v_max_fp);
    float a = fp_to_f(a_max_fp);

    float accel_dist = (v * v) / (2.0f * a);
    float decel_start = L_cm - accel_dist;

    float speed;
    if (decel_start <= accel_dist) {
        /* Triangle — no cruise phase */
        float peak_dist = L_cm / 2.0f;
        if (s <= peak_dist)
            speed = sqrtf(2.0f * a * s);
        else
            speed = sqrtf(2.0f * a * (L_cm - s));
    } else {
        if (s <= accel_dist)
            speed = sqrtf(2.0f * a * s);
        else if (s <= decel_start)
            speed = v;
        else
            speed = sqrtf(2.0f * a * (L_cm - s));
    }

    return speed;
}

/* ==========================================================================
 * TIME-OPTIMAL PATH — stop-graph Dijkstra
 * ========================================================================== */

uint16_t maze_time_optimal_path(const MazeGraph* graph,
                                uint16_t start,
                                uint16_t target,
                                int32_t v_max_fp,
                                int32_t a_max_fp,
                                uint16_t* path_out,
                                uint16_t max_len) {
    if (!graph || !path_out || max_len < 2) return 0;
    if (start >= graph->node_count || target >= graph->node_count) return 0;
    if (start == target) { path_out[0] = start; return 1; }

    uint16_t n = graph->node_count;
    float    best_time[MAZE_MAX_NODES];
    uint16_t prev_turn[MAZE_MAX_NODES];
    bool     settled[MAZE_MAX_NODES];

    for (uint16_t i = 0; i < n; i++) {
        best_time[i] = FLT_MAX;
        prev_turn[i] = MAZE_INVALID_NODE;
        settled[i] = false;
    }
    best_time[start] = 0.0f;

    for (uint16_t iter = 0; iter < n; iter++) {
        /* Find unvisited node with minimum best_time */
        uint16_t u = MAZE_INVALID_NODE;
        float    min_t = FLT_MAX;
        for (uint16_t i = 0; i < n; i++) {
            if (!settled[i] && best_time[i] < min_t) {
                min_t = best_time[i];
                u = i;
            }
        }
        if (u == MAZE_INVALID_NODE || u == target) break;
        settled[u] = true;

        /* From u, walk straight in each cardinal direction */
        for (uint8_t d = 0; d < 4; d++) {
            MazeHeading dir = (MazeHeading)d;

            uint16_t cur = u;
            float    dist_sum = 0.0f;

            for (;;) {
                /* Find explored neighbor in direction dir */
                uint16_t neighbors[4];
                uint8_t deg = maze_graph_get_explored_neighbors(graph, cur, neighbors, 4);
                int16_t next = -1;
                float   elen = 0.0f;

                for (uint8_t k = 0; k < deg; k++) {
                    MazeHeading h = maze_heading_between(graph->nodes[cur].coord,
                                                         graph->nodes[neighbors[k]].coord);
                    if (h == dir) {
                        int eidx = maze_graph_find_edge(graph, cur, neighbors[k]);
                        if (eidx >= 0) {
                            next = (int16_t)neighbors[k];
                            elen = (float)graph->edges[eidx].length_cm;
                            break;
                        }
                    }
                }
                if (next < 0) break;

                dist_sum += elen;
                cur = (uint16_t)next;

                /* run_time for the ENTIRE straight from u to cur */
                float rt_s = (float)maze_run_time_cs(dist_sum, v_max_fp, a_max_fp) / 100.0f;
                float total_t = best_time[u] + rt_s;

                if (total_t < best_time[cur]) {
                    best_time[cur] = total_t;
                    prev_turn[cur] = u;
                }
            }
        }
    }

    if (best_time[target] >= FLT_MAX) return 0;

    /* Reconstruct turn-node path */
    uint16_t turn_path[MAZE_MAX_NODES];
    uint16_t tlen = 0;
    uint16_t cur = target;
    while (cur != MAZE_INVALID_NODE && tlen < MAZE_MAX_NODES) {
        turn_path[tlen++] = cur;
        cur = prev_turn[cur];
    }

    /* Reverse */
    for (uint16_t i = 0; i < tlen / 2; i++) {
        uint16_t tmp = turn_path[i];
        turn_path[i] = turn_path[tlen - 1 - i];
        turn_path[tlen - 1 - i] = tmp;
    }

    /* Expand: fill intermediate nodes between consecutive turn nodes */
    uint16_t expanded[MAZE_MAX_PATH_LENGTH];
    uint16_t elen = 0;

    for (uint16_t ti = 0; ti < tlen && elen < max_len; ti++) {
        if (ti == 0) {
            expanded[elen++] = turn_path[ti];
            continue;
        }

        uint16_t from = turn_path[ti - 1];
        uint16_t to   = turn_path[ti];

        /* Walk from `from` to `to` filling intermediate nodes */
        MazeHeading dir = maze_heading_between(graph->nodes[from].coord,
                                                graph->nodes[to].coord);

        uint16_t walk = from;
        while (walk != to && elen < max_len) {
            uint16_t neighbors[4];
            uint8_t deg = maze_graph_get_explored_neighbors(graph, walk, neighbors, 4);
            int16_t next = -1;
            for (uint8_t k = 0; k < deg; k++) {
                MazeHeading h = maze_heading_between(graph->nodes[walk].coord,
                                                     graph->nodes[neighbors[k]].coord);
                if (h == dir) {
                    next = (int16_t)neighbors[k];
                    break;
                }
            }
            if (next < 0) break;
            walk = (uint16_t)next;
            expanded[elen++] = walk;
        }
    }

    if (elen > max_len) elen = max_len;
    memcpy(path_out, expanded, elen * sizeof(uint16_t));
    return elen;
}

uint32_t maze_time_optimal_cost(const MazeGraph* graph,
                                uint16_t start,
                                uint16_t target,
                                int32_t v_max_fp,
                                int32_t a_max_fp) {
    if (!graph || start >= graph->node_count || target >= graph->node_count)
        return UINT32_MAX;

    uint16_t n = graph->node_count;
    float    best_time[MAZE_MAX_NODES];
    bool     settled[MAZE_MAX_NODES];

    for (uint16_t i = 0; i < n; i++) {
        best_time[i] = FLT_MAX;
        settled[i] = false;
    }
    best_time[start] = 0.0f;

    for (uint16_t iter = 0; iter < n; iter++) {
        uint16_t u = MAZE_INVALID_NODE;
        float    min_t = FLT_MAX;
        for (uint16_t i = 0; i < n; i++) {
            if (!settled[i] && best_time[i] < min_t) {
                min_t = best_time[i];
                u = i;
            }
        }
        if (u == MAZE_INVALID_NODE || u == target) break;
        settled[u] = true;

        for (uint8_t d = 0; d < 4; d++) {
            MazeHeading dir = (MazeHeading)d;
            uint16_t cur = u;
            float    dist_sum = 0.0f;

            for (;;) {
                uint16_t neighbors[4];
                uint8_t deg = maze_graph_get_explored_neighbors(graph, cur, neighbors, 4);
                int16_t next = -1;
                float   elen = 0.0f;
                for (uint8_t k = 0; k < deg; k++) {
                    MazeHeading h = maze_heading_between(graph->nodes[cur].coord,
                                                         graph->nodes[neighbors[k]].coord);
                    if (h == dir) {
                        int eidx = maze_graph_find_edge(graph, cur, neighbors[k]);
                        if (eidx >= 0) {
                            next = (int16_t)neighbors[k];
                            elen = (float)graph->edges[eidx].length_cm;
                            break;
                        }
                    }
                }
                if (next < 0) break;
                dist_sum += elen;
                cur = (uint16_t)next;
                float rt_s = (float)maze_run_time_cs(dist_sum, v_max_fp, a_max_fp) / 100.0f;
                float total_t = best_time[u] + rt_s;
                if (total_t < best_time[cur]) {
                    best_time[cur] = total_t;
                }
            }
        }
    }

    if (best_time[target] >= FLT_MAX) return UINT32_MAX;
    return (uint32_t)(best_time[target] * 100.0f + 0.5f);
}

/* ==========================================================================
 * FAST-RUN PLAN (trajectory precomputation)
 * ========================================================================== */

bool maze_fastrun_build_plan(MazeRobot* robot,
                             int32_t v_max_fp,
                             int32_t a_max_fp) {
    if (!robot || !robot->graph) return false;
    if (!robot->target_found) return false;

    /* Find time-optimal path */
    uint16_t path_nodes[MAZE_MAX_PATH_LENGTH];
    uint16_t plen = maze_time_optimal_path(robot->graph,
                                           robot->graph->start_node,
                                           robot->graph->target_node,
                                           v_max_fp, a_max_fp,
                                           path_nodes, MAZE_MAX_PATH_LENGTH);
    if (plen == 0) return false;

    /* Store the path */
    memcpy(robot->fast_path_nodes, path_nodes, plen * sizeof(uint16_t));
    robot->fast_path_length = plen;

    /* Compute total time */
    robot->fast_path_time_s = (float)maze_time_optimal_cost(robot->graph,
                                                            robot->graph->start_node,
                                                            robot->graph->target_node,
                                                            v_max_fp, a_max_fp) / 100.0f;

    /* Build trajectory: group consecutive nodes into collinear straight runs,
     * sample each run at constant time intervals. */
    /* Simplified: store one trajectory point per node for now.
     * Full trapezoidal trajectory can be added later when the real robot
     * needs speed feed-forward. */

    /* For now, just store the path — trajectory precomputation is optional */
    robot->fast_traj_count = 0;
    robot->fast_traj = NULL;   /* No dynamic allocation */

    return true;
}
