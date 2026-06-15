/**
 * @file integration_test.c
 * @brief Full-mission integration test on the 27-node sample maze.
 *
 * Compile:
 *   gcc -std=c11 -Wall -Wextra -I../inc ../src/maze_graph.c ../src/maze_robot.c
 *        ../src/maze_explore.c ../src/maze_proof.c ../src/maze_fastrun.c
 *        ../src/maze_solver.c integration_test.c -lm -o integration_test
 *
 * Strategy (incremental discovery — simulates real robot behaviour):
 *   - All node coordinates and true-maze edges are embedded from sample_maze.json.
 *   - Initially only the START node exists in the graph; nothing else is known.
 *   - At each step the solver tells us which node to move to; we "execute" the
 *     move by adding the target node, the edge, and revealing any adjacent
 *     edges from the true-maze data.
 *   - This mirrors what the real robot does: it discovers the maze as it drives.
 */

#include "maze_solver.h"
#include "maze_robot.h"
#include "maze_graph.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

/* ==========================================================================
 * TRUE MAZE (sample_maze.json embedded as C data)
 * ========================================================================== */

static const MazeCoord true_coords[27] = {
    {0, 0}, {0, 20}, {0, 40}, {0, 60}, {0, 100}, {0, 120},
    {20, 40}, {20, 100}, {20, 120}, {40, 40}, {40, 60}, {40, 80},
    {60, 40}, {60, 80}, {-20, 40}, {-20, 60}, {-20, 80},
    {-40, 40}, {-40, 60}, {-40, 80}, {-40, 100}, {-60, 20},
    {-60, 80}, {-60, 120}, {-80, 40}, {-80, 60}, {-80, 80},
};

#define N_NODES 27
#define START_IDX 0
#define TARGET_IDX 22

static const uint16_t true_edges[][2] = {
    {0, 1}, {1, 2}, {1, 21}, {2, 3}, {2, 6}, {3, 4}, {3, 10}, {3, 15},
    {4, 5}, {4, 7}, {4, 20}, {7, 8}, {9, 10}, {9, 12}, {10, 11},
    {11, 13}, {12, 13}, {14, 15}, {14, 17}, {15, 16}, {16, 19},
    {17, 18}, {17, 24}, {18, 25}, {19, 20}, {19, 22}, {22, 23},
    {22, 26}, {24, 25}, {25, 26},
};
#define N_EDGES (sizeof(true_edges) / sizeof(true_edges[0]))

/* ==========================================================================
 * INCREMENTAL DISCOVERY HELPERS
 * ========================================================================== */

/** Find a true-maze node index matching the given coordinates.
 *  Returns -1 if no match. */
static int find_true_idx(MazeCoord c) {
    for (int i = 0; i < N_NODES; i++) {
        if (true_coords[i].x == c.x && true_coords[i].y == c.y) return i;
    }
    return -1;
}

/** Reveal all true-maze edges incident to the given graph node.
 *  Adds neighbor nodes and unexplored edges for every connection
 *  in the true maze. */
static void reveal_node(MazeGraph* g, uint16_t graph_node_id) {
    if (!g || graph_node_id >= g->node_count) return;

    /* Get coordinates of the graph node */
    const MazeNode* mn = maze_graph_get_node(g, graph_node_id);
    if (!mn) return;

    /* Find matching true-maze node index */
    int true_idx = find_true_idx(mn->coord);
    if (true_idx < 0) return;

    /* For every edge in the true maze incident to this node,
     * ensure the neighbor exists in the graph and add the edge. */
    for (int e = 0; e < (int)N_EDGES; e++) {
        uint16_t ta = true_edges[e][0];
        uint16_t tb = true_edges[e][1];

        int other_true = -1;
        if ((int)ta == true_idx) other_true = (int)tb;
        else if ((int)tb == true_idx) other_true = (int)ta;
        if (other_true < 0) continue;

        /* Add neighbor node if not already in graph */
        int o = maze_graph_find_node(g, true_coords[other_true]);
        if (o < 0) o = maze_graph_add_node(g, true_coords[other_true]);
        if (o < 0) continue;

        /* Add edge if not already present */
        maze_graph_add_edge(g, graph_node_id, (uint16_t)o);
    }
}

/* ==========================================================================
 * TEST
 * ========================================================================== */

static int tests_run = 0, tests_failed = 0;
#define T(n)  do { tests_run++; printf("  %-52s ", n); } while(0)
#define P()   do { printf("PASS\n"); } while(0)
#define F(m)  do { printf("FAIL: %s\n", m); tests_failed++; } while(0)

int main(void) {
    printf("=== Integration Test — Full Mission on sample_maze ===\n");

    MazeGraph g;
    MazeRobot r;

    /* ---- Initialise ---- */
    T("Init solver at start (0,0)");
    int rc = maze_solver_init(&g, &r, true_coords[START_IDX]);
    if (rc != MAZE_OK) { F("init failed"); goto done; }
    if (r.phase != MAZE_PHASE_EXPLORE) { F("should start in EXPLORE"); goto done; }
    P();

    /* ---- Seed: reveal what the robot can see from start ---- */
    T("Reveal start node and its edges");
    reveal_node(&g, 0);  /* start is always graph node 0 */
    if (g.node_count < 2) { F("should reveal neighbors"); goto done; }
    P();

    /* ---- Explore until DONE ---- */
    uint16_t max_steps = 500;
    uint16_t step       = 0;
    bool     target_seen = false;

    T("Full mission loop");
    for (step = 0; step < max_steps && r.phase != MAZE_PHASE_DONE; step++) {
        MazeCommand cmd = maze_solver_step(&r, MAZE_V_MAX_FP, MAZE_ACCEL_FP);

        if (cmd == MAZE_CMD_NONE) {
            if (r.phase == MAZE_PHASE_DONE) break;
            continue;
        }

        /* Simulate the move: after the robot physically drives to the
         * destination node, reveal that node and its incident edges
         * from the true maze map. */
        uint16_t arrived_at = r.current_node;

        /* Find the graph node at the target coordinates and set it */
        if (!target_seen) {
            int tgt = maze_graph_find_node(&g, true_coords[TARGET_IDX]);
            if (tgt >= 0 && (uint16_t)tgt == arrived_at) {
                target_seen = true;
                maze_robot_set_target(&r, (uint16_t)tgt);
            }
        }

        /* Reveal edges at the arrived node (discover what the robot sees) */
        reveal_node(&g, arrived_at);
    }

    if (step >= max_steps) { F("timeout — did not reach DONE"); goto done; }
    if (r.phase != MAZE_PHASE_DONE) { F("should end in DONE phase"); goto done; }
    printf("(%u steps, %u commands) ", step, r.command_count);
    P();

    /* ---- Verify mission structure ---- */
    T("Mission completed all phases");
    if (r.phase != MAZE_PHASE_DONE) { F("not DONE"); goto done; }
    if (!r.target_found) { F("target not found"); goto done; }
    P();

    T("Command log non-empty");
    if (r.command_count == 0) { F("no commands logged"); goto done; }
    printf("(%u cmds: ", r.command_count);
    for (uint16_t i = 0; i < 20 && i < r.command_count; i++)
        putchar(r.command_log[i]);
    if (r.command_count > 20) printf("...");
    printf(") ");
    P();

    T("Fast path computed");
    if (r.fast_path_length == 0) { F("no fast path"); goto done; }
    if (r.fast_path_time_s <= 0.0f) { F("fast path time zero"); goto done; }
    printf("(len=%u, time=%.2fs) ", r.fast_path_length, (double)r.fast_path_time_s);
    P();

    T("Proof state terminal");
    if (r.proof_state != MAZE_PROOF_PROVEN &&
        r.proof_state != MAZE_PROOF_FULL &&
        r.proof_state != MAZE_PROOF_DISABLED) {
        printf("(got %d) ", r.proof_state);
        F("proof state should be terminal");
        goto done;
    }
    printf("(%s) ",
           r.proof_state == MAZE_PROOF_PROVEN ? "PROVEN" :
           r.proof_state == MAZE_PROOF_FULL   ? "FULL" : "DISABLED");
    P();

    T("Start→target path exists");
    uint32_t dist = maze_graph_path_distance(&g, g.start_node, g.target_node);
    if (dist == UINT32_MAX) { F("no path"); goto done; }
    if (dist == 0) { F("zero distance"); goto done; }
    printf("(%u cm) ", (unsigned)dist);
    P();

    /* ---- Print full command log ---- */
    printf("\n  Command stream: %s\n", r.command_log);

done:
    printf("\n%d tests, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
