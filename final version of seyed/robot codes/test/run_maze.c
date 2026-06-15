/**
 * @file run_maze.c
 * @brief Generic maze runner — feeds any maze (from _maze_data.h) to the solver.
 *
 * Use via scripts/run_maze.py:
 *   python scripts/run_maze.py ../simulator/mazes/sample_maze4.json
 *
 * The Python script generates test/_maze_data.h from the .json, then compiles
 * and runs this file against it.
 *
 * Compile manually (from robot codes/):
 *   gcc -std=c11 -Wall -Wextra -pedantic -I inc -I test src/maze_graph.c src/maze_robot.c
 *        src/maze_explore.c src/maze_proof.c src/maze_fastrun.c
 *        src/maze_solver.c test/run_maze.c -lm -o build/run_maze.exe
 *
 * Incremental discovery:
 *   - Initially only the START node exists; nothing else is known.
 *   - At each step the solver picks where to go; we "execute" the move by
 *     adding the node, edge, and revealing incident edges from the true maze.
 *   - This mirrors what the real robot does.
 */

#include "maze_solver.h"
#include "maze_robot.h"
#include "maze_graph.h"
#include "_maze_data.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/* ==========================================================================
 * INCREMENTAL DISCOVERY HELPERS
 * ========================================================================== */

static int find_true_idx(MazeCoord c) {
    for (int i = 0; i < MAZE_DATA_N_NODES; i++) {
        if (maze_data_coords[i].x == c.x && maze_data_coords[i].y == c.y) return i;
    }
    return -1;
}

static void reveal_node(MazeGraph* g, uint16_t graph_node_id) {
    if (!g || graph_node_id >= g->node_count) return;

    const MazeNode* mn = maze_graph_get_node(g, graph_node_id);
    if (!mn) return;

    int true_idx = find_true_idx(mn->coord);
    if (true_idx < 0) return;

    for (int e = 0; e < (int)MAZE_DATA_N_EDGES; e++) {
        uint16_t ta = maze_data_edges[e][0];
        uint16_t tb = maze_data_edges[e][1];

        int other_true = -1;
        if ((int)ta == true_idx) other_true = (int)tb;
        else if ((int)tb == true_idx) other_true = (int)ta;
        if (other_true < 0) continue;

        int o = maze_graph_find_node(g, maze_data_coords[other_true]);
        if (o < 0) o = maze_graph_add_node(g, maze_data_coords[other_true]);
        if (o < 0) continue;

        maze_graph_add_edge(g, graph_node_id, (uint16_t)o);
    }
}

/* ==========================================================================
 * MAIN
 * ========================================================================== */

int main(void) {
    printf("=== Maze Run: %s ===\n", maze_data_name);
    printf("  Nodes: %d, Edges: %d\n", MAZE_DATA_N_NODES, (int)MAZE_DATA_N_EDGES);
    printf("  Start: (%d,%d)  Target: (%d,%d)\n\n",
           (int)maze_data_coords[MAZE_DATA_START_IDX].x,
           (int)maze_data_coords[MAZE_DATA_START_IDX].y,
           (int)maze_data_coords[MAZE_DATA_TARGET_IDX].x,
           (int)maze_data_coords[MAZE_DATA_TARGET_IDX].y);

    MazeGraph g;
    MazeRobot r;

    /* ---- Initialise ---- */
    int rc = maze_solver_init(&g, &r, maze_data_coords[MAZE_DATA_START_IDX]);
    if (rc != MAZE_OK) {
        printf("ERROR: maze_solver_init failed (code %d)\n", rc);
        return 1;
    }

    /* ---- Seed: reveal what the robot can see from start ---- */
    reveal_node(&g, 0);

    /* ---- Explore until DONE ---- */
    uint16_t max_steps = 2000;
    uint16_t step       = 0;
    bool     target_seen = false;

    for (step = 0; step < max_steps && r.phase != MAZE_PHASE_DONE; step++) {
        MazeCommand cmd = maze_solver_step(&r, MAZE_V_MAX_FP, MAZE_ACCEL_FP);

        if (cmd == MAZE_CMD_NONE) {
            if (r.phase == MAZE_PHASE_DONE) break;
            continue;
        }

        uint16_t arrived_at = r.current_node;

        if (!target_seen) {
            int tgt = maze_graph_find_node(&g, maze_data_coords[MAZE_DATA_TARGET_IDX]);
            if (tgt >= 0 && (uint16_t)tgt == arrived_at) {
                target_seen = true;
                maze_robot_set_target(&r, (uint16_t)tgt);
            }
        }

        reveal_node(&g, arrived_at);
    }

    if (step >= max_steps) {
        printf("ERROR: timeout after %u steps — did not reach DONE\n", step);
        return 1;
    }

    /* ---- Results ---- */
    printf("-- Results --\n");
    printf("  Steps:             %u\n", (unsigned)step);
    printf("  Commands:          %u\n", (unsigned)r.command_count);
    printf("  Target found:      %s\n", r.target_found ? "yes" : "no");
    printf("  Phase:             %s\n",
           r.phase == MAZE_PHASE_DONE ? "DONE" : "?");
    printf("  Proof state:       %s\n",
           r.proof_state == MAZE_PROOF_PROVEN  ? "PROVEN (early stop)" :
           r.proof_state == MAZE_PROOF_FULL    ? "FULL (mapped everything)" :
           r.proof_state == MAZE_PROOF_DISABLED ? "DISABLED" : "?");
    printf("  Fast path nodes:   %u\n", (unsigned)r.fast_path_length);
    printf("  Fast path time:    %.2f s\n", (double)r.fast_path_time_s);

    uint32_t dist = maze_graph_path_distance(&g, g.start_node, g.target_node);
    printf("  Shortest distance: %u cm\n", (unsigned)dist);

    printf("\n  Command stream (full mission): %s\n", r.command_log);

    if (r.fast_path_length >= 2) {
        char fast_cmds[512];
        uint16_t fc = 0;
        MazeHeading fh = MAZE_HEADING_NONE;
        for (uint16_t i = 1; i < r.fast_path_length && fc < 510; i++) {
            const MazeNode* na = maze_graph_get_node(&g, r.fast_path_nodes[i-1]);
            const MazeNode* nb = maze_graph_get_node(&g, r.fast_path_nodes[i]);
            if (!na || !nb) break;
            MazeCommand mc = maze_robot_compute_command(&fh, na->coord, nb->coord, i == 1);
            if (mc != MAZE_CMD_NONE) fast_cmds[fc++] = (char)mc;
        }
        fast_cmds[fc] = '\0';
        printf("  Fast-run command stream:          %s\n", fast_cmds);
    }

    return 0;
}
