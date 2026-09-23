/**
 * @file brain_host.c
 * @brief Host validation of the decision core: drive the true maze through the
 *        EXACT brain call loop, and check the plans it hands back.
 *
 * Run via scripts/run_brain.py, which generates test/_brain_data.h first:
 *
 *   python scripts/run_brain.py ../simulator/mazes/real_field.json
 *
 * WHAT MAKES THIS A REAL TEST
 * ---------------------------
 * The existing runners (run_maze.c, integration_test.c) both "drive" by
 * calling reveal_node(), which reads the TRUE maze and hands the solver every
 * neighbouring edge at its true coordinate.  That skips the sensor-driven
 * discovery path entirely — including _discover_branch() and its 20 cm
 * placeholder — which is precisely the path the real robot exercises.
 *
 * This harness models a robot instead:
 *
 *   - It stops at a node only where the firmware's own junction test would fire
 *     (a side sensor on the line, or a dead end).  A pure straight passthrough
 *     is driven through WITHOUT a report, so the brain has to reconstruct it
 *     from `dist_cm` — the case that would break a naive implementation.
 *   - It reports only what the sensors can see: exits relative to its own
 *     facing, a target flag, and a driven distance.  It never tells the brain
 *     a coordinate, a node id, or a compass direction.
 *
 * Then it checks the results without asking the brain to grade its own work:
 *
 *   1. Both plans are replayed on the TRUE maze from the true pose.  Home must
 *      land on the start node, fast must land on the target, and every step
 *      must follow a real edge.
 *   2. The fast path's time is compared against the time-optimal path computed
 *      over the ENTIRE true maze — the maze the brain never got to see.  If the
 *      early-stop proof is sound, these must be equal: that is the whole claim
 *      `proven_optimal` makes.
 */

#include "brain.h"
#include "maze_config.h"
#include "maze_graph.h"
#include "maze_fastrun.h"

/* Reuses the same generated header the other runners use, so there is one
 * JSON -> C generator in the project and one definition of the true maze.
 * See scripts/run_maze.py:generate_header(). */
#include "_maze_data.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ==========================================================================
 * THE TRUE MAZE, AS A SIMULATED ROBOT SEES IT
 * ========================================================================== */

/**
 * @brief  The node one cell from `node` in direction `dir`, or -1.
 *
 * @param dir  MazeHeading: 0=N, 1=W, 2=S, 3=E.  Y is up.
 */
static int true_neighbor(int node, int dir) {
    int16_t dx = 0, dy = 0;
    switch (dir) {
        case 0: dy = +MAZE_CELL_CM; break;
        case 1: dx = -MAZE_CELL_CM; break;
        case 2: dy = -MAZE_CELL_CM; break;
        case 3: dx = +MAZE_CELL_CM; break;
        default: return -1;
    }
    MazeCoord want;
    want.x = (int16_t)(maze_data_coords[node].x + dx);
    want.y = (int16_t)(maze_data_coords[node].y + dy);
    for (int e = 0; e < (int)MAZE_DATA_N_EDGES; e++) {
        int a = maze_data_edges[e][0];
        int b = maze_data_edges[e][1];
        int other = (a == node) ? b : ((b == node) ? a : -1);
        if (other < 0) continue;
        if (maze_data_coords[other].x == want.x &&
            maze_data_coords[other].y == want.y) return other;
    }
    return -1;
}

/**
 * @brief  Would the real robot stop at `node` having driven into it facing `dir`?
 *
 * Mirrors the firmware's junction test (firmware/Core/Src/main.c:1202-1214,
 * documented in SENSORS.md §2):
 *
 *     left_poss  = s[2] && (s[3] || s[4] || s[5] || s[6])
 *     right_poss = s[7] && (s[3] || s[4] || s[5] || s[6])
 *     dead end   = centre group all off, held for head_delay >= 50 ticks
 *
 * The `&& centre-on-line` guard is what makes this exact: a side sensor only
 * counts as a branch while the bar is still on the line, so a junction is
 * declared exactly when a LATERAL exit exists.  A node with only a forward exit
 * is a straight passthrough: no side sensor fires and the centre never goes
 * dark, so the robot drives through and never reports.  A dead end is the
 * opposite — the centre does go dark, so it always stops.
 */
static int stops_here(int node, int dir) {
    /* The target disc is an unconditional stop: the all-black row test fires
     * wherever it is, junction or not. */
    if (node == MAZE_DATA_TARGET_IDX) return 1;

    int fwd   = true_neighbor(node, dir) >= 0;
    int left  = true_neighbor(node, (dir + 1) & 3) >= 0;
    int right = true_neighbor(node, (dir + 3) & 3) >= 0;
    return (left || right || !fwd);
}

/** Fill the brain's input from what a robot standing at `node` facing `dir`
 *  would actually sense.  No coordinates, no ids, no compass. */
static void fill_report(BrainIn* in, int node, int dir, uint16_t dist_cm) {
    memset(in, 0, sizeof *in);
    in->front  = (true_neighbor(node, dir) >= 0) ? 1 : 0;
    in->left   = (true_neighbor(node, (dir + 1) & 3) >= 0) ? 1 : 0;
    in->right  = (true_neighbor(node, (dir + 3) & 3) >= 0) ? 1 : 0;
    in->back   = 1;                                   /* reversing is always possible */
    in->target = (node == MAZE_DATA_TARGET_IDX) ? 1 : 0;
    in->dist_cm = dist_cm;
}

/** Drive forward from `node` along `dir` cell by cell until the robot stops.
 *  Returns 0 on success, negative if it ran off the true maze. */
static int drive_until_stop(int* node, int dir, uint16_t* dist_cm) {
    *dist_cm = 0;
    for (int guard = 0; guard < MAZE_DATA_N_NODES + 2; guard++) {
        int next = true_neighbor(*node, dir);
        if (next < 0) return -1;          /* no line ahead — harness bug */
        *node = next;
        *dist_cm = (uint16_t)(*dist_cm + MAZE_CELL_CM);
        if (stops_here(*node, dir)) return 0;
    }
    return -2;
}

/* ==========================================================================
 * CHECKING THE RESULT WITHOUT TRUSTING THE BRAIN
 * ========================================================================== */

/**
 * @brief  Replay a command string on the TRUE maze.
 *
 * The only thing this shares with the brain is the meaning of F/L/R/B, which is
 * the interface itself — so it is an independent check of the brain's graph,
 * proof, and dead reckoning, not a restatement of them.
 *
 * @return 0 if it ends exactly on `goal`, negative otherwise.
 */
static int replay(const char* plan, int* node, int* heading, int goal,
                  int* steps, int trace) {
    static const char* k_name[4] = { "N", "W", "S", "E" };
    *steps = 0;
    if (trace) printf("      start at true node %d (%d,%d) facing %s\n",
                      *node, maze_data_coords[*node].x, maze_data_coords[*node].y,
                      k_name[*heading]);
    for (const char* p = plan; *p; p++) {
        switch (*p) {
            case 'F': break;
            case 'L': *heading = (*heading + 1) & 3; break;
            case 'R': *heading = (*heading + 3) & 3; break;
            case 'B': *heading = (*heading + 2) & 3; break;
            default:  return -1;              /* not a command we emit */
        }

        /* ONE COMMAND IS ONE LEG, NOT ONE CELL.  The robot turns as told and
         * then drives until it reaches somewhere it would stop — so a plan
         * command can carry it several cells.  Advancing a single cell per
         * command here would reject perfectly good plans. */
        int      from = *node;
        uint16_t drove = 0;
        int rc = drive_until_stop(node, *heading, &drove);
        if (rc != 0) {
            if (trace) printf("      '%c' -> %s: NO LINE from node %d (%d,%d)\n",
                              *p, k_name[*heading], from,
                              maze_data_coords[from].x,
                              maze_data_coords[from].y);
            return -2;
        }
        (*steps)++;
        if (trace) printf("      '%c' -> %s -> drove %u cm -> node %d (%d,%d)\n",
                          *p, k_name[*heading], drove, *node,
                          maze_data_coords[*node].x, maze_data_coords[*node].y);
    }
    return (*node == goal) ? 0 : -3;
}

/**
 * @brief  Time-optimal cost over the WHOLE true maze, start -> target.
 *
 * This is the number the brain is never allowed to look at during the search.
 * Comparing it against the brain's answer is the test of the early-stop proof:
 * if the brain stopped early and the times agree, stopping early cost nothing.
 */
static float true_optimal_time_s(void) {
    MazeGraph g;
    uint16_t  gid[MAZE_DATA_N_NODES];   /* data index -> graph node index */

    /* maze_graph_init() puts the START at index 0, so the graph's node
     * numbering will not match the data file's.  Keep the mapping rather than
     * assuming the two line up. */
    maze_graph_init(&g, maze_data_coords[MAZE_DATA_START_IDX]);
    gid[MAZE_DATA_START_IDX] = 0;

    for (int i = 0; i < MAZE_DATA_N_NODES; i++) {
        if (i == MAZE_DATA_START_IDX) continue;
        gid[i] = (uint16_t)maze_graph_add_node(&g, maze_data_coords[i]);
    }
    for (int e = 0; e < (int)MAZE_DATA_N_EDGES; e++) {
        maze_graph_add_edge(&g, gid[maze_data_edges[e][0]],
                                gid[maze_data_edges[e][1]]);
        /* Edges start unexplored.  The fast-run planner searches the map the
         * robot has DRIVEN, so an unexplored edge is not usable — mark them all,
         * or the oracle reports "no path" on a perfectly connected maze. */
        maze_graph_mark_explored(&g, gid[maze_data_edges[e][0]],
                                     gid[maze_data_edges[e][1]]);
    }

    g.target_node = gid[MAZE_DATA_TARGET_IDX];

    uint32_t cost = maze_time_optimal_cost(&g, gid[MAZE_DATA_START_IDX],
                                           gid[MAZE_DATA_TARGET_IDX],
                                           MAZE_V_MAX_FP, MAZE_ACCEL_FP);
    if (cost == UINT32_MAX) return -1.0f;
    return (float)cost / 100.0f;
}

/* ==========================================================================
 * MAIN
 * ========================================================================== */

static int tests_run = 0, tests_failed = 0;
#define T(n) do { tests_run++; printf("  %-56s ", n); fflush(stdout); } while (0)
#define OK() do { printf("PASS\n"); } while (0)
#define NO(m) do { printf("FAIL: %s\n", m); tests_failed++; } while (0)

int main(void) {
    printf("=== Brain Host Test: %s ===\n", maze_data_name);
    printf("  Nodes: %d, Edges: %d\n",
           MAZE_DATA_N_NODES, (int)MAZE_DATA_N_EDGES);
    printf("  Start: (%d,%d)  Target: (%d,%d)   cell: %d cm\n\n",
           maze_data_coords[MAZE_DATA_START_IDX].x,
           maze_data_coords[MAZE_DATA_START_IDX].y,
           maze_data_coords[MAZE_DATA_TARGET_IDX].x,
           maze_data_coords[MAZE_DATA_TARGET_IDX].y,
           MAZE_CELL_CM);

    /* ---- the mission ------------------------------------------------ */

    int      cur     = MAZE_DATA_START_IDX;   /* true maze node             */
    int      heading = 0;                     /* the brain's frame's north  */
    uint16_t dist    = 0;
    int      reports = 0;
    int      skipped = 0;                     /* cells driven through       */
    int      guard;

    BrainIn     in;
    BrainStatus st;
    char        decisions[4096];
    int         dlen = 0;

    brain_init();

    /* The first report is at the start node with dist 0 — this is how the
     * brain learns the start's exits. */
    fill_report(&in, cur, heading, 0);

    memset(&st, 0, sizeof st);

    printf("-- Decisions --\n");
    for (guard = 0; guard < 4000; guard++) {
        uint8_t move = brain_step(&in);
        brain_status(&st);

        /* Buffer every decision, then print.  Printing inline would interleave
         * with the ABORT messages below and make a failure hard to read. */
        if (dlen < (int)sizeof decisions - 64) {
            char mv[8];
            if (move == BRAIN_DONE) snprintf(mv, sizeof mv, "DONE");
            else                    snprintf(mv, sizeof mv, "%c", move);
            /* Print BOTH frames.  The brain's coordinates are its own
             * dead-reckoned frame; the true node's are the maze file's.  They
             * must differ by a fixed rotation+translation for the whole
             * mission — any drift shows up here as the offset changing. */
            dlen += snprintf(decisions + dlen, sizeof decisions - dlen,
                             "  #%-3d brain(%4d,%4d)  true %-2d (%4d,%4d)  "
                             "drove %3u cm  F%d L%d R%d%s  -> %s\n",
                             reports, st.x, st.y,
                             cur, maze_data_coords[cur].x,
                             maze_data_coords[cur].y, in.dist_cm,
                             in.front, in.left, in.right,
                             in.target ? "  TARGET" : "", mv);
        }

        if (move == BRAIN_DONE) break;

        /* execute the move */
        switch (move) {
            case 'F': break;
            case 'L': heading = (heading + 1) & 3; break;
            case 'R': heading = (heading + 3) & 3; break;
            case 'B': heading = (heading + 2) & 3; break;
            default:
                printf("%s", decisions);
                printf("\n  ABORT: brain returned '%c' (0x%02X), not a move\n",
                       move, move);
                goto done;
                break;
        }

        /* drive forward to the next place the robot would stop */
        {
            int before = cur;
            int rc = drive_until_stop(&cur, heading, &dist);
            if (rc != 0) {
                printf("%s", decisions);
                printf("\n  ABORT: no line ahead from true node %d facing %d\n",
                       before, heading);
                goto done;
            }
        }

        skipped += (dist / MAZE_CELL_CM) - 1;   /* passthroughs driven through */
        reports++;
        fill_report(&in, cur, heading, dist);
    }

done:
    printf("%s", decisions);
    printf("\n-- Results --\n");
    printf("  Reports:            %d\n", reports);
    printf("  Passthroughs skipped: %d cells driven through without a report\n",
           skipped);
    printf("  Final phase:        %d  (3 = DONE)\n", st.phase);
    printf("  Target found:       %s\n", st.target_found ? "yes" : "no");

    /* ---- 1. did it finish? ------------------------------------------ */

    T("brain_step() reached BRAIN_DONE");
    if (st.finished) OK(); else NO("still exploring");
    if (!st.finished) { printf("\n%d/%d checks passed\n", tests_run - tests_failed, tests_run); return 1; }

    const char* home = brain_home_path();
    const char* fast = brain_fast_path();

    printf("  Home path (%2zu):   %s\n", strlen(home), *home ? home : "(already home)");
    printf("  Fast path (%2zu):   %s\n", strlen(fast), *fast ? fast : "(none)");
    printf("  Fast time:          %.2f s\n", st.fast_time_s);

    /* ---- 2. is the fast path non-trivial? --------------------------- */

    T("A fast path was produced");
    if (*fast) OK(); else NO("empty — the target was never found?");

    /* ---- 3. replay home on the true maze ---------------------------- */

    T("Home path replays to the START node on the true maze");
    {
        int node = cur, head = heading, steps = 0;
        int rc = replay(home, &node, &head, MAZE_DATA_START_IDX, &steps, 1);
        if (rc == 0) OK();
        else { printf("\n"); NO(rc == -2 ? "asks for a link that does not exist"
                                         : "does not end on the start node"); }
        /* the fast path continues from wherever the home run left the robot */
        if (rc == 0) {
            T("Fast path replays to the TARGET node on the true maze");
            int steps2 = 0;
            int rc2 = replay(fast, &node, &head, MAZE_DATA_TARGET_IDX, &steps2,
                             *fast == '\0');
            if (rc2 == 0) OK();
            else { printf("\n"); NO(rc2 == -2 ? "asks for a link that does not exist"
                                              : "does not end on the target node"); }
        }
    }

    /* ---- 4. is it actually optimal? --------------------------------- */

    T("Fast-path time equals the time-optimal cost of the FULL maze");
    {
        float truth = true_optimal_time_s();
        printf("\n      brain %.2f s   full-maze optimum %.2f s   ",
               st.fast_time_s, truth);
        if (truth < 0.0f) { NO("no path exists in the true maze"); }
        else if (st.fast_time_s <= truth + 0.005f) OK();
        else NO("brain's route is SLOWER than the optimum — the early-stop "
                "proof let it stop too early");
    }

    /* ---- 5. can the FIRMWARE execute these strings? -----------------
     *
     * The brain's alphabet and the firmware's replay alphabet are not the
     * same thing, and nothing above this line would notice the difference:
     * the host replays understand every command the brain emits, so a plan
     * the robot cannot follow still "passes" checks 3 and 4.
     *
     * That is not hypothetical.  The legacy replay stages dispatched only
     * 'L'/'R'/'S'/'D' -- no 'F' and, fatally, no 'B' -- while the brain emits
     * 'F' and emits 'B' on any reversal (the real field's fast path is
     * BFFLRLF).  A 'B' would have fallen through every branch and the robot
     * would have sat at the junction until someone noticed.  The stages now
     * have all four cases (main.c replay_dispatch()), and this check is what
     * pins that down: it holds the brain to exactly the four letters the
     * firmware implements. */

    T("Both plans use only F/L/R/B (the firmware's replay alphabet)");
    {
        const char* plans[2];
        const char* names[2];
        int         bad = 0;
        int         i, k;

        plans[0] = home; names[0] = "home";
        plans[1] = fast; names[1] = "fast";

        for (k = 0; k < 2; k++) {
            for (i = 0; plans[k][i]; i++) {
                char c = plans[k][i];
                if (c != 'F' && c != 'L' && c != 'R' && c != 'B') {
                    printf("\n      %s path has '%c' (0x%02X) at index %d",
                           names[k], c, (unsigned char)c, i);
                    bad = 1;
                }
            }
        }
        if (!bad) OK(); else NO("a command the firmware cannot replay");
    }

    /* The 'D' terminator set_plan() appends needs one spare byte, and a plan
     * longer than the firmware's replay buffer would be silently truncated. */
    T("Both plans fit the firmware's replay buffer");
    {
        size_t h = strlen(home), f = strlen(fast);
        printf("\n      home %zu + sentinel, fast %zu + sentinel, buffer 200   ",
               h, f);
        if (h + 1 < 200 && f + 1 < 200) OK();
        else NO("a plan would be truncated by set_plan()");
    }

    printf("\n%d/%d checks passed\n", tests_run - tests_failed, tests_run);
    return tests_failed ? 1 : 0;
}

