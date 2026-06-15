/**
 * @file test_robot.c
 * @brief Unit tests for the robot module.
 *
 * Compile (from robot codes/):
 *   gcc -std=c11 -Wall -Wextra -pedantic -I inc src/maze_graph.c src/maze_robot.c test/test_robot.c -o build/test_robot.exe
 */

#include "maze_robot.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

static int tests_run = 0, tests_failed = 0;
#define TEST(n)  do { tests_run++; printf("  %-48s ", n); } while(0)
#define PASS()   do { printf("PASS\n"); } while(0)
#define FAIL(m)  do { printf("FAIL: %s\n", m); tests_failed++; } while(0)

/* ==========================================================================
 * ASetup — 4-node star graph
 *
 *      1 (0,20)  NORTH
 *       |
 *   3--0--2      WEST--START--EAST
 *       |
 *      4 (0,-20) SOUTH
 *
 * Node 0 = start (0,0).  All edges explored so we can test path-following.
 * ========================================================================== */

static MazeGraph g;
static MazeRobot r;

static void setup_star(void) {
    maze_graph_init(&g, (MazeCoord){0, 0});          /* node 0 */
    maze_graph_add_node(&g, (MazeCoord){0, 20});     /* node 1 (N) */
    maze_graph_add_node(&g, (MazeCoord){20, 0});     /* node 2 (E) */
    maze_graph_add_node(&g, (MazeCoord){-20, 0});    /* node 3 (W) */
    maze_graph_add_node(&g, (MazeCoord){0, -20});    /* node 4 (S) */
    maze_graph_add_edge(&g, 0, 1); maze_graph_mark_explored(&g, 0, 1);
    maze_graph_add_edge(&g, 0, 2); maze_graph_mark_explored(&g, 0, 2);
    maze_graph_add_edge(&g, 0, 3); maze_graph_mark_explored(&g, 0, 3);
    maze_graph_add_edge(&g, 0, 4); maze_graph_mark_explored(&g, 0, 4);
    maze_robot_init(&r, &g);
}

/* ==========================================================================
 * TESTS
 * ========================================================================== */

/* ---- Heading helpers ---- */
static void test_heading_opposite(void) {
    TEST("Heading opposite");
    if (maze_heading_opposite(MAZE_NORTH) != MAZE_SOUTH) { FAIL("N→S"); return; }
    if (maze_heading_opposite(MAZE_EAST)  != MAZE_WEST)  { FAIL("E→W"); return; }
    if (maze_heading_opposite(MAZE_SOUTH) != MAZE_NORTH) { FAIL("S→N"); return; }
    if (maze_heading_opposite(MAZE_WEST)  != MAZE_EAST)  { FAIL("W→E"); return; }
    PASS();
}

static void test_heading_cross(void) {
    TEST("Heading cross product");
    /* NORTH→WEST: left (+1) */
    if (maze_heading_cross(MAZE_NORTH, MAZE_WEST)  <= 0) { FAIL("N→W should be left (+)"); return; }
    /* NORTH→EAST: right (-1) */
    if (maze_heading_cross(MAZE_NORTH, MAZE_EAST) >= 0)  { FAIL("N→E should be right (-)"); return; }
    /* EAST→NORTH: left (+1) */
    if (maze_heading_cross(MAZE_EAST, MAZE_NORTH) <= 0)  { FAIL("E→N should be left (+)"); return; }
    /* EAST→SOUTH: right (-1) */
    if (maze_heading_cross(MAZE_EAST, MAZE_SOUTH) >= 0)  { FAIL("E→S should be right (-)"); return; }
    /* Same direction: 0 */
    if (maze_heading_cross(MAZE_NORTH, MAZE_NORTH) != 0) { FAIL("same should be 0"); return; }
    PASS();
}

/* ---- Command generation ---- */
static void test_commands(void) {
    TEST("Command generation (F/L/R/B)");
    setup_star();

    MazeHeading h = MAZE_HEADING_NONE;

    /* First move: 0→1 (NORTH) → should be F */
    MazeCommand cmd = maze_robot_compute_command(&h, (MazeCoord){0,0}, (MazeCoord){0,20}, true);
    if (cmd != MAZE_CMD_FORWARD) { FAIL("first move should be F"); return; }
    if (h != MAZE_NORTH) { FAIL("heading should be NORTH"); return; }

    /* Straight: 0→1 again (already heading N) */
    cmd = maze_robot_compute_command(&h, (MazeCoord){0,0}, (MazeCoord){0,20}, false);
    if (cmd != MAZE_CMD_FORWARD) { FAIL("straight should be F"); return; }

    /* Left: heading N, dest W (node 3) */
    cmd = maze_robot_compute_command(&h, (MazeCoord){0,0}, (MazeCoord){-20,0}, false);
    if (cmd != MAZE_CMD_LEFT) { printf("(got %c) ", (char)cmd); FAIL("N→W should be L"); return; }
    if (h != MAZE_WEST) { FAIL("heading should now be WEST"); return; }

    /* Right: heading W, dest N (node 1) */
    cmd = maze_robot_compute_command(&h, (MazeCoord){0,0}, (MazeCoord){0,20}, false);
    if (cmd != MAZE_CMD_RIGHT) { printf("(got %c) ", (char)cmd); FAIL("W→N should be R"); return; }
    if (h != MAZE_NORTH) { FAIL("heading should now be NORTH"); return; }

    /* U-turn: heading N, dest S (node 4) */
    cmd = maze_robot_compute_command(&h, (MazeCoord){0,0}, (MazeCoord){0,-20}, false);
    if (cmd != MAZE_CMD_BACK) { printf("(got %c) ", (char)cmd); FAIL("N→S should be B"); return; }
    if (h != MAZE_SOUTH) { FAIL("heading should now be SOUTH"); return; }

    PASS();
}

/* ---- Move tracking ---- */
static void test_move_to(void) {
    TEST("Move-to tracking");
    setup_star();

    /* Initial state */
    if (r.current_node != 0) { FAIL("should start at node 0"); return; }
    if (r.command_count != 0) { FAIL("no commands yet"); return; }

    /* Move 0→1 (first move) */
    int rc = maze_robot_move_to(&r, 1, true);
    if (rc != MAZE_OK) { FAIL("move_to failed"); return; }
    if (r.current_node != 1) { FAIL("should be at node 1"); return; }
    if (r.heading != MAZE_NORTH) { FAIL("heading should be NORTH"); return; }
    if (r.command_count != 1) { FAIL("should have 1 command"); return; }
    if (r.command_log[0] != 'F') { printf("(got %c) ", r.command_log[0]); FAIL("first cmd should be F"); return; }
    if (r.steps_taken != 1) { FAIL("step count"); return; }

    /* Edge 0-1 should be explored */
    if (!maze_graph_is_explored(&g, 0, 1)) { FAIL("edge should be explored"); return; }

    PASS();
}

/* ---- Frontier detection ---- */
static void test_frontiers(void) {
    TEST("Frontier detection");
    MazeGraph g2;
    maze_graph_init(&g2, (MazeCoord){0, 0});
    maze_graph_add_node(&g2, (MazeCoord){0, 20});
    maze_graph_add_node(&g2, (MazeCoord){20, 0});
    maze_graph_add_edge(&g2, 0, 1);
    maze_graph_add_edge(&g2, 0, 2);
    /* Explore only 0-1, leave 0-2 unexplored */
    maze_graph_mark_explored(&g2, 0, 1);

    MazeRobot r2;
    maze_robot_init(&r2, &g2);
    r2.visited_nodes[1] = true;  /* node 1 visited (driven there) */
    r2.current_node = 1;

    /* Node 0 is a frontier (visited, has unexplored edge 0-2) */
    uint16_t frontiers[10];
    uint8_t fc = maze_robot_find_frontiers(&r2, frontiers, 10);
    if (fc != 1) { printf("(got %u) ", fc); FAIL("should be 1 frontier"); return; }
    if (frontiers[0] != 0) { FAIL("frontier should be node 0"); return; }

    /* Node 1 has no unexplored edges → not a frontier */
    if (maze_robot_has_unexplored(&r2)) { FAIL("node 1 should not have unexplored"); return; }

    PASS();
}

/* ---- Branch ranking ---- */
static void test_branch_ranking(void) {
    TEST("Branch ranking (turn-minimising)");
    setup_star();
    r.heading = MAZE_NORTH;

    /* Unexplored candidates: node 2 (E), node 3 (W), node 4 (S).
     * From NORTH: straight → *no straight candidate*, nearest=F then L→W, R→E, B→S
     * Priority: straight(not here) > L(W) > R(E) > B(S)
     * Compass tie-break: N < E < S < W → but we only have 3 candidates here
     * Actually both W and E are 90° turns; L(W) has cost 1, R(E) has cost 2, B(S) has cost 3
     */
    uint16_t cand[3] = {2, 3, 4};  /* E, W, S — note all edges already explored, but ranking doesn't care */
    uint16_t ranked[3];
    maze_robot_rank_branches(&r, cand, 3, ranked);

    /* Expected order: W(3) first (L), E(2) second (R), S(4) last (B) */
    if (ranked[0] != 3) { printf("(got %u) ", ranked[0]); FAIL("W should be first (L)"); return; }
    if (ranked[1] != 2) { printf("(got %u) ", ranked[1]); FAIL("E should be second (R)"); return; }
    if (ranked[2] != 4) { printf("(got %u) ", ranked[2]); FAIL("S should be last (B)"); return; }

    PASS();
}

/* ---- Path following ---- */
static void test_path_follow(void) {
    TEST("Path following");
    setup_star();

    uint16_t p[] = {0, 1};  /* start → north */
    maze_robot_set_path(&r, p, 2);

    if (!maze_robot_has_path(&r)) { FAIL("should have a path"); return; }

    MazeCommand cmd = maze_robot_step_path(&r);
    if (cmd != MAZE_CMD_FORWARD) { FAIL("should execute F"); return; }
    if (r.current_node != 1) { FAIL("should be at node 1"); return; }
    if (maze_robot_has_path(&r)) { FAIL("path should be exhausted"); return; }

    PASS();
}

/* ==========================================================================
 * MAIN
 * ========================================================================== */
int main(void) {
    printf("=== maze_robot tests ===\n");

    test_heading_opposite();
    test_heading_cross();
    test_commands();
    test_move_to();
    test_frontiers();
    test_branch_ranking();
    test_path_follow();

    printf("\n%d tests, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
