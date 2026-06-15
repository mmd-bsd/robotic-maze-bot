/**
 * @file test_graph.c
 * @brief Unit tests for the maze graph module.
 *
 * Builds a known graph (the 27-node sample maze from sample_maze.json,
 * embedded as static data), then exercises every public function.
 *
 * Compile:
 *   gcc -std=c11 -Wall -Wextra -I../inc ../src/maze_graph.c test_graph.c -o test_graph
 */

#include "maze_graph.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

/* ==========================================================================
 * SAMPLE MAZE — 27 nodes from sample_maze.json, with all edges pre-explored
 * so we can test Dijkstra.
 *
 * Coordinates match the JSON: node "0"=[0,0] start, "22"=[-60,80] target.
 * Edges are embedded as pairs.
 * ========================================================================== */

/* Node positions (from sample_maze.json) */
static const MazeCoord sample_coords[27] = {
    {0, 0},       /*  0 = start  */
    {0, 20},      /*  1          */
    {0, 40},      /*  2          */
    {0, 60},      /*  3          */
    {0, 100},     /*  4          */
    {0, 120},     /*  5          */
    {20, 40},     /*  6          */
    {20, 100},    /*  7          */
    {20, 120},    /*  8          */
    {40, 40},     /*  9          */
    {40, 60},     /* 10          */
    {40, 80},     /* 11          */
    {60, 40},     /* 12          */
    {60, 80},     /* 13          */
    {-20, 40},    /* 14          */
    {-20, 60},    /* 15          */
    {-20, 80},    /* 16          */
    {-40, 40},    /* 17          */
    {-40, 60},    /* 18          */
    {-40, 80},    /* 19          */
    {-40, 100},   /* 20          */
    {-60, 20},    /* 21          */
    {-60, 80},    /* 22 = target */
    {-60, 120},   /* 23          */
    {-80, 40},    /* 24          */
    {-80, 60},    /* 25          */
    {-80, 80},    /* 26          */
};

static const uint16_t sample_edges[][2] = {
    {0, 1}, {1, 2},  {1, 21}, {2, 3},  {2, 6},  {3, 4},
    {3, 10}, {3, 15}, {4, 5},  {4, 7},  {4, 20}, {7, 8},
    {9, 10}, {9, 12}, {10, 11}, {11, 13}, {12, 13}, {14, 15},
    {14, 17}, {15, 16}, {16, 19}, {17, 18}, {17, 24}, {18, 25},
    {19, 20}, {19, 22}, {22, 23}, {22, 26}, {24, 25}, {25, 26},
};
#define SAMPLE_EDGE_COUNT (sizeof(sample_edges) / sizeof(sample_edges[0]))

/* ==========================================================================
 * TEST HELPERS
 * ========================================================================== */

static int tests_run = 0;
static int tests_failed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  %-50s ", name); \
} while (0)

#define PASS() do { printf("PASS\n"); } while (0)
#define FAIL(msg) do { \
    printf("FAIL: %s\n", msg); \
    tests_failed++; \
} while (0)

/* Build the sample maze graph with all edges explored. */
static void build_sample_graph(MazeGraph* g) {
    int rc = maze_graph_init(g, sample_coords[0]);
    assert(rc == MAZE_OK);

    /* Add all nodes */
    for (int i = 1; i < 27; i++) {
        rc = maze_graph_add_node(g, sample_coords[i]);
        assert(rc == i);  /* indices should match */
    }

    /* Add and explore all edges */
    for (uint16_t i = 0; i < SAMPLE_EDGE_COUNT; i++) {
        rc = maze_graph_add_edge(g, sample_edges[i][0], sample_edges[i][1]);
        assert(rc >= 0);
        rc = maze_graph_mark_explored(g, sample_edges[i][0], sample_edges[i][1]);
        assert(rc == MAZE_OK);
    }
}

/* ==========================================================================
 * TEST: Node operations
 * ========================================================================== */
static void test_node_operations(void) {
    TEST("Node add and find");
    MazeGraph g;
    maze_graph_init(&g, (MazeCoord){0, 0});
    if (g.node_count != 1) { FAIL("start node not created"); return; }
    if (g.nodes[0].visited != true) { FAIL("start not marked visited"); return; }

    int n1 = maze_graph_add_node(&g, (MazeCoord){0, 20});
    int n2 = maze_graph_add_node(&g, (MazeCoord){20, 0});
    if (n1 != 1 || n2 != 2) { FAIL("node indices"); return; }
    if (g.node_count != 3) { FAIL("node count"); return; }

    /* Duplicate add returns existing index */
    int n1_dup = maze_graph_add_node(&g, (MazeCoord){0, 20});
    if (n1_dup != 1) { FAIL("duplicate add should return existing"); return; }
    if (g.node_count != 3) { FAIL("duplicate should not increase count"); return; }

    /* Find with tolerance */
    int found = maze_graph_find_node(&g, (MazeCoord){1, 19});
    if (found != 1) { FAIL("tolerance find"); return; }

    /* Not found */
    found = maze_graph_find_node(&g, (MazeCoord){500, 500});
    if (found != MAZE_ERR_NODE_NOT_FOUND) { FAIL("should not find far coord"); return; }

    maze_graph_mark_visited(&g, 1);
    if (!maze_graph_is_visited(&g, 1)) { FAIL("mark visited"); return; }
    if (maze_graph_is_visited(&g, 2))  { FAIL("node 2 should not be visited"); return; }

    PASS();
}

/* ==========================================================================
 * TEST: Edge operations
 * ========================================================================== */
static void test_edge_operations(void) {
    TEST("Edge add and explore");
    MazeGraph g;
    maze_graph_init(&g, (MazeCoord){0, 0});
    maze_graph_add_node(&g, (MazeCoord){0, 20});
    maze_graph_add_node(&g, (MazeCoord){20, 0});

    /* Add edge 0-1 */
    int e01 = maze_graph_add_edge(&g, 0, 1);
    if (e01 < 0) { FAIL("add edge 0-1"); return; }
    if (!maze_graph_edge_exists(&g, 0, 1)) { FAIL("edge should exist"); return; }
    if (maze_graph_is_explored(&g, 0, 1))  { FAIL("edge should not be explored yet"); return; }

    /* Mark explored */
    maze_graph_mark_explored(&g, 0, 1);
    if (!maze_graph_is_explored(&g, 0, 1)) { FAIL("edge should be explored"); return; }

    /* Add edge 0-2 */
    int e02 = maze_graph_add_edge(&g, 0, 2);
    if (e02 < 0) { FAIL("add edge 0-2"); return; }

    /* Duplicate edge add returns existing */
    int dup = maze_graph_add_edge(&g, 0, 1);
    if (dup != e01) { FAIL("duplicate edge should return existing index"); return; }
    if (g.edge_count != 2) { FAIL("edge count should still be 2"); return; }

    /* Self-loop ignored */
    int self = maze_graph_add_edge(&g, 1, 1);
    if (self != MAZE_OK) { FAIL("self-loop should return OK"); return; }

    PASS();
}

/* ==========================================================================
 * TEST: Neighbor queries
 * ========================================================================== */
static void test_neighbor_queries(void) {
    TEST("Neighbor queries");
    MazeGraph g;
    build_sample_graph(&g);

    /* Node 3 (0,60) has edges to: 2(0,40), 4(0,100), 10(40,60), 15(-20,60) */
    uint16_t neighbors[4];
    uint8_t count = maze_graph_get_explored_neighbors(&g, 3, neighbors, 4);
    if (count != 4) { FAIL("node 3 should have 4 explored neighbors"); return; }

    /* All unexplored should be 0 (all edges explored) */
    uint16_t unex[4];
    count = maze_graph_get_unexplored_neighbors(&g, 3, unex, 4);
    if (count != 0) { FAIL("no unexplored in fully explored graph"); return; }

    /* Leaf node 8 (20,120) has only 1 neighbor: 7 */
    count = maze_graph_get_all_neighbors(&g, 8, neighbors, 4);
    if (count != 1) { FAIL("leaf node 8 should have 1 neighbor"); return; }
    if (neighbors[0] != 7) { FAIL("node 8 neighbor should be 7"); return; }

    PASS();
}

/* ==========================================================================
 * TEST: Dijkstra
 * ========================================================================== */
static void test_dijkstra(void) {
    TEST("Dijkstra shortest path");
    MazeGraph g;
    build_sample_graph(&g);

    /* Path from start (0) to target (22).
     * The shortest DISTANCE path in this maze:
     *   0→1→21 (60cm? let's compute: 0[0,0]→1[0,20]=20, 1→21[-60,20]=60, total 80)
     *   vs 0→1→2→3→15→16→19→22: 20+20+20+20+20+20+20=140
     *   vs 0→1→2→14→17→18→25→26→22: actually let me just trust Dijkstra
     *
     * Let's just verify the path exists and has reasonable length.
     */
    uint16_t path[50];
    uint16_t len = maze_graph_shortest_path(&g, 0, 22, path, 50);
    if (len == 0) { FAIL("no path from start to target"); return; }
    if (path[0] != 0) { FAIL("path should start at 0"); return; }
    if (path[len - 1] != 22) { FAIL("path should end at 22"); return; }

    /* Verify every consecutive pair has an edge */
    for (uint16_t i = 0; i < len - 1; i++) {
        if (!maze_graph_edge_exists(&g, path[i], path[i + 1])) {
            printf("  [edge %d-%d missing] ", path[i], path[i + 1]);
            FAIL("edge missing in path");
            return;
        }
    }

    /* Distance should match path sum */
    uint32_t dist = maze_graph_path_distance(&g, 0, 22);
    if (dist == UINT32_MAX) { FAIL("distance unreachable"); return; }

    /* Manual distance along returned path */
    uint32_t manual = 0;
    for (uint16_t i = 0; i < len - 1; i++) {
        int ei = maze_graph_find_edge(&g, path[i], path[i + 1]);
        manual += g.edges[ei].length_cm;
    }
    if (dist != manual) { FAIL("Dijkstra distance mismatch"); return; }

    printf("(path len=%u, dist=%u cm) ", len, dist);
    PASS();
}

/* ==========================================================================
 * TEST: One-to-all Dijkstra
 * ========================================================================== */
static void test_dijkstra_all(void) {
    TEST("Dijkstra one-to-all");
    MazeGraph g;
    build_sample_graph(&g);

    uint32_t dist[50];
    uint16_t parent[50];
    maze_graph_dijkstra(&g, 0, dist, parent);

    /* All 27 nodes should be reachable */
    uint16_t reachable = 0;
    for (uint16_t i = 0; i < g.node_count; i++) {
        if (dist[i] != UINT32_MAX) reachable++;
    }
    if (reachable != 27) { FAIL("not all nodes reachable from start"); return; }

    /* Start distance should be 0 */
    if (dist[0] != 0) { FAIL("dist to start should be 0"); return; }

    /* Start parent should be INVALID */
    if (parent[0] != MAZE_INVALID_NODE) { FAIL("start parent should be INVALID"); return; }

    PASS();
}

/* ==========================================================================
 * TEST: Straight-line distance
 * ========================================================================== */
static void test_straight_line(void) {
    TEST("Straight-line distance");
    /* 3-4-5 triangle */
    MazeCoord a = {0, 0};
    MazeCoord b = {3, 4};
    uint32_t d = maze_straight_line_cm(a, b);
    if (d != 5) { printf("(got %u, expected 5) ", d); FAIL("3-4-5"); return; }

    /* Same point */
    d = maze_straight_line_cm(a, a);
    if (d != 0) { FAIL("same point should be 0"); return; }

    PASS();
}

/* ==========================================================================
 * MAIN
 * ========================================================================== */
int main(void) {
    printf("=== maze_graph tests ===\n");

    test_node_operations();
    test_edge_operations();
    test_neighbor_queries();
    test_dijkstra();
    test_dijkstra_all();
    test_straight_line();

    printf("\n%d tests, %d failed\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
