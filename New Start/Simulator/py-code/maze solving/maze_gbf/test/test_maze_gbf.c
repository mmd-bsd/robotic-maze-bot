/**
 * @file test_maze_gbf.c
 * @brief Unit tests for maze navigation system
 *
 * This file contains unit tests for validating the core functionality
 * of the maze navigation system including graph operations and
 * the main navigation algorithm.
 */

#include "../inc/maze_gbf.h"
#include "../inc/maze_graph.h"
#include "../inc/maze_robot.h"
#include "../inc/maze_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

/*============================================================================
 * TEST UTILITIES
 *============================================================================*/

#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            printf("FAIL: %s\n", message); \
            printf("  at %s:%d\n", __FILE__, __LINE__); \
            return false; \
        } \
    } while(0)

#define TEST_START(name) \
    printf("Testing: %s...", name); \
    bool test_passed = true

#define TEST_END() \
    if (test_passed) { \
        printf(" PASS\n"); \
        return true; \
    } else { \
        return false; \
    }

/*============================================================================
 * GRAPH TESTS
 *============================================================================*/

bool test_graph_init() {
    TEST_START("Graph initialization");

    MazeGraph graph;
    int result = maze_graph_init(&graph, 10, 20);

    TEST_ASSERT(result == MAZE_SUCCESS, "Failed to initialize graph");
    TEST_ASSERT(graph.node_count == 0, "Node count should be 0");
    TEST_ASSERT(graph.edge_count == 0, "Edge count should be 0");
    TEST_ASSERT(graph.max_nodes == 10, "Max nodes not set correctly");
    TEST_ASSERT(graph.max_edges == 20, "Max edges not set correctly");
    TEST_ASSERT(graph.nodes != NULL, "Nodes array not allocated");
    TEST_ASSERT(graph.edges != NULL, "Edges array not allocated");
    TEST_ASSERT(graph.adj_list != NULL, "Adjacency list not allocated");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_add_node() {
    TEST_START("Graph add node");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    // Add first node
    MazeCoord coord1 = {0, 0};
    int node_id1 = maze_graph_add_node(&graph, coord1);

    TEST_ASSERT(node_id1 == 0, "First node ID should be 0");
    TEST_ASSERT(graph.node_count == 1, "Node count should be 1");
    TEST_ASSERT(graph.nodes[0].coord.x == 0, "Node X coordinate incorrect");
    TEST_ASSERT(graph.nodes[0].coord.y == 0, "Node Y coordinate incorrect");
    TEST_ASSERT(graph.nodes[0].visited == false, "Node should not be visited initially");

    // Add second node
    MazeCoord coord2 = {20, 0};
    int node_id2 = maze_graph_add_node(&graph, coord2);

    TEST_ASSERT(node_id2 == 1, "Second node ID should be 1");
    TEST_ASSERT(graph.node_count == 2, "Node count should be 2");

    // Add third node
    MazeCoord coord3 = {40, 0};
    int node_id3 = maze_graph_add_node(&graph, coord3);

    TEST_ASSERT(node_id3 == 2, "Third node ID should be 2");
    TEST_ASSERT(graph.node_count == 3, "Node count should be 3");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_find_by_coord() {
    TEST_START("Graph find node by coordinate");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    // Add nodes
    maze_graph_add_node(&graph, (MazeCoord){0, 0});
    maze_graph_add_node(&graph, (MazeCoord){20, 0});
    maze_graph_add_node(&graph, (MazeCoord){40, 0});

    // Find existing node
    int found_id = maze_graph_find_node_by_coord(&graph, (MazeCoord){20, 0});
    TEST_ASSERT(found_id == 1, "Should find node at (20, 0)");

    // Find with tolerance
    found_id = maze_graph_find_node_by_coord(&graph, (MazeCoord){22, 1});
    TEST_ASSERT(found_id == 1, "Should find node with tolerance");

    // Not found
    found_id = maze_graph_find_node_by_coord(&graph, (MazeCoord){100, 100});
    TEST_ASSERT(found_id == -1, "Should not find non-existent node");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_add_edge() {
    TEST_START("Graph add edge");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    // Add nodes
    maze_graph_add_node(&graph, (MazeCoord){0, 0});     // Node 0
    maze_graph_add_node(&graph, (MazeCoord){20, 0});    // Node 1
    maze_graph_add_node(&graph, (MazeCoord){40, 0});    // Node 2

    // Add edges
    int result = maze_graph_add_edge(&graph, 0, 1);
    TEST_ASSERT(result == MAZE_SUCCESS, "Failed to add edge 0-1");
    TEST_ASSERT(graph.edge_count == 1, "Edge count should be 1");
    TEST_ASSERT(graph.edges[0].node_a == 0, "Edge node_a incorrect");
    TEST_ASSERT(graph.edges[0].node_b == 1, "Edge node_b incorrect");
    TEST_ASSERT(graph.edges[0].explored == false, "Edge should not be explored initially");

    result = maze_graph_add_edge(&graph, 1, 2);
    TEST_ASSERT(result == MAZE_SUCCESS, "Failed to add edge 1-2");
    TEST_ASSERT(graph.edge_count == 2, "Edge count should be 2");

    // Add duplicate edge (should succeed, no duplicate added)
    result = maze_graph_add_edge(&graph, 0, 1);
    TEST_ASSERT(result == MAZE_SUCCESS, "Duplicate edge should return success");
    TEST_ASSERT(graph.edge_count == 2, "Edge count should still be 2");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_get_neighbors() {
    TEST_START("Graph get neighbors");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    // Create linear path: 0-1-2
    maze_graph_add_node(&graph, (MazeCoord){0, 0});
    maze_graph_add_node(&graph, (MazeCoord){20, 0});
    maze_graph_add_node(&graph, (MazeCoord){40, 0});
    maze_graph_add_edge(&graph, 0, 1);
    maze_graph_add_edge(&graph, 1, 2);

    // Test node 0 (should have 1 neighbor)
    uint16_t neighbors[4];
    uint16_t count = maze_graph_get_neighbors(&graph, 0, neighbors, 4);
    TEST_ASSERT(count == 1, "Node 0 should have 1 neighbor");
    TEST_ASSERT(neighbors[0] == 1, "Node 0's neighbor should be 1");

    // Test node 1 (should have 2 neighbors)
    count = maze_graph_get_neighbors(&graph, 1, neighbors, 4);
    TEST_ASSERT(count == 2, "Node 1 should have 2 neighbors");
    TEST_ASSERT(neighbors[0] == 0 || neighbors[1] == 0, "Node 1 should neighbor 0");
    TEST_ASSERT(neighbors[0] == 2 || neighbors[1] == 2, "Node 1 should neighbor 2");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_shortest_path() {
    TEST_START("Graph shortest path (BFS)");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    // Create path: 0-1-2-3
    maze_graph_add_node(&graph, (MazeCoord){0, 0});
    maze_graph_add_node(&graph, (MazeCoord){20, 0});
    maze_graph_add_node(&graph, (MazeCoord){40, 0});
    maze_graph_add_node(&graph, (MazeCoord){60, 0});
    maze_graph_add_edge(&graph, 0, 1);
    maze_graph_add_edge(&graph, 1, 2);
    maze_graph_add_edge(&graph, 2, 3);

    // Test shortest path
    uint16_t path[10];
    uint16_t path_len = maze_graph_shortest_path(&graph, 0, 3, path, 10);

    TEST_ASSERT(path_len == 4, "Path length should be 4");
    TEST_ASSERT(path[0] == 0, "Path should start at node 0");
    TEST_ASSERT(path[1] == 1, "Path should go through node 1");
    TEST_ASSERT(path[2] == 2, "Path should go through node 2");
    TEST_ASSERT(path[3] == 3, "Path should end at node 3");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_mark_edge_explored() {
    TEST_START("Graph mark edge explored");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    maze_graph_add_node(&graph, (MazeCoord){0, 0});
    maze_graph_add_node(&graph, (MazeCoord){20, 0});
    maze_graph_add_edge(&graph, 0, 1);

    // Edge should not be explored initially
    TEST_ASSERT(!maze_graph_is_edge_explored(&graph, 0, 1), "Edge should not be explored");

    // Mark as explored
    int result = maze_graph_mark_edge_explored(&graph, 0, 1);
    TEST_ASSERT(result == MAZE_SUCCESS, "Failed to mark edge explored");
    TEST_ASSERT(maze_graph_is_edge_explored(&graph, 0, 1), "Edge should be explored");

    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_graph_get_unexplored_neighbors() {
    TEST_START("Graph get unexplored neighbors");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    // Create: 0-1-2
    maze_graph_add_node(&graph, (MazeCoord){0, 0});
    maze_graph_add_node(&graph, (MazeCoord){20, 0});
    maze_graph_add_node(&graph, (MazeCoord){40, 0});
    maze_graph_add_edge(&graph, 0, 1);
    maze_graph_add_edge(&graph, 1, 2);

    // Mark edge 0-1 as explored
    maze_graph_mark_edge_explored(&graph, 0, 1);

    // Node 0 should have no unexplored neighbors
    uint16_t unexplored[4];
    uint16_t count = maze_graph_get_unexplored_neighbors(&graph, 0, unexplored, 4);
    TEST_ASSERT(count == 0, "Node 0 should have 0 unexplored neighbors");

    // Node 1 should have 1 unexplored neighbor (node 2)
    count = maze_graph_get_unexplored_neighbors(&graph, 1, unexplored, 4);
    TEST_ASSERT(count == 1, "Node 1 should have 1 unexplored neighbor");
    TEST_ASSERT(unexplored[0] == 2, "Node 1's unexplored neighbor should be 2");

    maze_graph_cleanup(&graph);

    TEST_END();
}

/*============================================================================
 * ROBOT TESTS
 *============================================================================*/

bool test_robot_init() {
    TEST_START("Robot initialization");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    maze_graph_add_node(&graph, (MazeCoord){0, 0});

    MazeRobot robot;
    int result = maze_robot_init(&robot, &graph, 0, 0);

    TEST_ASSERT(result == MAZE_SUCCESS, "Failed to initialize robot");
    TEST_ASSERT(robot.current_node_id == 0, "Robot should start at node 0");
    TEST_ASSERT(robot.start_node_id == 0, "Start node should be 0");
    TEST_ASSERT(robot.exploration_complete == false, "Exploration should not be complete");
    TEST_ASSERT(robot.target_found == false, "Target should not be found");

    maze_robot_cleanup(&robot);
    maze_graph_cleanup(&graph);

    TEST_END();
}

bool test_robot_update_position() {
    TEST_START("Robot update position");

    MazeGraph graph;
    maze_graph_init(&graph, 10, 20);

    maze_graph_add_node(&graph, (MazeCoord){0, 0});     // Node 0
    maze_graph_add_node(&graph, (MazeCoord){20, 0});    // Node 1
    maze_graph_add_edge(&graph, 0, 1);

    MazeRobot robot;
    maze_robot_init(&robot, &graph, 0, 0);

    // Update position to node 1
    int result = maze_robot_update_position(&robot, 1);

    TEST_ASSERT(result == MAZE_SUCCESS, "Failed to update position");
    TEST_ASSERT(robot.current_node_id == 1, "Robot should be at node 1");
    TEST_ASSERT(maze_robot_is_node_visited(&robot, 1), "Node 1 should be marked visited");

    maze_robot_cleanup(&robot);
    maze_graph_cleanup(&graph);

    TEST_END();
}

/*============================================================================
 * UTILS TESTS
 *============================================================================*/

bool test_utils_coords_equal() {
    TEST_START("Utils coords equal");

    MazeCoord coord1 = {100, 200};
    MazeCoord coord2 = {100, 200};
    MazeCoord coord3 = {102, 201};  // Within tolerance
    MazeCoord coord4 = {110, 200};  // Outside tolerance

    TEST_ASSERT(maze_utils_coords_equal(coord1, coord2), "Identical coords should be equal");
    TEST_ASSERT(maze_utils_coords_equal(coord1, coord3), "Coords within tolerance should be equal");
    TEST_ASSERT(!maze_utils_coords_equal(coord1, coord4), "Coords outside tolerance should not be equal");

    TEST_END();
}

bool test_utils_manhattan_distance() {
    TEST_START("Utils Manhattan distance");

    MazeCoord coord1 = {0, 0};
    MazeCoord coord2 = {30, 40};

    uint16_t dist = maze_utils_manhattan_distance(coord1, coord2);
    TEST_ASSERT(dist == 70, "Manhattan distance should be 70");

    TEST_END();
}

bool test_utils_direction_to_string() {
    TEST_START("Utils direction to string");

    TEST_ASSERT(strcmp(maze_utils_direction_to_string(MAZE_DIR_STOP), "STOP") == 0, "STOP string incorrect");
    TEST_ASSERT(strcmp(maze_utils_direction_to_string(MAZE_DIR_FORWARD), "FORWARD") == 0, "FORWARD string incorrect");
    TEST_ASSERT(strcmp(maze_utils_direction_to_string(MAZE_DIR_LEFT), "LEFT") == 0, "LEFT string incorrect");
    TEST_ASSERT(strcmp(maze_utils_direction_to_string(MAZE_DIR_RIGHT), "RIGHT") == 0, "RIGHT string incorrect");
    TEST_ASSERT(strcmp(maze_utils_direction_to_string(MAZE_DIR_BACK), "BACK") == 0, "BACK string incorrect");

    TEST_END();
}

/*============================================================================
 * MAIN TEST RUNNER
 *============================================================================*/

typedef bool (*TestFunc)(void);

typedef struct {
    const char* name;
    TestFunc func;
} Test;

int main() {
    printf("=================================================\n");
    printf("Maze GBF Unit Tests\n");
    printf("=================================================\n\n");

    // Define all tests
    Test tests[] = {
        // Graph tests
        {"Graph initialization", test_graph_init},
        {"Graph add node", test_graph_add_node},
        {"Graph find by coord", test_graph_find_by_coord},
        {"Graph add edge", test_graph_add_edge},
        {"Graph get neighbors", test_graph_get_neighbors},
        {"Graph shortest path", test_graph_shortest_path},
        {"Graph mark edge explored", test_graph_mark_edge_explored},
        {"Graph get unexplored neighbors", test_graph_get_unexplored_neighbors},

        // Robot tests
        {"Robot initialization", test_robot_init},
        {"Robot update position", test_robot_update_position},

        // Utils tests
        {"Utils coords equal", test_utils_coords_equal},
        {"Utils Manhattan distance", test_utils_manhattan_distance},
        {"Utils direction to string", test_utils_direction_to_string},
    };

    // Run all tests
    int total_tests = sizeof(tests) / sizeof(tests[0]);
    int passed_tests = 0;

    for (int i = 0; i < total_tests; i++) {
        if (tests[i].func()) {
            passed_tests++;
        }
    }

    // Print summary
    printf("\n=================================================\n");
    printf("Test Results: %d/%d passed\n", passed_tests, total_tests);

    if (passed_tests == total_tests) {
        printf("All tests PASSED!\n");
        printf("=================================================\n");
        return 0;
    } else {
        printf("Some tests FAILED!\n");
        printf("=================================================\n");
        return 1;
    }
}
