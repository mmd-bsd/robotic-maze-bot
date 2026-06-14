/**
 * @file maze_graph.c
 * @brief Graph data structure and operations implementation
 *
 * This file implements all graph operations including node/edge management,
 * neighbor queries, and BFS shortest path finding.
 */

#include "maze_graph.h"
#include "maze_config.h"
#include <stdlib.h>
#include <stdlib.h>
#include <string.h>

/*============================================================================
 * INTERNAL HELPER FUNCTIONS
 *============================================================================*/

/**
 * @brief Find edge index in edges array
 *
 * Internal helper to find the index of an edge in the edges array.
 * Returns -1 if edge doesn't exist.
 */
static int find_edge_index(const MazeGraph* graph, uint16_t node_a, uint16_t node_b) {
    if (graph == NULL) {
        return -1;
    }

    // Ensure consistent ordering (node_a < node_b)
    if (node_a > node_b) {
        uint16_t temp = node_a;
        node_a = node_b;
        node_b = temp;
    }

    for (uint16_t i = 0; i < graph->edge_count; i++) {
        if (graph->edges[i].node_a == node_a && graph->edges[i].node_b == node_b) {
            return (int)i;
        }
    }

    return -1;
}

/*============================================================================
 * GRAPH INITIALIZATION AND CLEANUP
 *============================================================================*/

int maze_graph_init(MazeGraph* graph, uint16_t max_nodes, uint16_t max_edges) {
    if (graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (max_nodes == 0 || max_edges == 0) {
        return MAZE_ERR_INVALID_PARAM;
    }

    // Allocate nodes array
    graph->nodes = (MazeNode*)malloc(max_nodes * sizeof(MazeNode));
    if (graph->nodes == NULL) {
        return MAZE_ERR_NO_MEMORY;
    }

    // Allocate edges array
    graph->edges = (MazeEdge*)malloc(max_edges * sizeof(MazeEdge));
    if (graph->edges == NULL) {
        free(graph->nodes);
        return MAZE_ERR_NO_MEMORY;
    }

    // Allocate adjacency list array
    graph->adj_list = (MazeAdjListEntry**)malloc(max_nodes * sizeof(MazeAdjListEntry*));
    if (graph->adj_list == NULL) {
        free(graph->edges);
        free(graph->nodes);
        return MAZE_ERR_NO_MEMORY;
    }

    // Initialize adjacency lists to NULL
    for (uint16_t i = 0; i < max_nodes; i++) {
        graph->adj_list[i] = NULL;
    }

    // Initialize graph state
    graph->node_count = 0;
    graph->edge_count = 0;
    graph->max_nodes = max_nodes;
    graph->max_edges = max_edges;

    return MAZE_SUCCESS;
}

void maze_graph_cleanup(MazeGraph* graph) {
    if (graph == NULL) {
        return;
    }

    // Free adjacency list entries
    if (graph->adj_list != NULL) {
        for (uint16_t i = 0; i < graph->max_nodes; i++) {
            MazeAdjListEntry* entry = graph->adj_list[i];
            while (entry != NULL) {
                MazeAdjListEntry* next = entry->next;
                free(entry);
                entry = next;
            }
        }
        free(graph->adj_list);
    }

    // Free arrays
    if (graph->nodes != NULL) {
        free(graph->nodes);
    }
    if (graph->edges != NULL) {
        free(graph->edges);
    }

    // Clear pointers
    graph->nodes = NULL;
    graph->edges = NULL;
    graph->adj_list = NULL;
    graph->node_count = 0;
    graph->edge_count = 0;
}

void maze_graph_reset(MazeGraph* graph) {
    if (graph == NULL) {
        return;
    }

    // Free adjacency list entries
    for (uint16_t i = 0; i < graph->max_nodes; i++) {
        MazeAdjListEntry* entry = graph->adj_list[i];
        while (entry != NULL) {
            MazeAdjListEntry* next = entry->next;
            free(entry);
            entry = next;
        }
        graph->adj_list[i] = NULL;
    }

    // Reset counters
    graph->node_count = 0;
    graph->edge_count = 0;
}

/*============================================================================
 * NODE OPERATIONS
 *============================================================================*/

int maze_graph_add_node(MazeGraph* graph, MazeCoord coord) {
    if (graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (graph->node_count >= graph->max_nodes) {
        return MAZE_ERR_GRAPH_FULL;
    }

    // Assign sequential ID
    uint16_t new_id = graph->node_count;

    // Initialize node
    graph->nodes[new_id].id = new_id;
    graph->nodes[new_id].coord = coord;
    graph->nodes[new_id].visited = false;

    // Increment count
    graph->node_count++;

    MAZE_DEBUG_PRINT("Added node %d at (%d, %d)\n", new_id, coord.x, coord.y);

    return (int)new_id;
}

int maze_graph_find_node_by_coord(const MazeGraph* graph, MazeCoord coord) {
    if (graph == NULL) {
        return -1;
    }

    for (uint16_t i = 0; i < graph->node_count; i++) {
        int16_t dx = abs(graph->nodes[i].coord.x - coord.x);
        int16_t dy = abs(graph->nodes[i].coord.y - coord.y);

        if (dx <= MAZE_COORD_TOLERANCE && dy <= MAZE_COORD_TOLERANCE) {
            return (int)graph->nodes[i].id;
        }
    }

    return -1;  // Not found
}

MazeNode* maze_graph_get_node(const MazeGraph* graph, uint16_t node_id) {
    if (graph == NULL || node_id >= graph->node_count) {
        return NULL;
    }

    return &graph->nodes[node_id];
}

int maze_graph_mark_node_visited(MazeGraph* graph, uint16_t node_id) {
    if (graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (node_id >= graph->node_count) {
        return MAZE_ERR_NODE_NOT_FOUND;
    }

    graph->nodes[node_id].visited = true;
    return MAZE_SUCCESS;
}

bool maze_graph_is_node_visited(const MazeGraph* graph, uint16_t node_id) {
    if (graph == NULL || node_id >= graph->node_count) {
        return false;
    }

    return graph->nodes[node_id].visited;
}

/*============================================================================
 * EDGE OPERATIONS
 *============================================================================*/

int maze_graph_add_edge(MazeGraph* graph, uint16_t node_a, uint16_t node_b) {
    if (graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    if (node_a >= graph->node_count || node_b >= graph->node_count) {
        return MAZE_ERR_NODE_NOT_FOUND;
    }

    if (node_a == node_b) {
        return MAZE_ERR_INVALID_PARAM;  // No self-loops
    }

    if (graph->edge_count >= graph->max_edges) {
        return MAZE_ERR_GRAPH_FULL;
    }

    // Check if edge already exists
    if (maze_graph_edge_exists(graph, node_a, node_b)) {
        return MAZE_SUCCESS;  // Already exists
    }

    // Ensure consistent ordering (node_a < node_b)
    if (node_a > node_b) {
        uint16_t temp = node_a;
        node_a = node_b;
        node_b = temp;
    }

    // Add edge
    uint16_t edge_idx = graph->edge_count;
    graph->edges[edge_idx].node_a = node_a;
    graph->edges[edge_idx].node_b = node_b;
    graph->edges[edge_idx].explored = false;
    graph->edge_count++;

    // Update adjacency lists (undirected graph)
    // Add node_b to node_a's list
    MazeAdjListEntry* entry_a = (MazeAdjListEntry*)malloc(sizeof(MazeAdjListEntry));
    if (entry_a == NULL) {
        return MAZE_ERR_NO_MEMORY;
    }
    entry_a->neighbor_id = node_b;
    entry_a->next = graph->adj_list[node_a];
    graph->adj_list[node_a] = entry_a;

    // Add node_a to node_b's list
    MazeAdjListEntry* entry_b = (MazeAdjListEntry*)malloc(sizeof(MazeAdjListEntry));
    if (entry_b == NULL) {
        // Rollback entry_a
        graph->adj_list[node_a] = entry_a->next;
        free(entry_a);
        return MAZE_ERR_NO_MEMORY;
    }
    entry_b->neighbor_id = node_a;
    entry_b->next = graph->adj_list[node_b];
    graph->adj_list[node_b] = entry_b;

    MAZE_DEBUG_PRINT("Added edge %d-%d\n", node_a, node_b);

    return MAZE_SUCCESS;
}

int maze_graph_mark_edge_explored(MazeGraph* graph, uint16_t node_a, uint16_t node_b) {
    if (graph == NULL) {
        return MAZE_ERR_NULL_PTR;
    }

    int edge_idx = find_edge_index(graph, node_a, node_b);
    if (edge_idx < 0) {
        return MAZE_ERR_NODE_NOT_FOUND;  // Edge doesn't exist
    }

    graph->edges[edge_idx].explored = true;
    return MAZE_SUCCESS;
}

bool maze_graph_is_edge_explored(const MazeGraph* graph,
                                  uint16_t node_a,
                                  uint16_t node_b) {
    if (graph == NULL) {
        return false;
    }

    int edge_idx = find_edge_index(graph, node_a, node_b);
    if (edge_idx < 0) {
        return false;  // Edge doesn't exist
    }

    return graph->edges[edge_idx].explored;
}

bool maze_graph_edge_exists(const MazeGraph* graph,
                            uint16_t node_a,
                            uint16_t node_b) {
    if (graph == NULL) {
        return false;
    }

    return find_edge_index(graph, node_a, node_b) >= 0;
}

/*============================================================================
 * NEIGHBOR QUERIES
 *============================================================================*/

uint16_t maze_graph_get_neighbors(const MazeGraph* graph,
                                   uint16_t node_id,
                                   uint16_t* neighbors,
                                   uint16_t max_neighbors) {
    if (graph == NULL || neighbors == NULL) {
        return 0;
    }

    if (node_id >= graph->node_count) {
        return 0;
    }

    uint16_t count = 0;
    MazeAdjListEntry* entry = graph->adj_list[node_id];

    while (entry != NULL && count < max_neighbors) {
        neighbors[count++] = entry->neighbor_id;
        entry = entry->next;
    }

    return count;
}

uint16_t maze_graph_get_unexplored_neighbors(const MazeGraph* graph,
                                              uint16_t node_id,
                                              uint16_t* unexplored,
                                              uint16_t max_neighbors) {
    if (graph == NULL || unexplored == NULL) {
        return 0;
    }

    if (node_id >= graph->node_count) {
        return 0;
    }

    uint16_t count = 0;
    MazeAdjListEntry* entry = graph->adj_list[node_id];

    while (entry != NULL && count < max_neighbors) {
        // Check if edge to this neighbor is explored
        if (!maze_graph_is_edge_explored(graph, node_id, entry->neighbor_id)) {
            unexplored[count++] = entry->neighbor_id;
        }
        entry = entry->next;
    }

    return count;
}

uint16_t maze_graph_get_explored_neighbors(const MazeGraph* graph,
                                            uint16_t node_id,
                                            uint16_t* explored,
                                            uint16_t max_neighbors) {
    if (graph == NULL || explored == NULL) {
        return 0;
    }

    if (node_id >= graph->node_count) {
        return 0;
    }

    uint16_t count = 0;
    MazeAdjListEntry* entry = graph->adj_list[node_id];

    while (entry != NULL && count < max_neighbors) {
        // Check if edge to this neighbor is explored
        if (maze_graph_is_edge_explored(graph, node_id, entry->neighbor_id)) {
            explored[count++] = entry->neighbor_id;
        }
        entry = entry->next;
    }

    return count;
}

/*============================================================================
 * PATH FINDING (BFS)
 *============================================================================*/

uint16_t maze_graph_shortest_path(const MazeGraph* graph,
                                   uint16_t start_id,
                                   uint16_t end_id,
                                   uint16_t* path,
                                   uint16_t max_path_length) {
    if (graph == NULL || path == NULL) {
        return 0;
    }

    if (start_id >= graph->node_count || end_id >= graph->node_count) {
        return 0;
    }

    if (start_id == end_id) {
        path[0] = start_id;
        return 1;
    }

    // BFS implementation
    uint16_t queue[MAZE_MAX_NODES];
    uint16_t parent[MAZE_MAX_NODES];
    bool visited[MAZE_MAX_NODES];

    memset(visited, 0, sizeof(visited));
    for (uint16_t i = 0; i < MAZE_MAX_NODES; i++) {
        parent[i] = UINT16_MAX;
    }

    uint16_t queue_front = 0;
    uint16_t queue_back = 0;

    queue[queue_back++] = start_id;
    visited[start_id] = true;

    while (queue_front < queue_back) {
        uint16_t current = queue[queue_front++];

        if (current == end_id) {
            // Reconstruct path
            uint16_t path_len = 0;
            uint16_t temp = end_id;

            while (temp != UINT16_MAX) {
                if (path_len >= max_path_length) {
                    return 0;  // Path too long for buffer
                }
                path[path_len++] = temp;
                temp = parent[temp];
            }

            // Reverse path (currently backwards: end -> start)
            for (uint16_t i = 0; i < path_len / 2; i++) {
                uint16_t tmp = path[i];
                path[i] = path[path_len - 1 - i];
                path[path_len - 1 - i] = tmp;
            }

            return path_len;
        }

        // Get neighbors
        uint16_t neighbors[4];
        uint16_t neighbor_count = maze_graph_get_neighbors(
            graph, current, neighbors, 4);

        for (uint16_t i = 0; i < neighbor_count; i++) {
            uint16_t neighbor = neighbors[i];
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                queue[queue_back++] = neighbor;
            }
        }
    }

    return 0;  // No path found
}

uint16_t maze_graph_shortest_path_length(const MazeGraph* graph,
                                          uint16_t start_id,
                                          uint16_t end_id) {
    if (graph == NULL) {
        return 0;
    }

    if (start_id >= graph->node_count || end_id >= graph->node_count) {
        return 0;
    }

    if (start_id == end_id) {
        return 0;  // Same node, zero edges
    }

    // BFS to find distance (number of edges)
    uint16_t queue[MAZE_MAX_NODES];
    uint16_t distance[MAZE_MAX_NODES];
    bool visited[MAZE_MAX_NODES];

    memset(visited, 0, sizeof(visited));
    memset(distance, 0, sizeof(distance));

    uint16_t queue_front = 0;
    uint16_t queue_back = 0;

    queue[queue_back++] = start_id;
    visited[start_id] = true;

    while (queue_front < queue_back) {
        uint16_t current = queue[queue_front++];

        if (current == end_id) {
            return distance[current];
        }

        // Get neighbors
        uint16_t neighbors[4];
        uint16_t neighbor_count = maze_graph_get_neighbors(
            graph, current, neighbors, 4);

        for (uint16_t i = 0; i < neighbor_count; i++) {
            uint16_t neighbor = neighbors[i];
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                distance[neighbor] = distance[current] + 1;
                queue[queue_back++] = neighbor;
            }
        }
    }

    return 0;  // No path found
}

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

uint16_t maze_graph_get_node_count(const MazeGraph* graph) {
    if (graph == NULL) {
        return 0;
    }
    return graph->node_count;
}

uint16_t maze_graph_get_edge_count(const MazeGraph* graph) {
    if (graph == NULL) {
        return 0;
    }
    return graph->edge_count;
}

bool maze_graph_is_full(const MazeGraph* graph) {
    if (graph == NULL) {
        return true;
    }
    return (graph->node_count >= graph->max_nodes ||
            graph->edge_count >= graph->max_edges);
}
