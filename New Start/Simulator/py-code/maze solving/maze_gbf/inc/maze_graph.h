#ifndef MAZE_GRAPH_H
#define MAZE_GRAPH_H

#include "maze_types.h"

/**
 * @file maze_graph.h
 * @brief Graph data structure and operations API
 *
 * This file declares all functions for creating and manipulating
 * the maze graph structure, including node/edge management and
 * shortest path finding.
 */

/*============================================================================
 * GRAPH INITIALIZATION AND CLEANUP
 *============================================================================*/

/**
 * @brief Initialize a graph structure
 *
 * Allocates memory for the graph structure and internal arrays.
 * Must be called before using any other graph functions.
 *
 * @param graph Pointer to graph structure to initialize
 * @param max_nodes Maximum number of nodes the graph can hold
 * @param max_edges Maximum number of edges the graph can hold
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_graph_init(MazeGraph* graph, uint16_t max_nodes, uint16_t max_edges);

/**
 * @brief Clean up and free graph resources
 *
 * Frees all dynamically allocated memory associated with the graph.
 * After calling this function, the graph structure should not be used.
 *
 * @param graph Pointer to graph structure to clean up
 */
void maze_graph_cleanup(MazeGraph* graph);

/**
 * @brief Reset graph to empty state
 *
 * Clears all nodes and edges but keeps allocated memory.
 * Useful for reusing the same graph structure.
 *
 * @param graph Pointer to graph structure to reset
 */
void maze_graph_reset(MazeGraph* graph);

/*============================================================================
 * NODE OPERATIONS
 *============================================================================*/

/**
 * @brief Add a new node to the graph
 *
 * Creates a new node with the given coordinates and assigns it
 * a sequential ID (0, 1, 2, ...).
 *
 * @param graph Pointer to graph structure
 * @param coord Node coordinates
 * @return Node ID on success, MAZE_ERR_* on failure
 */
int maze_graph_add_node(MazeGraph* graph, MazeCoord coord);

/**
 * @brief Find node by coordinates
 *
 * Searches for a node within MAZE_COORD_TOLERANCE distance
 * of the given coordinates.
 *
 * @param graph Pointer to graph structure
 * @param coord Coordinates to search for
 * @return Node ID if found, -1 if not found
 */
int maze_graph_find_node_by_coord(const MazeGraph* graph, MazeCoord coord);

/**
 * @brief Get node by ID
 *
 * Returns pointer to the node structure with the given ID.
 *
 * @param graph Pointer to graph structure
 * @param node_id Node ID to retrieve
 * @return Pointer to node, or NULL if not found
 */
MazeNode* maze_graph_get_node(const MazeGraph* graph, uint16_t node_id);

/**
 * @brief Mark node as visited
 *
 * Sets the visited flag for the specified node.
 *
 * @param graph Pointer to graph structure
 * @param node_id Node ID to mark
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_graph_mark_node_visited(MazeGraph* graph, uint16_t node_id);

/**
 * @brief Check if node has been visited
 *
 * Returns the visited status of the specified node.
 *
 * @param graph Pointer to graph structure
 * @param node_id Node ID to check
 * @return true if visited, false otherwise
 */
bool maze_graph_is_node_visited(const MazeGraph* graph, uint16_t node_id);

/*============================================================================
 * EDGE OPERATIONS
 *============================================================================*/

/**
 * @brief Add an edge between two nodes
 *
 * Creates an undirected edge connecting two nodes.
 * If the edge already exists, this function returns MAZE_SUCCESS.
 *
 * @param graph Pointer to graph structure
 * @param node_a First node ID
 * @param node_b Second node ID
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_graph_add_edge(MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief Mark edge as explored
 *
 * Sets the explored flag for the edge connecting two nodes.
 *
 * @param graph Pointer to graph structure
 * @param node_a First node ID
 * @param node_b Second node ID
 * @return MAZE_SUCCESS on success, error code on failure
 */
int maze_graph_mark_edge_explored(MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief Check if edge is explored
 *
 * Returns the exploration status of the edge connecting two nodes.
 *
 * @param graph Pointer to graph structure
 * @param node_a First node ID
 * @param node_b Second node ID
 * @return true if edge exists and is explored, false otherwise
 */
bool maze_graph_is_edge_explored(const MazeGraph* graph,
                                  uint16_t node_a,
                                  uint16_t node_b);

/**
 * @brief Check if edge exists
 *
 * Returns whether an edge exists between two nodes.
 *
 * @param graph Pointer to graph structure
 * @param node_a First node ID
 * @param node_b Second node ID
 * @return true if edge exists, false otherwise
 */
bool maze_graph_edge_exists(const MazeGraph* graph,
                            uint16_t node_a,
                            uint16_t node_b);

/*============================================================================
 * NEIGHBOR QUERIES
 *============================================================================*/

/**
 * @brief Get all neighbors of a node
 *
 * Fills an array with IDs of all nodes connected to the given node.
 *
 * @param graph Pointer to graph structure
 * @param node_id Node to query
 * @param neighbors Output array for neighbor IDs
 * @param max_neighbors Maximum neighbors to return (array size)
 * @return Number of neighbors found
 */
uint16_t maze_graph_get_neighbors(const MazeGraph* graph,
                                   uint16_t node_id,
                                   uint16_t* neighbors,
                                   uint16_t max_neighbors);

/**
 * @brief Get unexplored neighbors of a node
 *
 * Fills an array with IDs of neighbors connected by unexplored edges.
 * These are the paths available for exploration.
 *
 * @param graph Pointer to graph structure
 * @param node_id Node to query
 * @param unexplored Output array for unexplored neighbor IDs
 * @param max_neighbors Maximum neighbors to return (array size)
 * @return Number of unexplored neighbors found
 */
uint16_t maze_graph_get_unexplored_neighbors(const MazeGraph* graph,
                                              uint16_t node_id,
                                              uint16_t* unexplored,
                                              uint16_t max_neighbors);

/**
 * @brief Get explored neighbors of a node
 *
 * Fills an array with IDs of neighbors connected by explored edges.
 * These are paths the robot has already traversed.
 *
 * @param graph Pointer to graph structure
 * @param node_id Node to query
 * @param explored Output array for explored neighbor IDs
 * @param max_neighbors Maximum neighbors to return (array size)
 * @return Number of explored neighbors found
 */
uint16_t maze_graph_get_explored_neighbors(const MazeGraph* graph,
                                            uint16_t node_id,
                                            uint16_t* explored,
                                            uint16_t max_neighbors);

/*============================================================================
 * PATH FINDING
 *============================================================================*/

/**
 * @brief Find shortest path between two nodes using BFS
 *
 * Computes the shortest path (in number of edges) between two nodes
 * using Breadth-First Search. The path is returned as an array of
 * node IDs from start to end.
 *
 * @param graph Pointer to graph structure
 * @param start_id Start node ID
 * @param end_id End node ID
 * @param path Output array for path (node IDs from start to end)
 * @param max_path_length Maximum path length (array size)
 * @return Path length (number of nodes), or 0 if no path found
 */
uint16_t maze_graph_shortest_path(const MazeGraph* graph,
                                   uint16_t start_id,
                                   uint16_t end_id,
                                   uint16_t* path,
                                   uint16_t max_path_length);

/**
 * @brief Calculate shortest path length between two nodes
 *
 * Returns the number of edges in the shortest path without storing
 * the actual path. Useful for comparing distances.
 *
 * @param graph Pointer to graph structure
 * @param start_id Start node ID
 * @param end_id End node ID
 * @return Path length in edges, or 0 if no path found
 */
uint16_t maze_graph_shortest_path_length(const MazeGraph* graph,
                                          uint16_t start_id,
                                          uint16_t end_id);

/*============================================================================
 * UTILITY FUNCTIONS
 *============================================================================*/

/**
 * @brief Get total number of nodes in graph
 *
 * @param graph Pointer to graph structure
 * @return Number of nodes
 */
uint16_t maze_graph_get_node_count(const MazeGraph* graph);

/**
 * @brief Get total number of edges in graph
 *
 * @param graph Pointer to graph structure
 * @return Number of edges
 */
uint16_t maze_graph_get_edge_count(const MazeGraph* graph);

/**
 * @brief Check if graph has reached capacity
 *
 * Returns true if either node or edge limit has been reached.
 *
 * @param graph Pointer to graph structure
 * @return true if at capacity, false otherwise
 */
bool maze_graph_is_full(const MazeGraph* graph);

#endif // MAZE_GRAPH_H
