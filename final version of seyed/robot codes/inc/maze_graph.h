/**
 * @file maze_graph.h
 * @brief Graph data structure and shortest-path algorithms (C port).
 *
 * Manages the robot's KNOWN MAP — the subset of the true maze that has been
 * physically discovered.  All pathfinding operates ONLY on explored edges
 * and visited nodes (never on the full maze, which the robot does not know).
 *
 * Key operations:
 *   - Node / edge CRUD with static arrays
 *   - Dijkstra shortest path by REAL DISTANCE (cm), not hop count
 *   - Dijkstra one-to-all distances (used for frontier selection)
 *   - Straight-line (Euclidean) distance for proof bounds
 *   - Cardinal-direction queries for command generation
 *
 * All distances are uint32_t (cm); the coordinate system uses int16_t.
 * The graph is UNDIRECTED — every edge is traversable both ways.
 */

#ifndef MAZE_GRAPH_H
#define MAZE_GRAPH_H

#include "maze_types.h"
#include "maze_config.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * GRAPH LIFECYCLE
 *============================================================================*/

/**
 * @brief  Initialise an empty graph.
 * @param  graph       Pointer to uninitialised MazeGraph.
 * @param  start_coord World coordinate of the start node (node 0).
 * @return MAZE_OK on success, MAZE_ERR_* on failure.
 */
int maze_graph_init(MazeGraph* graph, MazeCoord start_coord);

/**
 * @brief  Reset the graph to empty (keeps allocated arrays, zeroes counters).
 * @param  graph Pointer to the graph.
 */
void maze_graph_reset(MazeGraph* graph);

/*============================================================================
 * NODE OPERATIONS
 *============================================================================*/

/**
 * @brief  Add a new intersection node at the given coordinates.
 * @param  graph Pointer to the graph.
 * @param  coord World position (cm).
 * @return Node index (>= 0) on success, MAZE_ERR_GRAPH_FULL if at capacity.
 */
int maze_graph_add_node(MazeGraph* graph, MazeCoord coord);

/**
 * @brief  Find a node within MAZE_COORD_TOLERANCE_CM of the given position.
 * @param  graph Pointer to the graph.
 * @param  coord World position to search for.
 * @return Node index (>= 0) if found, MAZE_ERR_NODE_NOT_FOUND otherwise.
 */
int maze_graph_find_node(const MazeGraph* graph, MazeCoord coord);

/**
 * @brief  Mark a node as visited by the robot.
 * @param  graph   Pointer to the graph.
 * @param  node_id Node index.
 */
void maze_graph_mark_visited(MazeGraph* graph, uint16_t node_id);

/**
 * @brief  Check whether a node has been physically visited.
 */
bool maze_graph_is_visited(const MazeGraph* graph, uint16_t node_id);

/**
 * @brief  Get a const pointer to a node's data.
 * @return Pointer to the MazeNode, or NULL if node_id is out of range.
 */
const MazeNode* maze_graph_get_node(const MazeGraph* graph, uint16_t node_id);

/*============================================================================
 * EDGE OPERATIONS
 *============================================================================*/

/**
 * @brief  Add (or retrieve) an undirected edge between two nodes.
 *
 * If the edge already exists this is a no-op and returns its index.
 * The edge length is computed from node coordinates (Manhattan for
 * axis-aligned edges, straight-line for validation).
 *
 * @param  graph   Pointer to the graph.
 * @param  node_a  First node index.
 * @param  node_b  Second node index.
 * @return Edge index (>= 0) on success, MAZE_ERR_* on failure.
 */
int maze_graph_add_edge(MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief  Mark an edge as explored (robot has driven across it).
 * @return MAZE_OK on success.
 */
int maze_graph_mark_explored(MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief  Check whether an edge exists and has been explored.
 */
bool maze_graph_is_explored(const MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief  Check whether an edge exists (explored or not).
 */
bool maze_graph_edge_exists(const MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief  Find the edge index connecting two nodes.
 * @return Edge index (>= 0) if found, -1 otherwise.
 */
int maze_graph_find_edge(const MazeGraph* graph, uint16_t node_a, uint16_t node_b);

/**
 * @brief  Get a const pointer to an edge's data, or NULL.
 */
const MazeEdge* maze_graph_get_edge(const MazeGraph* graph, uint16_t edge_id);

/*============================================================================
 * NEIGHBOR QUERIES
 *============================================================================*/

/**
 * @brief  Fill `neighbors` with up to `max_n` node indices adjacent to `node_id`.
 *         Only returns neighbors connected by EXPLORED edges (the known map).
 * @return Number of neighbors written.
 */
uint8_t maze_graph_get_explored_neighbors(const MazeGraph* graph,
                                          uint16_t node_id,
                                          uint16_t* neighbors,
                                          uint8_t max_n);

/**
 * @brief  Fill `unexplored` with node indices connected to `node_id` by edges
 *         that exist but are NOT yet explored (frontier branches).
 * @return Number of unexplored neighbors written.
 */
uint8_t maze_graph_get_unexplored_neighbors(const MazeGraph* graph,
                                            uint16_t node_id,
                                            uint16_t* unexplored,
                                            uint8_t max_n);

/**
 * @brief  Fill `neighbors` with ALL node indices adjacent to `node_id`
 *         (both explored and unexplored edges).
 * @return Number of neighbors written.
 */
uint8_t maze_graph_get_all_neighbors(const MazeGraph* graph,
                                     uint16_t node_id,
                                     uint16_t* neighbors,
                                     uint8_t max_n);

/*============================================================================
 * PATH FINDING — Dijkstra by REAL DISTANCE on the KNOWN MAP
 *============================================================================*/

/**
 * @brief  Run Dijkstra from `start` to all reachable nodes on the KNOWN MAP.
 *
 * Only traverses EXPLORED edges.  Fills `dist_cm` (distance in cm) and
 * `parent` (predecessor node) arrays.  Unreachable nodes get dist = UINT32_MAX.
 *
 * @param  graph      Pointer to the graph.
 * @param  start      Source node index.
 * @param  dist_cm    Output: distance array [node_count], caller-allocated.
 * @param  parent     Output: predecessor array [node_count], caller-allocated.
 *                     Set to MAZE_INVALID_NODE for unreachable nodes.
 */
void maze_graph_dijkstra(const MazeGraph* graph,
                         uint16_t start,
                         uint32_t* dist_cm,
                         uint16_t* parent);

/**
 * @brief  Extract the shortest path from `start` to `end` on the known map.
 *
 * Uses Dijkstra internally.  The path is written into `path` as a sequence
 * of node indices from start to end (inclusive).
 *
 * @param  graph       Pointer to the graph.
 * @param  start       Source node index.
 * @param  end         Target node index.
 * @param  path        Output: node sequence [start .. end].
 * @param  max_len     Capacity of `path` array.
 * @return Number of nodes in the path (>= 2 if a path exists), or 0 if
 *         no path was found.
 */
uint16_t maze_graph_shortest_path(const MazeGraph* graph,
                                  uint16_t start,
                                  uint16_t end,
                                  uint16_t* path,
                                  uint16_t max_len);

/**
 * @brief  Return the real distance (cm) of the shortest known-map path
 *         between two nodes, or UINT32_MAX if unreachable.
 */
uint32_t maze_graph_path_distance(const MazeGraph* graph,
                                  uint16_t start,
                                  uint16_t end);

/*============================================================================
 * GEOMETRY
 *============================================================================*/

/**
 * @brief  Straight-line (Euclidean) distance between two coordinates (cm).
 *
 * Uses integer sqrt approximation.  This is the admissible heuristic used
 * by the early-stop proof's lower-bound calculation.
 */
uint32_t maze_straight_line_cm(MazeCoord a, MazeCoord b);

/*============================================================================
 * COUNT QUERIES
 *============================================================================*/

static inline uint16_t maze_graph_node_count(const MazeGraph* g) { return g->node_count; }
static inline uint16_t maze_graph_edge_count(const MazeGraph* g) { return g->edge_count; }

#ifdef __cplusplus
}
#endif

#endif /* MAZE_GRAPH_H */
