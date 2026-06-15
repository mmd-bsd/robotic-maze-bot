/**
 * @file maze_graph.c
 * @brief Graph data structure and shortest-path algorithms (C port).
 *
 * Implements the KNOWN MAP — the subset of the true maze the robot has
 * physically discovered.  All pathfinding operates ONLY on explored edges.
 *
 * Dijkstra uses a binary heap; typical maze size (< 50 nodes) makes this
 * comfortably fast even on Cortex-M0+.
 */

#include "maze_graph.h"
#include <string.h>   /* memset */
#include <limits.h>   /* UINT32_MAX */

/* ==========================================================================
 * INTERNAL HELPERS
 * ========================================================================== */

/** Swap two uint16_t values in place. */
static inline void _swap_u16(uint16_t* a, uint16_t* b) {
    uint16_t t = *a; *a = *b; *b = t;
}

/** Swap two uint32_t values in place. */
static inline void _swap_u32(uint32_t* a, uint32_t* b) {
    uint32_t t = *a; *a = *b; *b = t;
}

/* ---- Integer square-root (Babylonian / Newton) ---- */
static uint32_t _isqrt_u32(uint32_t n) {
    if (n <= 1) return n;
    uint32_t x = n;
    uint32_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

/* ---- Binary min-heap for Dijkstra ---- */
typedef struct {
    uint16_t node;       /**< Node index                          */
    uint32_t dist;       /**< Current shortest distance (cm)      */
} HeapEntry;

typedef struct {
    HeapEntry data[50];       /* MAZE_MAX_NODES                    */
    uint16_t  size;
} MinHeap;

static void _heap_push(MinHeap* h, uint16_t node, uint32_t dist) {
    uint16_t i = h->size++;
    h->data[i].node = node;
    h->data[i].dist = dist;
    /* sift up */
    while (i > 0) {
        uint16_t p = (i - 1) >> 1;
        if (h->data[p].dist <= h->data[i].dist) break;
        _swap_u16(&h->data[p].node, &h->data[i].node);
        _swap_u32(&h->data[p].dist, &h->data[i].dist);
        i = p;
    }
}

static HeapEntry _heap_pop(MinHeap* h) {
    HeapEntry top = h->data[0];
    if (h->size <= 1) {
        h->size = 0;
        return top;
    }
    h->data[0] = h->data[--h->size];
    /* sift down */
    uint16_t i = 0;
    for (;;) {
        uint16_t left  = (i << 1) + 1;
        uint16_t right = left + 1;
        uint16_t smallest = i;
        if (left  < h->size && h->data[left].dist  < h->data[smallest].dist) smallest = left;
        if (right < h->size && h->data[right].dist < h->data[smallest].dist) smallest = right;
        if (smallest == i) break;
        _swap_u16(&h->data[i].node, &h->data[smallest].node);
        _swap_u32(&h->data[i].dist, &h->data[smallest].dist);
        i = smallest;
    }
    return top;
}

/* ==========================================================================
 * GRAPH LIFECYCLE
 * ========================================================================== */

int maze_graph_init(MazeGraph* graph, MazeCoord start_coord) {
    if (!graph) return MAZE_ERR_NULL_PTR;

    memset(graph, 0, sizeof(*graph));

    /* Create the start node at index 0 */
    graph->nodes[0].coord    = start_coord;
    graph->nodes[0].visited  = true;    /* robot begins here */
    graph->nodes[0].connections = 0;
    graph->node_count = 1;
    graph->edge_count = 0;
    graph->start_node = 0;
    graph->target_node = MAZE_INVALID_NODE;

    return MAZE_OK;
}

void maze_graph_reset(MazeGraph* graph) {
    if (!graph) return;
    memset(graph, 0, sizeof(*graph));
}

/* ==========================================================================
 * NODE OPERATIONS
 * ========================================================================== */

int maze_graph_add_node(MazeGraph* graph, MazeCoord coord) {
    if (!graph) return MAZE_ERR_NULL_PTR;
    if (graph->node_count >= MAZE_MAX_NODES) return MAZE_ERR_GRAPH_FULL;

    /* Check if a node already exists at these coordinates */
    int existing = maze_graph_find_node(graph, coord);
    if (existing >= 0) return existing;

    uint16_t idx = graph->node_count;
    graph->nodes[idx].coord       = coord;
    graph->nodes[idx].connections = 0;
    graph->nodes[idx].visited     = false;
    graph->node_count++;
    return idx;
}

int maze_graph_find_node(const MazeGraph* graph, MazeCoord coord) {
    if (!graph) return MAZE_ERR_NODE_NOT_FOUND;

    int16_t tol = MAZE_COORD_TOLERANCE_CM;
    for (uint16_t i = 0; i < graph->node_count; i++) {
        int16_t dx = graph->nodes[i].coord.x - coord.x;
        if (dx < 0) dx = -dx;
        int16_t dy = graph->nodes[i].coord.y - coord.y;
        if (dy < 0) dy = -dy;
        if (dx <= tol && dy <= tol) return i;
    }
    return MAZE_ERR_NODE_NOT_FOUND;
}

void maze_graph_mark_visited(MazeGraph* graph, uint16_t node_id) {
    if (!graph || node_id >= graph->node_count) return;
    graph->nodes[node_id].visited = true;
}

bool maze_graph_is_visited(const MazeGraph* graph, uint16_t node_id) {
    if (!graph || node_id >= graph->node_count) return false;
    return graph->nodes[node_id].visited;
}

const MazeNode* maze_graph_get_node(const MazeGraph* graph, uint16_t node_id) {
    if (!graph || node_id >= graph->node_count) return NULL;
    return &graph->nodes[node_id];
}

/* ==========================================================================
 * EDGE OPERATIONS
 * ========================================================================== */

/** Cardinal direction from node a to node b (axis-aligned maze). */
static MazeHeading _direction_between(MazeCoord a, MazeCoord b) {
    int16_t dx = b.x - a.x;
    int16_t dy = b.y - a.y;
    /* Y is up: determine primary axis */
    int16_t adx = (dx < 0) ? -dx : dx;
    int16_t ady = (dy < 0) ? -dy : dy;
    if (ady >= adx) {
        return (dy >= 0) ? MAZE_NORTH : MAZE_SOUTH;
    } else {
        return (dx >= 0) ? MAZE_EAST  : MAZE_WEST;
    }
}

/** Manhattan distance for axis-aligned edges (fraction-free). */
static uint16_t _manhattan_cm(MazeCoord a, MazeCoord b) {
    int16_t dx = b.x - a.x; if (dx < 0) dx = -dx;
    int16_t dy = b.y - a.y; if (dy < 0) dy = -dy;
    return (uint16_t)(dx + dy);
}

int maze_graph_add_edge(MazeGraph* graph, uint16_t a, uint16_t b) {
    if (!graph) return MAZE_ERR_NULL_PTR;
    if (a >= graph->node_count || b >= graph->node_count) return MAZE_ERR_NODE_NOT_FOUND;
    if (a == b) return MAZE_OK;       /* self-loop, ignore */

    /* Enforce canonical order (a < b) */
    if (a > b) { uint16_t t = a; a = b; b = t; }

    /* Already exists? */
    int existing = maze_graph_find_edge(graph, a, b);
    if (existing >= 0) return existing;

    if (graph->edge_count >= MAZE_MAX_EDGES) return MAZE_ERR_GRAPH_FULL;

    uint16_t idx = graph->edge_count;
    graph->edges[idx].node_a   = a;
    graph->edges[idx].node_b   = b;
    graph->edges[idx].length_cm = _manhattan_cm(graph->nodes[a].coord,
                                                 graph->nodes[b].coord);
    graph->edges[idx].heading   = _direction_between(graph->nodes[a].coord,
                                                      graph->nodes[b].coord);
    graph->edges[idx].explored  = false;
    graph->edge_count++;

    /* Record the connection bit on both nodes */
    uint8_t heading_a_to_b = _direction_between(graph->nodes[a].coord,
                                                 graph->nodes[b].coord);
    uint8_t heading_b_to_a = (heading_a_to_b + 2) & 0x3;  /* opposite direction */
    graph->nodes[a].connections |= (1 << heading_a_to_b);
    graph->nodes[b].connections |= (1 << heading_b_to_a);

    return idx;
}

int maze_graph_mark_explored(MazeGraph* graph, uint16_t a, uint16_t b) {
    int idx = maze_graph_find_edge(graph, a, b);
    if (idx < 0) return idx;
    graph->edges[idx].explored = true;
    return MAZE_OK;
}

bool maze_graph_is_explored(const MazeGraph* graph, uint16_t a, uint16_t b) {
    int idx = maze_graph_find_edge(graph, a, b);
    if (idx < 0) return false;
    return graph->edges[idx].explored;
}

bool maze_graph_edge_exists(const MazeGraph* graph, uint16_t a, uint16_t b) {
    return maze_graph_find_edge(graph, a, b) >= 0;
}

int maze_graph_find_edge(const MazeGraph* graph, uint16_t a, uint16_t b) {
    if (!graph) return -1;
    if (a > b) { uint16_t t = a; a = b; b = t; }   /* canonical order */

    for (uint16_t i = 0; i < graph->edge_count; i++) {
        if (graph->edges[i].node_a == a && graph->edges[i].node_b == b)
            return i;
    }
    return -1;
}

const MazeEdge* maze_graph_get_edge(const MazeGraph* graph, uint16_t edge_id) {
    if (!graph || edge_id >= graph->edge_count) return NULL;
    return &graph->edges[edge_id];
}

/* ==========================================================================
 * NEIGHBOR QUERIES
 * ========================================================================== */

uint8_t maze_graph_get_explored_neighbors(const MazeGraph* graph,
                                          uint16_t node_id,
                                          uint16_t* neighbors,
                                          uint8_t max_n) {
    uint8_t count = 0;
    if (!graph || node_id >= graph->node_count || max_n == 0) return 0;

    for (uint16_t i = 0; i < graph->edge_count && count < max_n; i++) {
        const MazeEdge* e = &graph->edges[i];
        if (!e->explored) continue;
        uint16_t other = MAZE_INVALID_NODE;
        if (e->node_a == node_id)      other = e->node_b;
        else if (e->node_b == node_id) other = e->node_a;
        if (other != MAZE_INVALID_NODE) neighbors[count++] = other;
    }
    return count;
}

uint8_t maze_graph_get_unexplored_neighbors(const MazeGraph* graph,
                                            uint16_t node_id,
                                            uint16_t* unexplored,
                                            uint8_t max_n) {
    uint8_t count = 0;
    if (!graph || node_id >= graph->node_count || max_n == 0) return 0;

    for (uint16_t i = 0; i < graph->edge_count && count < max_n; i++) {
        const MazeEdge* e = &graph->edges[i];
        if (e->explored) continue;    /* only edges NOT yet driven */
        uint16_t other = MAZE_INVALID_NODE;
        if (e->node_a == node_id)      other = e->node_b;
        else if (e->node_b == node_id) other = e->node_a;
        if (other != MAZE_INVALID_NODE) unexplored[count++] = other;
    }
    return count;
}

uint8_t maze_graph_get_all_neighbors(const MazeGraph* graph,
                                     uint16_t node_id,
                                     uint16_t* neighbors,
                                     uint8_t max_n) {
    uint8_t count = 0;
    if (!graph || node_id >= graph->node_count || max_n == 0) return 0;

    for (uint16_t i = 0; i < graph->edge_count && count < max_n; i++) {
        const MazeEdge* e = &graph->edges[i];
        uint16_t other = MAZE_INVALID_NODE;
        if (e->node_a == node_id)      other = e->node_b;
        else if (e->node_b == node_id) other = e->node_a;
        if (other != MAZE_INVALID_NODE) neighbors[count++] = other;
    }
    return count;
}

/* ==========================================================================
 * DIJKSTRA — shortest path by REAL DISTANCE (cm) on the KNOWN MAP
 * ========================================================================== */

void maze_graph_dijkstra(const MazeGraph* graph,
                         uint16_t start,
                         uint32_t* dist_cm,
                         uint16_t* parent) {
    if (!graph || !dist_cm || !parent) return;

    uint16_t  n       = graph->node_count;
    bool      visited[50] = {0};        /* MAZE_MAX_NODES, stack-allocated */

    for (uint16_t i = 0; i < n; i++) {
        dist_cm[i] = UINT32_MAX;
        parent[i]  = MAZE_INVALID_NODE;
    }

    if (start >= n) return;

    dist_cm[start] = 0;
    MinHeap heap = {{{0, 0}}, 0};
    _heap_push(&heap, start, 0);

    while (heap.size > 0) {
        HeapEntry cur = _heap_pop(&heap);
        if (visited[cur.node]) continue;
        visited[cur.node] = true;

        /* Relax all explored neighbors */
        uint16_t neighbors[4];
        uint8_t  deg = maze_graph_get_explored_neighbors(graph, cur.node,
                                                          neighbors, 4);
        for (uint8_t k = 0; k < deg; k++) {
            uint16_t v = neighbors[k];
            if (visited[v]) continue;

            /* Edge length in cm */
            int edge_idx = maze_graph_find_edge(graph, cur.node, v);
            if (edge_idx < 0) continue;
            uint32_t edge_len = graph->edges[edge_idx].length_cm;

            uint32_t new_dist = cur.dist + edge_len;
            if (new_dist < dist_cm[v]) {
                dist_cm[v] = new_dist;
                parent[v]  = cur.node;
                _heap_push(&heap, v, new_dist);
            }
        }
    }
}

uint16_t maze_graph_shortest_path(const MazeGraph* graph,
                                  uint16_t start,
                                  uint16_t end,
                                  uint16_t* path,
                                  uint16_t max_len) {
    if (!graph || !path || max_len < 2) return 0;
    if (start >= graph->node_count || end >= graph->node_count) return 0;
    if (start == end) {
        path[0] = start;
        return 1;
    }

    uint32_t dist[50];
    uint16_t parent[50];
    maze_graph_dijkstra(graph, start, dist, parent);

    if (dist[end] == UINT32_MAX) return 0;   /* unreachable */

    /* Reconstruct path backwards, then reverse in-place */
    uint16_t rev[50];   /* MAZE_MAX_PATH_LENGTH */
    uint16_t len = 0;
    uint16_t cur = end;
    while (cur != MAZE_INVALID_NODE && len < 50) {
        rev[len++] = cur;
        cur = parent[cur];
    }

    /* len is now path length.  Reverse into `path`. */
    if (len > max_len) len = max_len;
    for (uint16_t i = 0; i < len; i++) {
        path[i] = rev[len - 1 - i];
    }
    return len;
}

uint32_t maze_graph_path_distance(const MazeGraph* graph,
                                  uint16_t start,
                                  uint16_t end) {
    if (!graph || start >= graph->node_count || end >= graph->node_count)
        return UINT32_MAX;

    uint32_t dist[50];
    uint16_t parent[50];
    maze_graph_dijkstra(graph, start, dist, parent);
    return dist[end];
}

/* ==========================================================================
 * GEOMETRY
 * ========================================================================== */

uint32_t maze_straight_line_cm(MazeCoord a, MazeCoord b) {
    int32_t dx = (int32_t)b.x - (int32_t)a.x;
    int32_t dy = (int32_t)b.y - (int32_t)a.y;
    uint32_t sq = (uint32_t)(dx * dx + dy * dy);
    return _isqrt_u32(sq);
}
