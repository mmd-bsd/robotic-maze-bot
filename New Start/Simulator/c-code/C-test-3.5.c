#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>

// --- Debug Print Control Macro ---
// Set to 1 to enable debug printing, 0 to disable (only final path will be printed)
#define ENABLE_DEBUG_PRINT 0

// Macro for conditional debug printing
#if ENABLE_DEBUG_PRINT
    #define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...) ((void)0)
#endif

#define MAX_NODES 100
#define MAX_NEIGHBORS 10
#define MAX_PATH_LENGTH 1000

// --- 1. Define Maze Structure ---
typedef struct {
    int x, y;
} Point;

typedef struct {
    int node;
    Point pos;
} NodePosition;

// Node positions
// --- ORIGINAL GRAPH (from integration_test_V1) - COMMENTED OUT ---
/*
NodePosition nodes_positions[] = {
    {0, {0, 80}},
    {1, {0, 180}},
    {2, {0, 280}},
    {3, {40, 20}},
    {4, {40, 40}},
    {5, {40, 80}},
    {6, {40, 260}},
    {7, {40, 280}},
    {8, {60, 20}},
    {9, {60, 40}},
    {10, {60, 80}},
    {11, {60, 180}},
    {12, {60, 220}},
    {13, {60, 240}},
    {14, {60, 280}},
    {15, {60, 300}},
    {16, {80, 140}},
    {17, {80, 180}},
    {18, {80, 200}},
    {19, {100, 0}},
    {20, {100, 40}},
    {21, {100, 60}},
    {22, {100, 140}},
    {23, {100, 160}},
    {24, {100, 180}},
    {25, {100, 200}},
    {26, {120, 100}},
    {27, {120, 140}},
    {28, {120, 160}},
    {29, {120, 180}},
    {30, {120, 240}},
    {31, {120, 280}},
    {32, {140, 0}},
    {33, {140, 40}},
    {34, {140, 80}},
    {35, {140, 100}},
    {36, {140, 120}},
    {37, {140, 240}},
    {38, {180, 20}},
    {39, {180, 40}},
    {40, {180, 60}},
    {41, {180, 100}},  // Target Node
    {42, {180, 220}},
    {43, {180, 280}},
    {44, {200, 80}},
    {45, {200, 100}},
    {46, {220, 220}},
    {47, {220, 240}},
    {48, {220, 280}},
    {49, {240, 60}},
    {50, {240, 80}},
    {51, {240, 100}},
    {52, {240, 180}},
    {53, {240, 200}},
    {54, {240, 240}},
    {55, {260, 60}},
    {56, {260, 80}},
    {57, {260, 100}},
    {58, {280, 60}},
    {59, {280, 80}},
    {60, {280, 100}},
    {61, {280, 120}},
    {62, {280, 140}},
    {63, {300, 80}},
    {64, {300, 140}},
    {65, {300, 200}},
    {66, {320, 80}},
    {67, {320, 140}},
};

int NUM_NODES = 68;
int START_NODE = 2;
int TARGET_NODE = 41;
*/

// --- NEW GRAPH (from sim_V12_GBF_stable.py) ---
NodePosition nodes_positions[] = {
    {0, {0, 0}},      // start
    {1, {0, 20}},
    {2, {0, 40}},
    {3, {0, 60}},
    {4, {0, 100}},
    {5, {0, 120}},
    {6, {20, 40}},
    {7, {20, 100}},
    {8, {20, 120}},
    {9, {40, 40}},
    {10, {40, 60}},
    {11, {40, 80}},
    {12, {60, 40}},
    {13, {60, 80}},
    {14, {-20, 40}},
    {15, {-20, 60}},
    {16, {-20, 80}},
    {17, {-40, 40}},
    {18, {-40, 60}},
    {19, {-40, 80}},
    {20, {-40, 100}},
    {21, {-60, 20}},
    {22, {-60, 80}},  // Target Node
    {23, {-60, 120}},
    {24, {-80, 40}},
    {25, {-80, 60}},
    {26, {-80, 80}},
};

int NUM_NODES = 27;
int START_NODE = 0;
int TARGET_NODE = 22;

// Edges list
typedef struct {
    int u, v;
} Edge;

// --- ORIGINAL GRAPH EDGES (from integration_test_V1) - COMMENTED OUT ---
/*
Edge edges_list[] = {
    {13, 14},
    {11, 17},
    {1, 11},
    {26, 27},
    {22, 27},
    {0, 1},
    {34, 35},
    {41, 45},
    {47, 48},
    {66, 67},
    {43, 48},
    {53, 54},
    {63, 66},
    {26, 35},
    {50, 51},
    {56, 57},
    {13, 30},
    {6, 7},
    {35, 36},
    {57, 60},
    {30, 31},
    {40, 41},
    {23, 24},
    {64, 67},
    {62, 64},
    {21, 40},
    {10, 11},
    {64, 65},
    {8, 38},
    {31, 43},
    {19, 20},
    {2, 7},
    {32, 33},
    {42, 43},
    {58, 59},
    {16, 17},
    {47, 54},
    {38, 39},
    {24, 25},
    {28, 29},
    {8, 9},
    {16, 22},
    {18, 25},
    {4, 5},
    {53, 65},
    {3, 4},
    {55, 56},
    {59, 63},
    {39, 40},
    {36, 61},
    {17, 18},
    {27, 28},
    {55, 58},
    {42, 46},
    {21, 22},
    {49, 50},
    {4, 9},
    {46, 47},
    {61, 62},
    {29, 52},
    {50, 56},
    {14, 15},
    {7, 14},
    {5, 10},
    {44, 45},
    {52, 53},
    {0, 5},
    {30, 37},
    {23, 28},
    {44, 50},
    {63, 64},
    {12, 13},
    {59, 60},
    {14, 31},
};

int NUM_EDGES = 75;
*/

// --- NEW GRAPH EDGES (from sim_V12_GBF_stable.py) ---
Edge edges_list[] = {
    {0, 1},
    {1, 2},
    {1, 21},
    {2, 3},
    {2, 6},
    {3, 4},
    {3, 10},
    {3, 15},
    {4, 5},
    {4, 7},
    {4, 20},
    {7, 8},
    {9, 10},
    {9, 12},
    {10, 11},
    {11, 13},
    {12, 13},
    {14, 15},
    {14, 17},
    {15, 16},
    {16, 19},
    {17, 18},
    {17, 24},
    {18, 25},
    {19, 20},
    {19, 22},
    {22, 23},
    {22, 26},
    {24, 25},
    {25, 26},
};

int NUM_EDGES = 29;

// --- 2. Graph Structure ---
typedef struct {
    int neighbors[MAX_NEIGHBORS];
    int num_neighbors;
} AdjacencyList;

typedef struct {
    AdjacencyList adj[MAX_NODES];
    Point positions[MAX_NODES];
    int num_nodes;
} Graph;

// --- 3. Queue for BFS ---
typedef struct {
    int items[MAX_NODES];
    int front;
    int rear;
} Queue;

Queue* create_queue() {
    Queue* q = (Queue*)malloc(sizeof(Queue));
    q->front = -1;
    q->rear = -1;
    return q;
}

bool is_empty(Queue* q) {
    return q->rear == -1;
}

void enqueue(Queue* q, int value) {
    if (q->rear == MAX_NODES - 1) {
        DEBUG_PRINT("Queue is full\n");
        return;
    }
    if (q->front == -1) {
        q->front = 0;
    }
    q->rear++;
    q->items[q->rear] = value;
}

int dequeue(Queue* q) {
    if (is_empty(q)) {
        return -1;
    }
    int item = q->items[q->front];
    q->front++;
    if (q->front > q->rear) {
        q->front = q->rear = -1;
    }
    return item;
}

// --- 4. Set operations (using boolean arrays) ---
typedef struct {
    bool members[MAX_NODES * MAX_NODES];
} EdgeSet;

typedef struct {
    bool members[MAX_NODES];
    int count;
} NodeSet;

void edge_set_init(EdgeSet* set) {
    memset(set->members, false, sizeof(set->members));
}

int edge_to_index(int u, int v) {
    if (u > v) {
        int temp = u;
        u = v;
        v = temp;
    }
    return u * MAX_NODES + v;
}

void edge_set_add(EdgeSet* set, int u, int v) {
    set->members[edge_to_index(u, v)] = true;
}

bool edge_set_contains(EdgeSet* set, int u, int v) {
    return set->members[edge_to_index(u, v)];
}

void node_set_init(NodeSet* set) {
    memset(set->members, false, sizeof(set->members));
    set->count = 0;
}

void node_set_add(NodeSet* set, int node) {
    if (!set->members[node]) {
        set->members[node] = true;
        set->count++;
    }
}

bool node_set_contains(NodeSet* set, int node) {
    return set->members[node];
}

// --- 5. Graph Functions ---
void init_graph(Graph* g) {
    g->num_nodes = NUM_NODES;
    
    // Initialize adjacency lists
    for (int i = 0; i < MAX_NODES; i++) {
        g->adj[i].num_neighbors = 0;
    }
    
    // Add positions
    for (int i = 0; i < NUM_NODES; i++) {
        g->positions[i] = nodes_positions[i].pos;
    }
    
    // Build adjacency list
    for (int i = 0; i < NUM_EDGES; i++) {
        int u = edges_list[i].u;
        int v = edges_list[i].v;
        
        if (g->adj[u].num_neighbors < MAX_NEIGHBORS) {
            g->adj[u].neighbors[g->adj[u].num_neighbors++] = v;
        }
        if (g->adj[v].num_neighbors < MAX_NEIGHBORS) {
            g->adj[v].neighbors[g->adj[v].num_neighbors++] = u;
        }
    }
}

// BFS to find shortest path (with constraints for exploration)
bool bfs_shortest_path(Graph* g, int start, int target, int* path, int* path_length, NodeSet* known_nodes, EdgeSet* known_edges) {
    int parent[MAX_NODES];
    bool visited[MAX_NODES];
    memset(visited, false, sizeof(visited));
    memset(parent, -1, sizeof(parent));
    
    Queue* q = create_queue();
    visited[start] = true;
    enqueue(q, start);
    
    while (!is_empty(q)) {
        int current = dequeue(q);
        
        if (current == target) {
            // Reconstruct path
            int temp_path[MAX_NODES];
            int idx = 0;
            int node = target;
            
            while (node != -1) {
                temp_path[idx++] = node;
                node = parent[node];
            }
            
            // Reverse path
            *path_length = idx;
            for (int i = 0; i < idx; i++) {
                path[i] = temp_path[idx - 1 - i];
            }
            
            free(q);
            return true;
        }
        
        // Only traverse edges in known map
        for (int i = 0; i < g->adj[current].num_neighbors; i++) {
            int neighbor = g->adj[current].neighbors[i];
            
            // Check if edge is in known map
            if (!node_set_contains(known_nodes, neighbor)) {
                continue;
            }
            if (!edge_set_contains(known_edges, current, neighbor)) {
                continue;
            }
            
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                enqueue(q, neighbor);
            }
        }
    }
    
    free(q);
    return false;
}

// BFS to find shortest path using FULL graph (for optimal path calculation after exploration)
bool bfs_shortest_path_full(Graph* g, int start, int target, int* path, int* path_length) {
    int parent[MAX_NODES];
    bool visited[MAX_NODES];
    memset(visited, false, sizeof(visited));
    memset(parent, -1, sizeof(parent));
    
    Queue* q = create_queue();
    visited[start] = true;
    enqueue(q, start);
    
    while (!is_empty(q)) {
        int current = dequeue(q);
        
        if (current == target) {
            // Reconstruct path
            int temp_path[MAX_NODES];
            int idx = 0;
            int node = target;
            
            while (node != -1) {
                temp_path[idx++] = node;
                node = parent[node];
            }
            
            // Reverse path
            *path_length = idx;
            for (int i = 0; i < idx; i++) {
                path[i] = temp_path[idx - 1 - i];
            }
            
            free(q);
            return true;
        }
        
        // Use FULL graph - no constraints
        for (int i = 0; i < g->adj[current].num_neighbors; i++) {
            int neighbor = g->adj[current].neighbors[i];
            
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                parent[neighbor] = current;
                enqueue(q, neighbor);
            }
        }
    }
    
    free(q);
    return false;
}

// BFS to find shortest path length
int bfs_shortest_path_length(Graph* g, int start, int target, NodeSet* known_nodes, EdgeSet* known_edges) {
    int path[MAX_NODES];
    int path_length;
    if (bfs_shortest_path(g, start, target, path, &path_length, known_nodes, known_edges)) {
        return path_length - 1; // Return number of edges, not nodes
    }
    return INT_MAX;
}

// --- 6. Robot Structure ---
typedef enum {
    NORTH = 0,  // (0, 1)
    EAST = 1,   // (1, 0)
    SOUTH = 2,  // (0, -1)
    WEST = 3    // (-1, 0)
} Direction;

typedef struct {
    Graph* graph;
    int start_node;
    int target_node;
    int current_node;
    EdgeSet visited_edges;
    NodeSet visited_nodes;
    int path_history_log[MAX_PATH_LENGTH];
    int path_history_length;
    bool exploration_complete;
    bool target_found_once;
    Direction current_heading;  // Current direction robot is facing
    char command_string[MAX_PATH_LENGTH * 3];  // Movement commands (F, L, R, B)
    int command_string_length;
} Robot;

void robot_init(Robot* robot, Graph* g, int start, int target) {
    robot->graph = g;
    robot->start_node = start;
    robot->target_node = target;
    robot->current_node = start;
    edge_set_init(&robot->visited_edges);
    node_set_init(&robot->visited_nodes);
    node_set_add(&robot->visited_nodes, start);
    robot->path_history_log[0] = start;
    robot->path_history_length = 1;
    robot->exploration_complete = false;
    robot->target_found_once = false;
    robot->current_heading = NORTH;  // Initial heading (will be set based on first move)
    robot->command_string[0] = '\0';
    robot->command_string_length = 0;
}

void robot_get_unexplored_neighbors(Robot* robot, int node, int* unexplored, int* count) {
    *count = 0;
    for (int i = 0; i < robot->graph->adj[node].num_neighbors; i++) {
        int neighbor = robot->graph->adj[node].neighbors[i];
        if (!edge_set_contains(&robot->visited_edges, node, neighbor)) {
            unexplored[(*count)++] = neighbor;
        }
    }
}

void robot_find_all_frontiers(Robot* robot, int* frontiers, int* count) {
    *count = 0;
    int unexplored[MAX_NEIGHBORS];
    int unexplored_count;
    
    for (int i = 0; i < MAX_NODES; i++) {
        if (node_set_contains(&robot->visited_nodes, i)) {
            robot_get_unexplored_neighbors(robot, i, unexplored, &unexplored_count);
            if (unexplored_count > 0) {
                frontiers[(*count)++] = i;
            }
        }
    }
}

// Get direction from point A to point B
Direction get_direction(Point a, Point b) {
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    
    // Determine primary direction (prioritize larger movement)
    if (abs(dy) > abs(dx)) {
        if (dy > 0) return NORTH;
        else return SOUTH;
    } else {
        if (dx > 0) return EAST;
        else return WEST;
    }
}

// Get relative turn needed to face target direction
// Returns positive = left turns, negative = right turns
// Note: When heading SOUTH, right hand is WEST (robot's perspective)
int get_turns_needed(Direction current, Direction target) {
    int diff = (target - current + 4) % 4;
    if (diff == 0) return 0;      // Same direction
    if (diff == 1) return -1;     // Need 1 right turn (e.g., SOUTH->WEST: right turn)
    if (diff == 2) return -2;     // Need 2 right turns (180 degrees)
    return 1;                      // diff == 3, need 1 left turn (e.g., SOUTH->EAST: left turn)
}

// Add command to command string
void add_command(Robot* robot, char cmd) {
    if (robot->command_string_length < MAX_PATH_LENGTH * 3 - 1) {
        robot->command_string[robot->command_string_length++] = cmd;
        robot->command_string[robot->command_string_length] = '\0';
    }
}

void robot_update_position(Robot* robot, int next_node) {
    int u = robot->current_node < next_node ? robot->current_node : next_node;
    int v = robot->current_node < next_node ? next_node : robot->current_node;
    edge_set_add(&robot->visited_edges, u, v);
    node_set_add(&robot->visited_nodes, next_node);
    
    // Calculate movement commands
    Point current_pos = robot->graph->positions[robot->current_node];
    Point next_pos = robot->graph->positions[next_node];
    Direction target_dir = get_direction(current_pos, next_pos);
    
    // Determine if we need to turn or go backward
    if (robot->path_history_length == 1) {
        // First move: set initial heading
        robot->current_heading = target_dir;
        add_command(robot, 'F');
    } else {
        // Check if target is opposite direction (backward)
        Direction opposite = (robot->current_heading + 2) % 4;
        if (target_dir == opposite) {
            // Go backward (robot can move backward without turning)
            add_command(robot, 'B');
            robot->current_heading = opposite;  // Heading flips when going backward
        } else if (target_dir == robot->current_heading) {
            // Same direction, move forward
            add_command(robot, 'F');
        } else {
            // Need to turn
            // Note: L and R commands include moving forward, so no need to add 'F' after
            int turns = get_turns_needed(robot->current_heading, target_dir);
            if (turns > 0) {
                // Turn left (positive = left turns)
                for (int i = 0; i < turns; i++) {
                    add_command(robot, 'L');
                    robot->current_heading = (robot->current_heading + 3) % 4;  // Left turn (-1 mod 4 = 3)
                }
            } else if (turns < 0) {
                // Turn right (negative = right turns)
                int right_turns = -turns;  // Convert to positive number
                for (int i = 0; i < right_turns; i++) {
                    add_command(robot, 'R');
                    robot->current_heading = (robot->current_heading + 1) % 4;  // Right turn
                }
            }
            // L and R already include forward movement, so no 'F' needed here
        }
    }
    
    robot->current_node = next_node;
    robot->path_history_log[robot->path_history_length++] = next_node;
}

void robot_move_to_node(Robot* robot, int target_node) {
    if (robot->current_node == target_node) {
        return;
    }
    
    int path[MAX_NODES];
    int path_length;
    
    if (bfs_shortest_path(robot->graph, robot->current_node, target_node, path, &path_length, 
                         &robot->visited_nodes, &robot->visited_edges)) {
        if (path_length > 1) {
            robot_update_position(robot, path[1]);
        }
    } else {
        DEBUG_PRINT("Error: Cannot find path from %d to %d\n", robot->current_node, target_node);
    }
}

void robot_move(Robot* robot) {
    if (robot->exploration_complete) {
        return;
    }
    
    // Check if we just found the target
    if (robot->current_node == robot->target_node && !robot->target_found_once) {
        DEBUG_PRINT("--- Target Found! Continuing to explore... ---\n");
        robot->target_found_once = true;
    }
    
    // Priority 1: Are there unexplored paths right here?
    int unexplored[MAX_NEIGHBORS];
    int unexplored_count;
    robot_get_unexplored_neighbors(robot, robot->current_node, unexplored, &unexplored_count);
    
    if (unexplored_count > 0) {
        robot_update_position(robot, unexplored[0]);
        return;
    }
    
    // Priority 2: Find nearest frontier
    int frontiers[MAX_NODES];
    int frontier_count;
    robot_find_all_frontiers(robot, frontiers, &frontier_count);
    
    if (frontier_count == 0) {
        // No frontiers left, go home
        if (robot->current_node == robot->start_node) {
            DEBUG_PRINT("Exploration complete! Returned to start.\n");
            robot->exploration_complete = true;
        } else {
            robot_move_to_node(robot, robot->start_node);
        }
        return;
    }
    
    // Find nearest frontier
    int nearest_frontier = -1;
    int min_dist = INT_MAX;
    
    for (int i = 0; i < frontier_count; i++) {
        int path_len = bfs_shortest_path_length(robot->graph, robot->current_node, 
                                                 frontiers[i], 
                                                 &robot->visited_nodes, 
                                                 &robot->visited_edges);
        if (path_len < min_dist) {
            min_dist = path_len;
            nearest_frontier = frontiers[i];
        }
    }
    
    if (nearest_frontier != -1) {
        robot_move_to_node(robot, nearest_frontier);
    } else {
        DEBUG_PRINT("Error: No reachable frontiers.\n");
        robot->exploration_complete = true;
    }
}

// --- 7. Visualization Output (console-based) ---
void print_maze_state(Graph* g, Robot* robot, int* optimal_path, int optimal_path_length) {
    DEBUG_PRINT("\n=== Maze State ===\n");
    DEBUG_PRINT("Current node: %d\n", robot->current_node);
    DEBUG_PRINT("Visited nodes: %d\n", robot->visited_nodes.count);
    DEBUG_PRINT("Visited edges: ");
    
    int edge_count = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = i + 1; j < MAX_NODES; j++) {
            if (edge_set_contains(&robot->visited_edges, i, j)) {
                DEBUG_PRINT("(%d-%d) ", i, j);
                edge_count++;
            }
        }
    }
    DEBUG_PRINT("\nTotal visited edges: %d\n", edge_count);
    
    if (optimal_path != NULL && optimal_path_length > 0) {
        DEBUG_PRINT("Optimal path: ");
        for (int i = 0; i < optimal_path_length; i++) {
            DEBUG_PRINT("%d ", optimal_path[i]);
            if (i < optimal_path_length - 1) DEBUG_PRINT("-> ");
        }
        DEBUG_PRINT("\n");
    }
    DEBUG_PRINT("==================\n");
}

// --- 8. Main Simulation ---
int main() {
    Graph g;
    init_graph(&g);
    
    Robot robot;
    robot_init(&robot, &g, START_NODE, TARGET_NODE);
    
    int* shortest_path_nodes = NULL;
    int shortest_path_length = 0;
    
    int max_steps = 500;
    for (int i = 0; i < max_steps; i++) {
        print_maze_state(&g, &robot, NULL, 0);
        DEBUG_PRINT("Step: %d\n", i);
        
        // Output current movement commands (as robot explores)
        if (robot.command_string_length > 0 && (i % 10 == 0 || robot.exploration_complete)) {
            printf("Exploration commands (so far): %s\n", robot.command_string);
        }
        
        if (robot.exploration_complete) {
            DEBUG_PRINT("Simulation finished in %d steps.\n", i);
            
            DEBUG_PRINT("Calculating shortest path...\n");
            shortest_path_nodes = (int*)malloc(MAX_NODES * sizeof(int));
            // Use BFS for shortest path by NUMBER OF STEPS (not distance)
            if (bfs_shortest_path_full(&g, START_NODE, TARGET_NODE, shortest_path_nodes, 
                                      &shortest_path_length)) {
                // Final optimal path is always printed (not wrapped in DEBUG_PRINT)
                printf("Optimal path: ");
                for (int j = 0; j < shortest_path_length; j++) {
                    printf("%d", shortest_path_nodes[j]);
                    if (j < shortest_path_length - 1) printf(" -> ");
                }
                printf("\n");
                
                // Generate movement commands for optimal path
                char optimal_commands[MAX_PATH_LENGTH * 3];
                int cmd_len = 0;
                Direction opt_heading = NORTH;
                
                for (int j = 0; j < shortest_path_length - 1; j++) {
                    Point current_pos = g.positions[shortest_path_nodes[j]];
                    Point next_pos = g.positions[shortest_path_nodes[j + 1]];
                    Direction target_dir = get_direction(current_pos, next_pos);
                    
                    if (j == 0) {
                        opt_heading = target_dir;
                        optimal_commands[cmd_len++] = 'F';
                    } else {
                        Direction opposite = (opt_heading + 2) % 4;
                        if (target_dir == opposite) {
                            optimal_commands[cmd_len++] = 'B';
                            opt_heading = opposite;
                        } else if (target_dir == opt_heading) {
                            optimal_commands[cmd_len++] = 'F';
                        } else {
                            // Need to turn
                            // Note: L and R commands include moving forward, so no need to add 'F' after
                            int turns = get_turns_needed(opt_heading, target_dir);
                            if (turns > 0) {
                                // Turn left
                                for (int k = 0; k < turns; k++) {
                                    optimal_commands[cmd_len++] = 'L';
                                    opt_heading = (opt_heading + 3) % 4;
                                }
                            } else if (turns < 0) {
                                // Turn right
                                int right_turns = -turns;
                                for (int k = 0; k < right_turns; k++) {
                                    optimal_commands[cmd_len++] = 'R';
                                    opt_heading = (opt_heading + 1) % 4;
                                }
                            }
                            // L and R already include forward movement, so no 'F' needed here
                        }
                    }
                }
                optimal_commands[cmd_len] = '\0';
                printf("Movement commands: %s\n", optimal_commands);
            }
            
            break;
        }
        
        robot_move(&robot);
    }
    
    if (!robot.exploration_complete) {
        DEBUG_PRINT("Simulation stopped after %d steps. Map not fully explored.\n", max_steps);
        // If exploration didn't complete, still try to find and print the path if it exists
        // Use BFS for shortest path by NUMBER OF STEPS (not distance)
        shortest_path_nodes = (int*)malloc(MAX_NODES * sizeof(int));
        if (bfs_shortest_path_full(&g, START_NODE, TARGET_NODE, shortest_path_nodes, 
                                   &shortest_path_length)) {
            printf("Optimal path: ");
            for (int j = 0; j < shortest_path_length; j++) {
                printf("%d", shortest_path_nodes[j]);
                if (j < shortest_path_length - 1) printf(" -> ");
            }
            printf("\n");
            
            // Generate movement commands for optimal path
            char optimal_commands[MAX_PATH_LENGTH * 3];
            int cmd_len = 0;
            Direction opt_heading = NORTH;
            
            for (int j = 0; j < shortest_path_length - 1; j++) {
                Point current_pos = g.positions[shortest_path_nodes[j]];
                Point next_pos = g.positions[shortest_path_nodes[j + 1]];
                Direction target_dir = get_direction(current_pos, next_pos);
                
                if (j == 0) {
                    opt_heading = target_dir;
                    optimal_commands[cmd_len++] = 'F';
                } else {
                    Direction opposite = (opt_heading + 2) % 4;
                    if (target_dir == opposite) {
                        optimal_commands[cmd_len++] = 'B';
                        opt_heading = opposite;
                    } else if (target_dir == opt_heading) {
                        optimal_commands[cmd_len++] = 'F';
                    } else {
                        // Need to turn
                        // Note: L and R commands include moving forward, so no need to add 'F' after
                        int turns = get_turns_needed(opt_heading, target_dir);
                        if (turns > 0) {
                            // Turn left
                            for (int k = 0; k < turns; k++) {
                                optimal_commands[cmd_len++] = 'L';
                                opt_heading = (opt_heading + 3) % 4;
                            }
                        } else if (turns < 0) {
                            // Turn right
                            int right_turns = -turns;
                            for (int k = 0; k < right_turns; k++) {
                                optimal_commands[cmd_len++] = 'R';
                                opt_heading = (opt_heading + 1) % 4;
                            }
                        }
                        // L and R already include forward movement, so no 'F' needed here
                    }
                }
            }
            optimal_commands[cmd_len] = '\0';
            printf("Movement commands: %s\n", optimal_commands);
        }
    }
    
    if (shortest_path_nodes != NULL) {
        free(shortest_path_nodes);
    }
    
    return 0;
}

