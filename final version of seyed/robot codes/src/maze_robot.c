/**
 * @file maze_robot.c
 * @brief Robot state management, command generation, and frontier detection.
 *
 * Implements the heading-to-command translation from ALGORITHMS.md §5,
 * frontier detection for the exploration module, and path following.
 */

#include "maze_robot.h"
#include <string.h>

/* ==========================================================================
 * HEADING HELPERS — unit vectors for N/E/S/W (Y is up)
 * ========================================================================== */

typedef struct { int8_t dx, dy; } HeadingVec;

static const HeadingVec heading_vecs[4] = {
    { 0,  1},   /* MAZE_NORTH = 0  (+Y) */
    {-1,  0},   /* MAZE_WEST  = 1  (-X) */
    { 0, -1},   /* MAZE_SOUTH = 2  (-Y) */
    { 1,  0},   /* MAZE_EAST  = 3  (+X) */
};

/* ==========================================================================
 * ROBOT LIFECYCLE
 * ========================================================================== */

int maze_robot_init(MazeRobot* robot, MazeGraph* graph) {
    if (!robot || !graph) return MAZE_ERR_NULL_PTR;
    memset(robot, 0, sizeof(*robot));

    robot->graph         = graph;
    robot->current_node  = 0;               /* start node is always index 0 */
    robot->current_coord = graph->nodes[0].coord;
    robot->heading       = MAZE_HEADING_NONE;
    robot->phase         = MAZE_PHASE_EXPLORE;
    robot->proof_state   = MAZE_PROOF_SEARCHING;
    robot->target_found  = false;
    robot->use_proof     = true;            /* optimized search ON by default */

    /* Mark start as visited in robot's own tracking */
    robot->visited_nodes[0] = true;

    return MAZE_OK;
}

void maze_robot_reset(MazeRobot* robot) {
    if (!robot) return;
    MazeGraph* g = robot->graph;   /* preserve graph reference */
    memset(robot, 0, sizeof(*robot));
    robot->graph = g;
    robot->visited_nodes[0] = true;
}

/* ==========================================================================
 * HEADING & DIRECTION
 * ========================================================================== */

MazeHeading maze_heading_opposite(MazeHeading h) {
    if (h >= 4) return h;
    return (MazeHeading)(((uint8_t)h + 2) & 0x3);
}

MazeHeading maze_heading_between(MazeCoord a, MazeCoord b) {
    int16_t dx = b.x - a.x;
    int16_t dy = b.y - a.y;
    int16_t adx = (dx < 0) ? -dx : dx;
    int16_t ady = (dy < 0) ? -dy : dy;

    if (ady >= adx)
        return (dy >= 0) ? MAZE_NORTH : MAZE_SOUTH;
    else
        return (dx >= 0) ? MAZE_EAST  : MAZE_WEST;
}

int8_t maze_heading_cross(MazeHeading h1, MazeHeading h2) {
    if (h1 >= 4 || h2 >= 4) return 0;
    const HeadingVec* v1 = &heading_vecs[h1];
    const HeadingVec* v2 = &heading_vecs[h2];
    /* cross = dx1*dy2 - dy1*dx2 */
    return (int8_t)(v1->dx * v2->dy - v1->dy * v2->dx);
}

MazeCommand maze_robot_compute_command(MazeHeading* heading,
                                       MazeCoord from,
                                       MazeCoord to,
                                       bool is_first) {
    if (!heading) return MAZE_CMD_NONE;

    MazeHeading new_h = maze_heading_between(from, to);

    if (is_first || *heading == MAZE_HEADING_NONE) {
        /* First move — just face the direction and go forward */
        *heading = new_h;
        return MAZE_CMD_FORWARD;
    }

    MazeHeading prev = *heading;

    /* Same direction → straight */
    if (new_h == prev) {
        /* heading doesn't change */
        return MAZE_CMD_FORWARD;
    }

    /* Opposite direction → U-turn */
    if (new_h == maze_heading_opposite(prev)) {
        *heading = new_h;
        return MAZE_CMD_BACK;
    }

    /* Cross product determines left vs right */
    int8_t cross = maze_heading_cross(prev, new_h);
    *heading = new_h;

    if (cross > 0) return MAZE_CMD_LEFT;
    else           return MAZE_CMD_RIGHT;
    /* cross == 0 only for same/opposite, already handled */
}

/* ==========================================================================
 * POSITION & MOVEMENT
 * ========================================================================== */

int maze_robot_move_to(MazeRobot* robot, uint16_t next_id, bool is_first) {
    if (!robot || !robot->graph) return MAZE_ERR_NULL_PTR;
    if (next_id >= robot->graph->node_count) return MAZE_ERR_NODE_NOT_FOUND;

    MazeCoord from = robot->current_coord;
    MazeCoord to   = robot->graph->nodes[next_id].coord;

    /* Compute and log the command */
    MazeCommand cmd = maze_robot_compute_command(&robot->heading, from, to, is_first);
    if (cmd != MAZE_CMD_NONE) {
        maze_robot_log_command(robot, cmd);
    }

    /* Mark the edge as explored */
    maze_graph_mark_explored(robot->graph, robot->current_node, next_id);

    /* Update accumulated distance */
    int eidx = maze_graph_find_edge(robot->graph, robot->current_node, next_id);
    if (eidx >= 0) {
        robot->total_distance_cm += robot->graph->edges[eidx].length_cm;
    }

    /* Move to the new node */
    robot->current_node  = next_id;
    robot->current_coord = to;
    robot->visited_nodes[next_id] = true;
    maze_graph_mark_visited(robot->graph, next_id);

    robot->steps_taken++;

    return MAZE_OK;
}

void maze_robot_arrive(MazeRobot* robot, uint16_t node_id) {
    if (!robot) return;
    robot->visited_nodes[node_id] = true;
    maze_graph_mark_visited(robot->graph, node_id);
}

/* ==========================================================================
 * FRONTIER DETECTION
 * ========================================================================== */

uint8_t maze_robot_find_frontiers(const MazeRobot* robot,
                                  uint16_t* frontiers,
                                  uint8_t max_frontiers) {
    uint8_t count = 0;
    if (!robot || !robot->graph || max_frontiers == 0) return 0;

    for (uint16_t i = 0; i < robot->graph->node_count && count < max_frontiers; i++) {
        /* Must be a visited node */
        if (!robot->visited_nodes[i]) continue;

        /* Must have at least one unexplored edge */
        uint16_t unex[4];
        uint8_t n = maze_graph_get_unexplored_neighbors(robot->graph, i, unex, 4);
        if (n > 0) {
            frontiers[count++] = i;
        }
    }
    return count;
}

uint16_t maze_robot_nearest_frontier(const MazeRobot* robot) {
    if (!robot || !robot->graph) return MAZE_INVALID_NODE;

    uint16_t frontiers[MAZE_MAX_FRONTIERS];
    uint8_t fcount = maze_robot_find_frontiers(robot, frontiers, MAZE_MAX_FRONTIERS);
    if (fcount == 0) return MAZE_INVALID_NODE;

    /* Find the one with the shortest Dijkstra distance */
    uint16_t best   = MAZE_INVALID_NODE;
    uint32_t best_d = UINT32_MAX;

    for (uint8_t i = 0; i < fcount; i++) {
        uint32_t d = maze_graph_path_distance(robot->graph,
                                              robot->current_node,
                                              frontiers[i]);
        if (d < best_d) {
            best_d = d;
            best   = frontiers[i];
        }
    }
    return best;
}

bool maze_robot_has_unexplored(const MazeRobot* robot) {
    if (!robot || !robot->graph) return false;
    uint16_t unex[4];
    uint8_t n = maze_graph_get_unexplored_neighbors(robot->graph,
                                                     robot->current_node,
                                                     unex, 4);
    return n > 0;
}

/* ==========================================================================
 * BRANCH PRIORITY — turn-minimising (straight > left > right > back)
 * ========================================================================== */

/**
 * Turn cost: F(straight)=0, L=1, R=2, B(back)=3.
 * Tie-break by compass order: N(0), E(3), S(2), W(1).
 *   Actually:  N=0, E=3, S=2, W=1  gives N < W < S < E.
 * For a deterministic tie-break let's use  N < E < S < W  ordering.
 *   So compass_rank: N=0, E=1, S=2, W=3.
 */
static uint8_t _compass_rank(MazeHeading h) {
    /* remap N(0)→0, W(1)→3, S(2)→2, E(3)→1 */
    static const uint8_t map[4] = {0, 3, 2, 1};
    return (h < 4) ? map[h] : 99;
}

static uint8_t _turn_cost(MazeHeading current, MazeHeading target) {
    if (current == target)                 return 0;   /* F */
    if (target == maze_heading_opposite(current)) return 3;   /* B */

    int8_t cross = maze_heading_cross(current, target);
    if (cross > 0) return 1;   /* L */
    else           return 2;   /* R */
}

void maze_robot_rank_branches(const MazeRobot* robot,
                              const uint16_t* candidates,
                              uint8_t count,
                              uint16_t* ranked) {
    if (!robot || !candidates || !ranked || count == 0) return;

    /* Copy candidates */
    for (uint8_t i = 0; i < count; i++) ranked[i] = candidates[i];

    /* Simple insertion sort by (turn_cost, compass_rank, id) */
    for (uint8_t i = 1; i < count; i++) {
        uint16_t key = ranked[i];
        uint8_t  j   = i;

        MazeHeading h_key = maze_heading_between(robot->current_coord,
                                                  robot->graph->nodes[key].coord);
        uint8_t cost_key = _turn_cost(robot->heading, h_key);
        uint8_t comp_key = _compass_rank(h_key);

        while (j > 0) {
            uint16_t prev = ranked[j - 1];
            MazeHeading h_prev = maze_heading_between(robot->current_coord,
                                                       robot->graph->nodes[prev].coord);
            uint8_t cost_prev = _turn_cost(robot->heading, h_prev);
            uint8_t comp_prev = _compass_rank(h_prev);

            if (cost_key < cost_prev ||
                (cost_key == cost_prev && comp_key < comp_prev) ||
                (cost_key == cost_prev && comp_key == comp_prev && key < prev)) {
                ranked[j] = prev;
                j--;
            } else {
                break;
            }
        }
        ranked[j] = key;
    }
}

/* ==========================================================================
 * PATH FOLLOWING
 * ========================================================================== */

void maze_robot_set_path(MazeRobot* robot,
                         const uint16_t* path,
                         uint16_t length) {
    if (!robot || !path || length == 0) return;
    if (length > MAZE_MAX_PATH_LENGTH) length = MAZE_MAX_PATH_LENGTH;

    /* Copy path, skip the first node (robot is already there) */
    if (length > 1 && path[0] == robot->current_node) {
        memcpy(robot->path_buffer, path + 1, (length - 1) * sizeof(uint16_t));
        robot->path_length = length - 1;
    } else {
        memcpy(robot->path_buffer, path, length * sizeof(uint16_t));
        robot->path_length = length;
    }
    robot->path_index = 0;
}

bool maze_robot_has_path(const MazeRobot* robot) {
    if (!robot) return false;
    return robot->path_index < robot->path_length;
}

MazeCommand maze_robot_step_path(MazeRobot* robot) {
    if (!robot || !maze_robot_has_path(robot)) return MAZE_CMD_NONE;

    uint16_t next_id = robot->path_buffer[robot->path_index];
    robot->path_index++;

    bool is_first = (robot->heading == MAZE_HEADING_NONE);
    int rc = maze_robot_move_to(robot, next_id, is_first);

    if (rc != MAZE_OK) return MAZE_CMD_NONE;

    /* Return the last command logged */
    if (robot->command_count > 0)
        return (MazeCommand)robot->command_log[robot->command_count - 1];
    return MAZE_CMD_NONE;
}

/* ==========================================================================
 * COMMAND LOG
 * ========================================================================== */

void maze_robot_log_command(MazeRobot* robot, MazeCommand cmd) {
    if (!robot || cmd == MAZE_CMD_NONE) return;
    if (robot->command_count >= MAZE_COMMAND_LOG_SIZE - 1) return;
    robot->command_log[robot->command_count++] = (char)cmd;
    robot->command_log[robot->command_count]   = '\0';
}
