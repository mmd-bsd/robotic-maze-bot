/**
 * @file maze_robot.h
 * @brief Robot state management, command generation, and frontier detection.
 *
 * Tracks the robot's heading, position, and command stream.  Generates F/L/R/B
 * movement commands from heading deltas using the cross-product method from
 * ALGORITHMS.md §5.  Detects frontiers (visited nodes with unexplored edges)
 * for the exploration module.
 *
 * Command generation (ALGORITHMS.md §5):
 *   prev_heading == new_heading     → F (straight)
 *   new_heading == opposite(prev)   → B (U-turn)
 *   cross(prev, new) > 0            → L (left)
 *   cross(prev, new) < 0            → R (right)
 *
 *   cross((dx1,dy1), (dx2,dy2)) = dx1*dy2 - dy1*dx2
 *
 * With Y up: NORTH=(0,+1), WEST=(-1,0), SOUTH=(0,-1), EAST=(+1,0)
 *   NORTH→WEST:  cross=0*0-1*(-1)=+1 → L ✓
 *   NORTH→EAST:  cross=0*0-1*1=-1    → R ✓
 */

#ifndef MAZE_ROBOT_H
#define MAZE_ROBOT_H

#include "maze_types.h"
#include "maze_config.h"
#include "maze_graph.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * ROBOT LIFECYCLE
 *============================================================================*/

/**
 * @brief  Initialise the robot at the start node.
 * @param  robot Pointer to uninitialised MazeRobot.
 * @param  graph Pointer to the robot's graph (must already be init'd).
 * @return MAZE_OK.
 */
int maze_robot_init(MazeRobot* robot, MazeGraph* graph);

/**
 * @brief  Reset all robot state (keeps the graph reference).
 */
void maze_robot_reset(MazeRobot* robot);

/*============================================================================
 * HEADING & DIRECTION
 *============================================================================*/

/** Opposite heading (180° turn). */
MazeHeading maze_heading_opposite(MazeHeading h);

/**
 * @brief  Compute heading from `a` to `b` (primary axis, Y is up).
 */
MazeHeading maze_heading_between(MazeCoord a, MazeCoord b);

/**
 * @brief  Cross product of two heading vectors: cross(h1, h2).
 * @return > 0 = left, < 0 = right, 0 = same or opposite.
 */
int8_t maze_heading_cross(MazeHeading h1, MazeHeading h2);

/**
 * @brief  Compute the F/L/R/B command when moving from `from` to `to`
 *         given the robot's current heading.  Updates `*heading` in place.
 * @param  heading  Current heading (updated to the new heading after the move).
 * @param  from     Departure node coordinate.
 * @param  to       Arrival node coordinate.
 * @param  is_first If true, this is the first move — emits F and sets heading.
 * @return MazeCommand (F/L/R/B), or MAZE_CMD_NONE on error.
 */
MazeCommand maze_robot_compute_command(MazeHeading* heading,
                                       MazeCoord from,
                                       MazeCoord to,
                                       bool is_first);

/*============================================================================
 * POSITION & MOVEMENT
 *============================================================================*/

/**
 * @brief  Move the robot to a new node.  Updates position, heading, visited
 *         tracking, and appends the F/L/R/B command to the log.
 *
 * @param  robot    Robot state.
 * @param  next_id  Node index to move to.
 * @param  is_first True if this is the very first move of the mission.
 * @return MAZE_OK on success.
 */
int maze_robot_move_to(MazeRobot* robot, uint16_t next_id, bool is_first);

/**
 * @brief  Record that the robot has visually arrived at a node (for stub
 *         display / frontier appearance).  Marks the node visited in graph.
 */
void maze_robot_arrive(MazeRobot* robot, uint16_t node_id);

/*============================================================================
 * FRONTIER DETECTION
 *============================================================================*/

/**
 * @brief  Find all frontier nodes on the known map.
 *
 * A frontier is a VISITED node that still has at least one incident edge
 * that is NOT yet explored (i.e., an unexplored branch).
 *
 * @param  robot          Robot state.
 * @param  frontiers      Output array of frontier node indices.
 * @param  max_frontiers  Capacity of `frontiers`.
 * @return Number of frontiers written.
 */
uint8_t maze_robot_find_frontiers(const MazeRobot* robot,
                                  uint16_t* frontiers,
                                  uint8_t max_frontiers);

/**
 * @brief  Find the nearest frontier by real travel distance (Dijkstra on
 *         the known map).  Returns MAZE_INVALID_NODE if none remain.
 */
uint16_t maze_robot_nearest_frontier(const MazeRobot* robot);

/**
 * @brief  Check whether the current node has any unexplored edges.
 */
bool maze_robot_has_unexplored(const MazeRobot* robot);

/*============================================================================
 * BRANCH PRIORITY (turn-minimising)
 *============================================================================*/

/**
 * @brief  Rank unexplored neighbors by the turn cost from the current heading.
 *         Lower rank = fewer turns.  Tie-break: N, E, S, W compass order.
 *
 *         Priority: straight (F) < left (L) < right (R) < reverse (B)
 *
 * @param  robot       Robot state (current node + heading).
 * @param  candidates  Array of unexplored neighbor node indices.
 * @param  count       Number of candidates.
 * @param  ranked      Output: candidates sorted by priority (best first).
 *                      Must be at least `count` elements.
 */
void maze_robot_rank_branches(const MazeRobot* robot,
                              const uint16_t* candidates,
                              uint8_t count,
                              uint16_t* ranked);

/*============================================================================
 * PATH FOLLOWING
 *============================================================================*/

/** Store a multi-step path for the robot to follow node-by-node. */
void maze_robot_set_path(MazeRobot* robot,
                         const uint16_t* path,
                         uint16_t length);

/** True if the robot is currently following a stored path. */
bool maze_robot_has_path(const MazeRobot* robot);

/**
 * @brief  Execute ONE step along the stored path.
 *         Advances path_index, emits the command, updates heading/position.
 * @return The command executed, or MAZE_CMD_NONE if the path is exhausted.
 */
MazeCommand maze_robot_step_path(MazeRobot* robot);

/*============================================================================
 * COMMAND LOG
 *============================================================================*/

/** Append a command to the log. */
void maze_robot_log_command(MazeRobot* robot, MazeCommand cmd);

/*============================================================================
 * UTILITY
 *============================================================================*/

/** Get the robot's current node index. */
static inline uint16_t maze_robot_current_node(const MazeRobot* r) {
    return r->current_node;
}

/** Check if target has been found. */
static inline bool maze_robot_target_found(const MazeRobot* r) {
    return r->target_found;
}

/** Mark the target as found at a specific node. */
static inline void maze_robot_set_target(MazeRobot* r, uint16_t node_id) {
    r->target_found = true;
    r->graph->target_node = node_id;
}

#ifdef __cplusplus
}
#endif

#endif /* MAZE_ROBOT_H */
